"""VD3 actor contract and explicit, latched hardware preparation/stop states."""
import hashlib
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import torch
from vd1_control import build_actor_observation, _tool_quaternion
from dual_camera_perception import quaternion_wxyz_to_matrix

ROOT=Path(__file__).resolve().parents[1]
JOINTS=('FL_hip_joint','FR_hip_joint','RL_hip_joint','RR_hip_joint',
        'FL_thigh_joint','FR_thigh_joint','RL_thigh_joint','RR_thigh_joint','joint1',
        'FL_calf_joint','FR_calf_joint','RL_calf_joint','RR_calf_joint',
        'joint2','joint3','joint4','joint5','joint6')
ARM=np.array([8,13,14,15,16,17])
LEGS=np.array([i for i in range(18) if i not in ARM])
DEFAULT=np.array([.1,-.1,.1,-.1,.8,.8,1,1,0,-1.5,-1.5,-1.5,-1.5,2.55,-1.6,-1.4,0,0])
# Physical stance from the archived real deployment, not the training default.
STAND=DEFAULT.copy();STAND[:4]=0.;STAND[4:8]=.8
PRE_STAND=STAND.copy();PRE_STAND[:4]=[.1,-.1,.1,-.1]
PRE_STAND[4:8]=1.36;PRE_STAND[9:13]=-2.65
FIXED_KP=np.full(18,200.);FIXED_KP[9:13]=300.;FIXED_KP[ARM]=0.
FIXED_KD=np.full(18,3.);FIXED_KD[ARM]=0.
# Source: outputs/diagnosis_v256_20261029_100719/board_config.yaml.
STAND_KP=FIXED_KP*2.25
STAND_KD=np.full(18,15.);STAND_KD[ARM]=0.
POLICY_KP=np.full(18,160.);POLICY_KP[ARM]=0.
POLICY_KD=np.full(18,5.);POLICY_KD[ARM]=0.
ACTOR_HASH='51eda90b2b77204ef66d949523486250e88c7062aeb1e86f0dc5478924291c78'
R1,L1,A,B,X,Y=1,2,256,512,1024,2048
MOVE_SECONDS={'pre_stand':1.,'standing_up':2.,'arm_prepare':6.,'return_stand':3.,'arm_lower':6.,'lowering':5.}
MOVE_JOINTS={mode:(ARM if mode in ('arm_prepare','arm_lower') else LEGS) for mode in MOVE_SECONDS}
# Ground contact can leave the ab/adduction hips slightly wider than their
# original unloaded angle. Prone completion requires actual thigh/calf folding
# and settling; the hip reference continues to be held and its error logged.
FOLD=np.array([4,5,6,7,9,10,11,12])
MOVE_JOINTS['lowering']=FOLD

def limits():
    elements={j.get('name'):j for j in ET.parse(ROOT/'assets/robot.urdf').getroot().findall('joint')}
    bounds=[elements[name].find('limit') for name in JOINTS]
    return np.array([[float(e.get('lower')),float(e.get('upper'))] for e in bounds])

def position_target(target,bounds):
    """Keep measured encoders unchanged; bound only a position hold command.

    Real passive Z1 zero readings can cross a nominal URDF boundary by a few
    milliradians. Large violations still fail instead of silently moving a joint
    from an unknown position. Actor outputs already obey the same bounds.
    """
    value=np.asarray(target,dtype=float)
    if value.shape!=(18,) or not np.isfinite(value).all():
        raise ValueError('nonfinite or malformed position target')
    bounded=np.clip(value,bounds[:,0],bounds[:,1])
    if np.max(np.abs(value-bounded))>.01:
        raise ValueError('position target exceeds physical limits by more than 0.01 rad')
    return bounded,bool(np.any(value!=bounded))

def measured(state):
    q,qd=np.array(state.q),np.array(state.dq)
    quaternion=np.array(state.quaternion)
    if (not np.isfinite(np.r_[q,qd,quaternion,state.omega]).all()
            or not .95<np.linalg.norm(quaternion)<1.05):
        raise ValueError('invalid encoder/IMU data')
    # IMU orientation is used only for projected gravity, never heading/world position.
    gravity=quaternion_wxyz_to_matrix(quaternion).T@np.array([0.,0.,-1.])
    return q,qd,np.array(state.omega),gravity

class Actor:
    def __init__(self):
        path=ROOT/'assets/policy.pt'
        if hashlib.sha256(path.read_bytes()).hexdigest()!=ACTOR_HASH:
            raise ValueError('actor hash differs from the selected mirror model_2000')
        self.model=torch.jit.load(str(path),map_location='cpu').eval()
        self.last=np.zeros(18)
        self.bounds=limits()
        with torch.inference_mode():
            # Force lazy TorchScript compilation before a controller can acquire B2.
            for _ in range(5):self.model(torch.zeros(1,70))

    def reset(self):self.last[:]=0

    def step(self, state, base, tool):
        q,qd,omega,gravity=measured(state)
        observation=build_actor_observation(base,tool,omega,gravity,q,qd,DEFAULT,self.last)
        with torch.inference_mode():
            action=self.model(torch.from_numpy(observation).unsqueeze(0)).numpy()[0]
        if action.shape!=(18,) or not np.isfinite(action).all():raise ValueError('invalid actor output')
        self.last=action.copy() # raw action, no VA15 EMA/clamp/history
        target=DEFAULT+.25*action
        # Match physical joint range enforcement; do not re-label this as actor clipping.
        clipped=np.clip(target,self.bounds[:,0],self.bounds[:,1])
        return clipped,dict(observation=observation.tolist(),raw_action=action.tolist(),
                           target_limit_applied=bool(np.any(target!=clipped)))

class Remote:
    def __init__(self):self.previous=None
    def event(self,keys):
        previous=self.previous;self.previous=keys
        if keys&(L1|X)==L1|X:return 'emergency'
        if previous is None:return None
        edge=keys&~previous
        if edge&B:return 'down'
        if edge&X:return 'stop'
        if keys&(R1|Y)==R1|Y and previous&(R1|Y)!=R1|Y:return 'track'
        if edge&Y and not keys&R1:return 'prepare'
        if edge&A:return 'stand'
        return None

def blend(start,end,fraction):
    t=float(np.clip(fraction,0,1));t=t*t*(3-2*t)
    return start+(end-start)*t


class Session:
    """Stopping runs the actor at zero command, then returns to nominal stance.

    No support-contact bits are synthesized. The position return waits on actual
    joint speeds and IMU; it does not freeze a mid-swing half-crouched leg pose.
    """
    def __init__(self):
        self.mode='idle';self.entered=0.;self.target=DEFAULT.copy()
        self.prone=None;self.start=None;self.end=None;self.pending_down=False
        self.quiet_since=None;self.message='waiting for A';self.ready=False
        self.entry_target=DEFAULT.copy();self.policy_started=0.
        self.brake_arm_target=None
        self.arm_reference_time=None
        self.arm_reference_rate_rad_s=1.0
        self.arm_reference_limited=False
        self.motor_feedback_blocked=False
        self.brake_velocity=np.zeros(3);self.tool=np.r_[[-1.02,0,-.30],_tool_quaternion()]
        self.kp=FIXED_KP.copy();self.kd=FIXED_KD.copy()
        self.start_kp=self.kp.copy();self.start_kd=self.kd.copy()
        self.end_kp=self.kp.copy();self.end_kd=self.kd.copy()
        self.settle_q=None;self.policy_entry_kp=None;self.policy_entry_kd=None

    def transition(self, mode, now, message):
        self.mode=mode;self.entered=now;self.message=message;self.quiet_since=None

    def move(self,mode,now,end,message):
        self.start=self.target.copy();self.end=end.copy()
        self.start_kp=self.kp.copy();self.start_kd=self.kd.copy()
        if mode in ('pre_stand','standing_up','lowering'):
            self.end_kp=FIXED_KP.copy();self.end_kd=FIXED_KD.copy()
        elif mode in ('arm_prepare','return_stand'):
            self.end_kp=STAND_KP.copy();self.end_kd=STAND_KD.copy()
        else:self.end_kp=self.kp.copy();self.end_kd=self.kd.copy()
        self.transition(mode,now,message)

    def policy_gains(self,now):
        if self.policy_entry_kp is None:
            self.policy_entry_kp=self.kp.copy();self.policy_entry_kd=self.kd.copy()
        self.kp=blend(self.policy_entry_kp,POLICY_KP,now-self.policy_started)
        self.kd=blend(self.policy_entry_kd,POLICY_KD,now-self.policy_started)
        return self.kp,self.kd

    def actor_target(self,target,now):
        """Apply entry blending and the stop reference outside raw actor history.

        Legs keep the zero-command settling policy. Once stop is latched, its
        arm output must no longer inject new references during that settling
        interval. Preserve the last commanded reference and its load-bearing
        error, rather than replacing it with measured encoder positions.
        """
        target=np.asarray(target,dtype=float).copy()
        if now-self.policy_started<1:
            target=blend(self.entry_target,target,now-self.policy_started)
        self.arm_reference_limited=False
        if self.mode=='braking' and self.brake_arm_target is not None:
            target[ARM]=self.brake_arm_target
        else:
            # Bound only the submitted arm position reference. Raw actions and
            # measured encoder velocities remain untouched. A delayed Python
            # tick must not accumulate a large catch-up command.
            previous_time=self.policy_started if self.arm_reference_time is None else self.arm_reference_time
            dt=float(np.clip(now-previous_time,0.,.02))
            change=target[ARM]-self.target[ARM]
            bounded=np.clip(change,-self.arm_reference_rate_rad_s*dt,self.arm_reference_rate_rad_s*dt)
            self.arm_reference_limited=bool(np.any(change!=bounded))
            target[ARM]=self.target[ARM]+bounded
        self.arm_reference_time=now
        self.target=target
        return target

    def block_motor_feedback(self,now):
        """Latch out policy use of a frozen moving motor vector.

        Retain the last submitted support reference/gains. Do not run the
        braking actor with cached arm velocities or silently resume on one new
        reply. Native fault handling and the emergency button still apply.
        """
        if self.mode in ('idle','fault') or self.motor_feedback_blocked:return
        self.motor_feedback_blocked=True;self.ready=False
        self.transition('held',now,'Z1 moving motor feedback frozen: policy inhibited; last references held; no automatic restart')

    def event(self,event,now,q,visual_valid=False):
        if event=='emergency':self.transition('fault',now,'L1+X: damping/passive');return
        if self.mode=='fault':return
        if self.motor_feedback_blocked:return
        if event=='stand' and self.mode=='idle':
            self.prone=q.copy();self.target=q.copy();end=q.copy();end[LEGS]=PRE_STAND[LEGS]
            self.kp=FIXED_KP.copy();self.kd=FIXED_KD.copy()
            self.move('pre_stand',now,end,'A: legacy 1 s folded-leg preparation; arm stays stowed')
        elif event=='prepare' and self.mode=='standing':
            end=self.target.copy();end[ARM]=DEFAULT[ARM]
            self.ready=False;self.move('arm_prepare',now,end,'Y: prepare arm')
        elif event=='track' and self.mode=='ready' and visual_valid:
            self.entry_target=self.target.copy();self.policy_started=now
            self.brake_arm_target=None
            self.arm_reference_time=now
            self.policy_entry_kp=self.kp.copy();self.policy_entry_kd=self.kd.copy()
            self.transition('tracking',now,'R1+Y: track observed seam')
        elif event in ('stop','down','track'):
            if event=='down' and self.mode=='return_stand':self.pending_down=True
            if self.mode in ('tracking','braking'):
                self.pending_down|=event=='down'
                if self.mode!='braking':
                    self.brake_arm_target=self.target[ARM].copy()
                    self.transition('braking',now,'zero body command; hold last arm reference')
            elif event=='down' and self.mode in ('standing','ready'):
                self.pending_down=True;self.begin_down(now,q)
            elif event=='stop' and self.mode in ('pre_stand','standing_up','stand_settle','arm_prepare','arm_lower','lowering'):
                # Interruption is held explicitly; no queued move resumes itself.
                self.ready=False;self.transition('held',now,'interrupted preparation: hold last commanded reference')
            elif event=='down' and self.mode=='held':
                self.pending_down=True;self.begin_down(now,q)
            elif event=='down' and self.mode in ('pre_stand','standing_up','stand_settle','arm_prepare'):
                # Preserve the current motor reference and its load-bearing PD
                # error; rebasing it to measured q would abruptly remove torque.
                self.pending_down=True;self.begin_down(now,q)

    def begin_down(self,now,q):
        end=self.target.copy();end[ARM]=self.prone[ARM]
        if np.max(np.abs(q[ARM]-self.prone[ARM]))<=.15:
            # A-only stand failures already have the arm home. No six-second
            # arm move or unrelated stand-arrival gate should block lowering.
            self.ready=False;self.move('lowering',now,self.prone,'B: arm already home; lower legs')
        else:self.move('arm_lower',now,end,'B: arm returns before legs lower')

    def preparation_status(self,now,q):
        if self.mode not in MOVE_SECONDS and self.mode!='stand_settle':return {}
        indices=LEGS if self.mode=='stand_settle' else MOVE_JOINTS[self.mode]
        error=np.abs(q[indices]-self.end[indices])
        worst=int(indices[int(np.argmax(error))])
        return dict(stage=self.mode,elapsed_s=now-self.entered,duration_s=MOVE_SECONDS.get(self.mode,2.),
                    moving_group='arm' if self.mode in ('arm_prepare','arm_lower') else
                                 'thighs_and_calves' if self.mode=='lowering' else 'legs',
                    max_error_rad=float(error.max()),worst_joint=JOINTS[worst],
                    actual_rad=float(q[worst]),goal_rad=float(self.end[worst]))

    def advance(self,now,q,qd,omega,gravity,dt=.02):
        if self.motor_feedback_blocked:return self.target
        if self.mode in MOVE_SECONDS:
            duration=MOVE_SECONDS[self.mode];fraction=(now-self.entered)/duration
            # Legacy Interpolate is linear. Retain smooth transitions for arm,
            # policy stop and lowering; only the two get-up segments are linear.
            self.target=(self.start+(self.end-self.start)*np.clip(fraction,0,1)
                         if self.mode in ('pre_stand','standing_up') else blend(self.start,self.end,fraction))
            self.kp=blend(self.start_kp,self.end_kp,fraction)
            self.kd=blend(self.start_kd,self.end_kd,fraction)
            if now-self.entered>=duration:
                if self.mode=='pre_stand':
                    end=self.target.copy();end[LEGS]=STAND[LEGS]
                    self.move('standing_up',now,end,'legacy 2 s get-up; arm stays stowed')
                    return self.target
                if self.mode=='standing_up':
                    self.transition('stand_settle',now,'hold stance; ramp original real stand gains over 2 s')
                    self.kd=FIXED_KD.copy();self.kd[LEGS]=8.
                    self.start_kp=self.kp.copy();self.start_kd=self.kd.copy()
                    self.settle_q=None
                    return self.target
                # Check the joints moving in this stage. In particular, failed
                # stand legs must not prevent an already-arrived arm from
                # completing B and proceeding to lower the legs.
                status=self.preparation_status(now,q)
                folded_quiet=(self.mode!='lowering' or
                    np.max(np.abs(qd[FOLD]))<.2 and np.linalg.norm(omega)<.3 and gravity[2]<-.92)
                if status['max_error_rad']>.15 or not folded_quiet:
                    if now-self.entered>duration+4:
                        message=('%s did not arrive: %s error=%.3f rad (actual=%.3f goal=%.3f)' % (
                            self.mode,status['worst_joint'],status['max_error_rad'],status['actual_rad'],status['goal_rad']))
                        if not folded_quiet:message+='; folding/IMU not settled'
                        self.transition('held',now,message)
                    return self.target
                if self.mode=='arm_prepare':self.ready=True;self.transition('ready',now,'ready; R1+Y starts vision')
                elif self.mode=='return_stand':
                    self.ready=True
                    if self.pending_down:self.begin_down(now,q)
                    else:self.transition('ready',now,'stopped in nominal stand; explicit R1+Y to restart')
                elif self.mode=='arm_lower':
                    self.ready=False;self.move('lowering',now,self.prone,'B: lower legs to initial prone position')
                elif self.mode=='lowering':
                    self.pending_down=False;self.transition('idle',now,'prone; A to stand again')
        if self.mode=='stand_settle':
            elapsed=now-self.entered
            self.kp=blend(self.start_kp,STAND_KP,elapsed/2.)
            self.kd=blend(self.start_kd,STAND_KD,elapsed/2.)
            status=self.preparation_status(now,q)
            quiet=(status['max_error_rad']<=.15 and np.max(np.abs(qd[LEGS]))<.2
                   and np.linalg.norm(omega)<.3 and gravity[2]<-.92)
            if elapsed<1. or not quiet:
                self.quiet_since=None;self.settle_q=None
            elif self.settle_q is None or np.max(np.abs(q[LEGS]-self.settle_q))>.03:
                self.quiet_since=now;self.settle_q=q[LEGS].copy()
            if elapsed>=2. and self.quiet_since is not None and now-self.quiet_since>=.5:
                self.transition('standing',now,'original real stand gains active; Y prepares arm')
            elif elapsed>6.:
                self.transition('held',now,'stand_settle did not arrive: %s error=%.3f rad; no automatic retry' % (
                    status['worst_joint'],status['max_error_rad']))
        if self.mode=='braking':
            self.brake_velocity*=np.exp(-dt/.20)
            quiet=(np.max(np.abs(qd[LEGS]))<.5 and np.linalg.norm(omega)<.3
                   and gravity[2]<-.92 and np.max(np.abs(self.brake_velocity))<.005)
            self.quiet_since=now if quiet and self.quiet_since is None else self.quiet_since
            if not quiet:self.quiet_since=None
            if now-self.entered>1 and self.quiet_since is not None and now-self.quiet_since>.4:
                # Begin at the last commanded target, not a fabricated foot support.
                end=self.target.copy();end[LEGS]=STAND[LEGS]
                self.move('return_stand',now,end,'stop: blend to complete symmetric hardware stance')
            elif now-self.entered>5:
                self.transition('fault',now,'zero-command actor did not settle in 5 s')
        return self.target
