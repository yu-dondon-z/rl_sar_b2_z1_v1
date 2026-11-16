"""Local sensor odometry and capture-to-control transport; no simulator state.

Inputs: encoder q/qd, IMU gyro/gravity, actual foot load sensors, camera clocks.
The local reference starts at identity, not a simulator/world base pose. Loaded
feet are approximately stationary (with spherical-foot rolling compensation).
This estimate can drift with slip, calibration and attitude error. It is NOT
ground-truth odometry, a contact estimator, or new actor observations.
"""
from collections import deque
from copy import deepcopy
import math
import xml.etree.ElementTree as ET

import numpy as np

from vd12_visual_control import VD12VisualController
from vd5_visual_control import _plane
from vd1_control import _check_payload

FEET = ('FL', 'FR', 'RL', 'RR')


def finite(value, shape):
    a = np.asarray(value, dtype=float)
    if a.shape != shape or not np.isfinite(a).all():
        raise ValueError('Nonfinite or incorrectly shaped motion sensor input')
    return a


def skew(v):
    x, y, z = v
    return np.array(((0., -z, y), (z, 0., -x), (-y, x, 0.)))


def exp_rotation(vector):
    angle = float(np.linalg.norm(vector))
    if angle < 1.e-12:
        return np.eye(3)
    k = skew(np.asarray(vector) / angle)
    return np.eye(3) + math.sin(angle)*k + (1.-math.cos(angle))*(k @ k)


def align_vectors(first, second):
    cross = np.cross(first, second)
    cosine = float(np.clip(first @ second, -1., 1.))
    if cosine < -.99:
        raise ValueError('Inconsistent IMU gravity/gyro orientation')
    k = skew(cross)
    return np.eye(3) + k + k @ k / (1.+cosine)


def matrix_quaternion(matrix):
    # Davenport's symmetric eigenproblem, returned as normalized wxyz.
    r = finite(matrix, (3, 3))
    k = np.array(((r[0,0]-r[1,1]-r[2,2], r[0,1]+r[1,0], r[0,2]+r[2,0], r[2,1]-r[1,2]),
                  (r[0,1]+r[1,0], r[1,1]-r[0,0]-r[2,2], r[1,2]+r[2,1], r[0,2]-r[2,0]),
                  (r[0,2]+r[2,0], r[1,2]+r[2,1], r[2,2]-r[0,0]-r[1,1], r[1,0]-r[0,1]),
                  (r[2,1]-r[1,2], r[0,2]-r[2,0], r[1,0]-r[0,1], np.trace(r)))) / 3.
    _, vectors = np.linalg.eigh(k)
    q = vectors[:, -1][[3, 0, 1, 2]]
    return q if q[0] >= 0. else -q


class LegKinematics:
    """URDF leg-only analytic FK/Jacobian from measured encoders."""
    def __init__(self, urdf, joint_names):
        self.names = tuple(joint_names)
        tree = ET.parse(urdf).getroot()
        by_child = {j.find('child').get('link'): j for j in tree.findall('joint')}
        self.chains = []
        for foot in FEET:
            chain, link = [], foot+'_foot'
            while link != 'base_link':
                j = by_child[link]
                kind, name = j.get('type'), j.get('name')
                o = j.find('origin')
                p = np.fromstring(o.get('xyz', '0 0 0'), sep=' ') if o is not None else np.zeros(3)
                rpy = np.fromstring(o.get('rpy', '0 0 0'), sep=' ') if o is not None else np.zeros(3)
                from vd5_mujoco_control import quaternion_from_euler_xyz, quaternion_matrix
                rotation = quaternion_matrix(quaternion_from_euler_xyz(*rpy))
                if kind == 'fixed':
                    index, axis = None, np.zeros(3)
                elif kind == 'revolute' and name in self.names:
                    index = self.names.index(name)
                    axis = np.fromstring(j.find('axis').get('xyz'), sep=' ')
                    axis /= np.linalg.norm(axis)
                else:
                    raise ValueError('Unsupported leg encoder chain: '+name)
                chain.append((p, rotation, index, axis))
                link = j.find('parent').get('link')
            self.chains.append(chain[::-1])

    def evaluate(self, q, qd):
        q, qd = finite(q, (len(self.names),)), finite(qd, (len(self.names),))
        feet, velocities, omegas = [], [], []
        for chain in self.chains:
            r, p, joints = np.eye(3), np.zeros(3), []
            for offset, rotation, index, axis in chain:
                p = p + r @ offset
                r = r @ rotation
                if index is not None:
                    joints.append((index, p.copy(), r @ axis))
                    r = r @ exp_rotation(axis*q[index])
            velocity, omega = np.zeros(3), np.zeros(3)
            for index, origin, axis in joints:
                velocity += np.cross(axis, p-origin)*qd[index]
                omega += axis*qd[index]
            feet.append(p)
            velocities.append(velocity)
            omegas.append(omega)
        return np.array(feet), np.array(velocities), np.array(omegas)


class SensorMotionHistory:
    """200Hz relative motion from actual sensors, explicitly bounded prediction."""
    def __init__(self, kinematics, *, max_prediction_s=.05):
        self.kinematics = kinematics
        self.max_prediction_s = float(max_prediction_s)
        if not math.isfinite(self.max_prediction_s) or not 0. < self.max_prediction_s <= .05:
            raise ValueError('Inertial-only prediction must be bounded by 50ms')
        self.samples = deque()
        self.last_loaded_time = None
        self.diagnostic = {}

    def sample(self, clock_s, sensor, loads_n):
        clock_s = float(clock_s)
        if not math.isfinite(clock_s) or clock_s < 0:
            raise ValueError('Invalid motion sample clock')
        gyro, gravity = finite(sensor['gyro_b'], (3,)), finite(sensor['gravity_b'], (3,))
        loads = finite(loads_n, (4,))
        if abs(np.linalg.norm(gravity)-1.) > .001 or np.any(loads < 0.):
            raise ValueError('Invalid gravity or foot pressure sensor')
        feet, vf, wf = self.kinematics.evaluate(sensor['q'], sensor['qd'])
        old = self.samples[-1] if self.samples else None
        dt = clock_s-old['time_s'] if old else 0.
        if old and not 0. < dt <= .010000001:
            raise ValueError('Motion history has a duplicate, reversed or missing sensor sample')
        rotation = np.eye(3) if old is None else old['rotation'] @ exp_rotation(.5*(old['gyro']+gyro)*dt)
        if old is None:
            self.initial_gravity = gravity.copy()
            self.initial_time = clock_s
            # Explicit stationary-reset velocity PRIOR, not a measurement.
            # Nominal simulation resets at rest; hardware needs its own valid
            # stationary initialization. No future sensor timestamp is made up.
        rotation = align_vectors(rotation @ gravity, self.initial_gravity) @ rotation
        # Use the actual pressure only to select support. The no-slip velocity
        # model is an assumption, never relabelled as measured base velocity.
        up = -gravity
        candidates = -vf - np.cross(gyro, feet) + .032*np.cross(wf+gyro, up)
        acceleration = finite(sensor['accelerometer_b_m_s2'], (3,))
        imu_offset = np.array([0., -.02341, .04927])
        alpha = (gyro-old['gyro'])/dt if old else np.zeros(3)
        accel_b = acceleration + 9.81*gravity - np.cross(alpha, imu_offset) - np.cross(gyro,np.cross(gyro,imu_offset))
        accel_local = rotation @ accel_b
        predicted_local = (old['velocity_local']+.5*dt*(old['accel_local']+accel_local)
                           if old else np.zeros(3))
        predicted_b = rotation.T @ predicted_local
        loaded = loads > 20.
        # Pressure during a landing does NOT imply a stationary foot. Reject
        # candidates inconsistent with short-term measured inertial motion,
        # rather than treating initial contact/compression as zero velocity.
        selected = loaded & (np.linalg.norm(candidates-predicted_b, axis=1) <= .20)
        prediction = np.count_nonzero(selected) < 2
        if not prediction:
            velocity_b = np.median(candidates[selected], axis=0)
            residual = float(np.max(np.linalg.norm(candidates[selected]-velocity_b, axis=1)))
            self.last_loaded_time = clock_s
        else:
            residual = None
            anchor = self.initial_time if self.last_loaded_time is None else self.last_loaded_time
            if clock_s-anchor > self.max_prediction_s + 1.e-9:
                raise ValueError('Insufficient loaded feet for bounded local odometry')
            velocity_b = predicted_b
        velocity_local = rotation @ velocity_b
        if np.linalg.norm(velocity_local) > 2.:
            raise ValueError('Implausible encoder-derived local velocity')
        position = np.zeros(3) if old is None else old['position'] + .5*dt*(old['velocity_local']+velocity_local)
        row = dict(time_s=clock_s, rotation=rotation, position=position, gyro=gyro.copy(),
                   velocity_local=velocity_local, accel_local=accel_local)
        self.samples.append(row)
        while len(self.samples)>2 and self.samples[1]['time_s'] < clock_s-2.:
            self.samples.popleft()
        self.diagnostic = dict(sample_time_s=clock_s, support_sensor_count=int(np.count_nonzero(loaded)),
            consistent_support_count=int(np.count_nonzero(selected)),
            loaded_foot_consistency_limit_m_s=.20,
            support_sensor_loads_n=loads.tolist(), loaded_foot_velocity_residual_m_s=residual,
            short_velocity_prediction=bool(prediction), estimated_velocity_b_m_s=velocity_b.tolist(),
            source='IMU_gyro_gravity_accelerometer_plus_encoders_and_actual_foot_load_sensors',
            assumption='loaded_foot_no_slip_spherical_rolling_on_locally_level_ground',
            initialization='zero_velocity_stationary_nominal_reset_prior_NOT_measured',
            simulator_pose_input=False, sent_command_input=False, prediction_bound_s=self.max_prediction_s)
        return dict(self.diagnostic)

    def pose_at(self, clock_s):
        clock_s = float(clock_s)
        for row in reversed(self.samples):
            if abs(row['time_s']-clock_s) <= 1.e-8:
                return row['rotation'].copy(), row['position'].copy()
        raise ValueError('No synchronized sensor motion sample for capture/control clock')

    def relative(self, capture_s, control_s):
        if control_s < capture_s-1.e-9:
            raise ValueError('Motion transport from a future capture')
        rc, pc = self.pose_at(capture_s)
        rn, pn = self.pose_at(control_s)
        return rn.T @ rc, rn.T @ (pc-pn)

    def transport(self, packet, control_s):
        # Keep the existing image boundary's recursive truth-field rejection
        # even when this adapter is called outside the deployment publisher.
        _check_payload(packet)
        result = deepcopy(packet)
        for stream in ('front', 'rear'):
            item = result[stream]
            r, t = self.relative(item['capture_sim_time_s'], control_s)
            points = np.asarray(item.get('points_b', []), dtype=float)
            if points.size:
                if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
                    raise ValueError('Invalid image path for motion transport')
                item['points_b'] = (points @ r.T+t).tolist()
            if item.get('ground_plane_b') is not None:
                plane = _plane(item['ground_plane_b'])
                normal = r @ plane[:3]
                item['ground_plane_b'] = np.r_[normal, plane[3]-normal @ t].tolist()
            item['source_coordinate_frame'] = item.get('frame_id', 'b2_base_at_capture')
            item['frame_id'] = 'b2_base_at_control_sensor_estimate'
            item['coordinate_time_s'] = float(control_s)
            item['coordinate_transport'] = 'leg_IMU_local_estimate; original capture clocks retained'
        return result


class FootLoadSensors:
    """Named MuJoCo touch sensor boundary, never contact buffers/world state."""
    def __init__(self, model):
        import mujoco
        self.addresses = []
        for foot in FEET:
            sensor = model.sensor(foot+'_foot_load')
            site = model.site(foot+'_foot_load_site')
            if (sensor.type != mujoco.mjtSensor.mjSENS_TOUCH or sensor.dim != 1
                    or sensor.objid != site.id or site.bodyid != model.body(foot+'_foot').id):
                raise ValueError('Foot load sensor binding differs')
            self.addresses.append(int(sensor.adr))
        imu = model.sensor('imu_acceleration')
        if (imu.type != mujoco.mjtSensor.mjSENS_ACCELEROMETER or imu.dim != 3
                or imu.objid != model.site('imu').id):
            raise ValueError('IMU accelerometer binding differs')
        self.accel_address = int(imu.adr)

    def read(self, data):
        return finite(data.sensordata[self.addresses], (4,)).copy()

    def proprioception(self, sensor, data):
        result = dict(sensor)
        result['accelerometer_b_m_s2'] = finite(
            data.sensordata[self.accel_address:self.accel_address+3], (3,)).copy()
        return result


class SensorRearPathMemory:
    """Original bounded/gap-aware cache, transported by SENSOR local motion.

    Capture stamps remain original; the points arriving at ingest have already
    been transported to this control tick, so store using this tick's pose.
    """
    def __init__(self, history, horizon_s):
        self.history, self.horizon_s = history, horizon_s
        self.frames, self.identity = deque(), None
        self.last_stamp = None

    def advance(self, now, dt, sent_command):
        # Deliberately do not integrate the sent command as measured motion.
        self.frames = deque(f for f in self.frames if now-f['stamp'] <= self.horizon_s)

    def current_pose(self):
        return self.history.pose_at(self.history.samples[-1]['time_s'])

    def ingest(self, points, plane, basis, generation, sequence, stamp, now):
        identity = (generation, sequence)
        if self.identity is not None:
            if identity < self.identity:
                raise ValueError('Sensor path memory identity moved backwards')
            if identity == self.identity:
                if stamp != self.last_stamp:
                    raise ValueError('Repeated cached frame changed capture timestamp')
                return
            if generation != self.identity[0]:
                self.frames.clear()
        self.identity, self.last_stamp = identity, stamp
        if now-stamp > self.horizon_s:
            return
        points = np.asarray(points, dtype=float)
        origin = -plane[3]*plane[:3]
        xy = (points-origin) @ basis[:, :2]
        keep = np.abs(xy[:,1]) <= .35
        if np.count_nonzero(keep) < 6 or np.ptp(xy[keep,0]) < .10:
            return
        points, xy = points[keep], xy[keep]
        r, p = self.current_pose()
        for frame in self.frames:
            old = ((frame['points']-p) @ r-origin) @ basis[:, :2]
            order = np.argsort(old[:,0])
            overlap = (xy[:,0] >= old[:,0].min()) & (xy[:,0] <= old[:,0].max())
            if np.count_nonzero(overlap) >= 6:
                residual = xy[overlap,1] - np.interp(xy[overlap,0], old[order,0], old[order,1])
                if np.median(np.abs(residual)) > .025:
                    self.frames.clear()
                    break
        self.frames.append(dict(points=points @ r.T+p, rotation=r, position=p,
                                stamp=float(stamp), sequence=int(sequence)))

    def paths(self, plane, basis, now):
        self.advance(now, 0., None)
        r, p = self.current_pose()
        origin = -plane[3]*plane[:3]
        points, sources = [], []
        for frame in self.frames:
            displacement = float(np.linalg.norm(p-frame['position']))
            angle = math.acos(float(np.clip((np.trace(r.T @ frame['rotation'])-1)/2, -1, 1)))
            if displacement > .16 or angle > .15:
                continue
            xy = ((frame['points']-p) @ r-origin) @ basis[:, :2]
            points.extend(xy)
            sources.extend([(frame['stamp'], frame['sequence'], displacement)]*len(xy))
        if len(points) < 6:
            return []
        points, sources = np.array(points), np.array(sources)
        grouped, metadata = [], []
        bins = np.floor(points[:,0]/.005).astype(int)
        for key in np.unique(bins):
            mask = bins == key
            if np.ptp(points[mask,1]) <= .03:
                grouped.append(np.median(points[mask], axis=0))
                metadata.append(sources[mask])
        if len(grouped) < 6:
            return []
        grouped = np.asarray(grouped)
        cuts = np.flatnonzero(np.linalg.norm(np.diff(grouped, axis=0), axis=1) > .025)+1
        paths = []
        for indices in np.split(np.arange(len(grouped)), cuts):
            if len(indices) < 6 or np.ptp(grouped[indices,0]) < .12:
                continue
            source = np.concatenate([metadata[i] for i in indices])
            paths.append((origin+grouped[indices] @ basis[:,:2].T, dict(
                rear_memory_source_age_max_s=float(now-source[:,0].min()),
                rear_memory_source_age_min_s=float(now-source[:,0].max()),
                rear_memory_source_capture_timestamp_min=float(source[:,0].min()),
                rear_memory_source_sequences=sorted(set(source[:,1].astype(int).tolist())),
                rear_memory_command_displacement_max_m=None,
                rear_memory_sensor_displacement_max_m=float(source[:,2].max()),
                rear_memory_point_count=len(indices), rear_memory_motion_source='leg_IMU_local_estimate',
                rear_memory_max_age_s=self.horizon_s)))
        return paths


class MotionCompensatedVisualController(VD12VisualController):
    def __init__(self, *args, motion_history, **kwargs):
        super().__init__(*args, **kwargs)
        from vd5_mujoco_control import quaternion_matrix
        self.motion = motion_history
        self.initial_pose = self.motion.pose_at(self.motion.samples[-1]['time_s'])
        self.initial_tcp = self._start_position.copy()
        self.initial_quaternion = self._start_quaternion.copy()
        self.initial_axis_rotation = quaternion_matrix(self._start_quaternion)
        if self._path_memory is not None:
            self._path_memory = SensorRearPathMemory(self.motion, self.rear_path_memory_s)
        self.transport_error, self.raw_digest = None, None
        self.previous_work_point = None
        self.pending_work_point = None
        self.continuity_diagnostic = {}
        self.command_anchor = None
        self.support_wait = None
        self.wait_packet = None

    def _digest(self, packet):
        # Compensated geometry legitimately changes as the base moves while
        # processing the SAME image. Verify immutability on the RAW payload.
        return self.raw_digest

    def _validate(self, packet, now):
        if self.transport_error is not None:
            raise ValueError(self.transport_error)
        if self._generation is not None:
            if packet['capture_generation'] != self._generation:
                raise ValueError('capture generation changed; explicit reset with new encoder FK is required')
            if (packet['rear']['sequence'] == self._sequence
                    and self.raw_digest != self._payload_digest):
                raise ValueError('repeated frame sequence changed its image-derived geometry')
        if self.wait_packet is not None:
            previous_identity, previous_stamps, previous_digest = self.wait_packet
            identity = (packet['capture_generation'], packet['rear']['sequence'])
            stamps = tuple(packet[name]['capture_timestamp'] for name in ('front', 'rear'))
            if identity < previous_identity:
                raise ValueError('camera capture identity moved backwards during support wait')
            if identity == previous_identity and (stamps != previous_stamps or self.raw_digest != previous_digest):
                raise ValueError('repeated frame changed its capture timestamp or image-derived geometry during support wait')
            if identity > previous_identity and any(a <= b for a, b in zip(stamps, previous_stamps)):
                raise ValueError('new camera sequence did not advance capture timestamps during support wait')
        result = super()._validate(packet, now)
        rotation, position = self.motion.pose_at(self.control_time_s)
        target_local = rotation @ np.asarray(result[-2])+position
        self.pending_work_point = (self.control_time_s, target_local)
        self.continuity_diagnostic = dict(rear_work_point_continuity_checked=True,
            rear_work_point_continuity_motion='sensor_local_estimate_not_world_pose',
            rear_work_point_continuity_action='reject_and_latch_stop_no_filter_or_hold',
            rear_work_point_step_m=None, rear_work_point_step_limit_m=None)
        if self.previous_work_point is not None:
            previous_time, previous_target = self.previous_work_point
            interval = self.control_time_s-previous_time
            if interval <= 0. or (interval > .040000001 and self.support_wait is None):
                raise ValueError('Rear work-point continuity clock is not a consecutive control sample')
            # Reuse the observed-path cache's 25mm identity tolerance plus the
            # configured path advance. This is an added measurement rejection,
            # never a command smoother or a relaxed physical protection.
            # Waiting does not buy a larger jump allowance. Re-observation
            # must still agree with the last accepted sensor-local work point.
            limit = .025 + self.speed*(.02 if self.support_wait is not None else interval)
            displacement = float(np.linalg.norm(target_local-previous_target))
            self.continuity_diagnostic.update(rear_work_point_step_m=displacement,
                rear_work_point_step_limit_m=limit, rear_work_point_step_interval_s=interval)
            if displacement > limit:
                raise ValueError('Rear visual work point jumped %.1fmm above %.1fmm sensor-local continuity bound' %
                                 (1000.*displacement, 1000.*limit))
        if result[-1].get('rear_memory_used'):
            result[-1]['rear_target_source'] = 'rear_rgbd_observed_path_sensor_motion_propagation'
        return result

    def _wait_for_support(self, result, packet, now):
        """Zero commanded travel; transport an OLD command, not a measurement.

        Physics, sensors and cameras keep running. No new pixels/path/plane,
        timestamps or contact are invented. Only this typed support shortfall
        is recoverable; every other error retains the existing latched stop.
        """
        if self.support_wait is None:
            self.support_wait = (now, self.control_time_s)
        anchor = self.command_anchor
        rotation, position = self.motion.pose_at(self.control_time_s)
        from vd5_mujoco_control import quaternion_multiply
        self._position = rotation.T @ (anchor['position_local']-position)
        self._quaternion = quaternion_multiply(
            matrix_quaternion(rotation.T @ anchor['rotation']), anchor['quaternion'])
        self._velocity = np.zeros(3)
        self._failed_reason = None  # Clear ONLY the typed, recoverable failure.
        self._last_now = now
        self.wait_packet = ((packet['capture_generation'], packet['rear']['sequence']),
            tuple(packet[name]['capture_timestamp'] for name in ('front', 'rear')), self.raw_digest)
        status = result['status']
        status.update(valid=False, measurement_valid=False, measurement_is_new=False,
            control_state='rear_support_wait', control_action='zero_travel_hold_last_command',
            runtime_must_terminate_on_invalid=False, failure_latched=False,
            rear_target_held=True, rear_target_directly_observed=False,
            rear_target_estimated=False, target_estimated=False, rear_memory_used=False,
            tool_pose_source='last_accepted_command_sensor_transport_NOT_reobserved',
            rear_target_source='held_command_NOT_visual_measurement',
            support_wait_elapsed_s=now-self.support_wait[0],
            support_wait_elapsed_sim_s=self.control_time_s-self.support_wait[1],
            held_setpoint_age_s=now-anchor['accepted_at'],
            held_setpoint_age_sim_s=self.control_time_s-anchor['accepted_sim_s'],
            held_setpoint_source=anchor['source'],
            held_setpoint_source_age_s=now-anchor['source']['capture_timestamp'],
            rear_target_source_age_s=None,
            capture_generation=packet['capture_generation'], source_sequence=packet['rear']['sequence'],
            capture_timestamps=[packet[name]['capture_timestamp'] for name in ('front', 'rear')],
            capture_age_s=now-min(packet[name]['capture_timestamp'] for name in ('front', 'rear')))
        result.update(base_command_b=self._velocity.tolist(),
                      tool_pose_b=np.r_[self._position, self._quaternion].tolist())

    def update(self, packet, now, dt, *, control_time_s):
        self.control_time_s = float(control_time_s)
        self.pending_work_point = None
        self.continuity_diagnostic = {}
        try:
            self.raw_digest = VD12VisualController._digest(packet)
            compensated = self.motion.transport(packet, control_time_s)
            rn, pn = self.motion.pose_at(control_time_s)
            ri, pi = self.initial_pose
            self._start_position = rn.T @ (ri @ self.initial_tcp+pi-pn)
            # Training uses q_base^-1 * q_start, not a fresh matrix-to-q
            # canonicalization at every control tick. At the pi-yaw working
            # pose, forcing q.w>=0 flips all four actor inputs as q.w crosses
            # zero even though the physical attitude is continuous.
            from vd5_mujoco_control import quaternion_multiply
            self._start_quaternion = quaternion_multiply(
                matrix_quaternion(rn.T @ ri), self.initial_quaternion)
            self.transport_error = None
        except (KeyError, TypeError, ValueError, IndexError) as error:
            compensated, self.transport_error = None, str(error)
        result = super().update(compensated, now, dt)
        status = result['status']
        status.update(self.continuity_diagnostic)
        if (not status['valid'] and status.get('failure_code') == 'rear_target_support_shortfall'
                and self.command_anchor is not None):
            self._wait_for_support(result, compensated, now)
        if status['valid']:
            resumed = self.support_wait is not None
            rotation, position = self.motion.pose_at(control_time_s)
            self.command_anchor = dict(position_local=rotation @ self._position+position,
                rotation=rotation.copy(), quaternion=self._quaternion.copy(),
                accepted_at=now, accepted_sim_s=float(control_time_s),
                source=dict(capture_generation=status['rear_target_source_capture_generation'],
                    sequences=list(status['rear_target_source_sequences']),
                    capture_timestamp=status['rear_target_source_capture_timestamp']))
            self.support_wait = self.wait_packet = None
            self.previous_work_point = self.pending_work_point
            status.update(control_action='track', resumed_from_rear_support_wait=resumed)
        status.update(motion_compensation='sensor_leg_IMU_capture_to_control',
            motion_estimate=dict(self.motion.diagnostic), control_sensor_time_s=control_time_s,
            startup_initial_anchor='encoder_FK_anchored_in_sensor_local_frame',
            motion_is_ground_truth=False, capture_timestamps_renewed=False)
        if status.get('rear_memory_used'):
            status['rear_target_estimate_assumption'] = 'previously_observed_path_propagated_by_leg_IMU_local_estimate'
            status['reason'] = 'Previously observed image path transported by sensor local motion estimate'
        return result
