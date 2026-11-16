"""Regression cases for idle startup timing and asynchronous receipt races."""
from pathlib import Path
from contextlib import ExitStack,redirect_stdout
import gzip
import io
import json
import queue
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock,patch

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'python')]
from feedback import FeedbackWait,Z1MotorResponse,sample_feedback


class FeedbackTests(unittest.TestCase):
    def state(self,b2=100.,z1=100.):
        return SimpleNamespace(b2_time=b2,z1_time=z1,tick=900,z1_mode=[1],fault=0)

    def test_asynchronous_update_uses_clock_after_snapshot_without_clamping_age(self):
        calls=[];value=self.state(100.001,100.0015)
        old_now=100.
        self.assertLess(old_now-value.z1_time,0) # Previous loop ordering rejects this valid state.
        def snapshot(lib):calls.append('snapshot');return value
        def clock():calls.append('clock');return 100.002
        state,now,report=sample_feedback(None,snapshot,clock)
        self.assertEqual(calls,['snapshot','clock']);self.assertTrue(report['fresh'])
        self.assertAlmostEqual(report['b2_age_s'],.001)
        self.assertAlmostEqual(report['z1_age_s'],.0005)
        self.assertEqual(report['z1_controller_reply_age_s'],report['z1_age_s'])
        self.assertIsNone(report['z1_motor_sample_age_s'])
        self.assertFalse(report['z1_motor_freshness_known'])
        self.assertEqual(report['feedback_freshness_scope'],'B2_DDS_and_Z1_controller_reply')
        self.assertEqual(state.b2_time,100.001);self.assertEqual(state.z1_time,100.0015)

    def test_idle_after_eight_seconds_does_not_exit_on_one_invalid_sample(self):
        wait=FeedbackWait()
        for step in range(600):
            now=step*.02
            # One 20-ms stale observation every second, including after 8 s.
            self.assertIsNone(wait.update(dict(fresh=step%50!=49),now))
        self.assertIsNone(wait.update(dict(fresh=True),12.))
        self.assertEqual(wait.gap(12.),0.)

    def test_continuous_missing_feedback_exits_after_eight_seconds_and_resets_on_recovery(self):
        wait=FeedbackWait()
        self.assertIsNone(wait.update(dict(fresh=False),10.))
        self.assertIsNone(wait.update(dict(fresh=False),17.99))
        self.assertIn('continuous',wait.update(dict(fresh=False),18.))
        self.assertIsNone(wait.update(dict(fresh=True),19.))
        self.assertIsNone(wait.update(dict(fresh=False),30.))
        self.assertIsNone(wait.update(dict(fresh=False),37.99))
        self.assertIn('continuous',wait.update(dict(fresh=False),38.))

    def test_after_takeover_a_real_expired_sample_fails_without_startup_grace(self):
        wait=FeedbackWait()
        self.assertIsNone(wait.update(dict(fresh=True),100.,acquired=True))
        state,now,report=sample_feedback(None,lambda lib:self.state(100.02,99.8),lambda:100.03)
        self.assertFalse(report['fresh'])
        self.assertGreater(report['z1_age_s'],.1)
        self.assertIn('after takeover',wait.update(report,now,acquired=True))

    def test_missing_future_and_stale_timestamps_are_not_falsely_marked_fresh(self):
        for b2,z1 in ((0.,100.),(100.,0.),(100.001,100.),(100.,100.001),(99.8,100.),(100.,99.8)):
            with self.subTest(b2=b2,z1=z1):
                state,now,report=sample_feedback(None,lambda lib:self.state(b2,z1),lambda:100.)
                self.assertFalse(report['fresh'])
                self.assertEqual(state.b2_time,b2);self.assertEqual(state.z1_time,z1)

    def test_cached_moving_vector_is_diagnosed_without_inventing_motor_time(self):
        from native_io import State
        state=State();state.z1_time=99.;state.dq[14]=1.2
        monitor=Z1MotorResponse()
        self.assertFalse(monitor.observe(state,100.)['z1_motor_vector_repeated_while_moving'])
        result=monitor.observe(state,100.25)
        self.assertTrue(result['z1_motor_vector_repeated_while_moving'])
        self.assertEqual(result['z1_motor_vector_unchanged_s'],.25)
        self.assertEqual(state.z1_time,99.)
        state.q[14]=.001
        self.assertFalse(monitor.observe(state,100.3)['z1_motor_vector_repeated_while_moving'])

    def test_stationary_quantization_is_not_reported_as_a_moving_vector(self):
        from native_io import State
        state=State();monitor=Z1MotorResponse()
        monitor.observe(state,100.)
        self.assertFalse(monitor.observe(state,110.)['z1_motor_vector_repeated_while_moving'])


class FeedbackMainLoopTests(unittest.TestCase):
    def exercise_idle(self,missing,takeover=False,post_stale=False,frozen_moving=False):
        """Run the real main loop with in-memory IO; never load native IO."""
        import run
        from native_io import State
        from control import DEFAULT,limits,Session
        clock=SimpleNamespace(now=100.,step=0)
        lib=Mock();lib.vd2_open.return_value=0
        output=Mock();output.get_nowait.side_effect=queue.Empty
        stop=Mock();stop.is_set.return_value=False
        context=Mock();context.Queue.return_value=output;context.Event.return_value=stop
        context.Process.return_value.is_alive.return_value=True
        def snapshot(unused):
            clock.step+=1;clock.now=100.+clock.step*.02
            state=State();state.q[:]=DEFAULT;state.quaternion[0]=1;state.tick=clock.step
            state.z1_mode[0]=1;state.b2_time=clock.now-.002
            state.z1_time=clock.now-(.2 if missing(clock.step) else .001)
            if frozen_moving:state.dq[13]=2.36
            if takeover:
                state.q[:8]=[.29,-.29,.29,-.29,1.21,1.21,1.21,1.21]
                state.q[9:13]=[-2.8]*4;state.q[8]=0;state.q[13:18]=[0]*5
                state.keys=256 if clock.step==2 else 0
                if clock.step>=3:
                    state.q[0]=.27 # Actual pose changes while RPC is pending.
                    if post_stale:state.b2_time=clock.now-.2
            return state
        lib.vd2_acquire.return_value=0
        session=Session()
        if frozen_moving:
            session.mode='braking';session.entered=100.;session.policy_started=100.
            session.brake_arm_target=DEFAULT[[8,13,14,15,16,17]].copy()
        actor=SimpleNamespace(bounds=limits(),step=Mock(return_value=(DEFAULT.copy(),dict(observation=[0]*70))))
        with tempfile.TemporaryDirectory() as directory,ExitStack() as stack:
            root=Path(directory);(root/'config').mkdir()
            (root/'config/deployment.yaml').write_text(json.dumps(dict(forward_speed_mps=.2,
                policy_period_s=.02,z1_controller_ip='127.0.0.1',z1_to_port=8071,z1_own_port=8072)))
            for owner,name,value in ((run,'ROOT',root),(run,'library',Mock(return_value=lib)),
                (run,'snapshot',snapshot),(run,'calibration',Mock()),(run,'Actor',Mock(return_value=actor)),
                (run,'Session',Mock(return_value=session)),
                (run,'URDFToolKinematics',Mock()),(run,'send',Mock()),
                (run.mp,'get_context',Mock(return_value=context)),
                (run.time,'monotonic',lambda:clock.now),(run.time,'sleep',lambda seconds:None),
                (sys,'argv',['run.py','--run','--seconds','12'])):
                stack.enter_context(patch.object(owner,name,value))
            captured=stack.enter_context(redirect_stdout(io.StringIO()))
            error=None
            try:run.main()
            except RuntimeError as exc:error=str(exc)
            if takeover:lib.vd2_acquire.assert_called_once()
            else:
                lib.vd2_acquire.assert_not_called()
                if not frozen_moving:run.send.assert_not_called()
            lib.vd2_close.assert_called_once()
            logs=list(root.glob('runs/*/control.jsonl.gz'));self.assertEqual(len(logs),1)
            with gzip.open(logs[0],'rt') as stream:records=[json.loads(line) for line in stream]
            return error,records,clock.step,captured.getvalue()

    def test_idle_main_survives_async_updates_and_intermittent_old_z1_for_twelve_seconds(self):
        error,records,steps,output=self.exercise_idle(lambda step:step%50==49)
        self.assertIsNone(error);self.assertGreaterEqual(steps,600)
        self.assertIn('idle: waiting for A',output)
        healthy=[record for record in records if record.get('fresh')]
        self.assertTrue(any(record['monotonic']>111 for record in healthy))
        self.assertTrue(all(record['b2_age_s']>=0 and record['z1_age_s']>=0 for record in healthy))
        self.assertTrue(any(not record['fresh'] and record['z1_age_s']>.1 for record in records))

    def test_main_stops_actor_on_cached_moving_arm_even_with_fresh_controller_replies(self):
        error,records,steps,output=self.exercise_idle(lambda step:False,frozen_moving=True)
        self.assertIsNone(error)
        before=[r for r in records if r['state']=='braking']
        after=[r for r in records if r.get('motor_feedback_blocked')]
        self.assertTrue(before);self.assertTrue(after)
        self.assertLessEqual(after[0]['monotonic']-records[0]['monotonic'],.221)
        self.assertTrue(all(r['fresh'] for r in after))
        self.assertTrue(all('observation' not in r for r in after))
        self.assertTrue(all(r['state']=='held' for r in after))
        self.assertTrue(all(r['target_q']==before[-1]['target_q'] for r in after))
        self.assertTrue(all(r['leg_kp']==before[-1]['leg_kp'] for r in after))

    def test_idle_main_logs_actual_z1_age_and_exits_on_continuous_loss(self):
        error,records,steps,output=self.exercise_idle(lambda step:True)
        self.assertIn('continuous seconds',error);self.assertLess(steps,450)
        self.assertTrue(records);self.assertTrue(all(not record['fresh'] for record in records))
        self.assertGreaterEqual(records[-1]['waiting_s'],8.)
        self.assertAlmostEqual(records[-1]['z1_age_s'],.2)
        self.assertEqual(records[-1]['native_fault'],0)

    def test_getup_first_reference_uses_post_acquire_encoder_sample(self):
        error,records,steps,output=self.exercise_idle(lambda step:False,takeover=True)
        self.assertIsNone(error)
        first=next(r for r in records if r.get('state')=='pre_stand')
        self.assertEqual(first['q'][0],.27);self.assertEqual(first['target_q'][0],.27)
        self.assertEqual(first['event'],'stand')

    def test_expired_post_acquire_feedback_is_rejected_before_motion(self):
        error,records,steps,output=self.exercise_idle(lambda step:False,takeover=True,post_stale=True)
        self.assertIn('after takeover',error)
        self.assertFalse(any(r.get('state')=='pre_stand' for r in records))


if __name__=='__main__':unittest.main()
