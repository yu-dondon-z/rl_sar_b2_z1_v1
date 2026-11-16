"""CPU-only provenance and orchestration checks; no simulator is launched."""

import ast
import contextlib
import copy
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
import deploy_vd12_vision as entry
from vd1_frames import VD1FramePublisher
from vd12_visual_control import VD12VisualController
from vd5_mujoco_control import quaternion_from_euler_xyz
from vd5_visual_control import SCHEMA


def packet(generation=42, sequence=0, stamp=10.):
    def stream(xs):
        xs = np.asarray(xs)
        return dict(valid=True, capture_generation=generation, sequence=sequence,
                    capture_timestamp=stamp, ground_plane_b=[0., 0., 1., .5],
                    points_b=np.c_[xs, np.zeros_like(xs), np.full_like(xs, -.5)].tolist())
    return dict(schema=SCHEMA, valid=True, capture_generation=generation,
                front=stream(np.linspace(.15, 2., 60)),
                rear=stream(np.linspace(-1.5, -.2, 60)))


class VisionEntryCPU(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = (ROOT / 'scripts/deploy_vd12_vision.py').read_text()
        cls.tree = ast.parse(cls.source)
        cls.main = next(node for node in cls.tree.body
                        if isinstance(node, ast.FunctionDef) and node.name == 'main')

    def text(self, node):
        # The deployed crack_vision interpreter is Python 3.8 (no ast.unparse).
        return ast.get_source_segment(self.source, node)

    def publisher(self, directory):
        calibration = dict(position=[0., 0., 0.], quaternion_wxyz=[1., 0., 0., 0.],
                           intrinsic_matrix=[[100., 0., 3.], [0., 100., 2.], [0., 0., 1.]])
        publisher = VD1FramePublisher(directory, calibration, calibration, None, generation=42)
        image = np.zeros((4, 6, 3), dtype=np.uint8)
        self.assertTrue(publisher.publish(image, image, np.ones((4, 6)), front_frame=0,
                                         rear_frame=0, capture_timestamp=10., sim_time_s=0.))
        return publisher

    def controller(self):
        return VD12VisualController([-.95, 0., .35], quaternion_from_euler_xyz(0., .3, np.pi),
                                    stale_timeout_s=.15)

    def test_default_is_final_actor_and_explicit_rhanet(self):
        args = entry.parse([])
        self.assertEqual(args.checkpoint.name, 'model_5999.pt')
        self.assertEqual(args.vision_mode, 'rhanet')
        self.assertFalse(args.viewer)
        self.assertEqual(entry.parse(['--vision-mode', 'classical']).vision_mode, 'classical')
        self.assertEqual(args.duration, 30.)
        self.assertEqual(args.rear_centerline_mode, 'cross_sections')
        self.assertEqual(args.rear_path_memory_s, .6)
        self.assertEqual(args.rear_occlusion_extension, .05)
        self.assertFalse(args.historical_tangent_estimate)
        self.assertFalse(entry.parse(['--no-historical-tangent-estimate']).historical_tangent_estimate)
        self.assertTrue(entry.parse(['--historical-tangent-estimate']).historical_tangent_estimate)
        # The reusable library remains conservative unless the runtime opts in.
        controller = self.controller()
        self.assertIsNone(controller._path_memory)
        self.assertEqual(controller.max_occlusion_extension, 0.)

    def test_duration_and_timing_reject_unsupported_values(self):
        for flag, values in (
            ('--duration', ['0', '.005', '.019', '.021', '30.02', 'nan', 'inf']),
            ('--vision-max-age', ['0', '.099', '1.01', 'nan', 'inf']),
            ('--warmup-timeout', ['0', '.99', '90.01', 'nan', 'inf']),
            ('--speed', ['0', '-.2', '.401', 'nan', 'inf']),
            ('--curvature', ['-.01', '.151', 'nan', 'inf']),
            ('--rear-path-memory-s', ['-.01', '.801', 'nan', 'inf']),
            ('--rear-occlusion-extension', ['-.001', '.0501', 'nan', 'inf']),
        ):
            for value in values:
                with self.subTest(flag=flag, value=value), contextlib.redirect_stderr(io.StringIO()):
                    with self.assertRaises(SystemExit):
                        entry.parse([flag, value])
        for duration in ('.02', '.06', '30'):
            self.assertEqual(entry.parse(['--duration', duration]).duration, float(duration))
        for value in ('0', '.6', '.8'):
            self.assertEqual(entry.parse(['--rear-path-memory-s', value]).rear_path_memory_s, float(value))
        for value in ('0', '.03', '.05'):
            self.assertEqual(entry.parse(['--rear-occlusion-extension', value]).rear_occlusion_extension,
                             float(value))
        self.assertEqual(entry.parse(['--rear-centerline-mode', 'skeleton']).rear_centerline_mode,
                         'skeleton')
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            entry.parse(['--rear-centerline-mode', 'invented'])

    def test_only_published_pair_enters_control(self):
        with tempfile.TemporaryDirectory() as directory:
            publisher = self.publisher(directory)
            path = Path(directory) / 'packet.json'
            image = packet()
            path.write_text(json.dumps(image))
            actual, reason = entry.visual_packet(path, publisher)
            self.assertEqual(actual, image)
            self.assertEqual(reason, '')
            self.assertTrue(self.controller().update(actual, now=10., dt=.02)['status']['valid'])
            for change in ('unpublished', 'generation', 'forged_stamp', 'unpaired'):
                forged = copy.deepcopy(image)
                if change == 'unpublished':
                    for stream in ('front', 'rear'):
                        forged[stream]['sequence'] = 1
                elif change == 'generation':
                    forged['capture_generation'] = 43
                elif change == 'forged_stamp':
                    forged['rear']['capture_timestamp'] = 10.01
                else:
                    forged['rear']['sequence'] = 1
                path.write_text(json.dumps(forged))
                with self.subTest(change=change):
                    actual, reason = entry.visual_packet(path, publisher)
                    self.assertIsNone(actual)
                    self.assertTrue(reason)

    def test_nested_privileged_fields_are_rejected_before_controller(self):
        with tempfile.TemporaryDirectory() as directory:
            publisher = self.publisher(directory)
            path = Path(directory) / 'packet.json'
            for key in ('world_pose', 'root_pos', 'actual_tcp', 'seam_points_w', 'ground_truth'):
                image = packet()
                image['rear']['metadata'] = {'nested': [{key: [0., 0., 0.]}]}
                path.write_text(json.dumps(image))
                with self.subTest(key=key):
                    actual, reason = entry.visual_packet(path, publisher)
                    self.assertIsNone(actual)
                    self.assertIn('privileged field', reason)

    def test_simulated_capture_clock_is_checked_against_publication_and_current_clock(self):
        image = packet()
        for name in ('front', 'rear'):
            image[name]['capture_sim_time_s'] = .10
        ages = entry.capture_clock_ages(image, .16, {0: .10}, .15)
        self.assertAlmostEqual(ages['capture_age_sim_s'], .06)
        for change in ('stale', 'future', 'forged', 'unpublished', 'missing', 'nan'):
            candidate = copy.deepcopy(image)
            now, history = .16, {0: .10}
            if change == 'stale': now = .26
            if change == 'future': now = .09
            if change == 'forged': candidate['front']['capture_sim_time_s'] = .12
            if change == 'unpublished': history = {}
            if change == 'missing': candidate['rear'].pop('capture_sim_time_s')
            if change == 'nan': candidate['rear']['capture_sim_time_s'] = float('nan')
            with self.subTest(change=change), self.assertRaises(ValueError):
                entry.capture_clock_ages(candidate, now, history, .15)
        # A fresh host timestamp cannot renew an old simulated capture.
        with tempfile.TemporaryDirectory() as directory:
            publisher = self.publisher(directory)
            path = Path(directory) / 'packet.json'
            for name in ('front', 'rear'):
                image[name]['capture_sim_time_s'] = 0.
            path.write_text(json.dumps(image))
            self.assertIsNotNone(entry.visual_packet(path, publisher, sim_time_s=.10,
                                                     capture_sim_times={0: 0.}, max_age_s=.15)[0])
            actual, reason = entry.visual_packet(path, publisher, sim_time_s=.16,
                                                 capture_sim_times={0: 0.}, max_age_s=.15)
            self.assertIsNone(actual)
            self.assertIn('simulated capture is stale', reason)

    def test_missing_malformed_and_nonfinite_packets_fail_closed(self):
        with tempfile.TemporaryDirectory() as directory:
            publisher = self.publisher(directory)
            path = Path(directory) / 'packet.json'
            self.assertIsNone(entry.visual_packet(path, publisher)[0])
            for encoded in ('{', 'null', '[]', '{"valid": true}',
                            json.dumps(dict(packet(), extra=float('nan')))):
                path.write_text(encoded)
                with self.subTest(encoded=encoded):
                    self.assertIsNone(entry.visual_packet(path, publisher)[0])

    def test_publication_and_repeated_processing_do_not_refresh_capture_age(self):
        with tempfile.TemporaryDirectory() as directory:
            publisher = self.publisher(directory)
            path = Path(directory) / 'packet.json'
            image = packet()
            image['timestamp'] = 1.e12  # A processing timestamp is not capture freshness.
            path.write_text(json.dumps(image))
            actual, _ = entry.visual_packet(path, publisher)
            controller = self.controller()
            first = controller.update(actual, now=10., dt=.02)
            held = controller.update(actual, now=10.10, dt=.02)
            expired = controller.update(actual, now=10.16, dt=.02)
            self.assertTrue(first['status']['measurement_is_new'])
            self.assertTrue(held['status']['valid'])
            self.assertFalse(held['status']['measurement_is_new'])
            self.assertAlmostEqual(held['status']['capture_age_s'], .10)
            self.assertFalse(expired['status']['valid'])
            self.assertTrue(expired['status']['failure_latched'])
            self.assertAlmostEqual(expired['status']['capture_age_s'], .16)
            np.testing.assert_array_equal(expired['base_command_b'], [0., 0., 0.])

    def test_actor_commands_are_visual_and_proprioceptive_not_world_teacher(self):
        calls = [node for node in ast.walk(self.main) if isinstance(node, ast.Call)]
        actor_call = next(node for node in calls if self.text(node.func) == 'shared.actor_observation')
        self.assertEqual([self.text(value) for value in actor_call.args],
                         ['base_command', 'pose', 'sensor', 'default', 'previous'])
        values = {self.text(node.targets[0]): self.text(node.value)
                  for node in ast.walk(self.main) if isinstance(node, ast.Assign)
                  and len(node.targets) == 1}
        self.assertEqual(values['base_command'], "np.asarray(visual['base_command_b'])")
        self.assertEqual(values['pose'], "np.asarray(visual['tool_pose_b'])")
        self.assertEqual(values['sensor'], 'adapter.read_proprioception(data)')
        # Curvature defines rendered pixels only; never a controller/teacher argument.
        curvature = [node for node in ast.walk(self.main)
                     if isinstance(node, ast.Attribute) and isinstance(node.value, ast.Name)
                     and node.value.id == 'args' and node.attr == 'curvature']
        self.assertEqual(len(curvature), 1)
        controller = next(node for node in ast.walk(self.main)
                          if isinstance(node, ast.FunctionDef) and node.name == 'make_controller')
        self.assertNotIn('data.', self.text(controller))
        self.assertIn('VD12VisualController(initial_p, initial_q', self.text(controller))
        self.assertIn('trailing_distance=.8', self.text(controller))
        self.assertIn('rear_path_memory_s=args.rear_path_memory_s', self.text(controller))
        self.assertIn('max_occlusion_extension=args.rear_occlusion_extension', self.text(controller))
        self.assertFalse(any(self.text(node.func).endswith('.world_seam_command') for node in calls))

    def test_memory_advances_prior_sent_command_before_current_validation(self):
        controller = VD12VisualController([-.95, 0., .35],
                                          quaternion_from_euler_xyz(0., .3, np.pi),
                                          stale_timeout_s=.15, rear_path_memory_s=.6,
                                          max_occlusion_extension=.05)
        prior = None
        for tick in range(61):
            stamp = 10. + tick * .02
            prior = controller.update(packet(sequence=tick, stamp=stamp), now=stamp, dt=.02)
            self.assertTrue(prior['status']['valid'], prior['status']['reason'])
        events = []
        advance = controller._path_memory.advance
        validate = controller._validate

        def observed_advance(now, dt, sent_command):
            events.append(('advance', now, dt, np.asarray(sent_command).copy()))
            return advance(now, dt, sent_command)

        def observed_validate(image, now):
            events.append(('validate', now))
            return validate(image, now)

        controller._path_memory.advance = observed_advance
        controller._validate = observed_validate
        result = controller.update(packet(sequence=61, stamp=11.22), now=11.22, dt=.02)
        self.assertTrue(result['status']['valid'], result['status']['reason'])
        self.assertEqual([event[0] for event in events], ['advance', 'validate'])
        self.assertEqual(events[0][1:3], (11.22, .02))
        np.testing.assert_array_equal(events[0][3], prior['base_command_b'])

    def test_runtime_records_memory_estimate_and_source_age_separately(self):
        self.assertIn("'--rear-centerline-mode', args.rear_centerline_mode", self.text(self.main))
        self.assertIn("'--rear-component-mode', 'vd14_rgb_evidence' if version == 14 else 'workpoint_support'",
                      self.text(self.main))
        self.assertIn("'--rear-target-occlusion-extension-m', str(args.rear_occlusion_extension)",
                      self.text(self.main))
        for required in ("memory_steps += int(status.get('rear_memory_used', False))",
                         "estimated_steps += int(status.get('target_estimated', False))",
                         "source_age = status.get('rear_target_source_age_s')",
                         'maximum_target_source_age_s=maximum_target_source_age',
                         'maximum_accepted_capture_age_s=maximum_capture_age',
                         'historical_path_actor_steps=memory_steps'):
            self.assertIn(required, self.text(self.main))

    def test_optional_cache_cannot_veto_valid_current_workpoint_from_far_tail(self):
        controller = VD12VisualController([-.95, 0., .35],
                                          quaternion_from_euler_xyz(0., .3, np.pi),
                                          rear_path_memory_s=.6, max_occlusion_extension=.05)
        image = packet()
        xs = np.linspace(-2., -.2, 240)
        ys = .5 * np.clip((-1.1 - xs) / .6, 0., 1.)
        points = np.c_[xs, ys, np.full_like(xs, -.5)]
        image['rear']['points_b'] = points.tolist()
        before = copy.deepcopy(image)
        output = controller.update(image, now=10., dt=.02)
        status = output['status']
        self.assertTrue(status['valid'], status['reason'])
        self.assertTrue(status['measurement_valid'])
        self.assertTrue(status['rear_target_directly_observed'])
        self.assertFalse(status['rear_memory_used'])
        np.testing.assert_allclose(status['rear_target_b'], [-.8, 0., -.5], atol=1.e-10)
        self.assertEqual(image, before)  # Working-point geometry is not cropped.
        self.assertTrue(controller._path_memory.frames)
        for frame in controller._path_memory.frames:
            self.assertLessEqual(np.max(np.abs(frame['points'][:, 1])), .35 + 1.e-12)

    def test_cache_short_support_is_skipped_without_fabricating_or_vetoing_target(self):
        controller = VD12VisualController([-.95, 0., .35],
                                          quaternion_from_euler_xyz(0., .3, np.pi),
                                          rear_path_memory_s=.6, max_occlusion_extension=.05)
        image = packet()
        xs = np.linspace(-.85, -.76, 40)
        image['rear']['points_b'] = np.c_[xs, np.zeros_like(xs), np.full_like(xs, -.5)].tolist()
        output = controller.update(image, now=10., dt=.02)
        status = output['status']
        self.assertTrue(status['valid'], status['reason'])
        self.assertTrue(status['rear_target_directly_observed'])
        self.assertFalse(status['rear_memory_used'])
        self.assertFalse(status['target_estimated'])
        self.assertEqual(list(controller._path_memory.frames), [])

    def test_capture_timestamp_precedes_render_and_is_not_rewritten(self):
        capture = next(node for node in ast.walk(self.main)
                       if isinstance(node, ast.FunctionDef) and node.name == 'capture')
        stamps = [node for node in ast.walk(capture) if isinstance(node, ast.Assign)
                  and any(isinstance(target, ast.Name) and target.id == 'stamp' for target in node.targets)]
        self.assertEqual(len(stamps), 1)
        self.assertEqual(self.text(stamps[0].value), 'time.monotonic()')
        render = [node for node in ast.walk(capture) if isinstance(node, ast.Call)
                  and self.text(node.func) == 'renderer.render']
        self.assertTrue(all(stamps[0].lineno < node.lineno for node in render))
        publish = next(node for node in ast.walk(capture) if isinstance(node, ast.Call)
                       and self.text(node.func) == 'publisher.publish')
        self.assertEqual(self.text(next(value.value for value in publish.keywords
                                         if value.arg == 'capture_timestamp')), 'stamp')

    def test_first_valid_used_once_then_invalid_stops_before_next_actor(self):
        loop = next(node for node in ast.walk(self.main) if isinstance(node, ast.For)
                    and isinstance(node.target, ast.Name) and node.target.id == 'step')
        first = next(node for node in loop.body if isinstance(node, ast.If)
                     and self.text(node.test) == 'step == 0')
        self.assertEqual(self.text(first.body[0]), 'visual = first_visual')
        self.assertFalse(any(isinstance(node, ast.Call) and self.text(node.func) == 'controller.update'
                             for node in ast.walk(ast.Module(body=first.body, type_ignores=[]))))
        invalid = next(node for node in loop.body if isinstance(node, ast.If)
                       and self.text(node.test) == "not status['valid'] and not support_waiting")
        self.assertTrue(any(isinstance(node, ast.Break) for node in invalid.body))
        # The sole exception is explicitly VD14+sensor transport, tested
        # behaviorally too; VD12/13 must still break before actor or physics.
        wait_guard = next(node for node in self.tree.body if isinstance(node, ast.FunctionDef)
                          and node.name == 'rear_support_wait_can_step')
        self.assertIn("version == 14 and motion_compensation == 'leg_imu'", self.text(wait_guard))
        self.assertIn("status.get('failure_code') == 'rear_target_support_shortfall'", self.text(wait_guard))
        for node in ast.walk(loop):
            if isinstance(node, ast.Call) and self.text(node.func) in (
                    'shared.actor_observation', 'adapter.apply_targets', 'constraint.step'):
                self.assertLess(invalid.end_lineno, node.lineno)

    def test_native_observer_and_clocks_remain_explicit(self):
        calls = [node for node in ast.walk(self.main) if isinstance(node, ast.Call)]
        step = next(node for node in calls if self.text(node.func) == 'constraint.step')
        observer = next(value.value for value in step.keywords if value.arg == 'evaluation_observer')
        self.assertEqual(self.text(observer), 'observe_native')
        self.assertIn('native_force_sample_time_s=(count-1)*.005', self.text(self.main))
        self.assertIn('state_sample_time_s=count*.005', self.text(self.main))
        snapshot = next(node for node in self.tree.body
                        if isinstance(node, ast.FunctionDef) and node.name == 'native_snapshot')
        self.assertIn('data.actuator_force[adapter.actuator_ids18]', self.text(snapshot))
        self.assertIn('data.qfrc_constraint[dofs]', self.text(snapshot))
        self.assertNotIn('target', self.text(snapshot))

    def test_child_viewer_renderer_cleanup_and_rejected_exit_are_present(self):
        cleanup = next(node for node in self.main.body if isinstance(node, ast.Try)).finalbody
        text = '\n'.join(self.text(node) for node in cleanup)
        for required in ('captures.close()', 'process.terminate()', 'process.wait(timeout=5.)',
                         'process.kill()', 'vision_log.close()', 'viewer.close()', 'renderer.close()',
                         "completed = reason == 'duration_complete'",
                         'passed = completed and support_wait_steps == 0', "output/'summary.json'"):
            self.assertIn(required, text)
        self.assertIn("return 0 if reason == 'duration_complete' else 2", self.text(self.main))


if __name__ == '__main__':
    unittest.main()
