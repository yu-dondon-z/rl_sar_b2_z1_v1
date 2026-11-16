import copy
import unittest

import numpy as np

from vd5_rear_path_memory import RearPathMemory, rotation
from vd5_visual_control import SCHEMA, VD5VisualController, rear_visual_target


class RearPathMemoryTests(unittest.TestCase):
    plane = np.array((0., 0., 1., .5))
    basis = np.eye(3)

    def points(self, start=-.77, end=-.60):
        x = np.linspace(start, end, 40)
        return np.c_[x, np.zeros_like(x), np.full_like(x, -.5)]

    def fixture(self):
        memory = RearPathMemory(.6)
        memory.advance(10., .02, (0., 0., 0.))
        memory.ingest(self.points(), self.plane, self.basis, 4, 2, 10., 10.)
        return memory

    def test_ahead_observation_reaches_work_point_without_extrapolation(self):
        memory = self.fixture()
        with self.assertRaisesRegex(ValueError, 'support is too short'):
            rear_visual_target(memory.paths(self.plane, self.basis, 10.)[0][0], self.plane, .8)
        memory.advance(10.2, .2, (.2, 0., 0.))
        path, diag = memory.paths(self.plane, self.basis, 10.2)[0]
        target, status = rear_visual_target(path, self.plane, .8)
        np.testing.assert_allclose(target, (-.8, 0., -.5), atol=1.e-10)
        self.assertTrue(status['rear_target_directly_observed'])
        self.assertAlmostEqual(diag['rear_memory_source_age_max_s'], .2)
        self.assertEqual(diag['rear_memory_source_sequences'], [2])
        self.assertAlmostEqual(diag['rear_memory_command_displacement_max_m'], .04)

    def test_capture_motion_uses_capture_time_and_preserves_curvature(self):
        memory = self.fixture()
        memory.advance(10.2, .2, (.2, 0., 0.))
        points = self.points()
        points[:, 1] = .1*points[:, 0]**2
        memory.ingest(points, self.plane, self.basis, 5, 0, 10.1, 10.2)
        # Capture happened halfway through the 4 cm commanded movement.
        stored = memory.frames[0]['points']
        np.testing.assert_allclose(stored[:, 0], points[:, 0]+.02)
        memory.advance(10.4, .2, (0., 0., .5))
        path, _ = memory.paths(self.plane, self.basis, 10.4)[0]
        expected = (stored-memory.pose[:2]) @ rotation(.1)
        self.assertAlmostEqual(path[0, 0], expected[0, 0], delta=.005)
        self.assertGreater(np.ptp(path[:, 1]), .01)

    def test_expiry_is_capture_based_and_repeated_sequence_cannot_renew_it(self):
        memory = self.fixture()
        memory.advance(10.5, .5, (0., 0., 0.))
        memory.ingest(self.points(), self.plane, self.basis, 4, 2, 10., 10.5)
        self.assertEqual(memory.frames[0]['stamp'], 10.)
        with self.assertRaisesRegex(ValueError, 'changed timestamp'):
            memory.ingest(self.points(), self.plane, self.basis, 4, 2, 10.5, 10.5)
        memory.advance(10.61, .11, (0., 0., 0.))
        self.assertEqual(memory.paths(self.plane, self.basis, 10.61), [])

    def test_gaps_and_disagreeing_paths_are_not_connected(self):
        memory = self.fixture()
        memory.ingest(self.points(-1.2, -1.), self.plane, self.basis, 4, 3, 10.01, 10.01)
        # Stamp outside the current motion ledger is ignored; advance and retry.
        memory.advance(10.02, .02, (0., 0., 0.))
        memory.ingest(self.points(-1.2, -1.), self.plane, self.basis, 4, 4, 10.02, 10.02)
        paths = memory.paths(self.plane, self.basis, 10.02)
        self.assertEqual(len(paths), 2)
        for path, _ in paths:
            with self.assertRaisesRegex(ValueError, 'support is too short|directly'):
                target, diag = rear_visual_target(path, self.plane, .9)
                if not diag['rear_target_directly_observed']:
                    raise ValueError('not directly supported')
        moved = self.points()
        moved[:, 1] = .05
        memory.ingest(moved, self.plane, self.basis, 4, 5, 10.02, 10.02)
        self.assertEqual(len(memory.frames), 1)
        self.assertAlmostEqual(memory.paths(self.plane, self.basis, 10.02)[0][0][0, 1], .05)

    def controller_fixture(self):
        def stream(x):
            return dict(valid=True, capture_generation=4, sequence=0, capture_timestamp=10.,
                        points_b=np.c_[x, np.zeros_like(x), np.full_like(x, -.5)].tolist(),
                        ground_plane_b=self.plane.tolist())
        packet = dict(schema=SCHEMA, valid=True, capture_generation=4,
                      front=stream(np.linspace(.15, 2., 40)), rear=stream(np.linspace(-.77, -.6, 40)))
        controller = VD5VisualController((-.8, 0., -.5), (1., 0., 0., 0.),
                                        trailing_distance=.8, ramp_s=.01, max_occlusion_extension=.05,
                                        rear_path_memory_s=.6)
        for i in range(26):
            for name in ('front', 'rear'):
                packet[name].update(sequence=i, capture_timestamp=10.+i*.02)
            controller.update(packet, 10.+i*.02, .02)
        missing = copy.deepcopy(packet)
        missing.update(valid=False)
        for name in ('front', 'rear'):
            missing[name].update(sequence=26, capture_timestamp=10.52)
        missing['rear'].update(valid=False, reason='geometric seam path is too short', points_b=[])
        return controller, missing

    def test_controller_can_use_history_without_current_target_visibility(self):
        controller, packet = self.controller_fixture()
        status = controller.update(packet, 10.52, .02)['status']
        self.assertTrue(status['valid'])
        self.assertFalse(status['measurement_valid'])
        self.assertEqual(status['control_state'], 'rear_memory')
        self.assertFalse(status['rear_target_directly_observed'])
        self.assertAlmostEqual(status['rear_target_b'][0], -.8, places=3)
        self.assertLessEqual(status['rear_memory_source_age_max_s'], .6)
        self.assertEqual(status['rear_memory_motion_source'], 'sent_base_commands_only')
        for i in range(27, 60):
            for name in ('front', 'rear'):
                packet[name].update(sequence=i, capture_timestamp=10.+i*.02)
            output = controller.update(packet, 10.+i*.02, .02)
        self.assertFalse(output['status']['valid'])

    def test_memory_does_not_override_pair_depth_front_or_generation_failure(self):
        for case in ('stale', 'pair', 'front', 'depth', 'generation'):
            controller, packet = self.controller_fixture()
            if case == 'stale': packet['rear']['capture_timestamp'] = 8.
            if case == 'pair': packet['rear']['sequence'] += 1
            if case == 'front': packet['front']['valid'] = False
            if case == 'depth': packet['rear']['reason'] = 'rear seam depth support too low (0.0)'
            if case == 'generation':
                packet['capture_generation'] = 5
                for name in ('front', 'rear'): packet[name]['capture_generation'] = 5
            self.assertFalse(controller.update(packet, 10.52, .02)['status']['valid'], case)


if __name__ == '__main__':
    unittest.main()
