"""VD12 selection reuses independent real pixels and unchanged geometry bounds."""
import base64
import hashlib
import json
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import zlib

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'scripts'), '/home/dong/b2_z1/scripts/vision',
               '/home/dong/RHACrackNet/code']
from crack_trajectory import CenterlineConfig
from dual_camera_perception import PerceptionConfig
from vd1_visual_rear import SeamOnlyDualPerception
from vd12_gun_mask import NearClippedEncoderGunMask
from vd12_visual_rear import VD12RearSupportPerception
from vd5_visual_control import rear_visual_target


class ComponentSelectionTests(unittest.TestCase):
    def setUp(self):
        self.perception = VD12RearSupportPerception(PerceptionConfig(mode='classical'),
                                                   rear_centerline_mode='cross_sections')
        self.config = CenterlineConfig(waypoint_spacing_px=6.)
        self.allowed = np.ones((320, 320), dtype=bool)
        self.context = dict(intrinsic=np.array([[200., 0., 160.], [0., 200., 160.], [0., 0., 1.]]),
                            transform=np.array([[0., -1., 0., -.8], [-1., 0., 0., 0.],
                                                [0., 0., -1., .5], [0., 0., 0., 1.]]),
                            plane=np.array([0., 0., 1., .5]), depth_m=np.ones((320, 320)),
                            allowed_pixels=self.allowed)
        self.perception._rear_allowed_pixels = self.allowed
        self.perception._rear_geometry_context = self.context

    def extract(self, mask):
        return self.perception._extract_with_roi(mask, self.config, (160., 210.))

    def test_short_anchor_does_not_hide_an_independent_supported_component(self):
        mask = np.zeros((320, 320), np.uint8)
        cv2.rectangle(mask, (155, 40), (165, 180), 255, -1)
        cv2.rectangle(mask, (155, 205), (165, 225), 255, -1)
        result = self.extract(mask)
        status = self.perception._rear_centerline_diagnostics
        self.assertTrue(status['rear_component_fallback'])
        self.assertTrue(status['rear_component_target_supported'])
        self.assertLessEqual(result.waypoints_uv[:, 1].max(), 180.)
        self.assertTrue(all(mask[v, u] for u, v in np.rint(result.waypoints_uv).astype(int)))
        self.assertEqual(sum(x['accepted'] for x in status['rear_component_candidates']), 1)
        json.dumps(status, allow_nan=False)

    def test_usable_original_anchor_keeps_its_original_extraction(self):
        mask = np.zeros((320, 320), np.uint8)
        cv2.rectangle(mask, (155, 60), (165, 230), 255, -1)
        with patch.object(self.perception, '_original_extract',
                          wraps=self.perception._original_extract) as extract:
            self.extract(mask)
        self.assertEqual(extract.call_count, 1)
        self.assertFalse(self.perception._rear_centerline_diagnostics['rear_component_fallback'])

    def test_two_short_components_are_not_joined_into_long_support(self):
        self.context['depth_m'][:] = .3
        self.context['plane'] = np.array([0., 0., 1., -.2])
        mask = np.zeros((320, 320), np.uint8)
        cv2.rectangle(mask, (155, 90), (165, 145), 255, -1)
        cv2.rectangle(mask, (155, 180), (165, 235), 255, -1)
        self.extract(mask)
        status = self.perception._rear_centerline_diagnostics
        self.assertFalse(status['rear_component_target_supported'])
        self.assertFalse(any(x['accepted'] for x in status['rear_component_candidates']))

    def test_missing_depth_cannot_be_replaced_by_the_measured_plane(self):
        mask = np.zeros((320, 320), np.uint8)
        cv2.rectangle(mask, (155, 60), (165, 230), 255, -1)
        self.context['depth_m'][:] = np.nan
        self.extract(mask)
        status = self.perception._rear_centerline_diagnostics
        self.assertFalse(status['rear_component_target_supported'])
        self.assertIn('depth support too low', status['rear_component_candidates'][0]['reason'])

    def test_work_point_beyond_original_near_field_bound_is_not_accepted(self):
        mask = np.zeros((320, 320), np.uint8)
        cv2.rectangle(mask, (155, 230), (165, 310), 255, -1)
        self.extract(mask)
        status = self.perception._rear_centerline_diagnostics
        self.assertFalse(status['rear_component_target_supported'])
        self.assertIn('0.25 m', status['rear_component_candidates'][0]['reason'])

    def test_front_without_rear_context_delegates_unchanged(self):
        self.perception._rear_geometry_context = None
        sentinel = object()
        with patch.object(SeamOnlyDualPerception, '_extract_with_roi', return_value=sentinel) as old:
            self.assertIs(self.extract(np.zeros((320, 320), np.uint8)), sentinel)
        old.assert_called_once()

    def test_invalid_work_point_configuration_rejected(self):
        for kwargs in ({'trailing_distance': 0.}, {'max_occlusion_extension': .051},
                       {'trailing_distance': float('nan')}):
            with self.assertRaises(ValueError):
                VD12RearSupportPerception(PerceptionConfig(mode='classical'), **kwargs)


# Original RHANet_dep threshold=.90, input=(360,640), detector SHA256
# f1eea726c85773c3ddb71649bdae097eff6ca7693c7f36ad488fa9f4643bdb80.
# The compressed uint8 mask is an archived neural output, NOT a fresh inference.
# Probability=mask/255 in this fixture tests geometry only; actual mean neural
# confidence (.99732981298) was checked separately by offline model inference.
SEQ94_MASK = (
    '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'
)


class RecordedFrameTests(unittest.TestCase):
    def test_archived_seq94_selects_the_other_real_neural_component(self):
        directory = ROOT/'outputs/vd12_vision/vision_20261115_145455_62a3axx4/rear'
        if not (directory/'frame_slot_6.json').is_file():
            self.skipTest('local archived seq94 RGB-D is not present')
        metadata = json.loads((directory/'frame_slot_6.json').read_text())
        self.assertEqual(metadata['sequence'], 94)
        rgb_path = directory/metadata['rgb_file']
        self.assertEqual(hashlib.sha256(rgb_path.read_bytes()).hexdigest(),
                         '15e20f446e527d4916618b03eb28f559633606f2fa4db9993f1bf522ef3eae12')
        rgb = cv2.imread(str(rgb_path))
        depth = cv2.imread(str(directory/metadata['depth_file']), cv2.IMREAD_UNCHANGED)
        mask = np.frombuffer(zlib.decompress(base64.b64decode(SEQ94_MASK)), np.uint8).reshape(480, 640)
        class ArchivedMask:
            def predict_bgr(self, image):
                return SimpleNamespace(mask=mask, probability=mask.astype(float)/255., inference_ms=0.)
        kwargs = dict(config=PerceptionConfig(mode='rhanet'), segmenter=ArchivedMask(),
                      gun_mask=NearClippedEncoderGunMask(
                          '/home/dong/b2_z1_VD2/source/b2_z1/b2_z1/assets/b2_z1/b2_z1_vd10_hollow_gun.urdf',
                          near_m=.07000000029802322), rear_centerline_mode='cross_sections')
        old = SeamOnlyDualPerception(**kwargs).process_rear(metadata, rgb, depth)
        self.assertFalse(old['valid'])
        self.assertAlmostEqual(old['rear_candidate_path_length_m'], .1084309399463503)
        perception = VD12RearSupportPerception(**kwargs)
        result = perception.process_rear(metadata, rgb, depth)
        self.assertTrue(result['valid'], result['reason'])
        self.assertTrue(result['rear_component_fallback'])
        self.assertEqual(result['rear_component_selected_id'], 1)
        self.assertEqual(len(result['points_b']), 23)
        self.assertAlmostEqual(result['path_length_m'], .22560702354665704)
        target, diagnostics = rear_visual_target(result['points_b'], result['ground_plane_b'], .8, .05)
        np.testing.assert_allclose(target, [-.7845543909, -.1681075495, -.5073861187], atol=1.e-9)
        self.assertAlmostEqual(diagnostics['rear_target_near_field_extension_m'], .10245852106536854)
        self.assertFalse(diagnostics['rear_target_directly_observed'])
        self.assertEqual(diagnostics['rear_target_source'], 'rear_rgbd_near_field_tangent_estimate')
        self.assertIsNone(perception._rear_geometry_context)
        json.dumps(result, allow_nan=False)


if __name__ == '__main__':
    unittest.main()
