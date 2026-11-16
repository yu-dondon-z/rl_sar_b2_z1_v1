"""Pixel-for-pixel checks against the unmodified full-image implementation."""
import unittest
from unittest.mock import patch
from pathlib import Path
import sys
import cv2
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'thirdparty/rhacracknet'))
import crack_trajectory.centerline as centerline

def reference_thinning(mask: np.ndarray, max_iterations: int = 256) -> np.ndarray:
    """Vectorized Zhang-Suen thinning, returned as a uint8 0/255 image."""
    image = (mask > 0).astype(np.uint8)
    if not np.any(image):
        return image

    for _ in range(max_iterations):
        changed = False
        for sub_iteration in (0, 1):
            padded = np.pad(image, 1, mode="constant")
            p2 = padded[:-2, 1:-1]
            p3 = padded[:-2, 2:]
            p4 = padded[1:-1, 2:]
            p5 = padded[2:, 2:]
            p6 = padded[2:, 1:-1]
            p7 = padded[2:, :-2]
            p8 = padded[1:-1, :-2]
            p9 = padded[:-2, :-2]
            neighbors = (p2, p3, p4, p5, p6, p7, p8, p9)
            neighbor_count = sum(neighbors)
            transitions = sum(
                ((neighbors[index] == 0) & (neighbors[(index + 1) % 8] == 1)).astype(np.uint8)
                for index in range(8)
            )
            if sub_iteration == 0:
                topology = (p2 * p4 * p6 == 0) & (p4 * p6 * p8 == 0)
            else:
                topology = (p2 * p4 * p8 == 0) & (p2 * p6 * p8 == 0)
            remove = (
                (image == 1)
                & (neighbor_count >= 2)
                & (neighbor_count <= 6)
                & (transitions == 1)
                & topology
            )
            if np.any(remove):
                image[remove] = 0
                changed = True
        if not changed:
            break
    return image.astype(np.uint8) * 255


def reference_centerline(mask, config, anchor):
    cleaned=centerline.clean_binary_mask(mask,config)
    skeleton=reference_thinning(cleaned,config.max_thinning_iterations)
    ordered=centerline._ordered_primary_path(skeleton,anchor)
    if len(ordered)<2:raise RuntimeError('No usable crack centerline was found after mask cleanup')
    smoothed=centerline._smooth_path(ordered,config.smooth_window)
    waypoints,length=centerline._resample_path(smoothed,config.waypoint_spacing_px)
    return centerline.CenterlineResult(cleaned,skeleton,ordered,waypoints,length)


class ThinningTests(unittest.TestCase):
    def masks(self):
        image=np.zeros((128,256),np.uint8)
        yield image.copy()
        cv2.line(image,(40,20),(180,110),255,21);yield image.copy()
        cv2.rectangle(image,(0,0),(60,15),255,-1);yield image.copy()
        cv2.rectangle(image,(210,70),(255,127),255,-1);yield image.copy()
        image[:]=255;yield image.copy()
        rng=np.random.default_rng(421)
        for _ in range(3):
            image=(rng.random((64,100))>.55).astype(np.uint8)*255;yield image
        image=np.zeros((1080,1920),np.uint8)
        cv2.polylines(image,[np.array([[1250,1000],[1190,890],[1120,750]])],False,255,24)
        yield image

    def test_foreground_crop_preserves_every_skeleton_pixel_including_image_borders(self):
        for index,image in enumerate(self.masks()):
            for iterations in (0,1,256):
                with self.subTest(index=index,iterations=iterations):
                    np.testing.assert_array_equal(centerline.thin_zhang_suen(image,iterations),
                                                  reference_thinning(image,iterations))

    def test_full_centerline_coordinates_masks_and_waypoints_are_unchanged(self):
        image=np.zeros((360,640),np.uint8)
        cv2.polylines(image,[np.array([[410,340],[400,240],[380,140],[350,50]])],False,255,23)
        config=centerline.CenterlineConfig(waypoint_spacing_px=6.)
        actual=centerline.extract_centerline(image,config,(320.,350.))
        with patch.object(centerline,'thin_zhang_suen',reference_thinning):
            expected=centerline.extract_centerline(image,config,(320.,350.))
        for field in ('cleaned_mask','skeleton','ordered_path_uv','waypoints_uv'):
            np.testing.assert_array_equal(getattr(actual,field),getattr(expected,field))
        self.assertEqual(actual.path_length_px,expected.path_length_px)

    def test_crop_preserves_cleanup_topology_anchor_choice_and_float32_waypoints(self):
        rng=np.random.default_rng(817)
        masks=list(self.masks())[:8]
        for _ in range(12):
            image=np.zeros((103,197),np.uint8)
            for _ in range(3):
                pts=rng.integers([0,0],[197,103],size=(3,2)).astype(np.int32)
                cv2.polylines(image,[pts],False,255,int(rng.integers(2,9)))
            masks.append(image)
        for index,image in enumerate(masks):
            for kernel in (0,2,3,7):
                for anchor in (None,(image.shape[1]/2+.125,image.shape[0]-.25),(-15.5,300.75)):
                    config=centerline.CenterlineConfig(min_component_area=4,morphology_kernel=kernel,
                        waypoint_spacing_px=6.)
                    with self.subTest(index=index,kernel=kernel,anchor=anchor):
                        try:expected=reference_centerline(image,config,anchor)
                        except RuntimeError:
                            with self.assertRaises(RuntimeError):centerline.extract_centerline(image,config,anchor)
                            continue
                        actual=centerline.extract_centerline(image,config,anchor)
                        for field in ('cleaned_mask','skeleton','ordered_path_uv','waypoints_uv'):
                            np.testing.assert_array_equal(getattr(actual,field),getattr(expected,field))
                        self.assertEqual(actual.path_length_px,expected.path_length_px)


if __name__=='__main__':unittest.main()
