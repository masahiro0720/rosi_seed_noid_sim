#!/usr/bin/env python3

import math
from pathlib import Path
import sys
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from person_localization_depth import cloud_uvs, median_xyz, sample_pixels
from rosi_seed_noid_sim.msg import PersonRoi


class DepthGeometryTests(unittest.TestCase):
    def test_gazebo_flat_cloud_maps_rgb_pixels_to_linear_indices(self):
        pixels = [(320, 240), (0, 0), (639, 479)]
        self.assertEqual(
            cloud_uvs(pixels, 640, 480, 307200, 1),
            [(153920, 0), (0, 0), (307199, 0)],
        )

    def test_organized_cloud_keeps_pixel_coordinates(self):
        pixels = [(12, 34)]
        self.assertEqual(cloud_uvs(pixels, 640, 480, 640, 480), pixels)

    def test_mismatched_cloud_layout_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "does not match"):
            cloud_uvs([(0, 0)], 640, 480, 10, 10)

    def test_median_rejects_nan_and_depth_outliers(self):
        result = median_xyz(
            [
                (0.0, 0.0, math.nan),
                (100.0, 100.0, 100.0),
                (-0.1, 0.2, 2.8),
                (0.0, 0.3, 2.9),
                (0.1, 0.4, 3.0),
            ]
        )
        self.assertEqual(result, (0.0, 0.3, 2.9))

    def test_median_accepts_configured_far_person_depth(self):
        result = median_xyz(
            [(-2.0, 0.1, 6.2), (-1.9, 0.2, 6.3), (-1.8, 0.3, 6.4)],
            maximum_depth=10.0,
        )
        self.assertEqual(result, (-1.9, 0.2, 6.3))

    def test_roi_sampling_stays_inside_image(self):
        roi = PersonRoi(
            image_width=640,
            image_height=480,
            x_offset=600,
            y_offset=440,
            width=100,
            height=100,
        )
        pixels = sample_pixels(roi, 7)
        self.assertEqual(len(pixels), 49)
        self.assertTrue(all(0 <= u < 640 and 0 <= v < 480 for u, v in pixels))


if __name__ == "__main__":
    unittest.main()
