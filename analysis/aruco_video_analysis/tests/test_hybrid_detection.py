import importlib.util
import sys
import unittest
from pathlib import Path

import cv2
import numpy as np


SCRIPT = Path(__file__).parents[1] / "east_aruco_video_analysis.py"
SPEC = importlib.util.spec_from_file_location("east_aruco_video_analysis", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class HybridDetectionTests(unittest.TestCase):
    def test_small_antialiased_dot_is_selected(self):
        frame = np.full((240, 240, 3), 245, dtype=np.uint8)
        cv2.circle(frame, (120, 120), 42, (0, 255, 255), -1)
        cv2.circle(frame, (124, 117), 3, (115, 115, 115), -1)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        result = MODULE.detect_target(
            frame, hsv, MODULE.DEFAULT_YELLOW, 120, "hybrid", 200, 2, 0.12, 0.55
        )
        self.assertEqual(result.selected_source, "dot")
        self.assertAlmostEqual(result.dot_centroid[0], 124, delta=0.5)
        self.assertAlmostEqual(result.dot_centroid[1], 117, delta=0.5)

    def test_hybrid_records_colour_fallback_when_dot_is_absent(self):
        frame = np.full((240, 240, 3), 245, dtype=np.uint8)
        cv2.circle(frame, (120, 120), 42, (255, 0, 255), -1)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        result = MODULE.detect_target(
            frame, hsv, MODULE.DEFAULT_MAGENTA, 120, "hybrid", 200, 2, 0.12, 0.55
        )
        self.assertEqual(result.selected_source, "colour_fallback")
        self.assertIsNone(result.dot_centroid)

    def test_three_marker_affine_uses_measured_reference_centres(self):
        image_centres = {0: (100, 100), 1: (300, 100), 3: (100, 400)}
        reference = {0: (0, 0), 1: (461, 0), 2: (461, 527), 3: (0, 527)}
        corners = []
        ids = []
        for marker_id, (x, y) in image_centres.items():
            corners.append(np.asarray([[(x-5, y-5), (x+5, y-5), (x+5, y+5), (x-5, y+5)]], dtype=np.float32))
            ids.append([marker_id])
        transform, seen, method = MODULE.make_homography(
            corners, np.asarray(ids, dtype=np.int32), reference, minimum_markers=3
        )
        self.assertEqual(seen, [0, 1, 3])
        self.assertEqual(method, "three_marker_affine")
        for marker_id, centre in image_centres.items():
            actual = MODULE.transform_point(centre, transform)
            np.testing.assert_allclose(actual, reference[marker_id], atol=1e-3)


if __name__ == "__main__":
    unittest.main()
