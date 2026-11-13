"""Regressions for a 0.6m body-to-scan clearance, independent of truth data."""
import math
import unittest

from fusion_yield_policy import raw_route_corridor
from huan_fusion_policy import IntegratedFusionPolicy
from test_fusion_yield import intent, raw
from test_huan_integration import camera


class YieldClearanceTests(unittest.TestCase):
    def measured(self, points, clearance=.6):
        data = raw_route_corridor(points, [0., 0.], [[0., 0.], [3., 0.]], 60, clearance)
        data["stamp"] = 11.
        return data

    def test_far_obstacle_no_longer_requests_wait(self):
        points = [[2.5, -.01], [2.5, 0.], [2.5, .01]]
        self.assertTrue(self.measured(points, None)["blocked"])
        self.assertFalse(self.measured(points)["blocked"])
        self.assertEqual(self.measured(points)["forward_corridor_returns"], 3)

    def test_clearance_is_from_body_not_robot_centre(self):
        result = self.measured([[1.10, -.01], [1.10, 0.], [1.10, .01]])
        self.assertTrue(result["blocked"])
        self.assertAlmostEqual(result["nearest_forward_corridor_clearance_m"], .58)
        self.assertAlmostEqual(result["centre_to_point_stop_radius_m"], 1.12)

    def test_exact_boundary_is_included(self):
        self.assertTrue(self.measured([[1.12, 0.]]*3)["blocked"])
        self.assertFalse(self.measured([[1.12001, 0.]]*3)["blocked"])

    def test_euclidean_surface_distance_handles_oblique_returns(self):
        distance = 1.119
        p = [distance*math.cos(.5), distance*math.sin(.5)]
        self.assertTrue(self.measured([p]*3)["blocked"])
        self.assertAlmostEqual(self.measured([p]*3)["nearest_forward_corridor_clearance_m"], .599)

    def test_three_returns_are_still_required(self):
        self.assertFalse(self.measured([[1., 0.]]*2)["blocked"])
        self.assertTrue(self.measured([[1., 0.]]*3)["blocked"])

    def test_behind_robot_and_outside_route_band_remain_ignored(self):
        self.assertFalse(self.measured([[-.8, 0.]]*3+[[.1, .83]]*3)["blocked"])

    def test_invalid_or_too_small_clearance_fails_closed(self):
        for c in (float("nan"), float("inf"), -.6, .06, .59, 3.1):
            result = self.measured([[1., 0.]]*3, c)
            self.assertFalse(result["valid"])
            policy = IntegratedFusionPolicy("lidar-only")
            self.assertEqual(policy.evaluate(11., intent(11.), result, [])["action"], "WAIT")

    def test_near_intrusion_stops_even_without_valid_camera(self):
        result = IntegratedFusionPolicy().evaluate(11., intent(11.), self.measured([[1., 0.]]*3), [], None)
        self.assertEqual(result["reason"], "LIDAR_CORRIDOR_BLOCKED")
        self.assertEqual(result["speed_cap_mps"], 0.)

    def test_camera_prediction_is_not_disabled_by_a_smaller_lidar_zone(self):
        policy = IntegratedFusionPolicy()
        for t in (10., 10.3, 10.6):
            policy.evaluate(t, intent(t), raw(t), [])
        # The person is currently far from the body but approaching the route.
        packet = camera(11., position=(2., 0., .9), velocity=(-1., 0., 0.))
        scan = self.measured([[1.75, 0.]]*3)
        self.assertFalse(scan["blocked"])
        result = policy.evaluate(11., intent(11.), scan, [], dict(packet=packet))
        self.assertEqual(result["reason"], "HUAN_FUSION_PREDICTED_CONFLICT")
        self.assertEqual(result["action"], "WAIT")


if __name__ == "__main__":
    unittest.main()
