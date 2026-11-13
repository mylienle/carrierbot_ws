"""Regression for scan arrival preceding its exact-time TF callback."""
from collections import deque
import sys
from threading import RLock
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from fusion_sensor_adapter import FusionSensors, project_scan, select_transformed_scan, SensorTimeWindow
from fusion_yield_policy import FusionYieldPolicy
from test_fusion_yield import intent


def scan(t):
    sec = int(t)
    return SimpleNamespace(header=SimpleNamespace(frame_id="laser",
        stamp=SimpleNamespace(sec=sec, nanosec=round((t-sec)*1e9))))


class ScanTFTests(unittest.TestCase):
    def test_actual_guest_future_tf_samples_select_an_available_fresh_scan(self):
        cases = [(23.531, 23.528, 23.499), (23.643, 23.627, 23.604),
                 (23.714, 23.727, 23.679), (23.751, 23.727, 23.714)]
        for now, newest, latest_tf in cases:
            with self.subTest(now=now):
                previous = newest-.1
                history = [(100.-.1/.35, scan(previous)), (100., scan(newest))]
                def lookup(target, child, requested):
                    if requested > latest_tf:
                        raise ValueError("Lookup would require extrapolation into the future")
                    return [0., 0.], [0., 0., 0., 1.], requested
                selected, p, q, r, received, skipped = select_transformed_scan(
                    history, now, 100., lookup, (ValueError,))
                t = selected.header.stamp.sec+selected.header.stamp.nanosec/1e9
                self.assertAlmostEqual(t, previous)
                self.assertTrue(-.02 <= now-t <= .3)
                self.assertLessEqual(100.-received, .6)
                self.assertEqual(skipped[0]["scan_stamp"], newest)

    def test_newest_scan_waits_for_tf_without_discarding_a_fresh_previous_scan(self):
        calls = []
        def lookup(target, child, t):
            calls.append((target, child, t))
            if t > 9.95:
                raise ValueError("future extrapolation")
            return [0., 0.], [0., 0., 0., 1.], t
        history = [(99.7, scan(9.9)), (100., scan(10.))]
        selected, p, q, r, received, skipped = select_transformed_scan(history, 10., 100., lookup, (ValueError,))
        self.assertEqual(selected.header.stamp.nanosec, 900000000)
        self.assertEqual(received, 99.7)
        self.assertEqual(skipped[0]["scan_stamp"], 10.)
        self.assertEqual(calls[-2:], [("odom", "laser", 9.9), ("odom", "base_footprint", 9.9)])
        self.assertTrue(all(t != 0. for target, child, t in calls))

    def test_no_latest_tf_substitution_if_all_exact_time_queries_fail(self):
        def lookup(*args):
            raise ValueError("TF not ready")
        with self.assertRaisesRegex(ValueError, "NO_FRESH_TIME_MATCHED_SCAN"):
            select_transformed_scan([(100., scan(10.))], 10., 100., lookup, (ValueError,))

    def test_fallback_never_relaxes_either_freshness_limit(self):
        def lookup(*args):
            self.fail("Stale scan must be rejected before a transform query")
        for history in ([(100., scan(9.6))], [(99.3, scan(9.9))], [(100., scan(10.1))]):
            with self.subTest(history=history), self.assertRaises(ValueError):
                select_transformed_scan(history, 10., 100., lookup, (ValueError,))

    def test_projection_reuse_keeps_current_route_tf_and_freshness_checks(self):
        sensor = FusionSensors.__new__(FusionSensors)
        sample = scan(10.)
        sample.ranges = [1.] * 30
        sample.range_min, sample.range_max = .1, 10.
        sample.angle_min = sample.angle_increment = 0.
        sensor.input_lock = RLock()
        sensor.scans = deque([(100., sample)])
        sensor.scan_wall = 100.
        sensor.projected_scan = sensor.huan = None
        sensor.yield_clearance_m = .6
        robot = {"x": 0., "y": 0.}
        pose = [0., 0.]
        sensor.now = lambda: 10.
        sensor.transform = lambda *args: (pose[:], [0., 0., 0., 1.], 10.)
        tf_module = SimpleNamespace(TransformException=RuntimeError)
        forward = {"route_corridor_xy_m": [[0., 0.], [3., 0.]]}
        left = {"route_corridor_xy_m": [[0., 0.], [0., 3.]]}
        with patch.dict(sys.modules, {"tf2_ros": tf_module}), \
             patch("fusion_sensor_adapter.time.monotonic", return_value=100.1), \
             patch("fusion_sensor_adapter.project_scan", wraps=project_scan) as project:
            self.assertTrue(sensor.raw_corridor(forward, robot)["blocked"])
            self.assertFalse(sensor.raw_corridor(left, robot)["blocked"])
            self.assertEqual(project.call_count, 1)
            pose[0] = .5
            self.assertFalse(sensor.raw_corridor(forward, robot)["blocked"])
            self.assertEqual(project.call_count, 2)
            sensor.now = lambda: 10.31
            with self.assertRaisesRegex(ValueError, "NO_FRESH_TIME_MATCHED_SCAN"):
                sensor.raw_corridor(forward, robot)
            self.assertEqual(project.call_count, 2)

    def test_full_pipeline_can_confirm_clear_with_regular_scan_tf_arrival_skew(self):
        policy = FusionYieldPolicy()
        history = deque(maxlen=120)
        actions = []
        for index in range(9):
            t = 10.+index*.1
            wall = 100.+index*.1/.35
            if not history:
                history.append((wall-.1/.35, scan(t-.1)))
            history.append((wall, scan(t)))
            def lookup(target, child, stamp):
                if stamp > t-.03:
                    raise ValueError("TF arrives after the newest scan")
                return [0., 0.], [0., 0., 0., 1.], stamp
            selected, p, q, r, received, skipped = select_transformed_scan(history, t, wall, lookup, (ValueError,))
            header = selected.header.stamp
            raw = {"valid": True, "blocked": False, "stamp": header.sec+header.nanosec/1e9}
            actions.append(policy.evaluate(t, intent(t), raw, [])["action"])
        self.assertEqual(actions[0], "WAIT")
        self.assertEqual(actions[-1], "BASELINE")

    def test_valid_previous_scan_still_blocks_a_person(self):
        policy = FusionYieldPolicy()
        plan = intent(10.)
        result = policy.evaluate(10., plan, {"valid": True, "blocked": True, "stamp": 9.9}, [])
        self.assertEqual(result["reason"], "LIDAR_CORRIDOR_BLOCKED")

    def test_preflight_needs_new_scans_and_resets_after_a_gap(self):
        window = SensorTimeWindow()
        self.assertFalse(window.observe(10., 9.9))
        self.assertFalse(window.observe(10.5, 9.9))
        self.assertFalse(window.observe(10.6, 10.5))
        self.assertTrue(window.observe(10.7, 10.6))
        window.reset()
        self.assertFalse(window.observe(10.8, 10.7))
        self.assertFalse(window.observe(11., 10.9))

    def test_a_frozen_scan_cannot_finish_clear_confirmation(self):
        policy = FusionYieldPolicy()
        first = policy.evaluate(10., intent(10.), {"valid": True, "blocked": False, "stamp": 10.}, [])
        last = policy.evaluate(10.6, intent(10.6), {"valid": True, "blocked": False, "stamp": 10.}, [])
        self.assertEqual(first["action"], "WAIT")
        self.assertEqual(last["reason"], "LIDAR_UNAVAILABLE")


if __name__ == "__main__":
    unittest.main()
