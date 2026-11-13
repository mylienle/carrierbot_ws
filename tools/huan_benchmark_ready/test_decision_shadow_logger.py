"""Exercise the read-only adapter using TF stubs, not a live ROS experiment."""
import io
import json
import sys
import types
import unittest
from unittest.mock import patch

from decision_shadow_logger import DecisionShadowLogger
from test_dynamic_decision_policy import sample
from test_fablab_route import fixture
from fablab_route import route_candidate


class FakeTime:
    def __init__(self, nanoseconds=0):
        self.nanoseconds = nanoseconds


class TransformError(Exception):
    pass


def transform(stamp):
    return types.SimpleNamespace(
        header=types.SimpleNamespace(stamp=types.SimpleNamespace(sec=int(stamp), nanosec=round((stamp-int(stamp))*1e9))),
        transform=types.SimpleNamespace(
            translation=types.SimpleNamespace(x=0., y=0., z=0.),
            rotation=types.SimpleNamespace(x=0., y=0., z=0., w=1.)))


class LoggerTests(unittest.TestCase):
    def setUp(self):
        parent = types.ModuleType("rclpy")
        parent.__path__ = []
        clock = types.ModuleType("rclpy.time")
        clock.Time = FakeTime
        tf = types.ModuleType("tf2_ros")
        tf.TransformException = TransformError
        self.modules = {"rclpy": parent, "rclpy.time": clock, "tf2_ros": tf}

    def observe(self, buffer):
        values = sample()
        stream = io.StringIO()
        logger = DecisionShadowLogger(buffer, stream)
        with patch.dict(sys.modules, self.modules):
            logger.observe(values["packet"], values["track"], values["match"], values["now"], True, False)
        return json.loads(stream.getvalue()), logger.finish()

    def test_unknown_motion_is_recorded_without_actuation(self):
        buffer = types.SimpleNamespace(lookup_transform=lambda *args: transform(10.))
        row, summary = self.observe(buffer)
        self.assertEqual(row["proposal"], "SLOW")
        self.assertFalse(row["actuation"])
        self.assertFalse(summary["recording_complete"])
        self.assertFalse(summary["closed_loop_avoidance_evaluated"])

    def test_future_latest_tf_is_replaced_by_query_at_now(self):
        calls = []
        def lookup(target, child, stamp):
            calls.append(stamp.nanoseconds)
            return transform(10.1 if len(calls) == 1 else 10.)
        row, summary = self.observe(types.SimpleNamespace(lookup_transform=lookup))
        self.assertEqual(calls, [0, 10000000000])
        self.assertEqual(row["robot_pose_odom"]["stamp"], 10.)

    def test_missing_tf_records_wait_instead_of_inventing_pose(self):
        def lookup(*args):
            raise TransformError("No robot TF")
        row, summary = self.observe(types.SimpleNamespace(lookup_transform=lookup))
        self.assertEqual(row["proposal"], "WAIT")
        self.assertIsNone(row["robot_pose_odom"])
        self.assertIn("tf_error", row)

    def test_missing_internal_route_never_falls_back_to_the_old_straight_candidate(self):
        values = sample()
        stream = io.StringIO()
        reader = types.SimpleNamespace(candidate=lambda robot, now: (None, "NO_ROUTE_STREAM"))
        buffer = types.SimpleNamespace(lookup_transform=lambda *args: transform(10.))
        logger = DecisionShadowLogger(buffer, stream, route_reader=reader)
        with patch.dict(sys.modules, self.modules):
            logger.observe(values["packet"], values["track"], values["match"], 10., True, False)
        row = json.loads(stream.getvalue())
        self.assertEqual(row["proposal"], "WAIT")
        self.assertEqual(row["reason"], "ROBOT_PLAN_UNAVAILABLE")
        self.assertIsNone(row["robot_plan"])
        self.assertEqual(row["route_error"], "NO_ROUTE_STREAM")
        self.assertEqual(logger.finish()["valid_route_records"], 0)
        self.assertFalse(logger.finish()["route_timing_accuracy_evaluated"])

    def test_internal_route_and_its_estimated_timing_are_recorded_with_provenance(self):
        snapshot, robot, convert = fixture()
        plan = route_candidate(snapshot, robot, 10., convert)
        values = sample(position=(1., 1.1))
        reader = types.SimpleNamespace(candidate=lambda pose, now: (plan, None))
        buffer = types.SimpleNamespace(lookup_transform=lambda *args: transform(10.))
        stream = io.StringIO()
        logger = DecisionShadowLogger(buffer, stream, route_reader=reader)
        with patch.dict(sys.modules, self.modules):
            logger.observe(values["packet"], values["track"], values["match"], 10., True, False)
        row = json.loads(stream.getvalue())
        self.assertEqual(row["robot_plan"]["route_snapshot"]["sequence"], 7)
        self.assertFalse(row["robot_plan"]["timing_validated"])
        self.assertEqual(row["proposal"], "SLOW")
        self.assertEqual(logger.finish()["valid_route_records"], 1)


if __name__ == "__main__":
    unittest.main()
