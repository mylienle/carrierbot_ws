"""Route geometry, exact-time transforms and zero-actuation mission setup."""
from copy import deepcopy
import math
import sys
import types
import unittest
from unittest.mock import patch

from fablab_route import SOURCE, TEST_WAYPOINTS, ensure_preview_mission, nominal_positions, route_candidate
from dynamic_decision_policy import DecisionPolicy
from test_dynamic_decision_policy import sample


def fixture():
    snapshot = {"schema_version": 1, "source": SOURCE, "frame_id": "map", "stamp": 10.,
                "sequence": 7, "active": True, "finished": False, "pose_tf_valid": True,
                "pose_tf_stamp": 10., "robot_pose_map": [.9, 0., 0.], "current_segment": 0,
                "waypoints_map": [[0., 0.], [1., 0.], [1., 1.], [5., 1.]],
                "linear_speed_mps": .12, "linear_speed_max_mps": .12,
                "aligning": False, "safety_stop": False, "avoidance_stop": False}
    robot = {"x": .9, "y": 0., "yaw": 0., "stamp": 10.}
    transform = {"x": 0., "y": 0., "yaw": 0., "stamp": 10.}
    return snapshot, robot, transform


class RouteTests(unittest.TestCase):
    def test_nominal_candidate_follows_corner_instead_of_extending_heading(self):
        snapshot, robot, transform = fixture()
        original = deepcopy(snapshot)
        plan = route_candidate(snapshot, robot, 10., transform)
        self.assertEqual(len(plan["positions"]), 3)
        self.assertAlmostEqual(plan["positions"][-1]["xy_m"][0], 1.)
        self.assertAlmostEqual(plan["positions"][-1]["xy_m"][1], .14)
        self.assertEqual(plan["route_geometry_odom_m"][-1], [5., 1.])
        self.assertFalse(plan["timing_validated"])
        self.assertEqual(snapshot, original)

    def test_map_route_is_rotated_and_translated_into_odom_at_source_stamp(self):
        snapshot, robot, transform = fixture()
        transform.update(x=10., y=-2., yaw=math.pi/2)
        robot.update(x=10., y=-1.1, yaw=math.pi/2)
        plan = route_candidate(snapshot, robot, 10., transform)
        self.assertAlmostEqual(plan["positions"][-1]["xy_m"][0], 9.86)
        self.assertAlmostEqual(plan["positions"][-1]["xy_m"][1], -1.)

    def test_completed_segments_are_not_reintroduced(self):
        snapshot, robot, transform = fixture()
        snapshot.update(current_segment=1, robot_pose_map=[1., .9, math.pi/2])
        robot.update(x=1., y=.9, yaw=math.pi/2)
        plan = route_candidate(snapshot, robot, 10., transform)
        self.assertEqual(plan["route_geometry_odom_m"], [[1., .9], [1., 1.], [5., 1.]])
        self.assertAlmostEqual(plan["positions"][-1]["xy_m"][0], 1.14)

    def test_route_end_is_held_until_the_model_horizon(self):
        points = nominal_positions([[0., 0.], [.05, 0.]], .12)
        self.assertEqual(points[-1], {"dt_s": 2., "xy_m": [.05, 0.]})
        self.assertTrue(all(b["dt_s"] > a["dt_s"] for a, b in zip(points, points[1:])))

    def test_guidance_stop_is_not_treated_as_free_motion(self):
        snapshot, robot, transform = fixture()
        snapshot["avoidance_stop"] = True
        plan = route_candidate(snapshot, robot, 10., transform)
        self.assertEqual(plan["positions"][0]["xy_m"], plan["positions"][-1]["xy_m"])
        values = sample(position=(1, 1))
        values.update(robot=robot, plan=plan)
        self.assertEqual(DecisionPolicy().evaluate(**values)["reason"], "GUIDANCE_STOP_OR_ALIGNMENT")

    def test_unknown_velocity_uses_the_bent_route_corridor(self):
        snapshot, robot, transform = fixture()
        plan = route_candidate(snapshot, robot, 10., transform)
        values = sample(position=(1., 1.1))
        values.update(robot=robot, plan=plan)
        result = DecisionPolicy().evaluate(**values)
        self.assertEqual(result["proposal"], "SLOW")
        self.assertEqual(result["risk"], "UNKNOWN")
        values = sample(position=(2., -.8))
        values.update(robot=robot, plan=plan)
        self.assertEqual(DecisionPolicy().evaluate(**values)["proposal"], "FOLLOW_LIDAR_ONLY")

    def test_stale_future_bad_geometry_and_pose_mismatch_are_rejected(self):
        for change in ({"stamp": 9.4}, {"stamp": 10.01}, {"current_segment": 99},
                       {"robot_pose_map": [5., 0., 0.]}, {"linear_speed_mps": .6},
                       {"waypoints_map": [[0., 0.], [float("nan"), 1.]]}):
            snapshot, robot, transform = fixture()
            snapshot.update(change)
            with self.subTest(change=change), self.assertRaises(ValueError):
                route_candidate(snapshot, robot, 10., transform)

    def test_latest_tf_cannot_substitute_for_the_route_time_tf(self):
        snapshot, robot, transform = fixture()
        transform["stamp"] = 10.1
        with self.assertRaisesRegex(ValueError, "EXACT_ROUTE_TIME"):
            route_candidate(snapshot, robot, 10., transform)


class MissionTests(unittest.TestCase):
    def setUp(self):
        self.reader = types.SimpleNamespace(latest={"sequence": 1, "waypoints_map": [], "finished": False},
                                            message_ready=lambda: True)
        self.sent, self.publisher_topics = [], []
        outer = self
        class Pose:
            def __init__(self):
                self.header = types.SimpleNamespace()
                self.pose = types.SimpleNamespace(orientation=types.SimpleNamespace(), position=types.SimpleNamespace())
        def create_publisher(kind, topic, qos):
            outer.publisher_topics.append(topic)
            def publish(message):
                point = [message.pose.position.x, message.pose.position.y]
                outer.sent.append(point)
                if not outer.reader.latest["waypoints_map"]:
                    outer.reader.latest["waypoints_map"] = [[0., 0.]]
                outer.reader.latest["waypoints_map"].append(point)
                outer.reader.latest["sequence"] += 1
                outer.reader.latest["active"] = True
            return types.SimpleNamespace(publish=publish)
        self.node = types.SimpleNamespace(
            create_publisher=create_publisher, destroy_publisher=lambda publisher: None,
            get_subscriptions_info_by_topic=lambda topic: [types.SimpleNamespace(node_name="fablab_avoidance_preview")],
            get_clock=lambda: types.SimpleNamespace(now=lambda: types.SimpleNamespace(to_msg=lambda: None)))
        rclpy = types.ModuleType("rclpy")
        rclpy.spin_once = lambda *args, **kwargs: None
        geometry = types.ModuleType("geometry_msgs.msg")
        geometry.PoseStamped = Pose
        self.modules = {"rclpy": rclpy, "geometry_msgs.msg": geometry}
        self.safety = lambda: {"ready": True, "blockers": []}

    def test_waypoints_are_sent_in_order_and_retry_does_not_duplicate_the_mission(self):
        with patch.dict(sys.modules, self.modules):
            first = ensure_preview_mission(self.node, self.reader, self.safety)
            second = ensure_preview_mission(self.node, self.reader, self.safety)
        self.assertEqual(self.sent, TEST_WAYPOINTS)
        self.assertEqual(self.publisher_topics, ["/avoidance_preview/waypoints"])
        self.assertEqual(first["waypoints_sent_this_run"], 3)
        self.assertEqual(second["waypoints_sent_this_run"], 0)

    def test_partial_mission_is_not_extended_as_a_new_task(self):
        self.reader.latest["waypoints_map"] = [[0., 0.], [1., 0.]]
        with patch.dict(sys.modules, self.modules), self.assertRaisesRegex(RuntimeError, "partial"):
            ensure_preview_mission(self.node, self.reader, self.safety)
        self.assertEqual(self.sent, [])

    def test_nonzero_wheel_command_aborts_before_sending_a_waypoint(self):
        self.safety = lambda: {"ready": False, "blockers": ["NONZERO_OR_INVALID_WHEEL_COMMAND_SEEN"]}
        with patch.dict(sys.modules, self.modules), self.assertRaisesRegex(RuntimeError, "isolation"):
            ensure_preview_mission(self.node, self.reader, self.safety)
        self.assertEqual(self.publisher_topics, [])


if __name__ == "__main__":
    unittest.main()
