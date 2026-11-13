"""Synthetic decision cases, separate from live Gazebo performance evidence."""
from copy import deepcopy
import unittest

from dynamic_decision_policy import DecisionPolicy, straight_candidate


def sample(position=(3, 0), velocity=None, stamp=10.0, now=None, speed=.12):
    if now is None:
        now = stamp
    track = {"id": 1, "timestamp": stamp, "reference_valid": True,
             "odom_frame_id": "odom", "odom_source": "tf2_ekf",
             "reference_query_stamp": stamp, "reference_transform_stamp": stamp,
             "confidence": .9, "pose_status": "OK", "depth_m": 3.0,
             "xyz_odom_m": [position[0], position[1], 1.1],
             "velocity_odom_valid": velocity is not None,
             "velocity_odom_mps": [velocity[0], velocity[1], 0.0] if velocity is not None else None}
    packet = {"timestamp": stamp, "tracks": [track]}
    match = {"matched": True, "center_odom_m": list(position), "radius_m": .25}
    robot = {"x": 0., "y": 0., "yaw": 0., "stamp": now}
    return dict(packet=packet, track=track, match=match, robot=robot,
                plan=straight_candidate(robot, now, speed=speed), now=now,
                position_ok=True, velocity_ok=velocity is not None)


class DecisionTests(unittest.TestCase):
    def test_unknown_velocity_is_not_stationary(self):
        result = DecisionPolicy().evaluate(**sample())
        self.assertEqual((result["proposal"], result["motion_status"], result["risk"]),
                         ("SLOW", "UNKNOWN", "UNKNOWN"))

    def test_unknown_motion_outside_corridor_is_not_a_clear_prediction(self):
        result = DecisionPolicy().evaluate(**sample(position=(3, 2)))
        self.assertEqual(result["proposal"], "FOLLOW_LIDAR_ONLY")
        self.assertEqual(result["risk"], "UNKNOWN")

    def test_crossing_at_same_time_requests_wait(self):
        result = DecisionPolicy().evaluate(**sample(position=(2, 1), velocity=(0, -.5), speed=1))
        self.assertEqual(result["proposal"], "WAIT")
        self.assertEqual(result["reason"], "PREDICTED_SPATIOTEMPORAL_CONFLICT")

    def test_intersecting_paths_at_different_times_do_not_force_wait(self):
        result = DecisionPolicy().evaluate(**sample(position=(2, 2), velocity=(0, -1)))
        self.assertEqual(result["proposal"], "FOLLOW_CANDIDATE")
        self.assertEqual(result["risk"], "CLEAR_WITHIN_MODEL")

    def test_object_moving_away_can_follow_candidate(self):
        result = DecisionPolicy().evaluate(**sample(velocity=(1, 0)))
        self.assertEqual(result["proposal"], "FOLLOW_CANDIDATE")

    def test_single_zero_speed_sample_does_not_confirm_stationary(self):
        result = DecisionPolicy().evaluate(**sample(position=(2, 0), velocity=(0, 0)))
        self.assertEqual(result["motion_status"], "LOW_SPEED_UNCONFIRMED")
        self.assertEqual(result["proposal"], "SLOW")

    def test_persistent_stationary_blocker_requests_a_detour_check(self):
        policy = DecisionPolicy()
        for t in (10., 10.5, 11.):
            result = policy.evaluate(**sample(position=(2, 0), velocity=(0, 0), stamp=t))
        self.assertEqual(result["proposal"], "CHECK_DETOUR")

    def test_missing_velocity_resets_stationary_confirmation(self):
        policy = DecisionPolicy()
        for t in (10., 10.5):
            policy.evaluate(**sample(position=(2, 0), velocity=(0, 0), stamp=t))
        policy.evaluate(**sample(position=(2, 0), velocity=None, stamp=11.))
        result = policy.evaluate(**sample(position=(2, 0), velocity=(0, 0), stamp=11.5))
        self.assertEqual(result["proposal"], "SLOW")

    def test_small_current_clearance_requests_wait_even_if_moving_away(self):
        result = DecisionPolicy().evaluate(**sample(position=(1, 0), velocity=(1, 0)))
        self.assertEqual(result["reason"], "CURRENT_CLEARANCE_SMALL")

    def test_missing_lidar_association_does_not_allow_follow(self):
        values = sample(velocity=(1, 0))
        values["match"] = None
        self.assertEqual(DecisionPolicy().evaluate(**values)["proposal"], "WAIT")

    def test_stale_vision_never_yields_a_clear_prediction(self):
        values = sample(velocity=(1, 0), stamp=10, now=11)
        result = DecisionPolicy().evaluate(**values)
        self.assertEqual(result["proposal"], "SLOW")
        self.assertEqual(result["risk"], "UNKNOWN")

    def test_stale_robot_tf_requests_wait(self):
        values = sample()
        values["robot"]["stamp"] = 9.
        self.assertEqual(DecisionPolicy().evaluate(**values)["reason"], "CURRENT_ROBOT_TF_UNAVAILABLE")

    def test_bad_track_id_is_handled_without_exception(self):
        values = sample()
        values["track"]["id"] = []
        values["position_ok"] = False
        self.assertEqual(DecisionPolicy().evaluate(**values)["proposal"], "WAIT")

    def test_risk_screening_does_not_rewrite_huan_packet(self):
        values = sample(velocity=(1, 0))
        original = deepcopy(values["packet"])
        result = DecisionPolicy().evaluate(**values)
        self.assertEqual(values["packet"], original)
        self.assertFalse(result["actuation"])
        self.assertEqual(result["robot_plan"]["kind"], "hypothetical_straight_not_executed")


if __name__ == "__main__":
    unittest.main()
