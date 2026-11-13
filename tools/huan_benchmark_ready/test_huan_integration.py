"""Contract and safety regressions for the Huan -> Fablab integration."""
from copy import deepcopy
import ast
import hashlib
import json
import math
from pathlib import Path
from types import ModuleType, SimpleNamespace
import tempfile
import unittest
from unittest.mock import patch
import importlib
import sys

from analyze_huan_run import analyze, outline_gaps, standing_outline_review
from fablab_route import nominal_positions
from fusion_sensor_adapter import match_blocking_arc, project_scan
from fusion_yield_policy import RELEASE_PLAN_LOOKAHEAD_S
from huan_cpu_metadata import MetadataPublisher, configure_runtime_threads
from huan_fusion_bridge import HuanFusionBridge, measured_scan, verify_vendor
from huan_fusion_policy import (ClosePassMonitor, IntegratedFusionPolicy, StationaryBlocker, identify_input_wait,
                                stabilize_standing_detour_release, standing_route_has_clearance)
from huan_fusion_vendor.base_handoff import still_plan
from test_fusion_yield import intent, raw
from fusion_yield_trial import fresh_wheel_speed, prepare_vision, synchronized_goal_ready
from ensure_huan_gate import ensure
from huan_trial_scoring import comparison_score


def camera(t=11., position=(3., 0., .9), velocity=(0., 0., 0.)):
    item = dict(id=1, timestamp=t, confidence=.9, reference_valid=True,
                odom_frame_id="odom", odom_source="tf2_ekf", pose_status="OK", depth_m=3.,
                reference_query_stamp=t, reference_transform_stamp=t, xyz_odom_m=list(position),
                velocity_odom_mps=list(velocity) if velocity is not None else None,
                velocity_odom_valid=velocity is not None)
    return dict(timestamp=t, frame_id="camera_rgb_optical_frame", source="gazebo", tracks=[item],
                performance=dict(publish_wall_monotonic=t))


def laser(t):
    return dict(stamp=t, wall=t, query_stamp=t, transform_stamp=t, sensor_pose=[0., 0., 0.],
                clusters=[dict(xy=[3., 0.], radius=.25, points=20, residual=.001)])


class HuanBoundaryTests(unittest.TestCase):
    def established(self):
        bridge = HuanFusionBridge()
        robot = [0., -1.6, 0.]
        for i in range(11):
            t = 10.+i*.1
            result = bridge.update(camera(t), t, laser(t), robot, still_plan(robot, t), t, t)
        self.assertTrue(result["assessment"]["valid"])
        self.assertTrue(bridge.handoff.cache[1]["confirmed"])
        return bridge

    def test_upstream_integrity_is_checked(self):
        self.assertEqual(len(verify_vendor()["files"]), 9)

    def test_unknown_velocity_is_not_turned_into_a_valid_camera_track(self):
        r = [0., 0., 0.]
        result = HuanFusionBridge().update(camera(11., velocity=None), 11., laser(11.), r,
                                           still_plan(r, 11.), 11., 11.)
        self.assertFalse(result["assessment"]["valid"])
        self.assertIsNone(result["packet"])

    def test_laser_continuation_keeps_camera_timestamp_and_distinct_source(self):
        h = self.established()
        p = dict(source="gazebo", timestamp=11.1, tracks=[], performance=dict(publish_wall_monotonic=11.1))
        r = [1., -1.6, 0.]
        result = h.update(p, 11.1, laser(11.1), r, still_plan(r, 11.1), 11.1, 11.1)
        self.assertEqual(result["sources"], ["laser_scan_tf"])
        track = result["packet"]["tracks"][0]
        self.assertEqual(track["vision_last_stamp"], 11.)
        self.assertEqual(track["lidar_support"]["stamp"], 11.1)
        self.assertTrue(result["assessment"]["valid"])

    def test_laser_loss_denies_continuation(self):
        h = self.established()
        r = [1., -1.6, 0.]
        result = h.update(dict(source="gazebo", timestamp=11.1, tracks=[], performance=dict(publish_wall_monotonic=11.1)), 11.1, None,
                          r, still_plan(r, 11.1), 11.1, 11.1)
        self.assertIsNone(result["packet"])

    def test_wrong_time_transform_denies_laser_support(self):
        h = self.established()
        scan = laser(11.1)
        scan["transform_stamp"] = 11.
        r = [1., -1.6, 0.]
        result = h.update(dict(source="gazebo", timestamp=11.1, tracks=[], performance=dict(publish_wall_monotonic=11.1)), 11.1, scan,
                          r, still_plan(r, 11.1), 11.1, 11.1)
        self.assertIsNone(result["packet"])

    def test_delayed_camera_receipt_never_renews_old_image(self):
        r = [0., 0., 0.]
        result = HuanFusionBridge().update(camera(10.), 11., laser(11.), r, still_plan(r, 11.), 11., 11.)
        self.assertIsNone(result["packet"])

    def test_camera_process_gap_denies_handoff_despite_fresh_laser(self):
        h = self.established()
        r = [1., -1.6, 0.]
        result = h.update(dict(source="gazebo", timestamp=11.1, tracks=[], performance=dict(publish_wall_monotonic=10.)), 10., laser(11.1),
                          r, still_plan(r, 11.1), 11.1, 11.1)
        self.assertIsNone(result["packet"])

    def test_queued_old_publication_cannot_renew_its_age_on_arrival(self):
        p = camera(10.9)
        p["performance"]["publish_wall_monotonic"] = 9.
        r = [0., 0., 0.]
        result = HuanFusionBridge().update(p, 11., laser(11.), r, still_plan(r, 11.), 11., 11.)
        self.assertIsNone(result["packet"])

    def test_wrong_source_cannot_initialize_fusion(self):
        p = camera()
        p["source"] = "real_robot"
        r = [0., 0., 0.]
        result = HuanFusionBridge().update(p, 11., laser(11.), r, still_plan(r, 11.), 11., 11.)
        self.assertIsNone(result["packet"])

    def test_raw_circle_is_measured_without_ground_truth(self):
        angles = [-math.pi+2*math.pi*i/3200 for i in range(3200)]
        ranges = []
        # Measured circle behind the scanner crosses the +/-pi scan seam.
        for angle in angles:
            along = -3.*math.cos(angle)
            disc = along*along-(9.-.25**2)
            ranges.append(along-math.sqrt(disc) if disc >= 0 and along > 0 else float("inf"))
        sample = measured_scan(ranges, -math.pi, 2*math.pi/3200, .05, 10., [0., 0., 0.], 11., 20.)
        self.assertEqual(len(sample["clusters"]), 1)
        self.assertAlmostEqual(sample["clusters"][0]["xy"][0], -3., places=6)
        self.assertAlmostEqual(sample["clusters"][0]["radius"], .25, places=6)
        self.assertEqual(sample["wall"], 20.)
        scan = SimpleNamespace(ranges=ranges, angle_min=-math.pi, angle_increment=2*math.pi/3200,
                               range_min=.05, range_max=10.)
        rotation = [0., 0., 0., 1.]
        hits = project_scan(scan, [0., 0.], rotation, [0., 0.])[0][::9]
        self.assertGreater(len(hits), 3)
        count = sample["clusters"][0]["points"]
        self.assertEqual(match_blocking_arc(scan, [0., 0.], rotation, hits, [-3., 0.], .25, count), hits)
        self.assertIsNone(match_blocking_arc(scan, [0., 0.], rotation, hits+[[-2.75, .003]], [-3., 0.], .25, count))
        self.assertIsNone(match_blocking_arc(scan, [0., 0.], rotation, hits, [-3., 0.], .25, count-1))


class HuanDecisionTests(unittest.TestCase):
    def ready(self, policy):
        for t in (10., 10.3, 10.6):
            policy.evaluate(t, intent(t), raw(t), [])

    def test_fused_clear_packet_never_releases_a_raw_lidar_obstacle(self):
        policy = IntegratedFusionPolicy()
        self.ready(policy)
        packet = camera(11., position=(8., 3., .9))
        result = policy.evaluate(11., intent(11.), raw(11., True), [], dict(packet=packet))
        self.assertEqual(result["action"], "WAIT")
        self.assertEqual(result["reason"], "LIDAR_CORRIDOR_BLOCKED")

    def test_fused_prediction_can_add_a_stop_before_raw_lidar_blocks(self):
        policy = IntegratedFusionPolicy()
        self.ready(policy)
        result = policy.evaluate(11., intent(11.), raw(11.), [], dict(packet=camera(11., position=(.8, 0., .9))))
        self.assertEqual(result["reason"], "HUAN_FUSION_PREDICTED_CONFLICT")
        self.assertEqual(result["speed_cap_mps"], 0.)

    def test_release_forecast_covers_full_horizon_after_a_clock_tick(self):
        policy = IntegratedFusionPolicy()
        plan = intent(11.)
        packet = dict(packet=camera(11.))
        for now in (11.08, 11.299):
            decision = policy.evaluate(now, plan, raw(now), [], packet)
            self.assertTrue(decision["fusion_assessment"]["robot_plan_valid"])
            self.assertTrue(decision["fusion_assessment"]["valid"])
        too_old = policy.evaluate(11.31, plan, raw(11.31), [], packet)
        self.assertEqual(too_old["reason"], "ROUTE_INTENT_UNAVAILABLE")
        self.assertFalse(too_old["fusion_assessment"]["valid"])

    def stationary_laser_packet(self, *, measured_velocity=None, stamp=10.9):
        support = dict(stamp=stamp, position=[.95, -1.39], velocity=[0., 0.], points=20,
                       residual=.001, radius=.25,
                       measured_velocity=[.02, 0.] if measured_velocity is None else measured_velocity)
        return dict(packet=dict(schema="camera_laser_handoff_v1", timestamp=11.,
                                tracks=[dict(id=1, source="laser_scan_tf", vision_last_stamp=10.,
                                             lidar_support=support)]))

    def test_reduced_reserve_only_changes_slow_laser_conflicts_during_detour(self):
        plan = intent(11.)
        plan["route_snapshot"]["avoidance_state"] = 1
        packet = self.stationary_laser_packet()
        standard = IntegratedFusionPolicy().evaluate(11., plan, raw(11.), [], packet)
        self.assertEqual(standard["reason"], "HUAN_FUSION_PREDICTED_CONFLICT")
        policy = IntegratedFusionPolicy(reduced_standing_laser_reserve=True)
        adjusted = policy.evaluate(11., plan, raw(11.), [], packet)["fusion_assessment"]
        obstacle = adjusted["obstacles"][0]
        self.assertAlmostEqual(obstacle["baseline_required_separation_m"], 1.455)
        self.assertAlmostEqual(obstacle["required_separation_m"], 1.335)
        self.assertEqual(adjusted["risk"], "CLEAR_WITHIN_MODEL")
        self.assertEqual(adjusted["baseline_risk"], "POTENTIAL_COLLISION")

        moved = self.stationary_laser_packet(measured_velocity=[.13, 0.])
        self.assertEqual(policy.evaluate(11., plan, raw(11.), [], moved)["reason"],
                         "HUAN_FUSION_PREDICTED_CONFLICT")
        plan["route_snapshot"]["avoidance_state"] = 0
        self.assertEqual(policy.evaluate(11., plan, raw(11.), [], packet)["reason"],
                         "HUAN_FUSION_PREDICTED_CONFLICT")

    def test_reduced_reserve_cannot_clear_raw_lidar_or_stale_inputs(self):
        policy = IntegratedFusionPolicy(reduced_standing_laser_reserve=True)
        plan = intent(11.)
        plan["route_snapshot"]["avoidance_state"] = 1
        result = policy.evaluate(11., plan, raw(11., True), [], self.stationary_laser_packet())
        self.assertEqual(result["reason"], "LIDAR_CORRIDOR_BLOCKED")
        stale = policy.evaluate(11., plan, raw(11.), [], self.stationary_laser_packet(stamp=10.6))
        self.assertFalse(stale["fusion_assessment"]["valid"])
        self.assertNotIn("baseline_risk", stale["fusion_assessment"])
        vision = policy.evaluate(11., plan, raw(11.), [], dict(packet=camera(11., position=(.8, 0., .9))))
        self.assertEqual(vision["reason"], "HUAN_FUSION_PREDICTED_CONFLICT")
        self.assertNotIn("baseline_risk", vision["fusion_assessment"])

    def crawl_sample(self, policy, now, *, speed=.04, position=2.56,
                     measured_velocity=None, blocked=False, gate=0, plan_age=0.):
        plan = intent(now-plan_age)
        points = [[.9, 0.], [4., 0.]]
        plan.update(route_geometry_odom_m=points, route_corridor_xy_m=points,
                    positions=nominal_positions(points, .12, horizon=RELEASE_PLAN_LOOKAHEAD_S))
        plan["route_snapshot"].update(avoidance_state=1, behavior_gate={"action": gate})
        laser_packet = self.stationary_laser_packet(stamp=now-.1, measured_velocity=measured_velocity)
        laser_packet["packet"]["timestamp"] = now
        laser_packet["packet"]["tracks"][0]["lidar_support"]["position"] = [position, 0.]
        scan = dict(raw(now, blocked), exact_sensor_time_tf=True)
        return policy.evaluate(now, plan, scan, [], laser_packet, measured_speed_mps=speed)

    def test_crawl_only_sends_capped_motion_after_clear_confirmation(self):
        policy = IntegratedFusionPolicy(crawl_on_predicted_conflict=True)
        first = self.crawl_sample(policy, 11.)
        self.assertTrue(first["crawl_forecast"]["selected"])
        self.assertEqual(first["reason"], "CONFIRMING_CLEAR_CORRIDOR")
        second = self.crawl_sample(policy, 11.55, plan_age=.08)
        self.assertEqual((second["action"], second["speed_cap_mps"], second["reason"]),
                         ("SLOW", .06, "HUAN_FUSION_CLEAR_AT_CRAWL"))
        self.assertEqual(second["crawl_forecast"]["nominal_risk"], "POTENTIAL_COLLISION")
        self.assertEqual(second["fusion_assessment"]["risk"], "CLEAR_WITHIN_MODEL")
        self.assertEqual(self.crawl_sample(IntegratedFusionPolicy(), 11.)["reason"],
                         "HUAN_FUSION_PREDICTED_CONFLICT")

    def test_crawl_denies_unverified_speed_raw_block_moving_track_and_slow_conflict(self):
        for kwargs in (dict(speed=None), dict(speed=.08), dict(gate=2), dict(measured_velocity=[.13, 0.])):
            policy = IntegratedFusionPolicy(crawl_on_predicted_conflict=True)
            decision = self.crawl_sample(policy, 11., **kwargs)
            self.assertEqual(decision["reason"], "HUAN_FUSION_PREDICTED_CONFLICT")
            self.assertNotIn("crawl_forecast", decision)
        policy = IntegratedFusionPolicy(crawl_on_predicted_conflict=True)
        blocked = self.crawl_sample(policy, 11., blocked=True)
        self.assertEqual(blocked["reason"], "LIDAR_CORRIDOR_BLOCKED")
        self.assertNotIn("crawl_forecast", blocked)
        slow_conflict = self.crawl_sample(policy, 11.1, position=2.39)
        self.assertFalse(slow_conflict["crawl_forecast"]["selected"])
        self.assertEqual(slow_conflict["reason"], "HUAN_FUSION_PREDICTED_CONFLICT")

    def test_crawl_and_reduced_reserve_keep_distinct_forecasts(self):
        policy = IntegratedFusionPolicy(crawl_on_predicted_conflict=True, reduced_standing_laser_reserve=True)
        decision = self.crawl_sample(policy, 11., position=2.45)
        self.assertTrue(decision["crawl_forecast"]["selected"])
        self.assertEqual(decision["crawl_forecast"]["nominal_baseline_risk"], "POTENTIAL_COLLISION")
        self.assertEqual(decision["crawl_forecast"]["nominal_risk"], "POTENTIAL_COLLISION")
        self.assertEqual(decision["fusion_assessment"]["risk"], "CLEAR_WITHIN_MODEL")

    def test_nearby_slow_applies_to_fused_caution_but_fused_wait_still_wins(self):
        standard = IntegratedFusionPolicy()
        nearby = IntegratedFusionPolicy(nearby_slow=True)
        for policy in (standard, nearby):
            self.ready(policy)

        def tick(policy, now, x, velocity):
            route = dict(intent(now), route_corridor_xy_m=[[.9, 0.], [3.9, 0.]])
            return policy.evaluate(now, route, raw(now), [],
                                   dict(packet=camera(now, position=(x, 0., .9), velocity=velocity)))

        self.assertEqual(tick(standard, 11., 4.5, (0., 0., 0.))["action"], "SLOW")
        self.assertEqual(tick(nearby, 11., 4.5, (0., 0., 0.))["action"], "BASELINE")
        self.assertEqual(tick(nearby, 11.1, 3.5, (0., 0., 0.))["action"], "SLOW")
        result = tick(nearby, 11.2, 4.5, (-2., 0., 0.))
        self.assertEqual(result["reason"], "HUAN_FUSION_PREDICTED_CONFLICT")
        self.assertFalse(result["fusion_observations"][0]["within_caution_corridor"])

    def test_persistent_fused_conflict_resets_clear_confirmation(self):
        policy = IntegratedFusionPolicy()
        self.ready(policy)
        for t in (11., 11.1, 11.3, 11.59):
            conflict = dict(packet=camera(t, position=(.8, 0., .9)))
            result = policy.evaluate(t, intent(t), raw(t), [], conflict)
            self.assertEqual(result["reason"], "HUAN_FUSION_PREDICTED_CONFLICT")
            self.assertIsNone(policy.base.clear_since)
        for t in (11.61, 11.9, 12.12):
            clear = dict(packet=camera(t, position=(8., 3., .9)))
            result = policy.evaluate(t, intent(t), raw(t), [], clear)
            expected = "WAIT" if t < 12.11 else "BASELINE"
            self.assertEqual(result["action"], expected)
        t = 12.16
        conflict = dict(packet=camera(t, position=(.8, 0., .9)))
        result = policy.evaluate(t, intent(t), raw(t, True), [], conflict)
        self.assertEqual(result["reason"], "LIDAR_CORRIDOR_BLOCKED")

    def test_new_model_does_not_trust_a_forged_assessment_boolean(self):
        policy = IntegratedFusionPolicy()
        self.ready(policy)
        result = policy.evaluate(11., intent(11.), raw(11.), [],
                                 dict(packet=None, assessment=dict(valid=True, risk="CLEAR_WITHIN_MODEL")))
        self.assertIn("UNAVAILABLE", result["fusion_status"])

    def test_missing_scan_cannot_be_cleared_by_camera(self):
        policy = IntegratedFusionPolicy()
        self.ready(policy)
        result = policy.evaluate(11., intent(11.), {"valid": False}, [], dict(packet=camera()))
        self.assertEqual(result["action"], "WAIT")

    def test_fused_conflict_vetoes_fast_resume_and_restarts_standard_confirmation(self):
        policy = IntegratedFusionPolicy(resume_fresh_scans=True)

        def tick(now, scan, fusion=None):
            return policy.evaluate(now, intent(now), dict(raw(scan), exact_sensor_time_tf=True), [], fusion)

        tick(10., 9.98)
        self.assertEqual(tick(10.6, 10.55)["action"], "BASELINE")
        policy.evaluate(10.66, None, None, [], transient_gap=True)
        self.assertEqual(tick(10.77, 10.71)["action"], "WAIT")
        result = tick(10.88, 10.81, dict(packet=camera(10.88, position=(.8, 0., .9))))
        self.assertEqual(result["reason"], "HUAN_FUSION_PREDICTED_CONFLICT")
        self.assertNotIn("resume_confirmation", result)
        self.assertEqual(tick(11., 10.96)["action"], "WAIT")
        self.assertEqual(tick(11.4, 11.35)["action"], "WAIT")
        self.assertEqual(tick(11.51, 11.46)["action"], "BASELINE")

    def test_lidar_only_mode_never_uses_camera_or_fused_predictions(self):
        policy = IntegratedFusionPolicy("lidar-only")
        self.ready(policy)
        result = policy.evaluate(11., intent(11.), raw(11.), [], dict(packet=camera(11., position=(.8, 0., .9))))
        self.assertEqual(result["action"], "BASELINE")
        self.assertEqual(result["fusion_status"], "NOT_SELECTED")

    def test_laser_fallback_is_labelled_and_does_not_invent_camera_velocity(self):
        policy = IntegratedFusionPolicy()
        self.ready(policy)
        result = policy.evaluate(11., intent(11.), raw(11.), [], None)
        self.assertEqual(result["fusion_status"], "LIDAR_FALLBACK_FUSION_UNAVAILABLE")
        self.assertEqual(result["fusion_observations"], [])

    def test_input_diagnostics_keep_wait_and_distinguish_vision_from_lidar(self):
        decision = IntegratedFusionPolicy().evaluate(11., None, {"valid": False}, [])
        result = identify_input_wait(decision, "vision_stream", "no packet")
        self.assertEqual(result["reason"], "VISION_STREAM_UNAVAILABLE")
        self.assertEqual(result["action_code"], 0)

    def test_standing_release_requests_same_speed_for_clear_and_slow(self):
        route = {"standing_detour_speed_limited": True, "behavior_gate": {"action": 0}}
        plan = {"route_snapshot": route}
        clear = {"action": "BASELINE", "action_code": 2, "speed_cap_mps": .12, "reason": "LIDAR_CLEAR"}
        caution = {"action": "SLOW", "action_code": 1, "speed_cap_mps": .06, "reason": "PERSON_NEAR"}
        adjusted = stabilize_standing_detour_release(clear, plan)
        self.assertEqual((adjusted["action"], adjusted["action_code"], adjusted["speed_cap_mps"]),
                         (caution["action"], caution["action_code"], caution["speed_cap_mps"]))
        self.assertEqual(adjusted["reason"], clear["reason"])
        self.assertIs(stabilize_standing_detour_release(caution, plan), caution)
        route.update(standing_detour_speed_limited=False, standing_detour_speed_cap_mps=.08)
        self.assertEqual(stabilize_standing_detour_release(clear, plan)["speed_cap_mps"], .06)

    def test_standing_release_keeps_stops_and_other_route_commands(self):
        route = {"standing_detour_speed_limited": True, "behavior_gate": {"action": 0}}
        plan = {"route_snapshot": route}
        stop = {"action": "WAIT", "action_code": 0, "speed_cap_mps": 0.}
        self.assertIs(stabilize_standing_detour_release(stop, plan), stop)
        clear = {"action": "BASELINE", "action_code": 2, "speed_cap_mps": .12}
        self.assertIs(stabilize_standing_detour_release(clear, None), clear)
        route["behavior_gate"]["action"] = 1
        self.assertIs(stabilize_standing_detour_release(clear, plan), clear)
        route.update(standing_detour_speed_limited=False, behavior_gate={"action": 0})
        self.assertIs(stabilize_standing_detour_release(clear, plan), clear)

    def test_sensor_modules_have_no_actuator_or_ground_truth_dependencies(self):
        root = Path(__file__).parent
        forbidden = {"gazebo_msgs", "nav2_msgs", "geometry_msgs"}
        for path in [root/"huan_fusion_bridge.py", root/"huan_fusion_policy.py", *list((root/"huan_fusion_vendor").glob("*.py"))]:
            tree = ast.parse(path.read_text(encoding="utf-8"))
            for node in ast.walk(tree):
                if isinstance(node, ast.ImportFrom):
                    self.assertNotIn((node.module or "").split(".")[0], forbidden, str(path))
                if isinstance(node, ast.Constant) and isinstance(node.value, str):
                    self.assertNotIn("/gazebo/model_states", node.value, str(path))
                    self.assertNotIn("cmd_vel_unstamped", node.value, str(path))


class GoalSlotTests(unittest.TestCase):
    def test_continues_preflight_and_retries_fresh_frame_inside_the_slot(self):
        target = 196.992
        current = SimpleNamespace(sim=192.354)
        report = {}
        checks = []

        def clock():
            return round(current.sim * 1e9), 0., True

        def inputs():
            checks.append(current.sim)
            return current.sim < target or current.sim >= 197.099

        def route():
            return current.sim >= target

        for sim in (192.354, 196.935, 197.01):
            current.sim = sim
            self.assertFalse(synchronized_goal_ready(target, clock, inputs, route, report))
        self.assertFalse(report["sensor_inputs_ready"])
        self.assertTrue(report["command_route_ready"])
        current.sim = 197.099
        self.assertTrue(synchronized_goal_ready(target, clock, inputs, route, report))
        self.assertEqual(checks, [192.354, 196.935, 197.01, 197.099])

    def test_denies_an_unavailable_route_and_a_slot_expiring_during_the_check(self):
        report = {}
        fresh = (17_120_000_000, 0., True)
        self.assertFalse(synchronized_goal_ready(17., lambda: fresh, lambda: True, lambda: False, report))
        self.assertFalse(report["command_route_ready"])

        snapshots = iter((fresh, (17_151_000_000, 0., True)))
        with self.assertRaisesRegex(RuntimeError, "Missed the synchronized goal slot"):
            synchronized_goal_ready(17., lambda: next(snapshots), lambda: True, lambda: True, report)

        self.assertFalse(synchronized_goal_ready(17., lambda: (fresh[0], .3, False),
                                                 lambda: self.fail("stale clock used"),
                                                 lambda: self.fail("stale clock used"), report))


class ClosePassTests(unittest.TestCase):
    def setUp(self):
        self.monitor = ClosePassMonitor()
        self.monitor.arm(dict(track_id=1, center_odom=[3.2, 0.]))
        self.plan = dict(route_geometry_odom_m=[[.8, 0.], [2.2, -1.2], [4.2, -1.2], [5.4, 0.]],
                         route_revision=3, route_snapshot={"standing_detour_active": True},
                         map_to_odom={"x": .10, "y": -.10, "yaw": 0.})

    def sample(self, *, track_id=1, source="laser_scan_tf", stamp=9.8, position=(3.2, 0.),
               velocity=(0., 0.), lidar_confirmed=True, action="SLOW", now=10., reason=None, exact_tf=True):
        risk = dict(valid=True, obstacles=[dict(id=track_id, lidar_confirmed=lidar_confirmed)])
        decision = dict(action=action, action_code=1 if action == "SLOW" else 0,
                        speed_cap_mps=.06 if action == "SLOW" else 0.,
                        reason=reason or ("PERSON_NEAR" if action == "SLOW" else "HUAN_FUSION_PREDICTED_CONFLICT"),
                        fusion_assessment=risk, route_revision=3,
                        raw_corridor={"valid": True, "exact_sensor_time_tf": exact_tf})
        track = dict(id=track_id, source=source,
                     lidar_support=dict(stamp=stamp, position=position, velocity=(0., 0.),
                                        measured_velocity=velocity, radius=.25))
        restricted = self.monitor.restrict(now, decision, dict(packet=dict(tracks=[track])), self.plan)
        return restricted, decision

    def test_fresh_matched_stationary_support_preserves_slow_or_existing_wait(self):
        for source in ("laser_scan_tf", "vision_rgbd_tf"):
            restricted, decision = self.sample(source=source)
            self.assertFalse(restricted)
            self.assertEqual((decision["action"], decision["speed_cap_mps"]), ("SLOW", .06))
        _, decision = self.sample(action="WAIT")
        self.assertEqual(decision["reason"], "HUAN_FUSION_PREDICTED_CONFLICT")

    def test_lost_or_unconfirmed_identity_and_stale_support_hold(self):
        for kwargs in (dict(track_id=2), dict(source="laser_scan_tf_reacquired"),
                       dict(lidar_confirmed=False), dict(stamp=9.6), dict(velocity=None)):
            restricted, decision = self.sample(**kwargs)
            self.assertTrue(restricted)
            self.assertEqual((decision["action_code"], decision["speed_cap_mps"]), (0, 0.))
            self.assertEqual(decision["reason"], "CLOSE_PASS_TRACK_UNAVAILABLE")
        self.assertFalse(self.sample()[0])

    def test_one_motion_sample_stops_immediately_and_only_new_scans_can_confirm(self):
        for now in (10., 10.02, 10.04):
            restricted, decision = self.sample(velocity=(0., .2), now=now, action="WAIT",
                                               reason="CONFIRMING_CLEAR_CORRIDOR")
            self.assertTrue(restricted)
            self.assertEqual(decision["reason"], "CLOSE_PASS_MOTION_UNCONFIRMED")
            self.assertEqual(decision["close_pass_evidence"]["scan_count"], 1)
        self.sample(velocity=(0., .2), stamp=9.9, now=10.1)
        _, decision = self.sample(velocity=(0., .2), stamp=10., now=10.2, action="WAIT")
        self.assertEqual(decision["reason"], "HUAN_FUSION_PREDICTED_CONFLICT")
        self.assertEqual(decision["close_pass_monitor"], "CLOSE_PASS_PERSON_MOVED")

    def test_motion_hold_can_release_only_after_same_track_is_stable_near_anchor(self):
        self.assertTrue(self.sample(velocity=(.31, 0.))[0])
        for stamp in (9.90, 10.05, 10.20):
            restricted, decision = self.sample(stamp=stamp, now=stamp+.10)
            self.assertTrue(restricted)
            self.assertEqual(decision["reason"], "CLOSE_PASS_RECHECKING_STATIONARITY")
        self.assertFalse(self.sample(stamp=10.31, now=10.41)[0])
        self.assertFalse(self.monitor.motion_check_pending)

    def test_offset_with_clear_route_is_revalidated_without_calling_it_motion(self):
        for stamp in (9.80, 9.94, 10.08):
            restricted, decision = self.sample(stamp=stamp, now=stamp+.10, position=(3.2, .2))
            self.assertTrue(restricted)
            self.assertEqual(decision["reason"], "CLOSE_PASS_RECHECKING_STATIONARITY")
        restricted, decision = self.sample(stamp=10.21, now=10.31, position=(3.2, .2))
        self.assertFalse(restricted)
        self.assertTrue(decision["close_pass_route_revalidated"])
        self.assertFalse(self.monitor.motion_detected)
        self.assertEqual(self.monitor.blocker["center_odom"], (3.2, .2))

    def test_offset_toward_route_requests_replan_while_waiting(self):
        for stamp in (9.80, 9.94, 10.08, 10.21):
            restricted, decision = self.sample(stamp=stamp, now=stamp+.10, position=(3.2, -.3))
            self.assertTrue(restricted)
        self.assertEqual(decision["reason"], "CLOSE_PASS_REPLAN_REQUIRED")
        self.assertEqual(decision["action_code"], 0)
        x, y = decision["close_pass_replan_request"]["center_map"]
        self.assertAlmostEqual(x, 3.1)
        self.assertAlmostEqual(y, -.2)

        _, invalid = self.sample(stamp=10.26, now=10.36, position=(3.2, -.3), exact_tf=False)
        self.assertNotIn("close_pass_replan_request", invalid)
        self.plan["route_geometry_odom_m"] = [[.8, 0.], [2.2, -1.5], [4.2, -1.5], [5.4, 0.]]
        restricted, verified = self.sample(stamp=10.31, now=10.41, position=(3.2, -.3))
        self.assertFalse(restricted)
        self.assertTrue(verified["close_pass_route_revalidated"])

    def test_route_revalidation_checks_waypoint_corner_cut(self):
        plan = {"route_geometry_odom_m": [[0., 0.], [3., 0.], [4.5, 2.598]]}
        self.assertFalse(standing_route_has_clearance(plan, [2.40, 1.04], .25))

    def recovery_tick(self, policy, now, scan, *, missing=False, blocked=False, exact=True, revision=3,
                      track_id=1, speed=0., position=(3.2, 0.), support_scan=None):
        plan = intent(now, revision)
        points = self.plan["route_geometry_odom_m"]
        plan.update(route_geometry_odom_m=points, route_corridor_xy_m=points,
                    positions=nominal_positions(points, .06, horizon=RELEASE_PLAN_LOOKAHEAD_S),
                    standing_detour_speed_cap_mps=.06)
        plan["route_snapshot"]["standing_detour_active"] = True
        fusion = None
        if not missing:
            fusion = HuanDecisionTests().stationary_laser_packet(stamp=scan if support_scan is None else support_scan)
            fusion["packet"]["timestamp"] = now
            track = fusion["packet"]["tracks"][0]
            track["id"] = track_id
            track["vision_last_stamp"] = now-.1
            track["lidar_support"].update(position=position, measured_velocity=[speed, 0.])
        decision = policy.evaluate(now, plan, dict(raw(scan, blocked), exact_sensor_time_tf=exact), [], fusion)
        policy.check_close_pass(self.monitor, now, decision, fusion, plan)
        return decision

    def test_close_pass_short_gap_waits_for_two_new_matching_scans_and_resumes_at_slow_cap(self):
        policy = IntegratedFusionPolicy()
        self.recovery_tick(policy, 9.4, 9.38)
        self.assertNotEqual(self.recovery_tick(policy, 10., 9.98)["action"], "WAIT")
        lost = self.recovery_tick(policy, 10.03, 10.01, missing=True)
        self.assertEqual((lost["reason"], lost["action"]), ("CLOSE_PASS_TRACK_UNAVAILABLE", "WAIT"))
        for now, scan in ((10.06, 10.02), (10.10, 10.07), (10.12, 10.07)):
            self.assertEqual(self.recovery_tick(policy, now, scan)["action"], "WAIT")
        recovered = self.recovery_tick(policy, 10.16, 10.14)
        self.assertEqual(recovered["reason"], "CLOSE_PASS_RESUMED_AFTER_TRACK_GAP")
        self.assertEqual((recovered["action"], recovered["speed_cap_mps"]), ("SLOW", .06))
        self.assertEqual(recovered["close_pass_resume_evidence"]["scan_stamps"], [10.07, 10.14])
        self.assertNotEqual(self.recovery_tick(policy, 10.19, 10.17)["action"], "WAIT")

    def test_close_pass_short_recovery_cannot_override_hazards_or_route_and_identity_changes(self):
        for kwargs in (dict(blocked=True), dict(exact=False), dict(support_scan=10.13),
                       dict(revision=4), dict(track_id=2),
                       dict(speed=.18), dict(position=(.9, -.2)), dict(missing=True)):
            with self.subTest(kwargs=kwargs):
                self.setUp()
                policy = IntegratedFusionPolicy()
                self.recovery_tick(policy, 9.4, 9.38)
                self.assertNotEqual(self.recovery_tick(policy, 10., 9.98)["action"], "WAIT")
                self.assertEqual(self.recovery_tick(policy, 10.03, 10.01, missing=True)["action"], "WAIT")
                self.assertEqual(self.recovery_tick(policy, 10.10, 10.07)["action"], "WAIT")
                hold = self.recovery_tick(policy, 10.16, 10.14, **kwargs)
                self.assertEqual(hold["action"], "WAIT")
                self.assertNotIn("close_pass_resume", hold)
                following = self.recovery_tick(policy, 10.22, 10.20)
                self.assertEqual(following["action"], "WAIT")
        self.setUp()
        policy = IntegratedFusionPolicy()
        self.recovery_tick(policy, 9.4, 9.38)
        self.assertNotEqual(self.recovery_tick(policy, 10., 9.98)["action"], "WAIT")
        self.assertEqual(self.recovery_tick(policy, 10.03, 10.01, missing=True)["action"], "WAIT")
        self.assertEqual(self.recovery_tick(policy, 10.23, 10.21)["action"], "WAIT")
        self.assertEqual(self.recovery_tick(policy, 10.28, 10.26)["action"], "WAIT")

    def checked_pass_tick(self, policy, now, *, matched=True, measured_speed=.04,
                          position=(.6, 1.05), velocity=(0., 0.), exact=True, extra_track=False,
                          camera_velocity=None, source="laser_scan_tf", missing=False,
                          scan_stamp=None, revision=3, camera_position=None):
        plan = intent(now, revision)
        points = [[0., 0.], [3., 0.]]
        plan.update(route_geometry_odom_m=points, route_corridor_xy_m=points,
                    positions=nominal_positions(points, .06, horizon=RELEASE_PLAN_LOOKAHEAD_S),
                    standing_detour_speed_cap_mps=.06)
        plan["route_snapshot"]["standing_detour_active"] = True
        scan = dict(raw(now-.02 if scan_stamp is None else scan_stamp, True),
                    exact_sensor_time_tf=exact, blocking_points=3)
        fusion = HuanDecisionTests().stationary_laser_packet(stamp=scan["stamp"], measured_velocity=velocity)
        fusion["packet"]["timestamp"] = now
        track = fusion["packet"]["tracks"][0]
        support = track["lidar_support"]
        support["position"] = position
        if source == "vision_rgbd_tf":
            packet = camera(now, position=(*(camera_position or position), .9),
                            velocity=camera_velocity or (0., 0., 0.))
            track.update(source=source, raw=packet["tracks"][0])
        if extra_track:
            other = deepcopy(fusion["packet"]["tracks"][0])
            other["id"] = 2
            other["lidar_support"].update(position=[.8, 0.], velocity=[0., -.35], measured_velocity=[0., -.35])
            fusion["packet"]["tracks"].append(other)
        def match(stamp, center, radius, count):
            self.assertEqual((stamp, center, radius, count), (scan["stamp"], position, .25, 20))
            return [[.6, .80], [.61, .81], [.62, .81]] if matched else None
        observations = []
        if camera_velocity is not None:
            packet = camera(now, position=(*position, .9), velocity=camera_velocity)
            observations.append(dict(packet=packet, track=packet["tracks"][0], position_ok=True, velocity_ok=True,
                                     match=dict(matched=True, center_odom_m=position, radius_m=.25)))
        if missing:
            fusion = None
        decision = policy.evaluate(now, plan, scan, observations, fusion, measured_speed_mps=measured_speed,
                                   close_monitor=self.monitor, match_blocking_returns=match)
        policy.check_close_pass(self.monitor, now, decision, fusion, plan)
        return decision

    def test_close_pass_keeps_scan_history_across_sources_and_waits_for_two_clear_scans(self):
        self.monitor.arm(dict(track_id=1, center_odom=[.6, 1.05]))
        policy = IntegratedFusionPolicy(reduced_standing_laser_reserve=True)
        for i in range(8):
            source = "vision_rgbd_tf" if i % 2 == 0 else "laser_scan_tf"
            decision = self.checked_pass_tick(policy, 11.+i*.08, source=source)
        self.assertEqual(decision["action"], "SLOW")
        lost = self.checked_pass_tick(policy, 11.61, missing=True)
        self.assertEqual(lost["action"], "WAIT")
        self.assertTrue(lost["standing_history_preserved"])
        first = self.checked_pass_tick(policy, 11.66, source="vision_rgbd_tf")
        self.assertEqual(first["action"], "WAIT")
        self.assertNotIn("standing_route_check", first)
        repeat = self.checked_pass_tick(policy, 11.68, source="vision_rgbd_tf", scan_stamp=11.66-.02)
        self.assertEqual(repeat["action"], "WAIT")
        second = self.checked_pass_tick(policy, 11.71)
        self.assertEqual(second["action"], "WAIT")
        self.assertEqual(second["standing_route_check_status"], "CHECKED")
        third = self.checked_pass_tick(policy, 11.76, source="vision_rgbd_tf")
        self.assertEqual((third["action"], third["speed_cap_mps"]), ("SLOW", .06))
        self.assertEqual(len(third["standing_route_resume_scans"]), 2)

    def test_close_pass_history_expires_on_long_gap_route_change_and_moving_camera(self):
        for kwargs in (dict(revision=4), dict(source="vision_rgbd_tf", camera_velocity=(0., -.18, 0.)),
                       dict(exact=False), dict(scan_stamp=11.93)):
            with self.subTest(kwargs=kwargs):
                self.setUp()
                self.monitor.arm(dict(track_id=1, center_odom=[.6, 1.05]))
                policy = IntegratedFusionPolicy(reduced_standing_laser_reserve=True)
                for i in range(8):
                    self.checked_pass_tick(policy, 11.+i*.08)
                self.assertEqual(self.checked_pass_tick(policy, 11.61, missing=True)["action"], "WAIT")
                now = 11.95 if "scan_stamp" in kwargs else 11.66
                result = self.checked_pass_tick(policy, now, **kwargs)
                self.assertEqual(result["action"], "WAIT")
                self.assertNotIn("standing_route_resume_scans", result)
                self.assertEqual(result["close_pass_evidence"]["scan_count"], 1)

    def test_close_pass_release_restarts_after_an_unmatched_return(self):
        self.monitor.arm(dict(track_id=1, center_odom=[.6, 1.05]))
        policy = IntegratedFusionPolicy(reduced_standing_laser_reserve=True)
        for i in range(4):
            self.checked_pass_tick(policy, 11.+i*.1)
        first = self.checked_pass_tick(policy, 11.45)
        self.assertEqual(first["action"], "WAIT")
        self.assertEqual(first["standing_route_check_status"], "CHECKED")
        other = self.checked_pass_tick(policy, 11.50, matched=False)
        self.assertEqual(other["reason"], "LIDAR_CORRIDOR_BLOCKED")
        restarted = self.checked_pass_tick(policy, 11.55)
        self.assertEqual(restarted["action"], "WAIT")
        self.assertEqual(self.checked_pass_tick(policy, 11.60)["action"], "SLOW")

    def test_close_pass_uses_checked_geometry_after_fresh_stationary_scans(self):
        self.monitor.arm(dict(track_id=1, center_odom=[.6, 1.05]))
        policy = IntegratedFusionPolicy(reduced_standing_laser_reserve=True)
        for t in (11., 11.1, 11.2, 11.3):
            held = self.checked_pass_tick(policy, t)
            self.assertEqual(held["reason"], "LIDAR_CORRIDOR_BLOCKED")
            self.assertNotIn("standing_route_check", held)
        for t in (11.41, 11.51, 11.61, 11.71, 11.81, 11.91):
            decision = self.checked_pass_tick(policy, t)
        self.assertEqual((decision["action"], decision["speed_cap_mps"]), ("SLOW", .06))
        self.assertEqual(decision["standing_route_check_status"], "CHECKED")
        self.assertEqual(decision["raw_corridor"]["baseline_blocking_points"], 3)
        self.assertEqual(decision["standing_route_check"]["original_fusion_risk"], "POTENTIAL_COLLISION")
        self.assertEqual(decision["fusion_assessment"]["risk"], "CLEAR_WITHIN_MODEL")
        still = self.checked_pass_tick(policy, 12., camera_velocity=(0., 0., 0.))
        self.assertEqual(still["action"], "SLOW")
        self.assertTrue(still["vision_observations"][0]["standing_route_checked"])

    def test_close_pass_keeps_unmatched_returns_motion_and_other_forecasts_blocking(self):
        for kwargs, reason in ((dict(matched=False), "LIDAR_CORRIDOR_BLOCKED"),
                               (dict(velocity=(.13, 0.)), "LIDAR_CORRIDOR_BLOCKED"),
                               (dict(exact=False), "LIDAR_CORRIDOR_BLOCKED"),
                               (dict(measured_speed=None), "LIDAR_CORRIDOR_BLOCKED"),
                               (dict(position=(.6, .98)), "LIDAR_CORRIDOR_BLOCKED"),
                               (dict(source="vision_rgbd_tf", camera_position=(.6, .85)), "LIDAR_CORRIDOR_BLOCKED"),
                               (dict(camera_velocity=(0., -.18, 0.)), "VISION_PREDICTED_CONFLICT"),
                               (dict(extra_track=True), "HUAN_FUSION_PREDICTED_CONFLICT")):
            with self.subTest(kwargs=kwargs):
                self.setUp()
                self.monitor.arm(dict(track_id=1, center_odom=[.6, 1.05]))
                policy = IntegratedFusionPolicy(reduced_standing_laser_reserve=True)
                for i in range(10):
                    self.checked_pass_tick(policy, 11.+i*.11)
                decision = self.checked_pass_tick(policy, 12.10, **kwargs)
                self.assertEqual((decision["action"], decision["reason"]), ("WAIT", reason))
                if "measured_speed" in kwargs:
                    self.assertEqual(decision["standing_route_check_status"], "WHEEL_ODOMETRY_UNAVAILABLE")
                if "matched" in kwargs:
                    self.assertEqual(decision["standing_route_check_status"], "UNCONFIRMED_BLOCKING_RETURNS")


class ReportingTests(unittest.TestCase):
    def test_standing_outline_uses_wheel_and_robot_heading(self):
        wheel, body = outline_gaps([0., 0.], 0., [0., 1.], .25)
        self.assertAlmostEqual(wheel, .44)
        self.assertAlmostEqual(body, .44)
        wheel, body = outline_gaps([0., 0.], math.pi/2, [0., 1.], .25)
        self.assertGreater(wheel, body)
        self.assertAlmostEqual(body, .50)

    def test_standing_outline_bound_excludes_missing_heading(self):
        rows = [dict(sim=stamp, route=dict(pose_tf_valid=valid, robot_pose_map=[0., y, 0.]),
                     scoring_only=dict(robot_truth=[0., y], person_truth=[0., 1.]))
                for stamp, y, valid in ((1., 0., True), (1.1, .10, True), (1.2, .8, False))]
        report = standing_outline_review(rows, .25)
        self.assertEqual(report["samples_with_valid_yaw"], 2)
        self.assertEqual(report["evaluated_intervals"], 1)
        self.assertAlmostEqual(report["closest_outline_sample"]["gap_m"], .34)
        self.assertAlmostEqual(report["minimum_interpolated_outline_lower_bound_m"], .29)

    def test_lidar_comparison_keeps_failed_camera_check_as_not_applicable(self):
        metrics = dict(passed=False, checks={"arrival": True, "circular_clearance_at_least_20cm": True,
                                            "vision_actually_limited_moving_speed": False})
        report = comparison_score(metrics, "lidar-only")
        self.assertTrue(report["passed"])
        self.assertFalse(report["original_9_checks"]["vision_actually_limited_moving_speed"])
        self.assertEqual(report["not_applicable_checks"], ["vision_actually_limited_moving_speed"])
        self.assertFalse(comparison_score(metrics, "legacy")["passed"])

    def test_clearance_failure_is_never_excluded_in_lidar_mode(self):
        metrics = dict(passed=False, checks={"circular_clearance_at_least_20cm": False,
                                            "vision_actually_limited_moving_speed": False})
        self.assertFalse(comparison_score(metrics, "lidar-only")["passed"])

    def test_huan_mode_cannot_pass_without_actual_fusion_input(self):
        metrics = dict(passed=True, checks={"arrival": True, "vision_actually_limited_moving_speed": True})
        self.assertFalse(comparison_score(metrics, "huan")["passed"])
        self.assertFalse(comparison_score(metrics, "huan", dict(fusion_ready_before_goal=True,
                                                             huan_fusion_used_during_mission=False))["passed"])

    def test_rejoin_scoring_retains_stop_evidence_and_requires_arrival_and_clearance(self):
        checks = dict(one_goal=True, arrival=True, matched_vision_positions=True,
                      vision_actually_limited_moving_speed=True, lidar_driven_wait_observed=False,
                      resumed_after_wait=False, moved_before_block=False,
                      person_crossed_during_mission=True, circular_clearance_at_least_20cm=True)
        integration = dict(fusion_ready_before_goal=True, huan_fusion_used_during_mission=True)
        metrics = dict(checks=checks)
        report = comparison_score(metrics, "huan", integration, early_rejoin=True,
                                  early_rejoin_observed=True)
        self.assertTrue(report["passed"])
        self.assertFalse(report["original_9_checks_passed"])
        self.assertEqual(report["original_9_checks"], checks)
        self.assertEqual(set(report["not_applicable_checks"]),
                         {"lidar_driven_wait_observed", "resumed_after_wait", "moved_before_block"})
        self.assertFalse(comparison_score(metrics, "huan", integration)["passed"])
        self.assertFalse(comparison_score(metrics, "huan", integration, early_rejoin=True)["passed"])
        for name in ("arrival", "circular_clearance_at_least_20cm"):
            rejected = dict(checks, **{name: False})
            self.assertFalse(comparison_score(dict(checks=rejected), "huan", integration,
                                              early_rejoin=True, early_rejoin_observed=True)["passed"])
    def test_reusing_matching_binary_does_not_change_baseline_pointer(self):
        with tempfile.TemporaryDirectory() as tmp:
            home = Path(tmp)
            tools = Path(__file__).parent
            workspace = home/"carrierbot-lien-gate.previous"
            executable = workspace/"install/carrierbot_navigation/lib/carrierbot_navigation/fablab_guidance_node"
            executable.parent.mkdir(parents=True)
            executable.touch()
            names = ("behavior_gate_source/fablab_guidance_node.cpp", "behavior_gate_source/route_preview.hpp",
                     "behavior_gate_source/behavior_gate.hpp", "recovery_source/avoidance_recovery.hpp")
            (workspace/"gate_build_manifest.json").write_text(json.dumps({
                name: hashlib.sha256((tools/name).read_bytes()).hexdigest() for name in names}))
            runs = home/"carrierbot_test_runs"
            runs.mkdir()
            baseline = runs/"behavior_workspace.txt"
            baseline.write_text(str(workspace)+"\n")
            prior = baseline.read_bytes()
            self.assertTrue(ensure(home, tools))
            self.assertEqual(baseline.read_bytes(), prior)
            self.assertEqual((runs/"behavior_huan_workspace.txt").read_text(), str(workspace)+"\n")
            (workspace/"gate_build_manifest.json").write_text("{}")
            self.assertFalse(ensure(home, tools))
            self.assertEqual(baseline.read_bytes(), prior)

    def test_cpu_entry_installs_metadata_without_overwriting_huan_core(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            vision, tools, out = root/"vision", root/"tools", root/"out"
            for folder in (vision, tools, out, vision/".venv-vm/bin", vision/"src"):
                folder.mkdir(parents=True, exist_ok=True)
            content = b"# unchanged Huan core fixture\n"
            (vision/"src/profile_capture.py").write_bytes(content)
            (tools/"huan_vision_core.json").write_text(json.dumps(dict(normalized_sha256={
                "src/profile_capture.py": hashlib.sha256(content).hexdigest()})))
            original = ('device="cpu"\nfrom src.torso_depth import PoseTorsoDetector, associate_poses, measure_torso_depth\n'
                        'poses = pose_detector.detect(frame) if pose_detector is not None else []\n'
                        'torch.set_num_threads(2)\n')
            (vision/"main_vm_cpu.py").write_text(original)
            (vision/"config.vm.cpu.yaml").write_text('detection: {class_id: 0}\ninput: {}\nprediction: {}\noutput: {}\n')
            for name in (".venv-vm/bin/python", "yolo11n.pt", "yolo11s-pose.pt"):
                (vision/name).touch()
            command = prepare_vision(vision, tools, out)
            generated = (out/"main_fusion_roi_cpu.py").read_text()
            self.assertIn("install_huan_metadata('original')", generated)
            self.assertIn("torch.set_num_threads(1)", generated)
            setters = [n.args[0].value for n in ast.walk(ast.parse(generated))
                       if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
                       and n.func.attr == "set_num_threads"]
            self.assertEqual(setters, [1, 1])
            self.assertIn("--config", command)
            self.assertEqual((vision/"main_vm_cpu.py").read_text(), original)
            self.assertEqual((vision/"src/profile_capture.py").read_bytes(), content)
            (vision/"src/profile_capture.py").write_text("# unexpected edit\n")
            with self.assertRaisesRegex(RuntimeError, "Vision core differs"):
                prepare_vision(vision, tools, out)

    def test_metadata_does_not_change_original_measurements(self):
        original = camera(11., velocity=None)
        original["tracks"][0]["reference_valid"] = False
        info = SimpleNamespace(header=SimpleNamespace(frame_id="camera_rgb_optical_frame"),
                               width=640, height=480, distortion_model="plumb_bob", d=[0.]*5)
        owner = SimpleNamespace(info=info, current=[SimpleNamespace(shape=(480, 640, 3))],
                                get_intrinsics=lambda: dict(fx=500., fy=500., cx=320., cy=240.))
        sent = []
        pub = MetadataPublisher(owner, SimpleNamespace(publish=lambda m: sent.append(json.loads(m.data))))
        pub.publish(SimpleNamespace(data=json.dumps(original)))
        self.assertEqual(sent[0]["tracks"], original["tracks"])
        self.assertEqual(sent[0]["timestamp"], original["timestamp"])
        self.assertEqual(sent[0]["camera_model"]["source"], "camera_info")
        owner.get_cpu_runtime = lambda: dict(torch_threads_actual=2)
        pub.publish(SimpleNamespace(data=json.dumps(original)))
        self.assertEqual(sent[-1]["runtime_cpu"]["torch_threads_actual"], 2)
        self.assertEqual(sent[-1]["tracks"], original["tracks"])

    def test_cpu_backend_setup_preserves_requested_threads(self):
        for threads in (1, 2):
            calls = []
            fake = SimpleNamespace(set_num_threads=lambda n: calls.append(n),
                                   get_num_threads=lambda: calls[-1], get_num_interop_threads=lambda: 1)
            utils = ModuleType("ultralytics.utils")
            utils.torch_utils = SimpleNamespace(NUM_THREADS=3)

            def setup_cpu_backend():
                # Each predictor selects its device on first use.
                fake.set_num_threads(utils.torch_utils.NUM_THREADS)

            source = "torch.set_num_threads(2)\nsetup_cpu_backend()\nsetup_cpu_backend()\n"
            with patch.dict(sys.modules, {"ultralytics": ModuleType("ultralytics"),
                                          "ultralytics.utils": utils}):
                exec(configure_runtime_threads(source, threads),
                     {"torch": fake, "setup_cpu_backend": setup_cpu_backend})
            self.assertEqual(calls, [threads, threads, threads])
        for unknown in ("pass\n", "torch.set_num_threads(4)\n", "torch.set_num_threads(2)\ntorch.set_num_threads(2)\n"):
            with self.assertRaisesRegex(ValueError, "CPU source differs"):
                configure_runtime_threads(unknown, 1)

    def test_input_only_completion_cannot_enter_goal_publishing_branch(self):
        tree = ast.parse((Path(__file__).parent/"fusion_yield_trial.py").read_text(encoding="utf-8"))
        branches = [node for node in ast.walk(tree) if isinstance(node, ast.If)
                    and isinstance(node.test, ast.Attribute) and node.test.attr == "input_only"
                    and isinstance(node.test.value, ast.Name) and node.orelse]
        self.assertEqual(len(branches), 1)
        result = dict(passed=False, navigation_evaluated=False, goal_sent_count=0)
        def forbidden_goal(message):
            self.fail("Input-only mode published a navigation goal")
        branch = ast.Module(body=branches, type_ignores=[])
        exec(compile(branch, "input_only_actual_branch", "exec"),
             dict(args=SimpleNamespace(input_only=True), result=result, print=lambda *a, **kw: None,
                  goals=SimpleNamespace(publish=forbidden_goal)))
        self.assertTrue(result["input_readiness_passed"])
        self.assertFalse(result["navigation_evaluated"])
        self.assertEqual(result["goal_sent_count"], 0)

    def test_wheel_odometry_subscribed_in_normal_trial_startup(self):
        tree = ast.parse((Path(__file__).parent/"fusion_yield_trial.py").read_text(encoding="utf-8"))
        main = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == "main")
        topics = {
            item.args[1].value: item.args[2]
            for node in main.body if isinstance(node, ast.Assign)
            and any(isinstance(target, ast.Name) and target.id == "subscriptions" for target in node.targets)
            for item in node.value.elts if isinstance(item, ast.Call) and isinstance(item.args[1], ast.Constant)
        }
        self.assertIsInstance(topics.get("/odom"), ast.Name)
        self.assertEqual(topics["/odom"].id, "wheel_odometry")

    def test_wheel_speed_requires_fresh_measurement_in_both_clocks(self):
        self.assertEqual(fresh_wheel_speed((9.9, 19.9, .04), 10., 20.), .04)
        self.assertEqual(fresh_wheel_speed((9.9, 19.9, 0.), 10., 20.), 0.)
        for sample in (None, (9.7, 19.9, .04), (9.9, 19.5, .04), (10.01, 19.9, .04),
                       (9.9, 20.01, .04), (9.9, 19.9, float("nan"))):
            self.assertIsNone(fresh_wheel_speed(sample, 10., 20.))

    def test_performance_report_separates_late_route_wait_from_person_wait(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)
            (path/"summary.json").write_text(json.dumps(dict(elapsed_wall_s=10., elapsed_sim_s=3.5)))
            rows = [dict(sim=0., wall=0., decision=dict(action="WAIT", reason="LIDAR_CORRIDOR_BLOCKED"),
                         route=dict(robot_pose_map=[1., 0., 0.]), guard_command=[0., 0.]),
                    dict(sim=1., wall=3., decision=dict(action="WAIT", reason="ROUTE_INTENT_UNAVAILABLE"),
                         route=dict(robot_pose_map=[3.1, 0., 0.]), guard_command=[0., 0.]),
                    dict(sim=1.5, wall=4., decision=dict(action="BASELINE", reason="CLEAR"),
                         route=dict(robot_pose_map=[3.2, 0., 0.]), guard_command=[.04, 0.])]
            (path/"trace.jsonl").write_text("\n".join(json.dumps(x) for x in rows))
            report = analyze(path)
            self.assertEqual(report["wait_after_x3_sim_s_by_reason"], {"ROUTE_INTENT_UNAVAILABLE": .5})
            self.assertEqual(report["wait_duration_sim_s_by_reason"]["LIDAR_CORRIDOR_BLOCKED"], 1.)
            self.assertAlmostEqual(report["baseline_guard_command_after_x3_median_mps"], .04)

    def test_head_on_report_measures_wait_from_the_observed_exit(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)
            (path/"summary.json").write_text(json.dumps({"scenario": "head-on-narrow", "clear_cruise_enabled": True,
                "head_on_timing": {"actor_clearance_sim_s": 10.5}}))
            rows = [dict(sim=10., decision=dict(action="WAIT", reason="VISION_PREDICTED_CONFLICT"),
                         route=dict(robot_pose_map=[1., -.3, 0.], avoidance_state=0), guard_command=[0., 0.]),
                    dict(sim=11., decision=dict(action="BASELINE", reason="CLEAR"),
                         route=dict(robot_pose_map=[1., -.3, 0.], avoidance_state=1, clear_cruise_active=True),
                         guard_command=[.10, 0.]),
                    dict(sim=11.25, decision=dict(action="BASELINE", reason="CLEAR"),
                         route=dict(robot_pose_map=[1.025, -.3, 0.], avoidance_state=1), guard_command=[.10, 0.])]
            (path/"trace.jsonl").write_text("\n".join(json.dumps(x) for x in rows))
            report = analyze(path)
            self.assertTrue(report["clear_cruise_enabled"])
            self.assertEqual(report["clear_cruise_active_sim_s"], .25)
            result = report["head_on"]
            self.assertEqual(result["wait_after_actor_clearance_sim_s_by_reason"], {"VISION_PREDICTED_CONFLICT": .5})
            self.assertEqual(result["stopped_command_for_obstacle_before_actor_clearance_sim_s"], .5)
            self.assertEqual(result["first_detour_sim_s"], 11.)
            self.assertEqual(result["first_forward_guard_command_delay_sim_s"], .5)
            self.assertEqual(result["max_abs_lateral_map_m"], .3)
            self.assertEqual(result["clear_cruise_after_actor_clearance_sim_s"], .25)


class StandingBlockerTests(unittest.TestCase):
    def setUp(self):
        self.tracker = StationaryBlocker()
        self.intent = {"route_revision": 1, "route_snapshot": {"avoidance_state": 0},
                       "route_corridor_xy_m": [[1., 0.], [4., 0.]],
                       "map_to_odom": {"x": 1., "y": 2., "yaw": math.pi/2}}

    def sample(self, stamp, *, now=None, center=(3., 0.), speed=0., valid_scan=True):
        decision = {"reason": "VISION_PREDICTED_CONFLICT", "action": "WAIT", "route_revision": self.intent["route_revision"],
                    "raw_corridor": {"valid": valid_scan, "exact_sensor_time_tf": True}}
        packet = camera(stamp, velocity=(0., speed, 0.))
        observed = {"packet": packet, "track": packet["tracks"][0], "position_ok": True, "velocity_ok": True,
                    "match": {"matched": True, "image_stamp": stamp, "center_odom_m": center, "radius_m": .25}}
        return self.tracker.observe(stamp+.1 if now is None else now, self.intent, decision, [observed])

    def test_three_seconds_of_fresh_matched_observations_allow_planning(self):
        for i in range(6):
            self.assertIsNone(self.sample(10.+i*.5))
        result = self.sample(13.)
        self.assertEqual(result["stationary_duration_sim_s"], 3.)
        self.assertAlmostEqual(result["center_map"][0], -2.)
        self.assertAlmostEqual(result["center_map"][1], -2.)
        self.assertEqual(result["center_odom"], (3., 0.))
        self.assertIsNone(self.sample(13., now=13.51))
        self.assertIsNone(self.sample(13.3, valid_scan=False))

    def test_sensor_gap_and_repeated_image_cannot_establish_stationarity(self):
        for i in range(5):
            self.assertIsNone(self.sample(10., now=10.+i*.1))
        self.assertIsNone(self.sample(10.6))
        for i in range(5):
            self.assertIsNone(self.sample(11.+i*.5))

    def test_crossing_and_changed_route_do_not_authorize_planning(self):
        for i in range(9):
            self.assertIsNone(self.sample(10.+i*.4, center=(3., -.6+i*.12), speed=.3))
        for i in range(6):
            self.assertIsNone(self.sample(14.+i*.5))
        self.intent["route_revision"] = 2
        self.assertIsNone(self.sample(17.))

    def test_person_beside_the_route_does_not_authorize_planning(self):
        for i in range(9):
            self.assertIsNone(self.sample(10.+i*.5, center=(3., 1.1)))

    def test_close_person_approaching_the_leg_reserves_room_before_becoming_stationary(self):
        def evaluate(stamp, now, lateral=None, speed=0.):
            decision = {"reason": "LIDAR_CLEAR_WITH_MATCHED_VISION", "action": "BASELINE",
                        "route_revision": 1, "raw_corridor": {"valid": True, "exact_sensor_time_tf": True}}
            packet = camera(stamp, velocity=(0., speed, 0.))
            observations = [] if lateral is None else [
                {"packet": packet, "track": packet["tracks"][0], "position_ok": True, "velocity_ok": True,
                 "match": {"matched": True, "image_stamp": stamp, "center_odom_m": [3.2, lateral], "radius_m": .25}}
            ]
            self.assertIsNone(self.tracker.observe(now, self.intent, decision, observations))
            return decision

        self.assertEqual(evaluate(10., 10.1, -2.2)["action"], "SLOW")
        self.assertEqual(evaluate(10.2, 10.3, -1.6, .25)["action"], "WAIT")
        self.assertTrue(evaluate(10.65, 10.65)["standing_approach_reserved"])
        self.assertEqual(evaluate(10.75, 10.8, 1.3, .25)["action"], "SLOW")
        self.assertEqual(evaluate(11., 11.)["action"], "BASELINE")
        self.assertEqual(evaluate(11.1, 11.2, -1.6, .25)["action"], "WAIT")
        self.assertEqual(evaluate(11.9, 11.9)["action"], "BASELINE")


class HuanClockTests(unittest.TestCase):
    def test_old_or_frozen_clock_cannot_become_fresh_through_receipt(self):
        clock = SimpleNamespace(Clock=type("Clock", (), {}))
        qos = SimpleNamespace(QoSProfile=lambda **kw: kw, QoSReliabilityPolicy=SimpleNamespace(BEST_EFFORT=1))
        with patch.dict(sys.modules, {"rosgraph_msgs": SimpleNamespace(msg=clock), "rosgraph_msgs.msg": clock,
                                      "rclpy": SimpleNamespace(qos=qos), "rclpy.qos": qos}):
            module = importlib.import_module("huan_fusion_vendor.sim_clock_feed")
        wall = [100.]
        callbacks = []
        node = SimpleNamespace(create_subscription=lambda kind, topic, cb, profile: callbacks.append(cb),
                               destroy_subscription=lambda sub: None)
        feed = module.SimulationClockFeed(node, lambda: wall[0])
        callbacks[0](SimpleNamespace(clock=SimpleNamespace(sec=10, nanosec=0)))
        self.assertTrue(feed.snapshot()[2])
        wall[0] = 100.3
        callbacks[0](SimpleNamespace(clock=SimpleNamespace(sec=10, nanosec=0)))
        self.assertFalse(feed.snapshot()[2])
        callbacks[0](SimpleNamespace(clock=SimpleNamespace(sec=9, nanosec=0)))
        self.assertEqual(feed.seconds(), 10.)
        self.assertFalse(feed.snapshot()[2])
        callbacks[0](SimpleNamespace(clock=SimpleNamespace(sec=10, nanosec=100000000)))
        self.assertTrue(feed.snapshot()[2])


if __name__ == "__main__":
    unittest.main()
