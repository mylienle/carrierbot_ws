"""Sensor decision regressions and test scoring, with no ROS dependency."""
from copy import deepcopy
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

from fablab_route import route_candidate
from fusion_fixture import (CrossingTiming, configure, configure_narrow_corridor, configure_occlusion_screen, crossing_y,
                            factory_arguments, insertion_report)
from fusion_yield_policy import FusionYieldPolicy, raw_route_corridor, release_intent
from fusion_yield_trial import TrialScore
from huan_trial_scoring import encounter_score, observe_occlusion, observe_standing_blocker
from test_fablab_route import fixture
from test_dynamic_decision_policy import sample


def intent(now=10., revision=1):
    snapshot, robot, transform = fixture()
    snapshot.update(stamp=now, pose_tf_stamp=now, route_revision=revision)
    robot["stamp"] = transform["stamp"] = now
    return release_intent(route_candidate(snapshot, robot, now, transform))


def raw(now=10., blocked=False):
    return {"valid": True, "blocked": blocked, "stamp": now}


def person(now=10., position=(1., 1.), velocity=None):
    values = sample(position=position, velocity=velocity, stamp=now, now=now)
    return {"packet": values["packet"], "track": values["track"], "position_ok": True,
            "velocity_ok": velocity is not None,
            "match": {"matched": True, "center_odom_m": list(position), "radius_m": .25}}


def evaluate(policy, t=10., observations=None, blocked=False, revision=1):
    return policy.evaluate(t, intent(t, revision), raw(t, blocked), observations or [])


class PolicyTests(unittest.TestCase):
    def test_raw_corridor_blocks_even_without_vision(self):
        self.assertEqual(evaluate(FusionYieldPolicy(), blocked=True)["reason"], "LIDAR_CORRIDOR_BLOCKED")

    def test_release_requires_continuous_clear_observations(self):
        policy = FusionYieldPolicy()
        self.assertEqual(evaluate(policy)["action"], "WAIT")
        self.assertEqual(evaluate(policy, 10.4)["action"], "WAIT")
        self.assertEqual(evaluate(policy, 10.6)["action"], "BASELINE")
        self.assertEqual(evaluate(policy, 10.7, blocked=True)["action"], "WAIT")
        self.assertEqual(evaluate(policy, 11.)["action"], "WAIT")
        self.assertEqual(evaluate(policy, 11.6)["action"], "BASELINE")

    def fast_resume_tick(self, policy, now, scan, *, revision=1, blocked=False):
        return policy.evaluate(now, intent(now, revision),
                               dict(raw(scan, blocked), exact_sensor_time_tf=True), [])

    def test_brief_gap_stops_then_waits_for_two_new_distinct_scans(self):
        policy = FusionYieldPolicy(resume_fresh_scans=True)
        self.fast_resume_tick(policy, 10., 9.98)
        self.assertEqual(self.fast_resume_tick(policy, 10.6, 10.55)["action"], "BASELINE")
        self.assertEqual(policy.evaluate(10.66, None, None, [], transient_gap=True)["action"], "WAIT")
        for now, scan in ((10.72, 10.63), (10.8, 10.74), (10.87, 10.74)):
            self.assertEqual(self.fast_resume_tick(policy, now, scan)["action"], "WAIT")
        result = self.fast_resume_tick(policy, 10.95, 10.84)
        self.assertEqual(result["action"], "BASELINE")
        self.assertEqual(result["resume_confirmation"], "TWO_NEW_CLEAR_EXACT_TF_SCANS")
        self.assertEqual(self.fast_resume_tick(policy, 11., 10.94)["action"], "BASELINE")

    def test_long_gap_route_change_and_blocker_each_require_standard_confirmation(self):
        for name in ("long_gap", "route_change", "blocker", "other_error", "interrupted_recovery"):
            with self.subTest(name=name):
                policy = FusionYieldPolicy(resume_fresh_scans=True)
                self.fast_resume_tick(policy, 10., 9.98)
                self.fast_resume_tick(policy, 10.6, 10.55)
                policy.evaluate(10.66, None, None, [], transient_gap=name != "other_error")
                revision = 2 if name == "route_change" else 1
                if name in ("blocker", "interrupted_recovery"):
                    self.fast_resume_tick(policy, 10.77, 10.71, blocked=name == "blocker")
                if name == "interrupted_recovery":
                    policy.evaluate(10.81, None, None, [], transient_gap=True)
                start = 10.88 if name == "long_gap" else 10.84
                for offset in (0., .11, .25, .49):
                    now = start+offset
                    self.assertEqual(self.fast_resume_tick(policy, now, now-.04,
                                                           revision=revision)["action"], "WAIT")
                self.assertEqual(self.fast_resume_tick(policy, start+.51, start+.45,
                                                       revision=revision)["action"], "BASELINE")

    def test_stale_or_invalid_lidar_never_releases(self):
        for scan in ({"valid": False}, raw(9.), raw(11.), {"valid": True, "blocked": False}):
            result = FusionYieldPolicy().evaluate(10., intent(), scan, [])
            self.assertEqual(result["reason"], "LIDAR_UNAVAILABLE")

    def test_unknown_velocity_near_route_slows_without_becoming_zero_velocity(self):
        policy = FusionYieldPolicy()
        evaluate(policy)
        row = person(10.6)
        original = deepcopy(row)
        result = evaluate(policy, 10.6, [row])
        self.assertEqual(result["action"], "SLOW")
        self.assertEqual(result["vision_observations"][0]["velocity_status"], "UNKNOWN")
        self.assertEqual(row, original)

    def test_nearby_slow_limits_caution_without_disabling_far_prediction(self):
        def tick(policy, now, observations):
            straight = dict(intent(now), route_corridor_xy_m=[[.9, 0.], [3.9, 0.]])
            return policy.evaluate(now, straight, raw(now), observations)

        standard = FusionYieldPolicy()
        nearby = FusionYieldPolicy(nearby_slow=True)
        for policy in (standard, nearby):
            tick(policy, 10., [])
        far = person(10.6, (4.5, 0.))
        self.assertEqual(tick(standard, 10.6, [far])["action"], "SLOW")
        self.assertEqual(tick(nearby, 10.6, [far])["action"], "BASELINE")
        self.assertEqual(tick(nearby, 10.7, [person(10.7, (3.5, 0.))])["action"], "SLOW")
        with patch("fusion_yield_policy.assess", return_value={"valid": True, "risk": "POTENTIAL_COLLISION"}):
            result = tick(nearby, 10.8, [person(10.8, (4.5, 0.), (-2., 0.))])
        self.assertEqual(result["reason"], "VISION_PREDICTED_CONFLICT")

    def test_vision_cannot_clear_a_lidar_blocker(self):
        policy = FusionYieldPolicy()
        evaluate(policy)
        self.assertEqual(evaluate(policy, 10.6, [person(10.6, (5., 5.), (1., 0.))], True)["action"], "WAIT")

    def test_stale_or_unmatched_vision_uses_explicit_lidar_fallback(self):
        for row in (person(9.), dict(person(10.6), match={"matched": False})):
            policy = FusionYieldPolicy()
            evaluate(policy)
            result = evaluate(policy, 10.6, [row])
            self.assertEqual(result["action"], "BASELINE")
            self.assertIn("VISION_UNAVAILABLE", result["reason"])

    def test_all_people_are_aggregated_not_last_track_wins(self):
        for rows in ([person(10.6), person(10.6, (5., 5.))],
                     [person(10.6, (5., 5.)), person(10.6)]):
            policy = FusionYieldPolicy()
            evaluate(policy)
            self.assertEqual(evaluate(policy, 10.6, rows)["action"], "SLOW")

    def test_valid_huan_prediction_can_only_restrict(self):
        policy = FusionYieldPolicy()
        evaluate(policy)
        with patch("fusion_yield_policy.assess", return_value={"valid": True, "risk": "POTENTIAL_COLLISION"}):
            result = evaluate(policy, 10.6, [person(10.6, (2., 1.), (0., -.5))])
        self.assertEqual(result["reason"], "VISION_PREDICTED_CONFLICT")

    def test_route_revision_change_resets_release_confirmation(self):
        policy = FusionYieldPolicy()
        evaluate(policy)
        self.assertEqual(evaluate(policy, 10.6)["action"], "BASELINE")
        self.assertEqual(evaluate(policy, 10.7, revision=2)["action"], "WAIT")

    def test_real_huan_assessor_receives_the_release_intent(self):
        policy = FusionYieldPolicy()
        evaluate(policy)
        observation = person(10.6, (2., 1.), (0., -.5))
        original = deepcopy(observation)
        result = evaluate(policy, 10.6, [observation])
        self.assertEqual(result["reason"], "VISION_PREDICTED_CONFLICT")
        self.assertTrue(result["vision_observations"][0]["assessment"]["valid"])
        self.assertEqual(observation, original)

    def test_counterfactual_release_intent_does_not_override_intrinsic_stop(self):
        snapshot, robot, transform = fixture()
        snapshot.update(avoidance_stop=True, route_revision=1)
        original = deepcopy(snapshot)
        held = route_candidate(snapshot, robot, 10., transform)
        proposed = release_intent(held)
        self.assertTrue(proposed["guidance_hold"])
        self.assertEqual(held["positions"][0]["xy_m"], held["positions"][-1]["xy_m"])
        self.assertNotEqual(proposed["positions"][0]["xy_m"], proposed["positions"][-1]["xy_m"])
        self.assertEqual(snapshot, original)

    def test_standing_detour_prediction_uses_the_applied_speed_cap(self):
        snapshot, robot, transform = fixture()
        snapshot.update(route_revision=1, standing_detour_active=True,
                        linear_speed_mps=.12, linear_speed_max_mps=.12)
        intent = release_intent(route_candidate(snapshot, robot, 10., transform))
        self.assertEqual(intent["speed_assumption_mps"], .06)
        points = [point["xy_m"] for point in intent["positions"]]
        self.assertAlmostEqual(sum(math.dist(a, b) for a, b in zip(points, points[1:])), .12)
        snapshot["standing_detour_speed_limited"] = False
        rejoin = release_intent(route_candidate(snapshot, robot, 10., transform))
        self.assertEqual(rejoin["speed_assumption_mps"], .12)
        snapshot["standing_detour_speed_cap_mps"] = .08
        post_pass = release_intent(route_candidate(snapshot, robot, 10., transform))
        self.assertEqual(post_pass["speed_assumption_mps"], .08)
        points = [point["xy_m"] for point in post_pass["positions"]]
        self.assertAlmostEqual(sum(math.dist(a, b) for a, b in zip(points, points[1:])), .16)
        snapshot["standing_detour_speed_cap_mps"] = .12
        with self.assertRaisesRegex(ValueError, "INVALID_STANDING_DETOUR_SPEED_CAP"):
            release_intent(route_candidate(snapshot, robot, 10., transform))

    def test_backward_clock_or_missing_route_holds(self):
        policy = FusionYieldPolicy()
        evaluate(policy)
        self.assertEqual(evaluate(policy, 9.)["action"], "WAIT")
        self.assertEqual(policy.evaluate(10., None, raw(), [])["action"], "WAIT")

    def test_forward_point_corridor_ignores_behind_and_side_wall(self):
        corridor = [[0., 0.], [3., 0.]]
        clear = [[-1., 0.]]*3+[[2., 2.]]*3
        self.assertFalse(raw_route_corridor(clear, [0., 0.], corridor, 60)["blocked"])
        self.assertTrue(raw_route_corridor(clear+[[2., .7]]*3, [0., 0.], corridor, 60)["blocked"])
        self.assertFalse(raw_route_corridor(clear, [0., 0.], corridor, 10)["valid"])

    def test_point_corridor_follows_a_corner(self):
        data = raw_route_corridor([[1.1, 1.5]]*3, [0., 0.], [[0., 0.], [1., 0.], [1., 2.]], 60)
        self.assertTrue(data["blocked"])


class FixtureTests(unittest.TestCase):
    def test_one_crossing_stays_outside_after_crossing(self):
        self.assertAlmostEqual(crossing_y(0.), -2.2)
        self.assertAlmostEqual(crossing_y(20.), -2.2)
        self.assertAlmostEqual(crossing_y(28.8), 0.)
        self.assertAlmostEqual(crossing_y(37.6), 2.2)
        self.assertAlmostEqual(crossing_y(400.), 2.2)

    def test_goal_slot_uses_first_observed_start_pose_and_ignores_standing_sway(self):
        timing = CrossingTiming()
        timing.observe(9.9, [0., 0.])
        self.assertIsNone(timing.goal_target)
        timing.observe(10., [3., -2.30])
        timing.observe(12., [3., -2.13])
        self.assertEqual(timing.goal_target, 17.)
        self.assertIsNone(timing.motion)
        with self.assertRaises(RuntimeError):
            timing.record_goal(16.99)
        timing.record_goal(17.1)
        timing.observe(30.6, [3., -2.04])
        timing.observe(38.8, [3., 0.])
        self.assertTrue(timing.report()["valid"])
        self.assertAlmostEqual(timing.report()["center_after_goal_sim_s"], 21.7)

    def test_late_goal_and_unaligned_crossing_are_separate_failures(self):
        timing = CrossingTiming()
        timing.observe(10., [3., -2.2])
        with self.assertRaisesRegex(RuntimeError, "Missed"):
            timing.record_goal(17.16)
        self.assertIsNone(timing.goal)
        timing.record_goal(17.)
        timing.observe(30.6, [3., -2.04])
        timing.observe(39.4, [3., 0.])
        self.assertFalse(timing.report()["valid"])
        self.assertAlmostEqual(timing.report()["center_after_goal_sim_s"], 22.4)

    def test_stage_assets_without_a_hidden_actor_in_startup_world(self):
        world = ET.fromstring('<world name="default"><actor name="person"><pose>0 0 0 0 0 0</pose>'
            '<skin><filename>unchanged.dae</filename></skin><animation name="standing"/><script/></actor></world>')
        with tempfile.TemporaryDirectory() as directory:
            config = configure(world, Path(directory))
            actor = ET.parse(config["actor_sdf"]).find("actor")
            proxy = ET.parse(config["proxy_sdf"]).find("model")
            self.assertIsNone(world.find("actor"))
            self.assertEqual(actor.findtext("skin/filename"), "unchanged.dae")
            self.assertEqual(actor.findtext("script/loop"), "false")
            self.assertEqual(proxy.findtext("link/collision/geometry/cylinder/radius"), ".25")
            self.assertIsNone(proxy.find("link/visual"))
            self.assertEqual(config["goal_map"], [4.5, 0.])

    def test_head_on_approaches_on_axis_then_leaves_to_the_side(self):
        world = ET.fromstring('<world><actor name="person"><pose>0 0 0 0 0 0</pose>'
                              '<skin/><animation name="walk"/></actor></world>')
        with tempfile.TemporaryDirectory() as directory:
            config = configure(world, Path(directory), "head-on-wide")
            actor = ET.parse(config["actor_sdf"]).find("actor")
            proxy = ET.parse(config["proxy_sdf"]).find("model")
            poses = {float(point.findtext("time")): list(map(float, point.findtext("pose").split()))
                     for point in actor.findall("script/trajectory/waypoint")}
            self.assertEqual(config["scenario"], "head-on-wide")
            self.assertEqual(poses[0.][:2], [4.8, 0.])
            self.assertEqual(poses[20.], poses[0.])
            self.assertEqual(poses[24.][:2], [3.8, 0.])
            self.assertEqual(poses[28.][:2], [2.8, 0.])
            self.assertEqual(poses[32.][:2], [2.8, 1.])
            self.assertEqual(poses[400.][:2], [2.8, 2.2])
            self.assertEqual(list(map(float, proxy.findtext("pose").split()))[:2], poses[0.][:2])

    def test_head_on_timing_requires_motion_and_a_lateral_exit_at_the_turn(self):
        for scenario in ("head-on-wide", "head-on-narrow"):
            with self.subTest(scenario=scenario):
                timing = CrossingTiming(scenario)
                timing.observe(9.9, [3., -2.2])
                self.assertIsNone(timing.goal_target)
                timing.observe(10., [4.8, 0.])
                timing.record_goal(17.)
                timing.observe(30.6, [4.649, 0.])
                timing.observe(45.2, [4.8, 1.8])
                self.assertFalse(timing.report()["valid"])
                timing.observe(45.2, [2.8, 1.8])
                self.assertTrue(timing.report()["valid"])
                self.assertAlmostEqual(timing.report()["actor_clearance_after_goal_sim_s"], 28.2)

    def test_narrow_corridor_collision_and_visual_leave_the_same_exit(self):
        box = "<geometry><box><size>8 .2 2.5</size></box></geometry>"
        world = ET.Element("world")
        for side in (-1, 1):
            model = ET.SubElement(world, "model", name="mission_wall_%d" % side)
            ET.SubElement(model, "pose").text = "2 %g 1.25 0 0 0" % (3.1*side)
            model.append(ET.fromstring('<link name="wall"><collision name="collision">'+box+
                '</collision><visual name="visual">'+box+'</visual></link>'))
        configure_narrow_corridor(world)
        for side, model in zip((-1, 1), world.findall("model")):
            y = float(model.findtext("pose").split()[1])
            self.assertAlmostEqual(y, float(side))
            spans = []
            for link in model.findall("link"):
                self.assertEqual(link.findtext("collision/geometry/box/size"),
                                 link.findtext("visual/geometry/box/size"))
                length, width, _ = map(float, link.findtext("collision/geometry/box/size").split())
                self.assertAlmostEqual(abs(y)-width/2, .9)
                x = float(link.findtext("pose").split()[0])
                spans.append((round(x-length/2, 4), round(x+length/2, 4)))
            self.assertEqual(spans, [(-2., 6.)] if side < 0 else [(-2., 2.2), (3.4, 6.)])

    def test_occlusion_screen_leaves_robot_route_clear_and_actor_starts_behind_it(self):
        world = ET.fromstring('<world><actor name="person"><pose>0 0 0 0 0 0</pose>'
                              '<skin/><animation name="stand"/></actor></world>')
        configure_occlusion_screen(world)
        screen = world.find("model[@name='occlusion_screen']")
        self.assertEqual(screen.findtext("link/collision/geometry/box/size"),
                         screen.findtext("link/visual/geometry/box/size"))
        with tempfile.TemporaryDirectory() as tmp:
            config = configure(world, Path(tmp), "occluded-crossing")
            xmin, ymin, xmax, ymax = config["occlusion_screen_xy_bounds_m"]
            self.assertLess(ymax, -(.52+.30))
            self.assertGreater(config["crossing_x_m"]-config["proxy_radius_m"], xmax)
            self.assertLess(ymin, config["crossing_y_m"][0])
            points = ET.parse(config["actor_sdf"]).findall("actor/script/trajectory/waypoint")
            poses = {float(p.findtext("time")): tuple(map(float, p.findtext("pose").split()[:2])) for p in points}
            self.assertEqual(poses[0.], (3.45, -2.2))
            self.assertEqual(poses[25.], poses[0.])
            self.assertEqual(poses[30.], (3.45, -.95))
            self.assertEqual(poses[400.], (3.45, 2.2))
        timing = CrossingTiming("occluded-crossing")
        timing.observe(10., [3.45, -2.2])
        timing.record_goal(17.)
        timing.observe(35.61, [3.45, -2.049])
        timing.observe(40., [3.45, -.95])
        timing.observe(43.8, [3.45, 0.])
        self.assertFalse(timing.report()["valid"])
        timing.observe(51., [3.45, 1.8])
        self.assertTrue(timing.report()["valid"])
        self.assertEqual(timing.report()["actor_screen_edge_sim_s"], 40.)

    def test_standing_blocker_enters_the_lane_then_holds_throughout_trial(self):
        world = ET.fromstring('<world><actor name="person"><pose>0 0 0 0 0 0</pose>'
                              '<skin/><animation name="stand"/></actor></world>')
        with tempfile.TemporaryDirectory() as tmp:
            config = configure(world, Path(tmp), "standing-blocker")
            self.assertEqual(config["standing_xy_m"], [3.2, 0.])
            self.assertEqual(config["goal_map"], [5.4, 0.])
            points = ET.parse(config["actor_sdf"]).findall("actor/script/trajectory/waypoint")
            poses = {float(p.findtext("time")): tuple(map(float, p.findtext("pose").split()[:2])) for p in points}
            self.assertEqual(poses[0.], (3.2, -2.2))
            self.assertEqual(poses[28.8], (3.2, 0.))
            self.assertEqual(poses[400.], (3.2, 0.))
            self.assertTrue(all(y <= 0. for _, y in poses.values()))
        timing = CrossingTiming("standing-blocker")
        timing.observe(10., [3.2, -2.2])
        timing.record_goal(17.)
        timing.observe(30.6, [3.2, -2.04])
        timing.observe(38.4, [3.2, -.10])
        self.assertTrue(timing.report()["valid"])
        self.assertAlmostEqual(timing.report()["center_entry_after_goal_sim_s"], 21.4)

    def test_native_factory_sends_xml_not_a_standalone_model_database_path(self):
        import json
        for tag in ("actor", "model"):
            with tempfile.TemporaryDirectory() as directory:
                path = Path(directory)/"a standalone file.sdf"
                sdf = '<sdf version="1.6">\n<%s name="fixture"/>\n</sdf>' % tag
                path.write_text(sdf, encoding="utf-8")
                args = factory_arguments(path)
                self.assertEqual(args[:3], ["gz", "topic", "-w"])
                self.assertTrue(args[-1].startswith('sdf: '))
                self.assertNotIn("sdf_filename:", args[-1])
                value = args[-1][len('sdf: '):-len(' allow_renaming: false')]
                self.assertEqual(json.loads(value), sdf)

    def test_factory_rejects_missing_invalid_and_ambiguous_sdf(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/"invalid.sdf"
            with self.assertRaises(FileNotFoundError):
                factory_arguments(path)
            for source in ('<sdf>', '<sdf><model/><actor/></sdf>', '<model/>'):
                path.write_text(source, encoding="utf-8")
                with self.assertRaises((ValueError, ET.ParseError)):
                    factory_arguments(path)

    def test_process_success_without_a_model_is_not_confirmation(self):
        self.assertFalse(insertion_report("proxy", 0, ["carrierbot"], .1, "")["confirmed"])
        self.assertFalse(insertion_report("proxy", None, ["proxy"], .1, "")["confirmed"])
        self.assertFalse(insertion_report("proxy", 0, ["proxy"], 1.5, "")["confirmed"])
        self.assertTrue(insertion_report("proxy", 0, ["proxy"], .1, "")["confirmed"])


class ScoreTests(unittest.TestCase):
    def test_head_on_baseline_allows_wait_and_resume_without_a_detour(self):
        score = TrialScore()
        score.minimum_clearance = .3
        metrics = score.finish([0., 0.], 0, True, .1, 1)
        self.assertTrue(encounter_score(metrics)["passed"])
        self.assertFalse(encounter_score(metrics, {"huan_fusion_used_during_mission": False})["passed"])
        metrics["checks"]["arrival"] = False
        self.assertFalse(encounter_score(metrics)["passed"])

    def test_occlusion_marks_only_applied_obstacle_commands(self):
        evidence = {"robot_advance_before_actor_motion_m": 0.}
        row = {"sim": 10., "scoring_only": {"robot_truth": [.5, 0.]},
               "route": {"robot_pose_map": [.5, 0., 0.]}, "matches": [],
               "decision": {"action": "WAIT", "reason": "ROUTE_INTENT_UNAVAILABLE"}, "guard_command": [0., 0.]}
        observe_occlusion(evidence, row, [0., 0.], 11., 20.)
        self.assertEqual(evidence["robot_advance_before_actor_motion_m"], .5)
        self.assertNotIn("first_obstacle_response", evidence)
        row.update(sim=12., matches=[{"position_ok": True}],
                   decision={"action": "WAIT", "reason": "LIDAR_CORRIDOR_BLOCKED"}, guard_command=[.1, 0.])
        observe_occlusion(evidence, row, [0., 0.], 11., 20.)
        self.assertNotIn("first_obstacle_response", evidence)
        row.update(sim=12.1, guard_command=[0., 0.])
        observe_occlusion(evidence, row, [0., 0.], 11., 20.)
        self.assertEqual(evidence["first_stereo_position_sim_s"], 12.)
        self.assertEqual(evidence["first_obstacle_response"]["sim"], 12.1)
        self.assertEqual(evidence["first_obstacle_wait"]["reason"], "LIDAR_CORRIDOR_BLOCKED")

    def test_standing_blocker_passage_requires_person_in_lane(self):
        evidence = {"robot_advance_before_actor_motion_m": 0., "samples_standing_in_lane": 0,
                    "samples_outside_lane_after_entry": 0}
        row = {"sim": 10., "scoring_only": {"robot_truth": [.5, 0.], "person_truth": [3.2, -1.]},
               "route": {"avoidance_state": 0}}
        observe_standing_blocker(evidence, row, [0., 0.], 11., 15.)
        self.assertEqual(evidence["robot_advance_before_actor_motion_m"], .5)
        row.update(sim=16., scoring_only={"robot_truth": [4., .9], "person_truth": [3.2, 1.]})
        observe_standing_blocker(evidence, row, [0., 0.], 11., 15.)
        self.assertNotIn("bypassed_standing_person_sim_s", evidence)
        self.assertEqual(evidence["samples_outside_lane_after_entry"], 1)
        row.update(sim=16.1, scoring_only={"robot_truth": [4., .9], "person_truth": [3.2, .08]},
                   route={"avoidance_state": 1})
        observe_standing_blocker(evidence, row, [0., 0.], 11., 15.)
        self.assertEqual(evidence["samples_standing_in_lane"], 1)
        self.assertEqual(evidence["first_detour_sim_s"], 16.1)
        self.assertEqual(evidence["bypassed_standing_person_sim_s"], 16.1)

    def test_arrival_alone_cannot_pass_without_sensor_influence(self):
        score = TrialScore()
        score.minimum_clearance = .3
        result = score.finish([0., 0.], 30, True, .1, 1)
        self.assertFalse(result["passed"])
        self.assertFalse(result["checks"]["vision_actually_limited_moving_speed"])
        self.assertFalse(result["checks"]["lidar_driven_wait_observed"])

    def test_wait_permits_only_the_requested_detour_and_keeps_drift_check(self):
        decision = {"action": "WAIT", "reason": "VISION_PREDICTED_CONFLICT", "route_revision": 1}
        route = {"route_revision": 1, "current_segment": 0, "waypoints_map": [[0., 0.], [5.4, 0.]],
                 "command_preview": {"linear_x": 0., "angular_z": 0.}}
        detour = dict(route, route_revision=2, current_segment=1, standing_detour_active=True,
                      waypoints_map=[[0., 0.], [2., -2.], [5.4, 0.]])
        for requested in (True, False):
            score = TrialScore()
            for stamp in (10., 10.5):
                score.observe(stamp, decision, route, [0., 0.], [0., 0.], [3.2, 0.])
            if requested:
                score.observe(10.6, dict(decision, standing_detour_request={"center_map": [3.2, 0.]}),
                              route, [0., 0.], [0., 0.], [3.2, 0.])
                score.observe(10.7, decision, detour, [0., 0.], [0., 0.], [3.2, 0.])
                with self.assertRaisesRegex(RuntimeError, "drifted"):
                    score.observe(10.8, decision, detour, [0., 0.], [.04, 0.], [3.2, 0.])
            else:
                with self.assertRaisesRegex(RuntimeError, "Route changed"):
                    score.observe(10.7, decision, detour, [0., 0.], [0., 0.], [3.2, 0.])

    def test_low_guard_output_does_not_hide_uncapped_upstream_command(self):
        score = TrialScore()
        decision = {"action": "SLOW", "reason": "VISION_PERSON_NEAR_REFERENCE_ROUTE", "vision_observations": []}
        route = {"command_preview": {"linear_x": .12}}
        score.observe(10., decision, route, [.06, 0.], [0., 0.], [3., 0.])
        with self.assertRaisesRegex(RuntimeError, "SLOW cap"):
            score.observe(10.5, decision, route, [.06, 0.], [0., 0.], [3., 0.])

    def test_circular_clearance_threshold_fails_even_if_no_contact(self):
        with self.assertRaisesRegex(RuntimeError, "clearance"):
            TrialScore().observe(10., {"action": "WAIT", "reason": "LIDAR_CORRIDOR_BLOCKED"}, {},
                                 [0., 0.], [0., 0.], [.9, 0.])

    def test_nonfinite_scoring_truth_cannot_be_accepted_as_clearance(self):
        with self.assertRaisesRegex(RuntimeError, "Non-finite"):
            TrialScore().observe(10., {}, {}, [0., 0.], [float("nan"), 0.], [3., 0.])


if __name__ == "__main__":
    unittest.main()
