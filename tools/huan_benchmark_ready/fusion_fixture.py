"""Stage a Gazebo person trajectory whose truth is used only for scoring."""
from copy import deepcopy
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

TOOLSET = "huan-fablab-benchmark-9-standing-close-pass"
MODE = "fusion_yield_crossing_v2"
HOLD = 20.0
SPEED = .25
START_Y = -2.2
GOAL_AFTER_EPOCH = 7.0
GOAL_LATE_LIMIT = .15
MOTION_Y_DELTA = .15
MOTION_TIMING_TOLERANCE = .60
CENTER_TIMING_TOLERANCE = .50
ACTOR = "person"
PROXY = "paired_person_collision"
HEAD_ON_SCENARIOS = ("head-on-wide", "head-on-narrow")
OCCLUDED_SCENARIO = "occluded-crossing"
STANDING_SCENARIO = "standing-blocker"
STANDING_CLOSE_PASS_GAP_M = 1.00
STANDING_CENTER_GAPS_M = (STANDING_CLOSE_PASS_GAP_M, 1.50, 1.60, 1.65)
SCENARIOS = ("crossing",) + HEAD_ON_SCENARIOS + (OCCLUDED_SCENARIO, STANDING_SCENARIO)
HEAD_ON_START_X = 4.8
HEAD_ON_TURN_X = 2.8
HEAD_ON_EXIT_Y = 2.2
HEAD_ON_CLEAR_Y = 1.8
HEAD_ON_TURN_TIME = HOLD+(HEAD_ON_START_X-HEAD_ON_TURN_X)/SPEED
NARROW_CORRIDOR_WIDTH = 1.8
NARROW_DOOR_WIDTH = 1.2
# Match the sensor gate's 0.30 m path margin with 0.02 m extra for the wall model.
NARROW_DETOUR_WALL_MARGIN_M = .32
OCCLUDED_CROSS_X = 3.45
OCCLUDED_HOLD = 25.0
OCCLUSION_SCREEN = (2.84, -2.90, 3.04, -.95)
STANDING_BLOCK_X = 3.2
STANDING_CENTER_ENTRY_Y = -.10
STANDING_GOAL_X = 5.4


def crossing_y(elapsed, hold=HOLD, end_y=2.2):
    return min(end_y, START_Y+SPEED*max(0., elapsed-hold))


def head_on_pose(elapsed):
    if elapsed <= HEAD_ON_TURN_TIME:
        return HEAD_ON_START_X-SPEED*max(0., elapsed-HOLD), 0., math.pi
    return HEAD_ON_TURN_X, min(HEAD_ON_EXIT_Y, SPEED*(elapsed-HEAD_ON_TURN_TIME)), math.pi/2.


def configure_narrow_corridor(world):
    """Move the mission walls inward and leave a side exit at the actor turn."""
    wall_thickness = .20
    wall_offset = (NARROW_CORRIDOR_WIDTH+wall_thickness)/2.
    door_start = HEAD_ON_TURN_X-NARROW_DOOR_WIDTH/2.
    door_end = HEAD_ON_TURN_X+NARROW_DOOR_WIDTH/2.
    for side, spans in ((-1, [(-2., 6.)]), (1, [(-2., door_start), (door_end, 6.)])):
        model = world.find("model[@name='mission_wall_%d']" % side)
        if (model is None or len(model.findall("link")) != 1 or model.find("pose") is None
                or model.find("link/pose") is not None):
            raise ValueError("Expected an original mission wall to build the narrow corridor")
        template = model.find("link")
        model.remove(template)
        model.find("pose").text = "0 %g 1.25 0 0 0" % (side*wall_offset)
        for index, (start, end) in enumerate(spans):
            link = deepcopy(template)
            link.set("name", "wall_%d" % index)
            link_pose = ET.Element("pose")
            link_pose.text = "%g 0 0 0 0 0" % ((start+end)/2.)
            link.insert(0, link_pose)
            for element in ("collision", "visual"):
                link.find(element+"/geometry/box/size").text = "%g %g 2.5" % (end-start, wall_thickness)
            model.append(link)


def configure_occlusion_screen(world):
    xmin, ymin, xmax, ymax = OCCLUSION_SCREEN
    model = ET.SubElement(world, "model", name="occlusion_screen")
    ET.SubElement(model, "static").text = "true"
    ET.SubElement(model, "pose").text = "%g %g 1.25 0 0 0" % ((xmin+xmax)/2., (ymin+ymax)/2.)
    link = ET.SubElement(model, "link", name="wall")
    for tag in ("collision", "visual"):
        element = ET.SubElement(link, tag, name=tag)
        box = ET.SubElement(ET.SubElement(element, "geometry"), "box")
        ET.SubElement(box, "size").text = "%g %g 2.5" % (xmax-xmin, ymax-ymin)
    ET.SubElement(ET.SubElement(element, "material"), "diffuse").text = ".55 .60 .65 1"


class CrossingTiming:
    """Time the Gazebo fixture; these observations never enter motion policy."""

    def __init__(self, scenario="crossing"):
        self.head_on = scenario in HEAD_ON_SCENARIOS
        self.occluded = scenario == OCCLUDED_SCENARIO
        self.standing = scenario == STANDING_SCENARIO
        self.crossing_x = {OCCLUDED_SCENARIO: OCCLUDED_CROSS_X,
                           STANDING_SCENARIO: STANDING_BLOCK_X}.get(scenario, 3.)
        self.hold = OCCLUDED_HOLD if self.occluded else HOLD
        self.center_y = STANDING_CENTER_ENTRY_Y if self.standing else 0.
        self.epoch = self.motion = self.center = self.goal = None
        self.edge = self.exit = None

    @property
    def goal_target(self):
        return None if self.epoch is None else self.epoch+GOAL_AFTER_EPOCH

    def at_start(self, person_xy):
        x, y = person_xy
        if self.head_on:
            return abs(x-HEAD_ON_START_X) <= .15 and abs(y) <= .15
        return abs(x-self.crossing_x) <= .05 and abs(y-START_Y) <= .15

    def observe(self, sim, person_xy):
        x, y = person_xy
        if not all(math.isfinite(v) for v in (sim, x, y)) or sim <= 0:
            return
        if self.epoch is None:
            # The walking animation moves the observed root slightly even during the hold.
            if self.at_start(person_xy):
                self.epoch = sim
            return
        if sim < self.epoch:
            return
        if self.head_on:
            moved = x <= HEAD_ON_START_X-MOTION_Y_DELTA
            reached_milestone = abs(x-HEAD_ON_TURN_X) <= .15 and y >= HEAD_ON_CLEAR_Y
        elif abs(x-self.crossing_x) <= .05:
            moved = y >= START_Y+MOTION_Y_DELTA
            reached_milestone = y >= self.center_y
            if self.occluded:
                if self.edge is None and y >= OCCLUSION_SCREEN[3]:
                    self.edge = sim
                if self.exit is None and y >= HEAD_ON_CLEAR_Y:
                    self.exit = sim
        else:
            return
        if self.motion is None and moved:
            self.motion = sim
        if self.center is None and reached_milestone:
            self.center = sim

    def record_goal(self, sim):
        target = self.goal_target
        if (self.goal is not None or target is None or not math.isfinite(sim)
                or not target <= sim <= target+GOAL_LATE_LIMIT):
            raise RuntimeError("Missed the synchronized goal slot; start a fresh Gazebo session")
        if self.motion is not None:
            raise RuntimeError("The actor moved before the goal slot")
        self.goal = sim

    def report(self):
        expected_motion = self.hold+MOTION_Y_DELTA/SPEED-GOAL_AFTER_EPOCH
        expected_center = (HEAD_ON_TURN_TIME+HEAD_ON_CLEAR_Y/SPEED if self.head_on
                           else self.hold+(self.center_y-START_Y)/SPEED)-GOAL_AFTER_EPOCH
        motion_delay = None if self.goal is None or self.motion is None else self.motion-self.goal
        center_delay = None if self.goal is None or self.center is None else self.center-self.goal
        report = dict(
            actor_epoch_sim_s=self.epoch, goal_target_sim_s=self.goal_target, goal_publish_sim_s=self.goal,
            actor_motion_threshold_m=MOTION_Y_DELTA, actor_motion_sim_s=self.motion,
            motion_after_goal_sim_s=motion_delay, expected_motion_after_goal_sim_s=expected_motion,
            motion_tolerance_sim_s=MOTION_TIMING_TOLERANCE,
            valid=(motion_delay is not None and center_delay is not None
                   and abs(motion_delay-expected_motion) <= MOTION_TIMING_TOLERANCE
                   and abs(center_delay-expected_center) <= CENTER_TIMING_TOLERANCE),
        )
        if self.head_on:
            report.update(actor_clearance_sim_s=self.center, actor_clearance_after_goal_sim_s=center_delay,
                          expected_actor_clearance_after_goal_sim_s=expected_center,
                          clearance_tolerance_sim_s=CENTER_TIMING_TOLERANCE)
        elif self.standing:
            report.update(blocker_center_entry_sim_s=self.center,
                          center_entry_after_goal_sim_s=center_delay,
                          expected_center_entry_after_goal_sim_s=expected_center,
                          center_tolerance_sim_s=CENTER_TIMING_TOLERANCE)
        else:
            report.update(center_crossing_sim_s=self.center, center_after_goal_sim_s=center_delay,
                          expected_center_after_goal_sim_s=expected_center,
                          center_tolerance_sim_s=CENTER_TIMING_TOLERANCE)
        if self.occluded:
            report.update(actor_screen_edge_sim_s=self.edge, actor_clearance_sim_s=self.exit)
            report["valid"] = report["valid"] and self.edge is not None and self.exit is not None
        return report


def configure(world, output, scenario="crossing"):
    if scenario not in SCENARIOS:
        raise ValueError("Unknown person scenario: " + scenario)
    head_on = scenario in HEAD_ON_SCENARIOS
    occluded = scenario == OCCLUDED_SCENARIO
    standing = scenario == STANDING_SCENARIO
    hold = OCCLUDED_HOLD if occluded else HOLD
    end_y = 0. if standing else 2.2
    actor = world.find("actor[@name='person']")
    if actor is None or actor.find("skin") is None or not actor.findall("animation"):
        raise ValueError("Missing original human skin/animation")
    if world.find("model[@name='%s']" % PROXY) is not None:
        raise ValueError("Duplicate crossing fixture")
    actor = deepcopy(actor)
    for old in actor.findall("script"):
        actor.remove(old)
    actor.find("pose").text = "0 0 0 0 0 0"
    script = ET.SubElement(actor, "script")
    ET.SubElement(script, "loop").text = "false"
    ET.SubElement(script, "auto_start").text = "true"
    trajectory = ET.SubElement(script, "trajectory", {
        "id": "0", "type": actor.find("animation").get("name"), "tension": "1"})
    if head_on:
        start_x, start_y, _ = head_on_pose(0.)
        motion_end = HEAD_ON_TURN_TIME+HEAD_ON_EXIT_Y/SPEED
    else:
        start_x = {OCCLUDED_SCENARIO: OCCLUDED_CROSS_X, STANDING_SCENARIO: STANDING_BLOCK_X}.get(scenario, 3.)
        start_y = START_Y
        motion_end = hold+(end_y-START_Y)/SPEED
    # Include the end of motion explicitly before its final hold.
    for t in [i/10. for i in range(round(10*motion_end)+1)] + [400.]:
        point = ET.SubElement(trajectory, "waypoint")
        ET.SubElement(point, "time").text = "%.1f" % t
        if head_on:
            x, y, yaw = head_on_pose(t)
            pose = "%.6f %.6f 0 0 0 %.6f" % (x, y, yaw)
        else:
            pose = "%g %.6f 0 0 0 0" % (start_x, crossing_y(t, hold, end_y))
        ET.SubElement(point, "pose").text = pose
    root = ET.Element("sdf", version="1.6")
    root.append(actor)
    actor_path = Path(output)/"fusion_actor.sdf"
    ET.ElementTree(root).write(str(actor_path), encoding="utf-8", xml_declaration=True)
    root = ET.Element("sdf", version="1.6")
    model = ET.SubElement(root, "model", name=PROXY)
    ET.SubElement(model, "static").text = "true"
    ET.SubElement(model, "pose").text = "%g %g .9 0 0 0" % (start_x, start_y)
    link = ET.SubElement(model, "link", name="body")
    ET.SubElement(link, "gravity").text = "false"
    collision = ET.SubElement(link, "collision", name="body_collision")
    cylinder = ET.SubElement(ET.SubElement(collision, "geometry"), "cylinder")
    ET.SubElement(cylinder, "radius").text = ".25"
    ET.SubElement(cylinder, "length").text = "1.8"
    proxy_path = Path(output)/"fusion_proxy.sdf"
    ET.ElementTree(root).write(str(proxy_path), encoding="utf-8", xml_declaration=True)
    world.remove(world.find("actor[@name='person']"))
    config = {"source": "lien_test_startup", "mode": MODE, "scenario": scenario,
            "actor_sdf": str(actor_path), "proxy_sdf": str(proxy_path),
            "goal_after_actor_epoch_sim_s": GOAL_AFTER_EPOCH,
            "proxy_radius_m": .25, "robot_radius_m": .52,
            "goal_map": [STANDING_GOAL_X if standing else 4.5, 0.], "repeat_requires_new_startup": True}
    if head_on:
        config.update(hold_before_approach_sim_s=HOLD, person_speed_mps=SPEED,
                      person_start_xy_m=[start_x, start_y], person_turn_xy_m=[HEAD_ON_TURN_X, 0.],
                      person_exit_xy_m=[HEAD_ON_TURN_X, HEAD_ON_EXIT_Y])
    else:
        config.update(hold_before_cross_sim_s=hold, crossing_speed_mps=SPEED,
                      crossing_x_m=start_x, crossing_y_m=[START_Y, end_y])
    if standing:
        config["standing_xy_m"] = [start_x, end_y]
    if occluded:
        config["occlusion_screen_xy_bounds_m"] = list(OCCLUSION_SCREEN)
    if scenario == "head-on-narrow":
        config.update(corridor_inner_width_m=NARROW_CORRIDOR_WIDTH,
                      side_exit_width_m=NARROW_DOOR_WIDTH, side_exit_center_x_m=HEAD_ON_TURN_X,
                      detour_wall_margin_m=NARROW_DETOUR_WALL_MARGIN_M)
    return config


def factory_arguments(path):
    # Gazebo ROS Foxy's SpawnEntity accepts model/light, not actor. The native
    # Gazebo factory supports actors. One publish only; the trial checks names.
    # World::ProcessFactoryMsgs resolves sdf_filename through ModelDatabase,
    # which expects a model manifest, not a standalone generated .sdf file.
    # Use Factory.sdf so both model and actor take the readString branch.
    sdf = Path(path).read_text(encoding="utf-8")
    root = ET.fromstring(sdf)
    entities = [child for child in root if child.tag in ("actor", "model")]
    if root.tag != "sdf" or len(entities) != 1:
        raise ValueError("Expected one model or actor in the factory SDF")
    return ["gz", "topic", "-w", "default", "-p", "/gazebo/default/factory", "-m",
            "sdf: " + json.dumps(sdf, ensure_ascii=False) + " allow_renaming: false"]


def insertion_report(name, returncode, model_names, state_age, log_tail):
    present = name in model_names
    return {"name": name, "transport": "gazebo_factory_inline_sdf_v2",
            "process_returncode": returncode, "process_running": returncode is None,
            "observed_in_model_states": present, "model_state_age_wall_s": state_age,
            "observed_model_names": sorted(model_names), "publisher_log_tail": log_tail,
            "confirmed": present and returncode == 0 and state_age <= .8}
