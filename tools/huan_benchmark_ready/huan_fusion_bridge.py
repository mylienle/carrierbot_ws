"""Measured data boundary for Huan fusion. No ROS actions, commands, or truth."""
import hashlib
import json
from pathlib import Path

from huan_fusion_vendor.base_handoff import clusters
from huan_fusion_vendor.dynamic_obstacles import assess
from huan_fusion_vendor.handoff import Handoff
from huan_fusion_vendor.publication_guard import current_packet
from huan_fusion_vendor.vision_contract import number, vector


def verify_vendor():
    root = Path(__file__).with_name("huan_fusion_vendor")
    manifest = json.loads((root / "manifest.json").read_text(encoding="utf-8"))
    for name, item in manifest["files"].items():
        if hashlib.sha256((root / name).read_bytes()).hexdigest() != item["vendored_sha256"]:
            raise RuntimeError("Huan source fingerprint mismatch: " + name)
        restored = (root / name).read_text(encoding="utf-8")
        for module in manifest["files"]:
            sibling = Path(module).stem
            restored = restored.replace("from ."+sibling+" import ", "from "+sibling+" import ")
        if hashlib.sha256(restored.encode("utf-8")).hexdigest() != item["upstream_normalized_sha256"]:
            raise RuntimeError("Unexpected algorithm change in vendored Huan module: " + name)
    return manifest


def measured_scan(ranges, angle_min, angle_increment, range_min, range_max,
                  pose, sensor_stamp, receipt_wall):
    """The caller supplies an exact-time transform; receipt is never renewed."""
    return dict(stamp=sensor_stamp, wall=receipt_wall, sensor_pose=list(pose),
                query_stamp=sensor_stamp, transform_stamp=sensor_stamp,
                clusters=clusters(ranges, angle_min, angle_increment, range_min, range_max, pose))


def fusion_risk(packet, intent, now):
    # These are the existing user's footprint/envelope, not the smaller .42m
    # default in the Jetson export. A model may add a restriction, not clear LiDAR.
    return assess(packet, intent, now, horizon=2., robot_radius=.52,
                  person_radius=.30, margin=.20, uncertainty_rate=.15)


class HuanFusionBridge:
    def __init__(self):
        self.provenance = verify_vendor()
        self.handoff = Handoff()
        self.counts = {"updates": 0, "valid_risk_updates": 0, "laser_source_updates": 0}
        self.camera_images = set()

    def update(self, packet, receipt_wall, scan, robot, intent, now, wall):
        self.counts["updates"] += 1
        if not isinstance(packet, dict) or packet.get("source") != "gazebo":
            packet, receipt_wall = None, None
        published_wall = None
        if isinstance(packet, dict) and isinstance(packet.get("performance"), dict):
            published_wall = packet["performance"].get("publish_wall_monotonic")
        if not (number(published_wall) and number(receipt_wall) and 0 <= published_wall <= receipt_wall+1e-6):
            packet, receipt_wall = None, None
        else:
            # Both processes are on this VM. Arrival of an old queued packet
            # must not renew its original publication-age permission.
            receipt_wall = published_wall
        pose = [robot.get(k) for k in ("x", "y", "yaw")] if isinstance(robot, dict) else robot
        if not vector(pose, 3):
            pose = None
        if (not isinstance(scan, dict) or not number(scan.get("stamp")) or
                not all(number(scan.get(k)) and abs(scan[k]-scan["stamp"]) <= .001
                        for k in ("query_stamp", "transform_stamp"))):
            scan = None
        fused = self.handoff.build(packet, receipt_wall, scan, pose, now, wall)
        fused = current_packet(fused, receipt_wall, now, wall)
        risk = fusion_risk(fused, intent, now)
        sources = [t["source"] for t in fused["tracks"]] if fused else []
        if risk["valid"]:
            self.counts["valid_risk_updates"] += 1
            if any(s.startswith("laser_scan_tf") for s in sources):
                self.counts["laser_source_updates"] += 1
            if "vision_rgbd_tf" in sources:
                self.camera_images.add(fused["timestamp"])
        return dict(packet=fused, assessment=risk, sources=sources,
                    diagnostic=dict(self.handoff.last_diagnostic),
                    status="FUSION_VALID" if risk["valid"] else "FUSION_UNAVAILABLE",
                    camera_image_stamp=packet.get("timestamp") if isinstance(packet, dict) else None,
                    camera_age_sim_s=now-packet["timestamp"] if isinstance(packet, dict) and number(packet.get("timestamp")) else None,
                    camera_publish_age_wall_s=wall-published_wall if number(published_wall) else None,
                    scan_stamp=scan.get("stamp") if isinstance(scan, dict) else None)

    def summary(self):
        return dict(self.counts, valid_camera_images=len(self.camera_images),
                    source_version="huan-sensor-fusion-20261106")
