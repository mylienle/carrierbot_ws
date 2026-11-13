#!/usr/bin/env python3
"""Compare pose inference on one captured RGB-D pair. Does not drive the robot."""
import json
from pathlib import Path
import sys
import tempfile
import time

import cv2
import numpy as np
import torch
from ultralytics import YOLO
import yaml


def main():
    vision = Path.home() / "carrierbot-vm.q09FUG/Robot_Vision_Training"
    sys.path.insert(0, str(vision))
    from src.ros_capture import ROSCapture, json_value
    from src.torso_depth import PoseTorsoDetector, associate_poses, bbox_iou, measure_torso_depth

    config = yaml.safe_load((vision / "config.vm.cpu.yaml").read_text(encoding="utf-8"))
    settings = dict(config["input"], max_frames=1)
    cap = ROSCapture(settings)
    try:
        ok, image = cap.read()
        if not ok:
            raise RuntimeError("No synchronized RGB-D frame")
        depth = cap.get_depth().copy()
        stamp = cap.get_timestamp()
        intrinsics = cap.get_intrinsics()
    finally:
        cap.release()

    root = Path.home() / "carrierbot_test_runs"
    root.mkdir(exist_ok=True)
    output = Path(tempfile.mkdtemp(prefix=time.strftime("pose_diagnosis_%Y%m%d_%H%M%S_"), dir=str(root)))
    cv2.imwrite(str(output / "rgb.png"), image)
    np.save(str(output / "depth.npy"), depth)
    print("DA CHUP MOT CAP RGB-D. LOG: " + str(output), flush=True)

    torch.set_num_threads(2)
    cv2.setNumThreads(1)
    detector = YOLO(str(vision / config["detection"]["model"]))
    detected = detector.predict(image, device="cpu", half=False, imgsz=416,
                                conf=config["detection"]["confidence"], classes=[0], verbose=False)[0]
    tracks = []
    if detected.boxes is not None:
        for index, box in enumerate(detected.boxes.xyxy.cpu().numpy()):
            tracks.append({"id": index, "bbox": box.tolist()})
    position = config["position_estimation"]
    pose = PoseTorsoDetector(str(vision / position["pose_model"]), position.get("pose_confidence", 0.35))
    pose.model.task = "pose"
    pose.model.overrides["task"] = "pose"
    report = {"stage": "offline_pose_diagnosis", "timestamp": stamp,
              "image_shape": list(image.shape), "depth_shape": list(depth.shape),
              "intrinsics": intrinsics, "detected_people": len(tracks), "detections": tracks,
              "note": "One frozen RGB-D pair for pose diagnosis. This does not evaluate live freshness or velocity.",
              "pose_confidence": pose.confidence,
              "min_keypoint_confidence": position.get("min_keypoint_conf", 0.5), "cases": []}

    def evaluate(label, poses, seconds):
        matches = associate_poses(tracks, poses, min_iou=position.get("min_match_iou", 0.3))
        measurements = []
        for track_index, pose_index in matches.items():
            candidate = poses[pose_index]
            measured = measure_torso_depth(
                depth, candidate["xy"], candidate["conf"],
                min_keypoint_conf=position.get("min_keypoint_conf", 0.5),
                min_depth_m=position.get("min_depth_m", 0.3),
                max_depth_m=position.get("max_depth_m", 8.0),
                min_valid_ratio=position.get("min_valid_ratio", 0.45),
                max_depth_mad_m=position.get("max_depth_mad_m", 0.2),
                allow_profile_fallback=position.get("allow_profile_fallback", True))
            measurements.append({"detection_index": track_index, "pose_index": pose_index,
                "pose_status": measured.status, "depth_m": measured.depth_m,
                "valid_ratio": measured.valid_ratio, "depth_mad_m": measured.depth_mad_m,
                "torso_keypoint_confidences": candidate["conf"][[5, 6, 11, 12]].tolist()})
        row = {"mode": label, "inference_wall_s": seconds, "pose_candidates": len(poses),
               "matched_people": len(matches),
               "iou_matrix": [[bbox_iou(t["bbox"], p["bbox"]) for p in poses] for t in tracks],
               "measurements": measurements}
        report["cases"].append(row)
        print(json.dumps(json_value(row), indent=2), flush=True)

    for size in (416, 640):
        pose.model.overrides.update(device="cpu", half=False, imgsz=size)
        started = time.monotonic()
        poses = pose.detect(image)
        evaluate("full_frame_%d" % size, poses, time.monotonic() - started)

    # Diagnostic ROI pass, using detector boxes from the same frozen image.
    # Offsets are restored before invoking Huan's existing association and depth functions.
    pose.model.overrides.update(device="cpu", half=False, imgsz=416)
    roi_poses = []
    started = time.monotonic()
    for track in tracks[:2]:
        x1, y1, x2, y2 = track["bbox"]
        pad_x, pad_y = (x2 - x1) * 0.2, (y2 - y1) * 0.2
        left, top = max(0, int(x1 - pad_x)), max(0, int(y1 - pad_y))
        right, bottom = min(image.shape[1], int(x2 + pad_x) + 1), min(image.shape[0], int(y2 + pad_y) + 1)
        if right <= left or bottom <= top:
            continue
        for candidate in pose.detect(image[top:bottom, left:right]):
            shifted = dict(candidate)
            shifted["bbox"] = candidate["bbox"] + np.array([left, top, left, top])
            shifted["xy"] = candidate["xy"] + np.array([left, top])
            roi_poses.append(shifted)
    evaluate("person_roi_416_first_two_detections", roi_poses, time.monotonic() - started)
    (output / "comparison.json").write_text(json.dumps(json_value(report), indent=2), encoding="utf-8")
    print("KET QUA: " + str(output / "comparison.json"), flush=True)
    print("ANH DA CHUP: " + str(output / "rgb.png"), flush=True)


if __name__ == "__main__":
    main()
