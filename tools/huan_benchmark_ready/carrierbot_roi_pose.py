"""Apply Huan's pose detector to person ROIs and return full-image coordinates."""
import math

from src.torso_depth import PoseTorsoDetector as FullFramePoseDetector


def padded_roi(bbox, width, height):
    try:
        x1, y1, x2, y2 = [float(value) for value in bbox]
    except (TypeError, ValueError):
        return None
    if not all(math.isfinite(value) for value in (x1, y1, x2, y2)):
        return None
    if x2 <= x1 or y2 <= y1:
        return None
    pad_x, pad_y = (x2 - x1) * 0.2, (y2 - y1) * 0.2
    left, top = max(0, int(x1 - pad_x)), max(0, int(y1 - pad_y))
    right, bottom = min(width, int(x2 + pad_x) + 1), min(height, int(y2 + pad_y) + 1)
    if right <= left or bottom <= top:
        return None
    return left, top, right, bottom


class PoseTorsoDetector(FullFramePoseDetector):
    def detect(self, frame, tracks):
        poses = []
        for track in tracks:
            region = padded_roi(track.get("bbox"), frame.shape[1], frame.shape[0])
            if region is None:
                continue
            left, top, right, bottom = region
            for candidate in super().detect(frame[top:bottom, left:right]):
                shifted = dict(candidate)
                shifted["bbox"] = candidate["bbox"] + (left, top, left, top)
                shifted["xy"] = candidate["xy"] + (left, top)
                poses.append(shifted)
        return poses
