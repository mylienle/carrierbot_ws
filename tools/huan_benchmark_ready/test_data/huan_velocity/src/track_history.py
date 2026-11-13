"""Luu lich su pixel, XYZ camera va (neu co) XYZ odom cho tung ID.

Chua uoc luong van toc hoac noi ID bi dut do occlusion.
Moi record la mot observation da duoc YOLO/BoT-SORT nhin thay.
"""

from collections import defaultdict, deque
import math


def _valid_xyz(xyz, require_positive_z=True):
    """Tra ve [X, Y, Z] (m) neu XYZ hop le, nguoc lai tra ve None."""
    if xyz is None:
        return None

    try:
        if len(xyz) != 3:
            return None
        values = [float(value) for value in xyz]
        if not all(math.isfinite(value) for value in values):
            return None
        if require_positive_z and values[2] <= 0.0:
            return None
        return values
    except (TypeError, ValueError, OverflowError):
        return None


class TrackHistory:
    def __init__(self, max_length=30):
        self.max_length = max_length
        self.history = defaultdict(
            lambda: deque(maxlen=self.max_length)
        )

    def update(self, tracks):
        for track in tracks:
            track_id = track["id"]
            xyz = _valid_xyz(track.get("xyz_camera_m"))
            xyz_odom = _valid_xyz(track.get("xyz_odom_m"), require_positive_z=False)

            # Luu ban sao, tranh thay doi du lieu khi track duoc update sau do.
            point = {
                "center_px": list(track["center_px"]),
                "timestamp": float(track["timestamp"]),
                "xyz_camera_m": xyz,
                "depth_valid": xyz is not None,
                "xyz_odom_m": xyz_odom,
                "odom_valid": xyz_odom is not None,
                "odom_frame_id": track.get("odom_frame_id", "none"),
                "odom_source": track.get("odom_source", "disabled"),
            }

            self.history[track_id].append(point)

    def get_history(self, track_id):
        # Vẫn tra ve danh sach record co 'center_px' de main.py ve trail.
        return list(self.history[track_id])
