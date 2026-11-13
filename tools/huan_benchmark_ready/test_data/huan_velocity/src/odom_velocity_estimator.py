"""Uoc luong van toc 3D trong he ODOM (m/s).

Input la cac observation XYZ, timestamp cua cung mot raw BoT-SORT ID.
Can XYZ odom da duoc chuyen doi dung timestamp tu pose/TF; khong tu xu ly ID switch.
"""

import numpy as np


class OdomVelocityEstimator:
    """Linear regression tren cua so thoi gian + median position + EMA velocity.

    - Khong noi toc do qua sample thieu odom, timestamp khong tang,
      gap thoi gian dai, hoac buoc nhay vi tri bat thuong.
    - De velocity=None khi chua du tin cay; KHONG coi None la 0 m/s.
    """

    def __init__(
        self,
        window_s=0.8,
        min_span_s=0.25,
        min_samples=4,
        max_gap_s=0.6,
        max_step_speed_mps=6.0,
        max_estimated_speed_mps=4.0,
        ema_alpha=0.4,
    ):
        self.window_s = float(window_s)
        self.min_span_s = float(min_span_s)
        self.min_samples = int(min_samples)
        self.max_gap_s = float(max_gap_s)
        self.max_step_speed_mps = float(max_step_speed_mps)
        self.max_estimated_speed_mps = float(max_estimated_speed_mps)
        self.ema_alpha = float(ema_alpha)
        self._last_filtered = {}  # id -> (t, np.array([vx, vy, vz]))

    @staticmethod
    def _xyz(record):
        xyz = record.get("xyz_odom_m")
        if xyz is None:
            return None
        try:
            values = np.asarray(xyz, dtype=float)
            if values.shape != (3,) or not np.isfinite(values).all():
                return None
        except (TypeError, ValueError, OverflowError):
            return None
        return values

    def update(self, tracks, track_history):
        """Sua track dict tai cho sau track_history.update(tracks).

        Them cac field velocity_odom_mps, speed_xy_mps, velocity_odom_valid,
        velocity_odom_status. Neu None thi khong co uoc luong dang tin.
        """
        for track in tracks:
            track_id = track["id"]
            track["velocity_odom_mps"] = None
            track["speed_xy_mps"] = None
            track["velocity_odom_valid"] = False
            track["velocity_odom_status"] = "WARMUP"

            history = track_history.get_history(track_id)
            if not history:
                continue

            t_now = float(track["timestamp"])
            if self._xyz(history[-1]) is None:
                track["velocity_odom_status"] = "NO_ODOM"
                self._last_filtered.pop(track_id, None)
                continue

            # Phat hien gap ngay ca khi mau truoc nam ngoai cua so window_s.
            if len(history) >= 2:
                previous_t = float(history[-2]["timestamp"])
                dt_latest = t_now - previous_t
                if not np.isfinite(dt_latest) or dt_latest <= 0:
                    track["velocity_odom_status"] = "BAD_TIME"
                    self._last_filtered.pop(track_id, None)
                    continue
                if dt_latest > self.max_gap_s:
                    track["velocity_odom_status"] = "TIME_GAP"
                    self._last_filtered.pop(track_id, None)
                    continue

            # Chi lay chuoi quan sat lien tuc ket thuc o frame hien tai.
            # Gap/khong depth/outlier -> khong suy dien qua vung mat du lieu.
            current_frame = track.get("odom_frame_id", "none")
            current_source = track.get("odom_source", "disabled")
            segment = []  # danh sach (timestamp, xyz), theo thu tu moi -> cu
            later_t = None
            later_xyz = None
            stopped_reason = None
            for record in reversed(history):
                if record.get("odom_frame_id", "none") != current_frame or record.get("odom_source", "disabled") != current_source:
                    stopped_reason = "FRAME_CHANGE"
                    break
                xyz = self._xyz(record)
                if xyz is None:
                    stopped_reason = "NO_ODOM_GAP"
                    break
                t = float(record["timestamp"])
                if not np.isfinite(t):
                    stopped_reason = "BAD_TIME"
                    break
                if t_now - t > self.window_s:
                    break
                if later_t is not None:
                    dt = later_t - t
                    if dt <= 0 or dt > self.max_gap_s:
                        stopped_reason = "TIME_GAP"
                        break
                    step_speed = float(np.linalg.norm(later_xyz - xyz) / dt)
                    if step_speed > self.max_step_speed_mps:
                        stopped_reason = "POSITION_JUMP"
                        break
                segment.append((t, xyz))
                later_t, later_xyz = t, xyz

            # Neu mot position jump ngay giua hai mau gan nhat, khoi dong lai.
            if len(segment) < self.min_samples:
                if stopped_reason in ("TIME_GAP", "POSITION_JUMP", "NO_ODOM_GAP", "FRAME_CHANGE"):
                    track["velocity_odom_status"] = stopped_reason
                self._last_filtered.pop(track_id, None)
                continue

            segment.reverse()  # cu -> moi
            ts = np.array([t for t, _ in segment], dtype=float)
            xyz = np.vstack([p for _, p in segment])
            if ts[-1] - ts[0] < self.min_span_s:
                self._last_filtered.pop(track_id, None)
                continue

            # Causal median qua toi da 3 observation, xu ly nhieu vi tri.
            smoothed_xyz = np.vstack([
                np.median(xyz[max(0, i - 2):i + 1], axis=0)
                for i in range(len(xyz))
            ])

            # Hoi quy XYZ = a + V*(t - t_mean) => V = covariance(t, xyz)/var(t)
            centered_t = ts - np.mean(ts)
            denom = float(centered_t @ centered_t)
            if denom <= 1e-12:
                self._last_filtered.pop(track_id, None)
                continue
            raw_v = (centered_t @ smoothed_xyz) / denom
            raw_speed = float(np.linalg.norm(raw_v))
            if raw_speed > self.max_estimated_speed_mps:
                track["velocity_odom_status"] = "SPEED_OUTLIER"
                self._last_filtered.pop(track_id, None)
                continue

            prev = self._last_filtered.get(track_id)
            if prev is not None and 0 < t_now - prev[0] <= self.max_gap_s:
                filtered_v = self.ema_alpha * raw_v + (1 - self.ema_alpha) * prev[1]
            else:
                filtered_v = raw_v

            self._last_filtered[track_id] = (t_now, filtered_v)
            vx, vy, vz = [float(v) for v in filtered_v]
            track["velocity_odom_mps"] = [vx, vy, vz]
            # Mat phang ngang odom la (X, Y); Z la chieu cao.
            track["speed_xy_mps"] = float(np.hypot(vx, vy))
            track["velocity_odom_valid"] = True
            track["velocity_odom_status"] = "OK"

        # Khong giu state filter cho ID khong thay >5 s (tranh tich luy).
        if tracks:
            t_latest = max(float(track["timestamp"]) for track in tracks)
            for key, (t_seen, _) in list(self._last_filtered.items()):
                if t_latest - t_seen > 5.0:
                    self._last_filtered.pop(key, None)
        return tracks
