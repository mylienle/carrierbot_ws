"""Exercise the real reviewed estimator against sparse, invalid and changing data."""
from copy import deepcopy
from pathlib import Path
import unittest

from huan_cpu_velocity import cpu_sparse_estimator
from replay_huan_velocity import compare, load_reviewed

ROOT = Path(__file__).resolve().parent
ORIGINAL, HISTORY = load_reviewed(ROOT/"test_data/huan_velocity", ROOT)


def track(t, x=0., xyz=True, frame="odom"):
    return dict(id=1, timestamp=t, center_px=[200., 180.],
                xyz_camera_m=[0., 0., 3.], xyz_odom_m=[x, 0., 1.] if xyz else None,
                odom_frame_id=frame, odom_source="tf2_ekf")


def feed(rows, estimator=None):
    estimator = estimator or cpu_sparse_estimator(ORIGINAL)()
    history, results = HISTORY(max_length=30), []
    for row in rows:
        rows_now = [deepcopy(row)]
        history.update(rows_now)
        estimator.update(rows_now, history)
        results.append(rows_now[0])
    return results


class CpuVelocityTests(unittest.TestCase):
    def test_four_sparse_measured_positions_produce_estimate_without_reducing_sample_count(self):
        rows = [track(i*.4, .25*i*.4) for i in range(4)]
        old = feed(rows, ORIGINAL())
        new = feed(rows)
        self.assertTrue(all(not t["velocity_odom_valid"] for t in old))
        self.assertTrue(all(not t["velocity_odom_valid"] for t in new[:3]))
        self.assertTrue(new[-1]["velocity_odom_valid"])
        # The unchanged causal median/regression has lag and underestimates this ramp.
        self.assertAlmostEqual(new[-1]["velocity_odom_mps"][0], .1625)
        for before, after in zip(rows, new):
            self.assertEqual(before["timestamp"], after["timestamp"])
            self.assertEqual(before["xyz_odom_m"], after["xyz_odom_m"])

    def test_a_gap_above_point_six_still_invalidates_velocity(self):
        result = feed([track(i*.4, .1*i) for i in range(4)]+[track(1.9, .4)])[-1]
        self.assertFalse(result["velocity_odom_valid"])
        self.assertEqual(result["velocity_odom_status"], "TIME_GAP")
        self.assertIsNone(result["velocity_odom_mps"])

    def test_missing_position_breaks_chain_and_is_never_zero_velocity(self):
        rows = [track(i*.3) for i in range(4)]+[track(1.2, xyz=False), track(1.5)]
        result = feed(rows)
        self.assertEqual(result[-2]["velocity_odom_status"], "NO_ODOM")
        self.assertEqual(result[-1]["velocity_odom_status"], "NO_ODOM_GAP")
        self.assertIsNone(result[-1]["velocity_odom_mps"])

    def test_position_jump_and_frame_change_still_rejected(self):
        rows = [track(i*.3) for i in range(4)]
        self.assertEqual(feed(rows+[track(1.2, 10.)])[-1]["velocity_odom_status"], "POSITION_JUMP")
        self.assertEqual(feed(rows+[track(1.2, frame="other")])[-1]["velocity_odom_status"], "FRAME_CHANGE")

    def test_non_increasing_time_still_rejected(self):
        rows = [track(i*.3) for i in range(4)]
        rows.append(track(rows[-1]["timestamp"]))
        self.assertEqual(feed(rows)[-1]["velocity_odom_status"], "BAD_TIME")

    def test_estimated_speed_limit_still_applies(self):
        result = feed([track(i*.18, 5.*i*.18) for i in range(8)])[-1]
        self.assertEqual(result["velocity_odom_status"], "SPEED_OUTLIER")
        self.assertFalse(result["velocity_odom_valid"])

    def test_replay_checks_original_flags_and_preserves_inputs(self):
        observed = feed([track(i*.4, .25*i*.4) for i in range(4)], ORIGINAL())
        packets = [dict(timestamp=row["timestamp"], tracks=[row]) for row in observed]
        before = deepcopy(packets)
        result = compare(packets, ORIGINAL, HISTORY)
        self.assertEqual(result["valid_velocity_tracks"], {"original": 0, "cpu-sparse": 1})
        self.assertTrue(result["original_status_flag_agreement"]["all_match"])
        self.assertFalse(result["candidate_only_for_next_live_input_test"])
        self.assertEqual(packets, before)
        packets[-1]["tracks"][0]["velocity_odom_valid"] = True
        self.assertFalse(compare(packets, ORIGINAL, HISTORY)["original_status_flag_agreement"]["all_match"])


if __name__ == "__main__":
    unittest.main()
