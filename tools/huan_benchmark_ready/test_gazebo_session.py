"""Regression: a second master port does not isolate ROS or scene services."""
from pathlib import Path
import tempfile
import unittest

from gazebo_session import ros_domain_conflicts, simulator_environment


class GazeboSessionTests(unittest.TestCase):
    def test_child_partition_is_shared_within_run_and_differs_between_runs(self):
        inherited = {"ROS_DOMAIN_ID": "39", "GAZEBO_MASTER_URI": "http://127.0.0.1:11349",
                     "IGN_PARTITION": "old_shared_partition"}
        first = simulator_environment(inherited, Path("/runs/lien_first"))
        second = simulator_environment(inherited, Path("/runs/lien_second"))
        self.assertNotEqual(first["IGN_PARTITION"], second["IGN_PARTITION"])
        self.assertEqual(inherited["IGN_PARTITION"], "old_shared_partition")
        self.assertEqual(first["ROS_DOMAIN_ID"], "39")
        self.assertEqual(first["GAZEBO_MASTER_URI"], inherited["GAZEBO_MASTER_URI"])

    def test_other_master_same_ros_domain_is_rejected_without_counting_wrappers(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            entries = [
                (4392, ["sh", "-c", "gzserver old.world"], "39"),
                (4394, ["gzserver", "old.world"], "39"),
                (6000, ["/usr/bin/gzserver", "unrelated.world"], "40"),
                (8198, ["gzclient"], "39"),
            ]
            for pid, arguments, domain in entries:
                process = root / str(pid)
                process.mkdir()
                process.joinpath("cmdline").write_bytes(("\0".join(arguments) + "\0").encode())
                process.joinpath("environ").write_bytes(
                    ("ROS_DOMAIN_ID=" + domain + "\0GAZEBO_MASTER_URI=http://127.0.0.1:11345\0").encode())
            conflicts = ros_domain_conflicts({"ROS_DOMAIN_ID": "39"}, root)
            self.assertEqual([p["pid"] for p in conflicts], [4394])
            self.assertEqual(conflicts[0]["gazebo_master_uri"], "http://127.0.0.1:11345")

    def test_exited_process_and_default_domain_do_not_block_domain_39(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            (root / "100").mkdir()
            (root / "101").mkdir()
            (root / "101/cmdline").write_bytes(b"gzserver\0default.world\0")
            (root / "101/environ").write_bytes(b"DISPLAY=:0\0")
            self.assertEqual(ros_domain_conflicts({"ROS_DOMAIN_ID": "39"}, root), [])
            self.assertEqual([p["pid"] for p in ros_domain_conflicts({}, root)], [101])


if __name__ == "__main__":
    unittest.main()
