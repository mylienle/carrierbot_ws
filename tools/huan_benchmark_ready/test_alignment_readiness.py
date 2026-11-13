"""Diagnosis must identify lost conditions without changing the acceptance gates."""
import unittest

from sensor_alignment_observer import AlignmentAvailability, readiness_report


class ReadinessTests(unittest.TestCase):
    def report(self, **changes):
        values = dict(status={"ready": True, "coupling_error_m": .01},
                      status_age=.1, scan_age=.1, wheel_age=.1, motion_seen=False,
                      input_publishers=[], wheel_publishers=["/carrierbot_safety_guard"])
        values.update(changes)
        return readiness_report(**values)

    def test_stationary_guarded_fixture_is_ready(self):
        self.assertTrue(self.report()["ready"])

    def test_stale_scan_reports_the_scan_instead_of_camera_failure(self):
        result = self.report(scan_age=.75)
        self.assertFalse(result["ready"])
        self.assertEqual(result["blockers"], ["SCAN_STALE_OR_MISSING"])

    def test_motion_or_an_extra_publisher_remains_rejected(self):
        result = self.report(motion_seen=True, input_publishers=["/motion_trial"],
                             wheel_publishers=["/carrierbot_safety_guard", "/teleop"])
        self.assertFalse(result["ready"])
        self.assertEqual(set(result["blockers"]), {"NONZERO_OR_INVALID_WHEEL_COMMAND_SEEN",
                         "GUARD_INPUT_HAS_PUBLISHER", "UNEXPECTED_WHEEL_PUBLISHERS"})

    def test_missing_fixture_and_wheel_messages_are_both_reported(self):
        result = self.report(status=None, status_age=None, wheel_age=None)
        self.assertEqual(set(result["blockers"]), {"NO_FIXTURE_STATUS", "WHEEL_COMMAND_STALE_OR_MISSING"})

    def test_transient_gap_resumes_only_after_fresh_data(self):
        monitor = AlignmentAvailability(pause_limit=5)
        self.assertEqual(monitor.update(self.report(), 10), "ready")
        stale = self.report(scan_age=.8)
        self.assertFalse(stale["ready"])
        self.assertEqual(monitor.update(stale, 11), "pause")
        self.assertEqual(monitor.update(stale, 12), "pause")
        self.assertEqual(monitor.update(self.report(), 13), "ready")
        self.assertEqual([r["event"] for r in monitor.events], ["paused", "resumed"])
        self.assertEqual(monitor.events[-1]["gap_wall_s"], 2)

    def test_persistent_missing_data_aborts(self):
        monitor = AlignmentAvailability(pause_limit=5)
        missing = self.report(status=None, scan_age=None)
        self.assertEqual(monitor.update(missing, 10), "pause")
        self.assertEqual(monitor.update(missing, 15), "abort")
        self.assertEqual(monitor.events[-1]["event"], "abort_gap_timeout")

    def test_nonzero_command_or_wrong_route_aborts_without_grace(self):
        for change in ({"motion_seen": True}, {"input_publishers": ["/teleop"]},
                       {"wheel_publishers": []}):
            monitor = AlignmentAvailability(pause_limit=5)
            self.assertEqual(monitor.update(self.report(**change), 10), "abort")
            self.assertEqual(monitor.events[-1]["event"], "abort_route_or_motion")


if __name__ == "__main__":
    unittest.main()
