"""Scoring must reject unsafe or incomplete wiring traces."""
from copy import deepcopy
import unittest

from behavior_gate_trial import PHASES, TraceVerifier, TrialSequence


def row(phase="SLOW", action=1, v=.06, x=0.):
    return {"phase": phase, "phase_elapsed_sim_s": .5, "phase_elapsed_wall_s": 1.5,
            "guard_command": [v, 0.], "truth_xy": [x, 0.],
            "route": {"waypoints_map": [[0., 0.], [1.2, 0.]], "current_segment": 0,
                      "route_revision": 1, "finished": False,
                      "command_preview": {"linear_x": v, "angular_z": 0.},
                      "behavior_gate": {"action": action, "reason": "directive_timeout" if phase == "DROP_HEARTBEAT" else "directive_wait"}}}


class GateTrialTests(unittest.TestCase):
    def test_schedule_contains_wait_slow_loss_and_resume_before_finish(self):
        clock = TrialSequence(10., 100.)
        self.assertEqual(clock.step(10.5, 101.)[:2], ("INITIAL_WAIT", 0))
        self.assertEqual(clock.step(11., 103.)[:2], ("BASELINE", 2))
        self.assertEqual([phase for phase, duration, action in PHASES],
                         ["INITIAL_WAIT", "BASELINE", "SLOW", "WAIT", "RESUME", "DROP_HEARTBEAT", "FINISH"])

    def test_clock_rollback_and_early_arrival_fail(self):
        for now, arrival in ((9., False), (10.1, True)):
            clock = TrialSequence(10., 100.)
            with self.assertRaises(RuntimeError):
                clock.step(now, 101., arrival)

    def test_slow_command_above_cap_fails(self):
        with self.assertRaisesRegex(RuntimeError, "cap"):
            TraceVerifier([1.2, 0.]).observe(row(v=.07))

    def test_a_low_guard_command_does_not_hide_an_uncapped_fablab_command(self):
        sample = row(v=.06)
        sample["route"]["command_preview"]["linear_x"] = .12
        with self.assertRaisesRegex(RuntimeError, "before the guard"):
            TraceVerifier([1.2, 0.]).observe(sample)

    def test_nonzero_wait_command_fails(self):
        with self.assertRaisesRegex(RuntimeError, "Nonzero"):
            TraceVerifier([1.2, 0.]).observe(row(phase="WAIT", action=0, v=.03))

    def test_stopped_commands_do_not_hide_physical_motion(self):
        verifier = TraceVerifier([1.2, 0.])
        verifier.observe(row(phase="WAIT", action=0, v=0.))
        with self.assertRaisesRegex(RuntimeError, "3 cm"):
            verifier.observe(row(phase="WAIT", action=0, v=0., x=.04))

    def test_route_change_while_waiting_fails(self):
        verifier = TraceVerifier([1.2, 0.])
        sample = row(phase="WAIT", action=0, v=0.)
        verifier.observe(sample)
        changed = deepcopy(sample)
        changed["route"]["route_revision"] = 2
        with self.assertRaisesRegex(RuntimeError, "changed the route"):
            verifier.observe(changed)

    def test_changing_the_original_goal_fails(self):
        sample = row()
        sample["route"]["waypoints_map"][-1] = [2., 0.]
        with self.assertRaisesRegex(RuntimeError, "goal was lost"):
            TraceVerifier([1.2, 0.]).observe(sample)

    def test_slow_cannot_pass_if_the_robot_never_moves(self):
        verifier = TraceVerifier([1.2, 0.])
        for index in range(5):
            verifier.observe(row(v=0., x=0.))
        result = verifier.finish()
        self.assertFalse(result["checks"]["SLOW_moved"])
        self.assertFalse(result["passed"])

    def test_heartbeat_stop_must_report_the_gate_timeout(self):
        sample = row(phase="DROP_HEARTBEAT", action=0, v=0.)
        sample["route"]["behavior_gate"]["reason"] = "directive_wait"
        with self.assertRaisesRegex(RuntimeError, "attributed"):
            TraceVerifier([1.2, 0.]).observe(sample)

    def test_finishing_route_can_report_wait_at_arrival(self):
        sample = row(phase="FINISH", action=0, v=0.)
        sample["route"]["finished"] = True
        TraceVerifier([1.2, 0.]).observe(sample)

    def test_incomplete_phases_cannot_pass(self):
        verifier = TraceVerifier([1.2, 0.])
        for i in range(4):
            verifier.observe(row(x=i*.02))
        result = verifier.finish()
        self.assertTrue(result["checks"]["SLOW_moved"])
        self.assertFalse(result["passed"])

    def test_complete_safe_trace_can_pass_all_phase_checks(self):
        verifier = TraceVerifier([1.2, 0.])
        x = 0.
        for phase, duration, action in PHASES:
            expected = 0 if action is None else action
            for index in range(3):
                if expected:
                    x += .02
                verifier.observe(row(phase=phase, action=expected, v=.06 if expected else 0., x=x))
        self.assertTrue(verifier.finish()["passed"])


if __name__ == "__main__":
    unittest.main()
