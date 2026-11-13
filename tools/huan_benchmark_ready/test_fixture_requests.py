"""A slow RPC must not crash the fixture or accept an unaligned proxy."""
import types
import unittest

from pair_person_lidar import FixtureRequests


class Future:
    def __init__(self):
        self.completed = False
        self.success = True
        self.error = None

    def done(self):
        return self.completed

    def result(self):
        if self.error is not None:
            raise self.error
        return types.SimpleNamespace(success=self.success)


def warmed_up():
    moves = FixtureRequests()
    for i in range(3):
        future = Future()
        moves.submit(future, i*.1, i*.1, (3., 0.))
        future.completed = True
        moves.poll(i*.1+.01)
    return moves


class FixtureRequestTests(unittest.TestCase):
    def test_timeout_retains_pending_request_and_is_not_ready(self):
        moves = warmed_up()
        future = Future()
        moves.submit(future, 1., 1., (3., .1))
        moves.poll(3.1)
        self.assertIs(moves.pending, future)
        self.assertIn("timed out", moves.error)
        self.assertFalse(moves.ready(True, .001))
        self.assertFalse(moves.needs_update((3., .6), 3.1))

    def test_late_success_still_requires_measured_alignment(self):
        moves = warmed_up()
        future = Future()
        moves.submit(future, 1., 1., (3., .1))
        moves.poll(3.1)
        future.completed = True
        moves.poll(3.2)
        self.assertIsNone(moves.error)
        self.assertIsNone(moves.pending)
        self.assertFalse(moves.ready(True, .12))
        self.assertTrue(moves.ready(True, .004))

    def test_duplicate_pending_request_is_refused(self):
        moves = FixtureRequests()
        first = Future()
        moves.submit(first, 0., 0., (3., 0.))
        with self.assertRaises(RuntimeError):
            moves.submit(Future(), 1., 1., (3., .2))
        self.assertIs(moves.pending, first)

    def test_stationary_target_is_skipped_after_handshake(self):
        moves = warmed_up()
        self.assertFalse(moves.needs_update((3., .001), .3))
        self.assertTrue(moves.needs_update((3., 0.), 1.3))
        self.assertTrue(moves.needs_update((3., .02), .3))

    def test_rejected_response_is_reported_and_can_retry(self):
        moves = FixtureRequests()
        future = Future()
        moves.submit(future, 0., 0., (3., 0.))
        future.completed, future.success = True, False
        moves.poll(.1)
        self.assertIn("rejected", moves.error)
        self.assertIsNone(moves.pending)
        self.assertTrue(moves.needs_update((3., 0.), .2))

    def test_rpc_response_exception_does_not_escape_poll(self):
        moves = FixtureRequests()
        future = Future()
        moves.submit(future, 0., 0., (3., 0.))
        future.completed, future.error = True, RuntimeError("Transport error")
        moves.poll(.1)
        self.assertIn("Transport error", moves.error)
        self.assertIsNone(moves.pending)

    def test_stale_or_invalid_geometry_cannot_be_ready(self):
        moves = warmed_up()
        self.assertFalse(moves.ready(False, .001))
        self.assertFalse(moves.ready(True, .051))
        self.assertFalse(moves.ready(True, float('nan')))


if __name__ == "__main__":
    unittest.main()
