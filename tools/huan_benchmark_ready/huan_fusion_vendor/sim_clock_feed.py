"""Foxy/Gazebo bridge clock: last received simulation stamp, never wall time."""
import time
from rosgraph_msgs.msg import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy


class SimulationClockFeed:
    def __init__(self, node, wall_clock=time.monotonic):
        self.node = node
        self.wall_clock = wall_clock
        self._sample = (0, None)
        self._subscription = None
        self._bound_wall = None
        self.generation = 0
        self._bind()

    def _bind(self):
        if self._subscription is not None:
            self.node.destroy_subscription(self._subscription)
        # Clock samples are absolute. After a suspended reader resumes, only
        # the newest sample is useful; historical reliable delivery is not.
        self._subscription = self.node.create_subscription(
            Clock, '/clock', self._receive,
            QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT))
        self._bound_wall = self.wall_clock()
        self.generation += 1

    def _receive(self, msg):
        stamp_ns = msg.clock.sec * 1_000_000_000 + msg.clock.nanosec
        if stamp_ns > self._sample[0]:
            # Replace one immutable tuple; the independent zero-only watchdog
            # never sees a new time paired with an old receive wall time.
            self._sample = (stamp_ns, self.wall_clock())

    def snapshot(self):
        stamp_ns, receipt = self._sample
        age = None if receipt is None else self.wall_clock() - receipt
        return stamp_ns, age, age is not None and 0. <= age <= .25

    def seconds(self):
        return self._sample[0] * 1e-9

    def rebind_if_stale(self):
        _, _, fresh = self.snapshot()
        if not fresh and self.wall_clock() - self._bound_wall >= .5:
            self._bind()
            return True
        return False
