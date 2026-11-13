#!/usr/bin/env python3
"""Move only the test person's collision proxy. No navigation or wheel publishers."""
import json
import math
import sys
import time

from person_fixture import ACTOR_NAME, PROXY_HEIGHT, PROXY_NAME, PROXY_RADIUS


class FixtureRequests:
    """Keep at most one request outstanding, including after a soft timeout."""
    def __init__(self):
        self.pending = None
        self.sent_wall = 0.0
        self.sent_sim = 0.0
        self.last_xy = None
        self.responses = 0
        self.error = None

    def submit(self, future, wall, sim, xy):
        if self.pending is not None:
            raise RuntimeError("A fixture request is already pending")
        self.pending = future
        self.sent_wall, self.sent_sim = wall, sim
        self.last_xy = tuple(xy)

    def poll(self, wall):
        if self.pending is None:
            return
        if self.pending.done():
            try:
                response = self.pending.result()
                if response is None or not response.success:
                    self.error = "Gazebo rejected fixture update"
                else:
                    self.responses += 1
                    self.error = None
            except Exception as error:
                self.error = "Fixture update response failed: " + str(error)
            self.pending = None
        elif wall-self.sent_wall > 2.0:
            # Do not cancel/reissue: an older request could still move the proxy later.
            # Report unavailable and retain the pending future until its real response.
            self.error = "Fixture update service timed out; waiting for pending response"

    def needs_update(self, xy, sim):
        if self.pending is not None:
            return False
        return (self.error is not None or self.last_xy is None or self.responses < 3
                or sim-self.sent_sim >= 1.0
                or math.hypot(xy[0]-self.last_xy[0], xy[1]-self.last_xy[1]) > .002)

    def ready(self, fresh, error_m):
        return (fresh and self.error is None and self.responses >= 3
                and error_m is not None and math.isfinite(error_m) and error_m <= .05)


def main():
    import rclpy
    from gazebo_msgs.msg import ModelStates
    from gazebo_msgs.srv import SetEntityState
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import String
    rclpy.init()
    node = rclpy.create_node("paired_person_fixture")
    node.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
    client = node.create_client(SetEntityState, "/gazebo/set_entity_state")
    publisher = node.create_publisher(String, "/test_fixture/person_lidar_status", 1)
    state = {"actor": None, "proxy": None, "received": 0.0, "model_error": None,
             "last_reported_error": None}
    moves = FixtureRequests()

    def receive(message):
        for name, key in ((ACTOR_NAME, "actor"), (PROXY_NAME, "proxy")):
            if name not in message.name:
                state["model_error"] = "Missing model: " + name
                return
            p = message.pose[message.name.index(name)].position
            if not all(math.isfinite(v) for v in (p.x, p.y, p.z)):
                state["model_error"] = "Non-finite fixture pose"
                return
            state[key] = (p.x, p.y, p.z)
        state["received"] = time.monotonic()

    subscription = node.create_subscription(ModelStates, "/gazebo/model_states", receive,
                                            qos_profile_sensor_data)

    def update():
        now = time.monotonic()
        sim = node.get_clock().now().nanoseconds / 1e9
        moves.poll(now)
        fresh = bool(state["received"]) and now - state["received"] <= 0.6
        if (fresh and not state["model_error"] and client.service_is_ready()
                and moves.needs_update(state["actor"][:2], sim)):
            request = SetEntityState.Request()
            request.state.name = PROXY_NAME
            request.state.reference_frame = "world"
            request.state.pose.position.x, request.state.pose.position.y = state["actor"][:2]
            request.state.pose.position.z = PROXY_HEIGHT / 2.0
            request.state.pose.orientation.w = 1.0
            try:
                future = client.call_async(request)
                moves.submit(future, now, sim, state["actor"][:2])
            except Exception as error:
                moves.error = "Could not send fixture update: " + str(error)
        error_m = None
        if state["actor"] is not None and state["proxy"] is not None:
            error_m = math.hypot(state["actor"][0] - state["proxy"][0],
                                 state["actor"][1] - state["proxy"][1])
        error = state["model_error"] or moves.error
        ready = moves.ready(fresh and not state["model_error"], error_m)
        if error != state["last_reported_error"]:
            print(("PAIR NOT READY: " + error) if error else
                  "PAIR RESPONSE RECOVERED: checking measured alignment", flush=True)
            state["last_reported_error"] = error
        publisher.publish(String(data=json.dumps({
            "stamp_sim": node.get_clock().now().nanoseconds / 1e9,
            "ready": ready, "actor": ACTOR_NAME, "collision_model": PROXY_NAME,
            "actor_xy_world": state["actor"][:2] if state["actor"] else None,
            "proxy_xy_world": state["proxy"][:2] if state["proxy"] else None,
            "coupling_error_m": error_m, "proxy_radius_m": PROXY_RADIUS,
            "responses": moves.responses, "error": error,
            "request_pending": moves.pending is not None,
            "pending_age_wall_s": now-moves.sent_wall if moves.pending is not None else None})))
        if state["model_error"]:
            raise RuntimeError(state["model_error"])

    # 10 Hz simulation time is enough for the 0.2 m/s fixture. Skip near-identical
    # targets after the initial handshake instead of pausing Gazebo for every tick.
    timer = node.create_timer(0.1, update)
    try:
        print("PAIR FIXTURE: person visual + collision-only cylinder; no robot commands", flush=True)
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
    except Exception as error:
        print("STOP: " + str(error), file=sys.stderr, flush=True)
        sys.exit(1)
