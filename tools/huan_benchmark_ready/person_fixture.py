"""An opt-in camera/LiDAR fixture. It never adds the person to the static map."""
import xml.etree.ElementTree as ET

ACTOR_NAME = "person"
PROXY_NAME = "paired_person_collision"
PROXY_RADIUS = 0.25
PROXY_HEIGHT = 1.8
PERIOD = 28.0


def crossing_y(t):
    t = float(t) % PERIOD
    if t < 6.0:
        return -0.8
    if t < 14.0:
        return -0.8 + 0.2 * (t - 6.0)
    if t < 20.0:
        return 0.8
    return 0.8 - 0.2 * (t - 20.0)


def configure(world):
    actor = world.find("actor[@name='person']")
    if actor is None or actor.find("skin") is None:
        raise ValueError("Expected the reviewed person actor with its existing skin")
    if world.find("model[@name='%s']" % PROXY_NAME) is not None:
        raise ValueError("Duplicate paired-person collision model")
    animations = actor.findall("animation")
    if not animations:
        raise ValueError("Missing person skeleton animation")
    for old in actor.findall("script"):
        actor.remove(old)
    script = ET.SubElement(actor, "script")
    ET.SubElement(script, "loop").text = "true"
    ET.SubElement(script, "auto_start").text = "true"
    trajectory = ET.SubElement(script, "trajectory", {
        "id": "0", "type": animations[0].get("name"), "tension": "1"})
    # Dense waypoints keep interpolation close to the requested piecewise-linear motion.
    for index in range(281):
        t = index / 10.0
        point = ET.SubElement(trajectory, "waypoint")
        ET.SubElement(point, "time").text = "%.1f" % t
        ET.SubElement(point, "pose").text = "3 %.6f 0 0 0 0" % crossing_y(t)
    proxy = ET.SubElement(world, "model", {"name": PROXY_NAME})
    ET.SubElement(proxy, "static").text = "true"
    ET.SubElement(proxy, "pose").text = "3 -0.8 0.9 0 0 0"
    link = ET.SubElement(proxy, "link", {"name": "body"})
    ET.SubElement(link, "gravity").text = "false"
    collision = ET.SubElement(link, "collision", {"name": "body_collision"})
    cylinder = ET.SubElement(ET.SubElement(collision, "geometry"), "cylinder")
    ET.SubElement(cylinder, "radius").text = str(PROXY_RADIUS)
    ET.SubElement(cylinder, "length").text = str(PROXY_HEIGHT)
    # No visual: RGB/depth sees the existing human mesh; the ray sensor sees this proxy.
    return {"actor": ACTOR_NAME, "collision_model": PROXY_NAME,
            "proxy_radius_m": PROXY_RADIUS, "proxy_height_m": PROXY_HEIGHT,
            "actor_motion": "x=3, y=-0.8..0.8; 0.2 m/s with 6 s end holds",
            "period_sim_s": PERIOD, "synchronization": "measured actor pose via Gazebo state service"}
