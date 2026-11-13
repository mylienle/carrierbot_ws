"""Sensor-only planar association for the controlled cylindrical person fixture."""
import math


def scan_clusters(ranges, angle_min, angle_increment, range_min, range_max, gap=0.15):
    clusters = []
    current = []
    for index, distance in enumerate(ranges):
        if not math.isfinite(distance) or not range_min <= distance <= range_max:
            if current:
                clusters.append(current)
            current = []
            continue
        angle = angle_min + index * angle_increment
        point = (distance * math.cos(angle), distance * math.sin(angle))
        if current and math.hypot(point[0] - current[-1][0], point[1] - current[-1][1]) >= gap:
            clusters.append(current)
            current = []
        current.append(point)
    if current:
        clusters.append(current)
    coverage = abs(angle_increment) * max(0, len(ranges) - 1)
    if len(clusters) > 1 and coverage >= 2 * math.pi - 0.05:
        first, last = clusters[0], clusters[-1]
        if (math.isfinite(ranges[0]) and math.isfinite(ranges[-1]) and
                math.hypot(first[0][0] - last[-1][0], first[0][1] - last[-1][1]) < gap):
            clusters = [last + first] + clusters[1:-1]
    return clusters


def fit_circle(points):
    if len(points) < 6:
        return None
    mx = sum(p[0] for p in points) / len(points)
    my = sum(p[1] for p in points) / len(points)
    shifted = [(x - mx, y - my) for x, y in points]
    xx = sum(x * x for x, y in shifted)
    yy = sum(y * y for x, y in shifted)
    xy = sum(x * y for x, y in shifted)
    det = xx * yy - xy * xy
    if det <= 1e-9 * max(xx * yy, 1e-12):
        return None
    bx = sum(x * (x * x + y * y) for x, y in shifted) / 2
    by = sum(y * (x * x + y * y) for x, y in shifted) / 2
    cx, cy = (bx * yy - by * xy) / det, (by * xx - bx * xy) / det
    radius = math.sqrt((xx + yy) / len(points) + cx * cx + cy * cy)
    center = (mx + cx, my + cy)
    rms = math.sqrt(sum((math.hypot(x - center[0], y - center[1]) - radius) ** 2
                        for x, y in points) / len(points))
    if not all(math.isfinite(v) for v in (*center, radius, rms)):
        return None
    return {"center": list(center), "radius_m": radius, "fit_rms_m": rms,
            "points": len(points)}


def transform_xy(point, translation, quaternion):
    x, y, z, w = quaternion
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    if not math.isfinite(norm) or norm < 1e-9:
        raise ValueError("Invalid TF rotation")
    x, y, z, w = (value / norm for value in quaternion)
    px, py = point
    return [translation[0] + (1 - 2*(y*y + z*z))*px + 2*(x*y - z*w)*py,
            translation[1] + 2*(x*y + z*w)*px + (1 - 2*(x*x + z*z))*py]


def associate(clusters, position, translation, quaternion, gate=0.4):
    candidates = []
    for cluster in clusters:
        circle = fit_circle(cluster)
        # This is a fixture check, not a general person-shape estimator.
        if circle is None or not 0.15 <= circle["radius_m"] <= 0.35 or circle["fit_rms_m"] > 0.035:
            continue
        circle["center_odom_m"] = transform_xy(circle["center"], translation, quaternion)
        center = circle["center_odom_m"]
        circle["center_difference_m"] = math.hypot(center[0] - position[0], center[1] - position[1])
        if circle["center_difference_m"] <= gate:
            candidates.append(circle)
    if len(candidates) != 1:
        return None, "AMBIGUOUS_CLUSTER" if candidates else "NO_MATCHING_CLUSTER"
    return candidates[0], "MATCHED"
