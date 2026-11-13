"""Offline regression checks; synthetic rays here are not Gazebo trial results."""
import math
import random
import unittest
import xml.etree.ElementTree as ET

from person_fixture import configure, crossing_y
from sensor_pair_geometry import associate, fit_circle, scan_clusters, transform_xy


def arc(center=(3.0, 0.0), radius=0.25, noise=0.0):
    rng = random.Random(42)
    points = []
    for index in range(81):
        angle = math.pi / 2 + math.pi * index / 80
        r = radius + rng.gauss(0.0, noise)
        points.append((center[0] + r * math.cos(angle), center[1] + r * math.sin(angle)))
    return points


class GeometryTests(unittest.TestCase):
    def test_half_circle_center_with_ray_scale_noise(self):
        circle = fit_circle(arc(noise=0.01))
        self.assertLess(math.hypot(circle["center"][0] - 3, circle["center"][1]), .03)
        self.assertLess(abs(circle["radius_m"] - .25), .03)
        self.assertLess(circle["fit_rms_m"], .02)

    def test_wall_is_not_a_person_cluster(self):
        wall = [(3.0, i / 100) for i in range(-40, 41)]
        match, status = associate([wall], [3, 0], (0, 0), (0, 0, 0, 1))
        self.assertIsNone(match)
        self.assertEqual(status, "NO_MATCHING_CLUSTER")

    def test_circle_fit_is_stable_far_from_origin(self):
        circle = fit_circle(arc(center=(10000, -30000)))
        self.assertAlmostEqual(circle["center"][0], 10000, places=7)
        self.assertAlmostEqual(circle["center"][1], -30000, places=7)

    def test_backwards_laser_rotation_and_translation(self):
        match, status = associate([arc(center=(-3, 0))], [3.1, .2],
                                  (.1, .2), (0, 0, 1, 0))
        self.assertEqual(status, "MATCHED")
        self.assertLess(match["center_difference_m"], 1e-8)

    def test_non_planar_rotation_keeps_xy_projection(self):
        p = transform_xy((1, 0), (0, 0), (0, math.sin(math.pi/4), 0, math.cos(math.pi/4)))
        self.assertAlmostEqual(p[0], 0, places=7)
        self.assertAlmostEqual(p[1], 0, places=7)

    def test_two_nearby_objects_are_ambiguous(self):
        match, status = associate([arc(center=(3, -.2)), arc(center=(3, .2))],
                                  [3, 0], (0, 0), (0, 0, 0, 1))
        self.assertIsNone(match)
        self.assertEqual(status, "AMBIGUOUS_CLUSTER")

    def test_wrong_size_or_distant_object_is_rejected(self):
        for points in (arc(radius=.8), arc(center=(5, 0))):
            self.assertIsNone(associate([points], [3, 0], (0, 0), (0, 0, 0, 1))[0])

    def test_invalid_ranges_separate_clusters(self):
        clusters = scan_clusters([1, 1, float('nan'), 1, 1], 0, .01, .5, 12)
        self.assertEqual([len(c) for c in clusters], [2, 2])

    def test_full_scan_wrap_is_one_cluster(self):
        ranges = [float('inf')] * 101
        ranges[:3] = ranges[-3:] = [3.0] * 3
        clusters = scan_clusters(ranges, -math.pi, 2*math.pi/100, .5, 12, gap=.25)
        self.assertEqual([len(c) for c in clusters], [6])


class FixtureTests(unittest.TestCase):
    def world(self):
        return ET.fromstring('<world><actor name="person"><skin/><animation name="standing"/>'
                             '<script><loop>false</loop></script></actor></world>')

    def test_fixture_has_collision_without_visible_cylinder(self):
        world = self.world()
        configure(world)
        proxy = world.find("model[@name='paired_person_collision']")
        self.assertEqual(proxy.findtext("link/collision/geometry/cylinder/radius"), "0.25")
        self.assertIsNone(proxy.find("link/visual"))
        self.assertEqual(world.findtext("actor/script/loop"), "true")
        self.assertEqual(len(world.findall("actor/script")), 1)

    def test_motion_has_holds_speed_and_continuous_loop(self):
        self.assertEqual(crossing_y(0), crossing_y(6))
        self.assertEqual(crossing_y(14), crossing_y(20))
        self.assertAlmostEqual(crossing_y(10.5) - crossing_y(10), .1)
        self.assertAlmostEqual(crossing_y(25.5) - crossing_y(25), -.1)
        self.assertAlmostEqual(crossing_y(28 - 1e-6), crossing_y(28), places=6)

    def test_duplicate_fixture_is_rejected(self):
        world = self.world()
        configure(world)
        with self.assertRaises(ValueError):
            configure(world)


if __name__ == "__main__":
    unittest.main()
