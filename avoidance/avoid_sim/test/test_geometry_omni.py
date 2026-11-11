#!/usr/bin/env python3
"""Unit tests for avoid_sim.geometry and avoid_sim.omni (no ROS master needed).

Run: python3 -m unittest discover -s avoidance/avoid_sim/test -v   (with the workspace sourced)
"""
import dataclasses
import math
import os
import unittest

import numpy as np
import rospkg
import yaml

from avoid_sim import cases
from avoid_sim.geometry import (CLASSIC_LAYOUT, ConfigSpace, Field, layout_violations, load_footprint,
                                pair_gap, sample_layout, sample_layout_route)
from avoid_sim.omni import NO_HIT, MaskRenderer, OmniModel, Scanner

RP = rospkg.RosPack()
FIELD = Field.from_yaml(os.path.join(RP.get_path('avoid_nav'), 'config', 'field.yaml'))
FOOT = load_footprint(os.path.join(RP.get_path('robot6th_description'), 'config', 'robot6th_footprint.yaml'))
CAM = yaml.safe_load(open(os.path.join(RP.get_path('avoid_nav'), 'config', 'camera.yaml')))['camera']


class GeometryTest(unittest.TestCase):
    def test_pair_gap_axis_vs_euclidean(self):
        # diagonal pair, 35.4 cm free in x and in y: 50 cm corner to corner, but the robot cannot pass
        p, q = (0.0, 0.0), (0.45 + 0.354, 0.45 + 0.354)
        self.assertAlmostEqual(pair_gap(p, q, 0.45, 'euclidean'), 0.5006, places=3)
        self.assertAlmostEqual(pair_gap(p, q, 0.45, 'axis'), 0.354, places=3)
        self.assertLess(pair_gap(p, (0.2, 0.0), 0.45, 'axis'), 0)          # overlapping

    def test_random_layouts_meet_spec_and_are_reachable(self):
        cs = ConfigSpace(FIELD, FOOT)
        for seed in range(20):
            layout, _ = sample_layout(FIELD, FOOT, np.random.default_rng(seed))
            self.assertEqual(len(layout), FIELD.count)
            self.assertEqual(layout_violations(layout, FIELD), [], 'seed %d' % seed)
            self.assertTrue(cs.reachable(layout), 'seed %d' % seed)
            ylim = FIELD.red_inner - FIELD.size / 2
            for x, y in layout:
                self.assertTrue(FIELD.obs_x_min <= x <= FIELD.obs_x_max and abs(y) <= ylim + 1e-9)

    def test_route_layouts_ignore_the_gap_spec_but_keep_a_wide_route(self):
        cs = ConfigSpace(FIELD, FOOT)
        with_violations = 0
        for seed in range(20):
            layout, _ = sample_layout_route(FIELD, FOOT, np.random.default_rng(seed))
            self.assertEqual(len(layout), FIELD.count)
            self.assertTrue(cs.route_wide(layout, FIELD.route_width), 'seed %d' % seed)
            self.assertTrue(cs.reachable(layout), 'seed %d' % seed)
            for i in range(len(layout)):                                    # touching is fine, overlapping is not
                for j in range(i + 1, len(layout)):
                    self.assertGreaterEqual(pair_gap(layout[i], layout[j], FIELD.size, 'axis', FIELD), 0.0)
            with_violations += bool(layout_violations(layout, FIELD))
        self.assertGreater(with_violations, 10)                              # boxes closer than the spec: the point

    def test_route_layouts_turn_the_boxes(self):
        self.assertTrue(FIELD.route_rotate)                                  # field.yaml default
        yaws = []
        for seed in range(20):
            layout, _ = sample_layout_route(FIELD, FOOT, np.random.default_rng(seed))
            for x, y, yaw in layout:
                yaws.append(math.degrees(yaw))
                corners = FIELD.obstacle_corners(x, y, yaw)                  # inside the lines (may touch them)
                self.assertLessEqual(np.abs(corners[:, 1]).max(), FIELD.red_inner + 1e-9, 'seed %d' % seed)
        self.assertTrue(all(0 <= a < 90 for a in yaws))
        self.assertGreater(sum(15 < a < 75 for a in yaws), len(yaws) / 2)   # mostly well turned

    def test_route_rotate_off_gives_the_old_layouts(self):
        # the first box of seed 42519 as the sampler placed it before boxes could turn
        f = dataclasses.replace(FIELD, route_rotate=False)
        layout, _ = sample_layout_route(f, FOOT, np.random.default_rng(42519))
        self.assertTrue(all(yaw == 0 for _, _, yaw in layout))
        self.assertAlmostEqual(layout[0][0], 2.0893, places=4)
        self.assertAlmostEqual(layout[0][1], -1.0018, places=4)

    def test_turned_box_occupancy_and_gap(self):
        cs = ConfigSpace(FIELD, FOOT)
        occ = cs.occupancy([(0.0, 0.0, math.pi / 4)])
        inner = np.abs(cs.y0 + np.arange(cs.ny) * cs.res) < FIELD.red_inner
        self.assertAlmostEqual(occ[inner].sum() * cs.res ** 2, FIELD.size ** 2, delta=0.02 * FIELD.size ** 2)
        e = FIELD.size / math.sqrt(2)                                        # a diamond: corners 0.318 m out
        self.assertEqual(occ[cs.cell(e - 0.01, 0.0)], 1)
        self.assertEqual(occ[cs.cell(0.2, 0.2)], 0)                          # inside the axis-aligned box only
        # its corner 5 cm from an axis-aligned box's face
        q = (e + FIELD.size / 2 + 0.05, 0.0)
        self.assertAlmostEqual(pair_gap((0.0, 0.0, math.pi / 4), q, FIELD.size, 'axis', FIELD), 0.05, places=6)
        self.assertLess(pair_gap((0.0, 0.0, math.pi / 4), (q[0] - 0.08, 0.0), FIELD.size, 'axis', FIELD), 0)

    def test_cases_store_the_yaw_of_turned_boxes(self):
        c = cases.make(1, 'route', (-2.0, 0.0, 0.0), [(0.5, 0.2, math.radians(30)), (1.0, -0.3, 0.0), (1.5, 0.0)], 'camera')
        self.assertEqual(c['obstacles'], [[0.5, 0.2, 30.0], [1.0, -0.3], [1.5, 0.0]])

    def test_route_width(self):
        # a wall of boxes across the corridor with one gap: a disc of route_width must fit through it
        cs = ConfigSpace(FIELD, FOOT)
        for gap, wide in ((0.53, True), (0.48, False)):                      # grid: conservative by ~2 cm
            layout = []
            for side in (1, -1):
                y = gap / 2 + FIELD.size / 2
                while y - FIELD.size / 2 < FIELD.red_inner:
                    layout.append((0.0, side * y))
                    y += FIELD.size
            self.assertEqual(cs.route_wide(layout, 0.50), wide, 'gap %.3f' % gap)
            self.assertTrue(cs.reachable(layout))                             # the 40 cm robot passes both

    def test_classic_layout_breaks_the_gap_spec(self):
        bad = layout_violations(CLASSIC_LAYOUT, FIELD)
        self.assertIn((0, 4), [(i, j) for i, j, _ in bad])                  # obstacles 1 and 5: 8 cm gap

    def test_cspace_uses_the_real_width(self):
        # a wall of boxes across the corridor with one gap; robot width is 0.402 m
        width = FOOT[:, 1].max() - FOOT[:, 1].min()
        cs = ConfigSpace(FIELD, FOOT)
        # the grid (1 cm) makes the check conservative by up to ~2 cm
        for gap, passable in ((width + 0.025, True), (width - 0.01, False)):
            layout = []
            for side in (1, -1):                                             # boxes from the gap to the line
                y = gap / 2 + FIELD.size / 2
                while y - FIELD.size / 2 < FIELD.red_inner:
                    layout.append((0.0, side * y))
                    y += FIELD.size
            self.assertEqual(cs.reachable(layout), passable, 'gap %.3f' % gap)


class OmniTest(unittest.TestCase):
    def setUp(self):
        self.m = OmniModel(CAM)
        no_struts = dict(CAM, strut_angles_deg=[])
        self.m0 = OmniModel(no_struts)

    def test_distance_table(self):
        # values computed from black_item's formula (docs, 2.3)
        for px, cm in ((40, 32.4), (80, 78.3), (100, 116.0), (120, 180.0), (140, 320.7)):
            self.assertAlmostEqual(float(self.m.distance_cm(px)), cm, delta=0.15)
        self.assertEqual(float(self.m.distance_cm(200)), NO_HIT)

    def scan_one(self, model, x, y, yaw, obstacles, field=FIELD, center_error=(0, 0)):
        tc = (model.cx + center_error[0], model.cy + center_error[1])
        black, red = MaskRenderer(model, field, true_center=tc).render(x, y, yaw, obstacles)
        sc = Scanner(model)
        return sc.scan(black), sc.scan(red)

    def test_obstacle_straight_ahead(self):
        # box face 1.0 m in front of the robot centre (robot well inside the corridor)
        black, _ = self.scan_one(self.m0, 0.0, 0.0, 0.0, [(1.0 + FIELD.size / 2, 0.0)])
        self.assertAlmostEqual(black[0], 100, delta=3)                      # line 0 = front
        self.assertGreater(black[60], 150)                                   # behind: nothing that close

    def test_turned_box_seen_at_its_corner(self):
        # a box turned by 45 deg points a corner at the robot: 0.318 m from its centre, not 0.225
        x = 1.0 + FIELD.size / math.sqrt(2)
        black, _ = self.scan_one(self.m0, 0.0, 0.0, 0.0, [(x, 0.0, math.pi / 4)])
        self.assertAlmostEqual(black[0], 100, delta=3)
        black, _ = self.scan_one(self.m0, 0.0, 0.0, 0.0, [(x, 0.0)])
        self.assertAlmostEqual(black[0], 100 + 100 * (FIELD.size / math.sqrt(2) - FIELD.size / 2), delta=3)

    def test_ray_squares(self):
        rng = np.random.default_rng(1)
        a = rng.uniform(-math.pi, math.pi, 500)
        ux, uy = np.cos(a), np.sin(a)
        centres = [(1.0, 0.3), (-0.7, -0.5), (0.2, 1.1)]
        t0, _ = MaskRenderer._ray_boxes(0.1, -0.05, ux, uy, [FIELD.obstacle_box(*c) for c in centres])
        t1, _ = MaskRenderer._ray_squares(0.1, -0.05, ux, uy, centres, FIELD.size / 2)
        self.assertTrue(np.allclose(np.where(np.isinf(t0), -1, t0), np.where(np.isinf(t1), -1, t1)))
        # towards (1, 0.3): past the corner of the axis-aligned box at (1, 0), into the turned one
        u = np.array([1.0]) / math.hypot(1.0, 0.3), np.array([0.3]) / math.hypot(1.0, 0.3)
        self.assertTrue(np.isinf(MaskRenderer._ray_squares(0.0, 0.0, *u, [(1.0, 0.0)], FIELD.size / 2)[0][0, 0]))
        self.assertTrue(np.isfinite(MaskRenderer._ray_squares(0.0, 0.0, *u, [(1.0, 0.0, math.pi / 4)], FIELD.size / 2)[0][0, 0]))

    def test_index_direction_is_counter_clockwise(self):
        # obstacle on the robot's left (+y): scan line 30 (= +90 deg)
        black, _ = self.scan_one(self.m0, 0.0, 0.0, 0.0, [(0.0, 0.8 + FIELD.size / 2)])
        self.assertAlmostEqual(black[30], 80, delta=3)
        self.assertGreater(black[90], 150)

    def test_inner_ring_reports_32cm_for_closer_obstacles(self):
        black, _ = self.scan_one(self.m0, 0.0, 0.0, 0.0, [(0.25 + FIELD.size / 2, 0.0)])
        self.assertAlmostEqual(black[0], int(self.m0.distance_cm(self.m0.inner)), delta=1)   # 32, not 25

    def test_red_line_distance_and_side(self):
        # robot 0.6 m right of the left red line: line inner edge at 0.575 m on the left
        _, red = self.scan_one(self.m0, 0.0, FIELD.red_inner - 0.575, 0.0, [])
        self.assertAlmostEqual(red[30], 57, delta=3)
        self.assertAlmostEqual(red[90], 238, delta=10)                      # right line 2.375 m away, coarse there

    def test_struts_give_phantom_hits(self):
        black, _ = self.scan_one(self.m, 0.0, 0.0, 0.0, [])
        inner = int(self.m.distance_cm(self.m.inner))
        phantom = [i for i, d in enumerate(black) if d <= inner + 1]
        self.assertTrue(phantom, 'struts should produce hits at the inner ring')
        for i in phantom:                                                    # only near strut angles
            self.assertTrue(min(abs((i * 3 - s + 180) % 360 - 180) for s in CAM['strut_angles_deg']) <= 3, i)

    def test_center_error_biases_distance(self):
        ok, _ = self.scan_one(self.m0, 0.0, 0.0, 0.0, [(1.0 + FIELD.size / 2, 0.0)])
        # line 0 runs (almost) straight down the image (front = 272 deg), so shift the centre along it
        bad, _ = self.scan_one(self.m0, 0.0, 0.0, 0.0, [(1.0 + FIELD.size / 2, 0.0)], center_error=(0, 10))
        diffs = np.abs(np.array(bad[:3], float) - np.array(ok[:3], float))
        self.assertGreater(diffs.max(), 15)                                  # ~2.4 cm/px at 1 m


if __name__ == '__main__':
    unittest.main()
