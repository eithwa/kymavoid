"""Field specification, robot footprint, obstacle layouts and reachability (no ROS dependency).

Field frame: x from the start towards the goal, y to the left, origin at the field centre.
Obstacles are axis-aligned squares. The robot never rotates, so its configuration space is
obtained exactly by dilating the obstacle map with the footprint mask (Minkowski sum).
"""
import math
from dataclasses import dataclass

import cv2
import numpy as np
import yaml


@dataclass
class Field:
    red_line_y: float = 1.5
    red_line_width: float = 0.05
    red_line_x_min: float = -3.0
    red_line_x_max: float = 3.0
    start_line_x: float = -2.0
    goal_line_x: float = 3.0
    start_x: float = -2.0
    start_y: float = 0.0
    start_yaw: float = 0.0
    wall_x: float = 3.55
    wall_y: float = 2.55
    wall_thickness: float = 0.15
    wall_height: float = 0.20
    # obstacles
    count: int = 8
    size: float = 0.45
    height: float = 1.0
    min_gap: float = 0.50
    gap_metric: str = 'axis'
    obs_x_min: float = -1.3
    obs_x_max: float = 2.5
    route_width: float = 0.50     # 'route' layouts: no gap spec, but one route at least this wide

    @property
    def red_inner(self):
        """|y| of the inner edge of the red lines; the robot must stay strictly inside it."""
        return self.red_line_y - self.red_line_width / 2

    @staticmethod
    def from_params(field, obstacles):
        """Build from the /field and /obstacles parameter dicts (avoid_nav/config/field.yaml)."""
        sp = field.get('start_pose', {})
        return Field(
            red_line_y=field['red_line_y'], red_line_width=field['red_line_width'],
            red_line_x_min=field['red_line_x_min'], red_line_x_max=field['red_line_x_max'],
            start_line_x=field['start_line_x'], goal_line_x=field['goal_line_x'],
            start_x=sp.get('x', field['start_line_x']), start_y=sp.get('y', 0.0), start_yaw=sp.get('yaw', 0.0),
            wall_x=field['wall_x'], wall_y=field['wall_y'],
            wall_thickness=field['wall_thickness'], wall_height=field['wall_height'],
            count=obstacles['count'], size=obstacles['size'], height=obstacles['height'],
            min_gap=obstacles['min_gap'], gap_metric=obstacles['gap_metric'],
            obs_x_min=obstacles['x_min'], obs_x_max=obstacles['x_max'],
            route_width=obstacles.get('route_width', 0.50))

    @staticmethod
    def from_yaml(path):
        d = yaml.safe_load(open(path))
        return Field.from_params(d['field'], d['obstacles'])

    def walls(self):
        """Low wall as 4 boxes (x0, y0, x1, y1)."""
        t = self.wall_thickness / 2
        X, Y = self.wall_x, self.wall_y
        return [(-X - t, -Y - t, X + t, -Y + t), (-X - t, Y - t, X + t, Y + t),
                (-X - t, -Y - t, -X + t, Y + t), (X - t, -Y - t, X + t, Y + t)]

    def red_strips(self):
        """Red lines as 2 thin boxes (x0, y0, x1, y1)."""
        y, w = self.red_line_y, self.red_line_width / 2
        return [(self.red_line_x_min, y - w, self.red_line_x_max, y + w),
                (self.red_line_x_min, -y - w, self.red_line_x_max, -y + w)]

    def obstacle_box(self, x, y):
        h = self.size / 2
        return (x - h, y - h, x + h, y + h)


def load_footprint(path):
    """Footprint polygon (N x 2, metres, base frame) from robot6th_footprint.yaml."""
    return np.array(yaml.safe_load(open(path))['footprint'], dtype=float)


CLASSIC_LAYOUT = [(1.01, 0.53), (-1.10, 0.85), (1.85, -0.84), (-0.83, -0.59),
                  (1.0, 0.0), (0.51, 0.98), (0.48, -1.04), (0.06, 0.01)]
"""Obstacle centres of the original avoid_challenge.world (does not meet the gap spec)."""


def pair_gap(p, q, size, metric):
    """Free gap between two axis-aligned squares; negative when they overlap."""
    gx = abs(p[0] - q[0]) - size
    gy = abs(p[1] - q[1]) - size
    if gx < 0 and gy < 0:
        return max(gx, gy)
    if metric == 'axis':
        return max(gx, gy)
    return math.hypot(max(gx, 0.0), max(gy, 0.0))


def layout_violations(layout, f):
    """Pairs closer than the spec, as (i, j, gap)."""
    bad = []
    for i in range(len(layout)):
        for j in range(i + 1, len(layout)):
            g = pair_gap(layout[i], layout[j], f.size, f.gap_metric)
            if g < f.min_gap - 1e-9:
                bad.append((i, j, g))
    return bad


class ConfigSpace:
    """Occupancy grid of the corridor and its C-space for the (non-rotating) robot footprint."""

    def __init__(self, f, footprint, res=0.01, margin=0.0):
        self.f, self.res = f, res
        self.x0, self.x1 = f.red_line_x_min - 0.6, f.red_line_x_max + 0.6
        self.y0, self.y1 = -f.red_inner - 0.3, f.red_inner + 0.3
        self.nx = int(round((self.x1 - self.x0) / res)) + 1
        self.ny = int(round((self.y1 - self.y0) / res)) + 1
        # kernel = footprint (plus an optional safety margin), anchored at the robot origin
        half = int(math.ceil((np.abs(footprint).max() + margin) / res)) + 1
        k = np.zeros((2 * half + 1, 2 * half + 1), np.uint8)
        pts = np.round(footprint / res + half).astype(np.int32)          # (col, row) = (x, y) / res + half
        cv2.fillPoly(k, [pts.reshape(-1, 1, 2)], 1)
        if margin > 0:
            m = int(round(margin / res))
            k = cv2.dilate(k, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * m + 1, 2 * m + 1)))
        self.kernel, self.half = k, half
        self.front = float(footprint[:, 0].max())

    def cell(self, x, y):
        return int(round((y - self.y0) / self.res)), int(round((x - self.x0) / self.res))

    def occupancy(self, layout):
        occ = np.zeros((self.ny, self.nx), np.uint8)
        ys = self.y0 + np.arange(self.ny) * self.res
        occ[np.abs(ys) >= self.f.red_inner, :] = 1                       # red lines and beyond
        for (x, y) in layout:
            x0, y0, x1, y1 = self.f.obstacle_box(x, y)
            # cells whose centre lies inside the box
            c0, c1 = int(math.ceil((x0 - self.x0) / self.res)), int(math.floor((x1 - self.x0) / self.res))
            r0, r1 = int(math.ceil((y0 - self.y0) / self.res)), int(math.floor((y1 - self.y0) / self.res))
            occ[max(r0, 0):min(r1, self.ny - 1) + 1, max(c0, 0):min(c1, self.nx - 1) + 1] = 1
        return occ

    def cspace(self, layout):
        """1 where the robot origin would make the footprint touch an obstacle or a red line."""
        return cv2.dilate(self.occupancy(layout), self.kernel, anchor=(self.half, self.half),
                          borderType=cv2.BORDER_CONSTANT, borderValue=0)

    def reachable(self, layout, start=None):
        """Can the robot get from the start pose to touching the goal line without touching anything?"""
        c = self.cspace(layout)
        sx, sy = start if start is not None else (self.f.start_x, self.f.start_y)
        r, col = self.cell(sx, sy)
        if c[r, col]:
            return False
        free = (1 - c).astype(np.uint8)
        mask = np.zeros((self.ny + 2, self.nx + 2), np.uint8)
        cv2.floodFill(free, mask, (col, r), 2, flags=4)
        goal_col = int(math.ceil((self.f.goal_line_x - self.front - self.x0) / self.res))
        return bool((free[:, goal_col:] == 2).any())

    def route_wide(self, layout, width, start=None):
        """Is there a route at least `width` wide from the start to the goal line?

        A disc of diameter `width` must be able to travel from the start until it touches the goal
        line without touching an obstacle or a red line. Width measured in every direction, diagonal
        passages too; the robot (40 cm, 44.5 cm across its corners) fits through any such route."""
        r = int(math.ceil(width / 2 / self.res))
        disc = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * r + 1, 2 * r + 1))
        c = cv2.dilate(self.occupancy(layout), disc, borderType=cv2.BORDER_CONSTANT, borderValue=0)
        sx, sy = start if start is not None else (self.f.start_x, self.f.start_y)
        row, col = self.cell(sx, sy)
        if c[row, col]:
            return False
        free = (1 - c).astype(np.uint8)
        mask = np.zeros((self.ny + 2, self.nx + 2), np.uint8)
        cv2.floodFill(free, mask, (col, row), 2, flags=4)
        goal_col = int(math.ceil((self.f.goal_line_x - width / 2 - self.x0) / self.res))
        return bool((free[:, goal_col:] == 2).any())


def _boxes_in_range(f, rng, accept):
    """f.count box centres drawn in the obstacle area, off the start, each passing accept(p, placed)."""
    y_lim = f.red_inner - f.size / 2                                         # boxes stay inside the lines
    start_box = (f.start_x - 0.25, f.start_y - 0.25, f.start_x + 0.25, f.start_y + 0.25)
    layout = []
    for _ in range(f.count):
        for _ in range(400):
            p = (rng.uniform(f.obs_x_min, f.obs_x_max), rng.uniform(-y_lim, y_lim))
            bx = f.obstacle_box(*p)
            if not (bx[2] < start_box[0] or bx[0] > start_box[2] or bx[3] < start_box[1] or bx[1] > start_box[3]):
                continue
            if accept(p, layout):
                layout.append(p)
                break
        else:
            return None
    return layout


def sample_layout_route(f, footprint, rng, max_attempts=20000):
    """Random obstacle layout without the gap spec: boxes may stand at any distance from each other
    (touching, never overlapping), so slits narrower than the robot appear. The only requirement is
    one route at least f.route_width wide from the start to the goal line.

    Returns (layout, attempts) where attempts counts rejected candidate layouts."""
    cs = ConfigSpace(f, footprint)
    attempts = 0
    while attempts < max_attempts:
        layout = _boxes_in_range(f, rng, lambda p, placed: all(pair_gap(p, q, f.size, 'axis') >= 0 for q in placed))
        if layout is not None and cs.route_wide(layout, f.route_width):
            return layout, attempts
        attempts += 1
    raise RuntimeError('no layout with a %.2f m route after %d attempts' % (f.route_width, max_attempts))


def sample_layout(f, footprint, rng, max_attempts=20000):
    """Random obstacle layout that meets the gap spec and keeps the course passable.

    Returns (layout, attempts) where attempts counts rejected candidate layouts."""
    cs = ConfigSpace(f, footprint)
    y_lim = f.red_inner - f.size / 2                                         # boxes stay inside the lines
    start_box = (f.start_x - 0.25, f.start_y - 0.25, f.start_x + 0.25, f.start_y + 0.25)
    attempts = 0
    while attempts < max_attempts:
        layout = []
        for _ in range(f.count):
            for _ in range(400):
                p = (rng.uniform(f.obs_x_min, f.obs_x_max), rng.uniform(-y_lim, y_lim))
                bx = f.obstacle_box(*p)
                if not (bx[2] < start_box[0] or bx[0] > start_box[2] or bx[3] < start_box[1] or bx[1] > start_box[3]):
                    continue
                if all(pair_gap(p, q, f.size, f.gap_metric) >= f.min_gap for q in layout):
                    layout.append(p)
                    break
            else:
                break
        if len(layout) == f.count and cs.reachable(layout):
            return layout, attempts
        attempts += 1
    raise RuntimeError('no valid obstacle layout after %d attempts' % max_attempts)


def footprint_world(footprint, x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.stack([x + c * footprint[:, 0] - s * footprint[:, 1],
                     y + s * footprint[:, 0] + c * footprint[:, 1]], -1)
