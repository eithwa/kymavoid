"""Panoramic-camera model of vision/black_item.

Instead of approximating black_item's output, this module renders the black and red binary
masks black_item would see and then runs black_item's own scan on them. That way the pixel
quantisation, the inner-ring behaviour, the strut false hits and the effect of a wrong
mirror-centre calibration all come out exactly as on the robot.

Conventions (see avoid_nav/config/camera.yaml):
  * image = black_item's frame (the raw camera frame flipped left/right), y down
  * image angle a is counter-clockwise with y up; scan line i lies at a = front + 3 i * index_direction
  * scan line i is robot angle +3 i deg (counter-clockwise from the front, REP 103)
  * distances are centimetres, 999 = nothing found (as black_item publishes them)
"""
import math

import numpy as np

NO_HIT = 999


class OmniModel:
    def __init__(self, cam):
        self.h_mm = float(cam['camera_high'])
        self.cx, self.cy = float(cam['center_x']), float(cam['center_y'])
        self.inner, self.outer = int(cam['inner']), int(cam['outer'])
        self.front = float(cam['front'])
        self.w, self.h = int(cam['image_width']), int(cam['image_height'])
        self.step = int(cam['scan_step_deg'])
        self.dir = int(cam.get('index_direction', 1))
        self.struts = [float(a) for a in cam.get('strut_angles_deg', [])]
        self.strut_hw = float(cam.get('strut_half_width_px', 1.5))
        self.n_lines = 360 // self.step

    # ---- mirror model: black_item NodeHandle::camera_f / Omni_distance -------------------
    def distance_cm(self, px):
        """Floor distance [cm] from the robot centre for a pixel radius (vectorised)."""
        px = np.asarray(px, dtype=float)
        m = (self.outer * 2 * 0.9784 * 0.0099) / 60.0
        f = 180.0 / (1.0 + 1.0 / m)
        c = 83.125
        b = c * 0.8722
        r = np.arctan2(f, px * 0.0099)
        with np.errstate(divide='ignore', invalid='ignore'):
            d = -self.h_mm * (b * b - c * c) * np.cos(r) / ((b * b + c * c) * np.sin(r) - 2 * b * c) * 0.1
        return np.where((d >= 0) & (d <= NO_HIT), d, NO_HIT)

    def robot_angle_deg(self, image_angle_deg):
        return (image_angle_deg - self.front) * self.dir

    def image_angle_deg(self, robot_angle_deg):
        return self.front + robot_angle_deg * self.dir


class MaskRenderer:
    """Renders black/red masks for a robot pose. The mirror sits at the true centre (tcx, tcy)."""

    def __init__(self, model, field, true_center=None, angle_bins=1440):
        self.m, self.f = model, field
        self.tcx, self.tcy = true_center if true_center is not None else (model.cx, model.cy)
        u, v = np.meshgrid(np.arange(model.w), np.arange(model.h))
        dx, dy = u - self.tcx, self.tcy - v
        self.r = np.hypot(dx, dy)
        a = np.degrees(np.arctan2(dy, dx))
        self.valid = (self.r >= model.inner - 2) & (self.r <= model.outer + 2)
        d = model.distance_cm(self.r) / 100.0                                  # floor distance [m]
        self.d = np.where(d >= NO_HIT / 100.0 - 1e-6, np.inf, d)               # beyond the horizon
        self.bins = angle_bins
        phi = model.robot_angle_deg(a) % 360.0
        self.bin_idx = (np.round(phi / 360.0 * angle_bins).astype(np.int64)) % angle_bins
        self.bin_phi = np.radians(np.arange(angle_bins) * 360.0 / angle_bins)
        strut = np.zeros_like(self.valid)
        for s in model.struts:
            da = np.radians((a - model.image_angle_deg(s) + 180.0) % 360.0 - 180.0)
            strut |= (np.abs(da) * self.r <= model.strut_hw) & (np.abs(da) < math.pi / 2)
        self.strut = strut & self.valid
        self.cam_h = model.h_mm / 1000.0

    @staticmethod
    def _ray_boxes(ox, oy, ux, uy, boxes):
        """Entry/exit distance of rays (N) through boxes (M, x0 y0 x1 y1); inf when missed."""
        if not boxes:
            n = len(ux)
            return np.full((n, 0), np.inf), np.full((n, 0), np.inf)
        b = np.asarray(boxes, dtype=float)
        with np.errstate(divide='ignore', invalid='ignore'):
            ix = 1.0 / ux[:, None]
            iy = 1.0 / uy[:, None]
            tx0, tx1 = (b[:, 0] - ox) * ix, (b[:, 2] - ox) * ix
            ty0, ty1 = (b[:, 1] - oy) * iy, (b[:, 3] - oy) * iy
        # rays parallel to an axis: inside the slab -> (-inf, inf), outside -> miss
        px = np.isinf(ix)
        tx0 = np.where(px, np.where((ox >= b[:, 0]) & (ox <= b[:, 2]), -np.inf, np.inf), tx0)
        tx1 = np.where(px, np.where((ox >= b[:, 0]) & (ox <= b[:, 2]), np.inf, -np.inf), tx1)
        py = np.isinf(iy)
        ty0 = np.where(py, np.where((oy >= b[:, 1]) & (oy <= b[:, 3]), -np.inf, np.inf), ty0)
        ty1 = np.where(py, np.where((oy >= b[:, 1]) & (oy <= b[:, 3]), np.inf, -np.inf), ty1)
        t_in = np.maximum(np.minimum(tx0, tx1), np.minimum(ty0, ty1))
        t_out = np.minimum(np.maximum(tx0, tx1), np.maximum(ty0, ty1))
        hit = (t_out >= np.maximum(t_in, 0.0))
        t_in = np.where(hit, np.maximum(t_in, 0.0), np.inf)
        t_out = np.where(hit, t_out, np.inf)
        return t_in, t_out

    def _occlusion_far(self, near, height):
        """A box of this height hides the floor from `near` up to this distance (camera at cam_h)."""
        if height >= self.cam_h:
            return np.full_like(near, np.inf)
        return near * self.cam_h / (self.cam_h - height)

    def render(self, x, y, yaw, obstacles):
        """(black, red) boolean masks for the robot at (x, y, yaw) [m, rad] and obstacle centres."""
        ang = yaw + self.bin_phi
        ux, uy = np.cos(ang), np.sin(ang)
        f = self.f
        ob_in, _ = self._ray_boxes(x, y, ux, uy, [f.obstacle_box(ox, oy) for ox, oy in obstacles])
        wa_in, _ = self._ray_boxes(x, y, ux, uy, f.walls())
        rd_in, rd_out = self._ray_boxes(x, y, ux, uy, f.red_strips())
        ob_near = ob_in.min(axis=1) if ob_in.shape[1] else np.full(self.bins, np.inf)
        ob_far = self._occlusion_far(ob_near, f.height)
        wa_near = wa_in.min(axis=1)
        wa_far = self._occlusion_far(wa_near, f.wall_height)
        d = self.d
        bi = self.bin_idx
        black = (d >= ob_near[bi]) & (d <= ob_far[bi])
        black |= (d >= wa_near[bi]) & (d <= wa_far[bi])
        black = (black & self.valid) | self.strut
        red = np.zeros_like(black)
        for k in range(rd_in.shape[1]):
            rin, rout = rd_in[:, k], rd_out[:, k]
            seen = rin < np.minimum(ob_near, wa_near)                            # not hidden behind a box
            red |= (d >= rin[bi]) & (d <= rout[bi]) & seen[bi]
        red &= self.valid & ~black
        return black, red


class Scanner:
    """black_item's scan: from the inner ring outwards, the first marked pixel on each line."""

    def __init__(self, model, center=None):
        self.m = model
        cx, cy = center if center is not None else (model.cx, model.cy)
        rs = np.arange(model.inner, model.outer + 1)
        a = np.radians(model.front + np.arange(model.n_lines) * model.step * model.dir)
        ca, sa = np.cos(a)[:, None], np.sin(a)[:, None]
        dis_x = np.trunc(ca * rs).astype(np.int64)                              # C++ int() truncation
        dis_y = np.trunc(sa * rs).astype(np.int64)
        self.ix = np.clip(int(cx) + dis_x, 0, model.w - 1)
        self.iy = np.clip(int(cy) - dis_y, 0, model.h - 1)
        self.px = np.hypot(dis_x, dis_y)

    def first_hits(self, mask):
        """Per scan line: index of the first marked pixel along the line, and whether there is one."""
        hits = mask[self.iy, self.ix]
        first = np.argmax(hits, axis=1)
        return first, hits[np.arange(len(first)), first]

    def hit_pixels(self, mask):
        """Image coordinates (x, y) of the first marked pixel of every line that found one."""
        first, found = self.first_hits(mask)
        lines = np.nonzero(found)[0]
        return self.ix[lines, first[lines]], self.iy[lines, first[lines]]

    def scan(self, mask, pixel_noise=0.0, dropout=0.0, rng=None):
        """Distances [cm, int] for every scan line, 999 when nothing is found."""
        first, found = self.first_hits(mask)
        px = np.where(found, self.px[np.arange(len(first)), first], self.px[:, -1])
        if rng is not None and pixel_noise > 0:
            px = px + np.where(found, rng.normal(0.0, pixel_noise, len(px)), 0.0)
        d = self.m.distance_cm(px)
        out = np.where(found, d, NO_HIT).astype(np.int32)                         # black_item stores int
        if rng is not None and dropout > 0:
            out = np.where(rng.random(len(out)) < dropout, NO_HIT, out)
        return out
