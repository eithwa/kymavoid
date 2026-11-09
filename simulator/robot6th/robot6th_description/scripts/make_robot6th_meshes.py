#!/usr/bin/env python3
"""Generate the robot6th visual mesh, collision mesh and 2D footprint from the CAD export.

Source: meshes/robot6th_cad_mm.stl, the original CAD export. It is in millimetres, the robot
front points to +y, it floats ~0.38 m above its own origin, and its origin is ~4 cm off the
left/right symmetry axis. Every model used to patch this differently (scale 0.0009, y +0.04,
z -0.68, yaw 1.57 ...), which made the simulated robot 10 % smaller than the real one.

Outputs, all in the robot frame (metres, x forward, y left, z up, z = 0 at the lowest point,
origin = footprint centre: x from the bounding box, y on the left/right symmetry axis):
  meshes/robot6th.stl            visual mesh, vertex-clustered (the CAD export has ~330k faces)
  meshes/robot6th_collision.stl  footprint polygon extruded to the robot height (concave, exact)
  config/robot6th_footprint.yaml footprint polygon for planners / safety checks / evaluation

Usage (from the robot6th_description directory):
  scripts/make_robot6th_meshes.py [--copy-to DIR ...] [--preview PNG]
"""
import argparse
import math
import os
import shutil
import struct

import cv2
import numpy as np

RES = 0.001          # footprint raster resolution [m]
HALF = 0.26          # raster half-width [m]
CLUSTER = 0.003      # vertex clustering cell for the visual mesh [m]
POLY_EPS = 0.0015    # footprint polygon simplification tolerance [m]
STL_DTYPE = np.dtype([('n', '<3f4'), ('v', '<9f4'), ('a', '<u2')])


def load_stl(path):
    data = open(path, 'rb').read()
    n = struct.unpack('<I', data[80:84])[0]
    return np.frombuffer(data[84:84 + n * 50], dtype=STL_DTYPE)['v'].reshape(-1, 3, 3).astype(np.float64)


def save_stl(path, tris, header):
    tris = np.asarray(tris, dtype=np.float64)
    nrm = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    ln = np.linalg.norm(nrm, axis=1, keepdims=True)
    nrm = np.where(ln > 0, nrm / np.maximum(ln, 1e-12), 0.0)
    rec = np.zeros(len(tris), dtype=STL_DTYPE)
    rec['n'] = nrm.astype(np.float32)
    rec['v'] = tris.reshape(-1, 9).astype(np.float32)
    with open(path, 'wb') as f:
        f.write(header.encode()[:80].ljust(80, b' '))
        f.write(struct.pack('<I', len(tris)))
        f.write(rec.tobytes())


def raster(tris):
    """Top-view mask with holes filled. Row = -x (front up), column = -y (robot left on the left)."""
    n = int(round(2 * HALF / RES))
    img = np.zeros((n, n), np.uint8)
    col = np.round((HALF - tris[..., 1]) / RES).astype(np.int32)
    row = np.round((HALF - tris[..., 0]) / RES).astype(np.int32)
    cv2.fillPoly(img, list(np.stack([col, row], -1)), 255)
    cnts, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    filled = np.zeros_like(img)
    cv2.drawContours(filled, [max(cnts, key=cv2.contourArea)], -1, 255, cv2.FILLED)
    return filled


def symmetry_shift(mask):
    """Column shift that best maps the mask onto its own left/right mirror, and that IoU."""
    best = (-1.0, 0)
    for s in range(-80, 81):
        m = np.roll(mask, -s, axis=1) > 0
        f = m[:, ::-1]
        best = max(best, ((m & f).sum() / max((m | f).sum(), 1), s))
    return best[1], best[0]


def cluster(tris, cell):
    v = tris.reshape(-1, 3)
    _, inv = np.unique(np.floor(v / cell).astype(np.int64), axis=0, return_inverse=True)
    inv = inv.reshape(-1)
    rep = np.zeros((inv.max() + 1, 3))
    np.add.at(rep, inv, v)
    rep /= np.bincount(inv)[:, None]
    faces = inv.reshape(-1, 3)
    faces = faces[(faces[:, 0] != faces[:, 1]) & (faces[:, 1] != faces[:, 2]) & (faces[:, 0] != faces[:, 2])]
    _, first = np.unique(np.sort(faces, axis=1), axis=0, return_index=True)
    return rep[faces[np.sort(first)]]


def ear_clip(poly):
    """Triangulate a simple CCW polygon; returns index triples."""
    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    idx = list(range(len(poly)))
    out = []
    while len(idx) > 3:
        for k in range(len(idx)):
            a, b, c = idx[k - 1], idx[k], idx[(k + 1) % len(idx)]
            if cross(poly[a], poly[b], poly[c]) <= 1e-12:
                continue                       # reflex or collinear corner
            if any(cross(poly[a], poly[b], poly[j]) >= 0 and cross(poly[b], poly[c], poly[j]) >= 0
                   and cross(poly[c], poly[a], poly[j]) >= 0 for j in idx if j not in (a, b, c)):
                continue                       # another vertex inside this ear
            out.append((a, b, c))
            idx.pop(k)
            break
        else:
            raise RuntimeError('ear clipping failed: polygon is not simple')
    out.append(tuple(idx))
    return out


def prism(poly, height):
    tris = []
    for a, b, c in ear_clip(poly):
        tris.append([(*poly[a], height), (*poly[b], height), (*poly[c], height)])   # top, normal +z
        tris.append([(*poly[a], 0.0), (*poly[c], 0.0), (*poly[b], 0.0)])            # bottom, normal -z
    for i in range(len(poly)):
        p, q = poly[i], poly[(i + 1) % len(poly)]                                    # CCW -> outward sides
        tris.append([(*p, 0.0), (*q, 0.0), (*q, height)])
        tris.append([(*p, 0.0), (*q, height), (*p, height)])
    return np.array(tris)


def main():
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # robot6th_description
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--copy-to', action='append', default=[], help='also copy both meshes into this directory')
    ap.add_argument('--preview', help='write a top-view preview PNG here')
    args = ap.parse_args()

    tris = load_stl(os.path.join(here, 'meshes', 'robot6th_cad_mm.stl')) * 0.001         # mm -> m
    x = -tris[..., 1]                                                                       # rotate +90 deg about z:
    y = tris[..., 0]                                                                        # CAD +y (front) -> +x
    tris = np.stack([x, y, tris[..., 2]], -1)

    cx = (tris[..., 0].max() + tris[..., 0].min()) / 2
    cy = (tris[..., 1].max() + tris[..., 1].min()) / 2
    tris[..., 0] -= cx
    tris[..., 1] -= cy
    shift, iou = symmetry_shift(raster(tris))
    tris[..., 1] += shift * RES                     # column axis is -y
    cy -= shift * RES
    zmin = tris[..., 2].min()
    tris[..., 2] -= zmin
    height = tris[..., 2].max()
    print('CAD origin -> robot origin: x %+.4f m, y %+.4f m, z %+.4f m (mirror IoU %.4f)'
          % (-cx, -cy, -zmin, iou))

    mask = raster(tris)
    cnt = max(cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[0], key=cv2.contourArea)
    px = cv2.approxPolyDP(cnt, POLY_EPS / RES, True).reshape(-1, 2)
    poly = np.stack([HALF - px[:, 1] * RES, HALF - px[:, 0] * RES], -1)
    area = 0.5 * np.sum(poly[:, 0] * np.roll(poly[:, 1], -1) - np.roll(poly[:, 0], -1) * poly[:, 1])
    if area < 0:
        poly, area, px = poly[::-1], -area, px[::-1]
    r = np.hypot(poly[:, 0], poly[:, 1])
    print('footprint: %d points, %.4f m^2, x [%+.3f, %+.3f], y [%+.3f, %+.3f], max radius %.3f m, height %.3f m'
          % (len(poly), area, poly[:, 0].min(), poly[:, 0].max(), poly[:, 1].min(), poly[:, 1].max(),
             r.max(), height))

    visual = cluster(tris, CLUSTER)
    collision = prism([tuple(p) for p in poly], height)
    mesh_dir = os.path.join(here, 'meshes')
    save_stl(os.path.join(mesh_dir, 'robot6th.stl'), visual,
             'robot6th visual: metres, x fwd, z0 floor, origin footprint centre')
    save_stl(os.path.join(mesh_dir, 'robot6th_collision.stl'), collision,
             'robot6th collision: footprint prism, metres, x fwd, z0 floor')
    print('visual mesh: %d -> %d faces; collision mesh: %d faces' % (len(tris), len(visual), len(collision)))

    with open(os.path.join(here, 'config', 'robot6th_footprint.yaml'), 'w') as f:
        f.write('# robot6th footprint in base_link (x forward, y left), metres, CCW, holes filled.\n')
        f.write('# Generated by scripts/make_robot6th_meshes.py from meshes/robot6th_cad_mm.stl; do not edit.\n')
        f.write('height: %.4f\n' % height)
        f.write('footprint:\n')
        f.writelines('  - [%.4f, %.4f]\n' % (a, b) for a, b in poly)

    for d in args.copy_to:
        for name in ('robot6th.stl', 'robot6th_collision.stl'):
            shutil.copyfile(os.path.join(mesh_dir, name), os.path.join(d, name))
        print('copied meshes to', d)

    if args.preview:
        vis = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR) // 2
        c = (int(HALF / RES), int(HALF / RES))
        cv2.circle(vis, c, int(0.20 / RES), (0, 0, 255), 2)                        # old r = 0.20 m model
        cv2.circle(vis, c, int(round(r.max() / RES)), (0, 200, 0), 2)              # bounding circle
        cv2.polylines(vis, [px.reshape(-1, 1, 2).astype(np.int32)], True, (255, 255, 0), 2)
        cv2.arrowedLine(vis, c, (c[0], c[1] - 80), (255, 0, 0), 3)
        cv2.imwrite(args.preview, cv2.resize(vis, (520, 520), interpolation=cv2.INTER_AREA))


if __name__ == '__main__':
    main()
