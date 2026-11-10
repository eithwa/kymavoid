#!/usr/bin/env python3
"""Batch evaluation of avoid_nav in avoid_sim, judged like the challenge: touching loses.

For every seed: /avoid_sim/reset -> /avoid/command start -> watch the ground truth until the
footprint touches the goal line (success), an obstacle (collision) or a red line (line), nav_node
gives up (safe_stop) or the time runs out (timeout). Judging uses the true pose and the real
footprint polygon, not Gazebo contacts.

  rosrun avoid_sim evaluate.py --seeds 1-30 [--start-error-xy 0.02 --start-error-yaw 1.0] [--csv out.csv]

Recorded cases (avoid_sim/cases/*.yaml, see avoid_sim/cases.py): every case is re-placed exactly
(obstacles, start pose, noise seed), run, and checked against its `expect`. Cases recorded with the
other sensor configuration are skipped. Verdicts:
  PASS        expectations met
  FAIL        a `fixed` case no longer meets them: a regression (exit code 1)
  OPEN        an `open` case (known problem) still fails
  NOW PASSES  an `open` case meets its expectations: consider marking it fixed
  SKIP        recorded with the other sensor configuration

  rosrun avoid_sim evaluate.py --cases regression.yaml [--only id1,id2] [--repeat 3]
"""
import argparse
import collections
import csv
import math
import os
import sys
import threading
import time

import numpy as np
import rospkg
import rospy
from avoid_msgs.msg import Status
from avoid_msgs.srv import Command, ResetField
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

from avoid_sim import cases as case_files
from avoid_sim.geometry import Field, footprint_world, load_footprint
from geometry_msgs.msg import Pose2D


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def seg_seg_dist(p1, p2, q1, q2):
    def pt_seg(p, a, b):
        ab = b - a
        t = np.clip(np.dot(p - a, ab) / max(np.dot(ab, ab), 1e-12), 0, 1)
        return np.linalg.norm(p - (a + t * ab))

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    d1, d2 = cross(q1, q2, p1), cross(q1, q2, p2)
    d3, d4 = cross(p1, p2, q1), cross(p1, p2, q2)
    if d1 * d2 < 0 and d3 * d4 < 0:
        return 0.0
    return min(pt_seg(p1, q1, q2), pt_seg(p2, q1, q2), pt_seg(q1, p1, p2), pt_seg(q2, p1, p2))


def point_in_poly(p, poly):
    x, y, inside = p[0], p[1], False
    for i in range(len(poly)):
        a, b = poly[i], poly[i - 1]
        if (a[1] > y) != (b[1] > y) and x < (b[0] - a[0]) * (y - a[1]) / (b[1] - a[1]) + a[0]:
            inside = not inside
    return inside


def poly_box_distance(poly, box):
    """0 when the footprint polygon touches/overlaps the axis-aligned box (x0, y0, x1, y1)."""
    x0, y0, x1, y1 = box
    corners = np.array([[x0, y0], [x1, y0], [x1, y1], [x0, y1]])
    if any(x0 <= p[0] <= x1 and y0 <= p[1] <= y1 for p in poly) or any(point_in_poly(c, poly) for c in corners):
        return 0.0
    best = 1e9
    for i in range(len(poly)):
        for j in range(4):
            best = min(best, seg_seg_dist(poly[i - 1], poly[i], corners[j - 1], corners[j]))
            if best == 0.0:
                return 0.0
    return best


class Trial:
    def __init__(self, f, fp, obstacles):
        self.f, self.fp = f, fp
        self.boxes = [f.obstacle_box(o.x, o.y) for o in obstacles]
        self.min_obs = self.min_line = 1e9
        self.loc_err = []
        self.result = None

    def judge(self, x, y, yaw):
        poly = footprint_world(self.fp, x, y, yaw)
        d_obs = 1e9
        for b in self.boxes:                                  # exact distance only for nearby boxes
            far = max(b[0] - x, x - b[2], b[1] - y, y - b[3], 0.0)
            d_obs = min(d_obs, far - 0.25 if far > 0.35 else poly_box_distance(poly, b))
        d_line = self.f.red_inner - np.abs(poly[:, 1]).max()
        self.min_obs, self.min_line = min(self.min_obs, d_obs), min(self.min_line, d_line)
        if d_obs <= 0:
            return 'collision'
        if d_line <= 0:
            return 'line'
        if poly[:, 0].max() >= self.f.goal_line_x:
            return 'success'
        return None


def parse_seeds(s):
    out = []
    for part in s.split(','):
        if '-' in part:
            a, b = part.split('-')
            out += list(range(int(a), int(b) + 1))
        else:
            out.append(int(part))
    return out


def check(case, row):
    """Failed expectations of a case for one run (empty = met)."""
    e = case.get('expect') or {'result': 'success'}
    out = []
    if 'result' in e and row['result'].split(':')[0] != e['result']:
        out.append('result %s, expected %s' % (row['result'], e['result']))
    if 'max_time' in e and row['time'] > e['max_time']:
        out.append('time %.1f s > %.1f s' % (row['time'], e['max_time']))
    if 'min_obstacle' in e and row['min_obstacle'] < e['min_obstacle']:
        out.append('obstacle gap %.3f m < %.3f m' % (row['min_obstacle'], e['min_obstacle']))
    if 'min_line' in e and row['min_line'] < e['min_line']:
        out.append('line gap %.3f m < %.3f m' % (row['min_line'], e['min_line']))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--seeds', default='1-10')
    ap.add_argument('--layout', default='random')
    ap.add_argument('--timeout', type=float, default=60.0, help='sim seconds per run')
    ap.add_argument('--start-error-xy', type=float, default=0.0, help='stddev [m]')
    ap.add_argument('--start-error-yaw', type=float, default=0.0, help='stddev [deg]')
    ap.add_argument('--csv', default='')
    ap.add_argument('--cases', nargs='+', default=[], help='case files (paths or names in avoid_sim/cases)')
    ap.add_argument('--only', default='', help='comma-separated case ids')
    ap.add_argument('--repeat', type=int, default=1, help='runs per case (the simulation is not bit-exact)')
    args = ap.parse_args(rospy.myargv()[1:])
    rospy.init_node('evaluate', anonymous=True)
    ev = Evaluator(args.timeout)
    if args.cases:
        sys.exit(run_cases(ev, args))
    rows = []
    for seed in parse_seeds(args.seeds):
        row = ev.run(dict(seed=seed, layout=args.layout, start_error_xy=args.start_error_xy,
                          start_error_yaw=math.radians(args.start_error_yaw)), 'seed %3d' % seed)
        if row:
            row = dict(seed=seed, **row)
            rows.append(row)
    summary(rows, args.csv)


def run_cases(ev, args):
    only = set(filter(None, args.only.split(',')))
    sensor = ev.sensor()
    todo = []
    for f in args.cases:
        path = case_files.resolve(f)
        for c in case_files.load(path):
            if not only or c['id'] in only:
                todo.append((os.path.basename(path), c))
    if only - {c['id'] for _, c in todo}:
        print('unknown case ids: %s' % sorted(only - {c['id'] for _, c in todo}))
    print('%d cases, nav_node sensor configuration: %s, %d run(s) each\n' % (len(todo), sensor, args.repeat))
    verdicts, rows = [], []
    for fname, c in todo:
        if c.get('sensor', 'camera') != sensor:
            verdicts.append((c['id'], 'SKIP', 'recorded with %s' % c.get('sensor')))
            continue
        x, y, yaw = c['start']
        req = dict(seed=int(c['seed']), layout='case', start_error_xy=0.0, start_error_yaw=0.0,
                   use_obstacles=[Pose2D(o[0], o[1], 0.0) for o in c['obstacles']],
                   use_start=[Pose2D(x, y, math.radians(yaw))])
        problems = []
        for k in range(args.repeat):
            row = ev.run(req, '%-24s' % (c['id'][:24] + ('#%d' % (k + 1) if args.repeat > 1 else '')),
                         float(c.get('timeout', args.timeout)))
            if row is None:
                problems.append('run %d did not start' % (k + 1))
                continue
            rows.append(dict(case=c['id'], run=k + 1, **row))
            problems += ['run %d: %s' % (k + 1, p) if args.repeat > 1 else p for p in check(c, row)]
        open_case = c.get('status', 'open') == 'open'
        if problems:
            verdicts.append((c['id'], 'OPEN' if open_case else 'FAIL', '; '.join(problems)))
        else:
            verdicts.append((c['id'], 'NOW PASSES' if open_case else 'PASS', c.get('note', '')[:60]))
    print('\n%-28s %-11s %s' % ('case', 'verdict', 'details'))
    for cid, v, d in verdicts:
        print('%-28s %-11s %s' % (cid, v, d))
    counts = collections.Counter(v for _, v, _ in verdicts)
    print('\n' + '  '.join('%s %d' % (k, counts[k]) for k in ('PASS', 'FAIL', 'OPEN', 'NOW PASSES', 'SKIP') if counts[k]))
    if args.csv and rows:
        write_csv(rows, args.csv)
    return 1 if counts['FAIL'] else 0


class Evaluator:
    def __init__(self, timeout):
        self.timeout = timeout
        self.f = Field.from_params(rospy.get_param('/field'), rospy.get_param('/obstacles'))
        self.fp = load_footprint(os.path.join(rospkg.RosPack().get_path('robot6th_description'), 'config', 'robot6th_footprint.yaml'))
        self.reset = rospy.ServiceProxy('/avoid_sim/reset', ResetField)
        self.command = rospy.ServiceProxy('/avoid/command', Command)
        self.reset.wait_for_service()
        self.command.wait_for_service()
        self.truth, self.est, self.status = {}, {}, {}
        self.hist = collections.deque(maxlen=200)   # (t, x, y, yaw) of the truth, to compare at the estimate's stamp
        self.lock = threading.Lock()                # hist: appended by the subscriber thread, read by run()
        # queue_size=1: never judge on a backlog of old poses
        rospy.Subscriber('/sim/odom_true', Odometry, self.on_truth, queue_size=1)
        rospy.Subscriber('/avoid/pose', PoseWithCovarianceStamped, lambda m: self.est.update(
            p=(m.pose.pose.position.x, m.pose.pose.position.y, yaw_of(m.pose.pose.orientation)), t=m.header.stamp.to_sec()),
            queue_size=1)
        rospy.Subscriber('/avoid/status', Status, lambda m: self.status.update(s=m), queue_size=1)

    def on_truth(self, m):
        p = (m.pose.pose.position.x, m.pose.pose.position.y, yaw_of(m.pose.pose.orientation))
        with self.lock:
            self.truth.update(p=p, t=m.header.stamp.to_sec())
            self.hist.append((m.header.stamp.to_sec(),) + p)

    def sensor(self):
        t0 = time.time()
        while 's' not in self.status and time.time() - t0 < 10 and not rospy.is_shutdown():
            time.sleep(0.1)
        return self.status['s'].sensor_config if 's' in self.status else 'unknown'

    def truth_at(self, t):
        with self.lock:
            h = list(self.hist)
        for a, b in zip(reversed(h[:-1]), reversed(h)):
            if a[0] <= t <= b[0]:
                w = (t - a[0]) / max(b[0] - a[0], 1e-9)
                return (a[1] + w * (b[1] - a[1]), a[2] + w * (b[2] - a[2]), a[3] + w * math.atan2(math.sin(b[3] - a[3]), math.cos(b[3] - a[3])))
        return None

    def run(self, reset_req, label, timeout=None):
        """reset -> start -> judge until the end; a result row, or None if it could not start."""
        timeout = timeout or self.timeout
        self.command('stop')
        r = self.reset(**reset_req)
        if not r.success:
            print('%s: reset failed: %s' % (label, r.message))
            return None
        rospy.sleep(1.5)                                       # let sensors refresh after the teleport
        res = self.command('start')
        if not res.success:
            print('%s: start refused: %s' % (label, res.message))
            return None
        tr = Trial(self.f, self.fp, r.obstacles)
        t0 = rospy.get_time()
        rate = rospy.Rate(50)
        path_len, last_xy = 0.0, None
        while not rospy.is_shutdown():
            x, y, yaw = self.truth['p']
            if last_xy:
                path_len += math.hypot(x - last_xy[0], y - last_xy[1])
            last_xy = (x, y)
            tr.result = tr.judge(x, y, yaw)
            if 'p' in self.est and rospy.get_time() - t0 > 0.3:     # skip estimates published before the start reset
                ref = self.truth_at(self.est['t'])                    # the truth at the same instant as the estimate
                if ref is not None:
                    ex, ey, eyaw = self.est['p']
                    tx, ty, tyaw = ref
                    tr.last = (tx, ex, self.truth['t'] - self.est['t'])
                    tr.loc_err.append((ex - tx, ey - ty, math.degrees(math.atan2(math.sin(eyaw - tyaw), math.cos(eyaw - tyaw)))))
            st = self.status.get('s')
            if tr.result is None and st is not None and st.state_name == 'SAFE_STOP':
                tr.result = 'safe_stop: ' + st.reason
            if tr.result is None and rospy.get_time() - t0 > timeout:
                tr.result = 'timeout'
            if tr.result:
                break
            rate.sleep()
        elapsed = rospy.get_time() - t0
        self.command('stop')
        le = np.array(tr.loc_err) if tr.loc_err else np.zeros((1, 3))
        row = dict(result=tr.result, time=round(elapsed, 2), mean_speed=round(path_len / max(elapsed, 1e-6), 3),
                   min_obstacle=round(tr.min_obs, 3), min_line=round(tr.min_line, 3), end_err_x=round(le[-1, 0], 3),
                   max_err_x=round(np.abs(le[:, 0]).max(), 3), max_err_y=round(np.abs(le[:, 1]).max(), 3),
                   max_err_yaw=round(np.abs(le[:, 2]).max(), 2),
                   start_x=round(r.start.x, 3), start_y=round(r.start.y, 3), start_yaw=round(math.degrees(r.start.theta), 2))
        print('%s  %-10s %5.1f s  %.2f m/s  min gap obstacle %.3f m  line %.3f m | x err at end %+.3f (max %.3f)  max y err %.3f  max yaw err %.2f deg'
              % (label, tr.result[:10], elapsed, row['mean_speed'], tr.min_obs, tr.min_line, row['end_err_x'], row['max_err_x'],
                 row['max_err_y'], row['max_err_yaw']))
        sys.stdout.flush()
        return row


def write_csv(rows, path):
    with open(path, 'w', newline='') as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    print('wrote', path)


def summary(rows, csv_path):
    ok = [r for r in rows if r['result'] == 'success']
    print('\n%d/%d success' % (len(ok), len(rows)), end='')
    if ok:
        print(' | time mean %.1f s (min %.1f, max %.1f) | min obstacle gap %.3f m | min line gap %.3f m'
              ' | |x err| at end max %.3f m | y err max %.3f m'
              % (np.mean([r['time'] for r in ok]), min(r['time'] for r in ok), max(r['time'] for r in ok),
                 min(r['min_obstacle'] for r in ok), min(r['min_line'] for r in ok),
                 max(abs(r['end_err_x']) for r in ok), max(r['max_err_y'] for r in ok)), end='')
    print()
    fails = {}
    for r in rows:
        if r['result'] != 'success':
            fails.setdefault(r['result'].split(':')[0], []).append(r['seed'])
    for k, v in fails.items():
        print('  %s: seeds %s' % (k, v))
    if csv_path and rows:
        write_csv(rows, csv_path)


if __name__ == '__main__':
    main()
