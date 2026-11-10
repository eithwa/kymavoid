// Reference tracking: the planner's polyline with its corners rounded, and the robot kept on it.
//
// Pure pursuit (nav_node until 2026-11-10, still there with tracker: 0) aimed at the farthest point of
// the polyline it could see within the lookahead. A short lookahead started turning at the corner
// itself: the velocity vector turns at a bounded acceleration, so the robot swung wide past the inner
// obstacle and could not get back in time for what came next. A long one aimed along chords that by
// construction graze the C-space boundary: the robot cut every corner and ran along the obstacles
// (seed 97075 with lookahead 1 m: 2-4 cm from the boxes, 10-14 cm with 0.35 m). Either way the path
// actually driven was neither the planned one nor the one the speed limits had been computed for.
//
// Here every corner becomes an arc tangent to both legs, so the turn starts before the corner, with
// the largest radius that keeps, all along the arc, the clearance the polyline had there - down to
// `keep` at most: a corner the planner took 15 cm from the C-space boundary may be cut to `keep`, one
// it took at 3 cm (a narrow gap) is not cut at all. The speed profile along the result keeps
// v^2 kappa within the lateral acceleration, so the robot can stay on it, and a cross-track term
// pulls it back when it is off.
#pragma once
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "avoid_nav/polygon.h"
#include "avoid_nav/speed_limits.h"

namespace avoid_nav
{
struct RefPoint
{
  double x, y;
  double s;      // arc length from the start [m]
  double kappa;  // |curvature| [1/m]: 1/R on arcs, 0 on straight pieces, kSharpCorner at an unrounded vertex
  double clear;  // C-space clearance there [m]
};

constexpr double kSharpCorner = 1e3;   // curvature of a vertex no arc fits into

struct RoundParams
{
  double keep = 0.06;   // [m] a rounded corner may come this close to the C-space boundary (never closer than the polyline itself)
  double r_max = 3.0;   // [m] larger arcs gain nothing at the speeds here
  double ds = 0.02;     // [m] sample spacing of the reference
  double tol = 0.005;   // [m] grid quantisation allowance
};

// Rounds the corners of `path` (polyline, first point = the robot) and samples the result every ds.
// clearance(x, y): distance to the C-space boundary. radii (optional): the arc radius chosen at every
// vertex (0 = left sharp, also for the end points).
template <class Clearance>
std::vector<RefPoint> roundedReference(const std::vector<Pt>& path_in, Clearance clearance, const RoundParams& p,
                                       std::vector<double>* radii = nullptr)
{
  std::vector<Pt> P;
  for (const Pt& q : path_in)
    if (P.empty() || std::hypot(q.x - P.back().x, q.y - P.back().y) > 1e-3)
      P.push_back(q);
  std::vector<RefPoint> out;
  const size_t n = P.size();
  if (radii)
    radii->assign(n, 0.0);
  if (n == 0)
    return out;
  if (n == 1)
  {
    out.push_back({P[0].x, P[0].y, 0.0, 0.0, clearance(P[0].x, P[0].y)});
    return out;
  }
  std::vector<double> L(n - 1);
  std::vector<Pt> u(n - 1);
  for (size_t i = 0; i + 1 < n; ++i)
  {
    L[i] = std::hypot(P[i + 1].x - P[i].x, P[i + 1].y - P[i].y);
    u[i] = {(P[i + 1].x - P[i].x) / L[i], (P[i + 1].y - P[i].y) / L[i]};
  }
  // turn angle and side at every interior vertex
  std::vector<double> th(n, 0.0), side(n, 1.0), want(n, 0.0);
  for (size_t k = 1; k + 1 < n; ++k)
  {
    const double cr = u[k - 1].x * u[k].y - u[k - 1].y * u[k].x, dt = u[k - 1].x * u[k].x + u[k - 1].y * u[k].y;
    th[k] = std::min(std::atan2(std::fabs(cr), dt), M_PI - 1e-3);
    side[k] = cr >= 0 ? 1.0 : -1.0;
    want[k] = th[k] > 1e-4 ? p.r_max * std::tan(th[k] / 2) : 0.0;
  }
  // A leg between two corners is shared in proportion to what each would like (a 2 deg kink needs
  // little of it); the first and the last leg belong to their one corner.
  auto share = [](double mine, double other) { return mine + other > 0 ? mine / (mine + other) : 0.5; };
  // sample the arc of tangent length t at vertex k: fn(x, y) for every point after the tangent point A
  auto arc = [&](size_t k, double t, auto&& fn) {
    const double R = t / std::tan(th[k] / 2);
    const Pt A{P[k].x - t * u[k - 1].x, P[k].y - t * u[k - 1].y};
    const Pt nrm{-side[k] * u[k - 1].y, side[k] * u[k - 1].x};     // towards the centre
    const Pt C{A.x + R * nrm.x, A.y + R * nrm.y};
    const Pt r0{-R * nrm.x, -R * nrm.y};                            // centre -> A
    const int m = std::max(2, static_cast<int>(std::ceil(R * th[k] / p.ds)));
    for (int j = 1; j <= m; ++j)
    {
      const double a = side[k] * th[k] * j / m, c = std::cos(a), s = std::sin(a);
      fn(C.x + c * r0.x - s * r0.y, C.y + s * r0.x + c * r0.y);
    }
    return R;
  };
  // the arc keeps min(keep, clearance of the polyline piece it replaces) at every point
  auto fits = [&](size_t k, double t) {
    double poly = std::numeric_limits<double>::infinity();
    for (double d = 0; d <= t + 1e-9; d += p.ds)
    {
      poly = std::min(poly, clearance(P[k].x - d * u[k - 1].x, P[k].y - d * u[k - 1].y));
      poly = std::min(poly, clearance(P[k].x + d * u[k].x, P[k].y + d * u[k].y));
    }
    const double need = std::min(p.keep, poly) - p.tol;
    bool ok = true;
    arc(k, t, [&](double x, double y) { ok = ok && clearance(x, y) >= need; });
    return ok;
  };
  std::vector<double> t(n, 0.0);
  for (size_t k = 1; k + 1 < n; ++k)
  {
    if (want[k] <= 0)
      continue;
    const double in = k >= 2 ? L[k - 1] * share(want[k], want[k - 1]) : L[k - 1];
    const double outl = k + 2 < n ? L[k] * share(want[k], want[k + 1]) : L[k];
    const double tmax = std::min(want[k], std::min(in, outl));
    if (fits(k, tmax))
      t[k] = tmax;
    else
    {
      double lo = 0, hi = tmax;                    // the cut grows with t
      for (int it = 0; it < 12; ++it)
      {
        const double mid = 0.5 * (lo + hi);
        (fits(k, mid) ? lo : hi) = mid;
      }
      t[k] = lo;
    }
    if (t[k] < 0.003)
      t[k] = 0;
  }
  // emit straight pieces and arcs
  double s = 0;
  auto emit = [&](double x, double y, double kappa) {
    if (!out.empty())
    {
      const double d = std::hypot(x - out.back().x, y - out.back().y);
      if (d < 1e-6)
      {
        out.back().kappa = std::max(out.back().kappa, kappa);
        return;
      }
      s += d;
    }
    out.push_back({x, y, s, kappa, clearance(x, y)});
  };
  auto line = [&](const Pt& a, const Pt& b) {
    const double l = std::hypot(b.x - a.x, b.y - a.y);
    const int m = std::max(1, static_cast<int>(std::ceil(l / p.ds)));
    for (int j = 1; j <= m; ++j)
      emit(a.x + (b.x - a.x) * j / m, a.y + (b.y - a.y) * j / m, 0.0);
  };
  emit(P[0].x, P[0].y, 0.0);
  Pt cur = P[0];
  for (size_t k = 1; k + 1 < n; ++k)
  {
    if (t[k] <= 0)
    {
      line(cur, P[k]);
      if (th[k] > 1e-3)
        out.back().kappa = kSharpCorner;
      cur = P[k];
      continue;
    }
    const Pt A{P[k].x - t[k] * u[k - 1].x, P[k].y - t[k] * u[k - 1].y};
    line(cur, A);
    const size_t first = out.size() - 1;                         // A: the turn starts there
    const double R = arc(k, t[k], [&](double x, double y) { emit(x, y, 0.0); });
    for (size_t j = first; j < out.size(); ++j)
      out[j].kappa = std::max(out[j].kappa, 1.0 / R);
    if (radii)
      (*radii)[k] = R;
    cur = {P[k].x + t[k] * u[k].x, P[k].y + t[k] * u[k].y};
  }
  line(cur, P[n - 1]);
  return out;
}

// A replanned path starts where the robot is, but its first leg may point elsewhere than the robot is
// going. Replace the start by the way the robot can actually get onto the path at its speed: an arc
// of radius v^2 / a_lat tangent to its velocity, then the tangent line to a path vertex (the first one
// outside that circle, at most `max_skip` vertices on; the skipped piece of path is replaced). Every
// point of the join must keep min(keep, the clearance of the path piece it replaces) - tol. The arc
// is returned as points ds apart, so roundedReference gives it curvature 1/R and the speed profile
// holds the current speed through it. False (path unchanged) if no clear join exists or none is needed.
//
// Without it the robot, at 1.1 m/s and 20-30 deg off the new first leg, was 12 cm off the reference
// at once and passed a box at 3.6 cm that the reference passed at 10 (seed 2). A virtual leg along the
// velocity, rounded like any corner, did not help: that arc starts behind the robot, which is late
// for it from the start (13 cm off in test_track against 25 cm without).
template <class Clearance>
bool joinFromVelocity(std::vector<Pt>& path, const Pt& vel, double a_lat, Clearance clearance, const RoundParams& p,
                      size_t max_skip = 4)
{
  const double v = std::hypot(vel.x, vel.y);
  if (path.size() < 2 || v < 0.05 || a_lat <= 0)
    return false;
  const Pt P0 = path[0], u{vel.x / v, vel.y / v};
  const double R = v * v / a_lat;
  if (R < 0.02)
    return false;                                     // slow enough to turn on the spot
  double poly = std::numeric_limits<double>::infinity();   // clearance of the replaced path piece
  for (size_t k = 1; k < path.size() && k <= max_skip; ++k)
  {
    const Pt a = path[k - 1], b = path[k];
    const double L = std::hypot(b.x - a.x, b.y - a.y);
    for (double d = 0; d <= L + 1e-9; d += p.ds)
      poly = std::min(poly, clearance(a.x + (b.x - a.x) * d / std::max(L, 1e-9), a.y + (b.y - a.y) * d / std::max(L, 1e-9)));
    const Pt Pk = path[k];
    const double cr = u.x * (Pk.y - P0.y) - u.y * (Pk.x - P0.x), dt = u.x * (Pk.x - P0.x) + u.y * (Pk.y - P0.y);
    if (k == 1 && std::fabs(cr) <= 1e-3 * std::hypot(cr, dt) && dt > 0)
      return false;                                   // already heading straight at it
    const double side = cr >= 0 ? 1.0 : -1.0;
    const Pt C{P0.x - side * u.y * R, P0.y + side * u.x * R};
    const double d = std::hypot(Pk.x - C.x, Pk.y - C.y);
    if (d <= R * 1.001)
      continue;                                       // inside the turning circle: try the next vertex
    const double beta = std::atan2(Pk.y - C.y, Pk.x - C.x), alpha = std::acos(R / d);
    const double gT = beta - side * alpha;            // angle of the tangent point on the circle
    const double g0 = std::atan2(P0.y - C.y, P0.x - C.x);
    double sweep = std::fmod(side * (gT - g0) + 4 * M_PI, 2 * M_PI);
    if (sweep > M_PI)
      return false;                                   // more than a half turn: not a join
    const double need = std::min(p.keep, poly) - p.tol;
    std::vector<Pt> join;
    const int m = std::max(1, static_cast<int>(std::ceil(R * sweep / p.ds)));
    bool ok = true;
    for (int j = 1; j <= m && ok; ++j)
    {
      const double g = g0 + side * sweep * j / m;
      join.push_back({C.x + R * std::cos(g), C.y + R * std::sin(g)});
      ok = clearance(join.back().x, join.back().y) >= need;
    }
    const Pt T = join.back();
    const double Lt = std::hypot(Pk.x - T.x, Pk.y - T.y);
    for (double e = 0; e <= Lt && ok; e += p.ds)
      ok = clearance(T.x + (Pk.x - T.x) * e / Lt, T.y + (Pk.y - T.y) * e / Lt) >= need;
    if (!ok)
      return false;
    std::vector<Pt> out{P0};
    out.insert(out.end(), join.begin(), join.end());
    out.insert(out.end(), path.begin() + k, path.end());
    path.swap(out);
    return true;
  }
  return false;
}

// Closest point of the reference (as a polyline through its samples) to (x, y), searched among the
// samples up to s_max: segment index, arc length and the point itself.
inline void closestOnRef(const std::vector<RefPoint>& r, double x, double y, size_t& seg, double& s_foot, Pt& foot,
                         double s_max = std::numeric_limits<double>::infinity())
{
  seg = 0;
  s_foot = 0;
  foot = {r.empty() ? x : r[0].x, r.empty() ? y : r[0].y};
  double best = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i + 1 < r.size() && r[i].s <= s_max; ++i)
  {
    const double dx = r[i + 1].x - r[i].x, dy = r[i + 1].y - r[i].y, l2 = dx * dx + dy * dy;
    const double w = l2 > 0 ? std::max(0.0, std::min(1.0, ((x - r[i].x) * dx + (y - r[i].y) * dy) / l2)) : 0.0;
    const double px = r[i].x + w * dx, py = r[i].y + w * dy, d = std::hypot(px - x, py - y);
    if (d < best)
    {
      best = d;
      seg = i;
      foot = {px, py};
      s_foot = r[i].s + w * (r[i + 1].s - r[i].s);
    }
  }
}

// Unit tangent of the reference at arc length s (of the segment containing it; the last one beyond).
inline Pt tangentAt(const std::vector<RefPoint>& r, double s)
{
  size_t i = 0;
  while (i + 2 < r.size() && r[i + 1].s <= s)
    ++i;
  const double dx = r[i + 1].x - r[i].x, dy = r[i + 1].y - r[i].y, l = std::hypot(dx, dy);
  return l > 0 ? Pt{dx / l, dy / l} : Pt{1.0, 0.0};
}

// Speed now from which the robot, reacting after t_react and braking at a_brake, is down to cap(q)
// at every sample q within `horizon` ahead of arc length s0. *at: distance to the limiting sample.
// Never below a cap itself: a robot already that slow need not brake for it (approachSpeed alone
// asks for less within cap x t_react of the sample, and on an arc - every sample just ahead - kept
// the robot 10-50 % under the arc's own limit).
template <class Cap>
double refCapLimit(const std::vector<RefPoint>& r, double s0, Cap cap, double a_brake, double t_react, double horizon,
                   double* at = nullptr)
{
  double best = std::numeric_limits<double>::infinity();
  if (at)
    *at = -1;
  for (const RefPoint& q : r)
  {
    if (q.s < s0)
      continue;
    if (q.s > s0 + horizon)
      break;
    const double c = cap(q), v = std::max(c, approachSpeed(q.s - s0, a_brake, t_react, c));
    if (v < best)
    {
      best = v;
      if (at)
        *at = q.s - s0;
    }
  }
  return best;
}

struct TrackParams
{
  double v_tight = 0.15, v_per_clearance = 8.0;   // clearance speed limit, as nav_node's
  double a_max = 1.0, a_brake = 1.2, t_react = 0.1;
  double a_lat = 0.8;          // [m/s^2] lateral acceleration the curvature limit plans with (< a_max: room to correct)
  double horizon = 4.0;        // [m] look this far along the reference for limits
  double preview_time = 0.1;   // [s] steer along the tangent this far ahead (command and odometry latency)
  double k_track = 2.0;        // [1/s] cross-track gain
  double min_cut = 0.02;       // [m] turnSpeed's drift floor, as nav_node's
};

// One control cycle on the reference: where the robot is on it, the direction to drive in and the
// speed limits that come from the reference. v_cap: the other limits (v_max, sensor range).
struct TrackStep
{
  Pt dir{1, 0};
  double s = 0, err = 0, clear = 0;
  double v_clear = std::numeric_limits<double>::infinity(), v_corner = std::numeric_limits<double>::infinity();
  double v_turn = std::numeric_limits<double>::infinity(), corner_at = -1;
  double v() const { return std::min(v_clear, std::min(v_corner, v_turn)); }
};

// Direction to drive in at speed v: the reference tangent `preview` metres ahead of the foot point,
// plus k_track x the offset from the robot back to the foot point; unit vector. *err: that offset [m].
inline Pt trackDirection(const std::vector<RefPoint>& r, const Pt& robot, double s_foot, const Pt& foot, double preview,
                         double v, double k_track, double* err = nullptr)
{
  const Pt t = tangentAt(r, s_foot + std::max(0.0, preview));
  const double ex = foot.x - robot.x, ey = foot.y - robot.y;
  if (err)
    *err = std::hypot(ex, ey);
  const double vv = std::max(v, 0.05);
  const double dx = vv * t.x + k_track * ex, dy = vv * t.y + k_track * ey, l = std::hypot(dx, dy);
  return l > 1e-9 ? Pt{dx / l, dy / l} : t;
}

// robot_clear: the C-space clearance where the robot is (it may be off the reference): caps the speed
// now like the reference's clearance ahead, and bounds the drift of a turn starting here.
inline TrackStep trackStep(const std::vector<RefPoint>& r, const Pt& pos, const Pt& vel, double v_cap, const TrackParams& p,
                           double robot_clear = std::numeric_limits<double>::infinity())
{
  TrackStep o;
  if (r.size() < 2)
    return o;
  size_t seg;
  Pt foot;
  closestOnRef(r, pos.x, pos.y, seg, o.s, foot);
  o.clear = std::min(robot_clear, std::min(r[seg].clear, r[seg + 1].clear));
  o.v_clear = std::min(p.v_tight + p.v_per_clearance * o.clear,
                       refCapLimit(r, o.s, [&](const RefPoint& q) { return p.v_tight + p.v_per_clearance * q.clear; },
                                   p.a_brake, p.t_react, p.horizon));
  o.v_corner = refCapLimit(
      r, o.s,
      [&](const RefPoint& q) {
        return q.kappa > 1e-9 ? std::max(p.v_tight, std::sqrt(p.a_lat / q.kappa)) : std::numeric_limits<double>::infinity();
      },
      p.a_brake, p.t_react, p.horizon, &o.corner_at);
  const double cur = std::hypot(vel.x, vel.y);
  const double v_pre = std::min(v_cap, std::min(o.v_clear, o.v_corner));
  o.dir = trackDirection(r, pos, o.s, foot, cur * p.preview_time, v_pre, p.k_track, &o.err);
  // still moving in another direction (a new route, off the reference): the turn starts here and
  // drifts R (1 - cos phi) off (speed_limits.h); in tune with the reference this never limits
  if (cur > 0.05)
  {
    const double phi = std::acos(std::max(-1.0, std::min(1.0, (vel.x * o.dir.x + vel.y * o.dir.y) / cur)));
    o.v_turn = std::max(p.v_tight, turnSpeed(phi, std::max(p.min_cut, o.clear), p.a_max));
  }
  return o;
}
}  // namespace avoid_nav
