// Speed limits for a holonomic robot whose velocity vector changes at a bounded acceleration
// (speeding up, braking and turning alike) and that acts on what it sees after a reaction time.
//
// nav_node used to limit the speed with rules tuned at 0.8 m/s (v_max cos(turn), straight-line
// braking towards the pursuit target). At higher speeds they let the robot arrive at corners faster
// than it can turn, overshoot into red lines and outrun the camera. These limits follow from the
// accelerations the command limiter actually uses, so they stay right for any v_max.
#pragma once
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "avoid_nav/polygon.h"

namespace avoid_nav
{
// Highest speed from which the robot, reacting after t_react seconds and then braking at a, is down
// to v_end within d metres:  v t_react + (v^2 - v_end^2) / 2a <= d.
inline double approachSpeed(double d, double a, double t_react, double v_end = 0.0)
{
  const double at = a * std::max(0.0, t_react);
  return std::max(0.0, -at + std::sqrt(at * at + v_end * v_end + 2.0 * a * std::max(0.0, d)));
}

// Turning through a path corner of angle theta on an arc tangent to both legs: an arc of radius R
// cuts the corner by R (1 / cos(theta / 2) - 1). Keeping that within `cut` bounds R, and the lateral
// acceleration v^2 / R <= a bounds the speed.
inline double cornerSpeed(double theta, double cut, double a)
{
  theta = std::min(std::fabs(theta), M_PI - 1e-3);
  if (theta < 1e-4)
    return std::numeric_limits<double>::infinity();
  return std::sqrt(a * std::max(0.0, cut) / (1.0 / std::cos(theta / 2) - 1.0));
}

// Already moving at an angle phi to the direction the robot has to go: the turn starts here, on an
// arc of radius v^2 / a that drifts R (1 - cos phi) off the new direction (at 90 deg and more, R).
inline double turnSpeed(double phi, double drift, double a)
{
  phi = std::min(std::fabs(phi), M_PI / 2);
  if (phi < 1e-4)
    return std::numeric_limits<double>::infinity();
  return std::sqrt(a * std::max(0.0, drift) / (1.0 - std::cos(phi)));
}

// Distance the robot origin can travel from (x, y) along unit (ux, uy) before it leaves the box
// [x_min, x_max] x [y_min, y_max] (the red lines and the low wall, already shrunk by the footprint).
// Negative when it already is beyond a side and moving further out; +inf when moving away from all.
inline double distanceToBounds(double x, double y, double ux, double uy, double x_min, double x_max, double y_min,
                               double y_max)
{
  double d = std::numeric_limits<double>::infinity();
  if (uy > 1e-9)
    d = std::min(d, (y_max - y) / uy);
  if (uy < -1e-9)
    d = std::min(d, (y - y_min) / -uy);
  if (ux > 1e-9)
    d = std::min(d, (x_max - x) / ux);
  if (ux < -1e-9)
    d = std::min(d, (x - x_min) / -ux);
  return d;
}

// A speed cap that varies along the path ahead (the clearance limit): sampled every `step` from the
// robot's point on segment path[seg] -> path[seg + 1] up to `horizon`, the robot must be able to brake
// down to the cap of every sample by the time it gets there. Without this the clearance limit only
// applied where the robot already was, and at speed it entered narrow passages far too fast.
template <class Cap>
double pathCapLimit(const std::vector<Pt>& path, size_t seg, const Pt& from, Cap cap, double a_brake, double t_react,
                    double horizon, double step = 0.05)
{
  double best = std::numeric_limits<double>::infinity();
  double s = 0;
  Pt p = from;
  for (size_t k = seg + 1; k < path.size() && s <= horizon; ++k)
  {
    const double len = std::hypot(path[k].x - p.x, path[k].y - p.y);
    for (double u = step; u <= len + 1e-9 && s + u <= horizon; u += step)
    {
      const Pt q{p.x + (path[k].x - p.x) * u / len, p.y + (path[k].y - p.y) * u / len};
      best = std::min(best, approachSpeed(s + u, a_brake, t_react, cap(q)));
    }
    s += len;
    p = path[k];
  }
  return best;
}

// Corners of a polyline path ahead of the robot. The robot stands at `from`, on segment
// path[seg] -> path[seg + 1]. Vertices closer than `merge` to the previous one are one corner (the
// smoothed A* path turns round a tight spot in several short steps). For every corner within
// `horizon`, the corner speed (cut = its clearance, at least min_cut) and the braking towards it give
// a limit on the speed now; the lowest is returned, with the distance to that corner in *at.
template <class Clearance>
double pathCornerLimit(const std::vector<Pt>& path, size_t seg, const Pt& from, Clearance clearance, double a_turn,
                       double a_brake, double t_react, double min_cut, double horizon, double merge = 0.15,
                       double* at = nullptr)
{
  double best = std::numeric_limits<double>::infinity();
  if (at)
    *at = -1;
  auto heading = [&](size_t i) { return std::atan2(path[i + 1].y - path[i].y, path[i + 1].x - path[i].x); };
  double s = std::hypot(path.size() > seg + 1 ? path[seg + 1].x - from.x : 0.0,
                        path.size() > seg + 1 ? path[seg + 1].y - from.y : 0.0);
  size_t k = seg + 1;
  while (k + 1 < path.size() && s <= horizon)
  {
    // one corner: vertex k and the vertices that follow it closely
    const double s_corner = s;
    double turn = 0, cut = clearance(k);
    size_t j = k;
    for (;;)
    {
      turn += std::remainder(heading(j) - heading(j - 1), 2 * M_PI);
      cut = std::min(cut, clearance(j));
      const double len = std::hypot(path[j + 1].x - path[j].x, path[j + 1].y - path[j].y);
      s += len;
      ++j;
      if (j + 1 >= path.size() || len >= merge)
        break;
    }
    const double v = approachSpeed(s_corner, a_brake, t_react, cornerSpeed(turn, std::max(min_cut, cut), a_turn));
    if (v < best)
    {
      best = v;
      if (at)
        *at = s_corner;
    }
    k = j;
  }
  return best;
}
}  // namespace avoid_nav
