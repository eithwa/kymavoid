// Obstacle layer in the local (odometry) frame, costmap style: every scan marks the cells it hits
// and clears the cells its beams pass through. Nothing expires with time: what is out of view
// (behind obstacles, inside the camera's inner ring) keeps its last observed state, what is in
// view is kept up to date. Ghosts left by odometry drift are cleared by the beams that see
// through them.
#pragma once
#include <sensor_msgs/LaserScan.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include "avoid_nav/polygon.h"

namespace avoid_nav
{
struct OccParams
{
  double res = 0.02;
  double x0 = -4.0, x1 = 4.5, y0 = -3.2, y1 = 3.2;   // local frame (starts on the field frame)
  int hit = 3, miss = 1;          // a hit is stronger than a pass-through: grazing beams must not erase
  int max_level = 8, min_level = -2;
  int occupied_level = 2;         // >= this is an obstacle (one hit is enough)
  double clear_margin = 0.10;     // stop clearing this short of the hit
};

class OccGrid
{
public:
  explicit OccGrid(const OccParams& p = OccParams()) : p_(p)
  {
    nx_ = static_cast<int>(std::ceil((p_.x1 - p_.x0) / p_.res));
    ny_ = static_cast<int>(std::ceil((p_.y1 - p_.y0) / p_.res));
    v_.assign(static_cast<size_t>(nx_) * ny_, 0);
  }

  void clear() { std::fill(v_.begin(), v_.end(), 0); }

  // Sensor at (sx, sy, sth) in the local frame when the scan was taken. Beams that are NaN (unknown)
  // or flagged (intensity > 0.5: camera inner ring, true range unknown) change nothing.
  void update(double sx, double sy, double sth, const sensor_msgs::LaserScan& m, double max_range)
  {
    const int n = m.ranges.size();
    if (n == 0)
      return;
    auto usable = [&](int k, double& r) {
      r = m.ranges[k];
      if (std::isnan(r) || (k < static_cast<int>(m.intensities.size()) && m.intensities[k] > 0.5))
        return false;
      if (!std::isfinite(r) || r > max_range)
        r = std::numeric_limits<double>::infinity();
      return true;
    };
    for (int k = 0; k < n; ++k)
    {
      double r;
      if (!usable(k, r))
        continue;
      // clear only where this beam and both neighbours see through: corners of an obstacle sit next
      // to beams that just miss them
      double reach = r;
      for (int dk : {-1, 1})
      {
        double rn;
        reach = usable((k + dk + n) % n, rn) ? std::min(reach, rn) : 0.0;
      }
      reach = std::min(reach, max_range) - p_.clear_margin;
      const double a = sth + m.angle_min + k * m.angle_increment;
      const double ca = std::cos(a), sa = std::sin(a);
      long last = -1;                                   // each cell once per beam
      for (double d = 0.5 * p_.res; d < reach; d += 0.5 * p_.res)
      {
        const long cell = static_cast<long>(row(sy + sa * d)) * nx_ + col(sx + ca * d);
        if (cell != last)
          add(sx + ca * d, sy + sa * d, -p_.miss);
        last = cell;
      }
      if (std::isfinite(r))
        add(sx + ca * r, sy + sa * r, p_.hit);
    }
  }

  // Centres of occupied cells (local frame); within `radius` of (cx, cy) if radius > 0.
  std::vector<Pt> occupied(double cx = 0, double cy = 0, double radius = -1) const
  {
    std::vector<Pt> out;
    int r0 = 0, r1 = ny_ - 1, c0 = 0, c1 = nx_ - 1;
    if (radius > 0)
    {
      c0 = std::max(0, col(cx - radius));
      c1 = std::min(nx_ - 1, col(cx + radius));
      r0 = std::max(0, row(cy - radius));
      r1 = std::min(ny_ - 1, row(cy + radius));
    }
    for (int r = r0; r <= r1; ++r)
      for (int c = c0; c <= c1; ++c)
        if (v_[static_cast<size_t>(r) * nx_ + c] >= p_.occupied_level)
          out.push_back({p_.x0 + (c + 0.5) * p_.res, p_.y0 + (r + 0.5) * p_.res});
    return out;
  }

  // Recovery: forget everything farther than radius from (cx, cy); it will be re-observed.
  void clearOutside(double cx, double cy, double radius)
  {
    for (int r = 0; r < ny_; ++r)
      for (int c = 0; c < nx_; ++c)
        if (std::hypot(p_.x0 + (c + 0.5) * p_.res - cx, p_.y0 + (r + 0.5) * p_.res - cy) > radius)
          v_[static_cast<size_t>(r) * nx_ + c] = 0;
  }

private:
  int col(double x) const { return static_cast<int>(std::floor((x - p_.x0) / p_.res)); }
  int row(double y) const { return static_cast<int>(std::floor((y - p_.y0) / p_.res)); }
  void add(double x, double y, int delta)
  {
    const int c = col(x), r = row(y);
    if (c < 0 || c >= nx_ || r < 0 || r >= ny_)
      return;
    int8_t& v = v_[static_cast<size_t>(r) * nx_ + c];
    v = static_cast<int8_t>(std::max(p_.min_level, std::min(p_.max_level, v + delta)));
  }

  OccParams p_;
  int nx_, ny_;
  std::vector<int8_t> v_;
};
}  // namespace avoid_nav
