// Obstacle layer in the local (odometry) frame, costmap style: every scan marks the cells it hits
// and clears the cells its beams pass through, as close to the hit as the reading's precision allows. Nothing expires with time: what is out of view
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
  // A hit is stronger than a pass-through: grazing beams must not erase. A precise hit marks an
  // obstacle at once; a coarse one (far camera reading, several cm per pixel) needs a second.
  int hit_precise = 3, hit_coarse = 1, miss = 1;
  double coarse_sigma = 0.03;     // [m] readings with a larger range uncertainty are coarse
  int max_level = 8, min_level = -2;
  int occupied_level = 2;         // >= this is an obstacle
  // A beam clears cells up to clear_sigmas range uncertainties short of its hit (at least
  // clear_margin). Near readings are precise, so they clear the noise far readings left in front
  // of a surface; a fixed 10 cm kept every such ghost forever (obstacles grew 4-10 cm).
  double clear_sigmas = 2.0;
  double clear_margin = 0.02;
  double clear_max = 0.10;        // never stop further short than this (far readings: no worse than before)
  // A beam next to one that hits does not clear within corner_protect of that hit: protects the
  // corners of real obstacles from grazing beams. (Limiting the whole beam to the neighbour's range
  // instead left the noise next to every corner, "tails" that closed passable 50 cm gaps.)
  double corner_protect = 0.03;
  double sigma_const = 0.02;      // [m] range uncertainty when no table is set (lidar)
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

  // Range uncertainty [m] per centimetre of range (index = range in cm); empty = sigma_const.
  void setSigmaTable(const std::vector<float>& sigma_by_cm) { sigma_cm_ = sigma_by_cm; }
  double sigma(double range) const
  {
    if (sigma_cm_.empty() || !std::isfinite(range))
      return p_.sigma_const;
    const size_t i = std::min(sigma_cm_.size() - 1, static_cast<size_t>(std::max(0.0, range * 100.0)));
    return sigma_cm_[i];
  }

  // Sensor at (sx, sy, sth) in the local frame when the scan was taken. Beams that are NaN (unknown)
  // or flagged (intensity > 0.5: camera inner ring, true range unknown) change nothing.
  // mark_from: hits closer than this only clear (the far layer: the precise grid has them).
  void update(double sx, double sy, double sth, const sensor_msgs::LaserScan& m, double max_range,
              double mark_from = 0.0)
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
      // neighbours: unknown (NaN, camera inner ring) -> clear nothing beside them; a hit protects the
      // cells around it (an obstacle corner between the two beams)
      double reach = std::min(r, max_range);
      double px[2], py[2];
      int np = 0;
      for (int dk : {-1, 1})
      {
        const int j = (k + dk + n) % n;
        double rn;
        if (!usable(j, rn))
          reach = 0.0;
        else if (std::isfinite(rn))
        {
          const double aj = sth + m.angle_min + j * m.angle_increment;
          px[np] = sx + std::cos(aj) * rn;
          py[np] = sy + std::sin(aj) * rn;
          ++np;
        }
      }
      if (reach > 0)
        reach -= std::min(p_.clear_max, std::max(p_.clear_margin, p_.clear_sigmas * sigma(reach)));
      const double a = sth + m.angle_min + k * m.angle_increment;
      const double ca = std::cos(a), sa = std::sin(a);
      const double prot2 = p_.corner_protect * p_.corner_protect;
      long last = -1;                                   // each cell once per beam
      for (double d = 0.5 * p_.res; d < reach; d += 0.5 * p_.res)
      {
        const double cx = sx + ca * d, cy = sy + sa * d;
        const long cell = static_cast<long>(row(cy)) * nx_ + col(cx);
        if (cell == last)
          continue;
        last = cell;
        // centre of this cell, compared with the neighbours' hits
        const double ccx = p_.x0 + (col(cx) + 0.5) * p_.res, ccy = p_.y0 + (row(cy) + 0.5) * p_.res;
        bool prot = false;
        for (int q = 0; q < np && !prot; ++q)
          prot = (ccx - px[q]) * (ccx - px[q]) + (ccy - py[q]) * (ccy - py[q]) < prot2;
        if (!prot)
          add(cx, cy, -p_.miss);
      }
      if (std::isfinite(r) && r >= mark_from)
        add(sx + ca * r, sy + sa * r, sigma(r) <= p_.coarse_sigma ? p_.hit_precise : p_.hit_coarse);
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
  std::vector<float> sigma_cm_;
};
}  // namespace avoid_nav
