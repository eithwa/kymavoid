// Mirror model of vision/black_item (NodeHandle::camera_f / Omni_distance).
#pragma once
#include <cmath>

namespace avoid_nav
{
// Floor distance [cm] from the robot centre for a pixel radius; 999 when out of the model's range.
inline double omniDistanceCm(double pixel, double camera_high_mm, double outer_px)
{
  const double m = (outer_px * 2 * 0.9784 * 0.0099) / 60.0;
  const double f = 180.0 / (1.0 + 1.0 / m);
  const double c = 83.125, b = c * 0.8722;
  const double r = std::atan2(f, pixel * 0.0099);
  const double d = -camera_high_mm * (b * b - c * c) * std::cos(r) / ((b * b + c * c) * std::sin(r) - 2 * b * c) * 0.1;
  return (d < 0 || d > 999) ? 999.0 : d;
}

// Inverse of omniDistanceCm (bisection; the model is monotonic in [inner, ~160 px]).
inline double omniPixel(double distance_cm, double camera_high_mm, double outer_px)
{
  double lo = 0.0, hi = 200.0;
  for (int i = 0; i < 50; ++i)
  {
    const double mid = 0.5 * (lo + hi);
    (omniDistanceCm(mid, camera_high_mm, outer_px) < distance_cm ? lo : hi) = mid;
  }
  return 0.5 * (lo + hi);
}
}  // namespace avoid_nav
