// Mirror model of vision/black_item (NodeHandle::camera_f / Omni_distance).
#pragma once
#include <ros/node_handle.h>

#include <algorithm>
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

// Range uncertainty [m] of a reading at range_m: sigma_px pixels of the mirror model there. The
// resolution falls fast with range (1 cm per pixel at 0.4 m, 5 cm at 2 m, 7.5 cm at 2.5 m).
inline double omniRangeSigma(double range_m, double camera_high_mm, double outer_px, double sigma_px)
{
  const double p = omniPixel(range_m * 100.0, camera_high_mm, outer_px);
  const double d0 = omniDistanceCm(p, camera_high_mm, outer_px), d1 = omniDistanceCm(p + 1.0, camera_high_mm, outer_px);
  if (d1 >= 999.0 || d0 >= 999.0)
    return 1.0;                                   // beyond the model: useless for clearing
  return sigma_px * std::max(0.0, d1 - d0) / 100.0;
}

// Mirror parameters as black_item used them: its own (/FIRA/vision/Center) on the real robot, where
// the distances it publishes were computed with them, otherwise /camera (camera.yaml, simulation).
inline bool loadCameraModel(ros::NodeHandle& nh, double& cam_high, int& inner, int& outer)
{
  nh.param("/camera/camera_high", cam_high, 654.0);
  nh.param("/camera/inner", inner, 40);
  nh.param("/camera/outer", outer, 229);
  if (!nh.hasParam("/FIRA/vision/Center/Inner"))
    return false;
  nh.getParam("/FIRA/vision/Center/Camera_high", cam_high);
  nh.getParam("/FIRA/vision/Center/Inner", inner);
  nh.getParam("/FIRA/vision/Center/Outer", outer);
  return true;
}
}  // namespace avoid_nav
