// Footprint polygon helpers (base frame, metres).
#pragma once
#include <ros/ros.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include <algorithm>
#include <limits>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace avoid_nav
{
struct Pt
{
  double x, y;
};
using Polygon = std::vector<Pt>;

// Footprint from a parameter holding [[x, y], ...] (robot6th_footprint.yaml loaded into /robot).
inline Polygon loadPolygon(const ros::NodeHandle& nh, const std::string& name)
{
  XmlRpc::XmlRpcValue v;
  if (!nh.getParam(name, v) || v.getType() != XmlRpc::XmlRpcValue::TypeArray || v.size() < 3)
    throw std::runtime_error("parameter " + name + " is not a polygon [[x, y], ...]");
  Polygon p;
  for (int i = 0; i < v.size(); ++i)
    p.push_back({static_cast<double>(v[i][0]), static_cast<double>(v[i][1])});
  return p;
}

inline bool inside(const Polygon& poly, const Pt& p)
{
  bool in = false;
  for (size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++)
  {
    const Pt& a = poly[i];
    const Pt& b = poly[j];
    if (((a.y > p.y) != (b.y > p.y)) && (p.x < (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x))
      in = !in;
  }
  return in;
}

inline double distanceToEdges(const Polygon& poly, const Pt& p)
{
  double best = std::numeric_limits<double>::infinity();
  for (size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++)
  {
    const Pt& a = poly[j];
    const Pt& b = poly[i];
    const double dx = b.x - a.x, dy = b.y - a.y, l2 = dx * dx + dy * dy;
    double t = l2 > 0 ? ((p.x - a.x) * dx + (p.y - a.y) * dy) / l2 : 0.0;
    t = std::max(0.0, std::min(1.0, t));
    best = std::min(best, std::hypot(p.x - (a.x + t * dx), p.y - (a.y + t * dy)));
  }
  return best;
}

// Inside the polygon or within `margin` of it.
inline bool insideOrNear(const Polygon& poly, const Pt& p, double margin)
{
  return inside(poly, p) || distanceToEdges(poly, p) <= margin;
}
}  // namespace avoid_nav
