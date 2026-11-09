// lidar_adapter: three Hokuyo scans -> one 360 deg scan in base_link, without the robot itself.
//
// The real lidars also hit the robot (brackets, cables, parts that are not in the CAD). Any point
// inside the footprint polygon or within self_margin of it is the robot: an obstacle cannot be
// inside the robot, so this needs no knowledge of which part was hit.
//
// Output bins (REP 117):
//   obstacle seen         -> nearest range
//   only free rays        -> +inf
//   only self hits / none -> NaN (unknown: the robot or a gap in coverage hides that direction)
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int32.h>
#include <tf2_ros/transform_listener.h>

#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "avoid_nav/polygon.h"

namespace avoid_nav
{
struct Source
{
  std::string topic;
  bool have_tf = false;
  double tx = 0, ty = 0, yaw = 0;          // laser frame in base_link
  ros::Time stamp;
  std::vector<Pt> obstacles, free_dirs;    // base_link points; free_dirs = far end of rays without a return
  int self_hits = 0;
  ros::Subscriber sub;
};

class LidarAdapter
{
public:
  LidarAdapter(ros::NodeHandle& nh, ros::NodeHandle& pnh) : tf_listener_(tf_buffer_)
  {
    std::vector<std::string> topics;
    pnh.param("inputs", topics, std::vector<std::string>{"scan_0", "scan_1", "scan_2"});
    pnh.param("frame_id", frame_, std::string("base_link"));
    pnh.param("beams", beams_, 720);
    pnh.param("range_max", range_max_, 8.0);
    pnh.param("self_margin", self_margin_, 0.03);
    pnh.param("max_age", max_age_, 0.25);
    footprint_ = loadPolygon(nh, "/robot/footprint");
    pub_ = nh.advertise<sensor_msgs::LaserScan>("scan", 5);
    self_pub_ = pnh.advertise<std_msgs::Int32>("self_hits", 5);
    sources_.resize(topics.size());
    for (size_t i = 0; i < topics.size(); ++i)
    {
      sources_[i].topic = topics[i];
      sources_[i].sub = nh.subscribe<sensor_msgs::LaserScan>(
          topics[i], 5, [this, i](const sensor_msgs::LaserScan::ConstPtr& m) { onScan(i, *m); });
    }
    ROS_INFO("lidar_adapter: %zu inputs, %d beams, self margin %.3f m, footprint %zu points", topics.size(), beams_,
             self_margin_, footprint_.size());
  }

private:
  bool lookup(Source& s, const std::string& laser_frame)
  {
    if (s.have_tf)
      return true;
    try
    {
      auto t = tf_buffer_.lookupTransform(frame_, laser_frame, ros::Time(0), ros::Duration(0.1));
      const auto& q = t.transform.rotation;
      s.tx = t.transform.translation.x;
      s.ty = t.transform.translation.y;
      s.yaw = std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
      s.have_tf = true;                                   // lidars are bolted on: static
    }
    catch (const tf2::TransformException& e)
    {
      ROS_WARN_THROTTLE(2.0, "lidar_adapter: no TF %s -> %s yet: %s", laser_frame.c_str(), frame_.c_str(), e.what());
    }
    return s.have_tf;
  }

  void onScan(size_t idx, const sensor_msgs::LaserScan& m)
  {
    Source& s = sources_[idx];
    if (!lookup(s, m.header.frame_id))
      return;
    s.stamp = m.header.stamp;
    s.obstacles.clear();
    s.free_dirs.clear();
    s.self_hits = 0;
    const double c = std::cos(s.yaw), sn = std::sin(s.yaw);
    for (size_t k = 0; k < m.ranges.size(); ++k)
    {
      const double a = m.angle_min + k * m.angle_increment;
      const double r = m.ranges[k];
      const bool ret = std::isfinite(r) && r >= m.range_min && r < m.range_max;
      const double rr = ret ? r : range_max_;
      const double lx = rr * std::cos(a), ly = rr * std::sin(a);
      const Pt p{s.tx + c * lx - sn * ly, s.ty + sn * lx + c * ly};
      if (!ret)
      {
        if (!std::isnan(r))
          s.free_dirs.push_back(p);
        continue;
      }
      if (insideOrNear(footprint_, p, self_margin_))
      {
        ++s.self_hits;
        continue;
      }
      s.obstacles.push_back(p);
    }
    publish(m.header.stamp);
  }

  int bin(const Pt& p) const
  {
    int b = static_cast<int>(std::floor((std::atan2(p.y, p.x) + M_PI) / (2 * M_PI) * beams_));
    return std::min(std::max(b, 0), beams_ - 1);
  }

  void publish(const ros::Time& stamp)
  {
    std::vector<float> ranges(beams_, std::numeric_limits<float>::quiet_NaN());
    std::vector<bool> free(beams_, false);
    int self_hits = 0;
    for (const Source& s : sources_)
    {
      if (s.stamp.isZero() || (stamp - s.stamp).toSec() > max_age_)
        continue;
      self_hits += s.self_hits;
      for (const Pt& p : s.free_dirs)
        free[bin(p)] = true;
      for (const Pt& p : s.obstacles)
      {
        const int b = bin(p);
        const float r = std::hypot(p.x, p.y);
        if (!(ranges[b] <= r))                            // NaN or farther
          ranges[b] = r;
      }
    }
    for (int b = 0; b < beams_; ++b)
      if (std::isnan(ranges[b]) && free[b])
        ranges[b] = std::numeric_limits<float>::infinity();

    sensor_msgs::LaserScan out;
    out.header.stamp = stamp;
    out.header.frame_id = frame_;
    out.angle_min = -M_PI;
    out.angle_increment = 2 * M_PI / beams_;
    out.angle_max = out.angle_min + out.angle_increment * (beams_ - 1);
    out.range_min = 0.0;
    out.range_max = range_max_;
    out.ranges = ranges;
    pub_.publish(out);
    std_msgs::Int32 n;
    n.data = self_hits;
    self_pub_.publish(n);
  }

  std::string frame_;
  int beams_ = 720;
  double range_max_ = 8.0, self_margin_ = 0.03, max_age_ = 0.25;
  Polygon footprint_;
  std::vector<Source> sources_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  ros::Publisher pub_, self_pub_;
};
}  // namespace avoid_nav

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lidar_adapter");
  ros::NodeHandle nh, pnh("~");
  avoid_nav::LidarAdapter adapter(nh, pnh);
  ros::spin();
  return 0;
}
