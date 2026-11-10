// camera_adapter: black_item output -> REP 103 laser scans.
//
//   /vision/BlackRealDis (120 x cm, 999 = none) -> /scan      obstacles, base_link
//   /vision/redRealDis                          -> /boundary  red line points, base_link
//
// Scan line i points index_direction * 3 i degrees from the robot front (camera.yaml).
//   999                 -> +inf  (nothing up to the camera's range)
//   mirror strut lines  -> NaN   (unknown: black_item reports the strut, not the world)
//   inner-ring value    -> kept, intensity 1: black_item reports anything closer than the inner
//                          ring (~32 cm) at the ring, so the true distance may be smaller
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int32MultiArray.h>

#include <cmath>
#include <limits>
#include <vector>

#include "avoid_nav/camera_model.h"

namespace avoid_nav
{
class CameraAdapter
{
public:
  CameraAdapter(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  {
    int inner = 40, outer = 229;
    double cam_high = 654.0;
    nh.param("/camera/scan_step_deg", step_deg_, 3);
    nh.param("/camera/index_direction", dir_, 1);
    // on the real robot black_item's own parameters are the truth: the distances it publishes were
    // computed with them, so the inner-ring value must match exactly
    if (loadCameraModel(nh, cam_high, inner, outer))
      ROS_INFO("camera_adapter: using black_item's mirror parameters (/FIRA/vision/Center)");
    std::vector<double> struts;
    nh.param("/camera/strut_angles_deg", struts, std::vector<double>());
    pnh.param("strut_mask_half_deg", strut_half_deg_, 2.0);
    pnh.param("frame_id", frame_, std::string("base_link"));
    nh.param("/camera/latency", latency_, 0.03);          // shared with avoid_sim's virtual camera
    pnh.param("latency", latency_, latency_);
    // black_item reports the first marked pixel, so on average half a pixel too far: undo that
    pnh.param("quantization_correction_px", quant_px_, 0.5);
    inner_range_ = omniDistanceCm(inner, cam_high, outer) / 100.0;
    cam_high_ = cam_high;
    outer_ = outer;
    n_ = 360 / step_deg_;
    masked_.assign(n_, false);
    for (int i = 0; i < n_; ++i)
      for (double s : struts)
      {
        double d = std::fmod(i * step_deg_ - s + 540.0, 360.0) - 180.0;
        if (std::fabs(d) <= strut_half_deg_)
          masked_[i] = true;
      }
    int n_masked = 0;
    for (bool m : masked_)
      n_masked += m;
    ROS_INFO("camera_adapter: %d scan lines, %d masked by struts, inner ring %.3f m", n_, n_masked, inner_range_);
    scan_pub_ = nh.advertise<sensor_msgs::LaserScan>("scan", 5);
    bound_pub_ = nh.advertise<sensor_msgs::LaserScan>("boundary", 5);
    black_sub_ = nh.subscribe("/vision/BlackRealDis", 5, &CameraAdapter::onBlack, this);
    red_sub_ = nh.subscribe("/vision/redRealDis", 5, &CameraAdapter::onRed, this);
  }

private:
  sensor_msgs::LaserScan convert(const std_msgs::Int32MultiArray& m, bool obstacles) const
  {
    sensor_msgs::LaserScan s;
    // black_item does not stamp its output: now - latency. Sim time starts at 0 and ros::Time cannot
    // be negative (it throws), so not before that.
    const ros::Time now = ros::Time::now();
    s.header.stamp = now.toSec() > latency_ ? now - ros::Duration(latency_) : now;
    s.header.frame_id = frame_;
    const double inc = dir_ * step_deg_ * M_PI / 180.0;
    s.angle_min = 0.0;
    s.angle_increment = inc;
    s.angle_max = inc * (n_ - 1);
    s.range_min = 0.0;
    s.range_max = 9.98;
    s.ranges.assign(n_, std::numeric_limits<float>::quiet_NaN());
    s.intensities.assign(n_, 0.0f);
    const int n = std::min<int>(n_, m.data.size());
    for (int i = 0; i < n; ++i)
    {
      const int v = m.data[i];
      if (obstacles && masked_[i])
        continue;                                                     // NaN: unknown
      if (v >= 999)
        s.ranges[i] = std::numeric_limits<float>::infinity();
      else
      {
        const bool clamped = v / 100.0 <= inner_range_ + 0.01;
        if (obstacles && clamped)
          s.intensities[i] = 1.0f;                                    // clamped at the inner ring
        double cm = v;
        if (quant_px_ > 0 && !clamped)
          cm = omniDistanceCm(omniPixel(v, cam_high_, outer_) - quant_px_, cam_high_, outer_);
        s.ranges[i] = cm / 100.0f;
      }
    }
    return s;
  }

  void onBlack(const std_msgs::Int32MultiArray& m) { scan_pub_.publish(convert(m, true)); }
  void onRed(const std_msgs::Int32MultiArray& m) { bound_pub_.publish(convert(m, false)); }

  int step_deg_ = 3, dir_ = 1, n_ = 120;
  double strut_half_deg_ = 2.0, latency_ = 0.03, inner_range_ = 0.32, quant_px_ = 0.5, cam_high_ = 654, outer_ = 229;
  std::string frame_;
  std::vector<bool> masked_;
  ros::Publisher scan_pub_, bound_pub_;
  ros::Subscriber black_sub_, red_sub_;
};
}  // namespace avoid_nav

int main(int argc, char** argv)
{
  ros::init(argc, argv, "camera_adapter");
  ros::NodeHandle nh, pnh("~");
  avoid_nav::CameraAdapter adapter(nh, pnh);
  ros::spin();
  return 0;
}
