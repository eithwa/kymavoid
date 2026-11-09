// nav_node: state machine, localisation, planning, speed planning and the safety layer.
//
// States
//   IDLE --start--> MAPPING (lidar: stand still, map the start) --> RUNNING
//   IDLE --start--> RUNNING (camera)
//   RUNNING --footprint touches the goal line--> FINISHED (brake to a stop)
//   active --stale required input / no path for too long--> SAFE_STOP (stopped, reason reported)
//   stop / reset --> IDLE
//
// Frames
//   local (L): wheel odometry + IMU heading only, never corrected. Obstacle memory lives here, so a
//              localisation correction cannot move obstacles relative to the robot.
//   field (F): EKF estimate (red-line corrections). Red lines, goal line and the wall live here.
//   Each cycle the obstacles are mapped L -> F with the same transform as the robot, so their
//   geometry relative to the robot is exactly what the sensors saw.
#include <avoid_msgs/Command.h>
#include <avoid_msgs/Status.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <tf2_ros/transform_broadcaster.h>

#include <cmath>
#include <sstream>
#include <string>
#include <deque>
#include <vector>

#include "avoid_nav/localizer.h"
#include "avoid_nav/occupancy.h"
#include "avoid_nav/planner.h"
#include "avoid_nav/polygon.h"

namespace avoid_nav
{
struct Pose2
{
  double x = 0, y = 0, th = 0;
};
inline Pt apply(const Pose2& T, const Pt& p)
{
  const double c = std::cos(T.th), s = std::sin(T.th);
  return {T.x + c * p.x - s * p.y, T.y + s * p.x + c * p.y};
}
inline Pose2 compose(const Pose2& a, const Pose2& b)
{
  const Pt t = apply(a, {b.x, b.y});
  return {t.x, t.y, wrapAngle(a.th + b.th)};
}
inline Pose2 inverse(const Pose2& a)
{
  const double c = std::cos(a.th), s = std::sin(a.th);
  return {-(c * a.x + s * a.y), -(-s * a.x + c * a.y), -a.th};
}

struct Input
{
  std::string name;
  double timeout;
  bool required;
  ros::Time last;
  double age(const ros::Time& now) const { return last.isZero() ? -1.0 : (now - last).toSec(); }
  bool fresh(const ros::Time& now) const { return !last.isZero() && (now - last).toSec() <= timeout; }
};


class NavNode
{
public:
  NavNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  {
    pnh.param("sensor_config", sensor_, std::string("camera"));
    pnh.param("rate", rate_, 20.0);
    pnh.param("plan_rate", plan_rate_, 10.0);
    pnh.param("mapping_duration", mapping_duration_, 1.0);
    double t_obs, t_odom, t_imu, t_bound;
    pnh.param("timeout_obstacles", t_obs, 0.3);
    pnh.param("timeout_odom", t_odom, 0.2);
    pnh.param("timeout_imu", t_imu, 0.3);
    pnh.param("timeout_boundary", t_bound, 0.3);
    obstacles_ = {"obstacles", t_obs, true, ros::Time()};
    odom_ = {"odom", t_odom, true, ros::Time()};
    imu_ = {"imu", t_imu, true, ros::Time()};
    boundary_ = {"boundary", t_bound, false, ros::Time()};
    // control
    pnh.param("v_max", v_max_, 0.8);
    pnh.param("a_max", a_max_, 1.0);
    pnh.param("a_brake", a_brake_, 1.2);
    pnh.param("stop_margin", stop_margin_, 0.03);
    pnh.param("v_tight", v_tight_, 0.15);          // speed with zero clearance
    pnh.param("v_per_clearance", v_per_clear_, 3.0);  // + this x clearance [m/s per m]
    pnh.param("lookahead", lookahead_, 0.35);
    pnh.param("k_heading", k_heading_, 1.5);
    pnh.param("w_max", w_max_, 1.0);
    pnh.param("safety_margin", safety_margin_, 0.03);
    pnh.param("red_line_max_range", red_max_range_, 1.5);  // real images: red lines unreliable beyond ~1 m
    pnh.param("wall_max_range", wall_max_range_, 2.5);
    pnh.param("max_obstacle_range", max_obs_range_, sensor_ == "camera" ? 2.5 : 6.0);
    pnh.param("line_margin_base", line_margin_base_, 0.02);
    pnh.param("line_margin_sigma", line_margin_sigma_, 2.0);
    pnh.param("no_path_timeout", no_path_timeout_, 3.0);
    pnh.param("clear_margin", clear_margin_, 0.10);
    OccParams op;
    op.clear_margin = clear_margin_;
    grid_ = OccGrid(op);
    pnh.param("recovery_after", recovery_after_, 1.0);

    // field + robot
    LocalizerParams lp;
    const double red_y = nh.param("/field/red_line_y", 1.5), red_w = nh.param("/field/red_line_width", 0.05);
    const double red_x0 = nh.param("/field/red_line_x_min", -3.0), red_x1 = nh.param("/field/red_line_x_max", 3.0);
    keep_.red_inner = red_y - red_w / 2;
    keep_.goal_x = nh.param("/field/goal_line_x", 3.0);
    const double wall_x = nh.param("/field/wall_x", 3.55), wall_y = nh.param("/field/wall_y", 2.55);
    const double wall_t = nh.param("/field/wall_thickness", 0.15);
    keep_.front_wall_x = wall_x - wall_t / 2;
    keep_.back_wall_x = -(wall_x - wall_t / 2);
    // known lines of the field, n . p = c with the course on the n . p < c side
    const double rx = std::max(std::fabs(red_x0), std::fabs(red_x1));
    red_lines_ = {{"red_left", 0, 1, keep_.red_inner, -rx, rx}, {"red_right", 0, -1, keep_.red_inner, -rx, rx}};
    walls_ = {{"wall_front", 1, 0, wall_x - wall_t / 2, -wall_y, wall_y},
              {"wall_back", -1, 0, wall_x - wall_t / 2, -wall_y, wall_y},
              {"wall_left", 0, 1, wall_y - wall_t / 2, -wall_x, wall_x},
              {"wall_right", 0, -1, wall_y - wall_t / 2, -wall_x, wall_x}};
    start_.x = nh.param("/field/start_pose/x", -2.0);
    start_.y = nh.param("/field/start_pose/y", 0.0);
    start_.th = nh.param("/field/start_pose/yaw", 0.0);
    loc_ = Localizer(lp);
    loc_.reset(start_.x, start_.y, start_.th);
    footprint_ = loadPolygon(nh, "/robot/footprint");
    planner_.setFootprint(footprint_);

    cmd_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 5);
    status_pub_ = nh.advertise<avoid_msgs::Status>("avoid/status", 5, true);
    pose_pub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("avoid/pose", 5);
    path_pub_ = nh.advertise<nav_msgs::Path>("avoid/path", 2);
    debug_pub_ = nh.advertise<std_msgs::String>("avoid/debug", 5);
    obstacles_pub_ = nh.advertise<sensor_msgs::PointCloud>("avoid/obstacles", 2);
    cspace_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("avoid/cspace", 1);
    subs_.push_back(nh.subscribe("scan", 5, &NavNode::onScan, this));
    subs_.push_back(nh.subscribe("boundary", 5, &NavNode::onBoundary, this));
    subs_.push_back(nh.subscribe("odom", 20, &NavNode::onOdom, this));
    subs_.push_back(nh.subscribe("imu/yaw", 20, &NavNode::onImu, this));
    cmd_srv_ = nh.advertiseService("avoid/command", &NavNode::onCommand, this);
    timer_ = nh.createTimer(ros::Duration(1.0 / rate_), &NavNode::tick, this);
    reason_ = "ready";
    ROS_INFO("nav_node: sensor %s, v_max %.2f m/s, footprint %zu points", sensor_.c_str(), v_max_, footprint_.size());
  }

private:
  enum State : uint8_t
  {
    IDLE = avoid_msgs::Status::IDLE,
    MAPPING = avoid_msgs::Status::MAPPING,
    RUNNING = avoid_msgs::Status::RUNNING,
    FINISHED = avoid_msgs::Status::FINISHED,
    SAFE_STOP = avoid_msgs::Status::SAFE_STOP
  };
  static const char* name(uint8_t s)
  {
    switch (s)
    {
      case IDLE: return "IDLE";
      case MAPPING: return "MAPPING";
      case RUNNING: return "RUNNING";
      case FINISHED: return "FINISHED";
      default: return "SAFE_STOP";
    }
  }
  bool active() const { return state_ == MAPPING || state_ == RUNNING || state_ == FINISHED; }

  // ---------------------------------------------------------------- inputs
  void onImu(const std_msgs::Float64& m)
  {
    imu_.last = ros::Time::now();
    imu_yaw_ = m.data;
    have_imu_ = true;
  }

  void onOdom(const nav_msgs::Odometry& m)
  {
    const ros::Time now = ros::Time::now();
    odom_.last = now;
    const auto& q = m.pose.pose.orientation;
    odom_pose_ = {m.pose.pose.position.x, m.pose.pose.position.y,
                  std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))};
    if (!have_imu_)
      return;
    if (!last_odom_.isZero())
    {
      const double dt = (now - last_odom_).toSec();
      if (dt > 0 && dt < 0.5)
      {
        const double vx = m.twist.twist.linear.x, vy = m.twist.twist.linear.y;
        const double dth = wrapAngle(imu_yaw_ - imu_at_last_odom_);
        // dead reckoning (local frame): heading straight from the IMU, odometry scale as estimated
        // by the EKF (changes smoothly, so the local frame never jumps)
        const double th = wrapAngle(start_.th + imu_yaw_ - imu0_);
        const double vxs = vx * (1 + loc_.scaleX()), vys = vy * (1 + loc_.scaleY());
        local_.x += (std::cos(th) * vxs - std::sin(th) * vys) * dt;
        local_.y += (std::sin(th) * vxs + std::cos(th) * vys) * dt;
        local_.th = th;
        loc_.predict(vx, vy, dth, dt);
      }
    }
    last_odom_ = now;
    imu_at_last_odom_ = imu_yaw_;
    history_.push_back({now.toSec(), local_});
    while (history_.size() > 2 && now.toSec() - history_.front().first > 2.0)
      history_.pop_front();
  }

  void onScan(const sensor_msgs::LaserScan& m)
  {
    const ros::Time now = ros::Time::now();
    obstacles_.last = now;
    if (!active())
      return;
    const double t = now.toSec();
    // the sensor saw the world from where the robot was at the scan stamp, not from where it is now
    const Pose2 at_scan = localAt(m.header.stamp.isZero() ? t : m.header.stamp.toSec());
    const Pose2 now_from_scan = compose(inverse(local_), at_scan);
    grid_.update(at_scan.x, at_scan.y, at_scan.th, m, max_obs_range_);
    std::vector<Pt> frame;
    for (size_t k = 0; k < m.ranges.size(); ++k)
    {
      const double r = m.ranges[k];
      if (!std::isfinite(r) || r > max_obs_range_)
        continue;                                   // NaN unknown, inf free
      if (k < m.intensities.size() && m.intensities[k] > 0.5)
        continue;                                   // camera inner ring: true distance unknown (memory has it)
      const double a = m.angle_min + k * m.angle_increment;
      frame.push_back(apply(now_from_scan, {r * std::cos(a), r * std::sin(a)}));
    }
    if (sensor_ == "camera")                          // the camera sees the black low wall; lidars do not
      countFits(loc_.updateLines(frame, walls_, wall_max_range_), walls_accepted_, walls_rejected_);
  }

  // local pose at time t (linear interpolation in the odometry history)
  Pose2 localAt(double t) const
  {
    if (history_.empty() || t >= history_.back().first)
      return local_;
    if (t <= history_.front().first)
      return history_.front().second;
    for (size_t i = history_.size() - 1; i > 0; --i)
    {
      const auto& a = history_[i - 1];
      const auto& b = history_[i];
      if (a.first <= t)
      {
        const double w = (b.first > a.first) ? (t - a.first) / (b.first - a.first) : 1.0;
        return {a.second.x + w * (b.second.x - a.second.x), a.second.y + w * (b.second.y - a.second.y),
                wrapAngle(a.second.th + w * wrapAngle(b.second.th - a.second.th))};
      }
    }
    return local_;
  }

  static void countFits(const std::vector<LineFit>& fits, int& acc, int& rej)
  {
    for (const LineFit& f : fits)
      if (f.ok)
        (f.accepted ? acc : rej)++;
  }

  void onBoundary(const sensor_msgs::LaserScan& m)
  {
    boundary_.last = ros::Time::now();
    if (!active() || sensor_ != "camera")
      return;
    const double t = ros::Time::now().toSec();
    const Pose2 now_from_scan = compose(inverse(local_), localAt(m.header.stamp.isZero() ? t : m.header.stamp.toSec()));
    std::vector<Pt> pts;
    for (size_t k = 0; k < m.ranges.size(); ++k)
    {
      const double r = m.ranges[k];
      if (!std::isfinite(r))
        continue;
      const double a = m.angle_min + k * m.angle_increment;
      pts.push_back(apply(now_from_scan, {r * std::cos(a), r * std::sin(a)}));
    }
    countFits(loc_.updateLines(pts, red_lines_, red_max_range_), lines_accepted_, lines_rejected_);
  }

  // ---------------------------------------------------------------- commands
  std::string staleRequired(const ros::Time& now) const
  {
    std::ostringstream os;
    for (const Input* in : {&obstacles_, &odom_, &imu_})
      if (in->required && !in->fresh(now))
      {
        os << (os.tellp() > 0 ? ", " : "") << in->name;
        if (in->last.isZero())
          os << " (never received)";
        else
          os << " (" << static_cast<int>(in->age(now) * 1000) << " ms old)";
      }
    return os.str();
  }

  void setState(uint8_t s, const std::string& reason)
  {
    if (s != state_)
      ROS_INFO("nav_node: %s -> %s: %s", name(state_), name(s), reason.c_str());
    state_ = s;
    reason_ = reason;
  }

  bool onCommand(avoid_msgs::Command::Request& req, avoid_msgs::Command::Response& res)
  {
    const ros::Time now = ros::Time::now();
    if (req.command == "start")
    {
      if (state_ == MAPPING || state_ == RUNNING)
      {
        res.success = false;
        res.message = std::string("already ") + name(state_);
        return true;
      }
      const std::string stale = staleRequired(now);
      if (!stale.empty())
      {
        res.success = false;
        res.message = "cannot start, stale inputs: " + stale;
        return true;
      }
      // the robot stands on the start pose: (re)anchor everything there
      loc_.reset(start_.x, start_.y, start_.th);
      local_ = start_;
      imu0_ = imu_yaw_;
      grid_.clear();
      history_.clear();
      path_.clear();
      cmd_ = geometry_msgs::Twist();
      lines_accepted_ = lines_rejected_ = walls_accepted_ = walls_rejected_ = 0;
      start_time_ = now;
      last_path_ok_ = now;
      setState(sensor_ == "lidar" ? MAPPING : RUNNING, "started");
    }
    else if (req.command == "stop")
      setState(IDLE, "stopped by command");
    else if (req.command == "reset")
      setState(IDLE, "reset");
    else
    {
      res.success = false;
      res.message = "unknown command '" + req.command + "' (start | stop | reset)";
      return true;
    }
    res.success = true;
    res.message = std::string("state ") + name(state_);
    publishStatus(now);
    return true;
  }

  // ---------------------------------------------------------------- cycle
  Pose2 fieldPose() const { return {loc_.x(), loc_.y(), loc_.theta()}; }

  void replan(const ros::Time& now)
  {
    const Pose2 F = fieldPose();
    const Pose2 T = compose(F, inverse(local_));               // local -> field
    const Pose2 base_from_local = inverse(local_);
    std::vector<Pt> obs;
    const std::vector<Pt> cells = grid_.occupied();
    occupied_cells_ = cells.size();
    obs.reserve(cells.size());
    for (const Pt& c : cells)
    {
      // an occupied cell inside the robot's own footprint is stale (odometry drift): an obstacle
      // cannot be inside the robot
      if (inside(footprint_, apply(base_from_local, c)))
        continue;
      obs.push_back(apply(T, c));
    }
    Keepout k = keep_;
    k.line_margin = line_margin_base_ + line_margin_sigma_ * std::sqrt(std::max(0.0, loc_.cov()(1, 1)));
    planner_.build(obs, F.th, k);
    path_ok_ = planner_.plan(F.x, F.y, path_);
    publishMaps(now, obs);
    if (path_ok_)
    {
      last_path_ok_ = now;
      recovered_ = false;
    }
    nav_msgs::Path p;
    p.header.stamp = now;
    p.header.frame_id = "field";
    for (const Pt& q : path_)
    {
      geometry_msgs::PoseStamped ps;
      ps.header = p.header;
      ps.pose.position.x = q.x;
      ps.pose.position.y = q.y;
      ps.pose.orientation.w = 1;
      p.poses.push_back(ps);
    }
    path_pub_.publish(p);
  }

  Pt lookaheadPoint(const Pose2& F) const
  {
    // closest point on the path, then lookahead_ further along it
    size_t seg = 0;
    double best = 1e9, best_t = 0;
    for (size_t i = 0; i + 1 < path_.size(); ++i)
    {
      const double dx = path_[i + 1].x - path_[i].x, dy = path_[i + 1].y - path_[i].y, l2 = dx * dx + dy * dy;
      double t = l2 > 0 ? ((F.x - path_[i].x) * dx + (F.y - path_[i].y) * dy) / l2 : 0;
      t = std::max(0.0, std::min(1.0, t));
      const double d = std::hypot(path_[i].x + t * dx - F.x, path_[i].y + t * dy - F.y);
      if (d < best)
      {
        best = d;
        seg = i;
        best_t = t;
      }
    }
    double left = lookahead_;
    Pt p{path_[seg].x + best_t * (path_[seg + 1].x - path_[seg].x), path_[seg].y + best_t * (path_[seg + 1].y - path_[seg].y)};
    for (size_t i = seg; i + 1 < path_.size(); ++i)
    {
      const Pt b = path_[i + 1];
      const double d = std::hypot(b.x - p.x, b.y - p.y);
      if (d >= left)
        return {p.x + (b.x - p.x) * left / d, p.y + (b.y - p.y) * left / d};
      left -= d;
      p = b;
    }
    return path_.back();
  }

  // Pure-pursuit target: the farthest point along the path, within the lookahead, that the robot can
  // reach in a straight line through free C-space. A* paths squeeze through gaps cell by cell; aiming
  // straight at a fixed lookahead point would cut the corner of a narrow gap and stall.
  Pt visibleTarget(const Pose2& F) const
  {
    const double full = lookahead_;
    Pt best = lookaheadAt(F, 0.02);
    for (double l = 0.04; l <= full + 1e-9; l += 0.02)
    {
      const Pt p = lookaheadAt(F, l);
      const double d = std::hypot(p.x - F.x, p.y - F.y);
      if (d < 1e-6)
        continue;
      if (planner_.freeDistance(F.x, F.y, (p.x - F.x) / d, (p.y - F.y) / d, d) < d - 0.01)
        break;
      best = p;
    }
    return best;
  }

  Pt lookaheadAt(const Pose2& F, double dist) const
  {
    const double keep = lookahead_;
    const_cast<NavNode*>(this)->lookahead_ = dist;
    const Pt p = lookaheadPoint(F);
    const_cast<NavNode*>(this)->lookahead_ = keep;
    return p;
  }

  // largest speed along base-frame unit (ux, uy) for which the footprint can stop before any
  // occupied cell; cells already inside the footprint are ignored (stale / drift)
  double safetySpeedLimit(double ux, double uy, double v) const
  {
    const double reach = v * v / (2 * a_brake_) + safety_margin_ + 0.05;
    const Pose2 base_from_local = inverse(local_);
    double s_hit = 1e9;
    for (const Pt& c : grid_.occupied(local_.x, local_.y, reach + 0.35))
    {
      const Pt p = apply(base_from_local, c);
      if (std::hypot(p.x, p.y) > reach + 0.35 || insideOrNear(footprint_, p, safety_margin_))
        continue;
      for (double s = 0.01; s <= reach; s += 0.01)
        if (insideOrNear(footprint_, {p.x - ux * s, p.y - uy * s}, safety_margin_))
        {
          s_hit = std::min(s_hit, s);
          break;
        }
    }
    if (s_hit > reach)
      return v;
    return std::sqrt(2 * a_brake_ * std::max(0.0, s_hit - safety_margin_));
  }

  geometry_msgs::Twist control(const Pose2& F, double dt)
  {
    geometry_msgs::Twist out;
    const double c = std::cos(F.th), s = std::sin(F.th);
    const double cur_fx = c * cmd_.linear.x - s * cmd_.linear.y, cur_fy = s * cmd_.linear.x + c * cmd_.linear.y;
    double des_fx = 0, des_fy = 0;
    if (path_ok_ && path_.size() >= 2)
    {
      const Pt t = visibleTarget(F);
      double ux = t.x - F.x, uy = t.y - F.y;
      const double n = std::hypot(ux, uy);
      if (n > 1e-6)
      {
        ux /= n;
        uy /= n;
        double v = v_max_;
        const double dfree = planner_.freeDistance(F.x, F.y, ux, uy, 3.0);
        v = std::min(v, std::sqrt(2 * a_brake_ * std::max(0.0, dfree - stop_margin_)));
        v = std::min(v, v_tight_ + v_per_clear_ * planner_.clearance(F.x, F.y));
        const double cur = std::hypot(cur_fx, cur_fy);
        if (cur > 0.05)
        {
          const double turn = std::acos(std::max(-1.0, std::min(1.0, (cur_fx * ux + cur_fy * uy) / cur)));
          v = std::min(v, std::max(0.15, v_max_ * std::cos(std::min(turn, M_PI / 2))));
        }
        des_fx = v * ux;
        des_fy = v * uy;
        last_vlimit_ = v;
        last_dfree_ = dfree;
      }
    }
    // acceleration limit on the velocity vector (braking may use a_brake)
    double dx = des_fx - cur_fx, dy = des_fy - cur_fy;
    const double dv = std::hypot(dx, dy);
    const double amax = (std::hypot(des_fx, des_fy) < std::hypot(cur_fx, cur_fy) ? a_brake_ : a_max_) * dt;
    if (dv > amax)
    {
      dx *= amax / dv;
      dy *= amax / dv;
    }
    double fx = cur_fx + dx, fy = cur_fy + dy;
    // base frame
    double bx = c * fx + s * fy, by = -s * fx + c * fy;
    // independent safety layer on the raw obstacle points
    const double v = std::hypot(bx, by);
    if (v > 1e-3)
    {
      const double vs = safetySpeedLimit(bx / v, by / v, v);
      if (vs < v)
      {
        bx *= vs / v;
        by *= vs / v;
        safety_active_ = true;
      }
      else
        safety_active_ = false;
    }
    out.linear.x = bx;
    out.linear.y = by;
    out.angular.z = std::max(-w_max_, std::min(w_max_, -k_heading_ * wrapAngle(F.th - start_.th)));
    return out;
  }

  geometry_msgs::Twist brake(double dt) const
  {
    geometry_msgs::Twist out = cmd_;
    const double v = std::hypot(cmd_.linear.x, cmd_.linear.y);
    const double nv = std::max(0.0, v - a_brake_ * dt);
    if (v > 1e-6)
    {
      out.linear.x *= nv / v;
      out.linear.y *= nv / v;
    }
    out.angular.z = 0;
    return out;
  }

  void tick(const ros::TimerEvent&)
  {
    const ros::Time now = ros::Time::now();
    const double dt = 1.0 / rate_;

    geometry_msgs::Twist cmd;
    if (active())
    {
      const std::string stale = staleRequired(now);
      if (!stale.empty() && state_ != FINISHED)
        setState(SAFE_STOP, "stale inputs: " + stale);
    }
    if (active())
    {
      if ((now - last_plan_).toSec() >= 1.0 / plan_rate_)
      {
        replan(now);
        last_plan_ = now;
      }
      const Pose2 F = fieldPose();
      if (state_ == MAPPING && (now - start_time_).toSec() >= mapping_duration_)
        setState(RUNNING, "start map done");
      if (state_ == RUNNING)
      {
        if (F.x + planner_.frontExtent() >= keep_.goal_x)
        {
          finish_time_ = (now - start_time_).toSec();
          std::ostringstream os;
          os << "goal line reached in " << finish_time_ << " s";
          setState(FINISHED, os.str());
        }
        else if (!path_ok_ && (now - last_path_ok_).toSec() > no_path_timeout_)
          setState(SAFE_STOP, "no path to the goal for " + std::to_string(no_path_timeout_) + " s");
        else if (!path_ok_ && !recovered_ && (now - last_path_ok_).toSec() > recovery_after_)
        {
          // recovery (as move_base clears its costmap): forget everything beyond 1 m and re-observe
          // it; the cells next to the robot stay, they may be inside the camera's inner ring
          grid_.clearOutside(local_.x, local_.y, 1.0);
          recovered_ = true;
          ROS_WARN("nav_node: no path for %.1f s, cleared the obstacle grid beyond 1 m", recovery_after_);
        }
        else
        {
          cmd = control(F, dt);
          std::ostringstream os;
          os << "running" << (path_ok_ ? "" : " (no path: braking)") << (safety_active_ ? " (safety limit)" : "");
          reason_ = os.str();
        }
      }
      if (state_ == FINISHED)
        cmd = brake(dt);
    }
    cmd_ = cmd;
    cmd_pub_.publish(cmd);
    publishPose(now);
    if (++tick_count_ % std::max(1, static_cast<int>(rate_ / 10.0)) == 0)
    {
      publishStatus(now);
      publishDebug();
    }
  }

  // ---------------------------------------------------------------- outputs
  void publishPose(const ros::Time& now)
  {
    geometry_msgs::PoseWithCovarianceStamped m;
    m.header.stamp = now;
    m.header.frame_id = "field";
    m.pose.pose.position.x = loc_.x();
    m.pose.pose.position.y = loc_.y();
    m.pose.pose.orientation.z = std::sin(loc_.theta() / 2);
    m.pose.pose.orientation.w = std::cos(loc_.theta() / 2);
    const auto& P = loc_.cov();
    m.pose.covariance[0] = P(0, 0);
    m.pose.covariance[1] = P(0, 1);
    m.pose.covariance[6] = P(1, 0);
    m.pose.covariance[7] = P(1, 1);
    m.pose.covariance[35] = P(2, 2);
    pose_pub_.publish(m);
    // field -> odom, so the odometry tree hangs off the field in RViz / the web UI
    const Pose2 T = compose(fieldPose(), inverse(odom_pose_));
    geometry_msgs::TransformStamped tf;
    tf.header = m.header;
    tf.child_frame_id = "odom";
    tf.transform.translation.x = T.x;
    tf.transform.translation.y = T.y;
    tf.transform.rotation.z = std::sin(T.th / 2);
    tf.transform.rotation.w = std::cos(T.th / 2);
    tfb_.sendTransform(tf);
  }

  // obstacle points and C-space in the field frame, for debugging and the web UI (2 Hz)
  void publishMaps(const ros::Time& now, const std::vector<Pt>& obs)
  {
    if ((now - last_maps_).toSec() < 0.5)
      return;
    last_maps_ = now;
    sensor_msgs::PointCloud pc;
    pc.header.stamp = now;
    pc.header.frame_id = "field";
    for (const Pt& q : obs)
    {
      geometry_msgs::Point32 p;
      p.x = q.x;
      p.y = q.y;
      pc.points.push_back(p);
    }
    obstacles_pub_.publish(pc);
    const cv::Mat& c = planner_.cspace();
    nav_msgs::OccupancyGrid g;
    g.header = pc.header;
    g.info.resolution = planner_.params().res;
    g.info.width = c.cols;
    g.info.height = c.rows;
    g.info.origin.position.x = planner_.params().x_min;
    g.info.origin.position.y = planner_.params().y_min;
    g.info.origin.orientation.w = 1;
    g.data.resize(c.rows * c.cols);
    for (int r = 0; r < c.rows; ++r)
      for (int col = 0; col < c.cols; ++col)
        g.data[r * c.cols + col] = c.at<uint8_t>(r, col) ? 100 : 0;
    cspace_pub_.publish(g);
  }

  void publishStatus(const ros::Time& now)
  {
    avoid_msgs::Status st;
    st.header.stamp = now;
    st.header.frame_id = "field";
    st.state = state_;
    st.state_name = name(state_);
    st.reason = reason_;
    st.sensor_config = sensor_;
    st.age_obstacles = obstacles_.age(now);
    st.age_boundary = boundary_.age(now);
    st.age_odom = odom_.age(now);
    st.age_imu = imu_.age(now);
    st.pose.x = loc_.x();
    st.pose.y = loc_.y();
    st.pose.theta = loc_.theta();
    st.pose_std_x = std::sqrt(loc_.cov()(0, 0));
    st.pose_std_y = std::sqrt(loc_.cov()(1, 1));
    st.pose_std_theta = std::sqrt(loc_.cov()(2, 2));
    st.cmd = cmd_;
    st.elapsed = (state_ == MAPPING || state_ == RUNNING) ? (now - start_time_).toSec()
                                                          : (state_ == FINISHED ? finish_time_ : 0.0);
    status_pub_.publish(st);
  }

  void publishDebug()
  {
    std::ostringstream os;
    os << "{\"occupied_cells\": " << occupied_cells_ << ", \"path_ok\": " << (path_ok_ ? "true" : "false")
       << ", \"waypoints\": " << path_.size() << ", \"v_limit\": " << last_vlimit_ << ", \"d_free\": " << last_dfree_
       << ", \"safety\": " << (safety_active_ ? "true" : "false") << ", \"lines_accepted\": " << lines_accepted_
       << ", \"lines_rejected\": " << lines_rejected_ << ", \"walls_accepted\": " << walls_accepted_
       << ", \"walls_rejected\": " << walls_rejected_ << ", \"scale_x\": " << loc_.scaleX()
       << ", \"scale_y\": " << loc_.scaleY() << "}";
    std_msgs::String m;
    m.data = os.str();
    debug_pub_.publish(m);
  }

  // parameters
  std::string sensor_, reason_;
  double rate_ = 20, plan_rate_ = 10, mapping_duration_ = 1;
  double v_max_, a_max_, a_brake_, stop_margin_, v_tight_, v_per_clear_, lookahead_, k_heading_, w_max_;
  double safety_margin_, max_obs_range_, line_margin_base_, line_margin_sigma_, no_path_timeout_;
  double clear_margin_ = 0.10, recovery_after_ = 1.0;
  size_t occupied_cells_ = 0;
  bool recovered_ = false;
  // state
  uint8_t state_ = IDLE;
  ros::Time start_time_, last_plan_, last_path_ok_, last_odom_;
  double finish_time_ = 0;
  unsigned long tick_count_ = 0;
  Input obstacles_, odom_, imu_, boundary_;
  bool have_imu_ = false;
  double imu_yaw_ = 0, imu0_ = 0, imu_at_last_odom_ = 0;
  Pose2 start_, local_, odom_pose_;
  Localizer loc_;
  GridPlanner planner_;
  Keepout keep_;
  Polygon footprint_;
  OccGrid grid_;                                    // obstacle layer, local frame
  std::deque<std::pair<double, Pose2>> history_;   // (time, local pose), last 2 s
  std::vector<Pt> path_;
  bool path_ok_ = false, safety_active_ = false;
  double last_vlimit_ = 0, last_dfree_ = 0;
  int lines_accepted_ = 0, lines_rejected_ = 0, walls_accepted_ = 0, walls_rejected_ = 0;
  std::vector<FieldLine> red_lines_, walls_;
  double red_max_range_ = 1.5, wall_max_range_ = 2.5;
  geometry_msgs::Twist cmd_;
  // ROS
  ros::Publisher cmd_pub_, status_pub_, pose_pub_, path_pub_, debug_pub_, obstacles_pub_, cspace_pub_;
  ros::Time last_maps_;
  std::vector<ros::Subscriber> subs_;
  ros::ServiceServer cmd_srv_;
  ros::Timer timer_;
  tf2_ros::TransformBroadcaster tfb_;
};
}  // namespace avoid_nav

int main(int argc, char** argv)
{
  ros::init(argc, argv, "nav_node");
  ros::NodeHandle nh, pnh("~");
  avoid_nav::NavNode node(nh, pnh);
  ros::spin();
  return 0;
}
