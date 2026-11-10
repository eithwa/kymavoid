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

#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdlib>
#include <memory>
#include <mutex>
#include <thread>
#include <fstream>
#include <sstream>
#include <string>
#include <deque>
#include <vector>

#include "avoid_nav/camera_model.h"
#include "avoid_nav/localizer.h"
#include "avoid_nav/occupancy.h"
#include "avoid_nav/planner.h"
#include "avoid_nav/polygon.h"
#include "avoid_nav/speed_limits.h"

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
  NavNode(ros::NodeHandle& nh, ros::NodeHandle& pnh) : pnh_(pnh)
  {
    pnh.param("sensor_config", sensor_, std::string("camera"));
    pnh.param("rate", rate_, 20.0);
    pnh.param("plan_rate", plan_rate_, 10.0);
    pnh.param("mapping_duration", mapping_duration_, 1.0);
    pnh.param("max_imu_step", max_imu_step_, 0.3);   // [rad] per odometry message; more is a jump
    double t_obs, t_odom, t_imu, t_bound;
    pnh.param("timeout_obstacles", t_obs, 0.3);
    pnh.param("timeout_odom", t_odom, 0.2);
    pnh.param("timeout_imu", t_imu, 0.3);
    pnh.param("timeout_boundary", t_bound, 0.3);
    obstacles_ = {"obstacles", t_obs, true, ros::Time()};
    odom_ = {"odom", t_odom, true, ros::Time()};
    imu_ = {"imu", t_imu, true, ros::Time()};
    boundary_ = {"boundary", t_bound, false, ros::Time()};
    // range uncertainty of the obstacle readings, for clearing the grid: the camera's falls with range
    if (sensor_ == "camera")
    {
      double cam_high, sigma_px;
      int inner, outer;
      loadCameraModel(nh, cam_high, inner, outer);
      pnh.param("camera_sigma_px", sigma_px, 1.0);   // pixel noise + quantisation of black_item
      for (int cm = 0; cm <= 800; ++cm)
        sigma_table_.push_back(static_cast<float>(omniRangeSigma(std::max(cm, 1) / 100.0, cam_high, outer, sigma_px)));
      ROS_INFO("nav_node: camera range sigma %.3f m at 0.5 m, %.3f m at 1 m, %.3f m at 2 m", sigma_table_[50],
               sigma_table_[100], sigma_table_[200]);
    }
    // control and perception tuning: re-read at every start, editable from the web UI
    pnh.param("tuning_file", tuning_file_, std::string());
    loadTuning();

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
    planner_template_.setFootprint(footprint_);
    planner_ = std::make_shared<GridPlanner>(planner_template_);

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
    plan_thread_ = std::thread(&NavNode::planLoop, this);
  }

  ~NavNode()
  {
    {
      std::lock_guard<std::mutex> lock(mtx_);
      stop_thread_ = true;
    }
    plan_cv_.notify_all();
    if (plan_thread_.joinable())
      plan_thread_.join();
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

  // ---------------------------------------------------------------- tuning
  struct Tune
  {
    const char* name;
    double NavNode::*field;
    double def;
  };
  static const std::vector<Tune>& tunables()
  {
    static const std::vector<Tune> t = {
        {"v_max", &NavNode::v_max_, 0.8},
        {"a_max", &NavNode::a_max_, 1.0},
        {"a_brake", &NavNode::a_brake_, 1.2},
        {"react_time", &NavNode::react_time_, 0.1},        // [s] from seeing something to braking for it
        {"stop_margin", &NavNode::stop_margin_, 0.03},
        {"v_tight", &NavNode::v_tight_, 0.15},             // speed with zero clearance
        {"v_per_clearance", &NavNode::v_per_clear_, 8.0},  // + this x clearance [m/s per m]
        {"lookahead", &NavNode::lookahead_, 0.35},
        {"route_keep", &NavNode::route_keep_, 0.7},       // A* cost x this near the current path (hysteresis)
        {"k_heading", &NavNode::k_heading_, 1.5},
        {"w_max", &NavNode::w_max_, 1.0},
        // below the planner's 2 cm obstacle inflation: the safety layer must not veto paths the planner
        // finds (3 cm stopped the robot for good in a 49 cm gap with 1-2 cm of map noise)
        {"safety_margin", &NavNode::safety_margin_, 0.01},
        {"max_obstacle_range", &NavNode::max_obs_range_, -1},   // default depends on the sensor
        {"red_line_max_range", &NavNode::red_max_range_, 1.5},  // real images: red lines unreliable beyond ~1 m
        {"wall_max_range", &NavNode::wall_max_range_, 2.5},
        {"line_margin_base", &NavNode::line_margin_base_, 0.02},
        {"line_margin_sigma", &NavNode::line_margin_sigma_, 2.0},
        {"wall_margin", &NavNode::wall_margin_, 0.08},    // [m] + line_margin_sigma x sigma_x before the low wall
        {"clear_margin", &NavNode::clear_margin_, 0.02},    // beams clear up to max(this, clear_sigmas x sigma) short of a hit
        {"clear_sigmas", &NavNode::clear_sigmas_, 2.0},
        {"recovery_after", &NavNode::recovery_after_, 1.0},
        {"no_path_timeout", &NavNode::no_path_timeout_, 3.0},
    };
    return t;
  }

  void loadTuning()
  {
    for (const Tune& t : tunables())
      pnh_.param(t.name, this->*t.field, t.def >= 0 ? t.def : (sensor_ == "camera" ? 2.5 : 6.0));
    // the planner uses the same speed model and never plans closer to an obstacle than the safety
    // layer allows
    PlannerParams pp;
    pp.hard_inflation = safety_margin_;
    pp.keep_factor = route_keep_;
    pp.v_max = v_max_;
    pp.v_tight = v_tight_;
    pp.v_per_clearance = v_per_clear_;
    pp.a_max = a_max_;
    pp.a_brake = a_brake_;
    planner_template_ = GridPlanner(pp);
    planner_template_.setFootprint(footprint_);
    OccParams op;
    op.clear_margin = clear_margin_;
    op.clear_sigmas = clear_sigmas_;
    grid_ = OccGrid(op);
    grid_.setSigmaTable(sigma_table_);
  }

  // Write the current parameter-server values (what the next start will use) back to the tuning
  // file: values are replaced in place, comments and order are kept, missing keys are appended.
  bool saveTuning(std::string& msg)
  {
    if (tuning_file_.empty())
    {
      msg = "no ~tuning_file configured";
      return false;
    }
    std::vector<std::string> lines;
    {
      std::ifstream in(tuning_file_);
      for (std::string l; std::getline(in, l);)
        lines.push_back(l);
    }
    auto number = [&](const Tune& t) {
      double v = this->*t.field;
      pnh_.getParam(t.name, v);
      return v;
    };
    auto value = [&](const Tune& t) {
      std::ostringstream os;
      os << number(t);
      return os.str();
    };
    for (const Tune& t : tunables())
    {
      const std::string key = std::string(t.name) + ":";
      bool found = false;
      for (std::string& l : lines)
        if (l.compare(0, key.size(), key) == 0)
        {
          found = true;
          char* end = nullptr;
          const double old = std::strtod(l.c_str() + key.size(), &end);
          if (end != l.c_str() + key.size() && std::fabs(old - number(t)) < 1e-12)
            continue;                                   // unchanged: keep the line as written
          const size_t hash = l.find('#');
          std::string v = key + " " + value(t);
          if (hash != std::string::npos)
            v += std::string(std::max<size_t>(1, hash > v.size() ? hash - v.size() : 1), ' ') + l.substr(hash);
          l = v;
        }
      if (!found)
        lines.push_back(key + " " + value(t));
    }
    std::ofstream f(tuning_file_);
    for (const std::string& l : lines)
      f << l << "\n";
    if (!f)
    {
      msg = "cannot write " + tuning_file_;
      return false;
    }
    msg = "saved to " + tuning_file_;
    return true;
  }

  // ---------------------------------------------------------------- inputs
  void onImu(const std_msgs::Float64& m)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    imu_.last = ros::Time::now();
    imu_yaw_ = m.data;
    have_imu_ = true;
  }

  void onOdom(const nav_msgs::Odometry& m)
  {
    std::lock_guard<std::mutex> lock(mtx_);
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
        double dth = wrapAngle(imu_yaw_ - imu_at_last_odom_);
        if (std::fabs(dth) > max_imu_step_)
        {
          // an IMU jump (re-zeroed / glitch; the simulator re-draws the IMU offset at a field reset),
          // not a rotation: keep the heading continuous
          imu0_ = wrapAngle(imu0_ + dth);
          ROS_WARN("nav_node: IMU jumped %.1f deg between two odometry messages, ignored", dth * 180 / M_PI);
          dth = 0;
        }
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
    std::lock_guard<std::mutex> lock(mtx_);
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
    std::lock_guard<std::mutex> lock(mtx_);
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
    std::lock_guard<std::mutex> lock(mtx_);
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
      loadTuning();                                   // picks up parameters changed from the web UI
      anchorAtStart();
      max_plan_ms_ = 0;
      start_time_ = now;
      plan_now_ = true;                               // first plan right away
      plan_cv_.notify_one();
      last_path_ok_ = now;
      setState(sensor_ == "lidar" ? MAPPING : RUNNING, "started");
    }
    else if (req.command == "stop")
      setState(IDLE, "stopped by command");
    else if (req.command == "reset")
    {
      // the robot was put back on the start pose (or the simulator teleported it there)
      anchorAtStart();
      setState(IDLE, "reset: pose set to the start");
      publishCleared(now);
    }
    else if (req.command == "save")
    {
      res.success = saveTuning(res.message);
      return true;
    }
    else
    {
      res.success = false;
      res.message = "unknown command '" + req.command + "' (start | stop | reset | save)";
      return true;
    }
    res.success = true;
    res.message = std::string("state ") + name(state_);
    publishStatus(now);
    return true;
  }

  // The robot stands on the start pose: (re)anchor localisation and forget the obstacle memory.
  void anchorAtStart()
  {
    loc_.reset(start_.x, start_.y, start_.th);
    local_ = start_;
    imu0_ = imu_yaw_;
    grid_.clear();
    history_.clear();
    path_.clear();
    path_ok_ = false;
    occupied_cells_ = 0;                            // /avoid/debug must not keep showing the old map
    ++run_id_;                                      // plans computed before this are dropped
    cmd_ = geometry_msgs::Twist();
    lines_accepted_ = lines_rejected_ = walls_accepted_ = walls_rejected_ = 0;
  }

  // ---------------------------------------------------------------- cycle
  Pose2 fieldPose() const { return {loc_.x(), loc_.y(), loc_.theta()}; }

  // Planning runs in its own thread: building the C-space and A* take 20-100+ ms, which used to
  // block odometry, localisation and control in the single ROS thread (status gaps up to 0.46 s).
  // Snapshot under the lock, plan without it, swap the result in under the lock.
  void planLoop()
  {
    std::unique_lock<std::mutex> lk(mtx_);
    while (!stop_thread_ && ros::ok())
    {
      plan_cv_.wait_for(lk, std::chrono::milliseconds(static_cast<int>(1000.0 / plan_rate_)),
                        [this] { return stop_thread_ || plan_now_; });
      if (stop_thread_)
        break;
      plan_now_ = false;
      if (!active())
        continue;
      const unsigned run = run_id_;
      const ros::Time snap_time = ros::Time::now();             // the path and maps describe this moment
      const Pose2 F = fieldPose();
      const Pose2 T = compose(F, inverse(local_));             // local -> field
      const Pose2 base_from_local = inverse(local_);
      const std::vector<Pt> cells = grid_.occupied();
      Keepout k = keep_;
      k.line_margin = line_margin_base_ + line_margin_sigma_ * std::sqrt(std::max(0.0, loc_.cov()(1, 1)));
      k.goal_x += finishMargin();                                // the path reaches the finish condition
      auto pl = std::make_shared<GridPlanner>(planner_template_);   // copied under the lock: start replaces it
      const std::vector<Pt> previous = path_ok_ ? path_ : std::vector<Pt>();
      const double v0 = std::hypot(cmd_.linear.x, cmd_.linear.y);
      lk.unlock();

      std::vector<Pt> obs;
      obs.reserve(cells.size());
      for (const Pt& c : cells)
      {
        // an occupied cell inside the robot's own footprint is stale (odometry drift): an obstacle
        // cannot be inside the robot
        if (inside(footprint_, apply(base_from_local, c)))
          continue;
        obs.push_back(apply(T, c));
      }
      std::vector<Pt> path;
      const ros::WallTime w0 = ros::WallTime::now();
      pl->build(obs, F.th, k);
      const ros::WallTime w1 = ros::WallTime::now();
      const bool ok = pl->plan(F.x, F.y, path, &previous, v0);
      const ros::WallTime w2 = ros::WallTime::now();
      const ros::Time now = ros::Time::now();

      lk.lock();
      if (stop_thread_ || run != run_id_ || !active())
        continue;                                            // restarted or stopped meanwhile: drop it
      planner_ = pl;
      path_ = path;
      path_ok_ = ok;
      occupied_cells_ = cells.size();
      build_ms_ = (w1 - w0).toSec() * 1e3;
      plan_ms_ = (w2 - w1).toSec() * 1e3;
      max_plan_ms_ = std::max(max_plan_ms_, build_ms_ + plan_ms_);
      if (path_ok_)
      {
        last_path_ok_ = now;
        recovered_ = false;
      }
      // stamped with the snapshot time: the obstacles were mapped into the field frame with the pose
      // of that moment, 20-100+ ms before planning finished
      publishPath(snap_time);
      publishMaps(snap_time, obs, *pl);
    }
  }

  // After a reset: the obstacle memory, path and C-space are gone, say so to every viewer (in IDLE
  // nothing else would be published and the old map would stay on screen).
  void publishCleared(const ros::Time& now)
  {
    path_.clear();
    publishPath(now);
    sensor_msgs::PointCloud pc;
    pc.header.stamp = now;
    pc.header.frame_id = "field";
    obstacles_pub_.publish(pc);
    nav_msgs::OccupancyGrid g;
    g.header = pc.header;
    g.info.origin.orientation.w = 1;
    cspace_pub_.publish(g);
  }

  void publishPath(const ros::Time& now)
  {
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

  // closest point on the path (segment seg -> seg + 1)
  void closestOnPath(const Pose2& F, size_t& seg, Pt& p) const
  {
    seg = 0;
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
    p = {path_[seg].x + best_t * (path_[seg + 1].x - path_[seg].x), path_[seg].y + best_t * (path_[seg + 1].y - path_[seg].y)};
  }

  // the point `dist` further along the path than the robot's closest point
  Pt pointAlongPath(const Pose2& F, double dist) const
  {
    size_t seg;
    Pt p;
    closestOnPath(F, seg, p);
    double left = dist;
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

  // Pure-pursuit target: the farthest point along the path, within `full`, that the robot can
  // reach in a straight line through free C-space. A* paths squeeze through gaps cell by cell; aiming
  // straight at a fixed lookahead point would cut the corner of a narrow gap and stall. In a corridor
  // a few cm wide the A* path zigzags, so a near point may be hidden while a farther one is in view:
  // keep looking (stopping at the first hidden one left a 2 cm target whose line clipped the C-space,
  // and the robot stood still until the map noise changed).
  Pt visibleTarget(const Pose2& F, double full) const
  {
    Pt best = pointAlongPath(F, 0.02);
    for (double l = 0.04; l <= full + 1e-9; l += 0.02)
    {
      const Pt p = pointAlongPath(F, l);
      const double d = std::hypot(p.x - F.x, p.y - F.y);
      if (d < 1e-6)
        continue;
      if (planner_->freeDistance(F.x, F.y, (p.x - F.x) / d, (p.y - F.y) / d, d) >= d - 0.01)
        best = p;
    }
    return best;
  }

  // Largest speed along base-frame unit (ux, uy) from which the footprint, reacting after react_time
  // and braking at a_brake, stops before it comes within safety_margin of an occupied cell. A cell
  // already that close only forbids moving closer to it: sliding past it (a wall the planner's path
  // runs along) is fine, and ignoring it - as before - let the robot drive into a box it touched.
  // A cell inside the footprint is stale (odometry drift) and ignored.
  double safetySpeedLimit(double ux, double uy, double v) const
  {
    const double reach = v * react_time_ + v * v / (2 * a_brake_) + safety_margin_ + 0.05;
    const Pose2 base_from_local = inverse(local_);
    double s_hit = 1e9;
    for (const Pt& c : grid_.occupied(local_.x, local_.y, reach + 0.35))
    {
      const Pt p = apply(base_from_local, c);
      if (std::hypot(p.x, p.y) > reach + 0.35 || inside(footprint_, p))
        continue;
      const double d0 = distanceToEdges(footprint_, p);
      if (d0 <= safety_margin_)
      {
        if (distanceToEdges(footprint_, {p.x - ux * 0.01, p.y - uy * 0.01}) < d0 - 0.002)
          return 0.0;                                // moving into it
        continue;
      }
      for (double s = 0.01; s <= reach; s += 0.01)
        if (insideOrNear(footprint_, {p.x - ux * s, p.y - uy * s}, safety_margin_))
        {
          s_hit = std::min(s_hit, s);
          break;
        }
    }
    if (s_hit > reach)
      return v;
    return approachSpeed(s_hit - 0.01, a_brake_, react_time_);   // 0.01: the sweep's step
  }

  geometry_msgs::Twist control(const Pose2& F, double dt)
  {
    geometry_msgs::Twist out;
    const double c = std::cos(F.th), s = std::sin(F.th);
    const double cur_fx = c * cmd_.linear.x - s * cmd_.linear.y, cur_fy = s * cmd_.linear.x + c * cmd_.linear.y;
    const double cur = std::hypot(cur_fx, cur_fy);
    double des_fx = 0, des_fy = 0;
    speed_ = SpeedInfo();
    speed_.v_max = v_max_;
    speed_.limit = "no_path";                       // braking unless a target is found below
    if (path_ok_ && path_.size() >= 2)
    {
      // Pure pursuit looks further ahead at speed: turning towards a target L away takes about v^2 / L
      // of lateral acceleration, and the command limiter only allows a_max.
      const Pt t = visibleTarget(F, std::max(lookahead_, cur * cur / (2 * a_max_)));
      double ux = t.x - F.x, uy = t.y - F.y;
      const double n = std::hypot(ux, uy);
      if (n > 1e-6)
      {
        ux /= n;
        uy /= n;
        // All braking distances below include react_time; turns use a_max (the limiter's turning
        // acceleration) and may drift by the clearance there (2 cm at least: a path may run along the
        // C-space boundary, which is still 2 cm of obstacle inflation away from the cells).
        const double min_cut = 0.02;
        const double clear = planner_->clearance(F.x, F.y);
        const double dfree = planner_->freeDistance(F.x, F.y, ux, uy, 3.0);
        // never below v_tight: in a C-space corridor a few cm wide the line to the target can clip the
        // inflated boundary right away; creeping on is safe, the safety layer keeps 1 cm from the cells
        speed_.v_dfree = std::max(v_tight_, approachSpeed(dfree - stop_margin_, a_brake_, react_time_));
        speed_.v_clear = v_tight_ + v_per_clear_ * clear;
        if (cur > 0.05)
        {
          const double turn = std::acos(std::max(-1.0, std::min(1.0, (cur_fx * ux + cur_fy * uy) / cur)));
          speed_.v_turn = std::max(v_tight_, turnSpeed(turn, std::max(min_cut, clear), a_max_));
        }
        // corners further along the path: arrive at each slowly enough to turn through it
        size_t seg;
        Pt on;
        closestOnPath(F, seg, on);
        const double horizon = v_max_ * react_time_ + v_max_ * v_max_ / (2 * a_brake_) + 0.3;
        const auto& pl = *planner_;
        speed_.v_corner = std::max(
            v_tight_, pathCornerLimit(
                          path_, seg, on, [&](size_t k) { return pl.clearance(path_[k].x, path_[k].y); },
                          [&](size_t k, double ux, double uy) { return pl.freeDistance(path_[k].x, path_[k].y, ux, uy, 3.0); },
                          a_max_, a_brake_, react_time_, min_cut, horizon, 0.15, &speed_.corner_at));
        // the clearance limit also holds ahead: be down to it when entering a narrow passage
        speed_.v_clear = std::min(speed_.v_clear, pathCapLimit(path_, seg, on, [&](const Pt& q) { return v_tight_ + v_per_clear_ * pl.clearance(q.x, q.y); },
                                                               a_brake_, react_time_, horizon));
        // whatever the camera / lidars cannot see yet: stop within their range
        speed_.v_horizon = approachSpeed(max_obs_range_ - planner_->frontExtent() - stop_margin_, a_brake_, react_time_);
        double v = speed_.v_max;
        speed_.limit = "v_max";
        for (const auto& l : {std::make_pair(speed_.v_dfree, "dfree"), std::make_pair(speed_.v_clear, "clearance"),
                              std::make_pair(speed_.v_turn, "turn"), std::make_pair(speed_.v_corner, "corner"),
                              std::make_pair(speed_.v_horizon, "horizon")})
          if (l.first < v)
          {
            v = l.first;
            speed_.limit = l.second;
          }
        if (v > v_max_ - 0.01)
          speed_.limit = "v_max";                   // a limit within 1 cm/s of v_max is not limiting
        speed_.v_target = v;
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
    if (dv > amax && std::hypot(des_fx, des_fy) > std::hypot(cur_fx, cur_fy) + 1e-3)
      speed_.limit = "accel";                       // still accelerating towards the target speed
    double fx = cur_fx + dx, fy = cur_fy + dy;
    // The limiter turns the velocity only gradually, so at speed the robot keeps moving along its old
    // direction for a while: brake for the red lines and the low wall along the direction it really
    // moves in (the obstacles there are the safety layer's job).
    const double vf = std::hypot(fx, fy);
    if (vf > 1e-3)
    {
      const double lm = line_margin_base_ + line_margin_sigma_ * std::sqrt(std::max(0.0, loc_.cov()(1, 1)));
      double up = -1e9, down = -1e9, front = -1e9, back = -1e9;      // footprint extents at this heading
      for (const Pt& q : footprint_)
      {
        const double qx = c * q.x - s * q.y, qy = s * q.x + c * q.y;
        up = std::max(up, qy);
        down = std::max(down, -qy);
        front = std::max(front, qx);
        back = std::max(back, -qx);
      }
      // the low wall: the x estimate drifts more than its sigma says (8 cm behind the truth at the goal
      // line at 1 m/s, sigma_x 1 cm), and after the goal line the robot can only brake: keep wall_margin
      const double wm = wall_margin_ + line_margin_sigma_ * std::sqrt(std::max(0.0, loc_.cov()(0, 0)));
      const double dl = distanceToBounds(F.x, F.y, fx / vf, fy / vf, keep_.back_wall_x + back + wm, keep_.front_wall_x - front - wm,
                                         -(keep_.red_inner - lm) + down, keep_.red_inner - lm - up);
      speed_.v_line = approachSpeed(dl - stop_margin_, a_brake_, react_time_);
      if (speed_.v_line < vf)
      {
        fx *= speed_.v_line / vf;
        fy *= speed_.v_line / vf;
        speed_.limit = "line";
      }
    }
    // base frame
    double bx = c * fx + s * fy, by = -s * fx + c * fy;
    // independent safety layer on the raw obstacle points
    const double v = std::hypot(bx, by);
    if (v > 1e-3)
    {
      const double vs = safetySpeedLimit(bx / v, by / v, v);
      speed_.v_safety = vs;
      if (vs < v)
      {
        bx *= vs / v;
        by *= vs / v;
        safety_active_ = true;
        speed_.limit = "safety";
      }
      else
        safety_active_ = false;
    }
    out.linear.x = bx;
    out.linear.y = by;
    speed_.v_cmd = std::hypot(bx, by);
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

  // How far past the goal line the estimated front must be to call it finished: an x estimate a few
  // cm ahead of the truth made the robot stop short of the line (+3.6 cm once). The planner's goal
  // is moved by the same margin so the path reaches it; the wall margin leaves room behind the line.
  double finishMargin() const { return 0.03 + 2.0 * std::sqrt(std::max(0.0, loc_.cov()(0, 0))); }

  void tick(const ros::TimerEvent&)
  {
    std::lock_guard<std::mutex> lock(mtx_);
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
      const Pose2 F = fieldPose();
      if (state_ == MAPPING && (now - start_time_).toSec() >= mapping_duration_)
        setState(RUNNING, "start map done");
      if (state_ == RUNNING)
      {
        if (F.x + planner_->frontExtent() >= keep_.goal_x + finishMargin())
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
    if (state_ != RUNNING)
    {
      speed_ = SpeedInfo();
      speed_.v_cmd = std::hypot(cmd.linear.x, cmd.linear.y);
    }
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
  void publishMaps(const ros::Time& now, const std::vector<Pt>& obs, const GridPlanner& planner)
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
    const cv::Mat& c = planner.cspace();
    nav_msgs::OccupancyGrid g;
    g.header = pc.header;
    g.info.resolution = planner.params().res;
    g.info.width = c.cols;
    g.info.height = c.rows;
    g.info.origin.position.x = planner.params().x_min;
    g.info.origin.position.y = planner.params().y_min;
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
    // speed limits are +inf when nothing limits (no corner ahead, moving away from every line):
    // JSON has no inf, write the usual 99 ("not limiting")
    const auto lim = [](double v) { return std::isfinite(v) ? std::min(v, 99.0) : 99.0; };
    std::ostringstream os;
    os << "{\"occupied_cells\": " << occupied_cells_ << ", \"path_ok\": " << (path_ok_ ? "true" : "false")
       << ", \"waypoints\": " << path_.size() << ", \"v_limit\": " << lim(last_vlimit_) << ", \"d_free\": " << last_dfree_
       << ", \"safety\": " << (safety_active_ ? "true" : "false") << ", \"lines_accepted\": " << lines_accepted_
       << ", \"lines_rejected\": " << lines_rejected_ << ", \"walls_accepted\": " << walls_accepted_
       << ", \"walls_rejected\": " << walls_rejected_ << ", \"scale_x\": " << loc_.scaleX()
       << ", \"scale_y\": " << loc_.scaleY() << ", \"speed_limit\": \"" << (active() ? speed_.limit : "") << "\""
       << ", \"v_cmd\": " << speed_.v_cmd << ", \"v_target\": " << lim(speed_.v_target) << ", \"v_max\": " << speed_.v_max
       << ", \"v_dfree\": " << lim(speed_.v_dfree) << ", \"v_clearance\": " << lim(speed_.v_clear) << ", \"v_turn\": " << lim(speed_.v_turn)
       << ", \"v_corner\": " << lim(speed_.v_corner) << ", \"corner_at\": " << speed_.corner_at
       << ", \"v_horizon\": " << lim(speed_.v_horizon) << ", \"v_line\": " << lim(speed_.v_line)
       << ", \"v_safety\": " << lim(speed_.v_safety) << ", \"build_ms\": " << build_ms_ << ", \"plan_ms\": " << plan_ms_
       << ", \"max_plan_ms\": " << max_plan_ms_ << "}";
    std_msgs::String m;
    m.data = os.str();
    debug_pub_.publish(m);
  }

  // parameters
  ros::NodeHandle pnh_;
  std::string sensor_, reason_, tuning_file_;
  double rate_ = 20, plan_rate_ = 10, mapping_duration_ = 1, max_imu_step_ = 0.3;
  double v_max_, a_max_, a_brake_, react_time_ = 0.1, stop_margin_, v_tight_, v_per_clear_, lookahead_, k_heading_, w_max_;
  double safety_margin_, max_obs_range_, line_margin_base_, line_margin_sigma_, no_path_timeout_;
  double clear_margin_ = 0.02, clear_sigmas_ = 2.0, recovery_after_ = 1.0, route_keep_ = 0.7, wall_margin_ = 0.08;
  std::vector<float> sigma_table_;                 // range uncertainty per cm of range (camera), empty = lidar
  size_t occupied_cells_ = 0;
  bool recovered_ = false;
  // state
  uint8_t state_ = IDLE;
  ros::Time start_time_, last_path_ok_, last_odom_;
  double finish_time_ = 0;
  unsigned long tick_count_ = 0;
  Input obstacles_, odom_, imu_, boundary_;
  bool have_imu_ = false;
  double imu_yaw_ = 0, imu0_ = 0, imu_at_last_odom_ = 0;
  Pose2 start_, local_, odom_pose_;
  Localizer loc_;
  GridPlanner planner_template_;                   // footprint set, nothing built
  std::shared_ptr<const GridPlanner> planner_;     // latest built planner (swapped by planLoop)
  std::mutex mtx_;                                 // all state below/above: callbacks vs planLoop
  std::condition_variable plan_cv_;
  std::thread plan_thread_;
  bool stop_thread_ = false, plan_now_ = false;
  unsigned run_id_ = 0;
  Keepout keep_;
  Polygon footprint_;
  OccGrid grid_;                                    // obstacle layer, local frame
  std::deque<std::pair<double, Pose2>> history_;   // (time, local pose), last 2 s
  std::vector<Pt> path_;
  bool path_ok_ = false, safety_active_ = false;
  double last_vlimit_ = 0, last_dfree_ = 0;
  double build_ms_ = 0, plan_ms_ = 0, max_plan_ms_ = 0;     // wall time of the last replan / worst this run
  // speed limits of the last control cycle (m/s; 99 = not limiting) and which one decided
  struct SpeedInfo
  {
    double v_max = 0, v_dfree = 99, v_clear = 99, v_turn = 99, v_corner = 99, v_horizon = 99, v_line = 99, v_safety = 99;
    double v_target = 0, v_cmd = 0, corner_at = -1;   // corner_at: path distance to the limiting corner [m]
    std::string limit;
  } speed_;
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
