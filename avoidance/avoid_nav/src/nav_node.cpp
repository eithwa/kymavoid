// nav_node: state machine, /avoid/command, /avoid/status and the velocity output.
//
//   IDLE --start--> MAPPING (lidar: build the anchored map at the start) --> RUNNING
//   IDLE --start--> RUNNING (camera)
//   RUNNING --footprint touches the goal line--> FINISHED
//   any active state --stale required input--> SAFE_STOP (robot stopped, reason reported)
//   stop / reset --> IDLE
//
// Required inputs: obstacles (/scan), /odom, /imu/yaw. /boundary is reported but optional (the red
// line is often out of view). Localisation and planning are not implemented yet: in RUNNING the
// node commands zero velocity and says so in the status.
#include <avoid_msgs/Command.h>
#include <avoid_msgs/Status.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float64.h>

#include <sstream>
#include <string>
#include <vector>

namespace avoid_nav
{
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

    cmd_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 5);
    status_pub_ = nh.advertise<avoid_msgs::Status>("avoid/status", 5, true);
    subs_.push_back(nh.subscribe<sensor_msgs::LaserScan>("scan", 5, [this](const sensor_msgs::LaserScan::ConstPtr&) {
      obstacles_.last = ros::Time::now();
    }));
    subs_.push_back(nh.subscribe<sensor_msgs::LaserScan>("boundary", 5, [this](const sensor_msgs::LaserScan::ConstPtr&) {
      boundary_.last = ros::Time::now();
    }));
    subs_.push_back(nh.subscribe<nav_msgs::Odometry>("odom", 10, [this](const nav_msgs::Odometry::ConstPtr&) {
      odom_.last = ros::Time::now();
    }));
    subs_.push_back(nh.subscribe<std_msgs::Float64>("imu/yaw", 10, [this](const std_msgs::Float64::ConstPtr&) {
      imu_.last = ros::Time::now();
    }));
    cmd_srv_ = nh.advertiseService("avoid/command", &NavNode::onCommand, this);
    timer_ = nh.createTimer(ros::Duration(1.0 / rate_), &NavNode::tick, this);
    reason_ = "ready";
    ROS_INFO("nav_node: sensor config %s", sensor_.c_str());
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

  std::vector<const Input*> inputs() const { return {&obstacles_, &odom_, &imu_, &boundary_}; }

  std::string staleRequired(const ros::Time& now) const
  {
    std::ostringstream os;
    for (const Input* in : inputs())
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
      start_time_ = now;
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

  void tick(const ros::TimerEvent&)
  {
    const ros::Time now = ros::Time::now();
    geometry_msgs::Twist cmd;                               // zero unless a controller says otherwise
    if (state_ == MAPPING || state_ == RUNNING)
    {
      const std::string stale = staleRequired(now);
      if (!stale.empty())
        setState(SAFE_STOP, "stale inputs: " + stale);
      else if (state_ == MAPPING && (now - start_time_).toSec() >= mapping_duration_)
        setState(RUNNING, "anchored map done");
      else if (state_ == RUNNING)
        reason_ = "running (planner not implemented yet: commanding zero velocity)";
    }
    last_cmd_ = cmd;
    cmd_pub_.publish(cmd);
    if (++tick_count_ % std::max(1, static_cast<int>(rate_ / 10.0)) == 0)
      publishStatus(now);
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
    st.cmd = last_cmd_;
    st.elapsed = (state_ == MAPPING || state_ == RUNNING) ? (now - start_time_).toSec() : 0.0;
    status_pub_.publish(st);
  }

  std::string sensor_, reason_;
  double rate_ = 20.0, mapping_duration_ = 1.0;
  uint8_t state_ = IDLE;
  ros::Time start_time_;
  unsigned long tick_count_ = 0;
  Input obstacles_, odom_, imu_, boundary_;
  geometry_msgs::Twist last_cmd_;
  ros::Publisher cmd_pub_, status_pub_;
  std::vector<ros::Subscriber> subs_;
  ros::ServiceServer cmd_srv_;
  ros::Timer timer_;
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
