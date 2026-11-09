// Grid planner in the field frame.
//
// Occupancy = obstacle points (inflated by a small perception margin) + keep-out zones (red lines and
// beyond, the wall behind the goal). The C-space is the occupancy dilated by the footprint mask,
// rotated to the current heading: exact for a robot that does not rotate. A* runs on the free
// C-space towards "the footprint touches the goal line", with a cost that prefers clearance.
#pragma once
#include <opencv2/core.hpp>

#include <vector>

#include "avoid_nav/polygon.h"

namespace avoid_nav
{
struct PlannerParams
{
  double res = 0.01;                  // 1 cm: a 50 cm gap leaves only 5 cm per side for the 40 cm robot
  double x_min = -3.4, x_max = 3.7, y_min = -1.7, y_max = 1.7;   // field frame
  double obstacle_inflation = 0.02;   // obstacle cells are 2 cm: covers centre-to-surface + noise
  double clearance_pref = 0.15;       // closer than this to the C-space boundary costs extra
  double clearance_weight = 2.0;
  double start_snap = 0.10;           // if the robot cell is blocked, accept a free cell this close
};

struct Keepout
{
  double red_inner = 1.475;     // |y| of the red lines' inner edge
  double line_margin = 0.0;     // extra keep-out inside the red lines (localisation uncertainty)
  double goal_x = 3.0;          // goal line
  double back_wall_x = -3.475;  // inner faces of the low wall in front of/behind the course
  double front_wall_x = 3.475;
};

class GridPlanner
{
public:
  explicit GridPlanner(const PlannerParams& p = PlannerParams());
  void setFootprint(const Polygon& fp);

  // Rebuild the maps. Obstacle points in the field frame; theta = current heading.
  void build(const std::vector<Pt>& obstacles, double theta, const Keepout& k);

  // A* from (x, y) to any pose whose footprint touches the goal line. Smoothed waypoints (field frame).
  bool plan(double x, double y, std::vector<Pt>& path) const;

  // Distance the robot origin can move from (x, y) along unit (ux, uy) before the footprint touches
  // the C-space (capped at max_d).
  double freeDistance(double x, double y, double ux, double uy, double max_d) const;

  // Distance from (x, y) to the nearest C-space obstacle (0 if inside one).
  double clearance(double x, double y) const;
  bool free(double x, double y) const;
  double frontExtent() const { return front_; }
  const cv::Mat& cspace() const { return cspace_; }   // CV_8U, 1 = blocked; row = y, col = x
  const PlannerParams& params() const { return p_; }

private:
  bool cell(double x, double y, int& r, int& c) const;
  bool lineOfSight(int r0, int c0, int r1, int c1) const;

  PlannerParams p_;
  Polygon fp_;
  double front_ = 0.2;
  int nx_, ny_;
  Keepout keep_;
  cv::Mat occ_, cspace_, dist_;   // dist_: CV_32F metres to the nearest blocked cell
};
}  // namespace avoid_nav
