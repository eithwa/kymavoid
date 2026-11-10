// Grid planner in the field frame.
//
// Occupancy = obstacle points (inflated by a small perception margin) + keep-out zones (red lines and
// beyond, the wall behind the goal). The C-space is the occupancy dilated by the footprint mask,
// rotated to the current heading: exact for a robot that does not rotate. A* runs towards "the
// footprint touches the goal line", with a cost that prefers clearance and travel time.
//
// Two boundaries: the hard C-space (obstacles inflated by hard_inflation = the safety layer's margin)
// is never entered; between it and the soft C-space (obstacle_inflation) lies a band the A* may cross
// at band_cost. Clearance, speed limits and line of sight all use the soft one. With a single 2 cm
// boundary, 1-2 cm of map noise closed a 49 cm gap's 5 cm wide C-space corridor now and then, the
// planner switched to a detour and the robot turned back and forth.
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
  double obstacle_inflation = 0.02;   // soft: obstacle cells are 2 cm, covers centre-to-surface + noise
  double hard_inflation = 0.01;       // hard: never closer (nav_node sets the safety layer's margin)
  double band_cost = 3.0;             // step cost x this between the hard and the soft boundary
  double clearance_pref = 0.15;       // closer than this to the C-space boundary costs extra
  double clearance_weight = 2.0;
  // hysteresis: step cost x keep_factor within keep_radius of the previous path. Wide enough to cover
  // the whole passage the path is in (gaps are >= 50 cm, boxes 45 cm): it makes the planner stay with
  // a route, not with a line inside it (10 cm kept the robot on a line planned before an obstacle's
  // face was seen, 2 cm from it).
  double keep_factor = 0.7;
  double keep_radius = 0.30;
  // Travel time: a step costs at least the time it takes at nav_node's clearance speed limit
  // v_tight + v_per_clearance x clearance, relative to the speed the robot can reach there at all
  // (from its current speed at a_max; braking at a_brake for the wall behind the goal line), capped
  // at v_max. Without it the planner sometimes switched between the classic layout's L gap and the
  // detour on its left (5 of 22 runs turned back and forth, 0 of 18 with it).
  double v_max = 0.8, v_tight = 0.15, v_per_clearance = 8.0, a_max = 1.0, a_brake = 1.2;
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
  // `previous`: the path being followed; staying near it is cheaper (a new route has to be clearly
  // better before the robot turns away from the one it is on).
  // v0: the robot's current speed.
  bool plan(double x, double y, std::vector<Pt>& path, const std::vector<Pt>* previous = nullptr, double v0 = 0.0) const;

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
  // straight segment between path cells i and j through free (soft) C-space, keeping at every point
  // the clearance the A* path had around the corresponding cells
  bool lineOfSight(const std::vector<int>& cells, const std::vector<float>& need, size_t i, size_t j) const;
  float stepFactor(int r, int c, double v_reach) const;   // cost factor of a cell (>= 1)
  double reachSpeed(int r, int c, int sr, int sc, double v0) const;

  PlannerParams p_;
  Polygon fp_;
  double front_ = 0.2;
  int nx_, ny_;
  Keepout keep_;
  cv::Mat occ_, hard_, cspace_, dist_;   // hard_ / cspace_ (soft): 1 = blocked; dist_: CV_32F metres
                                         // to the soft boundary (0 inside it)
};
}  // namespace avoid_nav
