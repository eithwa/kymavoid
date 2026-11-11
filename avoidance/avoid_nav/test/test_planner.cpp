// GridPlanner (src/planner.cpp): the hard/soft C-space band, path hysteresis, smoothing that keeps
// the A* clearance, and the far obstacles' cost.
#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "avoid_nav/planner.h"

using namespace avoid_nav;

namespace
{
// 40 x 40 cm square robot, like robot6th (40.2 cm)
Polygon square() { return {{0.2, 0.2}, {-0.2, 0.2}, {-0.2, -0.2}, {0.2, -0.2}}; }

// obstacle cells (1 cm apart) filling [x0, x1] x [y0, y1]
void box(std::vector<Pt>& o, double x0, double x1, double y0, double y1)
{
  for (double x = x0; x <= x1 + 1e-9; x += 0.01)
    for (double y = y0; y <= y1 + 1e-9; y += 0.01)
      o.push_back({x, y});
}

GridPlanner planner(double hard = 0.01)
{
  PlannerParams p;
  p.hard_inflation = hard;
  GridPlanner g(p);
  g.setFootprint(square());
  return g;
}

// y of the path where it crosses x (linear interpolation; NaN if it never does)
double yAt(const std::vector<Pt>& path, double x)
{
  for (size_t i = 0; i + 1 < path.size(); ++i)
    if ((path[i].x - x) * (path[i + 1].x - x) <= 0 && path[i].x != path[i + 1].x)
      return path[i].y + (path[i + 1].y - path[i].y) * (x - path[i].x) / (path[i + 1].x - path[i].x);
  return std::nan("");
}

// smallest clearance along the path between x0 and x1 (sampled every cm)
double minClearance(const GridPlanner& g, const std::vector<Pt>& path, double x0, double x1)
{
  double best = 1e9;
  for (size_t i = 0; i + 1 < path.size(); ++i)
  {
    const double len = std::hypot(path[i + 1].x - path[i].x, path[i + 1].y - path[i].y);
    for (double u = 0; u <= len; u += 0.01)
    {
      const double x = path[i].x + (path[i + 1].x - path[i].x) * u / len, y = path[i].y + (path[i + 1].y - path[i].y) * u / len;
      if (x >= x0 && x <= x1)
        best = std::min(best, g.clearance(x, y));
    }
  }
  return best;
}
}  // namespace

// A wall across the field with one 49 cm gap (the classic layout's), and map noise sticking 2.5 cm
// into the gap from both sides: the soft (2 cm) corridor is closed, the hard (1 cm) one is not.
TEST(GridPlanner, NoiseInANarrowGapDoesNotCloseIt)
{
  std::vector<Pt> obs;
  box(obs, 0.0, 0.45, 0.245, 1.6);
  box(obs, 0.0, 0.45, -1.6, -0.245);
  box(obs, 0.20, 0.22, 0.22, 0.245);     // noise
  box(obs, 0.20, 0.22, -0.245, -0.22);
  std::vector<Pt> path;

  GridPlanner g = planner();
  g.build(obs, 0.0, Keepout());
  ASSERT_TRUE(g.plan(-2.0, 0.0, path));
  EXPECT_NEAR(yAt(path, 0.21), 0.0, 0.015) << "through the gap";

  GridPlanner single = planner(0.02);                    // the old single 2 cm boundary
  single.build(obs, 0.0, Keepout());
  EXPECT_FALSE(single.plan(-2.0, 0.0, path)) << "the old planner had no path (or a detour if there was one)";
}

// Two equally good 80 cm gaps: the planner stays with the one the previous path used.
TEST(GridPlanner, KeepsThePreviousRoute)
{
  std::vector<Pt> obs;
  box(obs, 0.0, 0.45, 1.0, 1.6);
  box(obs, 0.0, 0.45, -0.2, 0.2);
  box(obs, 0.0, 0.45, -1.6, -1.0);
  GridPlanner g = planner();
  g.build(obs, 0.0, Keepout());
  std::vector<Pt> path;
  ASSERT_TRUE(g.plan(-2.0, 0.0, path));
  const bool upper = yAt(path, 0.2) > 0;
  // the other gap as the previous path
  const double yo = upper ? -0.6 : 0.6;
  const std::vector<Pt> previous{{-2.0, 0.0}, {-0.5, yo}, {1.0, yo}, {3.0, yo}};
  ASSERT_TRUE(g.plan(-2.0, 0.0, path, &previous));
  EXPECT_EQ(yAt(path, 0.2) > 0, !upper) << "stays with the previous route";
}

// Out of a 49 cm gap, along a long obstacle (its lower face continues the gap's upper edge): the
// smoothed path must not run along it at the gap's clearance.
TEST(GridPlanner, SmoothingKeepsTheClearanceAfterAGap)
{
  std::vector<Pt> obs;
  box(obs, 0.0, 0.45, 0.245, 1.6);
  box(obs, 0.0, 0.45, -1.6, -0.245);
  box(obs, 0.45, 2.0, 0.245, 0.7);       // long obstacle after the gap, upper side
  GridPlanner g = planner();
  g.build(obs, 0.0, Keepout());
  std::vector<Pt> path;
  ASSERT_TRUE(g.plan(-2.0, 0.0, path));
  EXPECT_LT(minClearance(g, path, 0.1, 0.35), 0.05) << "tight in the gap";
  EXPECT_GT(minClearance(g, path, 1.0, 1.8), 0.10) << "not along the long obstacle at the gap's clearance";
}

namespace
{
// far-layer points (10 cm cells) filling [x0, x1] x [y0, y1]
std::vector<Pt> farBox(double x0, double x1, double y0, double y1)
{
  std::vector<Pt> f;
  for (double x = x0 + 0.05; x < x1; x += 0.1)
    for (double y = y0 + 0.05; y < y1; y += 0.1)
      f.push_back({x, y});
  return f;
}

// does the (smoothed) path enter the far obstacles' C-space between x0 and x1?
bool entersFar(const GridPlanner& g, const std::vector<Pt>& path, double x0, double x1)
{
  for (size_t i = 0; i + 1 < path.size(); ++i)
  {
    const double len = std::hypot(path[i + 1].x - path[i].x, path[i + 1].y - path[i].y);
    for (double u = 0; u <= len; u += 0.01)
    {
      const double x = path[i].x + (path[i + 1].x - path[i].x) * u / len, y = path[i].y + (path[i + 1].y - path[i].y) * u / len;
      if (x >= x0 && x <= x1 && g.farCost(x, y))
        return true;
    }
  }
  return false;
}
}  // namespace

// Seed 42519 (route layout) in short: 3 m ahead the only way on is to the left of a row of boxes seen
// only by far readings. Without them the path runs straight at the row and turns at the last moment;
// with them it heads for the opening at once, and the smoothing does not cut back into them.
TEST(GridPlanner, FarObstaclesTurnTheRouteEarly)
{
  const std::vector<Pt> none, far = farBox(1.0, 1.5, -1.5, 0.2);
  GridPlanner g = planner();
  std::vector<Pt> path;
  g.build(none, 0.0, Keepout());
  ASSERT_TRUE(g.plan(-2.0, 0.0, path));
  EXPECT_NEAR(yAt(path, 1.2), 0.0, 0.02) << "unknown: straight on";

  g.build(none, 0.0, Keepout(), far);
  ASSERT_TRUE(g.plan(-2.0, 0.0, path));
  EXPECT_GT(yAt(path, 1.2), 0.15 + 0.2 - PlannerParams().far_shrink - 0.01)
      << "past the row's far C-space (cells up to y = 0.15, half width 0.2 less far_shrink) on its open side";
  EXPECT_GT(yAt(path, -0.5), 0.12) << "already turning 1.5 m before it";
  EXPECT_FALSE(entersFar(g, path, -2.0, 3.0));
}

// The far C-space is where the footprint would cover a far point: for a robot reaching 0.3 m ahead and
// 0.1 m behind, a point at x = 1.0 costs from x = 0.7 to 1.1 (mirrored, it would be 0.9 to 1.3).
TEST(GridPlanner, FarCostIsTheFootprintsCSpace)
{
  PlannerParams p;
  p.far_shrink = 0.0;                                   // the footprint itself
  GridPlanner g(p);
  g.setFootprint({{0.3, 0.1}, {-0.1, 0.1}, {-0.1, -0.1}, {0.3, -0.1}});
  g.build(std::vector<Pt>(), 0.0, Keepout(), std::vector<Pt>{{1.0, 0.0}});
  EXPECT_FALSE(g.farCost(0.66, 0.0));
  EXPECT_TRUE(g.farCost(0.72, 0.0));
  EXPECT_TRUE(g.farCost(1.08, 0.0));
  EXPECT_FALSE(g.farCost(1.14, 0.0));
  EXPECT_TRUE(g.farCost(1.0, 0.08));
  EXPECT_FALSE(g.farCost(1.0, 0.14));
}

// Two rows of far cells with a gap between them; each row's first cells 5 cm into the gap (10 cm cells).
std::vector<Pt> farGap(double gap)
{
  std::vector<Pt> f;
  for (double x = 1.05; x < 1.5; x += 0.1)
    for (double y = gap / 2 - 0.05; y < 1.5; y += 0.1)
    {
      f.push_back({x, y});
      f.push_back({x, -y});
    }
  return f;
}

// A 50 cm gap (the spec's smallest) seen from far must not cost: with the full footprint its 10 cm C-space
// corridor is gone once the cells sit 5 cm into it. A 21 cm gap (seed 42519's dead end) still costs.
TEST(GridPlanner, FarCSpaceKeepsA50cmGapOpen)
{
  PlannerParams full;
  full.far_shrink = 0.0;
  for (bool shrunk : {true, false})
  {
    GridPlanner g(shrunk ? PlannerParams() : full);
    g.setFootprint(square());
    g.build(std::vector<Pt>(), 0.0, Keepout(), farGap(0.50));
    EXPECT_EQ(g.farCost(1.25, 0.0), !shrunk) << (shrunk ? "eroded: free" : "full footprint: costs");
  }
  GridPlanner g = planner();
  g.build(std::vector<Pt>(), 0.0, Keepout(), farGap(0.21));
  EXPECT_TRUE(g.farCost(1.25, 0.0)) << "a 21 cm gap still costs";
}

// Far readings are 10-20 cm per pixel: a row that looks closed from far away may have a gap. They make
// the route avoid it if it can, they never close it.
TEST(GridPlanner, FarObstaclesAreNotABoundary)
{
  GridPlanner g = planner();
  std::vector<Pt> path;
  g.build(std::vector<Pt>(), 0.0, Keepout(), farBox(1.0, 1.5, -1.5, 1.5));
  ASSERT_TRUE(g.plan(-2.0, 0.0, path)) << "still a path";
  EXPECT_NEAR(yAt(path, 1.2), 0.0, 0.05) << "straight through, the shortest way across";
  EXPECT_TRUE(g.free(1.2, 0.0));
  EXPECT_GT(g.freeDistance(-2.0, 0.0, 1.0, 0.0, 5.0), 4.0) << "no speed limit from it either";
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
