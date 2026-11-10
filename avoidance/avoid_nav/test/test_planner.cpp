// GridPlanner (src/planner.cpp): the hard/soft C-space band, path hysteresis, and smoothing that keeps
// the A* clearance.
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

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
