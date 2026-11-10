// OccGrid clearing rules (include/avoid_nav/occupancy.h):
//  - a precise near reading clears a ghost that a coarse far reading left in front of a surface
//    (the old fixed 10 cm stop could not, which narrowed passable gaps);
//  - cells within corner_protect of a neighbouring beam's hit are not cleared (obstacle corners);
//  - a coarse hit needs a second one before it counts as an obstacle, a precise one does not.
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "avoid_nav/occupancy.h"

using avoid_nav::OccGrid;
using avoid_nav::OccParams;
using avoid_nav::Pt;

namespace
{
const int N = 120;   // like the camera: one beam every 3 deg
const double INF = std::numeric_limits<double>::infinity();
const double Y = 0.005;   // sensors half a cell off the grid lines (2 cm cells, edges at y = 0)

sensor_msgs::LaserScan scan(const std::vector<std::pair<int, double>>& hits)
{
  sensor_msgs::LaserScan s;
  s.angle_min = 0.0;
  s.angle_increment = 2 * M_PI / N;
  s.ranges.assign(N, INF);
  for (const auto& h : hits)
    s.ranges[(h.first + N) % N] = h.second;
  return s;
}

bool occupiedAt(const OccGrid& g, double x, double y)
{
  for (const Pt& p : g.occupied(x, y, 0.015))
    if (std::fabs(p.x - x) < 0.011 && std::fabs(p.y - y) < 0.011)
      return true;
  return false;
}

// camera-like range uncertainty: 3 cm per metre (coarse beyond 1 m)
std::vector<float> cameraLike()
{
  std::vector<float> t;
  for (int cm = 0; cm <= 800; ++cm)
    t.push_back(0.03f * cm / 100.0f);
  return t;
}

// Ghost 8 cm in front of a wall at x = 1.01, left by two coarse readings from 2.5 m away; then the
// robot looks at the wall from 0.6 m.
bool ghostSurvivesNearView(const OccParams& p)
{
  OccGrid g(p);
  g.setSigmaTable(cameraLike());
  for (int k = 0; k < 2; ++k)
    g.update(-1.5, Y, 0.0, scan({{0, 2.43}}), 3.0);
  EXPECT_TRUE(occupiedAt(g, 0.93, 0.01)) << "two coarse hits must make the ghost";
  const double side = 0.61 / std::cos(2 * M_PI / N);
  for (int k = 0; k < 10; ++k)
    g.update(0.4, Y, 0.0, scan({{0, 0.61}, {1, side}, {-1, side}}), 3.0);
  EXPECT_TRUE(occupiedAt(g, 1.01, 0.01)) << "the real wall must stay";
  return occupiedAt(g, 0.93, 0.01);
}
}  // namespace

TEST(OccGrid, PreciseNearReadingClearsFarGhost)
{
  EXPECT_FALSE(ghostSurvivesNearView(OccParams()));
}

TEST(OccGrid, OldFixedTenCentimetreStopKeptTheGhost)
{
  OccParams old;
  old.clear_sigmas = 0.0;
  old.clear_margin = 0.10;
  EXPECT_TRUE(ghostSurvivesNearView(old));
}

// A cell on beam 0 within 3 cm of beam 1's hit (an obstacle corner between the beams) survives
// beam 0 seeing through; without the neighbouring hit the same cell is cleared.
TEST(OccGrid, CornerNextToNeighbourHitIsProtected)
{
  for (bool neighbour_hits : {true, false})
  {
    OccGrid g;
    for (int k = 0; k < 3; ++k)
      g.update(0.0, Y, 0.0, scan({{0, 0.41}}), 3.0);          // mark the cell at (0.41, 0.01)
    ASSERT_TRUE(occupiedAt(g, 0.41, 0.01));
    for (int k = 0; k < 10; ++k)
      g.update(0.0, Y, 0.0, neighbour_hits ? scan({{1, 0.40}}) : scan({}), 3.0);
    EXPECT_EQ(occupiedAt(g, 0.41, 0.01), neighbour_hits) << (neighbour_hits ? "protected" : "cleared");
  }
}

TEST(OccGrid, CoarseHitNeedsConfirmation)
{
  OccGrid coarse;
  coarse.setSigmaTable(std::vector<float>(801, 0.05f));
  coarse.update(0.0, Y, 0.0, scan({{0, 2.01}}), 3.0);
  EXPECT_FALSE(occupiedAt(coarse, 2.01, 0.01));
  coarse.update(0.0, Y, 0.0, scan({{0, 2.01}}), 3.0);
  EXPECT_TRUE(occupiedAt(coarse, 2.01, 0.01));

  OccGrid precise;                                              // no table: lidar, 2 cm
  precise.update(0.0, Y, 0.0, scan({{0, 2.01}}), 3.0);
  EXPECT_TRUE(occupiedAt(precise, 2.01, 0.01));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
