// Localizer (include/avoid_nav/localizer.h): a lost front wall is matched again, and nothing else is
// taken for it.
#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "avoid_nav/localizer.h"

using namespace avoid_nav;

namespace
{
const FieldLine FRONT{"wall_front", 1, 0, 3.475, -2.55, 2.55};

// base-frame points of a straight face parallel to the front wall, `dist` ahead of a robot facing +x
std::vector<Pt> face(double dist, double y0, double y1)
{
  std::vector<Pt> pts;
  for (double y = y0; y <= y1 + 1e-9; y += 0.02)
    pts.push_back({dist, y});
  return pts;
}

// the robot truly stands at x = 2.0 and sees the wall 1.475 m ahead; the filter starts at x0 and is
// confident (sigma_x 2 cm); returns its x after `scans` scans 1/30 s apart, standing still
double xAfter(double x0, const std::vector<Pt>& pts, int scans, const LocalizerParams& p = LocalizerParams())
{
  Localizer loc(p);
  loc.reset(x0, 0.0, 0.0);
  for (int k = 0; k < scans; ++k)
  {
    loc.predict(0.0, 0.0, 0.0, 1.0 / 30);
    loc.updateLines(pts, {FRONT}, 2.5);
  }
  return loc.x();
}
}  // namespace

TEST(Localizer, LostFrontWallIsMatchedAgain)
{
  const std::vector<Pt> wall = face(1.475, -0.8, 0.8);
  // 45 cm behind: outside the normal association gate
  LocalizerParams never;
  never.lost_scans = 1000000;
  EXPECT_NEAR(xAfter(1.55, wall, 60, never), 1.55, 1e-6) << "the old filter never matched it";
  const double x = xAfter(1.55, wall, 60);
  EXPECT_NEAR(x, 2.0, 0.03) << "matched again in the wide band";
  // and only after lost_scans scans in a row
  EXPECT_NEAR(xAfter(1.55, wall, LocalizerParams().lost_scans), 1.55, 1e-6);
}

TEST(Localizer, ABoxFaceIsNotTakenForTheWall)
{
  // the filter is right (x = 2.0); the wall is hidden, a box face 0.75 m before it is in view
  const std::vector<Pt> box = face(0.725, -0.225, 0.225);
  EXPECT_NEAR(xAfter(2.0, box, 60), 2.0, 1e-6);
  // a longer face, as long as a wall piece, 0.75 m short: still outside the 0.6 m band
  EXPECT_NEAR(xAfter(2.0, face(0.725, -0.6, 0.6), 60), 2.0, 1e-6);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
