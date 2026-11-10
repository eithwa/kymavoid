// Speed limits (include/avoid_nav/speed_limits.h): braking with a reaction time, corner and turn
// speeds from the turning acceleration, distances to the red lines / wall, corners along a path.
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "avoid_nav/speed_limits.h"

using namespace avoid_nav;

namespace
{
const double INF = std::numeric_limits<double>::infinity();
}

TEST(SpeedLimits, ApproachSpeedStopsWithinTheDistance)
{
  EXPECT_NEAR(approachSpeed(1.0, 1.2, 0.0), std::sqrt(2 * 1.2 * 1.0), 1e-9);   // plain v^2 = 2 a d
  for (double t : {0.0, 0.1, 0.3})
    for (double ve : {0.0, 0.5})
      for (double d : {0.05, 0.5, 2.5})
      {
        const double v = approachSpeed(d, 1.2, t, ve);
        // reaction distance + braking from v to v_end uses exactly d (or v is already <= v_end)
        if (v > ve)
          EXPECT_NEAR(v * t + (v * v - ve * ve) / (2 * 1.2), d, 1e-9) << t << " " << ve << " " << d;
      }
  EXPECT_EQ(approachSpeed(-0.1, 1.2, 0.1), 0.0);
  EXPECT_LT(approachSpeed(2.3, 1.2, 0.1), approachSpeed(2.3, 1.2, 0.0));   // reacting late costs speed
}

TEST(SpeedLimits, CornerSpeedFromCutAndAcceleration)
{
  // 90 deg corner, 10 cm cut: R = 0.1 / (sqrt(2) - 1) = 0.241 m, v = sqrt(1.0 * R)
  EXPECT_NEAR(cornerSpeed(M_PI / 2, 0.10, 1.0), std::sqrt(0.10 / (std::sqrt(2.0) - 1)), 1e-9);
  EXPECT_EQ(cornerSpeed(0.0, 0.05, 1.0), INF);
  EXPECT_LT(cornerSpeed(M_PI / 2, 0.02, 1.0), cornerSpeed(M_PI / 2, 0.10, 1.0));   // tighter = slower
  EXPECT_LT(cornerSpeed(M_PI / 2, 0.05, 1.0), cornerSpeed(M_PI / 6, 0.05, 1.0));   // sharper = slower
  EXPECT_GT(cornerSpeed(-M_PI / 6, 0.05, 1.0), 0.0);                               // either side
}

TEST(SpeedLimits, TurnSpeedDrift)
{
  // at 90 deg the arc of radius v^2 / a drifts R: v = sqrt(a drift); beyond 90 deg no faster
  EXPECT_NEAR(turnSpeed(M_PI / 2, 0.04, 1.0), 0.2, 1e-9);
  EXPECT_NEAR(turnSpeed(M_PI, 0.04, 1.0), 0.2, 1e-9);
  EXPECT_EQ(turnSpeed(0.0, 0.04, 1.0), INF);
  // the drift of an arc of radius v^2 / a turning by phi is R (1 - cos phi) = the allowed drift
  const double v = turnSpeed(0.5, 0.03, 1.0);
  EXPECT_NEAR(v * v / 1.0 * (1 - std::cos(0.5)), 0.03, 1e-9);
}

TEST(SpeedLimits, DistanceToBounds)
{
  // box x [-3, 3], y [-1, 1]; robot at (0, 0.5)
  EXPECT_NEAR(distanceToBounds(0, 0.5, 0, 1, -3, 3, -1, 1), 0.5, 1e-12);
  EXPECT_NEAR(distanceToBounds(0, 0.5, 0, -1, -3, 3, -1, 1), 1.5, 1e-12);
  const double r = std::sqrt(0.5);
  EXPECT_NEAR(distanceToBounds(0, 0.5, r, r, -3, 3, -1, 1), 0.5 / r, 1e-12);       // the line comes first
  EXPECT_NEAR(distanceToBounds(2.9, 0.5, r, r, -3, 3, -1, 1), 0.1 / r, 1e-12);     // the wall comes first
  EXPECT_LT(distanceToBounds(0, 1.1, 0, 1, -3, 3, -1, 1), 0.0);                    // beyond, going further
  EXPECT_NEAR(distanceToBounds(0, 1.1, 0, -1, -3, 3, -1, 1), 2.1, 1e-12);          // beyond, coming back
}

TEST(SpeedLimits, PathCornerLimitBrakesForTheCornerAhead)
{
  const auto clear = [](size_t) { return 0.05; };
  const auto open = [](size_t, double, double) { return 3.0; };
  // straight: no limit
  const std::vector<Pt> straight{{0, 0}, {1, 0}, {3, 0}};
  EXPECT_EQ(pathCornerLimit(straight, 0, {0, 0}, clear, open, 1.0, 1.2, 0.1, 0.02, 5.0), INF);
  // 90 deg corner 1.0 m ahead
  const std::vector<Pt> ell{{0, 0}, {1, 0}, {1, 1}};
  double at = 0;
  const double v = pathCornerLimit(ell, 0, {0, 0}, clear, open, 1.0, 1.2, 0.1, 0.02, 5.0, 0.15, &at);
  EXPECT_NEAR(at, 1.0, 1e-9);
  EXPECT_NEAR(v, approachSpeed(1.0, 1.2, 0.1, cornerSpeed(M_PI / 2, 0.05, 1.0)), 1e-9);
  // from half way along the first leg the corner is 0.5 m away: lower
  EXPECT_LT(pathCornerLimit(ell, 0, {0.5, 0}, clear, open, 1.0, 1.2, 0.1, 0.02, 5.0), v);
  // beyond the horizon: ignored
  EXPECT_EQ(pathCornerLimit(ell, 0, {0, 0}, clear, open, 1.0, 1.2, 0.1, 0.02, 0.5), INF);
  // the clearance below min_cut does not stop the robot dead
  EXPECT_GT(pathCornerLimit(ell, 0, {0.9, 0}, [](size_t) { return 0.0; }, open, 1.0, 1.2, 0.1, 0.02, 5.0), 0.0);
}

TEST(SpeedLimits, OvershootIntoAFaceAhead)
{
  // 51 deg turn with a box face 0.375 m beyond the corner (seed 16): sqrt(1.0 * 0.375 / sin 51 deg)
  EXPECT_NEAR(overshootSpeed(51 * M_PI / 180, 0.375, 1.0), 0.6947, 1e-3);
  EXPECT_NEAR(overshootSpeed(M_PI, 0.25, 1.0), 0.5, 1e-9);          // beyond 90 deg: R itself
  EXPECT_EQ(overshootSpeed(0.0, 0.1, 1.0), INF);
  // the same corner, wide on the inside, with and without a face ahead
  const std::vector<Pt> p{{0, 0}, {1, 0}, {1.6, 0.74}};
  const auto clear = [](size_t) { return 0.22; };
  const auto open = [](size_t, double, double) { return 3.0; };
  const auto face = [](size_t, double ux, double) { return ux > 0.99 ? 0.375 : 3.0; };
  const double v_open = pathCornerLimit(p, 0, {0.9, 0}, clear, open, 1.0, 1.2, 0.0, 0.02, 5.0);
  const double v_face = pathCornerLimit(p, 0, {0.9, 0}, clear, face, 1.0, 1.2, 0.0, 0.02, 5.0);
  EXPECT_GT(v_open, 1.3);                                              // the cut alone allows ~1.4 m/s
  // 0.1 m before the corner: brake down to the overshoot speed by then
  EXPECT_NEAR(v_face, approachSpeed(0.1, 1.2, 0.0, overshootSpeed(std::atan2(0.74, 0.6), 0.375, 1.0)), 1e-6);
  EXPECT_LT(v_face, 0.9);
}

TEST(SpeedLimits, ShortStepsAreOneCorner)
{
  // 90 deg turned in three 30 deg steps 5 cm apart: as slow as one 90 deg corner, not a 30 deg one
  std::vector<Pt> p{{0, 0}, {1, 0}};
  double h = 0;
  for (int k = 0; k < 3; ++k)
  {
    h += M_PI / 6;
    p.push_back({p.back().x + 0.05 * std::cos(h), p.back().y + 0.05 * std::sin(h)});
  }
  p.push_back({p.back().x, p.back().y + 1.0});
  const auto clear = [](size_t) { return 0.05; };
  const auto open = [](size_t, double, double) { return 3.0; };
  const double v = pathCornerLimit(p, 0, {0, 0}, clear, open, 1.0, 1.2, 0.0, 0.02, 5.0);
  EXPECT_NEAR(v, approachSpeed(1.0, 1.2, 0.0, cornerSpeed(M_PI / 2, 0.05, 1.0)), 1e-9);
  EXPECT_LT(v, approachSpeed(1.0, 1.2, 0.0, cornerSpeed(M_PI / 6, 0.05, 1.0)));
}

TEST(SpeedLimits, PathCapLimitBrakesBeforeANarrowPassage)
{
  // straight path; the cap drops to 0.3 m/s from x = 1.0 (a narrow passage)
  const std::vector<Pt> p{{0, 0}, {3, 0}};
  const auto cap = [](const Pt& q) { return q.x >= 1.0 - 1e-9 ? 0.3 : 5.0; };
  const double v = pathCapLimit(p, 0, {0, 0}, cap, 1.2, 0.1, 5.0);
  EXPECT_NEAR(v, approachSpeed(1.0, 1.2, 0.1, 0.3), 1e-6);   // the first sample inside decides
  EXPECT_LT(v, 5.0);
  // closer to the passage: lower; inside it: down to the cap one sample (5 cm) ahead (the cap where
  // the robot stands is nav_node's own clearance limit)
  EXPECT_LT(pathCapLimit(p, 0, {0.6, 0}, cap, 1.2, 0.1, 5.0), v);
  EXPECT_NEAR(pathCapLimit(p, 0, {1.2, 0}, cap, 1.2, 0.0, 5.0), approachSpeed(0.05, 1.2, 0.0, 0.3), 1e-6);
  // a passage beyond the horizon does not count: only the 5.0 cap of the samples within 0.5 m
  EXPECT_GT(pathCapLimit(p, 0, {0, 0}, cap, 1.2, 0.1, 0.5), 4.5);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
