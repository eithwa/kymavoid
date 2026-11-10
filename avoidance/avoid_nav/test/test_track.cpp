// Reference tracking (include/avoid_nav/track.h): corner rounding that keeps the clearance, and a
// closed loop through a corner with the command limiter nav_node uses.
#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "avoid_nav/track.h"

using namespace avoid_nav;

namespace
{
const auto kFar = [](double, double) { return 1.0; };

// C-space of a box filling the inside of the left turn at (1, 0): x <= 1 - c0 and y >= c0, so both
// legs run c0 from its faces (what a planner path round a box corner looks like)
struct BoxCorner
{
  double c0;
  double operator()(double x, double y) const { return std::hypot(std::max(0.0, x - (1 - c0)), std::max(0.0, c0 - y)); }
};

struct Trip
{
  double time = 0, x_turn = 1e9, x_max = -1e9, max_err = 0, min_clear = 1e9, y = 0;
};

// nav_node's control loop at 20 Hz on the reference: trackStep, then the velocity-vector limiter
// (a_max speeding up and turning, a_brake slowing down)
template <class Clearance>
Trip drive(const std::vector<RefPoint>& r, Clearance clear, const TrackParams& p, double y_end, Pt start = {1e9, 0},
           Pt v0 = {0, 0})
{
  const double dt = 0.05, v_max = 3.0;
  Trip o;
  double x = start.x < 1e8 ? start.x : r.front().x, y = start.x < 1e8 ? start.y : r.front().y, vx = v0.x, vy = v0.y;
  for (int i = 0; i < 2000 && y < y_end; ++i)
  {
    const TrackStep st = trackStep(r, {x, y}, {vx, vy}, v_max, p, clear(x, y));
    const double v = std::min(v_max, st.v()), cur = std::hypot(vx, vy);
    double dx = v * st.dir.x - vx, dy = v * st.dir.y - vy;
    const double dv = std::hypot(dx, dy), amax = (v < cur ? p.a_brake : p.a_max) * dt;
    if (dv > amax)
    {
      dx *= amax / dv;
      dy *= amax / dv;
    }
    vx += dx;
    vy += dy;
    x += vx * dt;
    y += vy * dt;
    o.time += dt;
    o.max_err = std::max(o.max_err, st.err);
    o.min_clear = std::min(o.min_clear, clear(x, y));
    o.x_max = std::max(o.x_max, x);
    if (vy > 0.02 && o.x_turn > 1e8)
      o.x_turn = x;
  }
  o.y = y;
  return o;
}

double minClear(const std::vector<RefPoint>& r)
{
  double m = 1e9;
  for (const RefPoint& q : r)
    m = std::min(m, q.clear);
  return m;
}

// left turn at (1, 0) towards +y
const std::vector<Pt> kCorner{{0, 0}, {1, 0}, {1, 1}};
}  // namespace

TEST(Rounding, OpenCornerGetsTheLargestArcTheLegsAllow)
{
  std::vector<double> radii;
  const auto r = roundedReference(kCorner, kFar, RoundParams(), &radii);
  ASSERT_GE(r.size(), 3u);
  EXPECT_NEAR(radii[1], 1.0, 1e-6);            // first and last leg are the corner's own: t = 1 m
  EXPECT_NEAR(r.back().s, M_PI / 2, 0.01);     // the whole path is the quarter circle
  EXPECT_NEAR(r.front().x, 0, 1e-9);
  EXPECT_NEAR(r.back().y, 1, 1e-9);
}

TEST(Rounding, ArcKeepsTheClearanceDownToKeep)
{
  // round a box corner the path passes 15 cm from: the arc may cut to keep, no further
  const BoxCorner d{0.15};
  RoundParams p;
  p.keep = 0.06;
  std::vector<double> radii;
  const auto r = roundedReference(kCorner, d, p, &radii);
  EXPECT_GT(radii[1], 0.2);                    // still a real arc ...
  EXPECT_LT(radii[1], 1.0);                    // ... but smaller than the legs allow
  EXPECT_GE(minClear(r), p.keep - p.tol - 1e-9);
  EXPECT_LT(minClear(r), p.keep + 0.02);       // and it does use the room
}

TEST(Rounding, TightCornerIsNotCutCloser)
{
  // the path passes only 4 cm from the box (a narrow gap): never closer than that
  const BoxCorner d{0.04};
  RoundParams p;
  const auto r = roundedReference(kCorner, d, p);
  EXPECT_GE(minClear(r), 0.04 - p.tol - 1e-9);
}

TEST(Rounding, ShortLegIsSharedByItsTwoCorners)
{
  // S bend: two opposite 90 deg corners 20 cm apart, each gets half of the leg between them
  const std::vector<Pt> path{{0, 0}, {1, 0}, {1, 0.2}, {2, 0.2}};
  std::vector<double> radii;
  const auto r = roundedReference(path, kFar, RoundParams(), &radii);
  EXPECT_NEAR(radii[1], 0.1, 1e-6);
  EXPECT_NEAR(radii[2], 0.1, 1e-6);
  for (size_t i = 1; i < r.size(); ++i)
    EXPECT_GT(r[i].s, r[i - 1].s);
}

TEST(Tracking, DirectionPullsBackOntoTheReference)
{
  const auto r = roundedReference({{0, 0}, {2, 0}}, kFar, RoundParams());
  size_t seg;
  double s, err;
  Pt foot;
  closestOnRef(r, 0.5, 0.1, seg, s, foot);
  EXPECT_NEAR(s, 0.5, 1e-9);
  const Pt u = trackDirection(r, {0.5, 0.1}, s, foot, 0.0, 0.5, 2.0, &err);
  EXPECT_NEAR(err, 0.1, 1e-9);
  EXPECT_GT(u.x, 0.9);
  EXPECT_LT(u.y, -0.3);                        // 0.5 m/s along, 0.2 m/s back
}

TEST(Tracking, ClosedLoopTurnsEarlyAndStaysOnTheArc)
{
  // 3 m run-up, then a 90 deg left turn round a box whose faces are 15 cm from the path
  const BoxCorner d{0.15};
  RoundParams rp;
  const std::vector<Pt> path{{-2, 0}, {1, 0}, {1, 2}};
  TrackParams p;
  p.preview_time = 0;                          // no latency in this loop
  const Trip rounded = drive(roundedReference(path, d, rp), d, p, 1.5);
  EXPECT_GT(rounded.y, 1.5);                   // got round
  EXPECT_LT(rounded.x_turn, 1.0 - 0.2);        // turning well before the corner
  EXPECT_LT(rounded.x_max, 1.0 + 0.01);        // no swinging wide past it
  EXPECT_LT(rounded.max_err, 0.015);           // on the arc (1 cm at the arc's own speed limit, 0.8 m/s^2)
  EXPECT_GE(rounded.min_clear, rp.keep - rp.tol - 0.005);   // and no cutting in

  // the same corner left sharp (what pure pursuit with a short lookahead meets): the turn starts
  // at the corner and the robot ends up outside it
  RoundParams sharp;
  sharp.r_max = 1e-6;
  const Trip late = drive(roundedReference(path, d, sharp), d, p, 1.5);
  EXPECT_GT(late.x_turn, 1.0 - 0.02);
  EXPECT_GT(late.x_max, rounded.x_max + 0.005);
}

TEST(Tracking, PreviewTurnsSlightlyEarlierStillClear)
{
  // with the default preview (latency compensation) and no latency to compensate, the robot runs
  // a little inside the arc: bounded
  const BoxCorner d{0.15};
  RoundParams rp;
  const Trip r = drive(roundedReference({{-2, 0}, {1, 0}, {1, 2}}, d, rp), d, TrackParams(), 1.5);
  EXPECT_GT(r.y, 1.5);
  EXPECT_LT(r.max_err, 0.03);
  EXPECT_GE(r.min_clear, rp.keep - 0.03);
}

TEST(Tracking, NewPathAtAnAngleIsJoinedTheWayTheRobotMoves)
{
  // a replan hands the robot, running at 1 m/s along +x, a path whose first leg climbs at 30 deg.
  // Joined from the velocity, the reference is what the robot can drive at that speed; without the
  // join the reference starts at 30 deg and the robot, still going along +x, is off it at once.
  const std::vector<Pt> path{{0, 0}, {1.0, 0.577}, {1.0, 3.0}};
  TrackParams p;
  p.preview_time = 0;
  const Pt vel{1.0, 0.0};
  std::vector<Pt> joined = path;
  ASSERT_TRUE(joinFromVelocity(joined, vel, p.a_lat, kFar, RoundParams()));
  const Trip with = drive(roundedReference(joined, kFar, RoundParams()), kFar, p, 1.5, {0, 0}, vel);
  const Trip without = drive(roundedReference(path, kFar, RoundParams()), kFar, p, 1.5, {0, 0}, vel);
  EXPECT_GT(with.y, 1.5);
  EXPECT_LT(with.max_err, 0.02);
  EXPECT_GT(without.max_err, 0.15);
}

TEST(Tracking, JoinThatWouldHitSomethingIsNotUsed)
{
  // the same join with a wall at x = 0.9 in the way of the wide arc: path left as planned
  const auto wall = [](double x, double) { return std::max(0.0, 0.9 - x); };
  const std::vector<Pt> path{{0, 0}, {0.5, 0.289}, {0.5, 3.0}};
  std::vector<Pt> joined = path;
  EXPECT_FALSE(joinFromVelocity(joined, {1.0, 0.0}, 0.8, wall, RoundParams()));
  EXPECT_EQ(joined.size(), path.size());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
