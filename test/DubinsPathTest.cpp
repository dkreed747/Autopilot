//---------------------------------------------------------------------------
// Copyright 2025 Pennsylvania State University
//
// Applied Research Laboratory
// Pennsylvania State University
// P.O. Box 30
// State College, PA 16804-0030
//
// DISTRIBUTION STATEMENT A. Approved for public release.
// Distribution is unlimited.
// This software was developed by the Department of the Navy,
// NAVSEA Unmanned and Small Combatants. It is provided under the terms of
// use found in the LICENSE file at the source code root directory.
//
//---------------------------------------------------------------------------

#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include "DubinsPath.h"

namespace arlcore::autopilot {

namespace {

constexpr double kPi = M_PI;

double angleErr(double a, double b) { return std::fabs(std::remainder(a - b, 2.0 * kPi)); }

//! \brief Endpoint check: sampling the full path length must land on the goal pose.
void expectReachesGoal(const Dubins2DPose& start, const Dubins2DPose& goal, double rho) {
  const auto path = DubinsPath::solve(start, goal, rho);
  ASSERT_TRUE(path.has_value());
  const Dubins2DPose end = path->sample(path->lengthM());
  EXPECT_NEAR(end.x, goal.x, 1e-6) << "word=" << path->word();
  EXPECT_NEAR(end.y, goal.y, 1e-6) << "word=" << path->word();
  EXPECT_LT(angleErr(end.theta, goal.theta), 1e-6) << "word=" << path->word();
}

}  // namespace

TEST(DubinsPathTest, StraightLineAhead) {
  const auto path = DubinsPath::solve({0.0, 0.0, 0.0}, {100.0, 0.0, 0.0}, 10.0);
  ASSERT_TRUE(path.has_value());
  EXPECT_NEAR(path->lengthM(), 100.0, 1e-9);
  const Dubins2DPose mid = path->sample(50.0);
  EXPECT_NEAR(mid.x, 50.0, 1e-9);
  EXPECT_NEAR(mid.y, 0.0, 1e-9);
}

TEST(DubinsPathTest, CoincidentPoseIsZeroLength) {
  const auto path = DubinsPath::solve({5.0, -3.0, 1.2}, {5.0, -3.0, 1.2}, 10.0);
  ASSERT_TRUE(path.has_value());
  EXPECT_NEAR(path->lengthM(), 0.0, 1e-9);
}

TEST(DubinsPathTest, UTurnIsTwoRadiiApartCircles) {
  // Goal directly to the left at 2*rho with reversed heading: pure half-circle (length pi*rho).
  const double rho = 20.0;
  const auto path = DubinsPath::solve({0.0, 0.0, 0.0}, {0.0, 2.0 * rho, kPi}, rho);
  ASSERT_TRUE(path.has_value());
  EXPECT_NEAR(path->lengthM(), kPi * rho, 1e-6);
  expectReachesGoal({0.0, 0.0, 0.0}, {0.0, 2.0 * rho, kPi}, rho);
}

TEST(DubinsPathTest, KnownWordSelection) {
  // Far goal straight ahead but offset left with aligned heading favors LSL.
  const auto lsl = DubinsPath::solve({0.0, 0.0, 0.0}, {200.0, 40.0, 0.0}, 10.0);
  ASSERT_TRUE(lsl.has_value());
  EXPECT_EQ(lsl->word()[1], 'S');
  // A close goal behind the vehicle requires a CCC word when d < 4*rho is violated only by
  // CSC candidates being longer; just verify a valid path exists and reaches the goal.
  expectReachesGoal({0.0, 0.0, 0.0}, {5.0, 5.0, kPi}, 10.0);
}

TEST(DubinsPathTest, DegenerateRadiusFallsBackToStraight) {
  const auto path = DubinsPath::solve({0.0, 0.0, 1.0}, {30.0, 40.0, -2.0}, 0.0);
  ASSERT_TRUE(path.has_value());
  EXPECT_NEAR(path->lengthM(), 50.0, 1e-9);
  const Dubins2DPose end = path->sample(path->lengthM());
  EXPECT_NEAR(end.x, 30.0, 1e-6);
  EXPECT_NEAR(end.y, 40.0, 1e-6);
}

TEST(DubinsPathTest, NonFiniteInputRejected) {
  EXPECT_FALSE(DubinsPath::solve({std::nan(""), 0.0, 0.0}, {1.0, 1.0, 0.0}, 10.0).has_value());
  EXPECT_FALSE(DubinsPath::solve({0.0, 0.0, 0.0}, {1.0, std::numeric_limits<double>::infinity(), 0.0}, 10.0)
                   .has_value());
}

TEST(DubinsPathTest, RandomizedEndpointCorrectness) {
  // The definitive solver check: for many random configurations the chosen shortest word,
  // integrated over its full length, must land exactly on the goal pose.
  std::mt19937 rng(42);
  std::uniform_real_distribution<double> pos(-500.0, 500.0);
  std::uniform_real_distribution<double> ang(-kPi, kPi);
  std::uniform_real_distribution<double> radius(1.0, 80.0);
  for (int i = 0; i < 2000; i++) {
    const Dubins2DPose start{pos(rng), pos(rng), ang(rng)};
    const Dubins2DPose goal{pos(rng), pos(rng), ang(rng)};
    expectReachesGoal(start, goal, radius(rng));
  }
}

TEST(DubinsPathTest, RandomizedShortestIsLowerBoundedByEuclidean) {
  std::mt19937 rng(7);
  std::uniform_real_distribution<double> pos(-300.0, 300.0);
  std::uniform_real_distribution<double> ang(-kPi, kPi);
  for (int i = 0; i < 500; i++) {
    const Dubins2DPose start{pos(rng), pos(rng), ang(rng)};
    const Dubins2DPose goal{pos(rng), pos(rng), ang(rng)};
    const auto path = DubinsPath::solve(start, goal, 25.0);
    ASSERT_TRUE(path.has_value());
    const double euclid = std::hypot(goal.x - start.x, goal.y - start.y);
    EXPECT_GE(path->lengthM(), euclid - 1e-6);
  }
}

TEST(DubinsPathTest, SamplingIsMonotonicAndContinuous) {
  const auto path = DubinsPath::solve({0.0, 0.0, 0.5}, {120.0, -60.0, -2.5}, 30.0);
  ASSERT_TRUE(path.has_value());
  Dubins2DPose prev = path->sample(0.0);
  for (double s = 1.0; s <= path->lengthM(); s += 1.0) {
    const Dubins2DPose cur = path->sample(s);
    const double step = std::hypot(cur.x - prev.x, cur.y - prev.y);
    EXPECT_LE(step, 1.0 + 1e-6);   // never jumps further than the arc step
    EXPECT_GT(step, 0.5);          // and always makes progress
    prev = cur;
  }
}

}  // namespace arlcore::autopilot
