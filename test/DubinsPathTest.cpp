#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <random>

#include "InternalTypes.h"
#include "autopilot/guidance/DubinsPath.hpp"

constexpr flt64_t kPi = M_PI;

static flt64_t angleErr(flt64_t a, flt64_t b) { return std::fabs(std::remainder(a - b, 2.0 * kPi)); }

//! \brief Endpoint check: sampling the full path length must land on the goal pose.
static void expectReachesGoal(const arlcore::autopilot::Dubins2DPose& start,
                              const arlcore::autopilot::Dubins2DPose& goal, flt64_t rho) {
  const auto path = arlcore::autopilot::DubinsPath::solve(start, goal, rho);
  ASSERT_TRUE(path.has_value());
  const arlcore::autopilot::Dubins2DPose end = path->sample(path->lengthM());
  EXPECT_NEAR(end.x, goal.x, 1e-6) << "word=" << path->word();
  EXPECT_NEAR(end.y, goal.y, 1e-6) << "word=" << path->word();
  EXPECT_LT(angleErr(end.theta, goal.theta), 1e-6) << "word=" << path->word();
}

TEST(DubinsPathTest, StraightLineAhead) {
  // GIVEN: start and goal aligned along +x, 100 m apart
  // WHEN: the path is solved
  const auto path = arlcore::autopilot::DubinsPath::solve({0.0, 0.0, 0.0}, {100.0, 0.0, 0.0}, 10.0);
  // THEN: it is a pure 100 m straight whose midpoint lies on the axis
  ASSERT_TRUE(path.has_value());
  EXPECT_NEAR(path->lengthM(), 100.0, 1e-9);
  const arlcore::autopilot::Dubins2DPose mid = path->sample(50.0);
  EXPECT_NEAR(mid.x, 50.0, 1e-9);
  EXPECT_NEAR(mid.y, 0.0, 1e-9);
}

TEST(DubinsPathTest, CoincidentPoseIsZeroLength) {
  // GIVEN: identical start and goal poses
  // WHEN: the path is solved
  const auto path = arlcore::autopilot::DubinsPath::solve({5.0, -3.0, 1.2}, {5.0, -3.0, 1.2}, 10.0);
  // THEN: a zero-length path is returned
  ASSERT_TRUE(path.has_value());
  EXPECT_NEAR(path->lengthM(), 0.0, 1e-9);
}

TEST(DubinsPathTest, UTurnIsTwoRadiiApartCircles) {
  // GIVEN: a goal directly to the left at 2*rho with reversed heading
  const flt64_t rho = 20.0;
  // WHEN: the path is solved
  const auto path = arlcore::autopilot::DubinsPath::solve({0.0, 0.0, 0.0}, {0.0, 2.0 * rho, kPi}, rho);
  // THEN: it is a pure half-circle (length pi*rho) that lands on the goal
  ASSERT_TRUE(path.has_value());
  EXPECT_NEAR(path->lengthM(), kPi * rho, 1e-6);
  expectReachesGoal({0.0, 0.0, 0.0}, {0.0, 2.0 * rho, kPi}, rho);
}

TEST(DubinsPathTest, KnownWordSelection) {
  // GIVEN: a far goal straight ahead but offset left with aligned heading
  // WHEN: the path is solved
  const auto lsl = arlcore::autopilot::DubinsPath::solve({0.0, 0.0, 0.0}, {200.0, 40.0, 0.0}, 10.0);
  // THEN: a word with a straight middle segment (LSL) is favored
  ASSERT_TRUE(lsl.has_value());
  EXPECT_EQ(lsl->word()[1], 'S');
  // WHEN: a close goal behind the vehicle is solved (CCC-word territory)
  // THEN: a valid path exists and reaches the goal
  expectReachesGoal({0.0, 0.0, 0.0}, {5.0, 5.0, kPi}, 10.0);
}

TEST(DubinsPathTest, DegenerateRadiusFallsBackToStraight) {
  // GIVEN: a zero turn radius
  // WHEN: the path is solved between arbitrary poses
  const auto path = arlcore::autopilot::DubinsPath::solve({0.0, 0.0, 1.0}, {30.0, 40.0, -2.0}, 0.0);
  // THEN: it falls back to the 50 m straight line onto the goal position
  ASSERT_TRUE(path.has_value());
  EXPECT_NEAR(path->lengthM(), 50.0, 1e-9);
  const arlcore::autopilot::Dubins2DPose end = path->sample(path->lengthM());
  EXPECT_NEAR(end.x, 30.0, 1e-6);
  EXPECT_NEAR(end.y, 40.0, 1e-6);
}

TEST(DubinsPathTest, NonFiniteInputRejected) {
  // GIVEN: poses containing NaN and infinity components
  // WHEN: the solver runs on each
  // THEN: no path is produced
  EXPECT_FALSE(arlcore::autopilot::DubinsPath::solve({std::nan(""), 0.0, 0.0}, {1.0, 1.0, 0.0}, 10.0).has_value());
  EXPECT_FALSE(
      arlcore::autopilot::DubinsPath::solve({0.0, 0.0, 0.0}, {1.0, std::numeric_limits<flt64_t>::infinity(), 0.0}, 10.0)
          .has_value());
}

TEST(DubinsPathTest, RandomizedEndpointCorrectness) {
  // GIVEN: a fixed-seed RNG over start/goal poses and turn radii
  std::mt19937 rng(42);
  std::uniform_real_distribution<flt64_t> pos(-500.0, 500.0);
  std::uniform_real_distribution<flt64_t> ang(-kPi, kPi);
  std::uniform_real_distribution<flt64_t> radius(1.0, 80.0);
  // WHEN: 2000 random configurations are solved
  // THEN: the chosen shortest word, integrated over its full length, lands on the goal pose
  for (int32_t i = 0; i < 2000; i++) {
    const arlcore::autopilot::Dubins2DPose start{pos(rng), pos(rng), ang(rng)};
    const arlcore::autopilot::Dubins2DPose goal{pos(rng), pos(rng), ang(rng)};
    expectReachesGoal(start, goal, radius(rng));
  }
}

TEST(DubinsPathTest, RandomizedShortestIsLowerBoundedByEuclidean) {
  // GIVEN: a fixed-seed RNG over start/goal poses
  std::mt19937 rng(7);
  std::uniform_real_distribution<flt64_t> pos(-300.0, 300.0);
  std::uniform_real_distribution<flt64_t> ang(-kPi, kPi);
  // WHEN: each random pair is solved
  // THEN: the path length is never shorter than the Euclidean distance
  for (int32_t i = 0; i < 500; i++) {
    const arlcore::autopilot::Dubins2DPose start{pos(rng), pos(rng), ang(rng)};
    const arlcore::autopilot::Dubins2DPose goal{pos(rng), pos(rng), ang(rng)};
    const auto path = arlcore::autopilot::DubinsPath::solve(start, goal, 25.0);
    ASSERT_TRUE(path.has_value());
    const flt64_t euclid = std::hypot(goal.x - start.x, goal.y - start.y);
    EXPECT_GE(path->lengthM(), euclid - 1e-6);
  }
}

TEST(DubinsPathTest, SamplingIsMonotonicAndContinuous) {
  // GIVEN: a solved path between two poses
  const auto path = arlcore::autopilot::DubinsPath::solve({0.0, 0.0, 0.5}, {120.0, -60.0, -2.5}, 30.0);
  ASSERT_TRUE(path.has_value());
  // WHEN: the path is sampled at 1 m increments
  // THEN: every step makes progress without jumping further than the arc step
  arlcore::autopilot::Dubins2DPose prev = path->sample(0.0);
  for (flt64_t s = 1.0; s <= path->lengthM(); s += 1.0) {
    const arlcore::autopilot::Dubins2DPose cur = path->sample(s);
    const flt64_t step = std::hypot(cur.x - prev.x, cur.y - prev.y);
    EXPECT_LE(step, 1.0 + 1e-6);  // never jumps further than the arc step
    EXPECT_GT(step, 0.5);         // and always makes progress
    prev = cur;
  }
}
