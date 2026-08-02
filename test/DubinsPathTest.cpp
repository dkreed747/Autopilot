#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <random>
#include <string>

#include "InternalTypes.h"
#include "autopilot/guidance/DubinsPath.hpp"

constexpr flt64_t kPi = M_PI;

static flt64_t angleErr(flt64_t a, flt64_t b) { return std::fabs(std::remainder(a - b, 2.0 * kPi)); }

//! \brief The curvature a segment of this type must report at radius `rho`.
static flt64_t expectedCurvature(arlcore::autopilot::DubinsSegment::Type type, flt64_t rho) {
  if (type == arlcore::autopilot::DubinsSegment::Type::LEFT) {
    return 1.0 / rho;
  }
  if (type == arlcore::autopilot::DubinsSegment::Type::RIGHT) {
    return -1.0 / rho;
  }
  return 0.0;
}

//! \brief Total turn accumulated by sampling theta, unwrapped across the 2*pi branch cut.
static flt64_t sampledHeadingChange(const arlcore::autopilot::DubinsPath& path, int32_t nodes) {
  flt64_t total = 0.0;
  flt64_t prev = path.sample(0.0).theta;
  for (int32_t k = 1; k <= nodes; k++) {
    const flt64_t cur = path.sample(path.lengthM() * static_cast<flt64_t>(k) / static_cast<flt64_t>(nodes)).theta;
    total += std::remainder(cur - prev, 2.0 * kPi);
    prev = cur;
  }
  return total;
}

//! \brief Occurrences of the least-seen word, or 0 until all six have appeared.
static int32_t minWordCount(const std::map<std::string, int32_t>& seen) {
  if (seen.size() < 6u) {
    return 0;
  }
  int32_t fewest = std::numeric_limits<int32_t>::max();
  for (const std::pair<const std::string, int32_t>& entry : seen) {
    fewest = std::min(fewest, entry.second);
  }
  return fewest;
}

//! \brief Total turn from the segment table, which is the exact analytic answer.
static flt64_t segmentHeadingChange(const arlcore::autopilot::DubinsPath& path) {
  flt64_t total = 0.0;
  for (const arlcore::autopilot::DubinsSegment& seg : path.segments()) {
    total += expectedCurvature(seg.type, path.rhoM()) * seg.lengthM;
  }
  return total;
}

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

TEST(DubinsPathTest, RhoReportsTheSolveRadius) {
  // GIVEN: a path solved at an arbitrary turn radius
  const auto path = arlcore::autopilot::DubinsPath::solve({0.0, 0.0, 0.3}, {120.0, 80.0, -1.1}, 17.5);
  ASSERT_TRUE(path.has_value());
  // WHEN: the radius is read back
  // THEN: it is the radius the path was solved with
  EXPECT_DOUBLE_EQ(path->rhoM(), 17.5);
  // WHEN: the radius was degenerate, so no turning circle exists
  const auto straight = arlcore::autopilot::DubinsPath::solve({0.0, 0.0, 1.0}, {30.0, 40.0, -2.0}, 0.0);
  ASSERT_TRUE(straight.has_value());
  // THEN: zero is reported rather than a sentinel radius
  EXPECT_DOUBLE_EQ(straight->rhoM(), 0.0);
}

TEST(DubinsPathTest, CurvatureIsPlusOverRhoOnLeftAndMinusOnRight) {
  // GIVEN: configurations chosen to exercise left arcs, right arcs and straights
  // (LEFT is counterclockwise in math theta, which is a decreasing azimuth)
  const flt64_t rho = 10.0;
  const arlcore::autopilot::Dubins2DPose starts[3] = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
  const arlcore::autopilot::Dubins2DPose goals[3] = {{200.0, 40.0, 0.0}, {200.0, -40.0, 0.0}, {5.0, 5.0, kPi}};
  int32_t leftSeen = 0;
  int32_t rightSeen = 0;
  int32_t straightSeen = 0;
  // WHEN: each segment is probed at its midpoint
  for (int32_t c = 0; c < 3; c++) {
    const auto path = arlcore::autopilot::DubinsPath::solve(starts[c], goals[c], rho);
    ASSERT_TRUE(path.has_value());
    const std::array<arlcore::autopilot::DubinsSegment, 3> segs = path->segments();
    flt64_t base = 0.0;
    for (std::size_t i = 0; i < 3; i++) {
      if (segs[i].lengthM > 1e-6) {
        // THEN: it reports +1/rho, -1/rho or 0 according to its type
        EXPECT_DOUBLE_EQ(path->curvatureAt(base + 0.5 * segs[i].lengthM), expectedCurvature(segs[i].type, rho))
            << "word=" << path->word() << " seg=" << i;
        leftSeen += segs[i].type == arlcore::autopilot::DubinsSegment::Type::LEFT ? 1 : 0;
        rightSeen += segs[i].type == arlcore::autopilot::DubinsSegment::Type::RIGHT ? 1 : 0;
        straightSeen += segs[i].type == arlcore::autopilot::DubinsSegment::Type::STRAIGHT ? 1 : 0;
      }
      base += segs[i].lengthM;
    }
  }
  // THEN: all three segment types were actually covered
  EXPECT_GT(leftSeen, 0);
  EXPECT_GT(rightSeen, 0);
  EXPECT_GT(straightSeen, 0);
}

TEST(DubinsPathTest, CurvatureAtBoundariesMatchesSampleClamping) {
  // GIVEN: a three-segment path
  const flt64_t rho = 12.0;
  const auto path = arlcore::autopilot::DubinsPath::solve({0.0, 0.0, 0.0}, {150.0, 60.0, 1.0}, rho);
  ASSERT_TRUE(path.has_value());
  const std::array<arlcore::autopilot::DubinsSegment, 3> segs = path->segments();
  ASSERT_GT(segs[0].lengthM, 1e-6);
  // WHEN: the exact segment boundary is probed
  // THEN: it belongs to the preceding segment, exactly as sample() clamps it
  EXPECT_DOUBLE_EQ(path->curvatureAt(segs[0].lengthM), expectedCurvature(segs[0].type, rho));
  // WHEN: arc lengths outside [0, lengthM()] are probed
  // THEN: they clamp to the endpoints instead of running off the segment table
  EXPECT_DOUBLE_EQ(path->curvatureAt(-5.0), path->curvatureAt(0.0));
  EXPECT_DOUBLE_EQ(path->curvatureAt(path->lengthM() + 100.0), path->curvatureAt(path->lengthM()));
}

TEST(DubinsPathTest, IntegratedCurvatureMatchesSampledHeadingChangeForEveryWord) {
  // GIVEN: a fixed-seed sweep mixing wide (CSC) and tight (CCC) configurations until every
  // one of the six canonical words has been exercised
  std::mt19937 rng(1337);
  std::uniform_real_distribution<flt64_t> wide(-400.0, 400.0);
  std::uniform_real_distribution<flt64_t> tight(-40.0, 40.0);
  std::uniform_real_distribution<flt64_t> ang(-kPi, kPi);
  std::uniform_real_distribution<flt64_t> wideRadius(8.0, 40.0);
  std::uniform_real_distribution<flt64_t> tightRadius(8.0, 30.0);
  std::map<std::string, int32_t> seen;
  constexpr int32_t kNodes = 4000;
  constexpr int32_t kRiemann = 20000;
  // WHEN: each path's turn is measured three independent ways
  for (int32_t i = 0; i < 20000 && minWordCount(seen) < 50; i++) {
    const bool compact = (i % 2) == 1;
    const flt64_t rho = compact ? tightRadius(rng) : wideRadius(rng);
    const arlcore::autopilot::Dubins2DPose start{compact ? tight(rng) : wide(rng), compact ? tight(rng) : wide(rng),
                                                 ang(rng)};
    const arlcore::autopilot::Dubins2DPose goal{compact ? tight(rng) : wide(rng), compact ? tight(rng) : wide(rng),
                                                ang(rng)};
    const auto path = arlcore::autopilot::DubinsPath::solve(start, goal, rho);
    ASSERT_TRUE(path.has_value());
    if (path->lengthM() < 1e-6) {
      continue;
    }
    seen[path->word()]++;

    // THEN: the exact segment-table turn agrees with the sampled heading change
    const flt64_t exact = segmentHeadingChange(path.value());
    EXPECT_NEAR(exact, sampledHeadingChange(path.value(), kNodes), 1e-9) << "word=" << path->word();

    // THEN: and a left-Riemann sum of curvatureAt agrees within its discretization bound
    const flt64_t ds = path->lengthM() / static_cast<flt64_t>(kRiemann);
    flt64_t riemann = 0.0;
    for (int32_t k = 0; k < kRiemann; k++) {
      riemann += path->curvatureAt(static_cast<flt64_t>(k) * ds) * ds;
    }
    EXPECT_NEAR(riemann, exact, 5.0 * ds / rho + 1e-9) << "word=" << path->word();
  }
  // THEN: all six canonical words were covered, so no word escaped the check
  EXPECT_EQ(seen.size(), 6u);
}

TEST(DubinsPathTest, DegenerateRadiusPathHasZeroCurvatureEverywhere) {
  // GIVEN: turn radii at and below the degenerate threshold
  for (flt64_t rho : {0.0, 5e-4}) {
    const auto path = arlcore::autopilot::DubinsPath::solve({0.0, 0.0, 1.0}, {30.0, 40.0, -2.0}, rho);
    ASSERT_TRUE(path.has_value());
    // WHEN: the resulting straight run is probed anywhere, inside or outside its range
    // THEN: curvature is exactly zero and never divides by the absent radius
    EXPECT_EQ(path->word(), "SSS");
    for (flt64_t s : {-1.0, 0.0, 25.0, 50.0, 60.0}) {
      EXPECT_DOUBLE_EQ(path->curvatureAt(s), 0.0) << "rho=" << rho << " s=" << s;
    }
  }
}

TEST(DubinsPathTest, CoincidentPosesHaveZeroLengthAndZeroCurvature) {
  // GIVEN: identical start and goal poses
  const auto path = arlcore::autopilot::DubinsPath::solve({5.0, -3.0, 1.2}, {5.0, -3.0, 1.2}, 10.0);
  ASSERT_TRUE(path.has_value());
  // WHEN: the zero-length path is probed at and past its end
  // THEN: curvature is zero without indexing past the segment table
  EXPECT_DOUBLE_EQ(path->lengthM(), 0.0);
  EXPECT_DOUBLE_EQ(path->curvatureAt(0.0), 0.0);
  EXPECT_DOUBLE_EQ(path->curvatureAt(5.0), 0.0);
}

TEST(DubinsPathTest, CurvatureIsFiniteAndBoundedForEveryRandomizedPath) {
  // GIVEN: a fixed-seed sweep of random configurations
  std::mt19937 rng(42);
  std::uniform_real_distribution<flt64_t> pos(-500.0, 500.0);
  std::uniform_real_distribution<flt64_t> ang(-kPi, kPi);
  std::uniform_real_distribution<flt64_t> radius(1.0, 80.0);
  // WHEN: each path is probed across its length
  for (int32_t i = 0; i < 2000; i++) {
    const arlcore::autopilot::Dubins2DPose start{pos(rng), pos(rng), ang(rng)};
    const arlcore::autopilot::Dubins2DPose goal{pos(rng), pos(rng), ang(rng)};
    const flt64_t rho = radius(rng);
    const auto path = arlcore::autopilot::DubinsPath::solve(start, goal, rho);
    ASSERT_TRUE(path.has_value());
    // THEN: curvature is always finite and never exceeds the solved radius' bound
    for (int32_t k = 0; k <= 4; k++) {
      const flt64_t s = path->lengthM() * static_cast<flt64_t>(k) / 4.0;
      EXPECT_TRUE(std::isfinite(path->curvatureAt(s)));
      EXPECT_LE(std::fabs(path->curvatureAt(s)), 1.0 / rho + 1e-12);
    }
  }
}
