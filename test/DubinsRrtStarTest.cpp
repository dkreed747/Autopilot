#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>

#include "InternalTypes.h"
#include "autopilot/guidance/DubinsRrtStar.hpp"

static arlcore::autopilot::DubinsRrtParams testParams() {
  arlcore::autopilot::DubinsRrtParams p;
  p.rhoM = 20.0;
  p.marginM = 5.0;
  p.seed = 42;
  p.maxIterations = 1500;
  p.timeBudgetMs = 2000.0;  // generous so the iteration cap binds (determinism)
  return p;
}

//! \brief Minimum clearance over every sample of the chain at 1 m resolution.
static flt64_t chainMinClearance(const std::vector<arlcore::autopilot::DubinsPath>& chain,
                                 const arlcore::autopilot::ZoneSet& zones) {
  flt64_t minClearance = 1e18;
  for (const arlcore::autopilot::DubinsPath& path : chain) {
    const int32_t steps = std::max(1, static_cast<int32_t>(std::ceil(path.lengthM())));
    for (int32_t i = 0; i <= steps; ++i) {
      const arlcore::autopilot::Dubins2DPose p = path.sample(path.lengthM() * i / steps);
      minClearance = std::min(minClearance, zones.clearanceM({p.x, p.y}));
    }
  }
  return minClearance;
}

//! \brief Largest positional discontinuity between consecutive chain segments.
static flt64_t chainMaxGap(const arlcore::autopilot::Dubins2DPose& start,
                           const std::vector<arlcore::autopilot::DubinsPath>& chain) {
  flt64_t maxGap = 0.0;
  arlcore::autopilot::Dubins2DPose prev = start;
  for (const arlcore::autopilot::DubinsPath& path : chain) {
    const arlcore::autopilot::Dubins2DPose head = path.sample(0.0);
    maxGap = std::max(maxGap, std::hypot(head.x - prev.x, head.y - prev.y));
    prev = path.sample(path.lengthM());
  }
  return maxGap;
}

TEST(DubinsRrtStarTest, FindsDetourAroundKeepOut) {
  // GIVEN: a keep-out box blocking the direct route from start to goal
  arlcore::autopilot::ZoneSet zones;
  zones.addZone(arlcore::autopilot::ZoneKind::KEEP_OUT,
                arlcore::autopilot::LocalPolygon({{100.0, -80.0}, {200.0, -80.0}, {200.0, 80.0}, {100.0, 80.0}}));
  const arlcore::autopilot::Dubins2DPose start{0.0, 0.0, 0.0};  // facing +x, blocked by the box
  const arlcore::autopilot::Dubins2DPose goal{300.0, 0.0, 0.0};

  // WHEN: the planner runs
  const auto chain = arlcore::autopilot::planDubinsRrtStar(start, goal, zones, testParams(), 0);

  // THEN: the chain is compliant at the margin, continuous, and ends at the goal
  ASSERT_TRUE(chain.has_value());
  ASSERT_FALSE(chain->empty());
  EXPECT_GE(chainMinClearance(chain.value(), zones), testParams().marginM - 1e-6);
  EXPECT_LT(chainMaxGap(start, chain.value()), 0.5);
  const arlcore::autopilot::Dubins2DPose end = chain->back().sample(chain->back().lengthM());
  EXPECT_NEAR(end.x, goal.x, 0.5);
  EXPECT_NEAR(end.y, goal.y, 0.5);
}

TEST(DubinsRrtStarTest, DeterministicForFixedSeed) {
  // GIVEN: a keep-out world with identical planner parameters
  arlcore::autopilot::ZoneSet zones;
  zones.addZone(arlcore::autopilot::ZoneKind::KEEP_OUT,
                arlcore::autopilot::LocalPolygon({{100.0, -80.0}, {200.0, -80.0}, {200.0, 80.0}, {100.0, 80.0}}));
  const arlcore::autopilot::Dubins2DPose start{0.0, 0.0, 0.0};
  const arlcore::autopilot::Dubins2DPose goal{300.0, 0.0, 0.0};

  // WHEN: the planner runs twice with the same salt
  const auto first = arlcore::autopilot::planDubinsRrtStar(start, goal, zones, testParams(), 7);
  const auto second = arlcore::autopilot::planDubinsRrtStar(start, goal, zones, testParams(), 7);

  // THEN: the resulting chains are identical segment by segment
  ASSERT_TRUE(first.has_value());
  ASSERT_TRUE(second.has_value());
  ASSERT_EQ(first->size(), second->size());
  for (std::size_t i = 0; i < first->size(); ++i) {
    EXPECT_DOUBLE_EQ((*first)[i].lengthM(), (*second)[i].lengthM());
    EXPECT_EQ((*first)[i].word(), (*second)[i].word());
  }

  // WHEN: the planner runs with a different salt (explores differently)
  const auto salted = arlcore::autopilot::planDubinsRrtStar(start, goal, zones, testParams(), 8);
  // THEN: it still runs to completion (paths may coincide in trivial worlds)
  EXPECT_TRUE(salted.has_value());
}

TEST(DubinsRrtStarTest, UnreachableGoalReturnsNullopt) {
  // GIVEN: a goal boxed in on all sides by a keep-out
  arlcore::autopilot::ZoneSet zones;
  zones.addZone(arlcore::autopilot::ZoneKind::KEEP_OUT,
                arlcore::autopilot::LocalPolygon({{250.0, -60.0}, {350.0, -60.0}, {350.0, 60.0}, {250.0, 60.0}}));
  const arlcore::autopilot::Dubins2DPose start{0.0, 0.0, 0.0};
  const arlcore::autopilot::Dubins2DPose goal{300.0, 0.0, 0.0};  // inside the keep-out

  arlcore::autopilot::DubinsRrtParams params = testParams();
  params.maxIterations = 400;  // fail fast

  // WHEN: the planner runs
  // THEN: no chain is returned
  EXPECT_FALSE(arlcore::autopilot::planDubinsRrtStar(start, goal, zones, params, 0).has_value());
}

TEST(DubinsRrtStarTest, StaysInsideKeepIn) {
  // GIVEN: a keep-in whose only compliant corridor is the gap above the keep-out
  arlcore::autopilot::ZoneSet zones;
  zones.addZone(arlcore::autopilot::ZoneKind::KEEP_IN,
                arlcore::autopilot::LocalPolygon({{-50.0, -150.0}, {350.0, -150.0}, {350.0, 150.0}, {-50.0, 150.0}}));
  zones.addZone(arlcore::autopilot::ZoneKind::KEEP_OUT,
                arlcore::autopilot::LocalPolygon({{100.0, -150.0}, {200.0, -150.0}, {200.0, 100.0}, {100.0, 100.0}}));
  const arlcore::autopilot::Dubins2DPose start{0.0, 0.0, 0.0};
  const arlcore::autopilot::Dubins2DPose goal{300.0, 0.0, 0.0};

  // WHEN: the planner runs
  const auto chain = arlcore::autopilot::planDubinsRrtStar(start, goal, zones, testParams(), 3);

  // THEN: the chain threads the corridor while staying compliant at the margin
  ASSERT_TRUE(chain.has_value());
  EXPECT_GE(chainMinClearance(chain.value(), zones), testParams().marginM - 1e-6);
}

TEST(DubinsRrtStarTest, EmptyKeepInIntersectionFails) {
  // GIVEN: two disjoint keep-in zones with no common area
  arlcore::autopilot::ZoneSet zones;
  zones.addZone(arlcore::autopilot::ZoneKind::KEEP_IN,
                arlcore::autopilot::LocalPolygon({{0.0, 0.0}, {100.0, 0.0}, {100.0, 100.0}, {0.0, 100.0}}));
  zones.addZone(arlcore::autopilot::ZoneKind::KEEP_IN,
                arlcore::autopilot::LocalPolygon({{500.0, 0.0}, {600.0, 0.0}, {600.0, 100.0}, {500.0, 100.0}}));
  // WHEN: a plan is requested between the two zones
  // THEN: no chain is returned
  EXPECT_FALSE(
      arlcore::autopilot::planDubinsRrtStar({50.0, 50.0, 0.0}, {550.0, 50.0, 0.0}, zones, testParams(), 0).has_value());
}
