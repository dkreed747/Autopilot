#include <gtest/gtest.h>

#include <cmath>

#include "autopilot/guidance/DubinsRrtStar.hpp"

namespace arlcore::autopilot {

static DubinsRrtParams testParams() {
  DubinsRrtParams p;
  p.rhoM = 20.0;
  p.marginM = 5.0;
  p.seed = 42;
  p.maxIterations = 1500;
  p.timeBudgetMs = 2000.0;  // generous so the iteration cap binds (determinism)
  return p;
}

//! \brief Minimum clearance over every sample of the chain at 1 m resolution.
static double chainMinClearance(const std::vector<DubinsPath>& chain, const ZoneSet& zones) {
  double minClearance = 1e18;
  for (const DubinsPath& path : chain) {
    const int steps = std::max(1, static_cast<int>(std::ceil(path.lengthM())));
    for (int i = 0; i <= steps; ++i) {
      const Dubins2DPose p = path.sample(path.lengthM() * i / steps);
      minClearance = std::min(minClearance, zones.clearanceM({p.x, p.y}));
    }
  }
  return minClearance;
}

//! \brief Largest positional discontinuity between consecutive chain segments.
static double chainMaxGap(const Dubins2DPose& start, const std::vector<DubinsPath>& chain) {
  double maxGap = 0.0;
  Dubins2DPose prev = start;
  for (const DubinsPath& path : chain) {
    const Dubins2DPose head = path.sample(0.0);
    maxGap = std::max(maxGap, std::hypot(head.x - prev.x, head.y - prev.y));
    prev = path.sample(path.lengthM());
  }
  return maxGap;
}

TEST(DubinsRrtStarTest, FindsDetourAroundKeepOut) {
  ZoneSet zones;
  zones.addZone(ZoneKind::KEEP_OUT, LocalPolygon({{100.0, -80.0}, {200.0, -80.0},
                                                  {200.0, 80.0}, {100.0, 80.0}}));
  const Dubins2DPose start{0.0, 0.0, 0.0};   // facing +x, blocked by the box
  const Dubins2DPose goal{300.0, 0.0, 0.0};

  const auto chain = planDubinsRrtStar(start, goal, zones, testParams(), 0);
  ASSERT_TRUE(chain.has_value());
  ASSERT_FALSE(chain->empty());

  // Compliant at the margin, continuous, and ends at the goal.
  EXPECT_GE(chainMinClearance(chain.value(), zones), testParams().marginM - 1e-6);
  EXPECT_LT(chainMaxGap(start, chain.value()), 0.5);
  const Dubins2DPose end = chain->back().sample(chain->back().lengthM());
  EXPECT_NEAR(end.x, goal.x, 0.5);
  EXPECT_NEAR(end.y, goal.y, 0.5);
}

TEST(DubinsRrtStarTest, DeterministicForFixedSeed) {
  ZoneSet zones;
  zones.addZone(ZoneKind::KEEP_OUT, LocalPolygon({{100.0, -80.0}, {200.0, -80.0},
                                                  {200.0, 80.0}, {100.0, 80.0}}));
  const Dubins2DPose start{0.0, 0.0, 0.0};
  const Dubins2DPose goal{300.0, 0.0, 0.0};

  const auto first = planDubinsRrtStar(start, goal, zones, testParams(), 7);
  const auto second = planDubinsRrtStar(start, goal, zones, testParams(), 7);
  ASSERT_TRUE(first.has_value());
  ASSERT_TRUE(second.has_value());
  ASSERT_EQ(first->size(), second->size());
  for (std::size_t i = 0; i < first->size(); ++i) {
    EXPECT_DOUBLE_EQ((*first)[i].lengthM(), (*second)[i].lengthM());
    EXPECT_EQ((*first)[i].word(), (*second)[i].word());
  }

  // A different salt explores differently (paths may coincide in trivial worlds, but the
  // planner must at least run to completion).
  const auto salted = planDubinsRrtStar(start, goal, zones, testParams(), 8);
  EXPECT_TRUE(salted.has_value());
}

TEST(DubinsRrtStarTest, UnreachableGoalReturnsNullopt) {
  // Goal boxed in on all sides.
  ZoneSet zones;
  zones.addZone(ZoneKind::KEEP_OUT, LocalPolygon({{250.0, -60.0}, {350.0, -60.0},
                                                  {350.0, 60.0}, {250.0, 60.0}}));
  const Dubins2DPose start{0.0, 0.0, 0.0};
  const Dubins2DPose goal{300.0, 0.0, 0.0};  // inside the keep-out

  DubinsRrtParams params = testParams();
  params.maxIterations = 400;  // fail fast
  EXPECT_FALSE(planDubinsRrtStar(start, goal, zones, params, 0).has_value());
}

TEST(DubinsRrtStarTest, StaysInsideKeepIn) {
  ZoneSet zones;
  zones.addZone(ZoneKind::KEEP_IN, LocalPolygon({{-50.0, -150.0}, {350.0, -150.0},
                                                 {350.0, 150.0}, {-50.0, 150.0}}));
  zones.addZone(ZoneKind::KEEP_OUT, LocalPolygon({{100.0, -150.0}, {200.0, -150.0},
                                                  {200.0, 100.0}, {100.0, 100.0}}));
  const Dubins2DPose start{0.0, 0.0, 0.0};
  const Dubins2DPose goal{300.0, 0.0, 0.0};

  const auto chain = planDubinsRrtStar(start, goal, zones, testParams(), 3);
  ASSERT_TRUE(chain.has_value());
  // The only compliant corridor is the gap above the keep-out, inside the keep-in.
  EXPECT_GE(chainMinClearance(chain.value(), zones), testParams().marginM - 1e-6);
}

TEST(DubinsRrtStarTest, EmptyKeepInIntersectionFails) {
  ZoneSet zones;
  zones.addZone(ZoneKind::KEEP_IN, LocalPolygon({{0.0, 0.0}, {100.0, 0.0}, {100.0, 100.0}, {0.0, 100.0}}));
  zones.addZone(ZoneKind::KEEP_IN, LocalPolygon({{500.0, 0.0}, {600.0, 0.0}, {600.0, 100.0}, {500.0, 100.0}}));
  EXPECT_FALSE(planDubinsRrtStar({50.0, 50.0, 0.0}, {550.0, 50.0, 0.0}, zones, testParams(), 0)
                   .has_value());
}

}  // namespace arlcore::autopilot
