#include "autopilot/guidance/DubinsRrtStar.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <random>

#include "InternalTypes.h"
#include "Logger.h"

namespace arlcore::autopilot {

struct Node {
  Dubins2DPose pose;
  int32_t parent = -1;
  DubinsPath edge;  // path from parent to this node (unset for the root)
  flt64_t cost = 0.0;

  Node(const Dubins2DPose& p, int32_t par, const DubinsPath& e, flt64_t c) : pose(p), parent(par), edge(e), cost(c) {}
  // Root only; safe to deref: planDubinsRrtStar rejects non-finite poses on entry.
  explicit Node(const Dubins2DPose& p) : pose(p), edge(*DubinsPath::solve(p, p, 1.0)) {}
};

static flt64_t euclidean(const Dubins2DPose& a, const Dubins2DPose& b) { return std::hypot(a.x - b.x, a.y - b.y); }

//! \brief Indices of the k nodes nearest to `pose`, Euclidean-prefiltered from 3k candidates.
static std::vector<int32_t> nearIndices(const std::vector<Node>& nodes, const Dubins2DPose& pose, int32_t k) {
  std::vector<int32_t> idx(nodes.size());
  for (std::size_t i = 0; i < nodes.size(); ++i) {
    idx[i] = static_cast<int32_t>(i);
  }
  const std::size_t keep = std::min(nodes.size(), static_cast<std::size_t>(3 * k));
  std::partial_sort(idx.begin(), idx.begin() + keep, idx.end(), [&](int32_t a, int32_t b) {
    return euclidean(nodes[a].pose, pose) < euclidean(nodes[b].pose, pose);
  });
  idx.resize(keep);
  return idx;
}

std::optional<std::vector<DubinsPath>> planDubinsRrtStar(const Dubins2DPose& start, const Dubins2DPose& goal,
                                                         const ZoneSet& zones, const DubinsRrtParams& params,
                                                         uint32_t seedSalt) {
  if (!std::isfinite(start.x) || !std::isfinite(start.y) || !std::isfinite(start.theta) || !std::isfinite(goal.x) ||
      !std::isfinite(goal.y) || !std::isfinite(goal.theta)) {
    return std::nullopt;
  }

  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::microseconds(static_cast<int64_t>(params.timeBudgetMs * 1000.0));
  std::mt19937 rng(params.seed ^ seedSalt);

  // Sampling domain: the keep-in intersection when one exists, else the start/goal AABB padded
  // generously so detours around keep-outs stay reachable.
  Vec2 lo{std::min(start.x, goal.x) - params.samplePadM, std::min(start.y, goal.y) - params.samplePadM};
  Vec2 hi{std::max(start.x, goal.x) + params.samplePadM, std::max(start.y, goal.y) + params.samplePadM};
  if (const auto keepIn = zones.keepInBounds(); keepIn.has_value()) {
    lo = keepIn->first;
    hi = keepIn->second;
    if (lo.x > hi.x || lo.y > hi.y) {
      UMAA_LOG_WARN(util::SYSTEM_LOGGER, "Dubins-RRT*: keep-in zones have empty intersection")
      return std::nullopt;
    }
  }
  std::uniform_real_distribution<flt64_t> sampleX(lo.x, hi.x);
  std::uniform_real_distribution<flt64_t> sampleY(lo.y, hi.y);
  std::uniform_real_distribution<flt64_t> sampleTheta(-M_PI, M_PI);
  std::uniform_real_distribution<flt64_t> unit(0.0, 1.0);

  const auto edgeClear = [&](const DubinsPath& path, flt64_t stepM) {
    return zones.pathClear(path, params.marginM, stepM);
  };

  std::vector<Node> nodes;
  nodes.emplace_back(start);

  // All goal connections are kept: the fine recheck may reject the cheapest.
  struct GoalLink {
    flt64_t cost;
    int32_t from;
    DubinsPath edge;
  };
  std::vector<GoalLink> goalLinks;

  const auto tryGoalConnection = [&](int32_t from) {
    const std::optional<DubinsPath> edge = DubinsPath::solve(nodes[from].pose, goal, params.rhoM);
    if (edge.has_value() && edgeClear(edge.value(), params.edgeCheckStepM)) {
      goalLinks.push_back(GoalLink{nodes[from].cost + edge->lengthM(), from, edge.value()});
    }
  };
  tryGoalConnection(0);

  for (int32_t iter = 0; iter < params.maxIterations; ++iter) {
    // Checked every iteration: near the end of a large tree one iteration costs far
    // more than the clock read, and this runs on the control thread.
    if (std::chrono::steady_clock::now() > deadline) {
      UMAA_LOG_INFO(util::SYSTEM_LOGGER, "Dubins-RRT*: time budget reached after " << iter << " iterations")
      break;
    }

    Dubins2DPose sample;
    if (unit(rng) < params.goalBias) {
      sample = goal;
    } else {
      sample = Dubins2DPose{sampleX(rng), sampleY(rng), sampleTheta(rng)};
      if (zones.clearanceM(Vec2{sample.x, sample.y}) < params.marginM) {
        continue;  // point-reject before any Dubins work
      }
    }

    const std::vector<int32_t> near = nearIndices(nodes, sample, params.nearK);

    // Choose parent: cheapest collision-free exact Dubins edge among the near set.
    int32_t bestParent = -1;
    flt64_t bestCost = std::numeric_limits<flt64_t>::max();
    std::optional<DubinsPath> bestEdge;
    int32_t exactChecked = 0;
    for (int32_t candidate : near) {
      if (exactChecked >= params.nearK) {
        break;
      }
      const std::optional<DubinsPath> edge = DubinsPath::solve(nodes[candidate].pose, sample, params.rhoM);
      if (!edge.has_value()) {
        continue;
      }
      ++exactChecked;
      const flt64_t cost = nodes[candidate].cost + edge->lengthM();
      if (cost < bestCost && edgeClear(edge.value(), params.edgeCheckStepM)) {
        bestCost = cost;
        bestParent = candidate;
        bestEdge = edge;
      }
    }
    if (bestParent < 0) {
      continue;
    }

    nodes.emplace_back(sample, bestParent, bestEdge.value(), bestCost);
    const int32_t newIndex = static_cast<int32_t>(nodes.size()) - 1;

    // Rewire: re-route near nodes through the new node when that is cheaper.
    exactChecked = 0;
    for (int32_t candidate : near) {
      if (candidate == bestParent || exactChecked >= params.nearK) {
        continue;
      }
      const std::optional<DubinsPath> edge = DubinsPath::solve(sample, nodes[candidate].pose, params.rhoM);
      if (!edge.has_value()) {
        continue;
      }
      ++exactChecked;
      const flt64_t cost = nodes[newIndex].cost + edge->lengthM();
      if (cost + 1e-9 < nodes[candidate].cost && edgeClear(edge.value(), params.edgeCheckStepM)) {
        nodes[candidate].parent = newIndex;
        nodes[candidate].edge = edge.value();
        nodes[candidate].cost = cost;
      }
    }

    tryGoalConnection(newIndex);
  }

  if (goalLinks.empty()) {
    UMAA_LOG_WARN(util::SYSTEM_LOGGER,
                  "Dubins-RRT*: no compliant path to the goal within budget (" << nodes.size() << " nodes)")
    return std::nullopt;
  }

  // Best-first over goal connections; each candidate chain is re-validated at the fine step
  // before being accepted (rewiring may have changed upstream edges since the link was made).
  std::sort(goalLinks.begin(), goalLinks.end(), [](const GoalLink& a, const GoalLink& b) { return a.cost < b.cost; });
  for (const GoalLink& link : goalLinks) {
    const flt64_t chainCost = nodes[link.from].cost + link.edge.lengthM();

    std::vector<DubinsPath> chain;
    chain.push_back(link.edge);
    bool valid = edgeClear(link.edge, params.finalCheckStepM);
    for (int32_t i = link.from; valid && i > 0; i = nodes[i].parent) {
      valid = edgeClear(nodes[i].edge, params.finalCheckStepM);
      chain.push_back(nodes[i].edge);
    }
    if (!valid) {
      continue;
    }
    std::reverse(chain.begin(), chain.end());
    UMAA_LOG_INFO(util::SYSTEM_LOGGER, "Dubins-RRT*: found compliant path of " << chain.size() << " segment(s), "
                                                                               << chainCost << " m (" << nodes.size()
                                                                               << " nodes)")
    return chain;
  }

  UMAA_LOG_WARN(util::SYSTEM_LOGGER, "Dubins-RRT*: every goal connection failed the fine recheck")
  return std::nullopt;
}

}  // namespace arlcore::autopilot
