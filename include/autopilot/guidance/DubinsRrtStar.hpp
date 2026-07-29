#ifndef AUTOPILOT_GUIDANCE_DUBINSRRTSTAR_HPP_
#define AUTOPILOT_GUIDANCE_DUBINSRRTSTAR_HPP_

#include <cstdint>
#include <optional>
#include <vector>

#include "autopilot/guidance/DubinsPath.hpp"
#include "autopilot/safety/ZoneGeometry.hpp"
#include "InternalTypes.h"

namespace arlcore::autopilot {

//! \brief Tuning for the Dubins-RRT* fallback planner.
struct DubinsRrtParams {
  flt64_t rhoM = 25.0;           // Dubins turn radius (same as the tracker's)
  flt64_t marginM = 5.0;         // required clearance from every zone boundary
  uint32_t seed = 12345;        // base RNG seed; XORed with the caller's salt
  int32_t maxIterations = 2000;
  flt64_t timeBudgetMs = 150.0;  // wall-clock cutoff (the iteration cap usually binds first)
  flt64_t goalBias = 0.10;
  int32_t nearK = 8;                // exact-Dubins neighbor count (prefiltered 3x by Euclidean)
  flt64_t edgeCheckStepM = 4.0;  // coarse in-tree edge sampling
  flt64_t finalCheckStepM = 1.0;  // fine recheck of the accepted solution
  flt64_t samplePadM = 200.0;    // pad around start/goal AABB when no keep-in bounds the domain
};

//! \brief Plan a zone-compliant, curvature-bounded path from `start` to `goal` (poses in the
//! caller's local frame, math convention) as a chain of exact Dubins paths; returns nullopt
//! when no compliant path is found within the iteration/time budget.
//! Deterministic for a fixed (seed ^ seedSalt).
std::optional<std::vector<DubinsPath>> planDubinsRrtStar(const Dubins2DPose& start,
                                                         const Dubins2DPose& goal,
                                                         const ZoneSet& zones,
                                                         const DubinsRrtParams& params,
                                                         uint32_t seedSalt);

}  // namespace arlcore::autopilot
#endif  // AUTOPILOT_GUIDANCE_DUBINSRRTSTAR_HPP_
