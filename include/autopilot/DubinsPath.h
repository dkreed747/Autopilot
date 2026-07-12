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

#ifndef APPS_AUTOPILOT_INCLUDE_AUTOPILOT_DUBINSPATH_H_
#define APPS_AUTOPILOT_INCLUDE_AUTOPILOT_DUBINSPATH_H_

#include <array>
#include <optional>
#include <string>

namespace arlcore::autopilot {

//! \brief A pose in a local 2D Cartesian plane using the math convention:
//! x/y in meters, theta in radians measured counterclockwise from the +x axis.
struct Dubins2DPose {
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
};

//! \brief One of the three segments of a Dubins path.
struct DubinsSegment {
  enum class Type { LEFT, STRAIGHT, RIGHT };
  Type type = Type::STRAIGHT;
  double lengthM = 0.0;  // arc length of this segment in meters
};

//! \brief A complete Dubins path: the shortest curvature-bounded (radius rho) path between
//! two poses in the plane, selected from the six canonical words LSL, LSR, RSL, RSR, RLR,
//! LRL (Shkel & Lumelsky closed forms). All six words are evaluated and the shortest valid
//! one is chosen, so a path is produced for every reachable configuration (which is all of
//! them for rho > 0).
class DubinsPath {
 public:
  //! \brief Solve the shortest Dubins path from `start` to `goal` with turn radius `rhoM`.
  //! Degenerate cases are handled: a non-positive/near-zero radius produces a straight
  //! segment toward the goal position, and coincident poses produce a zero-length path.
  //! Returns std::nullopt only for non-finite inputs.
  static std::optional<DubinsPath> solve(const Dubins2DPose& start, const Dubins2DPose& goal, double rhoM);

  //! \brief Total path length in meters.
  double lengthM() const { return lengths_[0] + lengths_[1] + lengths_[2]; }

  //! \brief The pose at arc length `sM` along the path (clamped to [0, lengthM()]).
  Dubins2DPose sample(double sM) const;

  //! \brief The three segments of the path (a degenerate word may contain zero-length segments).
  std::array<DubinsSegment, 3> segments() const;

  //! \brief The word name, e.g. "LSL" (useful for diagnostics/tests).
  std::string word() const;

 private:
  DubinsPath() = default;

  Dubins2DPose start_;
  double rho_ = 1.0;
  std::array<DubinsSegment::Type, 3> types_ = {DubinsSegment::Type::LEFT, DubinsSegment::Type::STRAIGHT,
                                               DubinsSegment::Type::LEFT};
  std::array<double, 3> lengths_ = {0.0, 0.0, 0.0};  // segment lengths in meters
};

}  // namespace arlcore::autopilot
#endif  // APPS_AUTOPILOT_INCLUDE_AUTOPILOT_DUBINSPATH_H_
