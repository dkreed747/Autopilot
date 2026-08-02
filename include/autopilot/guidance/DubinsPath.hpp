#ifndef AUTOPILOT_GUIDANCE_DUBINSPATH_HPP_
#define AUTOPILOT_GUIDANCE_DUBINSPATH_HPP_

#include <array>
#include <optional>
#include <string>

#include "InternalTypes.h"

namespace arlcore::autopilot {

//! \brief A pose in a local 2D Cartesian plane using the math convention:
//! x/y in meters, theta in radians measured counterclockwise from the +x axis.
struct Dubins2DPose {
  flt64_t x = 0.0;
  flt64_t y = 0.0;
  flt64_t theta = 0.0;
};

//! \brief One of the three segments of a Dubins path.
struct DubinsSegment {
  enum class Type { LEFT, STRAIGHT, RIGHT };
  Type type = Type::STRAIGHT;
  flt64_t lengthM = 0.0;  // arc length of this segment in meters
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
  static std::optional<DubinsPath> solve(const Dubins2DPose& start, const Dubins2DPose& goal, flt64_t rhoM);

  //! \brief Total path length in meters.
  flt64_t lengthM() const { return lengths_[0] + lengths_[1] + lengths_[2]; }

  //! \brief The pose at arc length `sM` along the path (clamped to [0, lengthM()]).
  Dubins2DPose sample(flt64_t sM) const;

  //! \brief The turn radius the path was solved with; 0 for a degenerate straight run that has
  //! no turning circle.
  flt64_t rhoM() const { return rho_; }

  //! \brief Signed curvature at arc length `sM` (clamped like sample()): +1/rho on a LEFT
  //! segment, -1/rho on a RIGHT one, 0 on a STRAIGHT one. Math convention, so positive is
  //! counterclockwise in theta and therefore a decreasing azimuth. Discontinuous at segment
  //! boundaries by construction; a boundary belongs to the preceding segment, matching sample().
  flt64_t curvatureAt(flt64_t sM) const;

  //! \brief The three segments of the path (a degenerate word may contain zero-length segments).
  std::array<DubinsSegment, 3> segments() const;

  //! \brief The word name, e.g. "LSL" (useful for diagnostics/tests).
  std::string word() const;

 private:
  DubinsPath() = default;

  Dubins2DPose start_;
  flt64_t rho_ = 1.0;
  std::array<DubinsSegment::Type, 3> types_ = {DubinsSegment::Type::LEFT, DubinsSegment::Type::STRAIGHT,
                                               DubinsSegment::Type::LEFT};
  std::array<flt64_t, 3> lengths_ = {0.0, 0.0, 0.0};  // segment lengths in meters
};

}  // namespace arlcore::autopilot
#endif  // AUTOPILOT_GUIDANCE_DUBINSPATH_HPP_
