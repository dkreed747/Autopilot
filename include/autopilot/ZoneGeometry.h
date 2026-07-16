//---------------------------------------------------------------------------
// Copyright 2026 Pennsylvania State University
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

#ifndef APPS_AUTOPILOT_INCLUDE_AUTOPILOT_ZONEGEOMETRY_H_
#define APPS_AUTOPILOT_INCLUDE_AUTOPILOT_ZONEGEOMETRY_H_

#include <optional>
#include <utility>
#include <vector>

#include "ConstraintTypes.h"
#include "DubinsPath.h"

namespace arlcore::autopilot {

//! \brief A 2D point/vector in a local tangent plane: x = east meters, y = north meters.
struct Vec2 {
  double x = 0.0;
  double y = 0.0;
};

//! \brief A simple polygon in the local plane, normalized to counter-clockwise winding with a
//! precomputed axis-aligned bounding box. Pure math: no UMAA or geographic dependencies.
class LocalPolygon {
 public:
  //! \brief Construct from at least 3 vertices (an open ring: the closing edge is implicit).
  //! Vertices are re-wound counter-clockwise if given clockwise.
  explicit LocalPolygon(std::vector<Vec2> vertices);

  //! \brief Whether `p` is inside the polygon (boundary counts as inside).
  bool contains(const Vec2& p) const;

  //! \brief Signed distance from `p` to the polygon: positive inside, negative outside, zero on
  //! the boundary. Magnitude is the Euclidean distance to the nearest boundary point.
  double signedDistance(const Vec2& p) const;

  //! \brief The boundary point nearest to `p`.
  Vec2 closestBoundaryPoint(const Vec2& p) const;

  const std::vector<Vec2>& vertices() const { return vertices_; }
  const Vec2& aabbMin() const { return aabbMin_; }
  const Vec2& aabbMax() const { return aabbMax_; }

 private:
  std::vector<Vec2> vertices_;
  Vec2 aabbMin_;
  Vec2 aabbMax_;
};

//! \brief Clearance at a point plus the direction that increases it fastest.
struct ClearanceInfo {
  double clearanceM = 0.0;
  Vec2 improveDir{1.0, 0.0};  // unit vector toward increasing clearance
};

//! \brief The active zones projected into one local frame, exposed as signed-clearance queries.
//!
//! Compliance convention: `clearanceM(p) > 0` means `p` satisfies every zone, and the value is
//! the distance to the nearest violating boundary. Margins stay implicit: callers compare the
//! clearance against their own margin instead of ever offsetting polygons, which sidesteps the
//! robustness problems of non-convex polygon offsetting entirely. The clearance function is the
//! min of per-zone signed distances, hence 1-Lipschitz — which `raycastFirstHit` exploits for
//! sphere-tracing marches.
class ZoneSet {
 public:
  void addZone(ZoneKind kind, LocalPolygon polygon);

  bool empty() const { return zones_.empty(); }
  std::size_t size() const { return zones_.size(); }

  //! \brief Signed compliance clearance at `p` (min over all zones). Positive = compliant.
  double clearanceM(const Vec2& p) const;

  //! \brief Clearance at `p` plus the unit direction that increases the binding zone's clearance.
  ClearanceInfo clearanceInfo(const Vec2& p) const;

  //! \brief Whether `p` keeps at least `marginM` clearance from every zone boundary.
  bool pointCompliant(const Vec2& p, double marginM) const { return clearanceM(p) >= marginM; }

  //! \brief Whether every sample of segment a->b (step `stepM`, endpoints included) keeps
  //! `marginM` clearance.
  bool segmentClear(const Vec2& a, const Vec2& b, double marginM, double stepM) const;

  //! \brief Whether every sample of a Dubins path (planned in this frame, offset by `origin`)
  //! keeps `marginM` clearance. Sampling at step delta with clearance >= m at every sample
  //! guarantees true clearance >= m - delta/2 - delta^2/(8*rho) on curvature-rho arcs; callers
  //! pick stepM (and margin) accordingly.
  bool pathClear(const DubinsPath& path, double marginM, double stepM) const;

  //! \brief Distance along the ray from `origin` in direction `dir` (unit vector) at which the
  //! clearance first drops below `marginM`, or nullopt if the ray stays clear out to `maxRangeM`.
  //! If the origin itself is below margin, returns 0.
  std::optional<double> raycastFirstHit(const Vec2& origin, const Vec2& dir, double marginM,
                                        double maxRangeM) const;

  //! \brief The nearest point to `p` with clearance >= `marginM`: iterative projection along the
  //! clearance gradient, with an expanding ring search as fallback for multi-zone corners.
  //! Returns nullopt when no compliant point is found within the search budget.
  std::optional<Vec2> nearestCompliantPoint(const Vec2& p, double marginM) const;

  //! \brief The intersection AABB of all keep-in zones (a natural sampling domain), or nullopt
  //! when there is no keep-in zone. An empty (inverted) box means the keep-ins are disjoint.
  std::optional<std::pair<Vec2, Vec2>> keepInBounds() const;

 private:
  struct Zone {
    ZoneKind kind;
    LocalPolygon polygon;
  };

  //! \brief Signed compliance clearance contributed by one zone.
  static double zoneClearance(const Zone& z, const Vec2& p);

  std::vector<Zone> zones_;
};

}  // namespace arlcore::autopilot
#endif  // APPS_AUTOPILOT_INCLUDE_AUTOPILOT_ZONEGEOMETRY_H_
