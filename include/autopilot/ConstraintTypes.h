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

#ifndef APPS_AUTOPILOT_INCLUDE_AUTOPILOT_CONSTRAINTTYPES_H_
#define APPS_AUTOPILOT_INCLUDE_AUTOPILOT_CONSTRAINTTYPES_H_

#include <cstdint>
#include <optional>
#include <vector>

#include "NumericGuid.h"

namespace arlcore::autopilot {

//! \brief Whether a water zone must contain the vehicle (KEEP_IN, UMAA INSIDE) or exclude it
//! (KEEP_OUT, UMAA OUTSIDE).
enum class ZoneKind {
  KEEP_IN,
  KEEP_OUT
};

//! \brief A geodetic position in degrees.
struct GeoPoint {
  double latDeg = 0.0;
  double lonDeg = 0.0;
};

//! \brief Ellipse parameters as carried by a UMAA EllipseVariant (converted to a conservative
//! polygon when the zone is ingested into the ZoneMap).
struct ZoneEllipse {
  GeoPoint center;
  double semiMajorM = 0.0;
  double semiMinorM = 0.0;
  double orientationRad = 0.0;  // rotation of the semi-major axis from true north, clockwise
};

//! \brief One shape of a water zone: a geodetic polygon, or an ellipse when `ellipse` is set.
struct ZoneShape {
  std::vector<GeoPoint> polygon;
  std::optional<ZoneEllipse> ellipse;
};

//! \brief The vertical extent a zone applies to, in canonical depth (meters below the surface,
//! positive down). A missing bound is unbounded in that direction. When the UMAA ceiling/floor
//! frame cannot be converted to depth (AGL/ASF/geodetic), `convertible` is false and the zone
//! is conservatively treated as always vertically applicable.
struct ElevationBand {
  std::optional<double> ceilingDepthM;  // shallowest depth the zone covers (smaller value)
  std::optional<double> floorDepthM;    // deepest depth the zone covers (larger value)
  bool convertible = true;

  //! \brief Whether this band overlaps the depth interval [minDepthM, maxDepthM].
  bool overlaps(double minDepthM, double maxDepthM) const {
    if (!convertible) {
      return true;
    }
    if (ceilingDepthM.has_value() && maxDepthM < ceilingDepthM.value()) {
      return false;
    }
    if (floorDepthM.has_value() && minDepthM > floorDepthM.value()) {
      return false;
    }
    return true;
  }
};

//! \brief One active water-zone constraint. UMAA semantics: a KEEP_IN (INSIDE) zone requires the
//! vehicle inside EVERY shape; a KEEP_OUT (OUTSIDE) zone requires it outside every shape.
struct ZoneRecord {
  arlcore::NumericGuid conditionalId;
  ZoneKind kind = ZoneKind::KEEP_OUT;
  std::vector<ZoneShape> shapes;
  ElevationBand band;
};

//! \brief An immutable snapshot of every active constraint, rebuilt by the constraint supervisor
//! whenever the active set changes. `revision` increments on every rebuild so consumers (planner,
//! brain) can cheaply detect change.
struct ConstraintSnapshot {
  uint64_t revision = 0;
  std::vector<ZoneRecord> zones;
  std::optional<double> minSpeedMps;
  std::optional<double> maxSpeedMps;
  std::optional<double> minDepthM;  // shallowest commanded depth allowed (dynamic ceiling)
  std::optional<double> maxDepthM;  // deepest commanded depth allowed (dynamic floor)
};

//! \brief Read-side interface the supervisor exposes to the brain/planner.
class IConstraintSource {
 public:
  virtual ~IConstraintSource() = default;

  //! \brief The current constraint snapshot (copied; safe to hold across ticks).
  virtual ConstraintSnapshot snapshot() const = 0;

  //! \brief The revision of the current snapshot, for cheap change detection.
  virtual uint64_t revision() const = 0;
};

//! \brief Gate the command providers consult before accepting new commands (false while the
//! autopilot is in safe mode).
class ISafetyGate {
 public:
  virtual ~ISafetyGate() = default;
  virtual bool commandsAllowed() const = 0;
};

}  // namespace arlcore::autopilot
#endif  // APPS_AUTOPILOT_INCLUDE_AUTOPILOT_CONSTRAINTTYPES_H_
