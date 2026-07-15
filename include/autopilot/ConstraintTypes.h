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

//! \brief One vertical bound of a zone, in the frame it was commanded in. DEPTH is meters
//! below the surface (positive down); ASF is meters above the sea floor (positive up).
//! Mixed-frame bands are first-class: e.g. ceiling at depth 0 with a floor 5 m above the
//! sea floor covers the whole water column except a near-bottom corridor.
struct ElevationBound {
  enum class Frame { DEPTH, ASF };
  Frame frame = Frame::DEPTH;
  double value = 0.0;
};

//! \brief The vertical extent a query applies to: a depth interval plus, when known, an
//! above-sea-floor interval (from the vehicle's altitudeASF). An absent ASF interval is
//! conservative: ASF-framed zone bounds cannot exonerate the zone without it.
struct ElevationEnvelope {
  double minDepthM = 0.0;
  double maxDepthM = 0.0;
  std::optional<double> minAsfM;
  std::optional<double> maxAsfM;

  static ElevationEnvelope atPoint(double depthM, std::optional<double> asfM = std::nullopt) {
    ElevationEnvelope env;
    env.minDepthM = depthM;
    env.maxDepthM = depthM;
    env.minAsfM = asfM;
    env.maxAsfM = asfM;
    return env;
  }
};

//! \brief The vertical extent a zone applies to. The ceiling is the shallow cutoff, the floor
//! the deep cutoff; each carries its own frame. A missing bound is unbounded in that
//! direction. When a UMAA ceiling/floor frame has no evaluable equivalent (AGL/geodetic),
//! `convertible` is false and the zone is conservatively treated as always applicable.
struct ElevationBand {
  std::optional<ElevationBound> ceiling;  // shallow cutoff
  std::optional<ElevationBound> floor;    // deep cutoff
  bool convertible = true;

  //! \brief Whether this band overlaps the envelope. Unknown envelope components never
  //! exonerate a bound (conservative).
  bool overlaps(const ElevationEnvelope& env) const {
    if (!convertible) {
      return true;
    }
    if (ceiling.has_value()) {
      // Everything shallower than the ceiling is outside the zone. Shallower means a smaller
      // depth, or a larger altitude above the sea floor.
      if (ceiling->frame == ElevationBound::Frame::DEPTH) {
        if (env.maxDepthM < ceiling->value) {
          return false;
        }
      } else if (env.minAsfM.has_value() && env.minAsfM.value() > ceiling->value) {
        return false;
      }
    }
    if (floor.has_value()) {
      // Everything deeper than the floor is outside the zone.
      if (floor->frame == ElevationBound::Frame::DEPTH) {
        if (env.minDepthM > floor->value) {
          return false;
        }
      } else if (env.maxAsfM.has_value() && env.maxAsfM.value() < floor->value) {
        return false;
      }
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
