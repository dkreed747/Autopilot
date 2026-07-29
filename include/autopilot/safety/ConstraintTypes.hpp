#ifndef AUTOPILOT_SAFETY_CONSTRAINTTYPES_HPP_
#define AUTOPILOT_SAFETY_CONSTRAINTTYPES_HPP_

#include <cstdint>
#include <optional>
#include <vector>

#include "InternalTypes.h"
#include "NumericGuid.h"

namespace arlcore::autopilot {

//! \brief Whether a water zone must contain the vehicle (KEEP_IN, UMAA INSIDE) or exclude it
//! (KEEP_OUT, UMAA OUTSIDE).
enum class ZoneKind { KEEP_IN, KEEP_OUT };

//! \brief A geodetic position in degrees.
struct GeoPoint {
  flt64_t latDeg = 0.0;
  flt64_t lonDeg = 0.0;
};

//! \brief Ellipse parameters as carried by a UMAA EllipseVariant (converted to a conservative
//! polygon when the zone is ingested into the ZoneMap).
struct ZoneEllipse {
  GeoPoint center;
  flt64_t semiMajorM = 0.0;
  flt64_t semiMinorM = 0.0;
  flt64_t orientationRad = 0.0;  // rotation of the semi-major axis from true north, clockwise
};

//! \brief One shape of a water zone: a geodetic polygon, or an ellipse when `ellipse` is set.
struct ZoneShape {
  std::vector<GeoPoint> polygon;
  std::optional<ZoneEllipse> ellipse;
};

//! \brief One vertical bound of a zone, in the frame it was commanded in: DEPTH is meters
//! below the surface (positive down); ASF is meters above the sea floor (positive up).
struct ElevationBound {
  enum class Frame { DEPTH, ASF };
  Frame frame = Frame::DEPTH;
  flt64_t value = 0.0;
};

//! \brief The vertical extent a query applies to: a depth interval plus, when known, an
//! above-sea-floor interval (from the vehicle's altitudeASF). An absent ASF interval is
//! conservative: ASF-framed zone bounds cannot exonerate the zone without it.
struct ElevationEnvelope {
  flt64_t minDepthM = 0.0;
  flt64_t maxDepthM = 0.0;
  std::optional<flt64_t> minAsfM;
  std::optional<flt64_t> maxAsfM;

  static ElevationEnvelope atPoint(flt64_t depthM, std::optional<flt64_t> asfM = std::nullopt) {
    ElevationEnvelope env;
    env.minDepthM = depthM;
    env.maxDepthM = depthM;
    env.minAsfM = asfM;
    env.maxAsfM = asfM;
    return env;
  }
};

//! \brief The vertical extent a zone applies to; a missing bound is unbounded in that
//! direction, and a non-convertible UMAA frame (AGL/geodetic) conservatively makes the zone
//! always applicable.
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
      // Shallower than the ceiling is outside the zone: smaller depth, or larger altitude
      // above the sea floor.
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
  std::optional<flt64_t> minSpeedMps;
  std::optional<flt64_t> maxSpeedMps;
  std::optional<flt64_t> minDepthM;  // shallowest commanded depth allowed (dynamic ceiling)
  std::optional<flt64_t> maxDepthM;  // deepest commanded depth allowed (dynamic floor)
};

}  // namespace arlcore::autopilot
#endif  // AUTOPILOT_SAFETY_CONSTRAINTTYPES_HPP_
