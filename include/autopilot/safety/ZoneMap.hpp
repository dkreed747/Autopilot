#ifndef AUTOPILOT_SAFETY_ZONEMAP_HPP_
#define AUTOPILOT_SAFETY_ZONEMAP_HPP_

#include <optional>
#include <vector>

#include <GeographicLib/LocalCartesian.hpp>

#include "autopilot/config/AutopilotConfig.hpp"
#include "autopilot/safety/ConstraintTypes.hpp"
#include "autopilot/safety/ZoneGeometry.hpp"
#include "InternalTypes.h"

namespace arlcore::autopilot {

//! \brief Compliance classification of a position against the active zones.
enum class ZoneCompliance {
  COMPLIANT,  // clearance >= compliance hysteresis
  MARGINAL,   // satisfied, but within the hysteresis band of a boundary
  VIOLATION   // inside a keep-out / outside a keep-in
};

//! \brief Geodetic store of the active water zones, shared by the supervisor (violation truth),
//! the planners (obstacle queries), and command validation.
//! Zones are stored geodetically and projected on demand into any caller frame, so all views
//! agree; ellipse shapes are converted to conservative polygons at ingest (circumscribed for
//! keep-out, inscribed for keep-in).
class ZoneMap {
 public:
  explicit ZoneMap(const ZonesConfig& config);

  //! \brief Replace the zone store with the zones of a new constraint snapshot.
  void ingest(const ConstraintSnapshot& snapshot);

  //! \brief Whether any zone is currently active (at any depth).
  bool hasZones() const { return !zones_.empty(); }

  //! \brief Revision of the snapshot last ingested.
  uint64_t revision() const { return revision_; }

  //! \brief Build the zone set for a caller-owned frame, keeping only zones whose elevation band
  //! overlaps `envelope` (padded by the configured elevation margin).
  ZoneSet activeSet(const GeographicLib::LocalCartesian& frame, const ElevationEnvelope& envelope) const;

  //! \brief Classify a geodetic position at vehicle depth `depthM` (and, when known, altitude
  //! above the sea floor `asfM` — required to gate ASF-framed zone bands) against the zones.
  ZoneCompliance classify(const GeoPoint& position, flt64_t depthM,
                          std::optional<flt64_t> asfM = std::nullopt) const;

  //! \brief Compliance clearance (meters, positive = compliant) of a position.
  flt64_t clearanceM(const GeoPoint& position, flt64_t depthM,
                    std::optional<flt64_t> asfM = std::nullopt) const;

  //! \brief Whether a commanded point keeps `marginM` clearance (command-time validation).
  bool pointCompliant(const GeoPoint& position, flt64_t depthM, flt64_t marginM,
                      std::optional<flt64_t> asfM = std::nullopt) const;

  //! \brief Clearance of a position over an explicit vertical envelope (for points whose
  //! depth is not known exactly, e.g. waypoints commanded in the ASF frame).
  flt64_t clearanceM(const GeoPoint& position, const ElevationEnvelope& envelope) const;

  //! \brief The map's own anchor frame (unset until the first zone is ingested). Callers that
  //! need raw ZoneSet queries at the vehicle (vector avoidance, recovery) project through it.
  const std::optional<GeographicLib::LocalCartesian>& anchor() const { return anchor_; }

  const ZonesConfig& config() const { return config_; }

 private:
  struct StoredZone {
    ZoneKind kind = ZoneKind::KEEP_OUT;
    std::vector<std::vector<GeoPoint>> rings;  // one geodetic ring per shape
    ElevationBand band;
  };

  //! \brief Set the anchor from the first shape seen, if not anchored yet.
  void ensureAnchor(const ConstraintSnapshot& snapshot);

  //! \brief Convert an ellipse to a conservative geodetic polygon in the anchor frame.
  std::vector<GeoPoint> ellipseToRing(const ZoneEllipse& ellipse, ZoneKind kind) const;

  ZonesConfig config_;
  std::optional<GeographicLib::LocalCartesian> anchor_;
  std::vector<StoredZone> zones_;
  uint64_t revision_ = 0;
};

}  // namespace arlcore::autopilot
#endif  // AUTOPILOT_SAFETY_ZONEMAP_HPP_
