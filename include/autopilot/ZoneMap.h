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

#ifndef APPS_AUTOPILOT_INCLUDE_AUTOPILOT_ZONEMAP_H_
#define APPS_AUTOPILOT_INCLUDE_AUTOPILOT_ZONEMAP_H_

#include <optional>
#include <vector>

#include <GeographicLib/LocalCartesian.hpp>

#include "AutopilotConfig.h"
#include "ConstraintTypes.h"
#include "ZoneGeometry.h"

namespace arlcore::autopilot {

//! \brief The depth interval (positive down, meters) a query applies to; zones whose elevation
//! band does not overlap it are not obstacles for that query.
struct ElevationEnvelope {
  double minDepthM = 0.0;
  double maxDepthM = 0.0;
};

//! \brief Compliance classification of a position against the active zones.
enum class ZoneCompliance {
  COMPLIANT,  // clearance >= compliance hysteresis
  MARGINAL,   // satisfied, but within the hysteresis band of a boundary
  VIOLATION   // inside a keep-out / outside a keep-in
};

//! \brief Geodetic store of the active water zones, shared by the supervisor (violation truth),
//! the planners (obstacle queries), and command validation.
//!
//! Zones are stored geodetically and projected on demand: the map owns one LocalCartesian anchor
//! fixed at the first ingest (zones are global and long-lived), while planners keep their own
//! per-plan frames and ask for `activeSet(plannerFrame, envelope)` — projecting the same geodetic
//! vertices into either frame is exact, so both views agree. Ellipse shapes are converted to
//! conservative polygons at ingest (circumscribed for keep-out, inscribed for keep-in).
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

  //! \brief Classify a geodetic position at vehicle depth `depthM` against the active zones.
  ZoneCompliance classify(const GeoPoint& position, double depthM) const;

  //! \brief Compliance clearance (meters, positive = compliant) of a position at depth `depthM`.
  double clearanceM(const GeoPoint& position, double depthM) const;

  //! \brief Whether a commanded point keeps `marginM` clearance (command-time validation).
  bool pointCompliant(const GeoPoint& position, double depthM, double marginM) const;

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
#endif  // APPS_AUTOPILOT_INCLUDE_AUTOPILOT_ZONEMAP_H_
