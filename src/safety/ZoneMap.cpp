#include "autopilot/safety/ZoneMap.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

#include "InternalTypes.h"
#include "Logger.h"

namespace arlcore::autopilot {

ZoneMap::ZoneMap(const ZonesConfig& config) : config_(config) {}

// TODO(@user): the anchor pins to the first ingested vertex forever; ellipse zones
// materialized far from it distort slightly. Revisit if operating areas span >10 km.
void ZoneMap::ensureAnchor(const ConstraintSnapshot& snapshot) {
  if (anchor_.has_value()) {
    return;
  }
  for (const ZoneRecord& zone : snapshot.zones) {
    for (const ZoneShape& shape : zone.shapes) {
      if (shape.ellipse.has_value()) {
        anchor_.emplace(shape.ellipse->center.latDeg, shape.ellipse->center.lonDeg, 0.0);
        return;
      }
      if (!shape.polygon.empty()) {
        anchor_.emplace(shape.polygon.front().latDeg, shape.polygon.front().lonDeg, 0.0);
        return;
      }
    }
  }
}

std::vector<GeoPoint> ZoneMap::ellipseToRing(const ZoneEllipse& ellipse, ZoneKind kind) const {
  const int32_t n = std::max(config_.ellipseSegments, 8);
  // Circumscribing a keep-out grows the forbidden region and inscribing a keep-in shrinks the
  // allowed one — both conservative.
  const flt64_t scale = kind == ZoneKind::KEEP_OUT ? 1.0 / std::cos(M_PI / n) : 1.0;
  const flt64_t a = ellipse.semiMajorM * scale;
  const flt64_t b = ellipse.semiMinorM * scale;
  // Semi-major axis orientation is a bearing (clockwise from true north) in an east/north plane.
  const Vec2 uMajor{std::sin(ellipse.orientationRad), std::cos(ellipse.orientationRad)};
  const Vec2 uMinor{std::cos(ellipse.orientationRad), -std::sin(ellipse.orientationRad)};

  flt64_t cx = 0.0;
  flt64_t cy = 0.0;
  flt64_t cz = 0.0;
  anchor_->Forward(ellipse.center.latDeg, ellipse.center.lonDeg, 0.0, cx, cy, cz);

  std::vector<GeoPoint> ring;
  ring.reserve(n);
  for (int32_t k = 0; k < n; ++k) {
    const flt64_t t = 2.0 * M_PI * k / n;
    const flt64_t ex = cx + a * std::cos(t) * uMajor.x + b * std::sin(t) * uMinor.x;
    const flt64_t ny = cy + a * std::cos(t) * uMajor.y + b * std::sin(t) * uMinor.y;
    GeoPoint p;
    flt64_t h = 0.0;
    anchor_->Reverse(ex, ny, 0.0, p.latDeg, p.lonDeg, h);
    ring.push_back(p);
  }
  return ring;
}

void ZoneMap::ingest(const ConstraintSnapshot& snapshot) {
  ensureAnchor(snapshot);
  zones_.clear();
  for (const ZoneRecord& zone : snapshot.zones) {
    StoredZone stored;
    stored.kind = zone.kind;
    stored.band = zone.band;
    for (const ZoneShape& shape : zone.shapes) {
      if (shape.ellipse.has_value()) {
        stored.rings.push_back(ellipseToRing(shape.ellipse.value(), zone.kind));
      } else if (shape.polygon.size() >= 3) {
        stored.rings.push_back(shape.polygon);
      } else {
        UMAA_LOG_WARN(util::SYSTEM_LOGGER, "Ignoring water-zone shape with fewer than 3 vertices")
      }
    }
    if (!stored.rings.empty()) {
      zones_.push_back(std::move(stored));
    }
  }
  revision_ = snapshot.revision;
}

ZoneSet ZoneMap::activeSet(const GeographicLib::LocalCartesian& frame, const ElevationEnvelope& envelope) const {
  ZoneSet set;
  ElevationEnvelope padded = envelope;
  padded.minDepthM -= config_.elevationMarginM;
  padded.maxDepthM += config_.elevationMarginM;
  if (padded.minAsfM.has_value()) {
    padded.minAsfM = padded.minAsfM.value() - config_.elevationMarginM;
  }
  if (padded.maxAsfM.has_value()) {
    padded.maxAsfM = padded.maxAsfM.value() + config_.elevationMarginM;
  }
  for (const StoredZone& zone : zones_) {
    if (!zone.band.overlaps(padded)) {
      continue;
    }
    for (const std::vector<GeoPoint>& ring : zone.rings) {
      std::vector<Vec2> vertices;
      vertices.reserve(ring.size());
      for (const GeoPoint& gp : ring) {
        flt64_t x = 0.0;
        flt64_t y = 0.0;
        flt64_t z = 0.0;
        frame.Forward(gp.latDeg, gp.lonDeg, 0.0, x, y, z);
        vertices.push_back(Vec2{x, y});
      }
      set.addZone(zone.kind, LocalPolygon(std::move(vertices)));
    }
  }
  return set;
}

flt64_t ZoneMap::clearanceM(const GeoPoint& position, const ElevationEnvelope& envelope) const {
  if (!anchor_.has_value() || zones_.empty()) {
    return std::numeric_limits<flt64_t>::max();
  }
  const ZoneSet set = activeSet(anchor_.value(), envelope);
  if (set.empty()) {
    return std::numeric_limits<flt64_t>::max();
  }
  flt64_t x = 0.0;
  flt64_t y = 0.0;
  flt64_t z = 0.0;
  anchor_->Forward(position.latDeg, position.lonDeg, 0.0, x, y, z);
  return set.clearanceM(Vec2{x, y});
}

flt64_t ZoneMap::clearanceM(const GeoPoint& position, flt64_t depthM, std::optional<flt64_t> asfM) const {
  return clearanceM(position, ElevationEnvelope::atPoint(depthM, asfM));
}

ZoneCompliance ZoneMap::classify(const GeoPoint& position, flt64_t depthM, std::optional<flt64_t> asfM) const {
  const flt64_t clearance = clearanceM(position, depthM, asfM);
  if (clearance < 0.0) {
    return ZoneCompliance::VIOLATION;
  }
  if (clearance < config_.complianceHysteresisM) {
    return ZoneCompliance::MARGINAL;
  }
  return ZoneCompliance::COMPLIANT;
}

bool ZoneMap::pointCompliant(const GeoPoint& position, flt64_t depthM, flt64_t marginM,
                             std::optional<flt64_t> asfM) const {
  return clearanceM(position, depthM, asfM) >= marginM;
}

}  // namespace arlcore::autopilot
