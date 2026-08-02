#include "autopilot/safety/ZoneGeometry.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <utility>

#include "InternalTypes.h"

namespace arlcore::autopilot {

constexpr flt64_t EPS = 1e-9;

static flt64_t dot(const Vec2& a, const Vec2& b) { return a.x * b.x + a.y * b.y; }
static flt64_t norm(const Vec2& a) { return std::sqrt(dot(a, a)); }
static Vec2 sub(const Vec2& a, const Vec2& b) { return {a.x - b.x, a.y - b.y}; }
static Vec2 add(const Vec2& a, const Vec2& b) { return {a.x + b.x, a.y + b.y}; }
static Vec2 scale(const Vec2& a, flt64_t s) { return {a.x * s, a.y * s}; }

//! \brief Nearest point to `p` on segment a->b.
static Vec2 closestOnSegment(const Vec2& p, const Vec2& a, const Vec2& b) {
  const Vec2 ab = sub(b, a);
  const flt64_t len2 = dot(ab, ab);
  if (len2 < EPS) {
    return a;
  }
  const flt64_t t = std::clamp(dot(sub(p, a), ab) / len2, 0.0, 1.0);
  return add(a, scale(ab, t));
}

//! \brief Twice the signed area of the polygon (positive when counter-clockwise).
static flt64_t signedArea2(const std::vector<Vec2>& v) {
  flt64_t area2 = 0.0;
  for (std::size_t i = 0, j = v.size() - 1; i < v.size(); j = i++) {
    area2 += (v[j].x * v[i].y - v[i].x * v[j].y);
  }
  return area2;
}

LocalPolygon::LocalPolygon(std::vector<Vec2> vertices) : vertices_(std::move(vertices)) {
  if (vertices_.size() < 3) {
    throw std::invalid_argument("LocalPolygon needs at least 3 vertices");
  }
  if (signedArea2(vertices_) < 0.0) {
    std::reverse(vertices_.begin(), vertices_.end());
  }
  aabbMin_ = aabbMax_ = vertices_.front();
  for (const Vec2& v : vertices_) {
    aabbMin_.x = std::min(aabbMin_.x, v.x);
    aabbMin_.y = std::min(aabbMin_.y, v.y);
    aabbMax_.x = std::max(aabbMax_.x, v.x);
    aabbMax_.y = std::max(aabbMax_.y, v.y);
  }
}

bool LocalPolygon::contains(const Vec2& p) const {
  if (p.x < aabbMin_.x || p.x > aabbMax_.x || p.y < aabbMin_.y || p.y > aabbMax_.y) {
    return false;
  }
  // Crossing-number test with an explicit boundary check so "on the edge" counts as inside.
  bool inside = false;
  for (std::size_t i = 0, j = vertices_.size() - 1; i < vertices_.size(); j = i++) {
    const Vec2& a = vertices_[j];
    const Vec2& b = vertices_[i];
    const Vec2 c = closestOnSegment(p, a, b);
    if (norm(sub(p, c)) < EPS) {
      return true;
    }
    const bool crosses = (b.y > p.y) != (a.y > p.y);
    if (crosses) {
      const flt64_t xCross = b.x + (p.y - b.y) * (a.x - b.x) / (a.y - b.y);
      if (p.x < xCross) {
        inside = !inside;
      }
    }
  }
  return inside;
}

Vec2 LocalPolygon::closestBoundaryPoint(const Vec2& p) const {
  Vec2 best = vertices_.front();
  flt64_t bestDist = std::numeric_limits<flt64_t>::max();
  for (std::size_t i = 0, j = vertices_.size() - 1; i < vertices_.size(); j = i++) {
    const Vec2 c = closestOnSegment(p, vertices_[j], vertices_[i]);
    const flt64_t d = norm(sub(p, c));
    if (d < bestDist) {
      bestDist = d;
      best = c;
    }
  }
  return best;
}

flt64_t LocalPolygon::signedDistance(const Vec2& p) const {
  const flt64_t d = norm(sub(p, closestBoundaryPoint(p)));
  return contains(p) ? d : -d;
}

void ZoneSet::addZone(ZoneKind kind, LocalPolygon polygon) { zones_.push_back(Zone{kind, std::move(polygon)}); }

flt64_t ZoneSet::zoneClearance(const Zone& z, const Vec2& p) {
  const flt64_t sd = z.polygon.signedDistance(p);
  // KEEP_OUT is compliant outside the polygon, KEEP_IN inside it.
  return z.kind == ZoneKind::KEEP_OUT ? -sd : sd;
}

flt64_t ZoneSet::clearanceM(const Vec2& p) const {
  flt64_t clearance = std::numeric_limits<flt64_t>::max();
  for (const Zone& z : zones_) {
    const flt64_t c = zoneClearance(z, p);
    // std::min(a, NaN) returns a, so a non-finite clearance would be silently dropped from the
    // minimum and the zone would stop constraining anything. Geometry we cannot evaluate has to
    // read as maximally violating, not as absent.
    if (!std::isfinite(c)) {
      return -std::numeric_limits<flt64_t>::max();
    }
    clearance = std::min(clearance, c);
  }
  return clearance;
}

ClearanceInfo ZoneSet::clearanceInfo(const Vec2& p) const {
  ClearanceInfo info;
  info.clearanceM = std::numeric_limits<flt64_t>::max();
  const Zone* binding = nullptr;
  for (const Zone& z : zones_) {
    const flt64_t c = zoneClearance(z, p);
    if (!std::isfinite(c)) {
      // Same reasoning as clearanceM: an unevaluable zone must bind, not disappear.
      info.clearanceM = -std::numeric_limits<flt64_t>::max();
      binding = &z;
      break;
    }
    if (c < info.clearanceM) {
      info.clearanceM = c;
      binding = &z;
    }
  }
  if (binding == nullptr) {
    return info;
  }
  const Vec2 boundary = binding->polygon.closestBoundaryPoint(p);
  Vec2 dir = sub(p, boundary);
  const flt64_t len = norm(dir);
  if (len < EPS) {
    // On the boundary: fall back to probing which side improves the clearance.
    const flt64_t probe = 0.5;
    Vec2 bestDir{1.0, 0.0};
    flt64_t bestClearance = -std::numeric_limits<flt64_t>::max();
    for (int32_t k = 0; k < 8; ++k) {
      const flt64_t a = 2.0 * M_PI * k / 8.0;
      const Vec2 d{std::cos(a), std::sin(a)};
      const flt64_t c = clearanceM(add(p, scale(d, probe)));
      if (c > bestClearance) {
        bestClearance = c;
        bestDir = d;
      }
    }
    info.improveDir = bestDir;
    return info;
  }
  dir = scale(dir, 1.0 / len);
  // `dir` points from the boundary toward p; on the violating side improving means heading
  // back through the boundary, i.e. -dir.
  info.improveDir = info.clearanceM >= 0.0 ? dir : scale(dir, -1.0);
  return info;
}

bool ZoneSet::segmentClear(const Vec2& a, const Vec2& b, flt64_t marginM, flt64_t stepM) const {
  if (zones_.empty()) {
    return true;
  }
  const flt64_t length = norm(sub(b, a));
  const int32_t steps = std::max(1, static_cast<int32_t>(std::ceil(length / std::max(stepM, 0.01))));
  for (int32_t i = 0; i <= steps; ++i) {
    const flt64_t t = static_cast<flt64_t>(i) / steps;
    const Vec2 p = add(a, scale(sub(b, a), t));
    if (clearanceM(p) < marginM) {
      return false;
    }
  }
  return true;
}

bool ZoneSet::pathClear(const DubinsPath& path, flt64_t marginM, flt64_t stepM) const {
  if (zones_.empty()) {
    return true;
  }
  const flt64_t length = path.lengthM();
  const int32_t steps = std::max(1, static_cast<int32_t>(std::ceil(length / std::max(stepM, 0.01))));
  for (int32_t i = 0; i <= steps; ++i) {
    const flt64_t s = length * i / steps;
    const Dubins2DPose pose = path.sample(s);
    if (clearanceM(Vec2{pose.x, pose.y}) < marginM) {
      return false;
    }
  }
  return true;
}

std::optional<flt64_t> ZoneSet::raycastFirstHit(const Vec2& origin, const Vec2& dir, flt64_t marginM,
                                                flt64_t maxRangeM) const {
  if (zones_.empty()) {
    return std::nullopt;
  }
  // Sphere tracing: clearanceM is 1-Lipschitz, so from a point with clearance c the nearest
  // sub-margin point is at least (c - marginM) away — the march can safely jump that far.
  constexpr flt64_t MIN_STEP = 0.05;
  flt64_t s = 0.0;
  while (s <= maxRangeM) {
    const Vec2 p = add(origin, scale(dir, s));
    const flt64_t c = clearanceM(p) - marginM;
    if (c <= EPS) {
      return s;
    }
    s += std::max(c, MIN_STEP);
  }
  return std::nullopt;
}

std::optional<Vec2> ZoneSet::nearestCompliantPoint(const Vec2& p, flt64_t marginM) const {
  if (zones_.empty()) {
    return p;
  }
  // Iterative projection along the binding zone's clearance gradient; a few iterations handle
  // points binding several zones.
  constexpr int32_t MAX_PROJECTIONS = 12;
  constexpr flt64_t OVERSHOOT = 1e-3;
  Vec2 q = p;
  for (int32_t i = 0; i < MAX_PROJECTIONS; ++i) {
    const ClearanceInfo info = clearanceInfo(q);
    if (info.clearanceM >= marginM) {
      return q;
    }
    q = add(q, scale(info.improveDir, marginM - info.clearanceM + OVERSHOOT));
  }
  // Fallback: expanding ring search around p (handles disjoint or wedge-shaped compliant space).
  const flt64_t startClearance = clearanceM(p);
  const flt64_t deficit = std::max(marginM - startClearance, 1.0);
  for (flt64_t radius = deficit; radius <= 64.0 * deficit; radius *= 1.5) {
    constexpr int32_t DIRECTIONS = 32;
    for (int32_t k = 0; k < DIRECTIONS; ++k) {
      const flt64_t a = 2.0 * M_PI * k / DIRECTIONS;
      const Vec2 candidate = add(p, Vec2{radius * std::cos(a), radius * std::sin(a)});
      if (clearanceM(candidate) >= marginM) {
        return candidate;
      }
    }
  }
  return std::nullopt;
}

std::optional<std::pair<Vec2, Vec2>> ZoneSet::keepInBounds() const {
  bool found = false;
  Vec2 lo{-std::numeric_limits<flt64_t>::max(), -std::numeric_limits<flt64_t>::max()};
  Vec2 hi{std::numeric_limits<flt64_t>::max(), std::numeric_limits<flt64_t>::max()};
  for (const Zone& z : zones_) {
    if (z.kind != ZoneKind::KEEP_IN) {
      continue;
    }
    found = true;
    lo.x = std::max(lo.x, z.polygon.aabbMin().x);
    lo.y = std::max(lo.y, z.polygon.aabbMin().y);
    hi.x = std::min(hi.x, z.polygon.aabbMax().x);
    hi.y = std::min(hi.y, z.polygon.aabbMax().y);
  }
  if (!found) {
    return std::nullopt;
  }
  return std::make_pair(lo, hi);
}

}  // namespace arlcore::autopilot
