#include "autopilot/guidance/DubinsPath.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

#include "InternalTypes.h"

namespace arlcore::autopilot {

constexpr flt64_t kTwoPi = 2.0 * M_PI;

//! \brief Normalize an angle into [0, 2*pi).
static flt64_t mod2pi(flt64_t theta) {
  flt64_t v = std::fmod(theta, kTwoPi);
  if (v < 0.0) {
    v += kTwoPi;
  }
  return v;
}

using SegType = DubinsSegment::Type;

//! \brief One candidate word: three segment types plus normalized params (t, p, q).
//! Arc params are in radians; the straight param is distance in units of rho.
struct Word {
  SegType types[3];
  flt64_t t = 0.0;
  flt64_t p = 0.0;
  flt64_t q = 0.0;
  bool valid = false;

  flt64_t total() const { return t + p + q; }
};

// The six Shkel-Lumelsky closed forms. Inputs: alpha/beta are the start/goal headings in
// the frame whose +x axis points from start to goal position; d is the normalized distance.

static Word wordLSL(flt64_t alpha, flt64_t beta, flt64_t d) {
  Word w{{SegType::LEFT, SegType::STRAIGHT, SegType::LEFT}};
  const flt64_t sa = std::sin(alpha), sb = std::sin(beta), ca = std::cos(alpha), cb = std::cos(beta);
  const flt64_t pSq = 2.0 + d * d - 2.0 * std::cos(alpha - beta) + 2.0 * d * (sa - sb);
  if (pSq < 0.0) {
    return w;
  }
  const flt64_t tmp = std::atan2(cb - ca, d + sa - sb);
  w.t = mod2pi(-alpha + tmp);
  w.p = std::sqrt(pSq);
  w.q = mod2pi(beta - tmp);
  w.valid = true;
  return w;
}

static Word wordRSR(flt64_t alpha, flt64_t beta, flt64_t d) {
  Word w{{SegType::RIGHT, SegType::STRAIGHT, SegType::RIGHT}};
  const flt64_t sa = std::sin(alpha), sb = std::sin(beta), ca = std::cos(alpha), cb = std::cos(beta);
  const flt64_t pSq = 2.0 + d * d - 2.0 * std::cos(alpha - beta) + 2.0 * d * (sb - sa);
  if (pSq < 0.0) {
    return w;
  }
  const flt64_t tmp = std::atan2(ca - cb, d - sa + sb);
  w.t = mod2pi(alpha - tmp);
  w.p = std::sqrt(pSq);
  w.q = mod2pi(-beta + tmp);
  w.valid = true;
  return w;
}

static Word wordLSR(flt64_t alpha, flt64_t beta, flt64_t d) {
  Word w{{SegType::LEFT, SegType::STRAIGHT, SegType::RIGHT}};
  const flt64_t sa = std::sin(alpha), sb = std::sin(beta), ca = std::cos(alpha), cb = std::cos(beta);
  const flt64_t pSq = -2.0 + d * d + 2.0 * std::cos(alpha - beta) + 2.0 * d * (sa + sb);
  if (pSq < 0.0) {
    return w;
  }
  const flt64_t p = std::sqrt(pSq);
  const flt64_t tmp = std::atan2(-ca - cb, d + sa + sb) - std::atan2(-2.0, p);
  w.t = mod2pi(-alpha + tmp);
  w.p = p;
  w.q = mod2pi(-mod2pi(beta) + tmp);
  w.valid = true;
  return w;
}

static Word wordRSL(flt64_t alpha, flt64_t beta, flt64_t d) {
  Word w{{SegType::RIGHT, SegType::STRAIGHT, SegType::LEFT}};
  const flt64_t sa = std::sin(alpha), sb = std::sin(beta), ca = std::cos(alpha), cb = std::cos(beta);
  const flt64_t pSq = -2.0 + d * d + 2.0 * std::cos(alpha - beta) - 2.0 * d * (sa + sb);
  if (pSq < 0.0) {
    return w;
  }
  const flt64_t p = std::sqrt(pSq);
  const flt64_t tmp = std::atan2(ca + cb, d - sa - sb) - std::atan2(2.0, p);
  w.t = mod2pi(alpha - tmp);
  w.p = p;
  w.q = mod2pi(beta - tmp);
  w.valid = true;
  return w;
}

static Word wordRLR(flt64_t alpha, flt64_t beta, flt64_t d) {
  Word w{{SegType::RIGHT, SegType::LEFT, SegType::RIGHT}};
  const flt64_t sa = std::sin(alpha), sb = std::sin(beta), ca = std::cos(alpha), cb = std::cos(beta);
  const flt64_t tmp = (6.0 - d * d + 2.0 * std::cos(alpha - beta) + 2.0 * d * (sa - sb)) / 8.0;
  if (std::fabs(tmp) > 1.0) {
    return w;
  }
  const flt64_t p = mod2pi(kTwoPi - std::acos(tmp));
  const flt64_t t = mod2pi(alpha - std::atan2(ca - cb, d - sa + sb) + p / 2.0);
  w.t = t;
  w.p = p;
  w.q = mod2pi(alpha - beta - t + p);
  w.valid = true;
  return w;
}

static Word wordLRL(flt64_t alpha, flt64_t beta, flt64_t d) {
  Word w{{SegType::LEFT, SegType::RIGHT, SegType::LEFT}};
  const flt64_t sa = std::sin(alpha), sb = std::sin(beta), ca = std::cos(alpha), cb = std::cos(beta);
  const flt64_t tmp = (6.0 - d * d + 2.0 * std::cos(alpha - beta) + 2.0 * d * (sb - sa)) / 8.0;
  if (std::fabs(tmp) > 1.0) {
    return w;
  }
  const flt64_t p = mod2pi(kTwoPi - std::acos(tmp));
  const flt64_t t = mod2pi(-alpha + std::atan2(-ca + cb, d + sa - sb) + p / 2.0);
  w.t = t;
  w.p = p;
  w.q = mod2pi(mod2pi(beta) - alpha - t + p);
  w.valid = true;
  return w;
}

//! \brief Advance a pose along one segment by arc length s (meters).
static Dubins2DPose advance(const Dubins2DPose& from, SegType type, flt64_t sM, flt64_t rho) {
  Dubins2DPose out = from;
  switch (type) {
    case SegType::LEFT: {
      const flt64_t dTheta = sM / rho;
      out.x = from.x + rho * (std::sin(from.theta + dTheta) - std::sin(from.theta));
      out.y = from.y - rho * (std::cos(from.theta + dTheta) - std::cos(from.theta));
      out.theta = from.theta + dTheta;
      break;
    }
    case SegType::RIGHT: {
      const flt64_t dTheta = sM / rho;
      out.x = from.x - rho * (std::sin(from.theta - dTheta) - std::sin(from.theta));
      out.y = from.y + rho * (std::cos(from.theta - dTheta) - std::cos(from.theta));
      out.theta = from.theta - dTheta;
      break;
    }
    case SegType::STRAIGHT:
      out.x = from.x + sM * std::cos(from.theta);
      out.y = from.y + sM * std::sin(from.theta);
      break;
  }
  return out;
}

//! \brief Signed curvature of one segment type, math convention (LEFT positive).
static flt64_t segmentCurvature(SegType type, flt64_t rho) {
  if (rho <= 0.0) {
    return 0.0;
  }
  switch (type) {
    case SegType::LEFT:
      return 1.0 / rho;
    case SegType::RIGHT:
      return -1.0 / rho;
    case SegType::STRAIGHT:
      break;
  }
  return 0.0;
}

std::optional<DubinsPath> DubinsPath::solve(const Dubins2DPose& start, const Dubins2DPose& goal, flt64_t rhoM) {
  if (!std::isfinite(start.x) || !std::isfinite(start.y) || !std::isfinite(start.theta) || !std::isfinite(goal.x) ||
      !std::isfinite(goal.y) || !std::isfinite(goal.theta) || !std::isfinite(rhoM)) {
    return std::nullopt;
  }

  DubinsPath path;
  path.start_ = start;

  const flt64_t dx = goal.x - start.x;
  const flt64_t dy = goal.y - start.y;
  const flt64_t dist = std::hypot(dx, dy);

  // Degenerate radius: fall back to a straight run at the goal position (heading constraints
  // cannot be honored without a turning circle).
  constexpr flt64_t kMinRho = 1e-3;
  if (rhoM < kMinRho) {
    // No turning circle exists, so rho_ stays 0 rather than carrying a sentinel radius.
    path.rho_ = 0.0;
    path.start_.theta = std::atan2(dy, dx);
    path.types_ = {SegType::STRAIGHT, SegType::STRAIGHT, SegType::STRAIGHT};
    path.lengths_ = {dist, 0.0, 0.0};
    return path;
  }
  path.rho_ = rhoM;

  // Coincident poses: zero-length path.
  const flt64_t d = dist / rhoM;
  const flt64_t theta = (dist > 1e-9) ? mod2pi(std::atan2(dy, dx)) : 0.0;
  const flt64_t alpha = mod2pi(start.theta - theta);
  const flt64_t beta = mod2pi(goal.theta - theta);
  if (dist < 1e-9 && std::fabs(std::remainder(start.theta - goal.theta, kTwoPi)) < 1e-9) {
    path.types_ = {SegType::STRAIGHT, SegType::STRAIGHT, SegType::STRAIGHT};
    path.lengths_ = {0.0, 0.0, 0.0};
    return path;
  }

  const Word candidates[6] = {wordLSL(alpha, beta, d), wordRSR(alpha, beta, d), wordLSR(alpha, beta, d),
                              wordRSL(alpha, beta, d), wordRLR(alpha, beta, d), wordLRL(alpha, beta, d)};

  const Word* best = nullptr;
  for (const Word& w : candidates) {
    if (w.valid && (best == nullptr || w.total() < best->total())) {
      best = &w;
    }
  }
  // At least one CSC word is valid for every configuration with rho > 0; this is defensive.
  if (best == nullptr) {
    path.start_.theta = std::atan2(dy, dx);
    path.types_ = {SegType::STRAIGHT, SegType::STRAIGHT, SegType::STRAIGHT};
    path.lengths_ = {dist, 0.0, 0.0};
    return path;
  }

  path.types_ = {best->types[0], best->types[1], best->types[2]};
  path.lengths_ = {best->t * rhoM, best->p * rhoM, best->q * rhoM};
  return path;
}

Dubins2DPose DubinsPath::sample(flt64_t sM) const {
  flt64_t s = std::clamp(sM, 0.0, lengthM());
  Dubins2DPose pose = start_;
  for (std::size_t i = 0; i < 3; i++) {
    const flt64_t segLen = lengths_[i];
    if (s <= segLen) {
      return advance(pose, types_[i], s, rho_);
    }
    pose = advance(pose, types_[i], segLen, rho_);
    s -= segLen;
  }
  return pose;
}

flt64_t DubinsPath::curvatureAt(flt64_t sM) const {
  // The segment walk mirrors sample() exactly so the two can never disagree about which
  // segment owns a given arc length.
  flt64_t s = std::clamp(sM, 0.0, lengthM());
  for (std::size_t i = 0; i < 3; i++) {
    const flt64_t segLen = lengths_[i];
    if (s <= segLen) {
      return segmentCurvature(types_[i], rho_);
    }
    s -= segLen;
  }
  return segmentCurvature(types_[2], rho_);
}

std::array<DubinsSegment, 3> DubinsPath::segments() const {
  return {DubinsSegment{types_[0], lengths_[0]}, DubinsSegment{types_[1], lengths_[1]},
          DubinsSegment{types_[2], lengths_[2]}};
}

std::string DubinsPath::word() const {
  std::string out;
  for (std::size_t i = 0; i < 3; i++) {
    switch (types_[i]) {
      case SegType::LEFT:
        out += 'L';
        break;
      case SegType::STRAIGHT:
        out += 'S';
        break;
      case SegType::RIGHT:
        out += 'R';
        break;
    }
  }
  return out;
}

}  // namespace arlcore::autopilot
