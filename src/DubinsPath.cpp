//---------------------------------------------------------------------------
// Copyright 2025 Pennsylvania State University
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

#include "DubinsPath.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

namespace arlcore::autopilot {

namespace {

constexpr double kTwoPi = 2.0 * M_PI;

//! \brief Normalize an angle into [0, 2*pi).
double mod2pi(double theta) {
  double v = std::fmod(theta, kTwoPi);
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
  double t = 0.0;
  double p = 0.0;
  double q = 0.0;
  bool valid = false;

  double total() const { return t + p + q; }
};

// The six Shkel-Lumelsky closed forms. Inputs: alpha/beta are the start/goal headings in
// the frame whose +x axis points from start to goal position; d is the normalized distance.

Word wordLSL(double alpha, double beta, double d) {
  Word w{{SegType::LEFT, SegType::STRAIGHT, SegType::LEFT}};
  const double sa = std::sin(alpha), sb = std::sin(beta), ca = std::cos(alpha), cb = std::cos(beta);
  const double pSq = 2.0 + d * d - 2.0 * std::cos(alpha - beta) + 2.0 * d * (sa - sb);
  if (pSq < 0.0) {
    return w;
  }
  const double tmp = std::atan2(cb - ca, d + sa - sb);
  w.t = mod2pi(-alpha + tmp);
  w.p = std::sqrt(pSq);
  w.q = mod2pi(beta - tmp);
  w.valid = true;
  return w;
}

Word wordRSR(double alpha, double beta, double d) {
  Word w{{SegType::RIGHT, SegType::STRAIGHT, SegType::RIGHT}};
  const double sa = std::sin(alpha), sb = std::sin(beta), ca = std::cos(alpha), cb = std::cos(beta);
  const double pSq = 2.0 + d * d - 2.0 * std::cos(alpha - beta) + 2.0 * d * (sb - sa);
  if (pSq < 0.0) {
    return w;
  }
  const double tmp = std::atan2(ca - cb, d - sa + sb);
  w.t = mod2pi(alpha - tmp);
  w.p = std::sqrt(pSq);
  w.q = mod2pi(-beta + tmp);
  w.valid = true;
  return w;
}

Word wordLSR(double alpha, double beta, double d) {
  Word w{{SegType::LEFT, SegType::STRAIGHT, SegType::RIGHT}};
  const double sa = std::sin(alpha), sb = std::sin(beta), ca = std::cos(alpha), cb = std::cos(beta);
  const double pSq = -2.0 + d * d + 2.0 * std::cos(alpha - beta) + 2.0 * d * (sa + sb);
  if (pSq < 0.0) {
    return w;
  }
  const double p = std::sqrt(pSq);
  const double tmp = std::atan2(-ca - cb, d + sa + sb) - std::atan2(-2.0, p);
  w.t = mod2pi(-alpha + tmp);
  w.p = p;
  w.q = mod2pi(-mod2pi(beta) + tmp);
  w.valid = true;
  return w;
}

Word wordRSL(double alpha, double beta, double d) {
  Word w{{SegType::RIGHT, SegType::STRAIGHT, SegType::LEFT}};
  const double sa = std::sin(alpha), sb = std::sin(beta), ca = std::cos(alpha), cb = std::cos(beta);
  const double pSq = -2.0 + d * d + 2.0 * std::cos(alpha - beta) - 2.0 * d * (sa + sb);
  if (pSq < 0.0) {
    return w;
  }
  const double p = std::sqrt(pSq);
  const double tmp = std::atan2(ca + cb, d - sa - sb) - std::atan2(2.0, p);
  w.t = mod2pi(alpha - tmp);
  w.p = p;
  w.q = mod2pi(beta - tmp);
  w.valid = true;
  return w;
}

Word wordRLR(double alpha, double beta, double d) {
  Word w{{SegType::RIGHT, SegType::LEFT, SegType::RIGHT}};
  const double sa = std::sin(alpha), sb = std::sin(beta), ca = std::cos(alpha), cb = std::cos(beta);
  const double tmp = (6.0 - d * d + 2.0 * std::cos(alpha - beta) + 2.0 * d * (sa - sb)) / 8.0;
  if (std::fabs(tmp) > 1.0) {
    return w;
  }
  const double p = mod2pi(kTwoPi - std::acos(tmp));
  const double t = mod2pi(alpha - std::atan2(ca - cb, d - sa + sb) + p / 2.0);
  w.t = t;
  w.p = p;
  w.q = mod2pi(alpha - beta - t + p);
  w.valid = true;
  return w;
}

Word wordLRL(double alpha, double beta, double d) {
  Word w{{SegType::LEFT, SegType::RIGHT, SegType::LEFT}};
  const double sa = std::sin(alpha), sb = std::sin(beta), ca = std::cos(alpha), cb = std::cos(beta);
  const double tmp = (6.0 - d * d + 2.0 * std::cos(alpha - beta) + 2.0 * d * (sb - sa)) / 8.0;
  if (std::fabs(tmp) > 1.0) {
    return w;
  }
  const double p = mod2pi(kTwoPi - std::acos(tmp));
  const double t = mod2pi(-alpha + std::atan2(-ca + cb, d + sa - sb) + p / 2.0);
  w.t = t;
  w.p = p;
  w.q = mod2pi(mod2pi(beta) - alpha - t + p);
  w.valid = true;
  return w;
}

//! \brief Advance a pose along one segment by arc length s (meters).
Dubins2DPose advance(const Dubins2DPose& from, SegType type, double sM, double rho) {
  Dubins2DPose out = from;
  switch (type) {
    case SegType::LEFT: {
      const double dTheta = sM / rho;
      out.x = from.x + rho * (std::sin(from.theta + dTheta) - std::sin(from.theta));
      out.y = from.y - rho * (std::cos(from.theta + dTheta) - std::cos(from.theta));
      out.theta = from.theta + dTheta;
      break;
    }
    case SegType::RIGHT: {
      const double dTheta = sM / rho;
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

}  // namespace

std::optional<DubinsPath> DubinsPath::solve(const Dubins2DPose& start, const Dubins2DPose& goal, double rhoM) {
  if (!std::isfinite(start.x) || !std::isfinite(start.y) || !std::isfinite(start.theta) ||
      !std::isfinite(goal.x) || !std::isfinite(goal.y) || !std::isfinite(goal.theta) || !std::isfinite(rhoM)) {
    return std::nullopt;
  }

  DubinsPath path;
  path.start_ = start;

  const double dx = goal.x - start.x;
  const double dy = goal.y - start.y;
  const double dist = std::hypot(dx, dy);

  // Degenerate radius: fall back to a straight run at the goal position (heading constraints
  // cannot be honored without a turning circle).
  constexpr double kMinRho = 1e-3;
  if (rhoM < kMinRho) {
    path.rho_ = 1.0;
    path.start_.theta = std::atan2(dy, dx);
    path.types_ = {SegType::STRAIGHT, SegType::STRAIGHT, SegType::STRAIGHT};
    path.lengths_ = {dist, 0.0, 0.0};
    return path;
  }
  path.rho_ = rhoM;

  // Coincident poses: zero-length path.
  const double d = dist / rhoM;
  const double theta = (dist > 1e-9) ? mod2pi(std::atan2(dy, dx)) : 0.0;
  const double alpha = mod2pi(start.theta - theta);
  const double beta = mod2pi(goal.theta - theta);
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

Dubins2DPose DubinsPath::sample(double sM) const {
  double s = std::clamp(sM, 0.0, lengthM());
  Dubins2DPose pose = start_;
  for (int i = 0; i < 3; i++) {
    const double segLen = lengths_[i];
    if (s <= segLen) {
      return advance(pose, types_[i], s, rho_);
    }
    pose = advance(pose, types_[i], segLen, rho_);
    s -= segLen;
  }
  return pose;
}

std::array<DubinsSegment, 3> DubinsPath::segments() const {
  return {DubinsSegment{types_[0], lengths_[0]}, DubinsSegment{types_[1], lengths_[1]},
          DubinsSegment{types_[2], lengths_[2]}};
}

std::string DubinsPath::word() const {
  std::string out;
  for (int i = 0; i < 3; i++) {
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
