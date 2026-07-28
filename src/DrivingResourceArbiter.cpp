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

#include "DrivingResourceArbiter.h"

#include "Logger.h"

namespace arlcore::autopilot {

DrivingResourceArbiter::DrivingResourceArbiter(const ArbitrationConfig& config) : config_(config) {}

int DrivingResourceArbiter::priorityOf(DriveSource who, CommandClass cls) const {
  const ClassArbitrationPriorities& classPriorities =
      (cls == CommandClass::REMOTE) ? config_.remote : config_.local;
  switch (who) {
    case DriveSource::VECTOR:
      return classPriorities.vectorPriority;
    case DriveSource::WAYPOINT:
      return classPriorities.waypointPriority;
    case DriveSource::SAFE:
      return config_.safePriority;
    default:
      return -1;
  }
}

bool DrivingResourceArbiter::canDrive(DriveSource who, CommandClass cls) const {
  std::lock_guard<std::mutex> lock(mtx_);
  if (holder_ == DriveSource::NONE || holder_ == who) {
    return true;
  }
  return priorityOf(who, cls) > holderPriority_;
}

bool DrivingResourceArbiter::acquire(DriveSource who, CommandClass cls) {
  std::lock_guard<std::mutex> lock(mtx_);
  if (holder_ == who) {
    // Re-acquire by the holder (e.g. a new session of a different class): reprice the grant.
    holderPriority_ = priorityOf(who, cls);
    revoked_.erase(who);
    return true;
  }
  if (holder_ == DriveSource::NONE) {
    holder_ = who;
    holderPriority_ = priorityOf(who, cls);
    revoked_.erase(who);
    return true;
  }
  if (priorityOf(who, cls) > holderPriority_) {
    // Preempt the lower-priority holder.
    revoked_.insert(holder_);
    UMAA_LOG_INFO(util::SYSTEM_LOGGER, "Driving resource preempted by higher-priority source")
    holder_ = who;
    holderPriority_ = priorityOf(who, cls);
    revoked_.erase(who);
    return true;
  }
  // Equal or lower priority: denied.
  return false;
}

void DrivingResourceArbiter::release(DriveSource who) {
  std::lock_guard<std::mutex> lock(mtx_);
  if (holder_ == who) {
    holder_ = DriveSource::NONE;
    holderPriority_ = -1;
  }
  revoked_.erase(who);
}

DriveSource DrivingResourceArbiter::currentHolder() const {
  std::lock_guard<std::mutex> lock(mtx_);
  return holder_;
}

bool DrivingResourceArbiter::ownsResource(DriveSource who) const {
  std::lock_guard<std::mutex> lock(mtx_);
  return holder_ == who;
}

bool DrivingResourceArbiter::wasRevoked(DriveSource who) const {
  std::lock_guard<std::mutex> lock(mtx_);
  return revoked_.count(who) > 0;
}

}  // namespace arlcore::autopilot
