#include "autopilot/safety/ConstraintSupervisor.hpp"

#include <algorithm>
#include <optional>
#include <utility>

#include "autopilot/core/AutopilotBrain.hpp"
#include "DepthConditional.h"
#include "Logger.h"
#include "SpeedConditional.h"
#include "UmaaUtils.h"
#include "WaterZoneConditional.h"
#include "InternalTypes.h"

namespace arlcore::autopilot {

using arlcore::umaa::conditional::ConditionalBase;
using arlcore::umaa::conditional::DepthConditional;
using arlcore::umaa::conditional::SpeedConditional;
using arlcore::umaa::conditional::WaterZoneConditional;
using UMAA::Common::MaritimeEnumeration::ConditionalOperatorEnumModule::ConditionalOperatorEnumType;
using UMAA::Common::MaritimeEnumeration::WaterZoneKindEnumModule::WaterZoneKindEnumType;
using UMAA::Common::Measurement::ElevationVariantTypeEnum;
using UMAA::Common::Measurement::ElevationVariantTypeUnion;
using UMAA::MM::BaseType::ShapeVariantType;
using UMAA::MM::BaseType::ShapeVariantTypeEnum;
using UMAA::MM::ConditionalStateReport::ConditionalStateReportType;


//! \brief Convert a UMAA elevation bound to an evaluable zone bound. Depth is canonical, MSL
//! altitude negates to depth, and above-sea-floor keeps its own frame (evaluated against the
//! vehicle's altitudeASF). AGL/geodetic frames have no evaluable equivalent -> nullopt.
static std::optional<ElevationBound> toElevationBound(const ElevationVariantTypeUnion& elevation) {
  switch (elevation._d()) {
    case ElevationVariantTypeEnum::DEPTHVARIANT_D:
      return ElevationBound{ElevationBound::Frame::DEPTH, elevation.DepthVariantVariant().depth()};
    case ElevationVariantTypeEnum::ALTITUDEMSLVARIANT_D:
      return ElevationBound{ElevationBound::Frame::DEPTH,
                            -elevation.AltitudeMSLVariantVariant().altitude()};
    case ElevationVariantTypeEnum::ALTITUDEASFVARIANT_D:
      return ElevationBound{ElevationBound::Frame::ASF,
                            elevation.AltitudeASFVariantVariant().altitude()};
    default:
      return std::nullopt;
  }
}

//! \brief Convert a WaterZoneConditional into the app's zone record. Returns nullopt when the
//! zone carries no usable shape.
static std::optional<ZoneRecord> convertWaterZone(const WaterZoneConditional& zone) {
  ZoneRecord record;
  record.conditionalId = zone.getConditionalId();
  record.kind = zone.getConditionalWaterZoneKind() == WaterZoneKindEnumType::INSIDE
      ? ZoneKind::KEEP_IN : ZoneKind::KEEP_OUT;

  record.band.ceiling = toElevationBound(zone.getCeiling());
  record.band.floor = toElevationBound(zone.getFloor());
  if (!record.band.ceiling.has_value() || !record.band.floor.has_value()) {
    record.band.convertible = false;
    UMAA_LOG_WARN(util::SYSTEM_LOGGER, "Water zone " << record.conditionalId << " has a ceiling/floor "
      "frame with no evaluable equivalent; treating the zone as active at every depth")
  } else if (record.band.ceiling->frame == record.band.floor->frame) {
    // Same-frame bounds normalize so the ceiling is the shallow one: the smaller depth, or the
    // larger altitude above the sea floor.
    const bool depthFrame = record.band.ceiling->frame == ElevationBound::Frame::DEPTH;
    const bool swapped = depthFrame ? record.band.ceiling->value > record.band.floor->value
                                    : record.band.ceiling->value < record.band.floor->value;
    if (swapped) {
      std::swap(record.band.ceiling, record.band.floor);
    }
  }

  for (const ShapeVariantType& shape : zone.getZones()) {
    ZoneShape converted;
    if (shape.ShapeVariantTypeSubtypes()._d() == ShapeVariantTypeEnum::POLYGONVARIANT_D) {
      const auto& polygon = shape.ShapeVariantTypeSubtypes().PolygonVariantVariant();
      for (const auto& point : polygon.referencePoints()) {
        converted.polygon.push_back(GeoPoint{point.geodeticLatitude(), point.geodeticLongitude()});
      }
      if (converted.polygon.size() < 3) {
        UMAA_LOG_WARN(util::SYSTEM_LOGGER, "Water zone " << record.conditionalId
          << " polygon has fewer than 3 vertices; shape ignored")
        continue;
      }
    } else if (shape.ShapeVariantTypeSubtypes()._d() == ShapeVariantTypeEnum::ELLIPSEVARIANT_D) {
      const auto& ellipse = shape.ShapeVariantTypeSubtypes().EllipseVariantVariant();
      ZoneEllipse converted_ellipse;
      converted_ellipse.center = GeoPoint{ellipse.centerPosition().geodeticLatitude(),
                                          ellipse.centerPosition().geodeticLongitude()};
      converted_ellipse.semiMajorM = ellipse.semiMajorRadius();
      converted_ellipse.semiMinorM = ellipse.semiMinorRadius();
      converted_ellipse.orientationRad = ellipse.direction();
      converted.ellipse = converted_ellipse;
    } else {
      UMAA_LOG_WARN(util::SYSTEM_LOGGER, "Water zone " << record.conditionalId
        << " has an unsupported shape variant; shape ignored")
      continue;
    }
    record.shapes.push_back(std::move(converted));
  }

  if (record.shapes.empty()) {
    return std::nullopt;
  }
  return record;
}

static bool isUpperBoundOp(ConditionalOperatorEnumType op) {
  return op == ConditionalOperatorEnumType::LESS_THAN ||
         op == ConditionalOperatorEnumType::LESS_THAN_OR_EQUAL_TO;
}

//! \brief Fold a bound conditional (speed/depth) into the snapshot's min/max fields.
static void foldBound(ConditionalOperatorEnumType op, flt64_t value,
               std::optional<flt64_t>* minOut, std::optional<flt64_t>* maxOut) {
  if (isUpperBoundOp(op)) {
    *maxOut = maxOut->has_value() ? std::min(maxOut->value(), value) : value;
  } else {
    *minOut = minOut->has_value() ? std::max(minOut->value(), value) : value;
  }
}

ConstraintSupervisor::ConstraintSupervisor(const AutopilotConfig& config, NavState* nav, ZoneMap* zoneMap,
    std::shared_ptr<arlcore::io::SenderBase<ConditionalStateReportType>> stateReportSender,
    const arlcore::NumericGuid& sourceId) :
    config_(config),
    nav_(nav),
    zoneMap_(zoneMap),
    stateReportSender_(stateReportSender),
    sourceId_(sourceId) {
  setObserver_ = std::make_shared<CallbackObserver<ConditionalList>>(
      [this](const ConditionalList& all) { onConditionalSetChanged(all); });
  activeObserver_ = std::make_shared<CallbackObserver<ConditionalList>>(
      [this](const ConditionalList& active) { onActiveSetChanged(active); });
}

void ConstraintSupervisor::onConditionalSetChanged(const ConditionalList& all) {
  std::scoped_lock lock(mtx_);
  allConditionals_ = all;
}

void ConstraintSupervisor::onActiveSetChanged(const ConditionalList& active) {
  std::scoped_lock lock(mtx_);
  activeConditionals_ = active;
  activeDirty_ = true;
}

void ConstraintSupervisor::attachSafety(AutopilotBrain* brain,
                                        std::unique_ptr<ISafeModeStrategy> strategy) {
  brain_ = brain;
  strategy_ = std::move(strategy);
  UMAA_LOG_INFO(util::SYSTEM_LOGGER, "Safety supervisor armed (safe-mode strategy: "
    << (strategy_ ? strategy_->name() : "none") << ", grace " << config_.safety.gracePeriodS << " s)")
}

ConstraintSupervisor::SafetyState ConstraintSupervisor::safetyState() const {
  std::scoped_lock lock(mtx_);
  return state_;
}

void ConstraintSupervisor::update() {
  bool rebuild = false;
  {
    std::scoped_lock lock(mtx_);
    rebuild = activeDirty_;
    activeDirty_ = false;
  }
  if (rebuild) {
    rebuildSnapshot();
  }

  const auto now = std::chrono::steady_clock::now();
  if (now - lastStateReport_ >= std::chrono::milliseconds(config_.safety.stateReportPeriodMs)) {
    lastStateReport_ = now;
    publishStateReports();
  }

  if (brain_ != nullptr && strategy_) {
    updateSafety();
  }
}

flt64_t ConstraintSupervisor::gracePeriodS(ConstraintClass cls) const {
  switch (cls) {
    case ConstraintClass::ZONE:
      return config_.safety.graceZoneS.value_or(config_.safety.gracePeriodS);
    case ConstraintClass::SPEED:
      return config_.safety.graceSpeedS.value_or(config_.safety.gracePeriodS);
    case ConstraintClass::ELEVATION:
      return config_.safety.graceElevationS.value_or(config_.safety.gracePeriodS);
    default:
      return config_.safety.gracePeriodS;
  }
}

void ConstraintSupervisor::enterSafeMode(const char* why) {
  // The caller has already moved state_ to SAFE_MODE; this runs the entry actions unlocked.
  UMAA_LOG_ERROR(util::SYSTEM_LOGGER, "SAFE MODE engaged: " << why << " (strategy: "
    << strategy_->name() << ")")
  brain_->abortRecovery();
  strategy_->onEnter(brain_, nav_);
}

void ConstraintSupervisor::refreshSafeModePlan() {
  {
    std::scoped_lock lock(mtx_);
    if (state_ != SafetyState::SAFE_MODE) {
      return;
    }
  }
  if (brain_ == nullptr || strategy_ == nullptr) {
    return;
  }
  UMAA_LOG_INFO(util::SYSTEM_LOGGER, "Replanning the safe-mode maneuver from the current position")
  brain_->abortRecovery();
  strategy_->onEnter(brain_, nav_);
}

void ConstraintSupervisor::updateSafety() {
  const auto now = std::chrono::steady_clock::now();
  const std::optional<int64_t> poseAge = nav_->poseAgeMs();
  const bool poseStale = !poseAge.has_value() ||
                         poseAge.value() > config_.loop.navStalenessTimeoutMs;

  ConditionalList active;
  {
    std::scoped_lock lock(mtx_);
    active = activeConditionals_;
  }

  if (poseStale) {
    // Freeze every deadline while navigation is unusable (the brain is zero-holding anyway):
    // shift the reference timestamps forward by the elapsed interval.
    if (lastSafetyTick_.has_value()) {
      const auto dt = now - lastSafetyTick_.value();
      std::scoped_lock lock(mtx_);
      for (auto& [id, tracker] : trackers_) {
        tracker.confirmedAt += dt;
        if (tracker.compliantSince.has_value()) {
          tracker.compliantSince.value() += dt;
        }
      }
      if (allCompliantSince_.has_value()) {
        allCompliantSince_.value() += dt;
      }
    }
    lastSafetyTick_ = now;
    return;
  }
  lastSafetyTick_ = now;

  const std::optional<UMAA::SA::GlobalPoseStatus::GlobalPoseReportType> pose = nav_->pose();
  const GeoPoint at{pose.has_value() ? pose->position().geodeticLatitude() : 0.0,
                    pose.has_value() ? pose->position().geodeticLongitude() : 0.0};
  const flt64_t depthM = (pose.has_value() && pose->depth().has_value()) ? pose->depth().value() : 0.0;
  const std::optional<flt64_t> asfM = (pose.has_value() && pose->altitudeASF().has_value())
      ? std::optional<flt64_t>(pose->altitudeASF().value()) : std::nullopt;

  bool anyConfirmed = false;
  bool anyConfirmedZone = false;
  std::optional<std::chrono::steady_clock::time_point> earliestDeadline;

  {
  std::scoped_lock lock(mtx_);
  std::set<arlcore::NumericGuid> activeIds;
  for (const auto& conditional : active) {
    if (!conditional) {
      continue;
    }
    const arlcore::NumericGuid id = conditional->getConditionalId();
    activeIds.insert(id);
    ViolationTracker& tracker = trackers_[id];
    if (dynamic_cast<WaterZoneConditional*>(conditional.get()) != nullptr) {
      tracker.cls = ConstraintClass::ZONE;
    } else if (dynamic_cast<SpeedConditional*>(conditional.get()) != nullptr) {
      tracker.cls = ConstraintClass::SPEED;
    } else if (dynamic_cast<DepthConditional*>(conditional.get()) != nullptr) {
      tracker.cls = ConstraintClass::ELEVATION;
    }

    const std::optional<bool> eval = conditional->evaluateConditional();
    if (!eval.has_value()) {
      continue;  // not evaluable this tick; hold state
    }
    const bool violated = !eval.value();
    if (violated) {
      tracker.compliantSince.reset();
      if (!tracker.confirmed && ++tracker.rawViolatingTicks >= config_.safety.violationConfirmTicks) {
        tracker.confirmed = true;
        tracker.confirmedAt = now;
        UMAA_LOG_WARN(util::SYSTEM_LOGGER, "Constraint violation confirmed: "
          << conditional->getName() << " (" << id << "), grace "
          << gracePeriodS(tracker.cls) << " s")
      }
    } else {
      tracker.rawViolatingTicks = 0;
      if (tracker.confirmed) {
        // Clearing needs the predicate true for clear_hold_s — and, for zones, the position
        // classified COMPLIANT (outside the hysteresis band), not merely a hair's breadth in.
        const bool zoneCompliant = tracker.cls != ConstraintClass::ZONE || zoneMap_ == nullptr ||
            zoneMap_->classify(at, depthM, asfM) == ZoneCompliance::COMPLIANT;
        if (zoneCompliant) {
          if (!tracker.compliantSince.has_value()) {
            tracker.compliantSince = now;
          }
          if (std::chrono::duration<flt64_t>(now - tracker.compliantSince.value()).count() >=
              config_.safety.clearHoldS) {
            tracker.confirmed = false;
            UMAA_LOG_INFO(util::SYSTEM_LOGGER, "Constraint violation cleared: "
              << conditional->getName() << " (" << id << ")")
          }
        } else {
          tracker.compliantSince.reset();
        }
      }
    }

    if (tracker.confirmed) {
      anyConfirmed = true;
      anyConfirmedZone = anyConfirmedZone || tracker.cls == ConstraintClass::ZONE;
      const auto deadline = tracker.confirmedAt + std::chrono::duration_cast<
          std::chrono::steady_clock::duration>(std::chrono::duration<flt64_t>(gracePeriodS(tracker.cls)));
      if (!earliestDeadline.has_value() || deadline < earliestDeadline.value()) {
        earliestDeadline = deadline;
      }
    }
  }
  // Deactivated conditionals stop tracking (deleting an active constraint deactivates it).
  for (auto it = trackers_.begin(); it != trackers_.end();) {
    it = activeIds.count(it->first) == 0 ? trackers_.erase(it) : std::next(it);
  }

  if (anyConfirmed) {
    allCompliantSince_.reset();
  } else if (!allCompliantSince_.has_value()) {
    allCompliantSince_ = now;
  }
  }  // release the lock: the brain/strategy calls below must run unlocked

  // State transitions: everything runs on the single control thread; state_ writes take the
  // lock briefly, the brain/strategy calls happen unlocked (they take their own locks).
  const bool graceExpired = anyConfirmed && earliestDeadline.has_value() &&
                            now >= earliestDeadline.value();
  flt64_t allClearS = 0.0;
  SafetyState state = SafetyState::MONITORING;
  {
    std::scoped_lock lock(mtx_);
    state = state_;
    if (allCompliantSince_.has_value()) {
      allClearS = std::chrono::duration<flt64_t>(now - allCompliantSince_.value()).count();
    }
  }
  const auto setState = [this](SafetyState next) {
    std::scoped_lock lock(mtx_);
    state_ = next;
  };

  switch (state) {
    case SafetyState::MONITORING:
      if (graceExpired) {
        setState(SafetyState::SAFE_MODE);
        enterSafeMode("violation grace period expired");
      } else if (anyConfirmedZone) {
        setState(SafetyState::RECOVERING);
        UMAA_LOG_WARN(util::SYSTEM_LOGGER, "Zone violation: engaging recovery while the grace "
          "timer runs")
        brain_->beginRecovery();
      }
      break;
    case SafetyState::RECOVERING:
      if (graceExpired) {
        setState(SafetyState::SAFE_MODE);
        enterSafeMode("violation persisted past the grace period");
      } else if (!anyConfirmed) {
        setState(SafetyState::MONITORING);
        brain_->endRecovery();
      }
      break;
    case SafetyState::SAFE_MODE: {
      strategy_->onTick(brain_, nav_);
      // A strategy that manages its own release (SRP with accept_commands_after_srp) wins;
      // exit_on_all_clear additionally releases strategies that never signal readiness
      // (zero-speed hold) once every violation has stayed clear for the hold time.
      const bool allClearRelease = config_.safety.exitOnAllClear && !anyConfirmed &&
                                   allClearS >= config_.safety.clearHoldS;
      if (strategy_->readyToRelease() || allClearRelease) {
        setState(SafetyState::MONITORING);
        UMAA_LOG_INFO(util::SYSTEM_LOGGER, "SAFE MODE released; monitoring resumes")
        strategy_->onExit(brain_);
      }
      break;
    }
  }
}

void ConstraintSupervisor::rebuildSnapshot() {
  ConditionalList active;
  {
    std::scoped_lock lock(mtx_);
    active = activeConditionals_;
  }

  ConstraintSnapshot next;
  for (const std::shared_ptr<ConditionalBase>& conditional : active) {
    if (!conditional) {
      continue;
    }
    if (auto* zone = dynamic_cast<WaterZoneConditional*>(conditional.get())) {
      std::optional<ZoneRecord> record = convertWaterZone(*zone);
      if (record.has_value()) {
        next.zones.push_back(std::move(record.value()));
      }
    } else if (auto* speed = dynamic_cast<SpeedConditional*>(conditional.get())) {
      foldBound(speed->getConditionalOperatorEnum(), speed->getConditionalSpeed(),
                &next.minSpeedMps, &next.maxSpeedMps);
    } else if (auto* depth = dynamic_cast<DepthConditional*>(conditional.get())) {
      foldBound(depth->getConditionalOperatorEnum(), depth->getConditionalDepth(),
                &next.minDepthM, &next.maxDepthM);
    } else {
      UMAA_LOG_WARN(util::SYSTEM_LOGGER, "Active constraint " << conditional->getConditionalId()
        << " (" << conditional->getName() << ") is not a supported constraint type; it is "
        "monitored but not enforced by the autopilot")
    }
  }

  std::scoped_lock lock(mtx_);
  next.revision = ++revision_;
  snapshot_ = next;
  if (zoneMap_ != nullptr) {
    zoneMap_->ingest(snapshot_);
  }
  UMAA_LOG_INFO(util::SYSTEM_LOGGER, "Constraint snapshot rebuilt (revision " << snapshot_.revision
    << "): " << snapshot_.zones.size() << " zone(s)"
    << (snapshot_.maxSpeedMps.has_value() ? ", max speed " + std::to_string(snapshot_.maxSpeedMps.value()) : "")
    << (snapshot_.minSpeedMps.has_value() ? ", min speed " + std::to_string(snapshot_.minSpeedMps.value()) : "")
    << (snapshot_.maxDepthM.has_value() ? ", max depth " + std::to_string(snapshot_.maxDepthM.value()) : "")
    << (snapshot_.minDepthM.has_value() ? ", min depth " + std::to_string(snapshot_.minDepthM.value()) : ""))
}

void ConstraintSupervisor::publishStateReports() {
  if (!stateReportSender_) {
    return;
  }
  ConditionalList all;
  {
    std::scoped_lock lock(mtx_);
    all = allConditionals_;
  }

  std::set<arlcore::NumericGuid> current;
  for (const std::shared_ptr<ConditionalBase>& conditional : all) {
    if (!conditional) {
      continue;
    }
    const std::optional<bool> state = conditional->evaluateConditional();
    if (!state.has_value()) {
      continue;  // not evaluable yet (e.g. no nav data); publish nothing rather than a guess
    }
    ConditionalStateReportType report;
    report.source().id(sourceId_.getGuid());
    report.conditionalID(conditional->getConditionalId().getGuid());
    report.state(state.value());
    report.timeStamp(arlcore::umaa::getTimestamp());
    if (stateReportSender_->send(report) == arlcore::io::SendStatus::SUCCESS) {
      current.insert(conditional->getConditionalId());
    }
  }

  // Dispose state instances of conditionals that no longer exist.
  for (const arlcore::NumericGuid& id : publishedStateIds_) {
    if (current.count(id) == 0) {
      ConditionalStateReportType gone;
      gone.source().id(sourceId_.getGuid());
      gone.conditionalID(id.getGuid());
      gone.timeStamp(arlcore::umaa::getTimestamp());
      stateReportSender_->dispose(gone);
    }
  }
  publishedStateIds_ = std::move(current);
}

ConstraintSnapshot ConstraintSupervisor::snapshot() const {
  std::scoped_lock lock(mtx_);
  return snapshot_;
}

uint64_t ConstraintSupervisor::revision() const {
  std::scoped_lock lock(mtx_);
  return snapshot_.revision;
}

bool ConstraintSupervisor::commandsAllowed() const {
  std::scoped_lock lock(mtx_);
  // Recovery and safe mode both hold the vehicle; only MONITORING accepts new commands.
  return brain_ == nullptr || state_ == SafetyState::MONITORING;
}

}  // namespace arlcore::autopilot
