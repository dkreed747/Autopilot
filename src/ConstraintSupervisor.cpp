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

#include "ConstraintSupervisor.h"

#include <algorithm>
#include <optional>

#include "DepthConditional.h"
#include "Logger.h"
#include "SpeedConditional.h"
#include "UmaaUtils.h"
#include "WaterZoneConditional.h"

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

namespace {

//! \brief Convert a UMAA elevation bound to canonical depth (positive down). MSL altitude is
//! depth-negated; AGL/ASF/geodetic frames have no fixed depth equivalent and return nullopt.
std::optional<double> elevationToDepth(const ElevationVariantTypeUnion& elevation) {
  switch (elevation._d()) {
    case ElevationVariantTypeEnum::DEPTHVARIANT_D:
      return elevation.DepthVariantVariant().depth();
    case ElevationVariantTypeEnum::ALTITUDEMSLVARIANT_D:
      return -elevation.AltitudeMSLVariantVariant().altitude();
    default:
      return std::nullopt;
  }
}

//! \brief Convert a WaterZoneConditional into the app's zone record. Returns nullopt when the
//! zone carries no usable shape.
std::optional<ZoneRecord> convertWaterZone(const WaterZoneConditional& zone) {
  ZoneRecord record;
  record.conditionalId = zone.getConditionalId();
  record.kind = zone.getConditionalWaterZoneKind() == WaterZoneKindEnumType::INSIDE
      ? ZoneKind::KEEP_IN : ZoneKind::KEEP_OUT;

  // The zone's vertical extent: ceiling = shallower bound, floor = deeper bound. Frames that
  // cannot convert to depth make the band conservatively always-applicable.
  const std::optional<double> ceilingDepth = elevationToDepth(zone.getCeiling());
  const std::optional<double> floorDepth = elevationToDepth(zone.getFloor());
  if (!ceilingDepth.has_value() || !floorDepth.has_value()) {
    record.band.convertible = false;
    UMAA_LOG_WARN(util::SYSTEM_LOGGER, "Water zone " << record.conditionalId << " has a ceiling/floor "
      "frame with no depth equivalent; treating the zone as active at every depth")
  } else {
    record.band.ceilingDepthM = std::min(ceilingDepth.value(), floorDepth.value());
    record.band.floorDepthM = std::max(ceilingDepth.value(), floorDepth.value());
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

bool isUpperBoundOp(ConditionalOperatorEnumType op) {
  return op == ConditionalOperatorEnumType::LESS_THAN ||
         op == ConditionalOperatorEnumType::LESS_THAN_OR_EQUAL_TO;
}

//! \brief Fold a bound conditional (speed/depth) into the snapshot's min/max fields.
void foldBound(ConditionalOperatorEnumType op, double value,
               std::optional<double>* minOut, std::optional<double>* maxOut) {
  if (isUpperBoundOp(op)) {
    *maxOut = maxOut->has_value() ? std::min(maxOut->value(), value) : value;
  } else {
    *minOut = minOut->has_value() ? std::max(minOut->value(), value) : value;
  }
}

}  // namespace

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
  std::lock_guard<std::mutex> lock(mtx_);
  allConditionals_ = all;
}

void ConstraintSupervisor::onActiveSetChanged(const ConditionalList& active) {
  std::lock_guard<std::mutex> lock(mtx_);
  activeConditionals_ = active;
  activeDirty_ = true;
}

void ConstraintSupervisor::update() {
  bool rebuild = false;
  {
    std::lock_guard<std::mutex> lock(mtx_);
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
}

void ConstraintSupervisor::rebuildSnapshot() {
  ConditionalList active;
  {
    std::lock_guard<std::mutex> lock(mtx_);
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

  std::lock_guard<std::mutex> lock(mtx_);
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
    std::lock_guard<std::mutex> lock(mtx_);
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
  std::lock_guard<std::mutex> lock(mtx_);
  return snapshot_;
}

uint64_t ConstraintSupervisor::revision() const {
  std::lock_guard<std::mutex> lock(mtx_);
  return snapshot_.revision;
}

bool ConstraintSupervisor::commandsAllowed() const {
  // The violation FSM (safe mode) will gate this; without it engaged, commands are allowed.
  return true;
}

}  // namespace arlcore::autopilot
