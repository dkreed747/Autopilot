#include "clients/ConstraintsClient.hpp"

#include <algorithm>
#include <cstdio>
#include <utility>

#include "autopilot/guidance/MissionRoute.hpp"
#include "UmaaUtils.h"
#include "UuidFactory.h"
#include "InternalTypes.h"

namespace arlcore::autopilot::tools {

using arlcore::io::CycloneReader;
using arlcore::io::CycloneSender;
using arlcore::io::ReadStatus;
using arlcore::io::SendStatus;
using UMAA::Common::MaritimeEnumeration::ConditionalOperatorEnumModule::ConditionalOperatorEnumType;
using UMAA::Common::MaritimeEnumeration::WaterZoneKindEnumModule::WaterZoneKindEnumType;
using UMAA::Common::Measurement::DateTime;
using UMAA::Common::Measurement::ElevationVariantTypeEnum;
using UMAA::Common::Measurement::GeoPosition2D;
using UMAA::MM::ActiveConstraintsControl::ActiveConstraintsCommandAckReportType;
using UMAA::MM::ActiveConstraintsControl::ActiveConstraintsCommandStatusType;
using UMAA::MM::ActiveConstraintsControl::ActiveConstraintsCommandType;
using UMAA::MM::Conditional::DepthConditionalType;
using UMAA::MM::Conditional::SpeedConditionalType;
using UMAA::MM::Conditional::WaterZoneConditionalType;
using UMAA::MM::ConditionalControl::ConditionalAddCommandStatusType;
using UMAA::MM::ConditionalControl::ConditionalAddCommandType;
using UMAA::MM::ConditionalControl::ConditionalDeleteCommandStatusType;
using UMAA::MM::ConditionalControl::ConditionalDeleteCommandType;
using UMAA::MM::ConditionalReport::ConditionalReportType;
using UMAA::MM::ConditionalStateReport::ConditionalStateReportType;

std::string formatUuid(const arlcore::NumericGuid& guid) {
  const std::array<uint8_t, 16> b = guid.getGuid();
  char out[37];
  std::snprintf(out, sizeof(out),
                "%02x%02x%02x%02x-%02x%02x-%02x%02x-%02x%02x-%02x%02x%02x%02x%02x%02x",
                b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7],
                b[8], b[9], b[10], b[11], b[12], b[13], b[14], b[15]);
  return std::string(out);
}

static arlcore::NumericGuid parseOrMint(const std::string& id) {
  if (id.empty()) {
    return arlcore::UuidFactory::getInstance().generateGuid();
  }
  return arlcore::UuidFactory::getInstance().parseGuidFromString(id);
}

ConstraintsClient::ConstraintsClient(const dds::domain::DomainParticipant& participant,
                                     const dds::pub::qos::DataWriterQos& wqos,
                                     const dds::sub::qos::DataReaderQos& rqos,
                                     const dds::sub::qos::DataReaderQos& largeSetRqos,
                                     const arlcore::NumericGuid& destinationId,
                    const ClientIdentity& identity)
    : zoneWriter_(std::make_shared<CycloneSender<WaterZoneConditionalType>>(
          participant, UMAA::MM::Conditional::WaterZoneConditionalTypeTopic, wqos)),
      speedWriter_(std::make_shared<CycloneSender<SpeedConditionalType>>(
          participant, UMAA::MM::Conditional::SpeedConditionalTypeTopic, wqos)),
      depthWriter_(std::make_shared<CycloneSender<DepthConditionalType>>(
          participant, UMAA::MM::Conditional::DepthConditionalTypeTopic, wqos)),
      addSender_(std::make_shared<CycloneSender<ConditionalAddCommandType>>(
          participant, UMAA::MM::ConditionalControl::ConditionalAddCommandTypeTopic, wqos)),
      deleteSender_(std::make_shared<CycloneSender<ConditionalDeleteCommandType>>(
          participant, UMAA::MM::ConditionalControl::ConditionalDeleteCommandTypeTopic, wqos)),
      activeSender_(std::make_shared<CycloneSender<ActiveConstraintsCommandType>>(
          participant, UMAA::MM::ActiveConstraintsControl::ActiveConstraintsCommandTypeTopic, wqos)),
      reportReader_(std::make_shared<CycloneReader<ConditionalReportType>>(
          participant, UMAA::MM::ConditionalReport::ConditionalReportTypeTopic, rqos)),
      setReader_(std::make_shared<CycloneReader<SetElement>>(
          participant, UMAA::MM::ConditionalReport::ConditionalReportTypeConditionalsSetElementTopic,
          largeSetRqos)),
      zoneCache_(std::make_shared<CycloneReader<WaterZoneConditionalType>>(
          participant, UMAA::MM::Conditional::WaterZoneConditionalTypeTopic, rqos)),
      speedCache_(std::make_shared<CycloneReader<SpeedConditionalType>>(
          participant, UMAA::MM::Conditional::SpeedConditionalTypeTopic, rqos)),
      depthCache_(std::make_shared<CycloneReader<DepthConditionalType>>(
          participant, UMAA::MM::Conditional::DepthConditionalTypeTopic, rqos)),
      ackReader_(std::make_shared<CycloneReader<ActiveConstraintsCommandAckReportType>>(
          participant, UMAA::MM::ActiveConstraintsControl::ActiveConstraintsCommandAckReportTypeTopic,
          rqos)),
      addStatusReader_(std::make_shared<CycloneReader<ConditionalAddCommandStatusType>>(
          participant, UMAA::MM::ConditionalControl::ConditionalAddCommandStatusTypeTopic, rqos)),
      deleteStatusReader_(std::make_shared<CycloneReader<ConditionalDeleteCommandStatusType>>(
          participant, UMAA::MM::ConditionalControl::ConditionalDeleteCommandStatusTypeTopic, rqos)),
      activeStatusReader_(std::make_shared<CycloneReader<ActiveConstraintsCommandStatusType>>(
          participant, UMAA::MM::ActiveConstraintsControl::ActiveConstraintsCommandStatusTypeTopic, rqos)),
      stateReader_(std::make_shared<CycloneReader<ConditionalStateReportType>>(
          participant, UMAA::MM::ConditionalStateReport::ConditionalStateReportTypeTopic, rqos)),
      identity_(identity),
      destinationId_(destinationId) {}

void ConstraintsClient::sendAdd(const arlcore::NumericGuid& conditionalId, const std::string& name,
                                const std::string& topic, const arlcore::NumericGuid& specId,
                                const DateTime& stamp) {
  ConditionalType generic;
  generic.conditionalID(conditionalId.getGuid());
  generic.name(name);
  generic.specializationTopic(topic);
  generic.specializationID(specId.getGuid());
  generic.specializationTimestamp(stamp);

  ConditionalAddCommandType cmd;
  cmd.conditional(generic);
  cmd.timeStamp(stamp);
  cmd.source().id(identity_.sourceId.getGuid());
  cmd.source().parentID(identity_.platformId.getGuid());
  cmd.sessionID(arlcore::UuidFactory::getInstance().generateGuid().getGuid());
  cmd.destination().id(destinationId_.getGuid());
  addSender_->send(cmd);
}

//! \brief Fill a UMAA elevation bound in the requested frame ("depth" | "asf").
static void setElevationBound(UMAA::Common::Measurement::ElevationVariantType* bound, flt64_t value,
                              const std::string& frame) {
  if (frame == "asf") {
    bound->ElevationVariantTypeSubtypes().AltitudeASFVariantVariant(
        UMAA::Common::Measurement::AltitudeASFVariantType());
    bound->ElevationVariantTypeSubtypes().AltitudeASFVariantVariant().altitude(value);
  } else {
    bound->ElevationVariantTypeSubtypes().DepthVariantVariant(
        UMAA::Common::Measurement::DepthVariantType());
    bound->ElevationVariantTypeSubtypes().DepthVariantVariant().depth(value);
  }
}

std::string ConstraintsClient::upsertZone(const std::string& id, const std::string& name,
                                          bool keepIn,
                                          const std::vector<std::array<flt64_t, 2>>& polygonLatLon,
                                          flt64_t ceilingM, const std::string& ceilingFrame,
                                          flt64_t floorM, const std::string& floorFrame) {
  const arlcore::NumericGuid conditionalId = parseOrMint(id);
  const arlcore::NumericGuid specId = arlcore::UuidFactory::getInstance().generateGuid();
  const DateTime stamp = arlcore::umaa::getTimestamp();

  WaterZoneConditionalType spec;
  spec.specializationReferenceID(specId.getGuid());
  spec.specializationReferenceTimestamp(stamp);
  spec.zoneKind(keepIn ? WaterZoneKindEnumType::INSIDE : WaterZoneKindEnumType::OUTSIDE);
  setElevationBound(&spec.ceiling(), ceilingM, ceilingFrame);
  setElevationBound(&spec.floor(), floorM, floorFrame);

  UMAA::MM::BaseType::PolygonVariantType polygon;
  for (const auto& [lat, lon] : polygonLatLon) {
    polygon.referencePoints().push_back(GeoPosition2D(lat, lon));
  }
  UMAA::MM::BaseType::ShapeVariantType shape;
  shape.ShapeVariantTypeSubtypes().PolygonVariantVariant(polygon);
  spec.zone().push_back(shape);

  zoneWriter_->send(spec);
  sendAdd(conditionalId, name, UMAA::MM::Conditional::WaterZoneConditionalTypeTopic, specId, stamp);
  return formatUuid(conditionalId);
}

std::string ConstraintsClient::upsertSpeed(const std::string& id, const std::string& name,
                                           const std::string& op, flt64_t valueMps) {
  const arlcore::NumericGuid conditionalId = parseOrMint(id);
  const arlcore::NumericGuid specId = arlcore::UuidFactory::getInstance().generateGuid();
  const DateTime stamp = arlcore::umaa::getTimestamp();
  const auto conditionalOp = op == "gte" ? ConditionalOperatorEnumType::GREATER_THAN_OR_EQUAL_TO
                                         : ConditionalOperatorEnumType::LESS_THAN_OR_EQUAL_TO;
  const SpeedConditionalType spec(conditionalOp, valueMps, stamp, specId.getGuid());
  speedWriter_->send(spec);
  sendAdd(conditionalId, name, UMAA::MM::Conditional::SpeedConditionalTypeTopic, specId, stamp);
  return formatUuid(conditionalId);
}

std::string ConstraintsClient::upsertDepth(const std::string& id, const std::string& name,
                                           const std::string& op, flt64_t valueM) {
  const arlcore::NumericGuid conditionalId = parseOrMint(id);
  const arlcore::NumericGuid specId = arlcore::UuidFactory::getInstance().generateGuid();
  const DateTime stamp = arlcore::umaa::getTimestamp();
  const auto conditionalOp = op == "gte" ? ConditionalOperatorEnumType::GREATER_THAN_OR_EQUAL_TO
                                         : ConditionalOperatorEnumType::LESS_THAN_OR_EQUAL_TO;
  const DepthConditionalType spec(conditionalOp, valueM, stamp, specId.getGuid());
  depthWriter_->send(spec);
  sendAdd(conditionalId, name, UMAA::MM::Conditional::DepthConditionalTypeTopic, specId, stamp);
  return formatUuid(conditionalId);
}

bool ConstraintsClient::removeConstraint(const std::string& id) {
  // Deleting an active conditional implicitly deactivates it, but pruning the applied set
  // first keeps the standing ack (every console's source of truth) coherent.
  if (activeIds_.count(id) > 0) {
    std::vector<std::string> remaining(activeIds_.begin(), activeIds_.end());
    remaining.erase(std::remove(remaining.begin(), remaining.end(), id), remaining.end());
    setActive(remaining);
  }
  ConditionalDeleteCommandType cmd;
  cmd.conditionalID(arlcore::UuidFactory::getInstance().parseGuidFromString(id).getGuid());
  cmd.timeStamp(arlcore::umaa::getTimestamp());
  cmd.source().id(identity_.sourceId.getGuid());
  cmd.source().parentID(identity_.platformId.getGuid());
  cmd.sessionID(arlcore::UuidFactory::getInstance().generateGuid().getGuid());
  cmd.destination().id(destinationId_.getGuid());
  return deleteSender_->send(cmd) == SendStatus::SUCCESS;
}

bool ConstraintsClient::setActive(const std::vector<std::string>& ids) {
  ActiveConstraintsCommandType cmd;
  for (const std::string& id : ids) {
    cmd.constraintConditionalIDs().push_back(
        arlcore::UuidFactory::getInstance().parseGuidFromString(id).getGuid());
  }
  cmd.timeStamp(arlcore::umaa::getTimestamp());
  cmd.source().id(identity_.sourceId.getGuid());
  cmd.source().parentID(identity_.platformId.getGuid());
  cmd.sessionID(arlcore::UuidFactory::getInstance().generateGuid().getGuid());
  cmd.destination().id(destinationId_.getGuid());
  return activeSender_->send(cmd) == SendStatus::SUCCESS;
}

void ConstraintsClient::poll() {
  // Latest conditional report -> the working constraint list. The set is re-derived every
  // poll (not only when a report sample arrives): set elements and their disposals can land
  // after the metadata, and only a re-check picks them up.
  ConditionalReportType report;
  if (reportReader_->readLatest(&report) == ReadStatus::SUCCESS) {
    lastMetadata_ = report.conditionalsSetMetadata();
  }
  if (lastMetadata_.has_value()) {
    const auto result = setReader_.getSetFromMetadata(lastMetadata_.value());
    if (result.status == arlcore::umaa::LargeSetStatus::VALID_SET) {
      conditionals_.clear();
      if (auto set = result.set.lock()) {
        conditionals_.assign(set->begin(), set->end());
      }
    } else if (result.status == arlcore::umaa::LargeSetStatus::EMPTY_SET) {
      conditionals_.clear();  // every constraint deleted (EMPTY_SET carries no set pointer)
    } else if (result.status == arlcore::umaa::LargeSetStatus::STALE_METADATA) {
      // Stale metadata never becomes valid; stop re-deriving until a fresh report arrives.
      lastMetadata_.reset();
    }
  }

  // The standing ack mirrors the applied active set (restart recovery included).
  ActiveConstraintsCommandAckReportType ack;
  ReadStatus ackStatus = ackReader_->read(&ack);
  while (ackStatus != ReadStatus::NO_DATA && ackStatus != ReadStatus::ERROR) {
    if (ackStatus == ReadStatus::SUCCESS) {
      activeIds_.clear();
      for (const auto& rawId : ack.command().constraintConditionalIDs()) {
        activeIds_.insert(formatUuid(arlcore::NumericGuid(rawId)));
      }
      activeKnown_ = true;
    }
    ackStatus = ackReader_->read(&ack);
  }

  // Violation states (disposed instance = conditional deleted).
  ConditionalStateReportType stateSample;
  ReadStatus stateStatus = stateReader_->read(&stateSample);
  while (stateStatus != ReadStatus::NO_DATA && stateStatus != ReadStatus::ERROR) {
    const std::string id = formatUuid(arlcore::NumericGuid(stateSample.conditionalID()));
    if (stateStatus == ReadStatus::DISPOSED) {
      states_.erase(id);
    } else {
      states_[id] = stateSample.state();
    }
    stateStatus = stateReader_->read(&stateSample);
  }

  // Command status streams (any commander's; shown as console activity).
  ConditionalAddCommandStatusType addStatus;
  while (addStatusReader_->read(&addStatus) == ReadStatus::SUCCESS) {
    lastEvent_ = ConstraintEvent{"add", statusName(addStatus.commandStatus()),
                                 statusReasonName(addStatus.commandStatusReason()),
                                 std::string(addStatus.logMessage())};
  }
  ConditionalDeleteCommandStatusType deleteStatus;
  while (deleteStatusReader_->read(&deleteStatus) == ReadStatus::SUCCESS) {
    lastEvent_ = ConstraintEvent{"delete", statusName(deleteStatus.commandStatus()),
                                 statusReasonName(deleteStatus.commandStatusReason()),
                                 std::string(deleteStatus.logMessage())};
  }
  ActiveConstraintsCommandStatusType activeStatus;
  while (activeStatusReader_->read(&activeStatus) == ReadStatus::SUCCESS) {
    lastEvent_ = ConstraintEvent{"active", statusName(activeStatus.commandStatus()),
                                 statusReasonName(activeStatus.commandStatusReason()),
                                 std::string(activeStatus.logMessage())};
  }

  rebuildRecords();
}

void ConstraintsClient::rebuildRecords() {
  records_.clear();
  for (const ConditionalType& conditional : conditionals_) {
    ConstraintRecord record;
    record.id = formatUuid(arlcore::NumericGuid(conditional.conditionalID()));
    record.name = conditional.name();
    record.active = activeIds_.count(record.id) > 0;
    if (const auto it = states_.find(record.id); it != states_.end()) {
      record.state = it->second;
    }

    const std::string& topic = conditional.specializationTopic();
    if (topic == UMAA::MM::Conditional::WaterZoneConditionalTypeTopic) {
      record.type = "keep_out";
      if (const auto spec = zoneCache_.getSpecialization(conditional); spec.has_value()) {
        record.type = spec->zoneKind() == WaterZoneKindEnumType::INSIDE ? "keep_in" : "keep_out";
        const auto& ceiling = spec->ceiling().ElevationVariantTypeSubtypes();
        if (ceiling._d() == ElevationVariantTypeEnum::DEPTHVARIANT_D) {
          record.ceilingM = ceiling.DepthVariantVariant().depth();
          record.ceilingFrame = "depth";
        } else if (ceiling._d() == ElevationVariantTypeEnum::ALTITUDEASFVARIANT_D) {
          record.ceilingM = ceiling.AltitudeASFVariantVariant().altitude();
          record.ceilingFrame = "asf";
        }
        const auto& floor = spec->floor().ElevationVariantTypeSubtypes();
        if (floor._d() == ElevationVariantTypeEnum::DEPTHVARIANT_D) {
          record.floorM = floor.DepthVariantVariant().depth();
          record.floorFrame = "depth";
        } else if (floor._d() == ElevationVariantTypeEnum::ALTITUDEASFVARIANT_D) {
          record.floorM = floor.AltitudeASFVariantVariant().altitude();
          record.floorFrame = "asf";
        }
        for (const auto& shape : spec->zone()) {
          if (shape.ShapeVariantTypeSubtypes()._d() ==
              UMAA::MM::BaseType::ShapeVariantTypeEnum::POLYGONVARIANT_D) {
            for (const auto& point :
                 shape.ShapeVariantTypeSubtypes().PolygonVariantVariant().referencePoints()) {
              record.polygon.push_back({point.geodeticLatitude(), point.geodeticLongitude()});
            }
            break;  // the console edits single-polygon zones
          }
        }
      }
    } else if (topic == UMAA::MM::Conditional::SpeedConditionalTypeTopic) {
      record.type = "speed";
      if (const auto spec = speedCache_.getSpecialization(conditional); spec.has_value()) {
        record.value = spec->speed();
        record.op = spec->conditionalOp() == ConditionalOperatorEnumType::GREATER_THAN ||
                    spec->conditionalOp() == ConditionalOperatorEnumType::GREATER_THAN_OR_EQUAL_TO
                        ? "gte" : "lte";
      }
    } else if (topic == UMAA::MM::Conditional::DepthConditionalTypeTopic) {
      record.type = "depth";
      if (const auto spec = depthCache_.getSpecialization(conditional); spec.has_value()) {
        record.value = spec->depth();
        record.op = spec->conditionalOp() == ConditionalOperatorEnumType::GREATER_THAN ||
                    spec->conditionalOp() == ConditionalOperatorEnumType::GREATER_THAN_OR_EQUAL_TO
                        ? "gte" : "lte";
      }
    } else {
      record.type = "other";
    }
    records_.push_back(std::move(record));
  }
}

}  // namespace arlcore::autopilot::tools
