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

#include "AutopilotApp.h"

#include <chrono>
#include <memory>
#include <regex>
#include <string>
#include <thread>

#include <UMAA/SA/GlobalPoseStatus/GlobalPoseReportType.hpp>
#include <UMAA/SA/SpeedStatus/SpeedReportType.hpp>
#include <UMAA/SA/VelocityStatus/VelocityReportType.hpp>

#include "CycloneQosProviderWrapper.h"
#include "CycloneReader.h"
#include "CycloneSender.h"
#include "CycloneUtilities.h"
#include "Logger.h"
#include "UuidFactory.h"

namespace arlcore::autopilot {

using arlcore::io::CycloneReader;
using arlcore::io::CycloneSender;
using arlcore::umaa::services::GlobalPoseReportConsumer;
using arlcore::umaa::services::SpeedReportConsumer;
using arlcore::umaa::services::VelocityReportConsumer;
using arlcore::umaa::services::ReportProvider;

namespace {
arlcore::NumericGuid parseId(const std::string& uuid) {
  return arlcore::UuidFactory::getInstance().parseGuidFromString(uuid);
}

//! \brief A source ID must be a well-formed UUID string; anything else would silently
//! produce a garbage GUID and commands addressed to the configured ID would never match.
bool validSourceId(const std::string& uuid, const char* name) {
  static const std::regex kUuidPattern(
      "^[0-9a-fA-F]{8}-[0-9a-fA-F]{4}-[0-9a-fA-F]{4}-[0-9a-fA-F]{4}-[0-9a-fA-F]{12}$");
  if (std::regex_match(uuid, kUuidPattern)) {
    return true;
  }
  UMAA_LOG_ERROR(util::SYSTEM_LOGGER, "identity." << name << " is not a valid UUID: '" << uuid << "'")
  return false;
}
}  // namespace

bool AutopilotApp::initialize(const AutopilotConfig& config) {
  config_ = config;

  if (!validSourceId(config_.identity.vectorSourceId, "vector_source_id") ||
      !validSourceId(config_.identity.waypointSourceId, "waypoint_source_id") ||
      !validSourceId(config_.identity.specsSourceId, "specs_source_id") ||
      !validSourceId(config_.identity.capabilitiesSourceId, "capabilities_source_id") ||
      !validSourceId(config_.identity.navSourceId, "nav_source_id")) {
    return false;
  }

  // The platform capabilities drive the planner and tracker: a representative speed and the
  // max turn rate are required to derive the turn radius.
  const CapabilityLimits& surf = config_.platformCapabilities.surface;
  const bool hasSpeed = surf.cruisingSpeedMps.has_value() || surf.maxForwardSpeedMps.has_value();
  if (!hasSpeed || !surf.maxTurnRateRps.has_value() || surf.maxTurnRateRps.value() <= 0.0) {
    UMAA_LOG_ERROR(util::SYSTEM_LOGGER, "platform_capabilities.surface must define "
      "cruising/max forward speed and a positive max_turn_rate_rps (they drive the planner)")
    return false;
  }

  participant_ = arlcore::io::getDomainParticipant(config_.dds.domainId);
  subscriber_ = arlcore::io::createSubscriber(participant_);
  publisher_ = arlcore::io::createPublisher(participant_);

  // Honor the configured UMAA QoS profiles (reliable/transient-local); the wrapper falls back
  // to defaults when the file or profile cannot be resolved.
  arlcore::io::CycloneQosProviderWrapper qosProvider(config_.dds.qosFile, config_.dds.domainQosProfile);
  const auto rqos = qosProvider.datareader_qos();
  const auto wqos = qosProvider.datawriter_qos();
  const auto largeListRqos = qosProvider.datareader_qos(config_.dds.largeCollectionsQosProfile);

  // Vehicle-control strategy (only "sim" is provided here; extend by strategy type).
  if (config_.vehicleControlType != "sim") {
    UMAA_LOG_WARN(util::SYSTEM_LOGGER, "Unknown vehicle_control.type '" << config_.vehicleControlType
      << "', defaulting to sim")
  }
  vehicle_ = std::make_unique<SimVehicleControl>(
      config_.platformSpecs, config_.platformCapabilities, config_.simVehicle,
      parseId(config_.identity.navSourceId),
      std::make_shared<CycloneSender<UMAA::SA::GlobalPoseStatus::GlobalPoseReportType>>(
          participant_, UMAA::SA::GlobalPoseStatus::GlobalPoseReportTypeTopic, wqos),
      std::make_shared<CycloneSender<UMAA::SA::SpeedStatus::SpeedReportType>>(
          participant_, UMAA::SA::SpeedStatus::SpeedReportTypeTopic, wqos),
      std::make_shared<CycloneSender<UMAA::SA::VelocityStatus::VelocityReportType>>(
          participant_, UMAA::SA::VelocityStatus::VelocityReportTypeTopic, wqos));
  if (!vehicle_->initialize()) {
    UMAA_LOG_ERROR(util::SYSTEM_LOGGER, "Vehicle control failed to initialize")
    return false;
  }

  brain_ = std::make_unique<AutopilotBrain>(&navState_, vehicle_.get(), config_);

  // --- Navigation consumers (listeners) ---
  poseConsumer_ = std::make_shared<GlobalPoseReportConsumer>(
      std::make_shared<CycloneReader<UMAA::SA::GlobalPoseStatus::GlobalPoseReportType>>(
          participant_, UMAA::SA::GlobalPoseStatus::GlobalPoseReportTypeTopic, rqos));
  speedConsumer_ = std::make_shared<SpeedReportConsumer>(
      std::make_shared<CycloneReader<UMAA::SA::SpeedStatus::SpeedReportType>>(
          participant_, UMAA::SA::SpeedStatus::SpeedReportTypeTopic, rqos));
  velocityConsumer_ = std::make_shared<VelocityReportConsumer>(
      std::make_shared<CycloneReader<UMAA::SA::VelocityStatus::VelocityReportType>>(
          participant_, UMAA::SA::VelocityStatus::VelocityReportTypeTopic, rqos));

  poseObserver_ = std::make_shared<GlobalPoseObserver>(&navState_, brain_.get());
  speedObserver_ = std::make_shared<SpeedObserver>(&navState_);
  velocityObserver_ = std::make_shared<VelocityObserver>(&navState_);
  poseConsumer_->getReportSubject().registerObserver(poseObserver_);
  speedConsumer_->getReportSubject().registerObserver(speedObserver_);
  velocityConsumer_->getReportSubject().registerObserver(velocityObserver_);

  const double maxForwardSpeed = config_.platformCapabilities.surface.maxForwardSpeedMps.value_or(0.0);

  // --- Vector control provider ---
  auto vectorIo = std::make_shared<VectorControlServiceProviderIo>(
      std::make_shared<CycloneReader<GlobalVectorCommandType>>(
          participant_, UMAA::MO::GlobalVectorControl::GlobalVectorCommandTypeTopic, rqos),
      std::make_shared<CycloneSender<GlobalVectorCommandAckReportType>>(
          participant_, UMAA::MO::GlobalVectorControl::GlobalVectorCommandAckReportTypeTopic, wqos),
      std::make_shared<CycloneSender<GlobalVectorCommandStatusType>>(
          participant_, UMAA::MO::GlobalVectorControl::GlobalVectorCommandStatusTypeTopic, wqos),
      std::make_shared<CycloneSender<GlobalVectorExecutionStatusReportType>>(
          participant_, UMAA::MO::GlobalVectorControl::GlobalVectorExecutionStatusReportTypeTopic, wqos));
  vectorProvider_ = std::make_unique<VectorControlServiceProvider>(
      parseId(config_.identity.vectorSourceId), vectorIo, brain_.get(), maxForwardSpeed);

  // --- Waypoint control provider (with large-list element reader) ---
  auto waypointIo = std::make_shared<WaypointControlServiceProviderIo>(
      std::make_shared<CycloneReader<GlobalWaypointCommandType>>(
          participant_, UMAA::MO::GlobalWaypointControl::GlobalWaypointCommandTypeTopic, rqos),
      std::make_shared<CycloneSender<GlobalWaypointCommandAckReportType>>(
          participant_, UMAA::MO::GlobalWaypointControl::GlobalWaypointCommandAckReportTypeTopic, wqos),
      std::make_shared<CycloneSender<GlobalWaypointCommandStatusType>>(
          participant_, UMAA::MO::GlobalWaypointControl::GlobalWaypointCommandStatusTypeTopic, wqos),
      std::make_shared<CycloneSender<GlobalWaypointExecutionStatusReportType>>(
          participant_, UMAA::MO::GlobalWaypointControl::GlobalWaypointExecutionStatusReportTypeTopic, wqos),
      std::make_shared<CycloneReader<GlobalWaypointCommandTypeWaypointsListElement>>(
          participant_, UMAA::MO::GlobalWaypointControl::GlobalWaypointCommandTypeWaypointsListElementTopic,
          largeListRqos));
  waypointProvider_ = std::make_unique<WaypointControlServiceProvider>(
      parseId(config_.identity.waypointSourceId), waypointIo, brain_.get(), maxForwardSpeed,
      config_.planner.maxListWaitCycles);

  // --- Platform report providers: publish specs + capabilities once on startup ---
  specsReportProvider_ = std::make_unique<ReportProvider<UMAA::EO::UVPlatformSpecs::UVPlatformSpecsReportType>>(
      parseId(config_.identity.specsSourceId),
      std::make_shared<CycloneSender<UMAA::EO::UVPlatformSpecs::UVPlatformSpecsReportType>>(
          participant_, UMAA::EO::UVPlatformSpecs::UVPlatformSpecsReportTypeTopic, wqos));
  capabilitiesReportProvider_ =
      std::make_unique<ReportProvider<UMAA::EO::UVPlatformSpecs::UVPlatformCapabilitiesReportType>>(
          parseId(config_.identity.capabilitiesSourceId),
          std::make_shared<CycloneSender<UMAA::EO::UVPlatformSpecs::UVPlatformCapabilitiesReportType>>(
              participant_, UMAA::EO::UVPlatformSpecs::UVPlatformCapabilitiesReportTypeTopic, wqos));

  auto specs = vehicle_->getPlatformSpecs();
  auto capabilities = vehicle_->getPlatformCapabilities();
  specsReportProvider_->send(&specs);
  capabilitiesReportProvider_->send(&capabilities);

  UMAA_LOG_INFO(util::SYSTEM_LOGGER, "Autopilot initialized on domain " << config_.dds.domainId)
  return true;
}

void AutopilotApp::step() {
  // Cycle nav consumers first; their observers refresh nav state and drive the control
  // recompute (pose-triggered) before the providers publish status reflecting the latest state.
  poseConsumer_->cycle();
  speedConsumer_->cycle();
  velocityConsumer_->cycle();
  brain_->enforceNavStaleness();
  vectorProvider_->cycle();
  waypointProvider_->cycle();
}

void AutopilotApp::run() {
  running_ = true;
  const auto period = std::chrono::milliseconds(config_.loop.controlPeriodMs);
  UMAA_LOG_INFO(util::SYSTEM_LOGGER, "Autopilot control loop starting")
  while (running_) {
    step();
    std::this_thread::sleep_for(period);
  }
  UMAA_LOG_INFO(util::SYSTEM_LOGGER, "Autopilot control loop stopped")
}

void AutopilotApp::stop() {
  running_ = false;
  if (vehicle_) {
    vehicle_->shutdown();
  }
}

}  // namespace arlcore::autopilot
