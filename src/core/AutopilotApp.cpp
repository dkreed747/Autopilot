#include "autopilot/core/AutopilotApp.hpp"

#include <UMAA/SA/GlobalPoseStatus/GlobalPoseReportType.hpp>
#include <UMAA/SA/SpeedStatus/SpeedReportType.hpp>
#include <UMAA/SA/VelocityStatus/VelocityReportType.hpp>
#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "CycloneQosProviderWrapper.h"
#include "CycloneReader.h"
#include "CycloneSender.h"
#include "CycloneUtilities.h"
#include "InternalTypes.h"
#include "Logger.h"
#include "UuidFactory.h"
#include "autopilot/config/ConfigValidation.hpp"
#include "autopilot/safety/SafeModeStrategyFactory.hpp"
#include "autopilot/umaa/PlatformReportFactory.hpp"
#include "autopilot/vehicle/VehicleControlFactory.hpp"
#include "autopilot/vehicle/VehicleControlTypes.hpp"

namespace arlcore::autopilot {

using arlcore::io::CycloneReader;
using arlcore::io::CycloneSender;
using arlcore::umaa::services::GlobalPoseReportConsumer;
using arlcore::umaa::services::ReportProvider;
using arlcore::umaa::services::SpeedReportConsumer;
using arlcore::umaa::services::VelocityReportConsumer;

static arlcore::NumericGuid parseId(const std::string& uuid) {
  return arlcore::UuidFactory::getInstance().parseGuidFromString(uuid);
}

//! \brief A source ID must be a well-formed UUID string; anything else would silently
//! produce a garbage GUID and commands addressed to the configured ID would never match.
static bool validSourceId(const std::string& uuid, const char* name) {
  if (isValidUuid(uuid)) {
    return true;
  }
  UMAA_LOG_ERROR(util::SYSTEM_LOGGER, "identity." << name << " is not a valid UUID: '" << uuid << "'")
  return false;
}

bool AutopilotApp::initialize(const AutopilotConfig& config) {
  config_ = config;

  if (!validSourceId(config_.identity.platformId, "platform_id") ||
      !validSourceId(config_.identity.vectorSourceId, "vector_source_id") ||
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
    UMAA_LOG_ERROR(util::SYSTEM_LOGGER,
                   "platform_capabilities.surface must define "
                   "cruising/max forward speed and a positive max_turn_rate_rps (they drive the planner)")
    return false;
  }

  // Checked before any DDS resource is acquired: a config naming a strategy that does not exist
  // must fail immediately rather than after joining a domain, and the old behaviour of warning and
  // silently driving the simulator is the failure this refuses.
  if (!isKnownVehicleControlType(config_.vehicleControlType)) {
    UMAA_LOG_ERROR(util::SYSTEM_LOGGER, "vehicle_control.type '" << config_.vehicleControlType
                                                                 << "' is unknown; expected one of "
                                                                 << joinVehicleControlTypes())
    return false;
  }

  participant_ = arlcore::io::getDomainParticipant(config_.dds.domainId);

  // Honor the configured UMAA QoS profiles (reliable/transient-local); the wrapper falls back
  // to defaults when the file or profile cannot be resolved.
  arlcore::io::CycloneQosProviderWrapper qosProvider(config_.dds.qosFile, config_.dds.domainQosProfile);
  const auto rqos = qosProvider.datareader_qos();
  const auto wqos = qosProvider.datawriter_qos();
  const auto largeListRqos = qosProvider.datareader_qos(config_.dds.largeCollectionsQosProfile);

  // Vehicle-control strategy. The nav senders are only used by a strategy that is itself the
  // platform's navigation source; see NavReportSenders and docs/adding-a-vehicle.md.
  NavReportSenders navSenders;
  navSenders.pose = std::make_shared<CycloneSender<UMAA::SA::GlobalPoseStatus::GlobalPoseReportType>>(
      participant_, UMAA::SA::GlobalPoseStatus::GlobalPoseReportTypeTopic, wqos);
  navSenders.speed = std::make_shared<CycloneSender<UMAA::SA::SpeedStatus::SpeedReportType>>(
      participant_, UMAA::SA::SpeedStatus::SpeedReportTypeTopic, wqos);
  navSenders.velocity = std::make_shared<CycloneSender<UMAA::SA::VelocityStatus::VelocityReportType>>(
      participant_, UMAA::SA::VelocityStatus::VelocityReportTypeTopic, wqos);
  vehicle_ = makeVehicleControl(config_, navSenders);
  if (vehicle_ == nullptr) {
    return false;  // the factory logged the type it could not build
  }
  if (!vehicle_->initialize()) {
    UMAA_LOG_ERROR(util::SYSTEM_LOGGER, "Vehicle control failed to initialize")
    return false;
  }

  brain_ = std::make_unique<AutopilotBrain>(&navState_, vehicle_.get(), config_);

  // Navigation consumers (listeners)
  poseConsumer_ = std::make_shared<GlobalPoseReportConsumer>(
      std::make_shared<CycloneReader<UMAA::SA::GlobalPoseStatus::GlobalPoseReportType>>(
          participant_, UMAA::SA::GlobalPoseStatus::GlobalPoseReportTypeTopic, rqos));
  speedConsumer_ =
      std::make_shared<SpeedReportConsumer>(std::make_shared<CycloneReader<UMAA::SA::SpeedStatus::SpeedReportType>>(
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

  // MM constraint services first: the command providers take the supervisor's safety gate and
  // the shared zone map at construction.
  if (!initializeConstraintServices()) {
    return false;
  }

  // MM operational mode services next: the command providers take the mode gate at
  // construction too.
  if (!initializeOperationalModeServices()) {
    return false;
  }

  const flt64_t maxForwardSpeed = config_.platformCapabilities.surface.maxForwardSpeedMps.value_or(0.0);

  ElevationAdmissionLimits elevationLimits;
  elevationLimits.maxDepthM = config_.constraints.maxDepthM;
  elevationLimits.minAltitudeAsfM = config_.constraints.minAltitudeAsfM;
  elevationLimits.platformReportsAsf = config_.platformCapabilities.reportsAltitudeAsf;

  // Vector control provider
  auto vectorIo = std::make_shared<VectorControlServiceProviderIo>(
      std::make_shared<CycloneReader<GlobalVectorCommandType>>(
          participant_, UMAA::MO::GlobalVectorControl::GlobalVectorCommandTypeTopic, rqos),
      std::make_shared<CycloneSender<GlobalVectorCommandAckReportType>>(
          participant_, UMAA::MO::GlobalVectorControl::GlobalVectorCommandAckReportTypeTopic, wqos),
      std::make_shared<CycloneSender<GlobalVectorCommandStatusType>>(
          participant_, UMAA::MO::GlobalVectorControl::GlobalVectorCommandStatusTypeTopic, wqos),
      std::make_shared<CycloneSender<GlobalVectorExecutionStatusReportType>>(
          participant_, UMAA::MO::GlobalVectorControl::GlobalVectorExecutionStatusReportTypeTopic, wqos));
  vectorProvider_ = std::make_unique<VectorControlServiceProvider>(parseId(config_.identity.vectorSourceId), vectorIo,
                                                                   brain_.get(), maxForwardSpeed, supervisor_.get(),
                                                                   modeManager_.get(), brain_.get());
  vectorProvider_->setElevationAdmissionLimits(elevationLimits);

  // Waypoint control provider (with large-list element reader)
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
      config_.planner.maxListWaitCycles, supervisor_.get(), zoneMap_.get(), modeManager_.get());
  waypointProvider_->setElevationAdmissionLimits(elevationLimits);

  // Platform report providers: publish specs + capabilities once on startup
  specsReportProvider_ = std::make_unique<ReportProvider<UMAA::EO::UVPlatformSpecs::UVPlatformSpecsReportType>>(
      parseId(config_.identity.specsSourceId),
      std::make_shared<CycloneSender<UMAA::EO::UVPlatformSpecs::UVPlatformSpecsReportType>>(
          participant_, UMAA::EO::UVPlatformSpecs::UVPlatformSpecsReportTypeTopic, wqos));
  capabilitiesReportProvider_ =
      std::make_unique<ReportProvider<UMAA::EO::UVPlatformSpecs::UVPlatformCapabilitiesReportType>>(
          parseId(config_.identity.capabilitiesSourceId),
          std::make_shared<CycloneSender<UMAA::EO::UVPlatformSpecs::UVPlatformCapabilitiesReportType>>(
              participant_, UMAA::EO::UVPlatformSpecs::UVPlatformCapabilitiesReportTypeTopic, wqos));

  auto specs = makePlatformSpecsReport(config_.platformSpecs);
  auto capabilities = makePlatformCapabilitiesReport(config_.platformCapabilities);
  specsReportProvider_->send(&specs);
  capabilitiesReportProvider_->send(&capabilities);

  UMAA_LOG_INFO(util::SYSTEM_LOGGER, "Autopilot initialized on domain " << config_.dds.domainId)
  return true;
}

bool AutopilotApp::initializeConstraintServices() {
  if (config_.identity.constraintsSourceId.empty()) {
    UMAA_LOG_INFO(util::SYSTEM_LOGGER,
                  "identity.constraints_source_id not set; MM constraint "
                  "services disabled")
    return true;
  }
  if (!validSourceId(config_.identity.constraintsSourceId, "constraints_source_id")) {
    return false;
  }
  const arlcore::NumericGuid constraintsId = parseId(config_.identity.constraintsSourceId);

  arlcore::io::CycloneQosProviderWrapper qosProvider(config_.dds.qosFile, config_.dds.domainQosProfile);
  const auto rqos = qosProvider.datareader_qos();
  const auto wqos = qosProvider.datawriter_qos();
  const auto largeRqos = qosProvider.datareader_qos(config_.dds.largeCollectionsQosProfile);
  const auto largeWqos = qosProvider.datawriter_qos(config_.dds.largeCollectionsQosProfile);

  namespace cond = UMAA::MM::Conditional;
  namespace condReport = UMAA::MM::ConditionalReport;
  namespace condControl = UMAA::MM::ConditionalControl;
  namespace acControl = UMAA::MM::ActiveConstraintsControl;

  // Conditional report provider: owns the working conditional set and every payload topic
  auto reportIo = std::make_shared<arlcore::umaa::ConditionalReportProviderIo>(
      std::make_shared<CycloneSender<condReport::ConditionalReportType>>(participant_,
                                                                         condReport::ConditionalReportTypeTopic, wqos),
      std::make_shared<CycloneSender<condReport::ConditionalReportTypeConditionalsSetElement>>(
          participant_, condReport::ConditionalReportTypeConditionalsSetElementTopic, largeWqos),
      std::make_shared<CycloneSender<cond::ConstraintViolatedConditionalType>>(
          participant_, cond::ConstraintViolatedConditionalTypeTopic, wqos),
      std::make_shared<CycloneSender<cond::DepthConditionalType>>(participant_, cond::DepthConditionalTypeTopic, wqos),
      std::make_shared<CycloneSender<cond::DepthRateConditionalType>>(participant_, cond::DepthRateConditionalTypeTopic,
                                                                      wqos),
      std::make_shared<CycloneSender<cond::EmitterPresetConditionalType>>(
          participant_, cond::EmitterPresetConditionalTypeTopic, wqos),
      std::make_shared<CycloneSender<cond::ExpConditionalType>>(participant_, cond::ExpConditionalTypeTopic, wqos),
      std::make_shared<CycloneSender<cond::HeadingSectorConditionalType>>(
          participant_, cond::HeadingSectorConditionalTypeTopic, wqos),
      std::make_shared<CycloneSender<cond::LogicalANDConditionalType>>(participant_,
                                                                       cond::LogicalANDConditionalTypeTopic, wqos),
      std::make_shared<CycloneSender<cond::LogicalNOTConditionalType>>(participant_,
                                                                       cond::LogicalNOTConditionalTypeTopic, wqos),
      std::make_shared<CycloneSender<cond::LogicalORConditionalType>>(participant_, cond::LogicalORConditionalTypeTopic,
                                                                      wqos),
      std::make_shared<CycloneSender<cond::MissionStateConditionalType>>(participant_,
                                                                         cond::MissionStateConditionalTypeTopic, wqos),
      std::make_shared<CycloneSender<cond::ObjectiveStateConditionalType>>(
          participant_, cond::ObjectiveStateConditionalTypeTopic, wqos),
      std::make_shared<CycloneSender<cond::PitchRateConditionalType>>(participant_, cond::PitchRateConditionalTypeTopic,
                                                                      wqos),
      std::make_shared<CycloneSender<cond::RelativeSpeedConditionalType>>(
          participant_, cond::RelativeSpeedConditionalTypeTopic, wqos),
      std::make_shared<CycloneSender<cond::RollRateConditionalType>>(participant_, cond::RollRateConditionalTypeTopic,
                                                                     wqos),
      std::make_shared<CycloneSender<cond::SpeedConditionalType>>(participant_, cond::SpeedConditionalTypeTopic, wqos),
      std::make_shared<CycloneSender<cond::TaskStateConditionalType>>(participant_, cond::TaskStateConditionalTypeTopic,
                                                                      wqos),
      std::make_shared<CycloneSender<cond::TimeConditionalType>>(participant_, cond::TimeConditionalTypeTopic, wqos),
      std::make_shared<CycloneSender<cond::WaterZoneConditionalType>>(participant_, cond::WaterZoneConditionalTypeTopic,
                                                                      wqos),
      std::make_shared<CycloneSender<cond::YawRateConditionalType>>(participant_, cond::YawRateConditionalTypeTopic,
                                                                    wqos));
  conditionalReportProvider_ =
      std::make_shared<arlcore::umaa::conditional::ConditionalReportProvider>(constraintsId, reportIo);

  // Factory io: shares the SA nav consumers; the specialization readers see both the
  // console-published payloads and our own re-published instances (loopback)
  conditionalFactoryIo_ = std::make_shared<arlcore::umaa::ConditionalFactoryIo>(
      poseConsumer_, speedConsumer_, velocityConsumer_,
      std::make_shared<CycloneReader<cond::ConstraintViolatedConditionalType>>(
          participant_, cond::ConstraintViolatedConditionalTypeTopic, rqos),
      std::make_shared<CycloneReader<cond::DepthConditionalType>>(participant_, cond::DepthConditionalTypeTopic, rqos),
      std::make_shared<CycloneReader<cond::DepthRateConditionalType>>(participant_, cond::DepthRateConditionalTypeTopic,
                                                                      rqos),
      std::make_shared<CycloneReader<cond::EmitterPresetConditionalType>>(
          participant_, cond::EmitterPresetConditionalTypeTopic, rqos),
      std::make_shared<CycloneReader<cond::ExpConditionalType>>(participant_, cond::ExpConditionalTypeTopic, rqos),
      std::make_shared<CycloneReader<cond::HeadingSectorConditionalType>>(
          participant_, cond::HeadingSectorConditionalTypeTopic, rqos),
      std::make_shared<CycloneReader<cond::LogicalANDConditionalType>>(participant_,
                                                                       cond::LogicalANDConditionalTypeTopic, rqos),
      std::make_shared<CycloneReader<cond::LogicalNOTConditionalType>>(participant_,
                                                                       cond::LogicalNOTConditionalTypeTopic, rqos),
      std::make_shared<CycloneReader<cond::LogicalORConditionalType>>(participant_, cond::LogicalORConditionalTypeTopic,
                                                                      rqos),
      std::make_shared<CycloneReader<cond::MissionStateConditionalType>>(participant_,
                                                                         cond::MissionStateConditionalTypeTopic, rqos),
      std::make_shared<CycloneReader<cond::ObjectiveStateConditionalType>>(
          participant_, cond::ObjectiveStateConditionalTypeTopic, rqos),
      std::make_shared<CycloneReader<cond::PitchRateConditionalType>>(participant_, cond::PitchRateConditionalTypeTopic,
                                                                      rqos),
      std::make_shared<CycloneReader<cond::RelativeSpeedConditionalType>>(
          participant_, cond::RelativeSpeedConditionalTypeTopic, rqos),
      std::make_shared<CycloneReader<cond::RollRateConditionalType>>(participant_, cond::RollRateConditionalTypeTopic,
                                                                     rqos),
      std::make_shared<CycloneReader<cond::SpeedConditionalType>>(participant_, cond::SpeedConditionalTypeTopic, rqos),
      std::make_shared<CycloneReader<cond::TaskStateConditionalType>>(participant_, cond::TaskStateConditionalTypeTopic,
                                                                      rqos),
      std::make_shared<CycloneReader<cond::TimeConditionalType>>(participant_, cond::TimeConditionalTypeTopic, rqos),
      std::make_shared<CycloneReader<cond::WaterZoneConditionalType>>(participant_, cond::WaterZoneConditionalTypeTopic,
                                                                      rqos),
      std::make_shared<CycloneReader<cond::YawRateConditionalType>>(participant_, cond::YawRateConditionalTypeTopic,
                                                                    rqos));
  conditionalFactory_ = std::make_shared<arlcore::umaa::conditional::ConditionalFactory>(conditionalFactoryIo_);

  // Loopback read-back of our own report; the consumer resolves evaluable conditionals
  conditionalReportConsumer_ = std::make_shared<arlcore::umaa::conditional::ConditionalReportConsumer>(
      std::make_shared<CycloneReader<condReport::ConditionalReportType>>(participant_,
                                                                         condReport::ConditionalReportTypeTopic, rqos),
      std::make_shared<CycloneReader<condReport::ConditionalReportTypeConditionalsSetElement>>(
          participant_, condReport::ConditionalReportTypeConditionalsSetElementTopic, largeRqos),
      conditionalFactory_);

  // Standing ActiveConstraints provider: the applied set outlives any one commander
  auto activeIo = std::make_shared<ActiveConstraintsControlProviderIo>(
      std::make_shared<CycloneReader<acControl::ActiveConstraintsCommandType>>(
          participant_, acControl::ActiveConstraintsCommandTypeTopic, rqos),
      std::make_shared<CycloneSender<acControl::ActiveConstraintsCommandAckReportType>>(
          participant_, acControl::ActiveConstraintsCommandAckReportTypeTopic, wqos),
      std::make_shared<CycloneSender<acControl::ActiveConstraintsCommandStatusType>>(
          participant_, acControl::ActiveConstraintsCommandStatusTypeTopic, wqos));
  activeConstraintsProvider_ = std::make_shared<arlcore::umaa::conditional::ActiveConstraintsControlProvider>(
      constraintsId, activeIo, /*standingSession=*/true);
  conditionalReportConsumer_->registerObserver(activeConstraintsProvider_);

  // ConditionalControl Add/Delete providers (supported subset: water zone, speed, depth)
  auto addIo = std::make_shared<ConditionalAddProviderIo>(
      std::make_shared<CycloneReader<condControl::ConditionalAddCommandType>>(
          participant_, condControl::ConditionalAddCommandTypeTopic, rqos),
      std::make_shared<CycloneSender<condControl::ConditionalAddCommandAckReportType>>(
          participant_, condControl::ConditionalAddCommandAckReportTypeTopic, wqos),
      std::make_shared<CycloneSender<condControl::ConditionalAddCommandStatusType>>(
          participant_, condControl::ConditionalAddCommandStatusTypeTopic, wqos));
  conditionalAddProvider_ = std::make_unique<arlcore::umaa::conditional::ConditionalAddProvider>(
      constraintsId, addIo, conditionalReportProvider_, conditionalFactoryIo_,
      std::set<std::string>{cond::WaterZoneConditionalTypeTopic, cond::SpeedConditionalTypeTopic,
                            cond::DepthConditionalTypeTopic},
      static_cast<uint32_t>(config_.planner.maxListWaitCycles));

  auto deleteIo = std::make_shared<ConditionalDeleteProviderIo>(
      std::make_shared<CycloneReader<condControl::ConditionalDeleteCommandType>>(
          participant_, condControl::ConditionalDeleteCommandTypeTopic, rqos),
      std::make_shared<CycloneSender<condControl::ConditionalDeleteCommandAckReportType>>(
          participant_, condControl::ConditionalDeleteCommandAckReportTypeTopic, wqos),
      std::make_shared<CycloneSender<condControl::ConditionalDeleteCommandStatusType>>(
          participant_, condControl::ConditionalDeleteCommandStatusTypeTopic, wqos));
  conditionalDeleteProvider_ = std::make_unique<arlcore::umaa::conditional::ConditionalDeleteProvider>(
      constraintsId, deleteIo, conditionalReportProvider_);

  // Supervisor: snapshot authority, zone map feed, state reports, safety gate
  zoneMap_ = std::make_unique<ZoneMap>(config_.zones);
  auto stateSender = std::make_shared<CycloneSender<UMAA::MM::ConditionalStateReport::ConditionalStateReportType>>(
      participant_, UMAA::MM::ConditionalStateReport::ConditionalStateReportTypeTopic, wqos);
  supervisor_ = std::make_shared<ConstraintSupervisor>(config_, &navState_, zoneMap_.get(), stateSender, constraintsId);
  conditionalReportConsumer_->registerObserver(supervisor_->conditionalSetObserver());
  activeConstraintsProvider_->registerObserver(supervisor_->activeSetObserver());
  brain_->setConstraintSource(supervisor_.get());
  brain_->setZoneMap(zoneMap_.get());

  // Safe Return Path: loaded and validated at startup, executed by the SRP strategy
  std::string srpError;
  safeReturnPath_ = SafeReturnPath::load(config_.safety.safeMode.srp, &srpError);
  if (!srpError.empty()) {
    // An invalid SRP configuration is always a startup error when the SRP strategy was
    // requested: silently continuing without the safety artifact would be worse than refusing
    // to start.
    if (config_.safety.safeMode.strategy == "srp") {
      UMAA_LOG_ERROR(util::SYSTEM_LOGGER, "safe_mode.strategy is 'srp' but the SRP is invalid: " << srpError)
      return false;
    }
    UMAA_LOG_WARN(util::SYSTEM_LOGGER, "Ignoring invalid SRP configuration (strategy is '"
                                           << config_.safety.safeMode.strategy << "'): " << srpError)
  }
  if (config_.safety.safeMode.strategy == "srp" && !safeReturnPath_.has_value()) {
    UMAA_LOG_INFO(util::SYSTEM_LOGGER,
                  "safe_mode.strategy is 'srp' with no CSV configured; "
                  "falling back to zero-speed hold")
  }

  // Arm the violation FSM with the configured safe-mode strategy.
  supervisor_->attachSafety(brain_.get(), makeSafeModeStrategy(config_.safety, safeReturnPath_));

  UMAA_LOG_INFO(util::SYSTEM_LOGGER,
                "MM constraint services initialized (source " << config_.identity.constraintsSourceId << ")")
  return true;
}

bool AutopilotApp::initializeOperationalModeServices() {
  if (config_.identity.operationalModeControlSourceId.empty()) {
    UMAA_LOG_INFO(util::SYSTEM_LOGGER,
                  "identity.operational_mode_control_source_id not set; MM "
                  "operational mode services disabled")
    return true;
  }
  if (!validSourceId(config_.identity.operationalModeControlSourceId, "operational_mode_control_source_id") ||
      !validSourceId(config_.identity.operationalModeStatusSourceId, "operational_mode_status_source_id")) {
    // The pair is configured together: a control service without the status report is
    // UMAA-non-compliant, and a report without the control service is pointless.
    return false;
  }

  modeManager_ =
      std::make_unique<OperationalModeManager>(config_.operationalMode, parseId(config_.identity.platformId));

  arlcore::io::CycloneQosProviderWrapper qosProvider(config_.dds.qosFile, config_.dds.domainQosProfile);
  const auto rqos = qosProvider.datareader_qos();
  const auto wqos = qosProvider.datawriter_qos();

  operationalModeReportProvider_ =
      std::make_unique<ReportProvider<UMAA::MM::OperationalModeStatus::OperationalModeReportType>>(
          parseId(config_.identity.operationalModeStatusSourceId),
          std::make_shared<CycloneSender<UMAA::MM::OperationalModeStatus::OperationalModeReportType>>(
              participant_, UMAA::MM::OperationalModeStatus::OperationalModeReportTypeTopic, wqos));

  auto modeIo = std::make_shared<OperationalModeControlProviderIo>(
      std::make_shared<CycloneReader<OperationalModeCommandType>>(
          participant_, UMAA::MM::OperationalModeControl::OperationalModeCommandTypeTopic, rqos),
      std::make_shared<CycloneSender<OperationalModeCommandAckReportType>>(
          participant_, UMAA::MM::OperationalModeControl::OperationalModeCommandAckReportTypeTopic, wqos),
      std::make_shared<CycloneSender<OperationalModeCommandStatusType>>(
          participant_, UMAA::MM::OperationalModeControl::OperationalModeCommandStatusTypeTopic, wqos));
  operationalModeProvider_ = std::make_unique<OperationalModeControlProvider>(
      parseId(config_.identity.operationalModeControlSourceId), modeIo, modeManager_.get());

  modeManager_->setModeChangedCallback([this](OperationalMode mode) {
    publishOperationalMode(mode);
    if (vehicle_) {
      vehicle_->onOperationalModeChanged(mode);
    }
    // Leaving MANUAL lands in STANDBY and the human may have driven anywhere, so replan a
    // stale safe-mode plan; any other STANDBY entry must not disturb a running safe maneuver.
    if (supervisor_ && lastReportedMode_ == OperationalMode::MANUAL && mode == OperationalMode::STANDBY) {
      supervisor_->refreshSafeModePlan();
    }
    lastReportedMode_ = mode;
  });

  // The initial mode comes from the first manual poll (a hardware strategy may boot engaged;
  // the sim never does) and is published through the callback.
  modeManager_->beginStep(vehicle_->isManualEngaged());

  UMAA_LOG_INFO(util::SYSTEM_LOGGER, "MM operational mode services initialized (control source "
                                         << config_.identity.operationalModeControlSourceId << ")")
  return true;
}

void AutopilotApp::publishOperationalMode(OperationalMode mode) {
  if (!operationalModeReportProvider_) {
    return;
  }
  using OperationalModeEnumType = UMAA::Common::MaritimeEnumeration::OperationalModeEnumModule::OperationalModeEnumType;
  UMAA::MM::OperationalModeStatus::OperationalModeReportType report;
  switch (mode) {
    case OperationalMode::MANUAL:
      report.operationalMode() = OperationalModeEnumType::MANUAL;
      break;
    case OperationalMode::REMOTE:
      report.operationalMode() = OperationalModeEnumType::REMOTE;
      break;
    case OperationalMode::AUTONOMOUS:
      report.operationalMode() = OperationalModeEnumType::AUTONOMOUS;
      break;
    case OperationalMode::STANDBY:
    default:
      report.operationalMode() = OperationalModeEnumType::STANDBY;
      break;
  }
  // ReportProvider::send stamps only source().id(); parentID identifies the platform.
  report.source().parentID(parseId(config_.identity.platformId).getGuid());
  operationalModeReportProvider_->send(&report);
  lastModeReportAt_ = std::chrono::steady_clock::now();
}

bool AutopilotApp::commandClassActive(CommandClass cls) const {
  if (!modeManager_) {
    return false;
  }
  // Any session still tracked by a provider is non-terminal (terminal sessions are reaped
  // within the provider's cycle), including held-at-ISSUED sessions of the other class.
  if (vectorProvider_) {
    for (const auto& cmd : vectorProvider_->getActiveCommands()) {
      if (modeManager_->classify(cmd.source()) == cls) {
        return true;
      }
    }
  }
  if (waypointProvider_) {
    for (const auto& cmd : waypointProvider_->getActiveCommands()) {
      if (modeManager_->classify(cmd.source()) == cls) {
        return true;
      }
    }
  }
  return false;
}

void AutopilotApp::step() {
  // Cycle nav consumers first; speed/velocity before pose so the pose-triggered control
  // recompute pairs the pose with the speed published in the same vehicle tick.
  speedConsumer_->cycle();
  velocityConsumer_->cycle();
  poseConsumer_->cycle();
  brain_->enforceNavStaleness();
  // Constraint pipeline: mutate the set (add/delete), read the report back over loopback,
  // resolve the active subset, then let the supervisor rebuild its snapshot — all before the
  // command providers cycle, so safety decisions are visible to them within the same tick.
  if (conditionalAddProvider_) {
    conditionalAddProvider_->cycle();
  }
  if (conditionalDeleteProvider_) {
    conditionalDeleteProvider_->cycle();
  }
  if (conditionalReportConsumer_) {
    conditionalReportConsumer_->cycle();
  }
  if (activeConstraintsProvider_) {
    activeConstraintsProvider_->cycle();
  }
  if (supervisor_) {
    supervisor_->update();
  }
  // Operational mode: manual edges then explicit mode commands first (manual wins every
  // same-tick race, providers admit against the freshest mode); idle-revert evaluates AFTER
  // the providers so a command arriving this very tick suppresses it.
  if (modeManager_) {
    modeManager_->beginStep(vehicle_->isManualEngaged());
  }
  if (operationalModeProvider_) {
    operationalModeProvider_->cycle();
  }
  vectorProvider_->cycle();
  waypointProvider_->cycle();
  if (modeManager_) {
    modeManager_->endStep(commandClassActive(CommandClass::LOCAL), commandClassActive(CommandClass::REMOTE));
    // Periodic republish (on top of on-change publication) so consumers can treat report
    // age as a liveness signal rather than "time since the last mode change".
    if (std::chrono::steady_clock::now() - lastModeReportAt_ >= std::chrono::seconds(1)) {
      publishOperationalMode(modeManager_->mode());
    }
  }
}

void AutopilotApp::run() {
  const auto period = std::chrono::milliseconds(config_.loop.controlPeriodMs);
  UMAA_LOG_INFO(util::SYSTEM_LOGGER, "Autopilot control loop starting")
  // Absolute scheduling (like the sim thread) so processing time does not stretch the
  // period; run() never writes the stop flag, so a signal delivered at startup cannot race.
  auto next = std::chrono::steady_clock::now() + period;
  while (!stopRequested_) {
    step();
    std::this_thread::sleep_until(next);
    next += period;
    const auto now = std::chrono::steady_clock::now();
    if (now > next) {
      next = now + period;  // a stalled tick (e.g. a long replan) resets rather than bursts
    }
  }
  // Vehicle teardown happens here on the loop thread, NOT in stop(): stop()
  // runs in signal-handler context on an arbitrary thread, and a repeated
  // SIGTERM delivered on the sim thread would make it join itself (EDEADLK).
  if (vehicle_) {
    vehicle_->shutdown();
  }
  UMAA_LOG_INFO(util::SYSTEM_LOGGER, "Autopilot control loop stopped")
}

void AutopilotApp::stop() { stopRequested_ = true; }

}  // namespace arlcore::autopilot
