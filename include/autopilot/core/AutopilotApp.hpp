#ifndef AUTOPILOT_CORE_AUTOPILOTAPP_HPP_
#define AUTOPILOT_CORE_AUTOPILOTAPP_HPP_

#include <UMAA/EO/UVPlatformSpecs/UVPlatformCapabilitiesReportType.hpp>
#include <UMAA/EO/UVPlatformSpecs/UVPlatformSpecsReportType.hpp>
#include <UMAA/MM/OperationalModeStatus/OperationalModeReportType.hpp>
#include <atomic>
#include <chrono>
#include <dds/dds.hpp>
#include <memory>

#include "ActiveConstraintsControlProvider.h"
#include "ConditionalAddProvider.h"
#include "ConditionalDeleteProvider.h"
#include "ConditionalFactory.h"
#include "ConditionalFactoryIo.h"
#include "ConditionalReportConsumer.h"
#include "ConditionalReportProvider.h"
#include "ConditionalReportProviderIo.h"
#include "GlobalPoseReportConsumer.h"
#include "ReportProvider.h"
#include "SpeedReportConsumer.h"
#include "VelocityReportConsumer.h"
#include "autopilot/config/AutopilotConfig.hpp"
#include "autopilot/core/AutopilotBrain.hpp"
#include "autopilot/core/NavState.hpp"
#include "autopilot/modes/OperationalModeControlProvider.hpp"
#include "autopilot/modes/OperationalModeManager.hpp"
#include "autopilot/safety/ConstraintSupervisor.hpp"
#include "autopilot/safety/SafeReturnPath.hpp"
#include "autopilot/safety/ZoneMap.hpp"
#include "autopilot/umaa/GlobalPoseObserver.hpp"
#include "autopilot/umaa/SpeedObserver.hpp"
#include "autopilot/umaa/VectorControlServiceProvider.hpp"
#include "autopilot/umaa/VelocityObserver.hpp"
#include "autopilot/umaa/WaypointControlServiceProvider.hpp"
#include "autopilot/vehicle/IVehicleControl.hpp"

namespace arlcore::autopilot {

//! \brief Top-level application: aggregates the DDS participant, UMAA consumers/providers,
//! brain, and vehicle-control strategy, and runs the single control loop.
class AutopilotApp {
 public:
  AutopilotApp() = default;

  //! \brief Wire up all components from configuration and publish the platform reports once.
  //! \return true on success
  bool initialize(const AutopilotConfig& config);

  //! \brief Run the control loop until stop() is called (or SIGINT).
  void run();

  //! \brief Request the control loop to exit.
  void stop();

  //! \brief Execute a single control iteration; exposed for deterministic testing.
  void step();

 private:
  AutopilotConfig config_;

  dds::domain::DomainParticipant participant_ = dds::core::null;

  NavState navState_;
  std::unique_ptr<IVehicleControl> vehicle_;
  std::unique_ptr<AutopilotBrain> brain_;

  std::shared_ptr<arlcore::umaa::services::GlobalPoseReportConsumer> poseConsumer_;
  std::shared_ptr<arlcore::umaa::services::SpeedReportConsumer> speedConsumer_;
  std::shared_ptr<arlcore::umaa::services::VelocityReportConsumer> velocityConsumer_;

  std::shared_ptr<GlobalPoseObserver> poseObserver_;
  std::shared_ptr<SpeedObserver> speedObserver_;
  std::shared_ptr<VelocityObserver> velocityObserver_;

  std::unique_ptr<VectorControlServiceProvider> vectorProvider_;
  std::unique_ptr<WaypointControlServiceProvider> waypointProvider_;

  std::unique_ptr<arlcore::umaa::services::ReportProvider<UMAA::EO::UVPlatformSpecs::UVPlatformSpecsReportType>>
      specsReportProvider_;
  std::unique_ptr<arlcore::umaa::services::ReportProvider<UMAA::EO::UVPlatformSpecs::UVPlatformCapabilitiesReportType>>
      capabilitiesReportProvider_;

  // MM operational mode services (constructed only when
  // identity.operational_mode_control_source_id is set).
  std::unique_ptr<OperationalModeManager> modeManager_;
  std::unique_ptr<OperationalModeControlProvider> operationalModeProvider_;
  std::unique_ptr<arlcore::umaa::services::ReportProvider<UMAA::MM::OperationalModeStatus::OperationalModeReportType>>
      operationalModeReportProvider_;
  OperationalMode lastReportedMode_ = OperationalMode::STANDBY;
  std::chrono::steady_clock::time_point lastModeReportAt_{};

  // MM constraint services (constructed only when identity.constraints_source_id is set); the
  // autopilot consumes its own published ConditionalReport over DDS loopback so every peer
  // sees the same single source of truth.
  std::shared_ptr<arlcore::umaa::conditional::ConditionalReportProvider> conditionalReportProvider_;
  std::shared_ptr<arlcore::umaa::ConditionalFactoryIo> conditionalFactoryIo_;
  std::shared_ptr<arlcore::umaa::conditional::ConditionalFactory> conditionalFactory_;
  std::shared_ptr<arlcore::umaa::conditional::ConditionalReportConsumer> conditionalReportConsumer_;
  std::shared_ptr<arlcore::umaa::conditional::ActiveConstraintsControlProvider> activeConstraintsProvider_;
  std::unique_ptr<arlcore::umaa::conditional::ConditionalAddProvider> conditionalAddProvider_;
  std::unique_ptr<arlcore::umaa::conditional::ConditionalDeleteProvider> conditionalDeleteProvider_;
  std::unique_ptr<ZoneMap> zoneMap_;
  std::shared_ptr<ConstraintSupervisor> supervisor_;
  std::optional<SafeReturnPath> safeReturnPath_;

  std::atomic<bool> stopRequested_{false};

  //! \brief Construct the MM conditional/constraint services and wire the supervisor.
  bool initializeConstraintServices();

  //! \brief Construct the MM operational mode manager/provider/report and publish the initial
  //! mode from the first manual poll.
  bool initializeOperationalModeServices();

  //! \brief Publish the OperationalModeStatus report for a mode change.
  void publishOperationalMode(OperationalMode mode);

  //! \brief Whether either driving provider has a non-terminal session of this class (feeds
  //! the manager's idle-revert; sessions of the other class never affect it).
  bool commandClassActive(CommandClass cls) const;
};

}  // namespace arlcore::autopilot
#endif  // AUTOPILOT_CORE_AUTOPILOTAPP_HPP_
