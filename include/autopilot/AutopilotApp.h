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

#ifndef APPS_AUTOPILOT_INCLUDE_AUTOPILOT_AUTOPILOTAPP_H_
#define APPS_AUTOPILOT_INCLUDE_AUTOPILOT_AUTOPILOTAPP_H_

#include <atomic>
#include <memory>

#include <dds/dds.hpp>

#include <UMAA/EO/UVPlatformSpecs/UVPlatformSpecsReportType.hpp>
#include <UMAA/EO/UVPlatformSpecs/UVPlatformCapabilitiesReportType.hpp>

#include "ActiveConstraintsControlProvider.h"
#include "AutopilotBrain.h"
#include "AutopilotConfig.h"
#include "ConditionalAddProvider.h"
#include "ConditionalDeleteProvider.h"
#include "ConditionalFactory.h"
#include "ConditionalFactoryIo.h"
#include "ConditionalReportConsumer.h"
#include "ConditionalReportProvider.h"
#include "ConditionalReportProviderIo.h"
#include "ConstraintSupervisor.h"
#include "GlobalPoseReportConsumer.h"
#include "NavObservers.h"
#include "NavState.h"
#include "ReportProvider.h"
#include "SafeReturnPath.h"
#include "SimVehicleControl.h"
#include "SpeedReportConsumer.h"
#include "VectorControlServiceProvider.h"
#include "VelocityReportConsumer.h"
#include "WaypointControlServiceProvider.h"
#include "ZoneMap.h"

namespace arlcore::autopilot {

//! \brief Top-level autopilot application. Aggregates the DDS participant, the three SA nav
//! consumers (driven as listeners), the two MO command providers, the autopilot brain, the
//! configured vehicle-control strategy, and the platform report providers. Runs a single
//! control loop.
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

  //! \brief Execute a single control iteration (cycle nav consumers then providers). Exposed
  //! for deterministic testing.
  void step();

 private:
  AutopilotConfig config_;

  dds::domain::DomainParticipant participant_ = dds::core::null;
  dds::sub::Subscriber subscriber_ = dds::core::null;
  dds::pub::Publisher publisher_ = dds::core::null;

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

  std::unique_ptr<arlcore::umaa::services::ReportProvider<
      UMAA::EO::UVPlatformSpecs::UVPlatformSpecsReportType>> specsReportProvider_;
  std::unique_ptr<arlcore::umaa::services::ReportProvider<
      UMAA::EO::UVPlatformSpecs::UVPlatformCapabilitiesReportType>> capabilitiesReportProvider_;

  // MM constraint services (constructed only when identity.constraints_source_id is set).
  // The autopilot both provides AND consumes its own ConditionalReport over DDS loopback:
  // the published report is the single source of truth every peer (console included) sees.
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

  std::atomic<bool> running_{false};

  //! \brief Construct the MM conditional/constraint services and wire the supervisor.
  bool initializeConstraintServices();
};

}  // namespace arlcore::autopilot
#endif  // APPS_AUTOPILOT_INCLUDE_AUTOPILOT_AUTOPILOTAPP_H_
