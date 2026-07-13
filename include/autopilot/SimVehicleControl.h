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

#ifndef APPS_AUTOPILOT_INCLUDE_AUTOPILOT_SIMVEHICLECONTROL_H_
#define APPS_AUTOPILOT_INCLUDE_AUTOPILOT_SIMVEHICLECONTROL_H_

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>

#include <GeographicLib/LocalCartesian.hpp>

#include <UMAA/SA/GlobalPoseStatus/GlobalPoseReportType.hpp>
#include <UMAA/SA/SpeedStatus/SpeedReportType.hpp>
#include <UMAA/SA/VelocityStatus/VelocityReportType.hpp>

#include "AutopilotConfig.h"
#include "IVehicleControl.h"
#include "NumericGuid.h"
#include "ReportProvider.h"
#include "SenderBase.h"

namespace arlcore::autopilot {

//! \brief A simulated vehicle-control strategy. It keeps an internal kinematic vehicle whose
//! limits come from the configured platform capabilities (max forward/reverse speed, max turn
//! rate, max depth-change rate) and integrates it on its own thread at the configured cycle
//! rate, acting on the most recent control-vector setpoint. Every cycle it publishes the
//! three SA navigation reports — Global Pose, Speed, and Velocity — closing the control loop
//! for the autopilot exactly as a real vehicle's navigation suite would.
//!
//! The strategy is transport-agnostic: it takes the three report senders (DDS in the app,
//! LocalReaderSender in tests) and signs reports with the configured nav source ID.
class SimVehicleControl : public IVehicleControl {
 public:
  SimVehicleControl(const PlatformSpecsConfig& specs, const PlatformCapabilitiesConfig& caps,
                    const SimVehicleConfig& simConfig, const arlcore::NumericGuid& navSourceId,
                    std::shared_ptr<arlcore::io::SenderBase<UMAA::SA::GlobalPoseStatus::GlobalPoseReportType>>
                        poseSender,
                    std::shared_ptr<arlcore::io::SenderBase<UMAA::SA::SpeedStatus::SpeedReportType>> speedSender,
                    std::shared_ptr<arlcore::io::SenderBase<UMAA::SA::VelocityStatus::VelocityReportType>>
                        velocitySender);

  ~SimVehicleControl() override;

  bool initialize() override;
  void shutdown() override;
  bool sendControlVector(const ControlVector& cv) override;
  UMAA::EO::UVPlatformSpecs::UVPlatformSpecsReportType getPlatformSpecs() const override;
  UMAA::EO::UVPlatformSpecs::UVPlatformCapabilitiesReportType getPlatformCapabilities() const override;

  //! \brief The most recent control vector handed to the strategy (for tests/diagnostics).
  std::optional<ControlVector> lastControlVector() const;

  //! \brief Count of control vectors received (for tests/diagnostics).
  uint64_t controlVectorCount() const;

  //! \brief Snapshot of the simulated truth state (for tests/diagnostics).
  struct SimState {
    double latitudeDeg = 0.0;
    double longitudeDeg = 0.0;
    double headingRad = 0.0;
    double speedMps = 0.0;
    double depthM = 0.0;
    double yawRateRps = 0.0;
  };
  SimState state() const;

  //! \brief Advance the simulation by one step and publish the nav reports. Runs on the sim
  //! thread; public so deterministic tests can drive it directly without the thread.
  void stepOnce(double dtS);

 private:
  void runLoop();
  double maxTurnRateRps() const;
  double maxForwardSpeedMps() const;
  double maxReverseSpeedMps() const;
  double maxDepthRateMps() const;
  void publishReports();

  PlatformSpecsConfig specs_;
  PlatformCapabilitiesConfig caps_;
  SimVehicleConfig simConfig_;

  arlcore::umaa::services::ReportProvider<UMAA::SA::GlobalPoseStatus::GlobalPoseReportType> poseProvider_;
  arlcore::umaa::services::ReportProvider<UMAA::SA::SpeedStatus::SpeedReportType> speedProvider_;
  arlcore::umaa::services::ReportProvider<UMAA::SA::VelocityStatus::VelocityReportType> velocityProvider_;

  mutable std::mutex mtx_;
  std::optional<ControlVector> setpoint_;
  uint64_t controlVectorCount_ = 0;

  // Simulated truth state, integrated in a local tangent plane anchored at the initial fix.
  GeographicLib::LocalCartesian frame_;
  double xEastM_ = 0.0;
  double yNorthM_ = 0.0;
  double headingRad_ = 0.0;
  double speedMps_ = 0.0;
  double depthM_ = 0.0;
  double yawRateRps_ = 0.0;

  std::thread simThread_;
  std::atomic<bool> running_{false};
};

}  // namespace arlcore::autopilot
#endif  // APPS_AUTOPILOT_INCLUDE_AUTOPILOT_SIMVEHICLECONTROL_H_
