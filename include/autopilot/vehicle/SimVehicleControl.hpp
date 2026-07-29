#ifndef AUTOPILOT_VEHICLE_SIMVEHICLECONTROL_HPP_
#define AUTOPILOT_VEHICLE_SIMVEHICLECONTROL_HPP_

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

#include "autopilot/config/AutopilotConfig.hpp"
#include "autopilot/vehicle/IVehicleControl.hpp"
#include "NumericGuid.h"
#include "ReportProvider.h"
#include "SenderBase.h"
#include "InternalTypes.h"

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
  SimVehicleControl(const PlatformCapabilitiesConfig& caps,
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

  //! \brief The sim platform never engages manual control; it always boots into STANDBY.
  bool isManualEngaged() const override { return false; }

  //! \brief The most recent control vector handed to the strategy (for tests/diagnostics).
  std::optional<ControlVector> lastControlVector() const;

  //! \brief Count of control vectors received (for tests/diagnostics).
  uint64_t controlVectorCount() const;

  //! \brief Snapshot of the simulated truth state (for tests/diagnostics).
  struct SimState {
    flt64_t latitudeDeg = 0.0;
    flt64_t longitudeDeg = 0.0;
    flt64_t headingRad = 0.0;
    flt64_t speedMps = 0.0;
    flt64_t depthM = 0.0;
    flt64_t yawRateRps = 0.0;
  };
  SimState state() const;

  //! \brief Advance the simulation by one step and publish the nav reports. Runs on the sim
  //! thread; public so deterministic tests can drive it directly without the thread.
  void stepOnce(flt64_t dtS);

 private:
  void runLoop();
  flt64_t maxTurnRateRps() const;
  flt64_t maxForwardSpeedMps() const;
  flt64_t maxReverseSpeedMps() const;
  flt64_t maxDepthRateMps() const;
  void publishReports();

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
  flt64_t xEastM_ = 0.0;
  flt64_t yNorthM_ = 0.0;
  flt64_t headingRad_ = 0.0;
  flt64_t speedMps_ = 0.0;
  flt64_t depthM_ = 0.0;
  flt64_t yawRateRps_ = 0.0;

  std::thread simThread_;
  std::atomic<bool> running_{false};
};

}  // namespace arlcore::autopilot
#endif  // AUTOPILOT_VEHICLE_SIMVEHICLECONTROL_HPP_
