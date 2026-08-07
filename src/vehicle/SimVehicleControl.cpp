#include "autopilot/vehicle/SimVehicleControl.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <utility>

#include "InternalTypes.h"
#include "Logger.h"
#include "autopilot/guidance/AngleMath.hpp"

namespace arlcore::autopilot {

using UMAA::SA::GlobalPoseStatus::GlobalPoseReportType;
using UMAA::SA::SpeedStatus::SpeedReportType;
using UMAA::SA::VelocityStatus::VelocityReportType;

SimVehicleControl::SimVehicleControl(const PlatformCapabilitiesConfig& caps, const SimVehicleConfig& simConfig,
                                     const arlcore::NumericGuid& navSourceId,
                                     const arlcore::NumericGuid& platformId,
                                     std::shared_ptr<arlcore::io::SenderBase<GlobalPoseReportType>> poseSender,
                                     std::shared_ptr<arlcore::io::SenderBase<SpeedReportType>> speedSender,
                                     std::shared_ptr<arlcore::io::SenderBase<VelocityReportType>> velocitySender)
    : caps_(caps),
      simConfig_(simConfig),
      poseProvider_(navSourceId, std::move(poseSender), platformId),
      speedProvider_(navSourceId, std::move(speedSender), platformId),
      velocityProvider_(navSourceId, std::move(velocitySender), platformId),
      frame_(simConfig.initialLatitudeDeg, simConfig.initialLongitudeDeg, 0.0),
      headingRad_(simConfig.initialHeadingRad) {}

SimVehicleControl::~SimVehicleControl() { shutdown(); }

flt64_t SimVehicleControl::maxTurnRateRps() const { return caps_.surface.maxTurnRateRps.value_or(0.25); }

flt64_t SimVehicleControl::maxForwardSpeedMps() const { return caps_.surface.maxForwardSpeedMps.value_or(5.0); }

flt64_t SimVehicleControl::maxReverseSpeedMps() const { return caps_.surface.maxReverseSpeedMps.value_or(0.0); }

flt64_t SimVehicleControl::maxDepthRateMps() const { return caps_.underwater.maxDepthChangeRateMps.value_or(0.5); }

bool SimVehicleControl::initialize() {
  if (running_) {
    return true;
  }
  if (simConfig_.cycleRateHz <= 0.0) {
    UMAA_LOG_ERROR(util::SYSTEM_LOGGER,
                   "SimVehicleControl cycle rate must be positive (got " << simConfig_.cycleRateHz << " Hz)")
    return false;
  }
  {
    std::scoped_lock lock(mtx_);
    xEastM_ = 0.0;
    yNorthM_ = 0.0;
    headingRad_ = simConfig_.initialHeadingRad;
    speedMps_ = 0.0;
    depthM_ = 0.0;
    yawRateRps_ = 0.0;
  }
  running_ = true;
  simThread_ = std::thread(&SimVehicleControl::runLoop, this);
  UMAA_LOG_INFO(util::SYSTEM_LOGGER, "SimVehicleControl initialized at " << simConfig_.cycleRateHz << " Hz (start "
                                                                         << simConfig_.initialLatitudeDeg << ", "
                                                                         << simConfig_.initialLongitudeDeg << ")")
  return true;
}

void SimVehicleControl::shutdown() {
  running_ = false;
  if (simThread_.joinable()) {
    simThread_.join();
  }
}

void SimVehicleControl::runLoop() {
  using clock = std::chrono::steady_clock;
  const auto period =
      std::chrono::duration_cast<clock::duration>(std::chrono::duration<flt64_t>(1.0 / simConfig_.cycleRateHz));
  auto last = clock::now();
  auto next = last + period;
  while (running_) {
    std::this_thread::sleep_until(next);
    next += period;
    const auto now = clock::now();
    const flt64_t dtS = std::chrono::duration<flt64_t>(now - last).count();
    last = now;
    stepOnce(dtS);
  }
}

bool SimVehicleControl::sendControlVector(const ControlVector& cv) {
  std::scoped_lock lock(mtx_);
  setpoint_ = cv;
  controlVectorCount_++;
  UMAA_LOG_DEBUG(util::SYSTEM_LOGGER, "SimVehicleControl setpoint: heading(rad)="
                                          << cv.headingRad << " speed(mps)=" << cv.speedMps
                                          << " elevation=" << (cv.elevationM.has_value() ? cv.elevationM.value() : 0.0))
  return true;
}

void SimVehicleControl::stepOnce(flt64_t dtS) {
  if (dtS <= 0.0) {
    return;
  }
  {
    std::scoped_lock lock(mtx_);

    // Act on the latest setpoint (hold current heading at zero speed when none arrived yet).
    const flt64_t targetHeading = setpoint_.has_value() ? setpoint_->headingRad : headingRad_;
    flt64_t targetSpeed = setpoint_.has_value() ? setpoint_->speedMps : 0.0;
    targetSpeed = std::clamp(targetSpeed, -maxReverseSpeedMps(), maxForwardSpeedMps());

    // Inner heading loop: a proportional servo saturated at the platform turn rate, with an
    // optional first-order actuator lag. A gain of 1/dtS reproduces a pure rate limiter, which
    // is what this was before it modeled a servo, so that behavior stays reachable by config.
    // The finiteness screens are not redundant with config validation: an infinite gain times a
    // zero heading error is NaN, which std::clamp propagates straight into the published pose, and
    // an infinite lag freezes the yaw rate at its initial value forever without a word.
    const flt64_t headingErr = wrapPi(targetHeading - headingRad_);
    const bool gainUsable = std::isfinite(simConfig_.headingGainRpsPerRad) && simConfig_.headingGainRpsPerRad > 0.0;
    const flt64_t gain = gainUsable ? simConfig_.headingGainRpsPerRad : 1.0 / dtS;
    const flt64_t omegaMax = maxTurnRateRps();
    const flt64_t demanded = std::clamp(gain * headingErr, -omegaMax, omegaMax);
    if (std::isfinite(simConfig_.headingLagS) && simConfig_.headingLagS > 0.0) {
      yawRateRps_ += (dtS / (simConfig_.headingLagS + dtS)) * (demanded - yawRateRps_);
    } else {
      yawRateRps_ = demanded;
    }
    headingRad_ = wrapPi(headingRad_ + yawRateRps_ * dtS);

    const flt64_t speedErr = targetSpeed - speedMps_;
    const flt64_t maxDv = std::max(0.0, simConfig_.accelMps2) * dtS;
    speedMps_ += std::clamp(speedErr, -maxDv, maxDv);

    // A DEPTH or above-floor (ASF/AGL) setpoint converts to a target depth against the
    // configured floor depth; other elevation frames are not modeled by the sim.
    if (caps_.underwaterEnabled && setpoint_.has_value() && setpoint_->elevationM.has_value()) {
      std::optional<flt64_t> targetDepth;
      if (setpoint_->elevationFrame == ElevationFrame::DEPTH) {
        targetDepth = setpoint_->elevationM.value();
      } else if (setpoint_->elevationFrame == ElevationFrame::ALTITUDE_ASF ||
                 setpoint_->elevationFrame == ElevationFrame::ALTITUDE_AGL) {
        targetDepth = simConfig_.floorDepthM - setpoint_->elevationM.value();
      }
      if (targetDepth.has_value()) {
        const flt64_t clamped = std::clamp(targetDepth.value(), 0.0, simConfig_.floorDepthM);
        const flt64_t depthErr = clamped - depthM_;
        const flt64_t maxDd = maxDepthRateMps() * dtS;
        depthM_ += std::clamp(depthErr, -maxDd, maxDd);
        depthM_ = std::clamp(depthM_, 0.0, simConfig_.floorDepthM);
      }
    }

    xEastM_ += (speedMps_ * std::sin(headingRad_) + simConfig_.currentEastMps) * dtS;
    yNorthM_ += (speedMps_ * std::cos(headingRad_) + simConfig_.currentNorthMps) * dtS;
  }
  publishReports();
}

void SimVehicleControl::publishReports() {
  flt64_t lat = 0.0;
  flt64_t lon = 0.0;
  flt64_t h = 0.0;
  flt64_t heading = 0.0;
  flt64_t speed = 0.0;
  flt64_t depth = 0.0;
  flt64_t yawRate = 0.0;
  {
    std::scoped_lock lock(mtx_);
    frame_.Reverse(xEastM_, yNorthM_, 0.0, lat, lon, h);
    heading = headingRad_;
    speed = speedMps_;
    depth = depthM_;
    yawRate = yawRateRps_;
  }

  GlobalPoseReportType pose;
  pose.position().geodeticLatitude(lat);
  pose.position().geodeticLongitude(lon);
  pose.attitude().yaw().yaw(heading);
  pose.course() = heading;
  if (caps_.underwaterEnabled) {
    pose.depth() = depth;
    // Height above the sea floor, from the configured floor depth (0 at the floor). Gated on the
    // declared capability so the sim can stand in for a platform with no altimeter.
    if (caps_.reportsAltitudeAsf) {
      pose.altitudeASF() = std::max(0.0, simConfig_.floorDepthM - depth);
    }
  }
  poseProvider_.send(&pose);

  SpeedReportType speedReport;
  const flt64_t groundE = speed * std::sin(heading) + simConfig_.currentEastMps;
  const flt64_t groundN = speed * std::cos(heading) + simConfig_.currentNorthMps;
  speedReport.speedOverGround() = std::hypot(groundE, groundN);
  speedProvider_.send(&speedReport);

  VelocityReportType velocity;
  velocity.velocity().northSpeed(groundN);
  velocity.velocity().eastSpeed(groundE);
  velocity.velocity().downSpeed(0.0);
  velocity.attitudeRate().yawRate(yawRate);
  velocityProvider_.send(&velocity);
}

std::optional<ControlVector> SimVehicleControl::lastControlVector() const {
  std::scoped_lock lock(mtx_);
  return setpoint_;
}

uint64_t SimVehicleControl::controlVectorCount() const {
  std::scoped_lock lock(mtx_);
  return controlVectorCount_;
}

SimVehicleControl::SimState SimVehicleControl::state() const {
  std::scoped_lock lock(mtx_);
  SimState s;
  flt64_t h = 0.0;
  frame_.Reverse(xEastM_, yNorthM_, 0.0, s.latitudeDeg, s.longitudeDeg, h);
  s.headingRad = headingRad_;
  s.speedMps = speedMps_;
  s.depthM = depthM_;
  s.yawRateRps = yawRateRps_;
  return s;
}

}  // namespace arlcore::autopilot
