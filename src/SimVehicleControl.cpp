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

#include "SimVehicleControl.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <utility>

#include "AngleMath.h"
#include "Logger.h"

namespace arlcore::autopilot {

using UMAA::EO::UVPlatformSpecs::UVPlatformSpecsReportType;
using UMAA::EO::UVPlatformSpecs::UVPlatformCapabilitiesReportType;
using UMAA::EO::UVPlatformSpecs::SurfaceCapabilityLimitsType;
using UMAA::EO::UVPlatformSpecs::UnderwaterCapabilityLimitsType;
using UMAA::SA::GlobalPoseStatus::GlobalPoseReportType;
using UMAA::SA::SpeedStatus::SpeedReportType;
using UMAA::SA::VelocityStatus::VelocityReportType;

namespace {
//! \brief Copy an optional config value into a generated @optional field.
void setIf(std::optional<double>& field, const std::optional<double>& value) {  // NOLINT(runtime/references)
  if (value.has_value()) {
    field = value.value();
  }
}
}  // namespace

SimVehicleControl::SimVehicleControl(
    const PlatformSpecsConfig& specs, const PlatformCapabilitiesConfig& caps,
    const SimVehicleConfig& simConfig, const arlcore::NumericGuid& navSourceId,
    std::shared_ptr<arlcore::io::SenderBase<GlobalPoseReportType>> poseSender,
    std::shared_ptr<arlcore::io::SenderBase<SpeedReportType>> speedSender,
    std::shared_ptr<arlcore::io::SenderBase<VelocityReportType>> velocitySender) :
    specs_(specs),
    caps_(caps),
    simConfig_(simConfig),
    poseProvider_(navSourceId, std::move(poseSender)),
    speedProvider_(navSourceId, std::move(speedSender)),
    velocityProvider_(navSourceId, std::move(velocitySender)),
    frame_(simConfig.initialLatitudeDeg, simConfig.initialLongitudeDeg, 0.0),
    headingRad_(simConfig.initialHeadingRad) {}

SimVehicleControl::~SimVehicleControl() {
  shutdown();
}

double SimVehicleControl::maxTurnRateRps() const {
  return caps_.surface.maxTurnRateRps.value_or(0.25);
}

double SimVehicleControl::maxForwardSpeedMps() const {
  return caps_.surface.maxForwardSpeedMps.value_or(5.0);
}

double SimVehicleControl::maxReverseSpeedMps() const {
  return caps_.surface.maxReverseSpeedMps.value_or(0.0);
}

double SimVehicleControl::maxDepthRateMps() const {
  return caps_.underwater.maxDepthChangeRateMps.value_or(0.5);
}

bool SimVehicleControl::initialize() {
  if (running_) {
    return true;
  }
  if (simConfig_.cycleRateHz <= 0.0) {
    UMAA_LOG_ERROR(util::SYSTEM_LOGGER, "SimVehicleControl cycle rate must be positive (got "
      << simConfig_.cycleRateHz << " Hz)")
    return false;
  }
  {
    std::lock_guard<std::mutex> lock(mtx_);
    xEastM_ = 0.0;
    yNorthM_ = 0.0;
    headingRad_ = simConfig_.initialHeadingRad;
    speedMps_ = 0.0;
    depthM_ = 0.0;
    yawRateRps_ = 0.0;
  }
  running_ = true;
  simThread_ = std::thread(&SimVehicleControl::runLoop, this);
  UMAA_LOG_INFO(util::SYSTEM_LOGGER, "SimVehicleControl initialized for platform '" << specs_.name
    << "' at " << simConfig_.cycleRateHz << " Hz (start "
    << simConfig_.initialLatitudeDeg << ", " << simConfig_.initialLongitudeDeg << ")")
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
  const auto period = std::chrono::duration_cast<clock::duration>(
      std::chrono::duration<double>(1.0 / simConfig_.cycleRateHz));
  auto last = clock::now();
  auto next = last + period;
  while (running_) {
    std::this_thread::sleep_until(next);
    next += period;
    const auto now = clock::now();
    const double dtS = std::chrono::duration<double>(now - last).count();
    last = now;
    stepOnce(dtS);
  }
}

bool SimVehicleControl::sendControlVector(const ControlVector& cv) {
  std::lock_guard<std::mutex> lock(mtx_);
  setpoint_ = cv;
  controlVectorCount_++;
  UMAA_LOG_DEBUG(util::SYSTEM_LOGGER, "SimVehicleControl setpoint: heading(rad)=" << cv.headingRad
    << " speed(mps)=" << cv.speedMps
    << " elevation=" << (cv.elevationM.has_value() ? cv.elevationM.value() : 0.0))
  return true;
}

void SimVehicleControl::stepOnce(double dtS) {
  if (dtS <= 0.0) {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mtx_);

    // Act on the latest setpoint (hold current heading at zero speed when none arrived yet).
    const double targetHeading = setpoint_.has_value() ? setpoint_->headingRad : headingRad_;
    double targetSpeed = setpoint_.has_value() ? setpoint_->speedMps : 0.0;
    targetSpeed = std::clamp(targetSpeed, -maxReverseSpeedMps(), maxForwardSpeedMps());

    // Turn toward the commanded heading, limited by the platform's max turn rate.
    const double headingErr = wrapPi(targetHeading - headingRad_);
    const double maxDelta = maxTurnRateRps() * dtS;
    const double applied = std::clamp(headingErr, -maxDelta, maxDelta);
    headingRad_ = wrapPi(headingRad_ + applied);
    yawRateRps_ = applied / dtS;

    // Accelerate toward the commanded speed, limited by the surge acceleration.
    const double speedErr = targetSpeed - speedMps_;
    const double maxDv = std::max(0.0, simConfig_.accelMps2) * dtS;
    speedMps_ += std::clamp(speedErr, -maxDv, maxDv);

    // Drive depth toward a commanded DEPTH or ALTITUDE_ASF (above sea floor) setpoint when
    // the platform supports it; both are converted to a target depth against the configured
    // floor depth. Other elevation frames are not modeled by the sim.
    if (caps_.underwaterEnabled && setpoint_.has_value() && setpoint_->elevationM.has_value()) {
      std::optional<double> targetDepth;
      if (setpoint_->elevationFrame == ElevationFrame::DEPTH) {
        targetDepth = setpoint_->elevationM.value();
      } else if (setpoint_->elevationFrame == ElevationFrame::ALTITUDE_ASF ||
                 setpoint_->elevationFrame == ElevationFrame::ALTITUDE_AGL) {
        targetDepth = simConfig_.floorDepthM - setpoint_->elevationM.value();
      }
      if (targetDepth.has_value()) {
        const double clamped = std::clamp(targetDepth.value(), 0.0, simConfig_.floorDepthM);
        const double depthErr = clamped - depthM_;
        const double maxDd = maxDepthRateMps() * dtS;
        depthM_ += std::clamp(depthErr, -maxDd, maxDd);
        depthM_ = std::clamp(depthM_, 0.0, simConfig_.floorDepthM);
      }
    }

    // Advance the position in the local tangent plane.
    xEastM_ += speedMps_ * dtS * std::sin(headingRad_);
    yNorthM_ += speedMps_ * dtS * std::cos(headingRad_);
  }
  publishReports();
}

void SimVehicleControl::publishReports() {
  double lat = 0.0;
  double lon = 0.0;
  double h = 0.0;
  double heading = 0.0;
  double speed = 0.0;
  double depth = 0.0;
  double yawRate = 0.0;
  {
    std::lock_guard<std::mutex> lock(mtx_);
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
    // Height above the sea floor, from the configured floor depth (0 at the floor).
    pose.altitudeASF() = std::max(0.0, simConfig_.floorDepthM - depth);
  }
  poseProvider_.send(&pose);

  SpeedReportType speedReport;
  speedReport.speedOverGround() = std::fabs(speed);
  speedProvider_.send(&speedReport);

  VelocityReportType velocity;
  velocity.velocity().northSpeed(speed * std::cos(heading));
  velocity.velocity().eastSpeed(speed * std::sin(heading));
  velocity.velocity().downSpeed(0.0);
  velocity.attitudeRate().yawRate(yawRate);
  velocityProvider_.send(&velocity);
}

std::optional<ControlVector> SimVehicleControl::lastControlVector() const {
  std::lock_guard<std::mutex> lock(mtx_);
  return setpoint_;
}

uint64_t SimVehicleControl::controlVectorCount() const {
  std::lock_guard<std::mutex> lock(mtx_);
  return controlVectorCount_;
}

SimVehicleControl::SimState SimVehicleControl::state() const {
  std::lock_guard<std::mutex> lock(mtx_);
  SimState s;
  double h = 0.0;
  frame_.Reverse(xEastM_, yNorthM_, 0.0, s.latitudeDeg, s.longitudeDeg, h);
  s.headingRad = headingRad_;
  s.speedMps = speedMps_;
  s.depthM = depthM_;
  s.yawRateRps = yawRateRps_;
  return s;
}

UVPlatformSpecsReportType SimVehicleControl::getPlatformSpecs() const {
  UVPlatformSpecsReportType report;
  report.name() = specs_.name;
  report.lengthAtWaterline() = specs_.lengthAtWaterlineM;
  report.beamAtWaterline() = specs_.beamAtWaterlineM;
  report.draft() = specs_.draftM;
  report.forwardDistance() = specs_.forwardDistanceM;
  report.aftDistance() = specs_.aftDistanceM;
  report.portDistance() = specs_.portDistanceM;
  report.starboardDistance() = specs_.starboardDistanceM;
  report.topDistance() = specs_.topDistanceM;
  report.bottomDistance() = specs_.bottomDistanceM;
  report.displacement() = specs_.displacementMetricTon;
  report.weightLight() = specs_.weightLightMetricTon;
  report.weightLoaded() = specs_.weightLoadedMetricTon;
  // centerOfBuoyancy, centerOfGravity, and referenceFrameOrigin keep generated defaults.
  return report;
}

UVPlatformCapabilitiesReportType SimVehicleControl::getPlatformCapabilities() const {
  UVPlatformCapabilitiesReportType report;
  report.minWaterDepth() = caps_.minWaterDepthM;

  SurfaceCapabilityLimitsType surface;
  setIf(surface.maxForwardSpeed(), caps_.surface.maxForwardSpeedMps);
  setIf(surface.maxReverseSpeed(), caps_.surface.maxReverseSpeedMps);
  setIf(surface.cruisingSpeed(), caps_.surface.cruisingSpeedMps);
  setIf(surface.maxTurnRate(), caps_.surface.maxTurnRateRps);
  setIf(surface.minSpeedInMedium(), caps_.surface.minSpeedInMediumMps);
  report.surfaceCapabilities() = surface;

  if (caps_.underwaterEnabled) {
    UnderwaterCapabilityLimitsType underwater;
    setIf(underwater.maxForwardSpeed(), caps_.underwater.maxForwardSpeedMps);
    setIf(underwater.maxReverseSpeed(), caps_.underwater.maxReverseSpeedMps);
    setIf(underwater.cruisingSpeed(), caps_.underwater.cruisingSpeedMps);
    setIf(underwater.maxTurnRate(), caps_.underwater.maxTurnRateRps);
    setIf(underwater.minSpeedInMedium(), caps_.underwater.minSpeedInMediumMps);
    setIf(underwater.maxDepthChangeRate(), caps_.underwater.maxDepthChangeRateMps);
    report.underwaterCapabilities() = underwater;
  }

  return report;
}

}  // namespace arlcore::autopilot
