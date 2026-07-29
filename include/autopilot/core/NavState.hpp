#ifndef AUTOPILOT_CORE_NAVSTATE_HPP_
#define AUTOPILOT_CORE_NAVSTATE_HPP_

#include <UMAA/SA/GlobalPoseStatus/GlobalPoseReportType.hpp>
#include <UMAA/SA/SpeedStatus/SpeedReportType.hpp>
#include <UMAA/SA/VelocityStatus/VelocityReportType.hpp>
#include <chrono>
#include <mutex>
#include <optional>

#include "InternalTypes.h"

namespace arlcore::autopilot {

//! \brief Thread-safe snapshot of the latest navigation data from the three SA services;
//! getters return copies so callers never hold the lock.
class NavState {
 public:
  void setPose(const UMAA::SA::GlobalPoseStatus::GlobalPoseReportType& pose) {
    std::scoped_lock lock(mtx_);
    pose_ = pose;
    poseReceivedAt_ = std::chrono::steady_clock::now();
  }
  void setSpeed(const UMAA::SA::SpeedStatus::SpeedReportType& speed) {
    std::scoped_lock lock(mtx_);
    speed_ = speed;
  }
  void setVelocity(const UMAA::SA::VelocityStatus::VelocityReportType& velocity) {
    std::scoped_lock lock(mtx_);
    velocity_ = velocity;
  }

  std::optional<UMAA::SA::GlobalPoseStatus::GlobalPoseReportType> pose() const {
    std::scoped_lock lock(mtx_);
    return pose_;
  }
  std::optional<UMAA::SA::SpeedStatus::SpeedReportType> speed() const {
    std::scoped_lock lock(mtx_);
    return speed_;
  }
  std::optional<UMAA::SA::VelocityStatus::VelocityReportType> velocity() const {
    std::scoped_lock lock(mtx_);
    return velocity_;
  }

  bool hasPose() const {
    std::scoped_lock lock(mtx_);
    return pose_.has_value();
  }

  //! \brief Milliseconds since the newest pose was received (nullopt before the first fix).
  std::optional<int64_t> poseAgeMs() const {
    std::scoped_lock lock(mtx_);
    if (!pose_.has_value()) {
      return std::nullopt;
    }
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - poseReceivedAt_)
        .count();
  }

  //! \brief Current ground speed if reported, else 0.
  flt64_t groundSpeedMps() const {
    std::scoped_lock lock(mtx_);
    if (speed_.has_value() && speed_->speedOverGround().has_value()) {
      return speed_->speedOverGround().value();
    }
    return 0.0;
  }

 private:
  mutable std::mutex mtx_;
  std::optional<UMAA::SA::GlobalPoseStatus::GlobalPoseReportType> pose_;
  std::chrono::steady_clock::time_point poseReceivedAt_{};
  std::optional<UMAA::SA::SpeedStatus::SpeedReportType> speed_;
  std::optional<UMAA::SA::VelocityStatus::VelocityReportType> velocity_;
};

}  // namespace arlcore::autopilot
#endif  // AUTOPILOT_CORE_NAVSTATE_HPP_
