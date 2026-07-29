#include "monitors/VectorActivityMonitor.hpp"

#include "autopilot/guidance/ControlVector.hpp"
#include "autopilot/guidance/MissionRoute.hpp"
#include "autopilot/guidance/ToleranceUtils.hpp"

namespace arlcore::autopilot::tools {

using arlcore::io::CycloneReader;
using arlcore::io::ReadStatus;
using arlcore::io::SampleEnvelope;
using StatusEnum = UMAA::Common::MaritimeEnumeration::CommandStatusEnumModule::
    CommandStatusEnumType;

static constexpr uint32_t kMaxCommandDrain = 32;
static constexpr size_t kMaxTrackedVectors = 16;
static constexpr double kTerminalEvictS = 60.0;

static void decodeVector(
    const UMAA::MO::GlobalVectorControl::GlobalVectorCommandType& cmd,
    ObservedVector* out) {
  const std::optional<DirectionValue> dir =
      tolerance::extractDirection(cmd.direction());
  out->headingRad =
      dir.has_value() ? std::optional<double>(dir->headingRad) : std::nullopt;
  const std::optional<SpeedValue> speed = tolerance::extractSpeed(cmd.speed());
  out->speedMps =
      speed.has_value() ? std::optional<double>(speed->speedMps) : std::nullopt;
  out->elevValueM.reset();
  out->elevFrame.clear();
  if (cmd.elevation().has_value()) {
    const std::optional<ElevationValue> elev =
        tolerance::extractElevation(cmd.elevation().value());
    if (elev.has_value()) {
      out->elevValueM = elev->valueM;
      out->elevFrame = elev->frame == ElevationFrame::DEPTH ? "depth" : "asf";
    }
  }
  out->endTime = cmd.endTime();
}

VectorActivityMonitor::VectorActivityMonitor(
    const dds::domain::DomainParticipant& participant,
    const dds::sub::qos::DataReaderQos& rqos)
    : cmdReader_(std::make_shared<CycloneReader<CommandType>>(
          participant,
          UMAA::MO::GlobalVectorControl::GlobalVectorCommandTypeTopic, rqos)),
      statusReader_(std::make_shared<CycloneReader<StatusType>>(
          participant,
          UMAA::MO::GlobalVectorControl::GlobalVectorCommandStatusTypeTopic,
          rqos)),
      execReader_(std::make_shared<CycloneReader<ExecType>>(
          participant,
          UMAA::MO::GlobalVectorControl::
              GlobalVectorExecutionStatusReportTypeTopic,
          rqos)) {}

void VectorActivityMonitor::poll() {
  const auto now = std::chrono::steady_clock::now();

  SampleEnvelope<CommandType> envelopes[kMaxCommandDrain];
  const size_t drained = cmdReader_->readUpToN(envelopes, kMaxCommandDrain);
  for (size_t i = 0; i < drained; ++i) {
    const CommandType& cmd = envelopes[i].data;
    const arlcore::NumericGuid session(cmd.sessionID());
    if (envelopes[i].status == ReadStatus::DISPOSED) {
      auto it = vectors_.find(session);
      if (it != vectors_.end()) {
        it->second.terminal = true;
        it->second.lastSeen = now;
      }
      continue;
    }
    if (envelopes[i].status != ReadStatus::SUCCESS) {
      continue;
    }
    ObservedVector& vec = vectors_[session];
    vec.sessionId = session;
    vec.sourceId = arlcore::NumericGuid(cmd.source().id());
    vec.sourceParentId = arlcore::NumericGuid(cmd.source().parentID());
    vec.lastSeen = now;
    decodeVector(cmd, &vec);
  }

  StatusType status;
  while (statusReader_->read(&status) == ReadStatus::SUCCESS) {
    auto it = vectors_.find(arlcore::NumericGuid(status.sessionID()));
    if (it == vectors_.end()) {
      continue;
    }
    it->second.lastStatus = statusName(status.commandStatus());
    it->second.lastReason = statusReasonName(status.commandStatusReason());
    it->second.lastSeen = now;
    if (status.commandStatus() == StatusEnum::COMPLETED ||
        status.commandStatus() == StatusEnum::FAILED ||
        status.commandStatus() == StatusEnum::CANCELED) {
      it->second.terminal = true;
    }
  }

  ExecType exec;
  while (execReader_->read(&exec) == ReadStatus::SUCCESS) {
    auto it = vectors_.find(arlcore::NumericGuid(exec.sessionID()));
    if (it == vectors_.end()) {
      continue;
    }
    it->second.execStatus = exec;
    it->second.execSeen = now;
  }

  for (auto it = vectors_.begin(); it != vectors_.end();) {
    const double idleS =
        std::chrono::duration<double>(now - it->second.lastSeen).count();
    if (it->second.terminal && idleS > kTerminalEvictS) {
      it = vectors_.erase(it);
    } else {
      ++it;
    }
  }
  while (vectors_.size() > kMaxTrackedVectors) {
    auto victim = vectors_.begin();
    for (auto it = vectors_.begin(); it != vectors_.end(); ++it) {
      const bool betterVictim =
          (it->second.terminal && !victim->second.terminal) ||
          (it->second.terminal == victim->second.terminal &&
           it->second.lastSeen < victim->second.lastSeen);
      if (betterVictim) {
        victim = it;
      }
    }
    vectors_.erase(victim);
  }
}

}  // namespace arlcore::autopilot::tools
