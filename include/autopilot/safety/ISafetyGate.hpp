#ifndef AUTOPILOT_SAFETY_ISAFETYGATE_HPP_
#define AUTOPILOT_SAFETY_ISAFETYGATE_HPP_

namespace arlcore::autopilot {

//! \brief Gate the command providers consult before accepting new commands (false while the
//! autopilot is in safe mode).
class ISafetyGate {
 public:
  virtual ~ISafetyGate() = default;
  virtual bool commandsAllowed() const = 0;
};

}  // namespace arlcore::autopilot
#endif  // AUTOPILOT_SAFETY_ISAFETYGATE_HPP_
