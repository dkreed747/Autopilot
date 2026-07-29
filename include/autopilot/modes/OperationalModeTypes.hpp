#ifndef AUTOPILOT_MODES_OPERATIONALMODETYPES_HPP_
#define AUTOPILOT_MODES_OPERATIONALMODETYPES_HPP_

#include <cstdint>

namespace arlcore::autopilot {

//! \brief Operational command-authority modes (UMAA MM OperationalMode). MANUAL
//! is owned by the platform strategy and is never commandable over DDS.
enum class OperationalMode : uint8_t { MANUAL, STANDBY, REMOTE, AUTONOMOUS };

//! \brief Command-source classification relative to this platform's identity:
//! LOCAL is the onboard autonomy (source parentID == identity.platform_id),
//! REMOTE is any other commander including sources with an unset parentID.
enum class CommandClass : uint8_t { LOCAL, REMOTE };

//! \brief Outcome of a mutating admission request for a driving command at
//! ISSUED.
enum class AdmissionDecision : uint8_t { ADMIT, HOLD };

}  // namespace arlcore::autopilot
#endif  // AUTOPILOT_MODES_OPERATIONALMODETYPES_HPP_
