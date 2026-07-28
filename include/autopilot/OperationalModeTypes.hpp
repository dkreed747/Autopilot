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

#ifndef APPS_AUTOPILOT_INCLUDE_AUTOPILOT_OPERATIONALMODETYPES_HPP_
#define APPS_AUTOPILOT_INCLUDE_AUTOPILOT_OPERATIONALMODETYPES_HPP_

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
#endif  // APPS_AUTOPILOT_INCLUDE_AUTOPILOT_OPERATIONALMODETYPES_HPP_
