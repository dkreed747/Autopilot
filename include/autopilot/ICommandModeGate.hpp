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

#ifndef APPS_AUTOPILOT_INCLUDE_AUTOPILOT_ICOMMANDMODEGATE_HPP_
#define APPS_AUTOPILOT_INCLUDE_AUTOPILOT_ICOMMANDMODEGATE_HPP_

#include <UMAA/Common/IdentifierType.hpp>
#include <cstdint>

#include "OperationalModeTypes.hpp"

namespace arlcore::autopilot {

//! \brief Operational-mode admission gate the driving command providers consult
//! (injected like ISafetyGate). The split matters: rejectedAtValidation() and
//! wouldAdmit() are side-effect-free and safe for the every-cycle
//! isCommandValid() re-run; requestAdmission() may fire an implicit mode
//! transition and must only be called from onIssued() after all other
//! validation passed, so an invalid command can never move the mode.
class ICommandModeGate {
 public:
  virtual ~ICommandModeGate() = default;

  //! \brief LOCAL iff the source's parentID equals this platform's id; REMOTE
  //! otherwise, including sources with an unset parentID.
  virtual CommandClass classify(
      const UMAA::Common::IdentifierType& source) const = 0;

  //! \brief Whether a command of this class must fail validation right now:
  //! always in MANUAL, and when out-of-mode under the
  //! commands_out_of_mode_are_failed policy. Held sessions must bypass this
  //! check so an authoritative flush can fail them with INTERRUPTED instead.
  virtual bool rejectedAtValidation(CommandClass cls) const = 0;

  //! \brief Whether a command of this class would be admitted right now,
  //! counting any implicit transition requestAdmission() would fire.
  virtual bool wouldAdmit(CommandClass cls) const = 0;

  //! \brief Admit a command of this class, firing a permitted implicit mode
  //! transition. HOLD means: park the session at ISSUED until the mode becomes
  //! compatible.
  virtual AdmissionDecision requestAdmission(CommandClass cls) = 0;

  //! \brief Whether commands of this class may keep executing under the current
  //! mode.
  virtual bool classAllowed(CommandClass cls) const = 0;

  //! \brief Bumped by every authoritative transition (explicit mode command,
  //! manual engagement). A held session whose recorded epoch went stale and
  //! whose class is still not admissible is flushed with INTERRUPTED; passive
  //! transitions (implicit entry, idle revert) never bump it, so held sessions
  //! survive them.
  virtual uint64_t authoritativeEpoch() const = 0;
};

}  // namespace arlcore::autopilot
#endif  // APPS_AUTOPILOT_INCLUDE_AUTOPILOT_ICOMMANDMODEGATE_HPP_
