#ifndef AUTOPILOT_TOOLS_CLIENTS_CONSTRAINTSCLIENT_HPP_
#define AUTOPILOT_TOOLS_CLIENTS_CONSTRAINTSCLIENT_HPP_

#include <array>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include <dds/dds.hpp>

#include <UMAA/Common/LargeSetMetadata.hpp>
#include <UMAA/MM/ActiveConstraintsControl/ActiveConstraintsCommandAckReportType.hpp>
#include <UMAA/MM/ActiveConstraintsControl/ActiveConstraintsCommandStatusType.hpp>
#include <UMAA/MM/ActiveConstraintsControl/ActiveConstraintsCommandType.hpp>
#include <UMAA/MM/Conditional/DepthConditionalType.hpp>
#include <UMAA/MM/Conditional/SpeedConditionalType.hpp>
#include <UMAA/MM/Conditional/WaterZoneConditionalType.hpp>
#include <UMAA/MM/ConditionalControl/ConditionalAddCommandStatusType.hpp>
#include <UMAA/MM/ConditionalControl/ConditionalAddCommandType.hpp>
#include <UMAA/MM/ConditionalControl/ConditionalDeleteCommandStatusType.hpp>
#include <UMAA/MM/ConditionalControl/ConditionalDeleteCommandType.hpp>
#include <UMAA/MM/ConditionalReport/ConditionalReportType.hpp>
#include <UMAA/MM/ConditionalStateReport/ConditionalStateReportType.hpp>

#include "clients/ClientIdentity.hpp"
#include "CycloneReader.h"
#include "CycloneSender.h"
#include "LargeSetReader.h"
#include "NumericGuid.h"
#include "SpecializationCache.h"
#include "InternalTypes.h"

namespace arlcore::autopilot::tools {

//! \brief Lowercase 8-4-4-4-12 UUID string for a GUID (the console's JSON id format; the
//! inverse of UuidFactory::parseGuidFromString).
std::string formatUuid(const arlcore::NumericGuid& guid);

//! \brief JSON-facing view of one constraint conditional from the autopilot's report.
struct ConstraintRecord {
  std::string id;    // conditionalID
  std::string name;
  std::string type;  // "keep_in" | "keep_out" | "speed" | "depth" | "other"
  std::vector<std::array<flt64_t, 2>> polygon;  // lat, lon (zones)
  std::optional<flt64_t> ceilingM;              // zone shallow bound, in ceilingFrame
  std::string ceilingFrame = "depth";          // "depth" (positive down) | "asf" (above sea floor)
  std::optional<flt64_t> floorM;                // zone deep bound, in floorFrame
  std::string floorFrame = "depth";
  std::optional<flt64_t> value;                 // speed (m/s) / depth (m)
  std::string op;                              // "lte" | "gte"
  bool active = false;
  std::optional<bool> state;  // last ConditionalStateReport (false = violated)
};

//! \brief One command-status transition observed on a constraint service.
struct ConstraintEvent {
  std::string service;  // "add" | "delete" | "active"
  std::string status;
  std::string reason;
  std::string message;
};

//! \brief The UMAA consumer side of the autopilot's constraint services: publishes
//! specialization payloads plus Add/Delete/ActiveConstraints commands and reads back the
//! reports, acks, and per-conditional states. The standing ActiveConstraints ack is the
//! authoritative applied set (the console's restart recovery).
class ConstraintsClient {
 public:
  //! \brief `destinationId` is the autopilot's constraints source ID
  //! (identity.constraints_source_id).
  ConstraintsClient(const dds::domain::DomainParticipant& participant,
                    const dds::pub::qos::DataWriterQos& wqos,
                    const dds::sub::qos::DataReaderQos& rqos,
                    const dds::sub::qos::DataReaderQos& largeSetRqos,
                    const arlcore::NumericGuid& destinationId,
                    const ClientIdentity& identity);

  //! \brief Create or update a water zone (empty `id` mints a new one); the ceiling/floor
  //! frames are "depth" (below the surface) or "asf" (above the sea floor) and may be mixed.
  std::string upsertZone(const std::string& id, const std::string& name, bool keepIn,
                         const std::vector<std::array<flt64_t, 2>>& polygonLatLon,
                         flt64_t ceilingM, const std::string& ceilingFrame,
                         flt64_t floorM, const std::string& floorFrame);

  //! \brief Create or update a speed constraint ("lte" = max speed, "gte" = min speed).
  std::string upsertSpeed(const std::string& id, const std::string& name, const std::string& op,
                          flt64_t valueMps);

  //! \brief Create or update a depth constraint ("lte" = max depth, "gte" = min depth).
  std::string upsertDepth(const std::string& id, const std::string& name, const std::string& op,
                          flt64_t valueM);

  //! \brief Delete a constraint (deactivating it first when it is in the applied set).
  bool removeConstraint(const std::string& id);

  //! \brief Command the applied active set (empty = none active).
  bool setActive(const std::vector<std::string>& ids);

  //! \brief Drain every subscription (report, set elements, ack, statuses, states).
  void poll();

  const std::vector<ConstraintRecord>& constraints() const { return records_; }
  const std::set<std::string>& activeIds() const { return activeIds_; }
  //! \brief Whether an ActiveConstraints ack has been seen (false = the applied set is
  //! unknown, e.g. the autopilot has not yet acknowledged any commander).
  bool activeKnown() const { return activeKnown_; }
  const std::optional<ConstraintEvent>& lastEvent() const { return lastEvent_; }

 private:
  using ConditionalType = UMAA::MM::Conditional::ConditionalType;
  using SetElement = UMAA::MM::ConditionalReport::ConditionalReportTypeConditionalsSetElement;

  //! \brief Publish the Add command for an already-published specialization payload.
  void sendAdd(const arlcore::NumericGuid& conditionalId, const std::string& name,
               const std::string& topic, const arlcore::NumericGuid& specId,
               const UMAA::Common::Measurement::DateTime& stamp);

  void rebuildRecords();

  // Payload + command writers
  std::shared_ptr<arlcore::io::CycloneSender<UMAA::MM::Conditional::WaterZoneConditionalType>> zoneWriter_;
  std::shared_ptr<arlcore::io::CycloneSender<UMAA::MM::Conditional::SpeedConditionalType>> speedWriter_;
  std::shared_ptr<arlcore::io::CycloneSender<UMAA::MM::Conditional::DepthConditionalType>> depthWriter_;
  std::shared_ptr<arlcore::io::CycloneSender<UMAA::MM::ConditionalControl::ConditionalAddCommandType>> addSender_;
  std::shared_ptr<arlcore::io::CycloneSender<UMAA::MM::ConditionalControl::ConditionalDeleteCommandType>>
      deleteSender_;
  std::shared_ptr<arlcore::io::CycloneSender<UMAA::MM::ActiveConstraintsControl::ActiveConstraintsCommandType>>
      activeSender_;

  // Read-back side
  std::shared_ptr<arlcore::io::CycloneReader<UMAA::MM::ConditionalReport::ConditionalReportType>> reportReader_;
  arlcore::umaa::LargeSetReader<ConditionalType, SetElement> setReader_;
  arlcore::umaa::SpecializationCache<UMAA::MM::Conditional::WaterZoneConditionalType> zoneCache_;
  arlcore::umaa::SpecializationCache<UMAA::MM::Conditional::SpeedConditionalType> speedCache_;
  arlcore::umaa::SpecializationCache<UMAA::MM::Conditional::DepthConditionalType> depthCache_;
  std::shared_ptr<arlcore::io::CycloneReader<
      UMAA::MM::ActiveConstraintsControl::ActiveConstraintsCommandAckReportType>> ackReader_;
  std::shared_ptr<arlcore::io::CycloneReader<
      UMAA::MM::ConditionalControl::ConditionalAddCommandStatusType>> addStatusReader_;
  std::shared_ptr<arlcore::io::CycloneReader<
      UMAA::MM::ConditionalControl::ConditionalDeleteCommandStatusType>> deleteStatusReader_;
  std::shared_ptr<arlcore::io::CycloneReader<
      UMAA::MM::ActiveConstraintsControl::ActiveConstraintsCommandStatusType>> activeStatusReader_;
  std::shared_ptr<arlcore::io::CycloneReader<
      UMAA::MM::ConditionalStateReport::ConditionalStateReportType>> stateReader_;

  ClientIdentity identity_;
  arlcore::NumericGuid destinationId_;

  std::optional<UMAA::Common::LargeSetMetadata> lastMetadata_;
  std::vector<ConditionalType> conditionals_;
  std::vector<ConstraintRecord> records_;
  std::set<std::string> activeIds_;
  bool activeKnown_ = false;
  std::map<std::string, bool> states_;
  std::optional<ConstraintEvent> lastEvent_;
};

}  // namespace arlcore::autopilot::tools

#endif  // AUTOPILOT_TOOLS_CLIENTS_CONSTRAINTSCLIENT_HPP_
