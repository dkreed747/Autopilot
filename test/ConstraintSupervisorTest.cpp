#include <gtest/gtest.h>

#include <memory>
#include <vector>

#include "DepthConditional.h"
#include "InternalTypes.h"
#include "LocalReaderSender.h"
#include "SpeedConditional.h"
#include "UuidFactory.h"
#include "WaterZoneConditional.h"
#include "autopilot/safety/ConstraintSupervisor.hpp"

using ConditionalOperatorEnumType =
    UMAA::Common::MaritimeEnumeration::ConditionalOperatorEnumModule::ConditionalOperatorEnumType;
using WaterZoneKindEnumType = UMAA::Common::MaritimeEnumeration::WaterZoneKindEnumModule::WaterZoneKindEnumType;
using DateTime = UMAA::Common::Measurement::DateTime;
using GeoPosition2D = UMAA::Common::Measurement::GeoPosition2D;
using ConditionalStateReportType = UMAA::MM::ConditionalStateReport::ConditionalStateReportType;

const DateTime kStamp(1, 100);

static std::shared_ptr<arlcore::umaa::conditional::WaterZoneConditional> makeZone(WaterZoneKindEnumType kind,
                                                                                  flt64_t ceilingDepthM,
                                                                                  flt64_t floorDepthM,
                                                                                  bool floorAsAsf = false) {
  const arlcore::NumericGuid conditionalId = arlcore::UuidFactory::getInstance().generateGuid();
  const arlcore::NumericGuid specId = arlcore::UuidFactory::getInstance().generateGuid();

  UMAA::MM::Conditional::WaterZoneConditionalType spec;
  spec.specializationReferenceID(specId.getGuid());
  spec.specializationReferenceTimestamp(kStamp);
  spec.zoneKind(kind);
  spec.ceiling().ElevationVariantTypeSubtypes().DepthVariantVariant(UMAA::Common::Measurement::DepthVariantType());
  spec.ceiling().ElevationVariantTypeSubtypes().DepthVariantVariant().depth(ceilingDepthM);
  if (floorAsAsf) {
    spec.floor().ElevationVariantTypeSubtypes().AltitudeASFVariantVariant(
        UMAA::Common::Measurement::AltitudeASFVariantType());
    spec.floor().ElevationVariantTypeSubtypes().AltitudeASFVariantVariant().altitude(floorDepthM);
  } else {
    spec.floor().ElevationVariantTypeSubtypes().DepthVariantVariant(UMAA::Common::Measurement::DepthVariantType());
    spec.floor().ElevationVariantTypeSubtypes().DepthVariantVariant().depth(floorDepthM);
  }

  UMAA::MM::BaseType::PolygonVariantType polygon;
  polygon.referencePoints().push_back(GeoPosition2D(39.000, -76.500));
  polygon.referencePoints().push_back(GeoPosition2D(39.001, -76.500));
  polygon.referencePoints().push_back(GeoPosition2D(39.001, -76.499));
  polygon.referencePoints().push_back(GeoPosition2D(39.000, -76.499));
  UMAA::MM::BaseType::ShapeVariantType shape;
  shape.ShapeVariantTypeSubtypes().PolygonVariantVariant(polygon);
  spec.zone().push_back(shape);

  const arlcore::umaa::conditional::ConditionalType base(conditionalId.getGuid(), "zone", specId.getGuid(), kStamp,
                                                         UMAA::MM::Conditional::WaterZoneConditionalTypeTopic);
  return std::make_shared<arlcore::umaa::conditional::WaterZoneConditional>(base, spec);
}

static std::shared_ptr<arlcore::umaa::conditional::SpeedConditional> makeSpeed(ConditionalOperatorEnumType op,
                                                                               flt64_t valueMps) {
  const arlcore::NumericGuid conditionalId = arlcore::UuidFactory::getInstance().generateGuid();
  const arlcore::NumericGuid specId = arlcore::UuidFactory::getInstance().generateGuid();
  const UMAA::MM::Conditional::SpeedConditionalType spec(op, valueMps, kStamp, specId.getGuid());
  const arlcore::umaa::conditional::ConditionalType base(conditionalId.getGuid(), "speed", specId.getGuid(), kStamp,
                                                         UMAA::MM::Conditional::SpeedConditionalTypeTopic);
  return std::make_shared<arlcore::umaa::conditional::SpeedConditional>(base, spec);
}

static std::shared_ptr<arlcore::umaa::conditional::DepthConditional> makeDepth(ConditionalOperatorEnumType op,
                                                                               flt64_t valueM) {
  const arlcore::NumericGuid conditionalId = arlcore::UuidFactory::getInstance().generateGuid();
  const arlcore::NumericGuid specId = arlcore::UuidFactory::getInstance().generateGuid();
  const UMAA::MM::Conditional::DepthConditionalType spec(op, valueM, kStamp, specId.getGuid());
  const arlcore::umaa::conditional::ConditionalType base(conditionalId.getGuid(), "depth", specId.getGuid(), kStamp,
                                                         UMAA::MM::Conditional::DepthConditionalTypeTopic);
  return std::make_shared<arlcore::umaa::conditional::DepthConditional>(base, spec);
}

class ConstraintSupervisorTest : public ::testing::Test {
 protected:
  void SetUp() override {
    config_.safety.stateReportPeriodMs = 0;  // publish on every update() for the tests
    zoneMap_ = std::make_unique<arlcore::autopilot::ZoneMap>(config_.zones);
    stateRw_ = std::make_shared<arlcore::io::LocalReaderSender<ConditionalStateReportType>>();
    supervisor_ = std::make_unique<arlcore::autopilot::ConstraintSupervisor>(
        config_, &nav_, zoneMap_.get(), stateRw_, arlcore::UuidFactory::getInstance().generateGuid());
  }

  arlcore::autopilot::AutopilotConfig config_;
  arlcore::autopilot::NavState nav_;
  std::unique_ptr<arlcore::autopilot::ZoneMap> zoneMap_;
  std::shared_ptr<arlcore::io::LocalReaderSender<ConditionalStateReportType>> stateRw_;
  std::unique_ptr<arlcore::autopilot::ConstraintSupervisor> supervisor_;
};

TEST_F(ConstraintSupervisorTest, BuildsSnapshotFromActiveConditionals) {
  // GIVEN: an active set with a keep-out zone, three speed bounds, and a depth cap
  arlcore::autopilot::ConditionalList active = {
      makeZone(WaterZoneKindEnumType::OUTSIDE, 5.0, 20.0),
      makeSpeed(ConditionalOperatorEnumType::LESS_THAN_OR_EQUAL_TO, 3.0),
      makeSpeed(ConditionalOperatorEnumType::LESS_THAN, 2.0),
      makeSpeed(ConditionalOperatorEnumType::GREATER_THAN_OR_EQUAL_TO, 1.0),
      makeDepth(ConditionalOperatorEnumType::LESS_THAN_OR_EQUAL_TO, 25.0),
  };

  // WHEN: the supervisor observes the set and updates
  supervisor_->activeSetObserver()->update(active);
  supervisor_->update();

  // THEN: the snapshot carries the zone, the most restrictive of the two upper speed bounds
  // (the lower bound rides along), the depth cap, and the zones land in the shared map
  const arlcore::autopilot::ConstraintSnapshot snapshot = supervisor_->snapshot();
  EXPECT_EQ(snapshot.revision, 1u);
  ASSERT_EQ(snapshot.zones.size(), 1u);
  EXPECT_EQ(snapshot.zones[0].kind, arlcore::autopilot::ZoneKind::KEEP_OUT);
  ASSERT_EQ(snapshot.zones[0].shapes.size(), 1u);
  EXPECT_EQ(snapshot.zones[0].shapes[0].polygon.size(), 4u);
  ASSERT_TRUE(snapshot.zones[0].band.ceiling.has_value());
  EXPECT_EQ(snapshot.zones[0].band.ceiling->frame, arlcore::autopilot::ElevationBound::Frame::DEPTH);
  EXPECT_DOUBLE_EQ(snapshot.zones[0].band.ceiling->value, 5.0);
  ASSERT_TRUE(snapshot.zones[0].band.floor.has_value());
  EXPECT_DOUBLE_EQ(snapshot.zones[0].band.floor->value, 20.0);

  ASSERT_TRUE(snapshot.maxSpeedMps.has_value());
  EXPECT_DOUBLE_EQ(snapshot.maxSpeedMps.value(), 2.0);
  ASSERT_TRUE(snapshot.minSpeedMps.has_value());
  EXPECT_DOUBLE_EQ(snapshot.minSpeedMps.value(), 1.0);
  ASSERT_TRUE(snapshot.maxDepthM.has_value());
  EXPECT_DOUBLE_EQ(snapshot.maxDepthM.value(), 25.0);
  EXPECT_FALSE(snapshot.minDepthM.has_value());

  EXPECT_TRUE(zoneMap_->hasZones());
  EXPECT_EQ(zoneMap_->revision(), snapshot.revision);
}

TEST_F(ConstraintSupervisorTest, MixedFrameZoneBandConverts) {
  // GIVEN: a zone with ceiling at depth 0 and floor 5 m above the sea floor
  supervisor_->activeSetObserver()->update({makeZone(WaterZoneKindEnumType::OUTSIDE, 0.0, 5.0, /*floorAsAsf=*/true)});

  // WHEN: the supervisor updates
  supervisor_->update();

  // THEN: both bounds keep their frames and the band is convertible
  const arlcore::autopilot::ConstraintSnapshot snapshot = supervisor_->snapshot();
  ASSERT_EQ(snapshot.zones.size(), 1u);
  const arlcore::autopilot::ElevationBand& band = snapshot.zones[0].band;
  EXPECT_TRUE(band.convertible);
  ASSERT_TRUE(band.ceiling.has_value());
  EXPECT_EQ(band.ceiling->frame, arlcore::autopilot::ElevationBound::Frame::DEPTH);
  EXPECT_DOUBLE_EQ(band.ceiling->value, 0.0);
  ASSERT_TRUE(band.floor.has_value());
  EXPECT_EQ(band.floor->frame, arlcore::autopilot::ElevationBound::Frame::ASF);
  EXPECT_DOUBLE_EQ(band.floor->value, 5.0);
}

TEST_F(ConstraintSupervisorTest, EmptyActiveSetClearsSnapshot) {
  // GIVEN: a supervisor holding a snapshot built from one speed conditional
  supervisor_->activeSetObserver()->update({makeSpeed(ConditionalOperatorEnumType::LESS_THAN, 2.0)});
  supervisor_->update();
  EXPECT_TRUE(supervisor_->snapshot().maxSpeedMps.has_value());

  // WHEN: the active set empties and the supervisor updates
  supervisor_->activeSetObserver()->update({});
  supervisor_->update();

  // THEN: the snapshot clears at a new revision and the zone map empties
  const arlcore::autopilot::ConstraintSnapshot snapshot = supervisor_->snapshot();
  EXPECT_EQ(snapshot.revision, 2u);
  EXPECT_FALSE(snapshot.maxSpeedMps.has_value());
  EXPECT_TRUE(snapshot.zones.empty());
  EXPECT_FALSE(zoneMap_->hasZones());
}

TEST_F(ConstraintSupervisorTest, NoRebuildWithoutActiveSetChange) {
  // GIVEN: a snapshot built at revision 1 from one speed conditional
  supervisor_->activeSetObserver()->update({makeSpeed(ConditionalOperatorEnumType::LESS_THAN, 2.0)});
  supervisor_->update();
  EXPECT_EQ(supervisor_->revision(), 1u);

  // WHEN: update() runs again with no active-set change
  supervisor_->update();
  supervisor_->update();

  // THEN: the revision does not move
  EXPECT_EQ(supervisor_->revision(), 1u);
}

TEST_F(ConstraintSupervisorTest, PublishesStateReportsAndDisposesRemoved) {
  // GIVEN: an evaluable speed conditional (5 < 10 -> true) and a zone with no pose (not evaluable)
  auto speed = makeSpeed(ConditionalOperatorEnumType::LESS_THAN, 10.0);
  UMAA::SA::SpeedStatus::SpeedReportType speedReport;
  speedReport.speedOverGround() = 5.0;
  speed->update(speedReport);

  auto zone = makeZone(WaterZoneKindEnumType::OUTSIDE, 0.0, 20.0);

  // WHEN: the supervisor observes both and updates
  supervisor_->conditionalSetObserver()->update({speed, zone});
  supervisor_->update();

  // THEN: only the evaluable conditional publishes a state; the zone stays silent instead
  // of guessing
  ConditionalStateReportType report;
  ASSERT_EQ(stateRw_->read(&report), arlcore::io::ReadStatus::SUCCESS);
  EXPECT_EQ(arlcore::NumericGuid(report.conditionalID()), speed->getConditionalId());
  EXPECT_TRUE(report.state());
  EXPECT_EQ(stateRw_->read(&report), arlcore::io::ReadStatus::NO_DATA);

  // WHEN: the speed constraint is violated and the next period runs
  speedReport.speedOverGround() = 15.0;
  speed->update(speedReport);
  supervisor_->update();

  // THEN: the report flips to state=false
  ASSERT_EQ(stateRw_->read(&report), arlcore::io::ReadStatus::SUCCESS);
  EXPECT_FALSE(report.state());

  // WHEN: the conditional is removed from the set
  supervisor_->conditionalSetObserver()->update({zone});
  supervisor_->update();

  // THEN: its state instance is disposed
  ASSERT_EQ(stateRw_->read(&report), arlcore::io::ReadStatus::DISPOSED);
  EXPECT_EQ(arlcore::NumericGuid(report.conditionalID()), speed->getConditionalId());
}

TEST_F(ConstraintSupervisorTest, CommandsAllowedWithoutSafeMode) {
  // GIVEN: a freshly constructed supervisor
  // WHEN: no violation has ever occurred
  // THEN: commands are allowed
  EXPECT_TRUE(supervisor_->commandsAllowed());
}
