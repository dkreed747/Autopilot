//---------------------------------------------------------------------------
// Copyright 2026 Pennsylvania State University
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

#include <gtest/gtest.h>

#include <memory>
#include <vector>

#include "DepthConditional.h"
#include "LocalReaderSender.h"
#include "SpeedConditional.h"
#include "UuidFactory.h"
#include "WaterZoneConditional.h"

#include "ConstraintSupervisor.h"

namespace arlcore::autopilot {

using arlcore::io::LocalReaderSender;
using arlcore::io::ReadStatus;
using arlcore::umaa::conditional::ConditionalType;
using arlcore::umaa::conditional::DepthConditional;
using arlcore::umaa::conditional::SpeedConditional;
using arlcore::umaa::conditional::WaterZoneConditional;
using UMAA::Common::MaritimeEnumeration::ConditionalOperatorEnumModule::ConditionalOperatorEnumType;
using UMAA::Common::MaritimeEnumeration::WaterZoneKindEnumModule::WaterZoneKindEnumType;
using UMAA::Common::Measurement::DateTime;
using UMAA::Common::Measurement::GeoPosition2D;
using UMAA::MM::ConditionalStateReport::ConditionalStateReportType;

namespace {

const DateTime kStamp(1, 100);

std::shared_ptr<WaterZoneConditional> makeZone(WaterZoneKindEnumType kind, double ceilingDepthM,
                                               double floorDepthM) {
  const arlcore::NumericGuid conditionalId = arlcore::UuidFactory::getInstance().generateGuid();
  const arlcore::NumericGuid specId = arlcore::UuidFactory::getInstance().generateGuid();

  UMAA::MM::Conditional::WaterZoneConditionalType spec;
  spec.specializationReferenceID(specId.getGuid());
  spec.specializationReferenceTimestamp(kStamp);
  spec.zoneKind(kind);
  spec.ceiling().ElevationVariantTypeSubtypes().DepthVariantVariant(UMAA::Common::Measurement::DepthVariantType());
  spec.ceiling().ElevationVariantTypeSubtypes().DepthVariantVariant().depth(ceilingDepthM);
  spec.floor().ElevationVariantTypeSubtypes().DepthVariantVariant(UMAA::Common::Measurement::DepthVariantType());
  spec.floor().ElevationVariantTypeSubtypes().DepthVariantVariant().depth(floorDepthM);

  UMAA::MM::BaseType::PolygonVariantType polygon;
  polygon.referencePoints().push_back(GeoPosition2D(39.000, -76.500));
  polygon.referencePoints().push_back(GeoPosition2D(39.001, -76.500));
  polygon.referencePoints().push_back(GeoPosition2D(39.001, -76.499));
  polygon.referencePoints().push_back(GeoPosition2D(39.000, -76.499));
  UMAA::MM::BaseType::ShapeVariantType shape;
  shape.ShapeVariantTypeSubtypes().PolygonVariantVariant(polygon);
  spec.zone().push_back(shape);

  const ConditionalType base(conditionalId.getGuid(), "zone", specId.getGuid(), kStamp,
                             UMAA::MM::Conditional::WaterZoneConditionalTypeTopic);
  return std::make_shared<WaterZoneConditional>(base, spec);
}

std::shared_ptr<SpeedConditional> makeSpeed(ConditionalOperatorEnumType op, double valueMps) {
  const arlcore::NumericGuid conditionalId = arlcore::UuidFactory::getInstance().generateGuid();
  const arlcore::NumericGuid specId = arlcore::UuidFactory::getInstance().generateGuid();
  const UMAA::MM::Conditional::SpeedConditionalType spec(op, valueMps, kStamp, specId.getGuid());
  const ConditionalType base(conditionalId.getGuid(), "speed", specId.getGuid(), kStamp,
                             UMAA::MM::Conditional::SpeedConditionalTypeTopic);
  return std::make_shared<SpeedConditional>(base, spec);
}

std::shared_ptr<DepthConditional> makeDepth(ConditionalOperatorEnumType op, double valueM) {
  const arlcore::NumericGuid conditionalId = arlcore::UuidFactory::getInstance().generateGuid();
  const arlcore::NumericGuid specId = arlcore::UuidFactory::getInstance().generateGuid();
  const UMAA::MM::Conditional::DepthConditionalType spec(op, valueM, kStamp, specId.getGuid());
  const ConditionalType base(conditionalId.getGuid(), "depth", specId.getGuid(), kStamp,
                             UMAA::MM::Conditional::DepthConditionalTypeTopic);
  return std::make_shared<DepthConditional>(base, spec);
}

}  // namespace

class ConstraintSupervisorTest : public ::testing::Test {
 protected:
  void SetUp() override {
    config_.safety.stateReportPeriodMs = 0;  // publish on every update() for the tests
    zoneMap_ = std::make_unique<ZoneMap>(config_.zones);
    stateRw_ = std::make_shared<LocalReaderSender<ConditionalStateReportType>>();
    supervisor_ = std::make_unique<ConstraintSupervisor>(config_, &nav_, zoneMap_.get(), stateRw_,
        arlcore::UuidFactory::getInstance().generateGuid());
  }

  AutopilotConfig config_;
  NavState nav_;
  std::unique_ptr<ZoneMap> zoneMap_;
  std::shared_ptr<LocalReaderSender<ConditionalStateReportType>> stateRw_;
  std::unique_ptr<ConstraintSupervisor> supervisor_;
};

TEST_F(ConstraintSupervisorTest, BuildsSnapshotFromActiveConditionals) {
  ConditionalList active = {
      makeZone(WaterZoneKindEnumType::OUTSIDE, 5.0, 20.0),
      makeSpeed(ConditionalOperatorEnumType::LESS_THAN_OR_EQUAL_TO, 3.0),
      makeSpeed(ConditionalOperatorEnumType::LESS_THAN, 2.0),
      makeSpeed(ConditionalOperatorEnumType::GREATER_THAN_OR_EQUAL_TO, 1.0),
      makeDepth(ConditionalOperatorEnumType::LESS_THAN_OR_EQUAL_TO, 25.0),
  };
  supervisor_->activeSetObserver()->update(active);
  supervisor_->update();

  const ConstraintSnapshot snapshot = supervisor_->snapshot();
  EXPECT_EQ(snapshot.revision, 1u);
  ASSERT_EQ(snapshot.zones.size(), 1u);
  EXPECT_EQ(snapshot.zones[0].kind, ZoneKind::KEEP_OUT);
  ASSERT_EQ(snapshot.zones[0].shapes.size(), 1u);
  EXPECT_EQ(snapshot.zones[0].shapes[0].polygon.size(), 4u);
  ASSERT_TRUE(snapshot.zones[0].band.ceilingDepthM.has_value());
  EXPECT_DOUBLE_EQ(snapshot.zones[0].band.ceilingDepthM.value(), 5.0);
  EXPECT_DOUBLE_EQ(snapshot.zones[0].band.floorDepthM.value(), 20.0);

  // Most restrictive of the two upper speed bounds; the lower bound rides along.
  ASSERT_TRUE(snapshot.maxSpeedMps.has_value());
  EXPECT_DOUBLE_EQ(snapshot.maxSpeedMps.value(), 2.0);
  ASSERT_TRUE(snapshot.minSpeedMps.has_value());
  EXPECT_DOUBLE_EQ(snapshot.minSpeedMps.value(), 1.0);
  ASSERT_TRUE(snapshot.maxDepthM.has_value());
  EXPECT_DOUBLE_EQ(snapshot.maxDepthM.value(), 25.0);
  EXPECT_FALSE(snapshot.minDepthM.has_value());

  // Zones were pushed into the shared map.
  EXPECT_TRUE(zoneMap_->hasZones());
  EXPECT_EQ(zoneMap_->revision(), snapshot.revision);
}

TEST_F(ConstraintSupervisorTest, EmptyActiveSetClearsSnapshot) {
  supervisor_->activeSetObserver()->update({makeSpeed(ConditionalOperatorEnumType::LESS_THAN, 2.0)});
  supervisor_->update();
  EXPECT_TRUE(supervisor_->snapshot().maxSpeedMps.has_value());

  supervisor_->activeSetObserver()->update({});
  supervisor_->update();
  const ConstraintSnapshot snapshot = supervisor_->snapshot();
  EXPECT_EQ(snapshot.revision, 2u);
  EXPECT_FALSE(snapshot.maxSpeedMps.has_value());
  EXPECT_TRUE(snapshot.zones.empty());
  EXPECT_FALSE(zoneMap_->hasZones());
}

TEST_F(ConstraintSupervisorTest, NoRebuildWithoutActiveSetChange) {
  supervisor_->activeSetObserver()->update({makeSpeed(ConditionalOperatorEnumType::LESS_THAN, 2.0)});
  supervisor_->update();
  EXPECT_EQ(supervisor_->revision(), 1u);
  supervisor_->update();
  supervisor_->update();
  EXPECT_EQ(supervisor_->revision(), 1u);
}

TEST_F(ConstraintSupervisorTest, PublishesStateReportsAndDisposesRemoved) {
  auto speed = makeSpeed(ConditionalOperatorEnumType::LESS_THAN, 10.0);
  UMAA::SA::SpeedStatus::SpeedReportType speedReport;
  speedReport.speedOverGround() = 5.0;
  speed->update(speedReport);  // evaluable: 5 < 10 -> true

  auto zone = makeZone(WaterZoneKindEnumType::OUTSIDE, 0.0, 20.0);  // no pose -> not evaluable

  supervisor_->conditionalSetObserver()->update({speed, zone});
  supervisor_->update();

  // Only the evaluable conditional publishes a state; the zone stays silent instead of guessing.
  ConditionalStateReportType report;
  ASSERT_EQ(stateRw_->read(&report), ReadStatus::SUCCESS);
  EXPECT_EQ(arlcore::NumericGuid(report.conditionalID()), speed->getConditionalId());
  EXPECT_TRUE(report.state());
  EXPECT_EQ(stateRw_->read(&report), ReadStatus::NO_DATA);

  // Violate the speed constraint: the next period reports state=false.
  speedReport.speedOverGround() = 15.0;
  speed->update(speedReport);
  supervisor_->update();
  ASSERT_EQ(stateRw_->read(&report), ReadStatus::SUCCESS);
  EXPECT_FALSE(report.state());

  // Removing the conditional disposes its state instance.
  supervisor_->conditionalSetObserver()->update({zone});
  supervisor_->update();
  ASSERT_EQ(stateRw_->read(&report), ReadStatus::DISPOSED);
  EXPECT_EQ(arlcore::NumericGuid(report.conditionalID()), speed->getConditionalId());
}

TEST_F(ConstraintSupervisorTest, CommandsAllowedWithoutSafeMode) {
  EXPECT_TRUE(supervisor_->commandsAllowed());
}

}  // namespace arlcore::autopilot
