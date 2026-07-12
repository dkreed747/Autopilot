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

#include <gtest/gtest.h>

#include "DrivingResourceArbiter.h"

namespace arlcore::autopilot {

namespace {
constexpr int kVectorPriority = 100;
constexpr int kWaypointPriority = 10;
}  // namespace

TEST(DrivingResourceArbiterTest, StartsUnowned) {
  DrivingResourceArbiter arbiter(kVectorPriority, kWaypointPriority);
  EXPECT_EQ(arbiter.currentHolder(), DriveSource::NONE);
  EXPECT_TRUE(arbiter.canDrive(DriveSource::VECTOR));
  EXPECT_TRUE(arbiter.canDrive(DriveSource::WAYPOINT));
}

TEST(DrivingResourceArbiterTest, WaypointAcquiresWhenFree) {
  DrivingResourceArbiter arbiter(kVectorPriority, kWaypointPriority);
  EXPECT_TRUE(arbiter.acquire(DriveSource::WAYPOINT));
  EXPECT_TRUE(arbiter.ownsResource(DriveSource::WAYPOINT));
  EXPECT_FALSE(arbiter.wasRevoked(DriveSource::WAYPOINT));
}

TEST(DrivingResourceArbiterTest, VectorPreemptsWaypoint) {
  DrivingResourceArbiter arbiter(kVectorPriority, kWaypointPriority);
  ASSERT_TRUE(arbiter.acquire(DriveSource::WAYPOINT));

  // Higher-priority vector preempts the waypoint route.
  EXPECT_TRUE(arbiter.canDrive(DriveSource::VECTOR));
  EXPECT_TRUE(arbiter.acquire(DriveSource::VECTOR));
  EXPECT_EQ(arbiter.currentHolder(), DriveSource::VECTOR);
  EXPECT_TRUE(arbiter.wasRevoked(DriveSource::WAYPOINT));
  EXPECT_FALSE(arbiter.ownsResource(DriveSource::WAYPOINT));
}

TEST(DrivingResourceArbiterTest, WaypointRejectedWhileVectorHolds) {
  DrivingResourceArbiter arbiter(kVectorPriority, kWaypointPriority);
  ASSERT_TRUE(arbiter.acquire(DriveSource::VECTOR));

  // Lower-priority waypoint cannot preempt and is denied.
  EXPECT_FALSE(arbiter.canDrive(DriveSource::WAYPOINT));
  EXPECT_FALSE(arbiter.acquire(DriveSource::WAYPOINT));
  EXPECT_EQ(arbiter.currentHolder(), DriveSource::VECTOR);
}

TEST(DrivingResourceArbiterTest, ReleaseFreesResourceAndClearsRevoked) {
  DrivingResourceArbiter arbiter(kVectorPriority, kWaypointPriority);
  ASSERT_TRUE(arbiter.acquire(DriveSource::WAYPOINT));
  ASSERT_TRUE(arbiter.acquire(DriveSource::VECTOR));
  ASSERT_TRUE(arbiter.wasRevoked(DriveSource::WAYPOINT));

  arbiter.release(DriveSource::VECTOR);
  EXPECT_EQ(arbiter.currentHolder(), DriveSource::NONE);

  // Waypoint can now acquire again, and acquiring clears its revoked flag.
  EXPECT_TRUE(arbiter.acquire(DriveSource::WAYPOINT));
  EXPECT_FALSE(arbiter.wasRevoked(DriveSource::WAYPOINT));
}

TEST(DrivingResourceArbiterTest, ReacquireBySameHolderIsIdempotent) {
  DrivingResourceArbiter arbiter(kVectorPriority, kWaypointPriority);
  EXPECT_TRUE(arbiter.acquire(DriveSource::VECTOR));
  EXPECT_TRUE(arbiter.acquire(DriveSource::VECTOR));
  EXPECT_EQ(arbiter.currentHolder(), DriveSource::VECTOR);
}

TEST(DrivingResourceArbiterTest, ConfigurablePriorityFlip) {
  // Flip priorities so waypoint outranks vector.
  DrivingResourceArbiter arbiter(/*vector=*/10, /*waypoint=*/100);
  ASSERT_TRUE(arbiter.acquire(DriveSource::VECTOR));
  EXPECT_TRUE(arbiter.acquire(DriveSource::WAYPOINT));
  EXPECT_EQ(arbiter.currentHolder(), DriveSource::WAYPOINT);
  EXPECT_TRUE(arbiter.wasRevoked(DriveSource::VECTOR));
}

}  // namespace arlcore::autopilot
