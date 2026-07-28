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
//! \brief Config with the default class split: safe > remote {500,400} > local {vector,waypoint}.
ArbitrationConfig makeConfig(int localVector = 100, int localWaypoint = 10, int safe = 1000) {
  ArbitrationConfig config;
  config.local.vectorPriority = localVector;
  config.local.waypointPriority = localWaypoint;
  config.safePriority = safe;
  return config;
}
}  // namespace

TEST(DrivingResourceArbiterTest, StartsUnowned) {
  DrivingResourceArbiter arbiter(makeConfig());
  EXPECT_EQ(arbiter.currentHolder(), DriveSource::NONE);
  EXPECT_TRUE(arbiter.canDrive(DriveSource::VECTOR));
  EXPECT_TRUE(arbiter.canDrive(DriveSource::WAYPOINT));
}

TEST(DrivingResourceArbiterTest, WaypointAcquiresWhenFree) {
  DrivingResourceArbiter arbiter(makeConfig());
  EXPECT_TRUE(arbiter.acquire(DriveSource::WAYPOINT));
  EXPECT_TRUE(arbiter.ownsResource(DriveSource::WAYPOINT));
  EXPECT_FALSE(arbiter.wasRevoked(DriveSource::WAYPOINT));
}

TEST(DrivingResourceArbiterTest, VectorPreemptsWaypoint) {
  DrivingResourceArbiter arbiter(makeConfig());
  ASSERT_TRUE(arbiter.acquire(DriveSource::WAYPOINT));

  // Higher-priority vector preempts the waypoint route.
  EXPECT_TRUE(arbiter.canDrive(DriveSource::VECTOR));
  EXPECT_TRUE(arbiter.acquire(DriveSource::VECTOR));
  EXPECT_EQ(arbiter.currentHolder(), DriveSource::VECTOR);
  EXPECT_TRUE(arbiter.wasRevoked(DriveSource::WAYPOINT));
  EXPECT_FALSE(arbiter.ownsResource(DriveSource::WAYPOINT));
}

TEST(DrivingResourceArbiterTest, WaypointRejectedWhileVectorHolds) {
  DrivingResourceArbiter arbiter(makeConfig());
  ASSERT_TRUE(arbiter.acquire(DriveSource::VECTOR));

  // Lower-priority waypoint cannot preempt and is denied.
  EXPECT_FALSE(arbiter.canDrive(DriveSource::WAYPOINT));
  EXPECT_FALSE(arbiter.acquire(DriveSource::WAYPOINT));
  EXPECT_EQ(arbiter.currentHolder(), DriveSource::VECTOR);
}

TEST(DrivingResourceArbiterTest, ReleaseFreesResourceAndClearsRevoked) {
  DrivingResourceArbiter arbiter(makeConfig());
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
  DrivingResourceArbiter arbiter(makeConfig());
  EXPECT_TRUE(arbiter.acquire(DriveSource::VECTOR));
  EXPECT_TRUE(arbiter.acquire(DriveSource::VECTOR));
  EXPECT_EQ(arbiter.currentHolder(), DriveSource::VECTOR);
}

TEST(DrivingResourceArbiterTest, ConfigurablePriorityFlip) {
  // Flip priorities so waypoint outranks vector.
  DrivingResourceArbiter arbiter(makeConfig(/*localVector=*/10, /*localWaypoint=*/100));
  ASSERT_TRUE(arbiter.acquire(DriveSource::VECTOR));
  EXPECT_TRUE(arbiter.acquire(DriveSource::WAYPOINT));
  EXPECT_EQ(arbiter.currentHolder(), DriveSource::WAYPOINT);
  EXPECT_TRUE(arbiter.wasRevoked(DriveSource::VECTOR));
}

TEST(DrivingResourceArbiterTest, SafePreemptsEverything) {
  DrivingResourceArbiter arbiter(makeConfig());
  ASSERT_TRUE(arbiter.acquire(DriveSource::VECTOR));

  EXPECT_TRUE(arbiter.acquire(DriveSource::SAFE));
  EXPECT_EQ(arbiter.currentHolder(), DriveSource::SAFE);
  EXPECT_TRUE(arbiter.wasRevoked(DriveSource::VECTOR));
}

TEST(DrivingResourceArbiterTest, NothingPreemptsSafe) {
  DrivingResourceArbiter arbiter(makeConfig());
  ASSERT_TRUE(arbiter.acquire(DriveSource::SAFE));

  EXPECT_FALSE(arbiter.acquire(DriveSource::VECTOR));
  EXPECT_FALSE(arbiter.acquire(DriveSource::WAYPOINT));
  EXPECT_FALSE(arbiter.canDrive(DriveSource::VECTOR));
  EXPECT_EQ(arbiter.currentHolder(), DriveSource::SAFE);

  // Once safe mode releases, normal commands flow again.
  arbiter.release(DriveSource::SAFE);
  EXPECT_TRUE(arbiter.acquire(DriveSource::WAYPOINT));
}

TEST(DrivingResourceArbiterTest, RemoteWaypointPreemptsLocalVector) {
  // GIVEN: a local vector command holds the driving resource
  DrivingResourceArbiter arbiter(makeConfig());
  ASSERT_TRUE(arbiter.acquire(DriveSource::VECTOR, CommandClass::LOCAL));

  // WHEN: a remote waypoint command acquires (default remote waypoint 400 > local vector 100)
  EXPECT_TRUE(arbiter.canDrive(DriveSource::WAYPOINT, CommandClass::REMOTE));
  EXPECT_TRUE(arbiter.acquire(DriveSource::WAYPOINT, CommandClass::REMOTE));

  // THEN: the local vector holder is revoked, never RESOURCE_REJECTED
  EXPECT_EQ(arbiter.currentHolder(), DriveSource::WAYPOINT);
  EXPECT_TRUE(arbiter.wasRevoked(DriveSource::VECTOR));
}

TEST(DrivingResourceArbiterTest, LocalVectorDeniedWhileRemoteWaypointHolds) {
  // GIVEN: a remote waypoint command holds the driving resource
  DrivingResourceArbiter arbiter(makeConfig());
  ASSERT_TRUE(arbiter.acquire(DriveSource::WAYPOINT, CommandClass::REMOTE));

  // WHEN: a local vector command tries to acquire (local vector 100 < remote waypoint 400)
  // THEN: it is denied and the remote holder keeps the resource
  EXPECT_FALSE(arbiter.canDrive(DriveSource::VECTOR, CommandClass::LOCAL));
  EXPECT_FALSE(arbiter.acquire(DriveSource::VECTOR, CommandClass::LOCAL));
  EXPECT_EQ(arbiter.currentHolder(), DriveSource::WAYPOINT);
}

TEST(DrivingResourceArbiterTest, SafePreemptsRemote) {
  // GIVEN: a remote vector command holds the driving resource
  DrivingResourceArbiter arbiter(makeConfig());
  ASSERT_TRUE(arbiter.acquire(DriveSource::VECTOR, CommandClass::REMOTE));

  // WHEN: safe mode acquires
  EXPECT_TRUE(arbiter.acquire(DriveSource::SAFE));

  // THEN: safe wins (1000 > remote vector 500) and the remote holder is revoked
  EXPECT_EQ(arbiter.currentHolder(), DriveSource::SAFE);
  EXPECT_TRUE(arbiter.wasRevoked(DriveSource::VECTOR));
}

TEST(DrivingResourceArbiterTest, ReacquireRepricesGrantByClass) {
  // GIVEN: the vector provider holds the resource for a local session
  DrivingResourceArbiter arbiter(makeConfig());
  ASSERT_TRUE(arbiter.acquire(DriveSource::VECTOR, CommandClass::LOCAL));

  // WHEN: the same provider re-acquires for a new remote session
  ASSERT_TRUE(arbiter.acquire(DriveSource::VECTOR, CommandClass::REMOTE));

  // THEN: the grant is repriced (remote vector 500), so a remote waypoint (400) is now denied
  EXPECT_FALSE(arbiter.acquire(DriveSource::WAYPOINT, CommandClass::REMOTE));
  EXPECT_EQ(arbiter.currentHolder(), DriveSource::VECTOR);
}

}  // namespace arlcore::autopilot
