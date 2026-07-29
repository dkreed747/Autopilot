#include <gtest/gtest.h>

#include <cstdint>

#include "autopilot/modes/DrivingResourceArbiter.hpp"

//! \brief Config with the default class split: safe > remote {500,400} > local {vector,waypoint}.
static arlcore::autopilot::ArbitrationConfig makeConfig(int32_t localVector = 100, int32_t localWaypoint = 10,
                                                        int32_t safe = 1000) {
  arlcore::autopilot::ArbitrationConfig config;
  config.local.vectorPriority = localVector;
  config.local.waypointPriority = localWaypoint;
  config.safePriority = safe;
  return config;
}

TEST(DrivingResourceArbiterTest, StartsUnowned) {
  // GIVEN: a freshly constructed arbiter
  arlcore::autopilot::DrivingResourceArbiter arbiter(makeConfig());

  // WHEN: no source has acquired the resource yet
  // THEN: the holder is NONE and any source may drive
  EXPECT_EQ(arbiter.currentHolder(), arlcore::autopilot::DriveSource::NONE);
  EXPECT_TRUE(arbiter.canDrive(arlcore::autopilot::DriveSource::VECTOR));
  EXPECT_TRUE(arbiter.canDrive(arlcore::autopilot::DriveSource::WAYPOINT));
}

TEST(DrivingResourceArbiterTest, WaypointAcquiresWhenFree) {
  // GIVEN: an arbiter with a free resource
  arlcore::autopilot::DrivingResourceArbiter arbiter(makeConfig());

  // WHEN: the waypoint source acquires
  EXPECT_TRUE(arbiter.acquire(arlcore::autopilot::DriveSource::WAYPOINT));

  // THEN: it owns the resource with no revoked flag
  EXPECT_TRUE(arbiter.ownsResource(arlcore::autopilot::DriveSource::WAYPOINT));
  EXPECT_FALSE(arbiter.wasRevoked(arlcore::autopilot::DriveSource::WAYPOINT));
}

TEST(DrivingResourceArbiterTest, VectorPreemptsWaypoint) {
  // GIVEN: the waypoint source holds the resource
  arlcore::autopilot::DrivingResourceArbiter arbiter(makeConfig());
  ASSERT_TRUE(arbiter.acquire(arlcore::autopilot::DriveSource::WAYPOINT));

  // WHEN: the higher-priority vector source acquires
  EXPECT_TRUE(arbiter.canDrive(arlcore::autopilot::DriveSource::VECTOR));
  EXPECT_TRUE(arbiter.acquire(arlcore::autopilot::DriveSource::VECTOR));

  // THEN: vector holds and the waypoint route is revoked
  EXPECT_EQ(arbiter.currentHolder(), arlcore::autopilot::DriveSource::VECTOR);
  EXPECT_TRUE(arbiter.wasRevoked(arlcore::autopilot::DriveSource::WAYPOINT));
  EXPECT_FALSE(arbiter.ownsResource(arlcore::autopilot::DriveSource::WAYPOINT));
}

TEST(DrivingResourceArbiterTest, WaypointRejectedWhileVectorHolds) {
  // GIVEN: the vector source holds the resource
  arlcore::autopilot::DrivingResourceArbiter arbiter(makeConfig());
  ASSERT_TRUE(arbiter.acquire(arlcore::autopilot::DriveSource::VECTOR));

  // WHEN: the lower-priority waypoint source tries to acquire
  // THEN: it cannot preempt, is denied, and vector keeps the resource
  EXPECT_FALSE(arbiter.canDrive(arlcore::autopilot::DriveSource::WAYPOINT));
  EXPECT_FALSE(arbiter.acquire(arlcore::autopilot::DriveSource::WAYPOINT));
  EXPECT_EQ(arbiter.currentHolder(), arlcore::autopilot::DriveSource::VECTOR);
}

TEST(DrivingResourceArbiterTest, ReleaseFreesResourceAndClearsRevoked) {
  // GIVEN: vector preempted waypoint and holds the resource
  arlcore::autopilot::DrivingResourceArbiter arbiter(makeConfig());
  ASSERT_TRUE(arbiter.acquire(arlcore::autopilot::DriveSource::WAYPOINT));
  ASSERT_TRUE(arbiter.acquire(arlcore::autopilot::DriveSource::VECTOR));
  ASSERT_TRUE(arbiter.wasRevoked(arlcore::autopilot::DriveSource::WAYPOINT));

  // WHEN: vector releases the resource
  arbiter.release(arlcore::autopilot::DriveSource::VECTOR);

  // THEN: the resource is free
  EXPECT_EQ(arbiter.currentHolder(), arlcore::autopilot::DriveSource::NONE);

  // WHEN: waypoint acquires again
  // THEN: the acquire succeeds and clears its revoked flag
  EXPECT_TRUE(arbiter.acquire(arlcore::autopilot::DriveSource::WAYPOINT));
  EXPECT_FALSE(arbiter.wasRevoked(arlcore::autopilot::DriveSource::WAYPOINT));
}

TEST(DrivingResourceArbiterTest, ReacquireBySameHolderIsIdempotent) {
  // GIVEN: the vector source holds the resource
  arlcore::autopilot::DrivingResourceArbiter arbiter(makeConfig());
  EXPECT_TRUE(arbiter.acquire(arlcore::autopilot::DriveSource::VECTOR));

  // WHEN: the same source acquires again
  EXPECT_TRUE(arbiter.acquire(arlcore::autopilot::DriveSource::VECTOR));

  // THEN: it still holds the resource
  EXPECT_EQ(arbiter.currentHolder(), arlcore::autopilot::DriveSource::VECTOR);
}

TEST(DrivingResourceArbiterTest, ConfigurablePriorityFlip) {
  // GIVEN: priorities flipped so waypoint outranks vector, with vector holding
  arlcore::autopilot::DrivingResourceArbiter arbiter(makeConfig(/*localVector=*/10, /*localWaypoint=*/100));
  ASSERT_TRUE(arbiter.acquire(arlcore::autopilot::DriveSource::VECTOR));

  // WHEN: the waypoint source acquires
  EXPECT_TRUE(arbiter.acquire(arlcore::autopilot::DriveSource::WAYPOINT));

  // THEN: waypoint holds and vector is revoked
  EXPECT_EQ(arbiter.currentHolder(), arlcore::autopilot::DriveSource::WAYPOINT);
  EXPECT_TRUE(arbiter.wasRevoked(arlcore::autopilot::DriveSource::VECTOR));
}

TEST(DrivingResourceArbiterTest, SafePreemptsEverything) {
  // GIVEN: the vector source holds the resource
  arlcore::autopilot::DrivingResourceArbiter arbiter(makeConfig());
  ASSERT_TRUE(arbiter.acquire(arlcore::autopilot::DriveSource::VECTOR));

  // WHEN: safe mode acquires
  EXPECT_TRUE(arbiter.acquire(arlcore::autopilot::DriveSource::SAFE));

  // THEN: safe holds and vector is revoked
  EXPECT_EQ(arbiter.currentHolder(), arlcore::autopilot::DriveSource::SAFE);
  EXPECT_TRUE(arbiter.wasRevoked(arlcore::autopilot::DriveSource::VECTOR));
}

TEST(DrivingResourceArbiterTest, NothingPreemptsSafe) {
  // GIVEN: safe mode holds the resource
  arlcore::autopilot::DrivingResourceArbiter arbiter(makeConfig());
  ASSERT_TRUE(arbiter.acquire(arlcore::autopilot::DriveSource::SAFE));

  // WHEN: vector and waypoint try to acquire
  // THEN: both are denied and safe keeps the resource
  EXPECT_FALSE(arbiter.acquire(arlcore::autopilot::DriveSource::VECTOR));
  EXPECT_FALSE(arbiter.acquire(arlcore::autopilot::DriveSource::WAYPOINT));
  EXPECT_FALSE(arbiter.canDrive(arlcore::autopilot::DriveSource::VECTOR));
  EXPECT_EQ(arbiter.currentHolder(), arlcore::autopilot::DriveSource::SAFE);

  // WHEN: safe mode releases
  arbiter.release(arlcore::autopilot::DriveSource::SAFE);

  // THEN: normal commands flow again
  EXPECT_TRUE(arbiter.acquire(arlcore::autopilot::DriveSource::WAYPOINT));
}

TEST(DrivingResourceArbiterTest, RemoteWaypointPreemptsLocalVector) {
  // GIVEN: a local vector command holds the driving resource
  arlcore::autopilot::DrivingResourceArbiter arbiter(makeConfig());
  ASSERT_TRUE(arbiter.acquire(arlcore::autopilot::DriveSource::VECTOR, arlcore::autopilot::CommandClass::LOCAL));

  // WHEN: a remote waypoint command acquires (default remote waypoint 400 > local vector 100)
  EXPECT_TRUE(arbiter.canDrive(arlcore::autopilot::DriveSource::WAYPOINT, arlcore::autopilot::CommandClass::REMOTE));
  EXPECT_TRUE(arbiter.acquire(arlcore::autopilot::DriveSource::WAYPOINT, arlcore::autopilot::CommandClass::REMOTE));

  // THEN: the local vector holder is revoked, never RESOURCE_REJECTED
  EXPECT_EQ(arbiter.currentHolder(), arlcore::autopilot::DriveSource::WAYPOINT);
  EXPECT_TRUE(arbiter.wasRevoked(arlcore::autopilot::DriveSource::VECTOR));
}

TEST(DrivingResourceArbiterTest, LocalVectorDeniedWhileRemoteWaypointHolds) {
  // GIVEN: a remote waypoint command holds the driving resource
  arlcore::autopilot::DrivingResourceArbiter arbiter(makeConfig());
  ASSERT_TRUE(arbiter.acquire(arlcore::autopilot::DriveSource::WAYPOINT, arlcore::autopilot::CommandClass::REMOTE));

  // WHEN: a local vector command tries to acquire (local vector 100 < remote waypoint 400)
  // THEN: it is denied and the remote holder keeps the resource
  EXPECT_FALSE(arbiter.canDrive(arlcore::autopilot::DriveSource::VECTOR, arlcore::autopilot::CommandClass::LOCAL));
  EXPECT_FALSE(arbiter.acquire(arlcore::autopilot::DriveSource::VECTOR, arlcore::autopilot::CommandClass::LOCAL));
  EXPECT_EQ(arbiter.currentHolder(), arlcore::autopilot::DriveSource::WAYPOINT);
}

TEST(DrivingResourceArbiterTest, SafePreemptsRemote) {
  // GIVEN: a remote vector command holds the driving resource
  arlcore::autopilot::DrivingResourceArbiter arbiter(makeConfig());
  ASSERT_TRUE(arbiter.acquire(arlcore::autopilot::DriveSource::VECTOR, arlcore::autopilot::CommandClass::REMOTE));

  // WHEN: safe mode acquires
  EXPECT_TRUE(arbiter.acquire(arlcore::autopilot::DriveSource::SAFE));

  // THEN: safe wins (1000 > remote vector 500) and the remote holder is revoked
  EXPECT_EQ(arbiter.currentHolder(), arlcore::autopilot::DriveSource::SAFE);
  EXPECT_TRUE(arbiter.wasRevoked(arlcore::autopilot::DriveSource::VECTOR));
}

TEST(DrivingResourceArbiterTest, ReacquireRepricesGrantByClass) {
  // GIVEN: the vector provider holds the resource for a local session
  arlcore::autopilot::DrivingResourceArbiter arbiter(makeConfig());
  ASSERT_TRUE(arbiter.acquire(arlcore::autopilot::DriveSource::VECTOR, arlcore::autopilot::CommandClass::LOCAL));

  // WHEN: the same provider re-acquires for a new remote session
  ASSERT_TRUE(arbiter.acquire(arlcore::autopilot::DriveSource::VECTOR, arlcore::autopilot::CommandClass::REMOTE));

  // THEN: the grant is repriced (remote vector 500), so a remote waypoint (400) is now denied
  EXPECT_FALSE(arbiter.acquire(arlcore::autopilot::DriveSource::WAYPOINT, arlcore::autopilot::CommandClass::REMOTE));
  EXPECT_EQ(arbiter.currentHolder(), arlcore::autopilot::DriveSource::VECTOR);
}
