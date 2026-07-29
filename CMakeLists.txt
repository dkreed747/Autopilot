# Autopilot application. Consumes UMAA MO Global Vector / Global Waypoint commands,
# deconflicts them, and drives a swappable vehicle-control strategy off the latest
# SA navigation data. Built on top of the umaa-cpp SDK.

cmake_minimum_required(VERSION 3.25)

project(autopilot VERSION 1.0.0
        DESCRIPTION "UMAA autopilot with mission-control web console"
        LANGUAGES CXX)

option(BUILD_SHARED_LIBS "Build shared libraries" ON)
option(AUTOPILOT_BUILD_TESTS "Build autopilot tests" OFF)
option(AUTOPILOT_USE_SYSTEM_UMAA_CPP "Use an installed umaa-cpp instead of the submodule" OFF)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# Relocatable install: binaries find the SDK lib wherever the prefix lands
# (bin/ next to lib64/, same convention as the umaa-cyclone-cpp package).
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH ON)
set(CMAKE_INSTALL_RPATH "$ORIGIN;$ORIGIN/../lib;$ORIGIN/../lib64")

include(GNUInstallDirs)

if(AUTOPILOT_BUILD_TESTS)
  enable_testing()
endif()

# The umaa-cpp SDK: pinned submodule by default, installed package on demand.
# Both modes provide the identical umaa-cpp::umaa-cpp target.
if(AUTOPILOT_USE_SYSTEM_UMAA_CPP)
  if(AUTOPILOT_BUILD_TESTS)
    message(FATAL_ERROR "autopilot tests need the SDK submodule "
      "(umaa-cpp::test-utils is not part of the installed package)")
  endif()
  find_package(umaa-cpp CONFIG REQUIRED)
else()
  if(NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/umaa-cpp/CMakeLists.txt)
    message(FATAL_ERROR
      "umaa-cpp submodule missing. Run: git submodule update --init --recursive")
  endif()
  if(AUTOPILOT_BUILD_TESTS)
    set(UMAA_CPP_BUILD_TESTS ON CACHE BOOL "" FORCE)   # also provides umaa-cpp::test-utils
  endif()
  add_subdirectory(umaa-cpp)
endif()

find_package(umaa_cyclone_cpp REQUIRED)   # autopilot sources include UMAA/... directly
find_package(Threads REQUIRED)

# Mission core: the slice the mission tools consume (config, route/guidance
# planning, zone geometry). Exported so the tools can move to their own repos
# later and find_package it. TODO(repo-split): publish as its own package.
add_library(autopilot-core
  src/config/ConfigValidation.cpp
  src/config/YamlConfigLoader.cpp
  src/guidance/DubinsPath.cpp
  src/guidance/DubinsPathPlanner.cpp
  src/guidance/DubinsRrtStar.cpp
  src/guidance/MissionRoute.cpp
  src/guidance/ToleranceUtils.cpp
  src/safety/ZoneGeometry.cpp
  src/safety/ZoneMap.cpp)
add_library(autopilot::core ALIAS autopilot-core)
set_target_properties(autopilot-core PROPERTIES
  VERSION ${PROJECT_VERSION}
  SOVERSION ${PROJECT_VERSION_MAJOR}
  POSITION_INDEPENDENT_CODE ON
  EXPORT_NAME core)
target_include_directories(autopilot-core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(autopilot-core PUBLIC
  umaa-cpp::umaa-cpp
  umaa_cyclone_cpp::umaa_cyclone_cpp)

# Application logic on top of the core: brain, supervisor, providers, modes,
# vehicle strategies.
add_library(autopilot-app
  src/core/AutopilotApp.cpp
  src/core/AutopilotBrain.cpp
  src/modes/DrivingResourceArbiter.cpp
  src/modes/OperationalModeControlProvider.cpp
  src/modes/OperationalModeManager.cpp
  src/safety/ConstraintClamp.cpp
  src/safety/ConstraintSupervisor.cpp
  src/safety/RecoveryGuidance.cpp
  src/safety/SafeModeStrategyFactory.cpp
  src/safety/SafeReturnPath.cpp
  src/safety/VectorZoneGuidance.cpp
  src/umaa/PlatformReportFactory.cpp
  src/umaa/VectorControlServiceProvider.cpp
  src/umaa/WaypointControlServiceProvider.cpp
  src/vehicle/SimVehicleControl.cpp)
add_library(autopilot::app ALIAS autopilot-app)
set_target_properties(autopilot-app PROPERTIES
  VERSION ${PROJECT_VERSION}
  SOVERSION ${PROJECT_VERSION_MAJOR}
  POSITION_INDEPENDENT_CODE ON
  EXPORT_NAME app)
target_link_libraries(autopilot-app PUBLIC autopilot::core)

add_executable(autopilot src/main.cpp)
target_link_libraries(autopilot PRIVATE autopilot::app)

# Copy runtime config next to the binary for convenience.
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config/autopilot.yaml
  ${CMAKE_BINARY_DIR}/autopilot.yaml COPYONLY)

add_subdirectory(tools)

# Tests (in-memory LocalReaderSender loopback IO from umaa-cpp::test-utils).
if(AUTOPILOT_BUILD_TESTS)
  find_package(GTest CONFIG REQUIRED)
  add_executable(autopilot_test
    test/AutopilotBrainSafetyTest.cpp
    test/ConstrainedPlannerTest.cpp
    test/ConstraintClampTest.cpp
    test/ConstraintSupervisorTest.cpp
    test/DrivingResourceArbiterTest.cpp
    test/DubinsPathPlannerTest.cpp
    test/DubinsPathTest.cpp
    test/DubinsRrtStarTest.cpp
    test/OperationalModeControlProviderTest.cpp
    test/OperationalModeGatingTest.cpp
    test/OperationalModeManagerTest.cpp
    test/PlatformReportFactoryTest.cpp
    test/RecoveryGuidanceTest.cpp
    test/SafeReturnPathTest.cpp
    test/SafetyEndToEndTest.cpp
    test/SimVehicleControlTest.cpp
    test/VectorZoneGuidanceTest.cpp
    test/YamlConfigLoaderTest.cpp
    test/ZoneGeometryTest.cpp
    test/ZoneMapTest.cpp)
  target_link_libraries(autopilot_test PRIVATE
    autopilot::app
    umaa-cpp::test-utils
    GTest::gtest_main
    GTest::gmock)
  add_test(NAME autopilot_test
    COMMAND autopilot_test --gtest_output=xml:test-reports/autopilot_test.xml)
endif()

# Install: shared runtime component (core lib + config) plus per-application
# components consumed by the container images.
install(TARGETS autopilot-core EXPORT autopilotTargets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT runtime
          NAMELINK_COMPONENT development
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT development)
install(TARGETS autopilot-app EXPORT autopilotTargets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT runtime-autopilot
          NAMELINK_COMPONENT development
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT development)
install(TARGETS autopilot
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT runtime-autopilot)
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/config/autopilot.yaml
  DESTINATION ${CMAKE_INSTALL_DATADIR}/autopilot COMPONENT runtime)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/autopilot
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} COMPONENT development
  FILES_MATCHING PATTERN "*.hpp")
install(EXPORT autopilotTargets
  FILE autopilotTargets.cmake
  NAMESPACE autopilot::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/autopilot
  COMPONENT development)

include(CMakePackageConfigHelpers)
configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/autopilotConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/autopilotConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/autopilot)
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/autopilotConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion)
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/autopilotConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/autopilotConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/autopilot
  COMPONENT development)
