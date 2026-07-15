#---------------------------------------------------------------------------
#  Copyright 2025 Pennsylvania State University
#
#  Applied Research Laboratory
#  Pennsylvania State University
#  P.O. Box 30
#  State College, PA 16804-0030
#
# DISTRIBUTION STATEMENT A. Approved for public release.
# Distribution is unlimited.
# This software was developed by the Department of the Navy,
# NAVSEA Unmanned and Small Combatants. It is provided under the terms of
# use found in the LICENSE file at the source code root directory.
#---------------------------------------------------------------------------

# Autopilot application. Consumes UMAA MO Global Vector / Global Waypoint commands,
# deconflicts them, and drives a swappable vehicle-control strategy off the latest
# SA navigation data. Built on top of the umaa-cpp SDK.

cmake_minimum_required(VERSION 3.25)

project(autopilot VERSION 0.1.0
        DESCRIPTION "UMAA autopilot with mission-control web console"
        LANGUAGES CXX)

option(BUILD_SHARED_LIBS "Build shared libraries (governs the umaa-cpp submodule)" ON)
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

# ---------------------------------------------------------------------------
# The umaa-cpp SDK: pinned submodule by default, installed package on demand.
# Both modes provide the identical umaa-cpp::umaa-cpp target.
# ---------------------------------------------------------------------------
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

# ---------------------------------------------------------------------------
# Autopilot library (all logic, so unit tests can link against it directly)
# ---------------------------------------------------------------------------
file(GLOB_RECURSE AUTOPILOT_LIB_SOURCES CONFIGURE_DEPENDS
  ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
list(REMOVE_ITEM AUTOPILOT_LIB_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp)

add_library(autopilot-lib STATIC ${AUTOPILOT_LIB_SOURCES})
target_include_directories(autopilot-lib PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include/autopilot)
target_link_libraries(autopilot-lib PUBLIC
  umaa-cpp::umaa-cpp
  umaa_cyclone_cpp::umaa_cyclone_cpp)

# Main executable.
add_executable(autopilot ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp)
target_link_libraries(autopilot PRIVATE autopilot-lib)

# Copy runtime config next to the binary for convenience.
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config/autopilot.yaml
  ${CMAKE_BINARY_DIR}/autopilot.yaml COPYONLY)

# ---------------------------------------------------------------------------
# Mission tools: shared UMAA consumer-side mission code (route building +
# command session client), used by the mission runner and the mission console.
# ---------------------------------------------------------------------------
add_library(mission-tools STATIC
  ${CMAKE_CURRENT_SOURCE_DIR}/tools/WaypointMissionClient.cpp)
target_include_directories(mission-tools PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/tools)
target_link_libraries(mission-tools PUBLIC autopilot-lib)

# Mission runner: end-to-end waypoint mission driver (UMAA consumer side).
add_executable(mission_runner ${CMAKE_CURRENT_SOURCE_DIR}/tools/mission_runner.cpp)
target_link_libraries(mission_runner PRIVATE mission-tools)

# Mission console: live monitoring + mission-command web GUI (REST/SSE bridge
# to DDS), serving the vanilla-JS app in tools/web via the vendored httplib.
add_executable(mission_console ${CMAKE_CURRENT_SOURCE_DIR}/tools/mission_console.cpp)
target_include_directories(mission_console SYSTEM PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/third-party/httplib
  ${CMAKE_CURRENT_SOURCE_DIR}/third-party)
target_link_libraries(mission_console PRIVATE mission-tools Threads::Threads)

# The console GUI is served from a web root next to the binary.
add_custom_command(TARGET mission_console POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_directory
    ${CMAKE_CURRENT_SOURCE_DIR}/tools/web ${CMAKE_BINARY_DIR}/web)

# ---------------------------------------------------------------------------
# Tests (in-memory LocalReaderSender loopback IO from umaa-cpp::test-utils)
# ---------------------------------------------------------------------------
if(AUTOPILOT_BUILD_TESTS)
  find_package(GTest CONFIG REQUIRED)
  file(GLOB AUTOPILOT_TEST_SOURCES CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/test/*.cpp)
  if(AUTOPILOT_TEST_SOURCES)
    add_executable(autopilot_test ${AUTOPILOT_TEST_SOURCES})
    target_link_libraries(autopilot_test PRIVATE
      autopilot-lib
      umaa-cpp::test-utils
      GTest::gtest_main)
    add_test(NAME autopilot_test
      COMMAND autopilot_test --gtest_output=xml:test-reports/autopilot_test.xml)
  endif()
endif()

# ---------------------------------------------------------------------------
# Install (consumed by the application image): binaries + web root + config.
# The SDK submodule's install rules ride along and ship lib64/libumaa-cpp.so*.
# ---------------------------------------------------------------------------
install(TARGETS autopilot mission_runner mission_console
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT runtime)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/tools/web/
  DESTINATION ${CMAKE_INSTALL_DATADIR}/autopilot/web COMPONENT runtime)
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/config/autopilot.yaml
  DESTINATION ${CMAKE_INSTALL_DATADIR}/autopilot COMPONENT runtime)
