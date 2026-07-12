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
# SA navigation data. Built on top of the umaa-sdk-common / umaa++ libraries.

# The autopilot is built at C++23 to match the SDK (its public headers are C++23),
# while the code itself favours conservative, C++17-flavoured idioms.
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_subdirectory(umaa-cpp)

set(AUTOPILOT_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include/autopilot)

# Library holding all autopilot logic (so unit tests can link against it directly).
file(GLOB_RECURSE AUTOPILOT_LIB_SOURCES CONFIGURE_DEPENDS
  ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
list(REMOVE_ITEM AUTOPILOT_LIB_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp)

add_library(autopilot-lib ${AUTOPILOT_LIB_SOURCES})

target_include_directories(autopilot-lib PUBLIC
  ${AUTOPILOT_INCLUDE_DIR}
  ${BASE_INCLUDE_DIRS}
  ${CYCLONE_INCLUDE_DIRS}
  ${GEOGRAPHIC_LIB_INCLUDES})

# UMAA generated headers are included by autopilot-lib's public headers, so expose them
# (as system includes) to anything that links the lib, including the test target.
target_include_directories(autopilot-lib SYSTEM PUBLIC ${UMAA_INCLUDES})

target_compile_options(autopilot-lib PRIVATE -fPIC -fpermissive)
target_link_libraries(autopilot-lib PUBLIC umaa-sdk-common umaa++ ${COMMON_LIB_DEPS})

# Main executable.
add_executable(autopilot ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp)
target_compile_options(autopilot PRIVATE -fPIC -fpermissive)
target_link_libraries(autopilot PRIVATE autopilot-lib)

# Copy runtime config next to the binary for convenience.
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config/autopilot.yaml
  ${CMAKE_BINARY_DIR}/autopilot.yaml COPYONLY)

# Shared UMAA consumer-side mission code (route building + command session client),
# used by the mission runner and the mission console.
add_library(mission-tools STATIC
  ${CMAKE_CURRENT_SOURCE_DIR}/tools/MissionRoute.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/tools/WaypointMissionClient.cpp)
target_include_directories(mission-tools PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/tools)
target_compile_options(mission-tools PRIVATE -fPIC -fpermissive)
target_link_libraries(mission-tools PUBLIC autopilot-lib)

# Mission runner: end-to-end waypoint mission driver (UMAA consumer side).
add_executable(mission_runner ${CMAKE_CURRENT_SOURCE_DIR}/tools/mission_runner.cpp)
target_compile_options(mission_runner PRIVATE -fPIC -fpermissive)
target_link_libraries(mission_runner PRIVATE mission-tools)

# Mission console: live monitoring + mission-command web GUI (REST/SSE bridge to DDS).
add_executable(mission_console ${CMAKE_CURRENT_SOURCE_DIR}/tools/mission_console.cpp)
target_include_directories(mission_console SYSTEM PRIVATE
  ${PROJECT_SOURCE_DIR}/third-party/httplib
  ${PROJECT_SOURCE_DIR}/third-party)
target_compile_options(mission_console PRIVATE -fPIC -fpermissive)
target_link_libraries(mission_console PRIVATE mission-tools)
find_package(Threads REQUIRED)
target_link_libraries(mission_console PRIVATE Threads::Threads)

# The console GUI is served from a web root next to the binary.
add_custom_command(TARGET mission_console POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_directory
    ${CMAKE_CURRENT_SOURCE_DIR}/tools/web ${CMAKE_BINARY_DIR}/web)

# Unit/integration tests reuse the SDK's GoogleTest + LocalReaderSender loopback IO,
# which are only available when the SDK test tree is enabled.
if(TEST_COMMON OR TEST_CYCLONE)
  file(GLOB AUTOPILOT_TEST_SOURCES CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/test/*.cpp)
  if(AUTOPILOT_TEST_SOURCES)
    add_executable(autopilot_test ${AUTOPILOT_TEST_SOURCES})
    target_include_directories(autopilot_test PRIVATE
      ${AUTOPILOT_INCLUDE_DIR}
      ${PROJECT_SOURCE_DIR}/test/io/dds/util
      ${UMAA_INCLUDES})
    target_compile_options(autopilot_test PRIVATE -fPIC -fpermissive)
    # gtest_main is provided by the SDK test tree (added before this subdirectory).
    target_link_libraries(autopilot_test PRIVATE autopilot-lib gtest_main)
    add_test(NAME autopilot_test COMMAND autopilot_test)
  endif()
endif()
