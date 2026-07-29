# Mission tools: UMAA consumer-side clients/monitors shared by the mission
# runner and the mission console. Depends only on autopilot::core so the tools
# can split into their own repos later.
# TODO(repo-split): move to dedicated repos consuming find_package(autopilot)
# plus per-tool subset configs.

add_library(mission-tools
  clients/ClientIdentity.cpp
  clients/ConstraintsClient.cpp
  clients/OperationalModeClient.cpp
  clients/VectorCommandClient.cpp
  clients/WaypointMissionClient.cpp
  monitors/VectorActivityMonitor.cpp
  monitors/WaypointActivityMonitor.cpp)
set_target_properties(mission-tools PROPERTIES
  VERSION ${PROJECT_VERSION}
  SOVERSION ${PROJECT_VERSION_MAJOR}
  POSITION_INDEPENDENT_CODE ON)
target_include_directories(mission-tools PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)
target_link_libraries(mission-tools PUBLIC autopilot::core)

# Mission runner: end-to-end waypoint mission driver (UMAA consumer side).
add_executable(mission_runner mission_runner.cpp)
target_link_libraries(mission_runner PRIVATE mission-tools)

# Mission console: live monitoring + mission-command web GUI (REST/SSE bridge
# to DDS), serving the vanilla-JS app in tools/web via the vendored httplib.
add_executable(mission_console mission_console.cpp)
target_include_directories(mission_console SYSTEM PRIVATE
  ${CMAKE_SOURCE_DIR}/third-party/httplib
  ${CMAKE_SOURCE_DIR}/third-party)
target_link_libraries(mission_console PRIVATE mission-tools Threads::Threads)

# The console GUI is served from a web root next to the binary. An always-run target (not a
# POST_BUILD step): web-only edits must land in the build dir even when the console binary is
# already up to date.
add_custom_target(console-web ALL
  COMMAND ${CMAKE_COMMAND} -E copy_directory
    ${CMAKE_CURRENT_SOURCE_DIR}/web ${CMAKE_BINARY_DIR}/web
  COMMENT "Mirroring tools/web into the build directory")
add_dependencies(mission_console console-web)

install(TARGETS mission-tools
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT runtime-tools
          NAMELINK_COMPONENT development
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT development)
install(TARGETS mission_console
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT runtime-console)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/web/
  DESTINATION ${CMAKE_INSTALL_DATADIR}/autopilot/web COMPONENT runtime-console)
install(TARGETS mission_runner
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT runtime-runner)
