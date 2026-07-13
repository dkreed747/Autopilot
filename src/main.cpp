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

#include <csignal>
#include <string>

#include "AutopilotApp.h"
#include "AutopilotConfig.h"
#include "Logger.h"
#include "YamlConfigLoader.h"

namespace {
arlcore::autopilot::AutopilotApp* g_app = nullptr;

void handleSignal(int /*signal*/) {
  if (g_app != nullptr) {
    g_app->stop();
  }
}
}  // namespace

//! \brief Autopilot entry point: load config from YAML, construct the AutopilotConfig, hand it
//! to AutopilotApp::initialize(), then run the control loop.
int main(int argc, char** argv) {
  const std::string configPath = (argc > 1) ? argv[1] : "autopilot.yaml";

  arlcore::autopilot::AutopilotConfig config;
  if (!arlcore::autopilot::YamlConfigLoader::load(configPath, &config)) {
    UMAA_LOG_ERROR(util::SYSTEM_LOGGER, "Failed to load autopilot configuration from " << configPath)
    return 1;
  }

  arlcore::autopilot::AutopilotApp app;
  if (!app.initialize(config)) {
    UMAA_LOG_ERROR(util::SYSTEM_LOGGER, "Autopilot failed to initialize")
    return 1;
  }

  g_app = &app;
  std::signal(SIGINT, handleSignal);
  std::signal(SIGTERM, handleSignal);

  app.run();
  return 0;
}
