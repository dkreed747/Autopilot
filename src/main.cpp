#include <csignal>
#include <string>

#include "autopilot/core/AutopilotApp.hpp"
#include "autopilot/config/AutopilotConfig.hpp"
#include "Logger.h"
#include "autopilot/config/YamlConfigLoader.hpp"

static arlcore::autopilot::AutopilotApp* g_app = nullptr;

static void handleSignal(int /*signal*/) {
  if (g_app != nullptr) {
    g_app->stop();
  }
}

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
