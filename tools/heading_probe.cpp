//! \brief Identifies the platform's inner heading loop from commanded-heading steps over the
//! UMAA Global Vector control service: the proportional gain, the turn-rate envelope, the
//! error at which the loop saturates, and the closed-loop lag. The path tracker's curvature
//! feedforward is sized from the reported time constant, so this runs before tuning (see
//! tools/README.md).

#include <UMAA/SA/GlobalPoseStatus/GlobalPoseReportType.hpp>
#include <UMAA/SA/SpeedStatus/SpeedReportType.hpp>
#include <UMAA/SA/VelocityStatus/VelocityReportType.hpp>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <filesystem>  // NOLINT(build/c++17)
#include <fstream>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "CycloneQosProviderWrapper.h"
#include "CycloneReader.h"
#include "CycloneUtilities.h"
#include "InternalTypes.h"
#include "UuidFactory.h"
#include "autopilot/config/AutopilotConfig.hpp"
#include "autopilot/config/ConfigValidation.hpp"
#include "autopilot/config/YamlConfigLoader.hpp"
#include "autopilot/guidance/AngleMath.hpp"
#include "autopilot/guidance/HeadingLoopIdentifier.hpp"
#include "clients/ClientIdentity.hpp"
#include "clients/VectorCommandClient.hpp"

using arlcore::autopilot::HeadingLoopModel;
using arlcore::autopilot::HeadingProbeSample;
using arlcore::autopilot::tools::VectorCommandClient;
using arlcore::autopilot::tools::VectorSetpoint;
using arlcore::io::CycloneReader;
using arlcore::io::ReadStatus;
using UMAA::SA::GlobalPoseStatus::GlobalPoseReportType;
using UMAA::SA::SpeedStatus::SpeedReportType;
using UMAA::SA::VelocityStatus::VelocityReportType;

static std::atomic<bool> g_stop{false};

static void handleSignal(int /*signal*/) { g_stop = true; }

//! \brief Everything the probe run is parameterised by.
struct ProbeOptions {
  std::string configPath = "autopilot.yaml";
  std::string outDir = "probe-out";
  std::optional<flt64_t> speedMps;
  std::vector<flt64_t> scheduleRad;
  flt64_t dwellMaxS = 45.0;
  flt64_t settleS = 5.0;
  flt64_t warmupS = 20.0;
  std::optional<flt64_t> elevValueM;
  std::string elevFrame = "depth";
  flt64_t verifyTol = 0.0;
  bool dryRun = false;
};

//! \brief Signed heading deltas in degrees; alternating and summing to zero so the vehicle
//! ends near its starting heading, and spanning both the unsaturated and saturated regions.
static std::vector<flt64_t> defaultScheduleRad() {
  const flt64_t deg = M_PI / 180.0;
  std::vector<flt64_t> out;
  for (const flt64_t d : {5.0, -5.0, 10.0, -10.0, 20.0, -20.0, 40.0, -40.0, 90.0, -90.0, 5.0, -5.0}) {
    out.push_back(d * deg);
  }
  return out;
}

static bool matchOption(const std::string& arg, const char* name, std::string* value) {
  const std::string prefix = std::string("--") + name + "=";
  if (arg.rfind(prefix, 0) != 0) {
    return false;
  }
  *value = arg.substr(prefix.size());
  return true;
}

//! \brief Parse a comma-separated degree list into radians; false on any malformed entry.
static bool parseSchedule(const std::string& text, std::vector<flt64_t>* out) {
  const flt64_t deg = M_PI / 180.0;
  std::size_t start = 0;
  while (start <= text.size()) {
    const std::size_t comma = text.find(',', start);
    const std::string field = text.substr(start, comma == std::string::npos ? std::string::npos : comma - start);
    if (field.empty()) {
      return false;
    }
    try {
      out->push_back(std::stod(field) * deg);
    } catch (const std::exception&) {
      return false;
    }
    if (comma == std::string::npos) {
      break;
    }
    start = comma + 1;
  }
  return !out->empty();
}

static bool parseArgs(int argc, char** argv, ProbeOptions* options) {
  int32_t positional = 0;
  for (int32_t i = 1; i < argc; i++) {
    const std::string arg = argv[i];
    std::string value;
    if (arg.rfind("--", 0) != 0) {
      if (positional == 0) {
        options->configPath = arg;
      } else if (positional == 1) {
        options->outDir = arg;
      } else {
        std::cerr << "unexpected argument '" << arg << "'" << std::endl;
        return false;
      }
      positional++;
    } else if (arg == "--dry-run") {
      options->dryRun = true;
    } else if (matchOption(arg, "speed", &value)) {
      options->speedMps = std::stod(value);
    } else if (matchOption(arg, "schedule", &value)) {
      options->scheduleRad.clear();
      if (!parseSchedule(value, &options->scheduleRad)) {
        std::cerr << "malformed --schedule (expected a comma-separated list of degrees)" << std::endl;
        return false;
      }
    } else if (matchOption(arg, "dwell-max", &value)) {
      options->dwellMaxS = std::stod(value);
    } else if (matchOption(arg, "settle", &value)) {
      options->settleS = std::stod(value);
    } else if (matchOption(arg, "warmup", &value)) {
      options->warmupS = std::stod(value);
    } else if (matchOption(arg, "depth", &value)) {
      options->elevValueM = std::stod(value);
      options->elevFrame = "depth";
    } else if (matchOption(arg, "asf", &value)) {
      options->elevValueM = std::stod(value);
      options->elevFrame = "asf";
    } else if (matchOption(arg, "verify-tol", &value)) {
      options->verifyTol = std::stod(value);
    } else {
      std::cerr << "unknown option '" << arg << "'" << std::endl;
      return false;
    }
  }
  if (options->scheduleRad.empty()) {
    options->scheduleRad = defaultScheduleRad();
  }
  return true;
}

static void writeIdentification(const std::string& path, const HeadingLoopModel& model, flt64_t speedMps,
                                const arlcore::autopilot::AutopilotConfig& config, std::size_t sampleCount,
                                const std::string& omegaSource) {
  const std::optional<flt64_t> configOmegaMax = config.platformCapabilities.surface.maxTurnRateRps;
  std::ofstream out(path);
  out.precision(10);
  out << "key,value\n";
  out << "speed_mps," << speedMps << "\n";
  out << "n_samples," << sampleCount << "\n";
  out << "omega_source," << omegaSource << "\n";
  out << "K_rps_per_rad," << model.gainRpsPerRad << "\n";
  out << "K_r2," << model.gainR2 << "\n";
  out << "K_n," << model.gainSamples << "\n";
  out << "K_port_rps_per_rad," << model.gainPortRpsPerRad << "\n";
  out << "K_stbd_rps_per_rad," << model.gainStbdRpsPerRad << "\n";
  out << "asymmetry_frac," << model.asymmetryFrac << "\n";
  out << "omega_max_rps," << model.omegaMaxRps << "\n";
  out << "omega_max_cv," << model.omegaMaxCv << "\n";
  out << "omega_max_windows," << model.omegaMaxWindows << "\n";
  // A gain that could not be identified is written blank, never as zero: a zero here would
  // read as a valid feedforward time constant if pasted into the config.
  const bool identified = model.gainRpsPerRad > 0.0;
  out << "e_sat_rad," << (identified ? std::to_string(model.satErrRad) : "") << "\n";
  out << "e_sat_deg," << (identified ? std::to_string(model.satErrRad * 180.0 / M_PI) : "") << "\n";
  out << "e_sat_knee_rad," << model.satErrKneeRad << "\n";
  out << "heading_loop_tau_s," << (identified ? std::to_string(model.headingLoopTauS) : "") << "\n";
  out << "decay_tau_s," << model.decayTauS << "\n";
  out << "model_consistency," << model.modelConsistency << "\n";
  out << "config_max_turn_rate_rps," << (configOmegaMax.has_value() ? configOmegaMax.value() : 0.0) << "\n";
  if (configOmegaMax.has_value() && configOmegaMax.value() > 0.0) {
    out << "omega_max_ratio_to_config," << model.omegaMaxRps / configOmegaMax.value() << "\n";
  }
  if (model.omegaMaxRps > 0.0) {
    out << "R_min_measured_m," << speedMps / model.omegaMaxRps << "\n";
  }
  out << "suggested_heading_loop_tau_s," << (identified ? std::to_string(model.headingLoopTauS) : "") << "\n";
  out << "suggested_ff_limit_rad," << (identified ? std::to_string(0.8 * model.satErrRad) : "") << "\n";
  out << "verdict," << model.verdict << "\n";
}

static void printModel(const HeadingLoopModel& model, flt64_t speedMps,
                       const arlcore::autopilot::AutopilotConfig& config) {
  const std::optional<flt64_t> configOmegaMax = config.platformCapabilities.surface.maxTurnRateRps;
  const bool identified = model.gainRpsPerRad > 0.0;
  const std::string unknown = "not identifiable";
  std::cout << "\n=== identified heading loop (verdict " << model.verdict << ") ===" << std::endl;
  std::cout << "  K              = ";
  if (identified) {
    std::cout << model.gainRpsPerRad << " rad/s per rad";
  } else {
    std::cout << unknown;
  }
  std::cout << "  (R^2=" << model.gainR2 << ", n=" << model.gainSamples << ")" << std::endl;
  std::cout << "  omega_max      = " << model.omegaMaxRps << " rad/s  (" << model.omegaMaxWindows
            << " windows, cv=" << model.omegaMaxCv << ")" << std::endl;
  std::cout << "  e_sat          = ";
  if (identified) {
    std::cout << model.satErrRad << " rad = " << model.satErrRad * 180.0 / M_PI << " deg";
  } else {
    std::cout << unknown;
  }
  std::cout << "   (knee estimate " << model.satErrKneeRad << " rad)" << std::endl;
  std::cout << "  tau (1/K)      = ";
  if (identified) {
    std::cout << model.headingLoopTauS << " s   <- planner.tracker.heading_loop_tau_s";
  } else {
    std::cout << unknown << "   (do NOT configure a time constant from this run)";
  }
  std::cout << std::endl;
  std::cout << "  decay tau      = " << model.decayTauS << " s   (consistency " << model.modelConsistency
            << ", 1.0 = first order)" << std::endl;
  if (identified) {
    std::cout << "  port/stbd K    = " << model.gainPortRpsPerRad << " / " << model.gainStbdRpsPerRad
              << "   (asymmetry " << model.asymmetryFrac * 100.0 << "%)" << std::endl;
  }
  if (model.omegaMaxRps > 0.0) {
    std::cout << "  R_min measured = " << speedMps / model.omegaMaxRps << " m at " << speedMps << " m/s" << std::endl;
  }
  if (configOmegaMax.has_value() && configOmegaMax.value() > 0.0 && model.omegaMaxRps > 0.0) {
    const flt64_t ratio = model.omegaMaxRps / configOmegaMax.value();
    std::cout << "  vs config max_turn_rate_rps=" << configOmegaMax.value() << ": ratio " << ratio << std::endl;
    if (std::fabs(ratio - 1.0) > 0.10) {
      std::cout << "  WARNING: the configured turn rate disagrees with the measured one by more than 10%."
                << " Every planned turn radius is derived from it, so fix the config before tuning." << std::endl;
    }
  }
  if (model.asymmetryFrac > 0.15) {
    std::cout << "  WARNING: port and starboard gains differ by more than 15%." << std::endl;
  }
  if (model.verdict == "SATURATION_DOMINATED") {
    std::cout << "  NOTE: almost every sample sits against the rate limit, so there is no linear region"
              << " to fit. Trust omega_max and the knee, not K." << std::endl;
  }
}

int main(int argc, char** argv) {
  ProbeOptions options;
  if (!parseArgs(argc, argv, &options)) {
    return 1;
  }

  arlcore::autopilot::AutopilotConfig config;
  if (!arlcore::autopilot::YamlConfigLoader::load(options.configPath, &config)) {
    std::cerr << "Failed to load config " << options.configPath << std::endl;
    return 1;
  }
  if (!arlcore::autopilot::isValidUuid(config.identity.vectorSourceId)) {
    std::cerr << "heading_probe requires a valid UUID for identity.vector_source_id (got '"
              << config.identity.vectorSourceId << "')" << std::endl;
    return 1;
  }
  const flt64_t speedMps = options.speedMps.has_value()
                               ? options.speedMps.value()
                               : config.platformCapabilities.surface.cruisingSpeedMps.value_or(3.0);
  if (speedMps <= 0.0) {
    std::cerr << "probe speed must be positive (got " << speedMps << ")" << std::endl;
    return 1;
  }

  flt64_t totalS = options.warmupS;
  for (std::size_t i = 0; i < options.scheduleRad.size(); i++) {
    totalS += options.dwellMaxS + options.settleS;
  }
  std::cout << "heading_probe: " << options.scheduleRad.size() << " steps at " << speedMps << " m/s, up to " << totalS
            << " s and roughly " << speedMps * totalS << " m of ground track." << std::endl;
  std::cout << "Ensure no zones or constraints are active: vector-mode zone guidance rewrites the"
            << " commanded heading and would invalidate the identification." << std::endl;
  if (options.dryRun) {
    return 0;
  }
  std::filesystem::create_directories(options.outDir);

  auto participant = arlcore::io::getDomainParticipant(config.dds.domainId);
  arlcore::io::CycloneQosProviderWrapper qosProvider(config.dds.qosFile, config.dds.domainQosProfile);
  const auto rqos = qosProvider.datareader_qos();
  const auto wqos = qosProvider.datawriter_qos();

  auto poseReader = std::make_shared<CycloneReader<GlobalPoseReportType>>(
      participant, UMAA::SA::GlobalPoseStatus::GlobalPoseReportTypeTopic, rqos);
  auto speedReader =
      std::make_shared<CycloneReader<SpeedReportType>>(participant, UMAA::SA::SpeedStatus::SpeedReportTypeTopic, rqos);
  auto velocityReader = std::make_shared<CycloneReader<VelocityReportType>>(
      participant, UMAA::SA::VelocityStatus::VelocityReportTypeTopic, rqos);

  // Like mission_runner, the probe acts as this platform's onboard autonomy so its commands
  // classify LOCAL and can drive the autopilot into AUTONOMOUS.
  VectorCommandClient client(participant, wqos, rqos,
                             arlcore::UuidFactory::getInstance().parseGuidFromString(config.identity.vectorSourceId),
                             arlcore::autopilot::tools::makeLocalAutonomyIdentity(config.identity));

  std::signal(SIGINT, handleSignal);
  std::signal(SIGTERM, handleSignal);

  std::this_thread::sleep_for(std::chrono::seconds(2));

  // Wait for a first fix so the schedule starts from the vehicle's actual heading.
  GlobalPoseReportType pose;
  flt64_t heading = 0.0;
  bool havePose = false;
  for (int32_t i = 0; i < 100 && !havePose && !g_stop; i++) {
    if (poseReader->readLatest(&pose) == ReadStatus::SUCCESS) {
      heading = pose.attitude().yaw().yaw();
      havePose = true;
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  if (!havePose) {
    std::cerr << "No Global Pose report within 10 s; is the platform running?" << std::endl;
    return 1;
  }

  VectorSetpoint setpoint;
  setpoint.headingRad = heading;
  setpoint.speedMps = speedMps;
  setpoint.elevValueM = options.elevValueM;
  setpoint.elevFrame = options.elevFrame;
  // A short end time is a deadman: a probe that dies stops the vehicle within 5 s.
  setpoint.timeoutS = 5.0;
  client.start(setpoint);
  if (!client.active()) {
    std::cerr << "Failed to publish the vector command" << std::endl;
    return 1;
  }

  std::ofstream probeCsv(options.outDir + "/probe.csv");
  probeCsv.precision(10);
  probeCsv << "elapsed_s,phase,step_index,step_deg,cmd_heading_rad,yaw_rad,heading_err_rad,"
           << "yaw_rate_rps,yaw_rate_fd_rps,sog_mps,lat_deg,lon_deg\n";

  std::vector<HeadingProbeSample> samples;
  const auto start = std::chrono::steady_clock::now();
  auto lastCommandAt = start;
  flt64_t commanded = heading;
  flt64_t lastSpeed = 0.0;
  std::optional<flt64_t> lastVelYawRate;
  std::chrono::steady_clock::time_point lastVelAt;
  std::optional<flt64_t> prevYaw;
  std::optional<flt64_t> prevYawAtS;
  int32_t reportedRateSamples = 0;
  int32_t totalSamples = 0;
  bool aborted = false;

  // phase -1 is the warmup at the starting heading; then each step runs STEP until settled or
  // the dwell cap, followed by a fixed SETTLE at the new heading.
  int32_t stepIndex = -1;
  flt64_t stepRad = 0.0;
  auto phaseStart = start;
  bool inSettle = false;
  std::optional<std::chrono::steady_clock::time_point> settledSince;

  while (!g_stop) {
    const auto now = std::chrono::steady_clock::now();
    const flt64_t elapsed = std::chrono::duration<flt64_t>(now - start).count();
    const flt64_t phaseElapsed = std::chrono::duration<flt64_t>(now - phaseStart).count();

    if (std::chrono::duration<flt64_t>(now - lastCommandAt).count() >= 0.5) {
      setpoint.headingRad = commanded;
      client.update(setpoint);
      lastCommandAt = now;
    }
    for (const auto& update : client.pollStatus()) {
      if (update.terminal && update.status != "COMPLETED") {
        std::cerr << "[" << elapsed << "s] command went terminal: " << update.status << " (" << update.logMessage << ")"
                  << std::endl;
        aborted = true;
      }
    }
    if (aborted) {
      break;
    }

    SpeedReportType speed;
    if (speedReader->readLatest(&speed) == ReadStatus::SUCCESS && speed.speedOverGround().has_value()) {
      lastSpeed = speed.speedOverGround().value();
    }
    VelocityReportType velocity;
    if (velocityReader->readLatest(&velocity) == ReadStatus::SUCCESS) {
      lastVelYawRate = velocity.attitudeRate().yawRate();
      lastVelAt = now;
    }

    // Polling faster than the publisher, with a taking read, yields exactly one row per
    // published pose and an exact sample interval.
    if (poseReader->readLatest(&pose) == ReadStatus::SUCCESS) {
      const flt64_t yaw = pose.attitude().yaw().yaw();
      const flt64_t err = arlcore::autopilot::wrapPi(commanded - yaw);
      std::optional<flt64_t> fdRate;
      if (prevYaw.has_value() && prevYawAtS.has_value() && elapsed > prevYawAtS.value()) {
        fdRate = arlcore::autopilot::wrapPi(yaw - prevYaw.value()) / (elapsed - prevYawAtS.value());
      }
      prevYaw = yaw;
      prevYawAtS = elapsed;

      const bool velFresh = lastVelYawRate.has_value() && std::chrono::duration<flt64_t>(now - lastVelAt).count() < 0.2;
      const char* phase = stepIndex < 0 ? "WARMUP" : (inSettle ? "SETTLE" : "STEP");
      probeCsv << elapsed << "," << phase << "," << stepIndex << "," << stepRad * 180.0 / M_PI << "," << commanded
               << "," << yaw << "," << err << ",";
      if (velFresh) {
        probeCsv << lastVelYawRate.value();
      }
      probeCsv << ",";
      if (fdRate.has_value()) {
        probeCsv << fdRate.value();
      }
      probeCsv << "," << lastSpeed << "," << pose.position().geodeticLatitude() << ","
               << pose.position().geodeticLongitude() << "\n";

      if (stepIndex >= 0) {
        totalSamples++;
        reportedRateSamples += velFresh ? 1 : 0;
        HeadingProbeSample sample;
        sample.elapsedS = elapsed;
        sample.headingErrRad = err;
        sample.yawRateRps = velFresh ? lastVelYawRate.value() : fdRate.value_or(0.0);
        sample.speedMps = lastSpeed;
        sample.stepIndex = stepIndex;
        sample.stepRad = stepRad;
        samples.push_back(sample);
      }

      // Settled means the error has stayed inside the tolerance for two continuous seconds;
      // a fixed dwell cannot work when the turn rate is the unknown being measured.
      const flt64_t tolerance = std::max(0.5 * M_PI / 180.0, 0.1 * std::fabs(stepRad));
      if (std::fabs(err) <= tolerance) {
        if (!settledSince.has_value()) {
          settledSince = now;
        }
      } else {
        settledSince.reset();
      }
    }

    const bool settled =
        settledSince.has_value() && std::chrono::duration<flt64_t>(now - settledSince.value()).count() >= 2.0;
    const bool warmupDone = stepIndex < 0 && phaseElapsed >= options.warmupS;
    const bool stepDone = stepIndex >= 0 && !inSettle && (settled || phaseElapsed >= options.dwellMaxS);
    const bool settleDone = inSettle && phaseElapsed >= options.settleS;

    if (warmupDone || settleDone) {
      const std::size_t next = static_cast<std::size_t>(stepIndex + 1);
      if (next >= options.scheduleRad.size()) {
        break;
      }
      stepIndex = static_cast<int32_t>(next);
      stepRad = options.scheduleRad[next];
      commanded = arlcore::autopilot::wrapPi(commanded + stepRad);
      inSettle = false;
      phaseStart = now;
      settledSince.reset();
      std::cout << "[" << elapsed << "s] step " << stepIndex << ": " << stepRad * 180.0 / M_PI << " deg" << std::endl;
    } else if (stepDone) {
      inSettle = true;
      phaseStart = now;
      if (!settled) {
        std::cout << "[" << elapsed << "s] step " << stepIndex << " hit the dwell cap without settling" << std::endl;
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  setpoint.headingRad = commanded;
  setpoint.speedMps = 0.0;
  client.update(setpoint);
  client.cancel();
  probeCsv.flush();

  if (samples.empty()) {
    std::cerr << "No samples captured" << std::endl;
    return 1;
  }
  const std::string omegaSource =
      reportedRateSamples >= static_cast<int32_t>(0.9 * totalSamples) ? "reported" : "differenced";
  if (omegaSource == "differenced") {
    std::cout << "WARNING: the platform did not report a yaw rate for most samples; the turn rate"
              << " was differenced from heading and is aliased by the sample interval." << std::endl;
  }

  const HeadingLoopModel model = arlcore::autopilot::identifyHeadingLoop(samples);
  printModel(model, speedMps, config);
  writeIdentification(options.outDir + "/identification.csv", model, speedMps, config, samples.size(), omegaSource);
  std::cout << "\nwrote " << options.outDir << "/{probe,identification}.csv" << std::endl;

  if (aborted || g_stop) {
    return 1;
  }
  const std::optional<flt64_t> configOmegaMax = config.platformCapabilities.surface.maxTurnRateRps;
  if (options.verifyTol > 0.0) {
    if (!configOmegaMax.has_value() || configOmegaMax.value() <= 0.0 || model.omegaMaxRps <= 0.0) {
      std::cerr << "--verify-tol needs both a configured and a measured turn rate" << std::endl;
      return 2;
    }
    const flt64_t ratio = model.omegaMaxRps / configOmegaMax.value();
    if (std::fabs(ratio - 1.0) > options.verifyTol) {
      std::cerr << "measured turn rate is " << ratio << "x the configured value, outside the " << options.verifyTol
                << " tolerance" << std::endl;
      return 2;
    }
  }
  return 0;
}
