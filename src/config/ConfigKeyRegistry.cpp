#include "autopilot/config/ConfigKeyRegistry.hpp"

#include <yaml-cpp/yaml.h>

#include <utility>

namespace arlcore::autopilot {

//! \brief The capability-limit leaves, which appear under both regimes.
static void appendCapabilityLimits(const std::string& prefix, std::vector<std::string>* out) {
  out->push_back(prefix + ".max_forward_speed_mps");
  out->push_back(prefix + ".max_reverse_speed_mps");
  out->push_back(prefix + ".cruising_speed_mps");
  out->push_back(prefix + ".max_turn_rate_rps");
  out->push_back(prefix + ".min_speed_in_medium_mps");
  out->push_back(prefix + ".max_depth_change_rate_mps");
}

const std::vector<std::string>& ConfigKeyRegistry::knownKeys() {
  // Built once. Mirrors YamlConfigLoader::load leaf for leaf; the shipped-config test in
  // YamlConfigLoaderTest fails if the two drift apart.
  static const std::vector<std::string> keys = [] {
    std::vector<std::string> k = {
        "dds.domain_id",
        "dds.qos_file",
        "dds.domain_qos_profile",
        "dds.large_collections_qos_profile",
        "identity.platform_id",
        "identity.vector_source_id",
        "identity.waypoint_source_id",
        "identity.specs_source_id",
        "identity.capabilities_source_id",
        "identity.nav_source_id",
        "identity.constraints_source_id",
        "identity.operational_mode_control_source_id",
        "identity.operational_mode_status_source_id",
        "operational_mode.allow_implicit_mode_transitions",
        "operational_mode.commands_out_of_mode_are_failed",
        "operational_mode.idle_revert_s",
        "arbitration.local.vector_priority",
        "arbitration.local.waypoint_priority",
        "arbitration.remote.vector_priority",
        "arbitration.remote.waypoint_priority",
        "arbitration.safe_priority",
        "loop.control_period_ms",
        "loop.nav_staleness_timeout_ms",
        "tolerances.vector.direction_rad",
        "tolerances.vector.speed_mps",
        "tolerances.vector.elevation_m",
        "tolerances.vector.hard",
        "tolerances.vector.failure_delay_s",
        "tolerances.waypoint_defaults.position_m",
        "tolerances.waypoint_defaults.yaw_rad",
        "tolerances.waypoint_defaults.elevation_m",
        "planner.lead_distance_m",
        "planner.turn_radius_margin",
        "planner.max_list_wait_cycles",
        "planner.max_misses_per_waypoint",
        "planner.elevation_counts_as_miss",
        "planner.max_replans",
        "planner.sample_step_m",
        "planner.tracker.heading_loop_tau_s",
        "planner.tracker.feedforward_limit_rad",
        "planner.tracker.cross_track_approach_rad",
        "planner.tracker.cross_track_gain_per_m",
        "planner.xte.kp_scale",
        "planner.xte.ki",
        "planner.xte.integrator_limit_rad",
        "planner.xte.integrator_gate_m",
        "planner.xte.correction_limit_rad",
        "planner.rrt.seed",
        "planner.rrt.max_iterations",
        "planner.rrt.time_budget_ms",
        "planner.rrt.goal_bias",
        "planner.rrt.near_k",
        "planner.rrt.edge_check_step_m",
        "planner.rrt.final_check_step_m",
        "constraints.max_speed_mps",
        "constraints.min_speed_mps",
        "constraints.max_depth_m",
        "constraints.min_depth_m",
        "constraints.min_altitude_asf_m",
        "zones.safety_margin_m",
        "zones.compliance_hysteresis_m",
        "zones.elevation_margin_m",
        "zones.ellipse_segments",
        "vector_avoidance.lookahead_rho_factor",
        "vector_avoidance.lookahead_speed_s",
        "vector_avoidance.exit_clear_factor",
        "vector_avoidance.exit_clear_ticks",
        "vector_avoidance.min_follow_s",
        "recovery.speed_mps",
        "recovery.complete_hold_s",
        "safety.grace_period_s",
        "safety.grace_overrides.zone_s",
        "safety.grace_overrides.speed_s",
        "safety.grace_overrides.elevation_s",
        "safety.violation_confirm_ticks",
        "safety.clear_hold_s",
        "safety.exit_on_all_clear",
        "safety.state_report_period_ms",
        "safety.safe_mode.strategy",
        "safety.safe_mode.srp.csv_path",
        "safety.safe_mode.srp.origin_lat_deg",
        "safety.safe_mode.srp.origin_lon_deg",
        "safety.safe_mode.srp.accept_commands_after_srp",
        "safety.safe_mode.srp.hold_radius_m",
        "safety.safe_mode.srp.reposition_speed_mps",
        "safety.safe_mode.srp.safe_elevation_m",
        "vehicle_control.type",
        "vehicle_control.sim.cycle_rate_hz",
        "vehicle_control.sim.initial_latitude_deg",
        "vehicle_control.sim.initial_longitude_deg",
        "vehicle_control.sim.initial_heading_rad",
        "vehicle_control.sim.accel_mps2",
        "vehicle_control.sim.heading_gain_rps_per_rad",
        "vehicle_control.sim.heading_lag_s",
        "vehicle_control.sim.floor_depth_m",
        "vehicle_control.sim.current_east_mps",
        "vehicle_control.sim.current_north_mps",
        "platform_specs.name",
        "platform_specs.length_at_waterline_m",
        "platform_specs.beam_at_waterline_m",
        "platform_specs.draft_m",
        "platform_specs.forward_distance_m",
        "platform_specs.aft_distance_m",
        "platform_specs.port_distance_m",
        "platform_specs.starboard_distance_m",
        "platform_specs.top_distance_m",
        "platform_specs.bottom_distance_m",
        "platform_specs.displacement_metric_ton",
        "platform_specs.weight_light_metric_ton",
        "platform_specs.weight_loaded_metric_ton",
        "platform_capabilities.min_water_depth_m",
        "platform_capabilities.underwater.enabled",
        "platform_capabilities.underwater.reports_altitude_asf",
        "console.platform_id",
        "console.source_id",
    };
    appendCapabilityLimits("platform_capabilities.surface", &k);
    appendCapabilityLimits("platform_capabilities.underwater", &k);
    return k;
  }();
  return keys;
}

std::string ConfigKeyRegistry::removalReason(const std::string& dottedKey) {
  static const std::vector<std::pair<std::string, std::string>> removed = {
      {"planner.xte.lead_time_s",
       "it was the inner heading loop's time constant under another name; set "
       "planner.tracker.heading_loop_tau_s instead, measured with tools/heading_probe"},
      {"planner.tracker.trim_gain",
       "the self-calibrating trim was removed: it degraded tracking at a correctly measured time "
       "constant because it learned arc-entry transients as a standing bias. Measure "
       "planner.tracker.heading_loop_tau_s with tools/heading_probe instead"},
      {"planner.tracker.trim_tau_limit_frac", "the self-calibrating trim was removed; see docs/tuning.md"},
      {"planner.tracker.trim_tau_limit_s",
       "it was renamed to trim_tau_limit_frac and then removed with the trim; see docs/tuning.md"},
      {"planner.tracker.trim_limit_rad", "the self-calibrating trim was removed; see docs/tuning.md"},
      {"planner.tracker.trim_rate_floor_frac", "the self-calibrating trim was removed; see docs/tuning.md"},
      {"planner.tracker.trim_cross_track_gate_m", "the self-calibrating trim was removed; see docs/tuning.md"},
      {"planner.tracker.saturation_frac",
       "it only gated the removed self-calibrating trim; the tracking law no longer judges loop "
       "saturation"},
  };
  for (const std::pair<std::string, std::string>& entry : removed) {
    if (entry.first == dottedKey) {
      return entry.second;
    }
  }
  return std::string();
}

//! \brief Recursive leaf walk. A map node descends; anything else is a leaf and gets classified.
static void collectNode(const YAML::Node& node, const std::string& prefix, std::vector<std::string>* unknown,
                        std::vector<std::string>* removed) {
  if (!node.IsDefined() || !node.IsMap()) {
    return;
  }
  for (YAML::const_iterator it = node.begin(); it != node.end(); ++it) {
    if (!it->first.IsScalar()) {
      continue;
    }
    const std::string path = prefix.empty() ? it->first.as<std::string>() : prefix + "." + it->first.as<std::string>();
    const std::string reason = ConfigKeyRegistry::removalReason(path);
    if (!reason.empty()) {
      if (removed != nullptr) {
        removed->push_back(path + ": " + reason);
      }
      continue;
    }
    if (it->second.IsMap()) {
      collectNode(it->second, path, unknown, removed);
      continue;
    }
    const std::vector<std::string>& known = ConfigKeyRegistry::knownKeys();
    bool found = false;
    for (const std::string& key : known) {
      if (key == path) {
        found = true;
        break;
      }
    }
    // A known key that carries a nested block is a structural mistake rather than a typo, but it
    // is still reported through the same channel; validation only needs to name it.
    if (!found && unknown != nullptr) {
      unknown->push_back(path);
    }
  }
}

void ConfigKeyRegistry::collect(const std::string& yamlPath, std::vector<std::string>* unknown,
                                std::vector<std::string>* removed) {
  YAML::Node root;
  try {
    root = YAML::LoadFile(yamlPath);
  } catch (const YAML::Exception&) {
    return;  // the loader reports the parse failure; this pass has nothing to add
  }
  collectNode(root, std::string(), unknown, removed);
}

}  // namespace arlcore::autopilot
