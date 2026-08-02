# Tuning the autopilot to a platform

The bring-up order below is not arbitrary: each step's inputs are the previous step's outputs.
Doing them out of order mostly produces plausible-looking numbers that are wrong.

1. [Platform capabilities](#1-platform-capabilities-first) — they derive the planned turn radius
2. [Measure the inner heading loop](#2-measure-the-inner-heading-loop) — `tools/heading_probe`
3. [Set the feedforward](#3-set-the-feedforward) — `heading_loop_tau_s`, `feedforward_limit_rad`
4. [Set the cross-track law](#4-set-the-cross-track-law) — approach angle, then gain
5. [Add the integral only if needed](#5-add-the-integral-only-if-needed) — `planner.xte.ki`
6. [Validate objectively](#6-validate-objectively) — `mission_runner` + `analyze_tracking.py`
7. [Symptom to knob](#7-symptom-to-knob)
8. [Cross-field rules worth knowing](#8-cross-field-rules-worth-knowing)

## 0. The law in one page

```
heading_cmd = wrap( path_tangent + feedforward - cross_track )

feedforward = clamp( omega_desired * heading_loop_tau_s, +/- feedforward_limit_rad )
omega_desired = ground_speed * path_curvature

cross_track = clamp( approach * (2/pi) * atan(kp_scale * gain_per_m * xte) + integral,
                     +/- correction_limit_rad )
```

Everything is in the azimuth frame: true north, clockwise positive, starboard-positive cross-track
error. The correction is *subtracted*, so a starboard offset commands a turn to port.

**Why the feedforward is the load-bearing term.** The platform accepts a heading, not a turn rate.
An inner heading loop of gain `K` produces a turn rate proportional to its heading error, so
holding an arc at `omega_desired` requires a *standing* heading error of `omega_desired / K`. The
feedforward supplies exactly that bias, which is why it is `omega_desired * tau` with
`tau = 1/K`. Without it the vehicle only turns once cross-track error has built up, which is why
it cuts every corner.

There is no in-flight adaptation. An earlier release carried a self-calibrating trim that tried to
learn `tau` from the measured yaw rate; it was removed because it made tracking worse. It computed
its shortfall during arc *entry*, where actuator lag makes the measured rate trail the demand, and
integrated that transient into a standing bias that then over-turned the whole steady-state arc.
**So `heading_loop_tau_s` must be measured.** That is what step 2 is for.

The law is stateless apart from the cross-track integral, so the same geometry always produces the
same command. Guidance recomputes on each pose sample, not on `loop.control_period_ms`.

## 1. Platform capabilities first

Everything downstream scales off the planned turn radius:

```
R_min    = cruising_speed_mps / max_turn_rate_rps     (falls back to max_forward_speed_mps)
R_plan   = max(1.0, turn_radius_margin) * R_min
```

Shipped: `3.0 / 0.2618 = 11.46 m`, `x 1.25 = 14.32 m`.

`platform_capabilities.surface` must define a representative speed and a **positive**
`max_turn_rate_rps`, or the app refuses to start. It refuses rather than defaulting because a
missing envelope would silently fall back to a conservative 25 m radius on a platform that never
asked for it, and every acceptance threshold derived from that radius would then describe a
different vehicle.

**Measure `max_turn_rate_rps`; do not take it from a spec sheet.** Step 2 cross-checks it, and
`heading_probe --verify-tol=0.10` exits 2 if the configured value disagrees by more than 10%.

`turn_radius_margin` is the turn authority the plan reserves for corrections. Below 1.0 the planner
would plan turns tighter than the platform can fly, so the factory floors it at the kinematic
minimum and validation warns. 1.25 leaves 20% of the envelope for the tracker.

## 2. Measure the inner heading loop

```bash
./tools/heading_probe autopilot.yaml probe-out --speed=3.0 \
    --schedule="5,-5,10,-10,40,-40,90,-90" --dwell-max=15 --warmup=3 --settle=2 --dry-run
```

`--dry-run` reports how much water the schedule needs. Drop it to run for real.

The schedule must contain **both** small steps and large ones. Small steps stay inside the linear
region and identify the gain; large steps saturate and identify the rate envelope. A schedule of
only large steps returns `SATURATION_DOMINATED` with no gain.

**Preconditions:** no zones and no constraints may be active. Vector-mode zone guidance rewrites
the commanded heading, which invalidates the identification silently.

`identification.csv`, field by field, with the action for each:

| Field | Meaning | What to do |
|---|---|---|
| `K_rps_per_rad` | turn rate per radian of heading error, unsaturated region | this is the gain; `tau = 1/K` |
| `K_r2`, `K_n` | fit quality and sample count | low R² means a nonlinear plant or too few unsaturated samples — add smaller steps |
| `K_port_*`, `K_stbd_*`, `asymmetry_frac` | per-direction gains | above 0.15 warns; the tracker has one gain, so expect a port/starboard split in the arc metrics |
| `omega_max_rps`, `omega_max_cv`, `omega_max_windows` | measured rate envelope | fewer than three saturated windows or >15% spread gives `UNRELIABLE_OMEGA_MAX` — add 40°+ steps |
| `e_sat_rad` / `e_sat_deg` | `omega_max / K`, the error at which the loop pins at its rate limit | the most useful single number for sizing the feedforward |
| `heading_loop_tau_s` | `1/K` | **the value to configure** |
| `decay_tau_s`, `model_consistency` | `decay_tau * K`; 1.0 means genuinely first order | far from 1.0 means actuator lag the feedforward cannot capture; expect residual arc error and do not chase it with gain |
| `omega_max_ratio_to_config` | measured vs configured envelope | must be ≈1.0, else go back to step 1 |
| `R_min_measured_m` | `speed / omega_max` | sanity-check against `R_min` from step 1 |
| `suggested_heading_loop_tau_s`, `suggested_ff_limit_rad` | the probe's own recommendations | starting points for step 3 |
| `verdict` | `OK`, `SATURATION_DOMINATED`, `UNRELIABLE_OMEGA_MAX`, `NO_DATA` | see below |

**A non-identifiable run writes the gain fields blank, never zero.** That is deliberate: pasting a
blank into config as `0` would silently disable the feedforward and the vehicle would cut every
corner with no diagnostic. If a field is blank, fix the schedule and re-run.

Then eyeball the fit:

```bash
python3 tools/plot_heading_probe.py probe-out
```

## 3. Set the feedforward

**`heading_loop_tau_s` = the probe's `heading_loop_tau_s`, i.e. `1/K`.**

This is the loop's *steady-state* gain, not its lag-inclusive closed-loop constant. A first-order
actuator lag has settled by the time an arc reaches steady state, so it does not change the
standing heading error the arc needs. Adding the lag in makes tracking monotonically worse: on the
in-repo simulator, moving `tau` from the correct 1.11 s toward the lag-inclusive 1.61 s takes the
mean arc offset from −0.04 m to −1.01 m.

Validation warns above 1.5 s. `0.0` disables the feedforward entirely — an A/B control, never a
shipping value.

**`feedforward_limit_rad`** is not arbitrary. On a planned arc the geometry asks for

```
omega_desired = v / R_plan = omega_max / turn_radius_margin
feedforward   = omega_desired * tau = e_sat / turn_radius_margin
```

so at the shipped margin of 1.25 the largest bias the geometry ever requests is `0.8 * e_sat` —
which is exactly what the probe writes as `suggested_ff_limit_rad`. And since the feedforward can
never exceed `omega_max * tau = e_sat`, **any limit above `e_sat` can never bind at all**.

At shipped values `e_sat = 0.2618 x 1.11 = 0.291 rad (16.6°)` and the geometry asks for
`0.233 rad (13.3°)`, so the shipped `feedforward_limit_rad: 0.7` is inert. Set it just above
`e_sat / turn_radius_margin` so it binds only on a replanned arc tighter than the planned radius.
Beyond `e_sat` you are commanding a heading the loop physically cannot chase, and the error simply
sits there.

## 4. Set the cross-track law

**`cross_track_approach_rad` first.** It is the hard ceiling on how far the cross-track term may
swing the command off the tangent, however large the error. Pick the worst intercept angle you are
willing to fly: 0.6 rad is 34°, giving a closure rate of `v * sin(0.6) = 1.69 m/s` at 3 m/s. Must
be in `(0, pi/2)`; beyond a right angle the correction points the command *across* the path.

**Then `cross_track_gain_per_m`.** Three equivalent readings; the third is the one that bounds it.

*Half-authority distance.* Since `atan(1) = pi/4`, at `|xte| = 1 / (kp_scale * gain_per_m)` the
correction is exactly half the approach angle. Shipped 0.15 puts that at 6.7 m. Pick the error at
which you want half your intercept angle, and invert.

*Origin slope.* Small-signal slope is `approach * (2/pi) * kp_scale * gain_per_m`.

**Cascade separation — compute this one for a new hull.** Linearising the closed loop,

```
d(xte)/dt  ~=  -v * approach * (2/pi) * kp_scale * gain_per_m * xte
T_xte      =   1 / (v * approach * (2/pi) * kp_scale * gain_per_m)
```

The outer cross-track loop must be **slower** than the inner heading loop or the two fight.
Requiring `T_xte >= k * heading_loop_tau_s` with `k` in 3–5 gives

```
gain_per_m <= pi / (2 * k * heading_loop_tau_s * v * approach * kp_scale)
```

At shipped values (`k=5`, `tau=1.11`, `v=3.0`, `approach=0.6`, `kp_scale=1`) that is **0.157**, and
the shipped 0.15 is that bound. Worked second example, a slower hull with `tau=2.0 s`, `v=1.5 m/s`,
`approach=0.5`: `pi / (2*5*2.0*1.5*0.5) = 0.209`.

Start at `k=5`. Move toward `k=3` only once the arc metrics are clean, and back off at the first
sign of heading overshoot.

Note this gain is a fixed constant, not derived from the turn radius. On a platform whose radius
differs a lot from 14 m, recompute it from the formula above rather than carrying 0.15 over.

**`kp_scale`** is a single scalar multiplier on the gain. Use it for A/B comparisons; ship 1.0 and
tune the gain itself.

**`correction_limit_rad`** clamps P and I together, so keep it above
`cross_track_approach_rad + integrator_limit_rad` or it silently caps the approach angle below what
you set. Validation warns if it does not.

## 5. Add the integral only if needed

Pure P leaves a standing offset when a lateral current or a rudder trim biases the hull. The
integral is the only term that nulls it. `planner.xte.ki` defaults to `0.0`; a field starting point
is 0.02–0.05 rad per meter-second.

Reproduce the symptom before trusting the fix: set `vehicle_control.sim.current_east_mps` to
0.3–0.5 and fly a north-south lawnmower. Confirm a standing `straight_xte_mean` at `ki = 0`, then
raise `ki` until it nulls.

Anti-windup is already in place: `integrator_gate_m` (5 m) integrates only near the line,
`integrator_limit_rad` (0.35 ≈ 20° of crab) bounds the worst case, and conditional integration
freezes the integral while the total correction is saturated in the same direction.

The integral resets at every leg boundary and replan, so `ki` cannot compensate for anything
shorter than a leg. Too high shows up as a slow oscillation that overshoots through zero.

## 6. Validate objectively

Use a **fresh autopilot process per recording.** `planned_path.csv` is planned from
`vehicle_control.sim.initial_*`, so a run that starts where the last one ended is not comparable
with its own plan.

```bash
printf 'east_m,north_m,speed_mps,capture_radius_m,arrival_yaw_rad\n0,100,3.0,2.5,0.0\n' > mission.csv
./build/tools/mission_runner autopilot.yaml out mission.csv
python3 tools/analyze_tracking.py out --gate
```

What good looks like — the shipped acceptance gates, and what each is telling you:

| Gate | Bound | Reading |
|---|---|---|
| geometry evaluable | flown/planned in `(0.90, 1.20)`, plan has curvature | outside this, every geometric metric is suppressed as untrustworthy |
| omega reported | `source=reported`, not `differenced` | differencing yaw aliases against the publisher and invents peaks near 200% of the limit |
| arc mean offset | `\|mean\| <= 0.50 m` | **negative means cutting inside**, so `tau` is too small; positive means overshooting, so `tau` is too large |
| arc p10 / min offset | `>= -1.00 m` / `>= -1.20 m` | the tail, not the average |
| port/stbd symmetry | `<= 0.30 m` | a split here is hull or actuator asymmetry, not a tracker knob |
| margin consumed | no sample at ≥95% of `R_plan - R_min` | the turn-radius reserve survives the turn |
| arc turn rate | median within 10% of `cruise / R_plan` | the vehicle is flying the planned arc, not a different one |
| over-limit samples | 0 above `omega_max` | if not, `max_turn_rate_rps` is wrong — re-probe |
| straight mean / p90 / max | `<= 0.10 / 0.20 / 0.50 m` | steady-state straight tracking, recovery excluded |

Also read `settle_distance_median_m` (how far past each turn until `|xte| < 0.5 m`) and
`exec_vs_geom_rms` (planner-reported vs geometrically derived cross-track; a large disagreement
means the recording and the planner disagree about where the path is).

Lock a passing run in with an `expectations.csv` and add it to the `analyze-baselines` CI loop.
`docs/mission-results/lawnmower-20m-r1` is the current reference; see `docs/README.md`.

## 7. Symptom to knob

| Symptom | First knob | Why |
|---|---|---|
| standing corner-cut on every arc, both directions | `heading_loop_tau_s` up (re-probe) | the arc bias is too small for the loop's gain |
| standing overshoot outside every arc | `heading_loop_tau_s` down (re-probe) | over-biased |
| cut on one side only | not a tracker knob | hull/actuator asymmetry — check `asymmetry_frac` |
| offset only on straights, same sign all run | `planner.xte.ki` up from 0 | standing current or trim |
| slow weaving on straights, period much longer than tau | `cross_track_gain_per_m` down (raise `k`) | the outer loop is not slower than the inner one |
| fast oscillation at the heading-loop period | not the tracker | the platform's own servo — check `model_consistency` |
| long settle after every turn | `cross_track_gain_per_m` or `cross_track_approach_rad` up | intercept too gentle |
| overshoot when rejoining after a large excursion | `cross_track_approach_rad` down | intercept too aggressive to arrest |
| `saturated_frac` high on ordinary arcs | `turn_radius_margin` up | the plan is eating the whole envelope |
| `over_limit_samples > 0` | `platform_capabilities.surface.max_turn_rate_rps` | re-probe with `--verify-tol` |
| periodic zero-speed holds, "Navigation stale" | fix the nav rate, or raise `loop.nav_staleness_timeout_ms` | guidance is starved of fixes |
| laggy control but a clean probe | pose publish rate | guidance recomputes per pose sample, not per control period |
| replans stall the loop | `planner.rrt.time_budget_ms` down | RRT* runs on the control thread |

## 8. Cross-field rules worth knowing

**Errors** (the config will not load):

- `arbitration.safe_priority` must exceed every local and remote priority. A safe-mode maneuver
  that loses the driving resource to an operator command is a safety system that cannot act.
- `cross_track_approach_rad` outside `(0, pi/2)`.
- `feedforward_limit_rad` or `cross_track_gain_per_m` at or below zero; `heading_loop_tau_s`
  negative.
- Inverted `constraints` min/max for speed or depth.
- `safety.safe_mode.strategy` outside `srp` / `zero_speed_hold`; an SRP CSV with no explicit origin.
- `vehicle_control.type` naming a strategy the factory does not implement.
- Any removed config key, e.g. `planner.xte.lead_time_s` or the `planner.tracker.trim_*` family.
  The error names the replacement. An unrecognised key only warns.

**Warnings** (loads, but look):

- `heading_loop_tau_s` above 1.5 s.
- Summed tracker authority above 1.75 rad.
- `correction_limit_rad` below `cross_track_approach_rad + integrator_limit_rad`.
- `turn_radius_margin` below 1.0.
- `rrt.time_budget_ms` above 4x the control period — it runs on the control thread.
- `nav_staleness_timeout_ms` below 2x the control period — spurious holds.
- `console.platform_id` equal to `identity.platform_id`, which makes the console a local autonomy
  rather than a remote operator and changes its arbitration priority.

## See also

- [adding-a-vehicle.md](adding-a-vehicle.md) — implementing `IVehicleControl` for a real platform
- [../tools/README.md](../tools/README.md) — the probe, runner, console and analysis scripts
- `config/autopilot.yaml` — every knob, with a comment saying why it exists
