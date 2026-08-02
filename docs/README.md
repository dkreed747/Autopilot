# docs

## Guides

- **[adding-a-vehicle.md](adding-a-vehicle.md)** — implement `IVehicleControl` for a real
  platform, register it with the factory, and satisfy the preconditions the autopilot checks at
  startup.
- **[tuning.md](tuning.md)** — the bring-up order once the strategy runs: platform capabilities,
  the heading-loop probe, the tracker gains, and objective validation.

Tool usage lives next to the tools, in [../tools/README.md](../tools/README.md).

## console/

Screenshots of the mission console, recorded against the simulated vehicle. Referenced from
`tools/README.md`; not used by any test or build step.

## mission-results/

Recorded reference runs. Each directory holds a `mission.csv` input, the `mission_runner` output
(`track.csv`, `planned_path.csv`, `waypoints.csv`, and `meta.csv` for post-refactor recordings),
and an `expectations.csv` locking the measured metrics into a band.

**The `analyze-baselines` CI job verifies every one of them**, so re-recording a directory breaks
CI until its `expectations.csv` is regenerated in the same change. Do that deliberately, not as a
side effect.

| Directory | Schema | Role |
|---|---|---|
| `lawnmower-20m-r1` | current | The reference run for the shipped tracking law. The only recording carrying `meta.csv`, `yaw_rate_rps` and planner curvature, so it is the only one that exercises the analysis's primary paths rather than its legacy fallbacks. CI additionally requires it to pass all the acceptance gates. |
| `.`, `lawnmower-10m`, `lawnmower-20m`, `lawnmower-40m` | legacy | Pre-refactor recordings. They **cannot** pass the acceptance gates and are not targets; they exist so a change to the analysis that silently alters a measurement is distinguishable from a change that improves the control law. |
| `depth-spiral` | legacy | A depth-rate-limited spiral whose loop-back passes the from-start preview cannot contain. Its geometric metrics must stay *refused* rather than reported as error; CI asserts that. |

Expectation bands are `+/-10%` of the measured value or `0.02`, whichever is larger. That is wide
enough that a refactor of the projection or the arc classifier does not trip them, and narrow
enough to catch a real tracking regression.

To record a new reference run:

```sh
./build/autopilot autopilot.yaml &                       # fresh process: the plan starts from
./build/tools/mission_runner autopilot.yaml out mission.csv   # vehicle_control.sim.initial_*
python3 tools/analyze_tracking.py out --gate
```
