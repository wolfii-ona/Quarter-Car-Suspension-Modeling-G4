# Quarter-Car Suspension Modeling and Tuning (Simscape Multibody)

Made by Group 4

A model of one corner of a car's suspension, built in Simscape Multibody. It comes with a
set of test roads, an automatic scoring system, a parameter sweep that tunes the suspension,
and a check on how sensitive the result is to manufacturing variation.

## Quick start

```matlab
cd('<this folder>')
addpath('tools')

initParams              % sets Ks, Cs and a default road signal
open_system('model')    % view the model
[T, R] = runAllTests()  % run all 5 road cases and save results/summary.png
```

Run `initParams` before opening or simulating the model. The suspension spring and damper
read the workspace variables `Ks` and `Cs`, so Simulink will show an unresolved variable
error until those exist.

## Files

### Model

| File | What it does |
|---|---|
| `model.slx` | The quarter-car model. Road plate, then tire spring and damper, then a 20 kg wheel, then the suspension spring and damper, then a 350 kg body. Outputs four signals: `x_road`, `x_wheel`, `x_body`, `az_body`. |
| `initParams.m` | Sets the baseline values (`Ks=20000`, `Cs=750`) and a default road signal so the model can run on its own. |

### Analysis

| File | What it does |
|---|---|
| `roadSuite.m` | Builds the 5 test roads: SpeedBump, Pothole, RoughRoad, Washboard, TwoBumps. |
| `scoreSuspension.m` | Scores one simulation. Returns body acceleration (comfort), suspension travel, tire deflection, whether it passed, and a single score. |
| `runAllTests.m` | Runs all 5 roads, returns a summary table, saves `results/summary.png`. |
| `tuneSuspension.m` | Sweeps `Ks` and `Cs` over a 10 by 9 grid across all 5 roads (450 simulations, run in parallel) and picks the best design that stays within the limits. |
| `robustnessTest.m` | Varies `Ks` and `Cs` by ±10% over 20 random trials and reports the worst results and the pass rate. |

### Helpers in `tools/`

| File | What it does |
|---|---|
| `getLoggedSignal.m` | Pulls a named output signal out of a simulation result. |
| `simulateRoadCase.m` | Runs one road case at a given `Ks` and `Cs`. |
| `configureModelIO_step1*.m` | A record of every edit made to `model.slx`, with the reasoning. Kept so the changes can be checked. Do not re-run these blindly, since two of them add and delete blocks. |
| `renameBlocks.m` | Renamed blocks to clearer names. Cosmetic only. |

### Generated output in `results/`

`summary.png`, `tuning_table.csv`, `tuning_heatmap.png`, `tuning_comparison.png`,
`robustness_table.csv`, `robustness_summary.png`, `step1_verification.png`

## Faults found in the original model

The model would not run when we started. Four separate faults were found, and each one only
showed up once we actually tried to simulate it. Each fix is explained in the matching script
in `tools/`.

1. **The ground reference was disconnected.** There were two separate World Frame blocks. All
   three joints were attached to the one that was not connected to the Solver Configuration,
   so the software could not solve the mechanics at all.
2. **The road joint was set up wrongly.** It needed `TorqueActuationMode = ComputedTorque` so
   the solver could work out the force required to follow the commanded road motion.
3. **The body and wheel masses were wrong.** Both were set to calculate mass from shape and
   density rather than from the mass values typed in. The simulation was running with a
   1000 kg body and a 3142 kg wheel instead of 350 kg and 20 kg.
4. **Both springs started out compressed.** They were set to be relaxed at zero length, but
   the parts they connect sit 0.5 m apart, so each spring was heavily squashed before gravity
   was even applied.

After these fixes the body settles 0.190 m below its starting height under its own weight,
which matches the hand calculation to within 0.03 mm. The body also starts at exactly
9.80665 m/s² downwards, which is free fall before the spring takes hold. Both checks confirm
the model behaves correctly.

## Results

**Baseline** (`Ks=20000`, `Cs=750`): fails all 5 roads. Suspension travel reaches 0.158 m
against a limit of 0.08 m.

**Tuned** (`Ks=20000`, `Cs=2250`): passes all 5 roads. Same spring, three times the damping.

| | Baseline | Tuned |
|---|---|---|
| Average body acceleration | 1.498 m/s² | 1.527 m/s² (1.9% worse) |
| Worst suspension travel | 0.158 m | 0.0586 m (63% better) |
| Worst tire deflection | 0.0356 m | 0.0200 m (44% better) |

**Robustness** (±10% on `Ks` and `Cs`, 20 trials): only 45% passed. The tuned design sits at
99.8% of the tire deflection limit, so it has almost no margin and ordinary component
variation pushes it over. The underlying reason is that tire deflection depends on how fast
the wheel bounces on the tire, which `Ks` and `Cs` have very little control over. The
Interpretation section of the Live Script explains this in full.

## Assumptions

* The limits we test against (suspension travel 0.08 m, tire deflection 0.02 m) are our own
  engineering assumptions. The project brief does not specify them.
* Suspension travel and tire deflection are measured as movement away from the resting
  position under the car's own weight, not as absolute distances.
* The first second of every simulation is ignored, because the model settles under gravity
  during that time.
* The rough road uses a fixed random seed (42) so every design is tested against the same
  road.

<u>We appreciate all kinds of useful feedback. This isn't perfect and we will keep on improving as we learn. </u>
