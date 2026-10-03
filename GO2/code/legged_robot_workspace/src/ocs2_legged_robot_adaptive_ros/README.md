# ocs2_legged_robot_adaptive_ros

Launch file:

- `launch/legged_robot_adaptive_ddp.launch`

The adaptive MPC node is:

- `legged_robot_adaptive_ddp_mpc`

The normal launch uses `legged_robot_adaptive_dummy` for the plant and reuses the original target and gait-command nodes from `ocs2_legged_robot_ros`. The added dummy keeps the original OCS2 rollout and only adds the explicitly configured matched wrench.

## Online update validation

Run this only inside the existing OCS2 workspace after sourcing `devel/setup.bash`.
It applies a known world-frame ConstantWrench6D to the dummy plant from 1 s to
4 s, records the synchronized estimate and sliding variable, then writes a CSV,
JSON summary, and PNG curve:

```bash
src/ocs2_legged_robot_adaptive_ros/scripts/run_adaptive_update_validation.sh
```

The default validation wrench is `[20, 0, 0, 0, 0, 0]` in
`[Fx, Fy, Fz, Tx, Ty, Tz]` order. Normal launch keeps the disturbance disabled
unless `disturbance_enabled:=true` is passed explicitly.

## Task B gate and experiment entry

Generate the three effective configurations and the machine-readable fairness
audit without collecting experiment data:

```bash
python3 src/ocs2_legged_robot_adaptive_ros/scripts/set_task_b_mode.py audit
```

The audit requires CLF ON, input-bias OFF, `mu=0.1`, `delta=5`, `K_D=I6`, the
fixed Lambda/Gamma matrices, zero initial estimate, `Yu=I6`, and proves that the
three generated configs differ only in `enableParameterUpdate` and
`enableAdaptiveDynamics`.

`run_task_b_experiment.sh MODE OUTPUT_DIR` is the later B2/B3 acquisition entry.
It schedules the +0.10 m target at MPC time 0.50 s (0.20 s transition), applies
the fixed +20 N world-frame Fx disturbance at 1.0--4.0 s, records from before
launch, and stops at MPC time 15.50 s. Do not use it for formal acquisition
until Gate B-0/B-1 have passed.

The synchronized `/task_b/equation10_sample` has 24 values: time/snapshot/mode
metadata, `Yu*pi_hat[6]`, candidate `-Yu*pi_hat/m[6]`, and the correction actually
enabled in MPC dynamics `[6]`. Raw `/legged_robot_mpc_observation` plus
`/task_b/policy_input_sample` preserve the executed policy input.

## Task C mass-mismatch interface

The dummy has a plant-only private parameter:

```text
~true_plant_mass_scale
```

Its default is `1.0`, which preserves the original Task A/B rollout exactly.
For a non-unit scale, the controller still uses the nominal URDF and nominal
`CentroidalModelInfo`, while only the dummy rollout uses a copied model info
whose `robotMass` is scaled. COM, inertia, task/reference files, costs, solver,
and adaptive settings are not changed.

Task C per-step diagnostics are separately gated by
`~task_c_diagnostics_enabled` (default `false`) so the added flow-map
evaluations do not perturb Task A/B timing. A Task C nominal/uncertain paired
comparison must set this flag to `true` for every group.

The latched `/task_c/model_identity` string records schema/model IDs, nominal
and true mass, mass scale, override status, diagnostics status, and controller
URDF path.

When enabled, `/task_c/model_mismatch_sample` is a 41-value
`Float64MultiArray` with schema `task_c_model_mismatch_v1_41_fields`:

```text
0       observation time
1       nominal mass
2       true plant mass
3       payload delta
4       true plant mass scale
5       override active
6:12    nominal base flow head(6)
12:18   true plant flow head(6)
18:24   true - nominal normalized dynamics residual
24:30   pi_star = -m_nominal * residual
30:36   nominal - pi_star/m_nominal - true reconstruction error
36      full flow residual norm
37      residual norm outside the matched head(6) channel
38      pi_star norm
39      reconstruction max absolute error
40      finite/valid flag
```

This interface is a controlled SRBD mass-scalar mismatch, not identification
of a complete physical payload with changed COM/inertia. No formal Task C mass
scale is selected by the C1 unit-test-only nonzero scale.

## PayloadMass1D development mode

`PayloadMass1D` is an explicit opt-in parameterization; the default remains
`constant_wrench_6d`. Select it only with:

```text
/adaptiveSettingsFile = $(find ocs2_legged_robot_adaptive)/config/adaptive_settings_payload_mass_1d.info
```

The frozen estimate stores `[delta_m_hat, 0, 0, 0, 0, 0]`, with `delta_m_hat`
in kg. Its paper Eq.(8)/(9) mass column is `[a_r-g, 0, 0, 0]`, and Eq.(10)
uses `m_hat=m_nominal+delta_m_hat` to scale the OCS2 normalized centroidal
flow exactly. The controller never reads `true_plant_mass_scale` or
`pi_star`.

The isolated implementation smoke entry is:

```bash
run_task_c_payload_mass_smoke.sh OUTPUT_DIR END_MPC_TIME TRUE_PLANT_MASS_SCALE
```

It records `/adaptive_update/payload_mass_sample` (21 fields: synchronized
metadata, nominal/delta/total mass, mass ratio, delta-mass derivative,
reference regressor column, and Eq.(10)-equivalent wrench). Smoke output must
not be mixed with or written over `records/task_c_mass`.

The formal runner supports three explicit groups:

```bash
run_task_c_payload_mass_formal.sh nominal NEW_OUTPUT_DIRECTORY
run_task_c_payload_mass_formal.sh uncertain NEW_OUTPUT_DIRECTORY
run_task_c_payload_mass_formal.sh uncertain_off NEW_OUTPUT_DIRECTORY
```

`nominal` uses the nominal plant and Adaptive ON; `uncertain` uses the +10%
mass plant and Adaptive ON; `uncertain_off` uses the same +10% mass plant with
only `enableParameterUpdate` and `enableAdaptiveDynamics` set to false. CLF,
cost, solver, target, and every numeric adaptive setting remain fixed. A new
three-repeat, three-group round-robin campaign can be acquired with:

```bash
run_task_c_payload_mass_adaptive_off_campaign.sh NEW_RECORDS_ROOT
```

The campaign refuses to overwrite non-empty runs, performs the nine-run
effective-configuration/source audit, and produces the common-grid ON/OFF
analysis under `analysis_adaptive_off`.

## Deterministic 10 cm locomotion smoke

The earlier Task C baseline is an all-feet-contact `STANCE` base-shift test.
The separate locomotion smoke automatically publishes the native
`static_walk` template, waits until a non-STANCE mode is actually executed,
and only then publishes the fixed absolute `x=0.00 -> 0.10 m` target scheduled
at MPC time `1.20 -> 1.40 s`:

```bash
run_task_c_locomotion_smoke.sh NEW_OUTPUT_DIRECTORY 12.0
```

It records executed modes and `/legged_robot/currentState` foot markers.  The
offline validator reports structural validity, gait/contact cycling, each
foot's x/z motion, and tracking quality as separate results.  This single
nominal run is a gait-pipeline smoke test, not formal adaptive evidence, and
must not overwrite the stance PayloadMass1D campaign.

The formal PayloadMass1D walking comparison uses a controller-identical
three-group round-robin campaign:

```bash
run_task_c_locomotion_payload_campaign.sh NEW_RECORDS_ROOT
```

All nine runs use the same `static_walk` gait and absolute target. The
uncertain plant is `1.10 * nominal_mass`; `uncertain_off` changes only
parameter update and adaptive dynamics. Each run must independently confirm
executed contact-mode cycling and swing motion of all four feet before it is
accepted into the common-grid analysis.
