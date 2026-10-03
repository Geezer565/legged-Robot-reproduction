# ocs2_legged_robot_adaptive

This package adds an adaptive CLF-MPC layer on top of the existing `ocs2_legged_robot` example.

## Active components

- `AdaptiveParameterUpdateModule`:
  - Eq. (8) online update and immutable per-solve snapshots.
- `AdaptiveLeggedRobotDynamics`:
  - Eq. (10) ConstantWrench6D correction of the OCS2 prediction dynamics.
- `AdaptiveClfConstraint`:
  - Eq. (11) as an OCS2 state-input soft constraint.
- `AdaptiveLeggedRobotInterface`:
  - Wraps `ocs2_legged_robot::LeggedRobotInterface`, reuses the original OCP, and injects adaptive terms.

## How to configure

The interface reads its active settings from:

- `config/adaptive_settings.info`, section `adaptive_clf_mpc`

Task B keeps CLF, costs, solver settings, gains, and penalties fixed. Its three
modes change only `enableParameterUpdate` and `enableAdaptiveDynamics`; see
`ocs2_legged_robot_adaptive_ros/scripts/set_task_b_mode.py`.

The legacy `adaptive_estimator` and `adaptive_clf` sections in the same file are
not read by `AdaptiveLeggedRobotInterface`.

## Task C ground truth

`MassMismatchGroundTruth` constructs nominal and true SRBD flow evaluators from
the same nominal Pinocchio/model information and changes only
`trueInfo.robotMass`. At one common `(t, x, u)` sample it defines

```text
Delta_f_6 = f_true.head(6) - f_nominal.head(6)
pi_star   = -m_nominal * Delta_f_6
```

so the current Eq. (10) convention closes exactly as
`f_nominal.head(6) - pi_star/m_nominal = f_true.head(6)`. The same true dynamics
object is cloneable for the animal_dummy rollout. This diagnostic is an
equivalent 6D wrench, not a mass estimate.

## D1B PayloadInertial10D unit integration

`PayloadInertial10D` defines a physical payload with ten linear spatial-inertia
parameters

```text
[m, hx, hy, hz, Jxx, Jxy, Jxz, Jyy, Jyz, Jzz]
```

about the nominal robot CoM, expressed in base axes. It provides a `6x10`
modified-reference regressor, physical-consistency checks, parallel-axis
composition into a plant-only `CentroidalModelInfo`, and a full-flow ground
truth evaluator. `PayloadInertialGroundTruth` retains both the first six
normalized-momentum residuals and the remaining coordinate-flow residuals.

The adaptive parameter dimension is now independent of the fixed six-axis
wrench dimension. `PayloadInertial10D` is accepted by the settings loader,
Eq. (8), immutable snapshots, the `6xp` Eq. (11) path, and the frozen
estimated-model full-flow/linearization path. Online raw estimates are never
silently clipped or projected: a nonphysical standalone increment is accepted
when the combined nominal+estimated model remains physical, while an invalid
combined total model fails explicitly.

`config/adaptive_settings_payload_inertial_10d_runtime_off.info` deliberately
keeps both update and adaptive dynamics disabled. D1B is a unit/runtime-off
integration gate only; 10D ROS v2 diagnostics, controller/plant coordinate
separation, animal_dummy smoke tests, gain selection, and formal experiments
remain later stages. The default Task A/B/C configurations are unchanged.
