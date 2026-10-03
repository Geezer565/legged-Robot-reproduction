# Full figure conditions

Exactly two figure types: [CoM velocity WORLD, angular velocity BODY], and BODY/CoM left-hand equivalent uncertainty [force, moment]. Physical velocities use recorded normalized momentum and the frozen nominal model, not pose derivatives. Each dashed velocity target is the target received in that method's run (matching color). V1.1 plots the learned product d=Y*pi, not its weights. Comparator compensation outputs are translated/rotated from their existing interface; this does not assert identical learning objectives. The grey V1.1 reference is reconstructed from nominal dynamics and measured velocity changes, not independent ground truth or the commanded external force. No position/path plots or smoothing.

Complete manifests, hashes, validation and frozen controller configurations:

```json
[
  {
    "source_directory": "/mnt/d/GO2/experi/results/v1_1/20260930T144035.376750Z_2180a5f3/main__flat_velocity_fy_sine__v11_equivalent_wrench_standard__r001",
    "manifest": {
      "command": [
        "roslaunch",
        "legged_controllers",
        "go2_method_comparison.launch",
        "case_dir:=/mnt/d/GO2/experi/results/v1_1/20260930T144035.376750Z_2180a5f3/main__flat_velocity_fy_sine__v11_equivalent_wrench_standard__r001",
        "controller_config:=/mnt/d/GO2/experi/results/v1_1/20260930T144035.376750Z_2180a5f3/main__flat_velocity_fy_sine__v11_equivalent_wrench_standard__r001/controller_config.yaml",
        "method:=v11_equivalent_wrench",
        "gui:=false",
        "gazebo_seed:=4102",
        "total_duration:=33.0",
        "motion_delay:=3.0",
        "motion_duration:=30.0",
        "vx:=0.45",
        "flat_motion:=true",
        "spawn_x:=0",
        "spawn_y:=0",
        "spawn_z:=0.35",
        "spawn_pitch:=0",
        "world:=/mnt/d/GO2/code/legged_robot_workspace/src/leggedcontrol_go2/legged_gazebo/worlds/myrbf_flat_control.world",
        "wrench_rate:=20.0",
        "wrench_hold:=0.25",
        "wrench_axis:=fy",
        "wrench_profile:=sine",
        "wrench_value:=0.0",
        "wrench_amplitude:=5.0",
        "wrench_frequency:=0.15",
        "wrench_secondary_amplitude:=0.0",
        "wrench_secondary_frequency:=0.3",
        "wrench_phase_deg:=90.0",
        "wrench_secondary_phase_deg:=90.0"
      ],
      "constraint_comparison_note": "Terrain traversal is not claimed; controller structural differences remain documented.",
      "constraint_surface_normal_world": [
        0,
        0,
        1
      ],
      "controller_config_sha256": "c72c8b79f4c5a3b3b5e26cdde9670e64398cb3062466626a173f9d4e70be9f68",
      "controller_config_source": "/mnt/d/GO2/code/legged_robot_workspace/src/leggedcontrol_go2/legged_controllers/config/go2/myrbf_v11_equivalent_wrench_go2.yaml",
      "controller_config_source_sha256": "ff1047d5e98244b0554211f7a4e0027903ba9707a0eab96d050279bd25558949",
      "data_kind": "simulation",
      "disturbance": {
        "amplitude": 5.0,
        "axis": "fy",
        "body": "go2::base",
        "command_hold_s": 0.25,
        "delay_s": 0.0,
        "duration_s": 33.0,
        "frequency": 0.15,
        "phase_deg": 90.0,
        "profile": "sine",
        "rate_hz": 20.0,
        "reference_frame": "world",
        "reference_point": [
          0.0,
          0.0,
          0.0
        ],
        "reference_point_source": "gazebo_world_net_wrench_zero_offset",
        "secondary_amplitude": 0.0,
        "secondary_frequency": 0.3,
        "secondary_phase_deg": 90.0,
        "torque_about": "go2::base link center of mass",
        "value": 0.0,
        "withdrawn_during_recording": false
      },
      "environment": {
        "DISPLAY": ":0",
        "GAZEBO_LOG_PATH": "/mnt/d/GO2/experi/results/v1_1/20260930T144035.376750Z_2180a5f3/main__flat_velocity_fy_sine__v11_equivalent_wrench_standard__r001/gazebo_logs",
        "GAZEBO_MASTER_URI": "http://127.0.0.1:58049",
        "ROS_HOME": "/mnt/d/GO2/experi/results/v1_1/20260930T144035.376750Z_2180a5f3/main__flat_velocity_fy_sine__v11_equivalent_wrench_standard__r001/ros_home",
        "ROS_LOG_DIR": "/mnt/d/GO2/experi/results/v1_1/20260930T144035.376750Z_2180a5f3/main__flat_velocity_fy_sine__v11_equivalent_wrench_standard__r001/ros_logs",
        "ROS_MASTER_URI": "http://127.0.0.1:60733",
        "ROS_PYTHON_LOG_CONFIG_FILE": "/opt/ros/noetic/etc/ros/python_logging.conf"
      },
      "experiment": "GO2-METHOD-COMPARISON",
      "finished_utc": "2026-09-30T15:03:21.678007+00:00",
      "force_estimate_convention": "body_equivalent_uncertainty_about_com_left_hand",
      "gait": "trot",
      "gui": false,
      "method": "v11_equivalent_wrench",
      "method_definition": {
        "clf_mode": "soft_squared_hinge",
        "clf_penalty_weight": 1e-05,
        "fidelity": "word_v1_1_fixed_gain_with_user_approved_soft_clf",
        "public_name": "MyRBF V1.1",
        "representation": "six_independent_scalar_gaussian_rbf_25_plus_bias",
        "update": "negative_gamma_h_G_transpose_composite_velocity_error"
      },
      "physics_step_s": 0.001,
      "planner": {
        "com_height_m": 0.33,
        "command_vx_mps": 0.45,
        "command_vy_mps": 0.0,
        "command_yaw_rate_rps": 0.0,
        "frame": "body",
        "gait_lead_s": 0.6,
        "heading_reference": "hold_initial_heading",
        "linear_velocity_target": "world_normalized_momentum_first3",
        "ramp_duration_s": 2.0,
        "rate_hz": 20.0,
        "reference_profile": "smooth_start_constant_velocity",
        "source": "cmd_vel",
        "terminal_hold_s": 0.0,
        "trajectory_id": "velocity",
        "trajectory_name": "constant_body_forward_velocity"
      },
      "raw_bag_recovered_from_active": true,
      "repetition": 1,
      "robot": "go2",
      "robust_mode": "none",
      "schema_version": 2,
      "seed": 4102,
      "simulator": "Gazebo Classic",
      "source_drift": [],
      "source_inventory_sha256": "bf5f34615a2a8158b50d6e45c952049633df4ff5cb1aa010ba98b558fac82f01",
      "started_utc": "2026-09-30T14:57:18.484715+00:00",
      "status": "recorded_validated",
      "study": "main",
      "terrain": "flat",
      "terrain_conditions": {
        "motion": true,
        "purpose": "flat_control_trial",
        "spawn": [
          0,
          0,
          0.35,
          0
        ],
        "surface_height_m": 0.0,
        "surface_normal_world": [
          0,
          0,
          1
        ]
      },
      "timing_s": {
        "final_hold": 0.0,
        "goal_delay": 3.0,
        "total": 33.0,
        "trajectory": 30.0
      },
      "variant": "standard"
    },
    "recording_validation": {
      "valid": true,
      "errors": [],
      "spans": {
        "state.csv": {
          "first_s": 0.0019999999999997797,
          "last_s": 32.982,
          "max_gap_s": 0.02200000000000024,
          "rows": 1650
        },
        "wrench.csv": {
          "first_s": 0.0019999999999997797,
          "last_s": 32.952000000000005,
          "max_gap_s": 0.05100000000000193,
          "rows": 680
        },
        "v11_equivalent_wrench.csv": {
          "first_s": 0.0040000000000000036,
          "last_s": 32.997,
          "max_gap_s": 0.03500000000000014,
          "rows": 32960
        }
      },
      "diagnostic_summary": {
        "samples": 32960,
        "maximum_clf_ec_violation": 61.83563848787171,
        "interpretation": "BODY/CoM left-hand equivalent uncertainty; reconstructed reference is not applied-force truth."
      },
      "campaign_start_ros": 1.5070000000000001,
      "note": "Validation includes recording integrity and basic fall detection; it is not a theorem-verification claim."
    },
    "controller_config": "activation_warmup: 1.0\nalgorithm: v11_equivalent_wrench\ncontrol_interface: model_and_wbc\nestimator_version: legacy\nlearning_target: equivalent_uncertainty_wrench_body_com_left_hand\nrepresentation: six_independent_scalar_gaussian_rbf_25_plus_bias\ntime_tolerance: 0.003\nupdate_period: 0.001\nv11_centers:\n- -1\n- -0.9166666666666666\n- -0.8333333333333333\n- -0.75\n- -0.6666666666666666\n- -0.5833333333333333\n- -0.5\n- -0.4166666666666667\n- -0.3333333333333333\n- -0.25\n- -0.1666666666666667\n- -0.0833333333333333\n- 0\n- 0.0833333333333333\n- 0.1666666666666667\n- 0.25\n- 0.3333333333333333\n- 0.4166666666666667\n- 0.5\n- 0.5833333333333333\n- 0.6666666666666666\n- 0.75\n- 0.8333333333333333\n- 0.9166666666666666\n- 1\nv11_clf_decay:\n- 0.2\n- 0.2\n- 0.2\n- 0.2\n- 0.2\n- 0.2\nv11_composite_ratio: 0.9\nv11_gamma:\n- 22.8221449\n- 22.8221449\n- 22.8221449\n- 0.00321391182\n- 0.0245852383\n- 0.0257958114\nv11_input_scale:\n- 0.02\n- 0.02\n- 0.02\n- 0.05\n- 0.05\n- 0.05\nv11_weight_limit:\n- 20\n- 20\n- 20\n- 3\n- 3\n- 3\nv11_widths:\n- 0.12\n- 0.12\n- 0.12\n- 0.12\n- 0.12\n- 0.12\n- 0.12\n- 0.12\n- 0.12\n- 0.12\n- 0.12\n- 0.12\n- 0.12\n- 0.12\n- 0.12\n- 0.12\n- 0.12\n- 0.12\n- 0.12\n- 0.12\n- 0.12\n- 0.12\n- 0.12\n- 0.12\n- 0.12\n"
  },
  {
    "source_directory": "/mnt/d/GO2/experi/results/v1_1/20260930T144035.376750Z_2180a5f3/main__flat_velocity_fy_sine__eth_standard__r001",
    "manifest": {
      "command": [
        "roslaunch",
        "legged_controllers",
        "go2_method_comparison.launch",
        "case_dir:=/mnt/d/GO2/experi/results/v1_1/20260930T144035.376750Z_2180a5f3/main__flat_velocity_fy_sine__eth_standard__r001",
        "controller_config:=/mnt/d/GO2/experi/results/v1_1/20260930T144035.376750Z_2180a5f3/main__flat_velocity_fy_sine__eth_standard__r001/controller_config.yaml",
        "method:=eth",
        "gui:=false",
        "gazebo_seed:=4102",
        "total_duration:=33.0",
        "motion_delay:=3.0",
        "motion_duration:=30.0",
        "vx:=0.45",
        "flat_motion:=true",
        "spawn_x:=0",
        "spawn_y:=0",
        "spawn_z:=0.35",
        "spawn_pitch:=0",
        "world:=/mnt/d/GO2/code/legged_robot_workspace/src/leggedcontrol_go2/legged_gazebo/worlds/myrbf_flat_control.world",
        "wrench_rate:=20.0",
        "wrench_hold:=0.25",
        "wrench_axis:=fy",
        "wrench_profile:=sine",
        "wrench_value:=0.0",
        "wrench_amplitude:=5.0",
        "wrench_frequency:=0.15",
        "wrench_secondary_amplitude:=0.0",
        "wrench_secondary_frequency:=0.3",
        "wrench_phase_deg:=90.0",
        "wrench_secondary_phase_deg:=90.0"
      ],
      "constraint_comparison_note": "Terrain traversal is not claimed; controller structural differences remain documented.",
      "constraint_surface_normal_world": [
        0,
        0,
        1
      ],
      "controller_config_sha256": "2eed455216fcdd385d6804be81aa4e9a7efd1255b7de1e682e787f685e57c115",
      "controller_config_source": "/mnt/d/GO2/code/legged_robot_workspace/src/leggedcontrol_go2/legged_controllers/config/go2/myrbf_shadow_eth_go2_closed_loop.yaml",
      "controller_config_source_sha256": "9b595cfaeda652c38a32ab540b56b9e26a4220b6c84e35309a8dd76dea178694",
      "data_kind": "simulation",
      "disturbance": {
        "amplitude": 5.0,
        "axis": "fy",
        "body": "go2::base",
        "command_hold_s": 0.25,
        "delay_s": 0.0,
        "duration_s": 33.0,
        "frequency": 0.15,
        "phase_deg": 90.0,
        "profile": "sine",
        "rate_hz": 20.0,
        "reference_frame": "world",
        "reference_point": [
          0.0,
          0.0,
          0.0
        ],
        "reference_point_source": "gazebo_world_net_wrench_zero_offset",
        "secondary_amplitude": 0.0,
        "secondary_frequency": 0.3,
        "secondary_phase_deg": 90.0,
        "torque_about": "go2::base link center of mass",
        "value": 0.0,
        "withdrawn_during_recording": false
      },
      "environment": {
        "DISPLAY": ":0",
        "GAZEBO_LOG_PATH": "/mnt/d/GO2/experi/results/v1_1/20260930T144035.376750Z_2180a5f3/main__flat_velocity_fy_sine__eth_standard__r001/gazebo_logs",
        "GAZEBO_MASTER_URI": "http://127.0.0.1:57273",
        "ROS_HOME": "/mnt/d/GO2/experi/results/v1_1/20260930T144035.376750Z_2180a5f3/main__flat_velocity_fy_sine__eth_standard__r001/ros_home",
        "ROS_LOG_DIR": "/mnt/d/GO2/experi/results/v1_1/20260930T144035.376750Z_2180a5f3/main__flat_velocity_fy_sine__eth_standard__r001/ros_logs",
        "ROS_MASTER_URI": "http://127.0.0.1:52351",
        "ROS_PYTHON_LOG_CONFIG_FILE": "/opt/ros/noetic/etc/ros/python_logging.conf"
      },
      "experiment": "GO2-METHOD-COMPARISON",
      "finished_utc": "2026-09-30T15:06:57.062725+00:00",
      "force_estimate_convention": "world_compensation_wrench_about_base_origin",
      "gait": "trot",
      "gui": false,
      "method": "eth",
      "method_definition": {
        "fidelity": "constant_wrench_specialization",
        "public_name": "ETH",
        "representation": "direct_parameter",
        "update": "identity_regressor_tracking_adaptation"
      },
      "physics_step_s": 0.001,
      "planner": {
        "com_height_m": 0.33,
        "command_vx_mps": 0.45,
        "command_vy_mps": 0.0,
        "command_yaw_rate_rps": 0.0,
        "frame": "body",
        "gait_lead_s": 0.6,
        "heading_reference": "hold_initial_heading",
        "linear_velocity_target": "world_normalized_momentum_first3",
        "ramp_duration_s": 2.0,
        "rate_hz": 20.0,
        "reference_profile": "smooth_start_constant_velocity",
        "source": "cmd_vel",
        "terminal_hold_s": 0.0,
        "trajectory_id": "velocity",
        "trajectory_name": "constant_body_forward_velocity"
      },
      "raw_bag_recovered_from_active": true,
      "repetition": 1,
      "robot": "go2",
      "robust_mode": "none",
      "schema_version": 2,
      "seed": 4102,
      "simulator": "Gazebo Classic",
      "source_drift": [],
      "source_inventory_sha256": "bf5f34615a2a8158b50d6e45c952049633df4ff5cb1aa010ba98b558fac82f01",
      "started_utc": "2026-09-30T15:03:59.301105+00:00",
      "status": "recorded_validated",
      "study": "main",
      "terrain": "flat",
      "terrain_conditions": {
        "motion": true,
        "purpose": "flat_control_trial",
        "spawn": [
          0,
          0,
          0.35,
          0
        ],
        "surface_height_m": 0.0,
        "surface_normal_world": [
          0,
          0,
          1
        ]
      },
      "timing_s": {
        "final_hold": 0.0,
        "goal_delay": 3.0,
        "total": 33.0,
        "trajectory": 30.0
      },
      "variant": "standard"
    },
    "recording_validation": {
      "valid": true,
      "errors": [],
      "spans": {
        "state.csv": {
          "first_s": 0.008000000000000007,
          "last_s": 32.983000000000004,
          "max_gap_s": 0.02800000000000047,
          "rows": 1650
        },
        "wrench.csv": {
          "first_s": 0.005999999999999783,
          "last_s": 32.953,
          "max_gap_s": 0.054999999999999716,
          "rows": 680
        },
        "shadow.csv": {
          "first_s": 0.01399999999999979,
          "last_s": 32.997,
          "max_gap_s": 0.01699999999999946,
          "rows": 4477
        }
      },
      "diagnostic_summary": null,
      "campaign_start_ros": 1.5190000000000001,
      "note": "Validation includes recording integrity and basic fall detection; it is not a theorem-verification claim."
    },
    "controller_config": "algorithm: v1\nallocator_damping: 1.0e-05\nallocator_friction_coefficient: 0.5\nallocator_maximum_force_delta: 15.0\nallocator_minimum_normal_force: 2.0\ncomparison_adaptation_gain:\n- 0.25\n- 0.25\n- 0.25\n- 0.25\n- 0.25\n- 0.25\ncomparison_method: eth\ncomparison_tracking_scale:\n- 1\n- 1\n- 1\n- 1\n- 1\n- 1\ncompensation_aware_observer: true\ncontrol_gain_vector:\n- 0.25\n- 0.5\n- 0.5\n- 0.5\n- 0.08\n- 0.08\ncontrol_interface: wrench_allocator\ncontrol_magnitude_limit:\n- 8.0\n- 8.0\n- 15.0\n- 1.5\n- 1.5\n- 1.5\ncontrol_rate_limit_vector:\n- 20.0\n- 20.0\n- 30.0\n- 3.0\n- 3.0\n- 3.0\nestimator_version: eth_direct\nfilter_alpha: 0.5\nlambda_angular: 1.5\nlambda_linear: 2.0\noutput_scale:\n- 18.40147689318011\n- 18.757520233566048\n- 17.534482646627026\n- 2.3370077074651903\n- 5.466041588930613\n- 1.7261893548458138\nrepresentation: direct_parameter\nresidual_scale:\n- 18.40147689318011\n- 18.757520233566048\n- 17.534482646627026\n- 2.3370077074651903\n- 5.466041588930613\n- 1.7261893548458138\ntime_tolerance: 0.003\nupdate_period: 0.01\nweight_limit:\n- 20\n- 20\n- 30\n- 5\n- 5\n- 5\n"
  },
  {
    "source_directory": "/mnt/d/GO2/experi/results/v1_1/20260930T144035.376750Z_2180a5f3/main__flat_velocity_fy_sine__liu_van_mpc_port_standard__r001",
    "manifest": {
      "command": [
        "roslaunch",
        "legged_controllers",
        "go2_method_comparison.launch",
        "case_dir:=/mnt/d/GO2/experi/results/v1_1/20260930T144035.376750Z_2180a5f3/main__flat_velocity_fy_sine__liu_van_mpc_port_standard__r001",
        "controller_config:=/mnt/d/GO2/experi/results/v1_1/20260930T144035.376750Z_2180a5f3/main__flat_velocity_fy_sine__liu_van_mpc_port_standard__r001/controller_config.yaml",
        "method:=liu_van_mpc_port",
        "gui:=false",
        "gazebo_seed:=4102",
        "total_duration:=33.0",
        "motion_delay:=3.0",
        "motion_duration:=30.0",
        "vx:=0.45",
        "flat_motion:=true",
        "spawn_x:=0",
        "spawn_y:=0",
        "spawn_z:=0.35",
        "spawn_pitch:=0",
        "world:=/mnt/d/GO2/code/legged_robot_workspace/src/leggedcontrol_go2/legged_gazebo/worlds/myrbf_flat_control.world",
        "wrench_rate:=20.0",
        "wrench_hold:=0.25",
        "wrench_axis:=fy",
        "wrench_profile:=sine",
        "wrench_value:=0.0",
        "wrench_amplitude:=5.0",
        "wrench_frequency:=0.15",
        "wrench_secondary_amplitude:=0.0",
        "wrench_secondary_frequency:=0.3",
        "wrench_phase_deg:=90.0",
        "wrench_secondary_phase_deg:=90.0"
      ],
      "constraint_comparison_note": "Terrain traversal is not claimed; controller structural differences remain documented.",
      "constraint_surface_normal_world": [
        0,
        0,
        1
      ],
      "controller_config_sha256": "65fdb401ff924828c360c15f72214a4d5e5ccd01bd995d7c03731db83ee85546",
      "controller_config_source": "/mnt/d/GO2/code/legged_robot_workspace/src/leggedcontrol_go2/legged_controllers/config/go2/myrbf_shadow_liu_hybrid_go2.yaml",
      "controller_config_source_sha256": "5033638316ff4230816e8b82b4dd5408fa98330f064b8bc90e9089d45915dbc2",
      "data_kind": "simulation",
      "disturbance": {
        "amplitude": 5.0,
        "axis": "fy",
        "body": "go2::base",
        "command_hold_s": 0.25,
        "delay_s": 0.0,
        "duration_s": 33.0,
        "frequency": 0.15,
        "phase_deg": 90.0,
        "profile": "sine",
        "rate_hz": 20.0,
        "reference_frame": "world",
        "reference_point": [
          0.0,
          0.0,
          0.0
        ],
        "reference_point_source": "gazebo_world_net_wrench_zero_offset",
        "secondary_amplitude": 0.0,
        "secondary_frequency": 0.3,
        "secondary_phase_deg": 90.0,
        "torque_about": "go2::base link center of mass",
        "value": 0.0,
        "withdrawn_during_recording": false
      },
      "environment": {
        "DISPLAY": ":0",
        "GAZEBO_LOG_PATH": "/mnt/d/GO2/experi/results/v1_1/20260930T144035.376750Z_2180a5f3/main__flat_velocity_fy_sine__liu_van_mpc_port_standard__r001/gazebo_logs",
        "GAZEBO_MASTER_URI": "http://127.0.0.1:52783",
        "ROS_HOME": "/mnt/d/GO2/experi/results/v1_1/20260930T144035.376750Z_2180a5f3/main__flat_velocity_fy_sine__liu_van_mpc_port_standard__r001/ros_home",
        "ROS_LOG_DIR": "/mnt/d/GO2/experi/results/v1_1/20260930T144035.376750Z_2180a5f3/main__flat_velocity_fy_sine__liu_van_mpc_port_standard__r001/ros_logs",
        "ROS_MASTER_URI": "http://127.0.0.1:52971",
        "ROS_PYTHON_LOG_CONFIG_FILE": "/opt/ros/noetic/etc/ros/python_logging.conf"
      },
      "experiment": "GO2-METHOD-COMPARISON",
      "finished_utc": "2026-09-30T15:10:11.495595+00:00",
      "force_estimate_convention": "world_compensation_wrench_about_base_origin",
      "gait": "trot",
      "gui": false,
      "method": "liu_van_mpc_port",
      "method_definition": {
        "fidelity": "architecture_preserving_go2_port",
        "public_name": "Liu-VAN",
        "representation": "dual_gaussian_rbf",
        "update": "composite_error_variable_step"
      },
      "physics_step_s": 0.001,
      "planner": {
        "com_height_m": 0.33,
        "command_vx_mps": 0.45,
        "command_vy_mps": 0.0,
        "command_yaw_rate_rps": 0.0,
        "frame": "body",
        "gait_lead_s": 0.6,
        "heading_reference": "hold_initial_heading",
        "linear_velocity_target": "world_normalized_momentum_first3",
        "ramp_duration_s": 2.0,
        "rate_hz": 20.0,
        "reference_profile": "smooth_start_constant_velocity",
        "source": "cmd_vel",
        "terminal_hold_s": 0.0,
        "trajectory_id": "velocity",
        "trajectory_name": "constant_body_forward_velocity"
      },
      "raw_bag_recovered_from_active": true,
      "repetition": 1,
      "robot": "go2",
      "robust_mode": "none",
      "schema_version": 2,
      "seed": 4102,
      "simulator": "Gazebo Classic",
      "source_drift": [],
      "source_inventory_sha256": "bf5f34615a2a8158b50d6e45c952049633df4ff5cb1aa010ba98b558fac82f01",
      "started_utc": "2026-09-30T15:07:03.180168+00:00",
      "status": "recorded_validated",
      "study": "main",
      "terrain": "flat",
      "terrain_conditions": {
        "motion": true,
        "purpose": "flat_control_trial",
        "spawn": [
          0,
          0,
          0.35,
          0
        ],
        "surface_height_m": 0.0,
        "surface_normal_world": [
          0,
          0,
          1
        ]
      },
      "timing_s": {
        "final_hold": 0.0,
        "goal_delay": 3.0,
        "total": 33.0,
        "trajectory": 30.0
      },
      "variant": "standard"
    },
    "recording_validation": {
      "valid": true,
      "errors": [],
      "spans": {
        "state.csv": {
          "first_s": 0.017000000000000126,
          "last_s": 32.991,
          "max_gap_s": 0.02499999999999991,
          "rows": 1650
        },
        "wrench.csv": {
          "first_s": 0.014999999999999902,
          "last_s": 32.96,
          "max_gap_s": 0.055000000000000604,
          "rows": 680
        },
        "shadow.csv": {
          "first_s": 0.02299999999999991,
          "last_s": 32.999,
          "max_gap_s": 0.015000000000000568,
          "rows": 4481
        }
      },
      "diagnostic_summary": null,
      "campaign_start_ros": 1.518,
      "note": "Validation includes recording integrity and basic fall detection; it is not a theorem-verification claim."
    },
    "controller_config": "algorithm: v1\nallocator_damping: 1.0e-05\nallocator_friction_coefficient: 0.5\nallocator_maximum_force_delta: 15.0\nallocator_minimum_normal_force: 2.0\ncomparison_adaptation_gain:\n- 0.025\n- 0.025\n- 0.025\n- 0.025\n- 0.025\n- 0.025\ncomparison_composite_ratio: 0.75\ncomparison_gaussian_width: 0.5\ncomparison_maximum_step: 1.0\ncomparison_method: liu_van_mpc_port\ncomparison_minimum_step: 0.2\ncomparison_tracking_scale:\n- 1\n- 1\n- 1\n- 1\n- 1\n- 1\ncompensation_aware_observer: true\ncontrol_gain_vector:\n- 0.25\n- 0.5\n- 0.5\n- 0.5\n- 0.08\n- 0.08\ncontrol_interface: wrench_allocator\ncontrol_magnitude_limit:\n- 8.0\n- 8.0\n- 15.0\n- 1.5\n- 1.5\n- 1.5\ncontrol_rate_limit_vector:\n- 20.0\n- 20.0\n- 30.0\n- 3.0\n- 3.0\n- 3.0\ndiscrepancy_scale:\n- 18.40147689318011\n- 18.757520233566048\n- 17.534482646627026\n- 2.3370077074651903\n- 5.466041588930613\n- 1.7261893548458138\nestimator_version: liu_dual_rbf\nfilter_alpha: 0.5\nlambda_angular: 1.5\nlambda_linear: 2.0\noutput_scale:\n- 18.40147689318011\n- 18.757520233566048\n- 17.534482646627026\n- 2.3370077074651903\n- 5.466041588930613\n- 1.7261893548458138\nrepresentation: dual_gaussian_rbf\nresidual_scale:\n- 18.40147689318011\n- 18.757520233566048\n- 17.534482646627026\n- 2.3370077074651903\n- 5.466041588930613\n- 1.7261893548458138\ntime_tolerance: 0.003\nupdate_period: 0.01\nweight_limit:\n- 20\n- 20\n- 30\n- 5\n- 5\n- 5\n"
  },
  {
    "source_directory": "/mnt/d/GO2/experi/results/v1_1/20261001T030145.612466Z_bcc7525f/main__flat_velocity_fy_sine__zhou_ssi_standard__r001",
    "manifest": {
      "command": [
        "roslaunch",
        "legged_controllers",
        "go2_method_comparison.launch",
        "case_dir:=/mnt/d/GO2/experi/results/v1_1/20261001T030145.612466Z_bcc7525f/main__flat_velocity_fy_sine__zhou_ssi_standard__r001",
        "controller_config:=/mnt/d/GO2/experi/results/v1_1/20261001T030145.612466Z_bcc7525f/main__flat_velocity_fy_sine__zhou_ssi_standard__r001/controller_config.yaml",
        "method:=zhou_ssi",
        "gui:=false",
        "gazebo_seed:=4102",
        "total_duration:=33.0",
        "motion_delay:=3.0",
        "motion_duration:=30.0",
        "vx:=0.45",
        "flat_motion:=true",
        "spawn_x:=0",
        "spawn_y:=0",
        "spawn_z:=0.35",
        "spawn_pitch:=0",
        "world:=/mnt/d/GO2/code/legged_robot_workspace/src/leggedcontrol_go2/legged_gazebo/worlds/myrbf_flat_control.world",
        "wrench_rate:=20.0",
        "wrench_hold:=0.25",
        "wrench_axis:=fy",
        "wrench_profile:=sine",
        "wrench_value:=0.0",
        "wrench_amplitude:=5.0",
        "wrench_frequency:=0.15",
        "wrench_secondary_amplitude:=0.0",
        "wrench_secondary_frequency:=0.3",
        "wrench_phase_deg:=90.0",
        "wrench_secondary_phase_deg:=90.0"
      ],
      "constraint_comparison_note": "Terrain traversal is not claimed; controller structural differences remain documented.",
      "constraint_surface_normal_world": [
        0,
        0,
        1
      ],
      "controller_config_sha256": "c5732d49474425fdc56ffe2c3289b7d59069e3272e08fdecc55e2c7033888abd",
      "controller_config_source": "/mnt/d/GO2/code/legged_robot_workspace/src/leggedcontrol_go2/legged_controllers/config/go2/myrbf_shadow_zhou_ssi_rff_go2.yaml",
      "controller_config_source_sha256": "8d6f7c3cf2b3a6c20087f2c5e330592c8f12c0d2dfc16540e5ff7dc5e7d4dab6",
      "data_kind": "simulation",
      "disturbance": {
        "amplitude": 5.0,
        "axis": "fy",
        "body": "go2::base",
        "command_hold_s": 0.25,
        "delay_s": 0.0,
        "duration_s": 33.0,
        "frequency": 0.15,
        "phase_deg": 90.0,
        "profile": "sine",
        "rate_hz": 20.0,
        "reference_frame": "world",
        "reference_point": [
          0.0,
          0.0,
          0.0
        ],
        "reference_point_source": "gazebo_world_net_wrench_zero_offset",
        "secondary_amplitude": 0.0,
        "secondary_frequency": 0.3,
        "secondary_phase_deg": 90.0,
        "torque_about": "go2::base link center of mass",
        "value": 0.0,
        "withdrawn_during_recording": false
      },
      "environment": {
        "DISPLAY": ":0",
        "GAZEBO_LOG_PATH": "/mnt/d/GO2/experi/results/v1_1/20261001T030145.612466Z_bcc7525f/main__flat_velocity_fy_sine__zhou_ssi_standard__r001/gazebo_logs",
        "GAZEBO_MASTER_URI": "http://127.0.0.1:42533",
        "ROS_HOME": "/mnt/d/GO2/experi/results/v1_1/20261001T030145.612466Z_bcc7525f/main__flat_velocity_fy_sine__zhou_ssi_standard__r001/ros_home",
        "ROS_LOG_DIR": "/mnt/d/GO2/experi/results/v1_1/20261001T030145.612466Z_bcc7525f/main__flat_velocity_fy_sine__zhou_ssi_standard__r001/ros_logs",
        "ROS_MASTER_URI": "http://127.0.0.1:41503",
        "ROS_PYTHON_LOG_CONFIG_FILE": "/opt/ros/noetic/etc/ros/python_logging.conf"
      },
      "experiment": "GO2-METHOD-COMPARISON",
      "finished_utc": "2026-10-01T03:05:19.241072+00:00",
      "force_estimate_convention": "world_compensation_wrench_about_base_origin",
      "gait": "trot",
      "gui": false,
      "method": "zhou_ssi",
      "method_definition": {
        "fidelity": "interface_matched_rff_ogd",
        "public_name": "Zhou-RFF",
        "representation": "random_fourier_features",
        "update": "self_supervised_online_gradient_descent"
      },
      "physics_step_s": 0.001,
      "planner": {
        "com_height_m": 0.33,
        "command_vx_mps": 0.45,
        "command_vy_mps": 0.0,
        "command_yaw_rate_rps": 0.0,
        "frame": "body",
        "gait_lead_s": 0.6,
        "heading_reference": "hold_initial_heading",
        "linear_velocity_target": "world_normalized_momentum_first3",
        "ramp_duration_s": 2.0,
        "rate_hz": 20.0,
        "reference_profile": "smooth_start_constant_velocity",
        "source": "cmd_vel",
        "terminal_hold_s": 0.0,
        "trajectory_id": "velocity",
        "trajectory_name": "constant_body_forward_velocity"
      },
      "raw_bag_recovered_from_active": true,
      "repetition": 1,
      "robot": "go2",
      "robust_mode": "none",
      "schema_version": 2,
      "seed": 4102,
      "simulator": "Gazebo Classic",
      "source_drift": [],
      "source_inventory_sha256": "2bc103b4d5efdcbda9249ccf5fb93e31b5f47233addab03f1ac2a1d81e3849c7",
      "started_utc": "2026-10-01T03:01:49.054551+00:00",
      "status": "recorded_validated",
      "study": "main",
      "terrain": "flat",
      "terrain_conditions": {
        "motion": true,
        "purpose": "flat_control_trial",
        "spawn": [
          0,
          0,
          0.35,
          0
        ],
        "surface_height_m": 0.0,
        "surface_normal_world": [
          0,
          0,
          1
        ]
      },
      "timing_s": {
        "final_hold": 0.0,
        "goal_delay": 3.0,
        "total": 33.0,
        "trajectory": 30.0
      },
      "variant": "standard"
    },
    "recording_validation": {
      "valid": true,
      "errors": [],
      "spans": {
        "state.csv": {
          "first_s": 0.014999999999999902,
          "last_s": 32.987,
          "max_gap_s": 0.025000000000000355,
          "rows": 1650
        },
        "wrench.csv": {
          "first_s": 0.017999999999999794,
          "last_s": 32.957,
          "max_gap_s": 0.05699999999999861,
          "rows": 680
        },
        "shadow.csv": {
          "first_s": 0.020000000000000018,
          "last_s": 32.999,
          "max_gap_s": 0.01699999999999946,
          "rows": 4479
        }
      },
      "diagnostic_summary": null,
      "campaign_start_ros": 1.5070000000000001,
      "note": "Validation includes recording integrity and basic fall detection; it is not a theorem-verification claim."
    },
    "controller_config": "algorithm: v1\nallocator_damping: 1.0e-05\nallocator_friction_coefficient: 0.5\nallocator_maximum_force_delta: 15.0\nallocator_minimum_normal_force: 2.0\nangular_extent: 4.0\ncomparison_method: zhou_ssi\ncompensation_aware_observer: true\ncontrol_gain_vector:\n- 0.25\n- 0.5\n- 0.5\n- 0.5\n- 0.08\n- 0.08\ncontrol_interface: wrench_allocator\ncontrol_magnitude_limit:\n- 8.0\n- 8.0\n- 15.0\n- 1.5\n- 1.5\n- 1.5\ncontrol_rate_limit_vector:\n- 20.0\n- 20.0\n- 30.0\n- 3.0\n- 3.0\n- 3.0\ndiscrepancy_scale:\n- 18.40147689318011\n- 18.757520233566048\n- 17.534482646627026\n- 2.3370077074651903\n- 5.466041588930613\n- 1.7261893548458138\nestimator_version: rff\nfilter_alpha: 0.5\nlambda_angular: 1.5\nlambda_linear: 2.0\nleakage:\n- 0\n- 0\n- 0\n- 0\n- 0\n- 0\nlinear_extent: 2.0\noutput_scale:\n- 18.40147689318011\n- 18.757520233566048\n- 17.534482646627026\n- 2.3370077074651903\n- 5.466041588930613\n- 1.7261893548458138\nprediction_gain:\n- 0\n- 0\n- 0\n- 0\n- 0\n- 0\nrepresentation: random_fourier_features\nresidual_scale:\n- 18.40147689318011\n- 18.757520233566048\n- 17.534482646627026\n- 2.3370077074651903\n- 5.466041588930613\n- 1.7261893548458138\nrff_feature_count: 50\nrff_frequency_std: 1.0\nrff_learning_rate:\n- 0.003\n- 0.003\n- 0.003\n- 0.003\n- 0.003\n- 0.003\nrff_projection_radius: 0.0\nrff_random_seed: 20260824\ntime_tolerance: 0.003\ntracking_gain:\n- 0\n- 0\n- 0\n- 0\n- 0\n- 0\nupdate_period: 0.01\nwidth_ratio: 0.85\n"
  }
]
```
