cmake_minimum_required(VERSION 3.0.2)
project(ocs2_legged_robot_adaptive_ros)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CATKIN_PACKAGE_DEPENDENCIES
  roslib
  cmake_modules
  tf
  urdf
  kdl_parser
  robot_state_publisher
  std_msgs
  geometry_msgs
  ocs2_core
  ocs2_oc
  ocs2_ddp
  ocs2_mpc
  ocs2_robotic_tools
  ocs2_pinocchio_interface
  ocs2_centroidal_model
  ocs2_robotic_assets
  ocs2_msgs
  ocs2_ros_interfaces
  ocs2_legged_robot
  ocs2_legged_robot_ros
  ocs2_legged_robot_adaptive
)

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_PACKAGE_DEPENDENCIES}
)

find_package(Boost REQUIRED COMPONENTS
  system
  filesystem
)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

find_package(PkgConfig REQUIRED)
pkg_check_modules(pinocchio REQUIRED pinocchio)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    matched_wrench_disturbance
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
  DEPENDS
    Boost
    pinocchio
)

include_directories(
  include
  ${pinocchio_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  ${pinocchio_LIBRARY_DIRS}
)

add_library(matched_wrench_disturbance
  src/MatchedWrenchDisturbance.cpp
)
add_dependencies(matched_wrench_disturbance
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(matched_wrench_disturbance ${catkin_LIBRARIES})
target_compile_options(matched_wrench_disturbance PUBLIC ${OCS2_CXX_FLAGS})

add_executable(legged_robot_adaptive_ddp_mpc
  src/LeggedRobotAdaptiveDdpMpcNode.cpp
)
add_dependencies(legged_robot_adaptive_ddp_mpc
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(legged_robot_adaptive_ddp_mpc
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)
target_compile_options(legged_robot_adaptive_ddp_mpc PRIVATE ${OCS2_CXX_FLAGS})

add_executable(legged_robot_adaptive_dummy
  src/LeggedRobotAdaptiveDummyNode.cpp
)
add_dependencies(legged_robot_adaptive_dummy
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(legged_robot_adaptive_dummy
  matched_wrench_disturbance
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)
target_compile_options(legged_robot_adaptive_dummy PRIVATE ${OCS2_CXX_FLAGS})

add_executable(deterministic_locomotion_scheduler
  src/DeterministicLocomotionSchedulerNode.cpp
)
add_dependencies(deterministic_locomotion_scheduler
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(deterministic_locomotion_scheduler
  ${catkin_LIBRARIES}
)
target_compile_options(deterministic_locomotion_scheduler PRIVATE ${OCS2_CXX_FLAGS})

add_executable(identification_reference_scheduler
  src/IdentificationReferenceSchedulerNode.cpp
)
add_dependencies(identification_reference_scheduler
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(identification_reference_scheduler
  ${catkin_LIBRARIES}
)
target_compile_options(identification_reference_scheduler PRIVATE ${OCS2_CXX_FLAGS})

# D5I1 pure offline profile/regressor dumper.  It has no ROS publishers and
# cannot start animal_dummy or modify an experiment graph.
add_executable(task_d_d5i1_profile_dump
  src/TaskDD5I1ProfileDump.cpp
)
add_dependencies(task_d_d5i1_profile_dump
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(task_d_d5i1_profile_dump
  ${catkin_LIBRARIES}
)
target_compile_options(task_d_d5i1_profile_dump PRIVATE ${OCS2_CXX_FLAGS})

add_executable(task_d_d5i1_harmonic_search
  src/TaskDD5I1HarmonicSearch.cpp
)
add_dependencies(task_d_d5i1_harmonic_search
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(task_d_d5i1_harmonic_search
  ${catkin_LIBRARIES}
)
target_compile_options(task_d_d5i1_harmonic_search PRIVATE ${OCS2_CXX_FLAGS})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_matched_wrench_disturbance
    test/testMatchedWrenchDisturbance.cpp
  )
  if(TARGET test_matched_wrench_disturbance)
    target_link_libraries(test_matched_wrench_disturbance matched_wrench_disturbance ${catkin_LIBRARIES} ${Boost_LIBRARIES} gtest_main)
    target_compile_options(test_matched_wrench_disturbance PRIVATE ${OCS2_CXX_FLAGS})
  endif()
  catkin_add_nosetests(test/test_analyze_adaptive_update.py)
  catkin_add_nosetests(test/test_set_adaptive_experiment_mode.py)
  catkin_add_nosetests(test/test_target_profiles.py)
  catkin_add_nosetests(test/test_experiment_metrics.py)
  catkin_add_nosetests(test/test_task_c_metrics.py)
  catkin_add_nosetests(test/test_analyze_task_b.py)
  catkin_add_nosetests(test/test_analyze_task_c_pairing.py)
  catkin_add_nosetests(test/test_task_d_mode.py)
  catkin_add_nosetests(test/test_task_d_d4_analysis.py)
  catkin_add_nosetests(test/test_task_d_d5_identifiability.py)
  catkin_add_nosetests(test/test_task_d_d5b_bias_attribution.py)
  catkin_add_nosetests(test/test_task_d_d5c_nominal_residual.py)
  catkin_add_nosetests(test/test_task_d_d5d_sigma_subterms.py)
  catkin_add_nosetests(test/test_task_d_d5e_feedback_pair.py)
  catkin_add_nosetests(test/test_task_d_d5f_gait_phase_covariance.py)
  catkin_add_nosetests(test/test_task_d_d5g_candidate_updates.py)
  catkin_add_nosetests(test/test_task_d_d5h0_shadow_diagnostics.py)
  catkin_add_nosetests(test/test_task_d_d5h1_mode.py)
  catkin_add_nosetests(test/test_task_d_d5h1_gate.py)
  catkin_add_nosetests(test/test_task_d_d5h1_formal.py)
  catkin_add_nosetests(test/test_task_d_d5h2_startup_gate.py)
  catkin_add_nosetests(test/test_task_d_d5i0_design_audit.py)
  catkin_add_nosetests(test/test_task_d_d5i1_profile_audit.py)
  catkin_add_nosetests(test/test_task_d_d5i2_smoke.py)
  catkin_add_nosetests(test/test_task_d_d5i3_calibration.py)
  catkin_add_nosetests(test/test_task_d_d5i4_smoke.py)
  catkin_add_nosetests(test/test_analyze_adaptive_method_comparison.py)
endif()

install(
  TARGETS
    matched_wrench_disturbance
    legged_robot_adaptive_ddp_mpc
    legged_robot_adaptive_dummy
    deterministic_locomotion_scheduler
    identification_reference_scheduler
    task_d_d5i1_profile_dump
    task_d_d5i1_harmonic_search
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(
  PROGRAMS
    scripts/aclf_observation_monitor.py
    scripts/aclf_small_step_target.py
    scripts/analyze_aclf_ab.py
    scripts/analyze_adaptive_update.py
    scripts/analyze_task_b.py
    scripts/analyze_task_c.py
    scripts/set_adaptive_experiment_mode.py
    scripts/target_profiles.py
    scripts/experiment_metrics.py
    scripts/analyze_task_b_smoke.py
    scripts/analyze_task_b_campaign.py
    scripts/analyze_task_c_campaign.py
    scripts/analyze_task_c_smoke.py
    scripts/analyze_payload_mass_smoke.py
    scripts/analyze_task_c_payload_mass_campaign.py
    scripts/analyze_task_c_locomotion_smoke.py
    scripts/analyze_task_d_payload_inertial_smoke.py
    scripts/analyze_task_d_d3_sanity.py
    scripts/analyze_task_d_d4_run.py
    scripts/analyze_task_d_d4_campaign.py
    scripts/analyze_task_d_d5_identifiability.py
    scripts/analyze_task_d_d5b_run.py
    scripts/analyze_task_d_d5b_campaign.py
    scripts/analyze_task_d_d5c_nominal_residual.py
    scripts/analyze_task_d_d5d_sigma_subterms.py
    scripts/analyze_task_d_d5e_feedback_pair.py
    scripts/analyze_task_d_d5f_gait_phase_covariance.py
    scripts/analyze_task_d_d5g_candidate_updates.py
    scripts/analyze_task_d_d5h0_shadow.py
    scripts/analyze_task_d_d5h0_campaign.py
    scripts/d5h0_shadow_diagnostics.py
    scripts/analyze_task_d_d5h1_smoke.py
    scripts/analyze_task_d_d5h1_smoke_set.py
    scripts/analyze_task_d_d5h1_formal_run.py
    scripts/analyze_task_d_d5h1_formal_campaign.py
    scripts/analyze_task_d_d5h2_startup_gate.py
    scripts/analyze_task_d_d5i0_design_audit.py
    scripts/analyze_task_d_d5i1_profile_audit.py
    scripts/analyze_task_d_d5i2_smoke.py
    scripts/analyze_task_d_d5i3_campaign.py
    scripts/analyze_task_d_d5i4_smoke.py
    scripts/analyze_task_d_d5i4_smoke_set.py
  scripts/analyze_rbf_closed_loop_validation.py
  scripts/analyze_rbf_tuned_campaign.py
  scripts/analyze_adaptive_method_comparison.py
    scripts/audit_task_d_d5h1_gate.py
    scripts/audit_task_d_d5h1_four_mode_gate.py
    scripts/audit_task_d_d5e_protocol.py
    scripts/audit_task_c_payload_mass_campaign.py
    scripts/audit_task_d_d3_sanity.py
    scripts/audit_task_d_d4_protocol.py
    scripts/audit_task_d_d4_campaign.py
    scripts/audit_task_d_d5b_protocol.py
    scripts/audit_task_d_d5b_campaign.py
    scripts/audit_task_c_payload_mass_adaptive_off.py
    scripts/audit_task_c_locomotion_payload_campaign.py
    scripts/audit_task_c_adaptive_off.py
    scripts/audit_task_c_campaign.py
    scripts/set_aclf_experiment_mode.py
    scripts/set_task_b_mode.py
    scripts/set_task_d_mode.py
    scripts/set_task_d_d5h1_mode.py
    scripts/set_task_d_d5i2_mode.py
    scripts/set_task_d_d5i3_mode.py
    scripts/set_task_d_d5i4_mode.py
    scripts/prepare_task_d_d5h1_formal_protocol.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(
  PROGRAMS
    scripts/run_adaptive_update_validation.sh
    scripts/run_task_b_comparison.sh
    scripts/run_task_c_comparison.sh
    scripts/run_task_b_campaign.sh
    scripts/summarize_task_b_acquisition.py
    scripts/run_task_b_experiment.sh
    scripts/run_task_c_adaptive_off_campaign.sh
    scripts/run_task_c_mass_campaign.sh
    scripts/run_task_c_mass_experiment.sh
    scripts/run_task_c_smoke.sh
    scripts/run_task_c_payload_mass_smoke.sh
    scripts/run_task_c_payload_mass_formal.sh
    scripts/run_task_c_payload_mass_campaign.sh
    scripts/run_task_c_payload_mass_adaptive_off_campaign.sh
    scripts/run_task_c_locomotion_smoke.sh
    scripts/run_task_c_locomotion_payload_experiment.sh
    scripts/run_task_c_locomotion_payload_campaign.sh
    scripts/run_task_d_payload_inertial_smoke.sh
    scripts/run_task_d_d3_sanity.sh
    scripts/run_task_d_d3_sanity_campaign.sh
    scripts/run_task_d_d4_experiment.sh
    scripts/run_task_d_d4_campaign.sh
    scripts/run_task_d_d5b_experiment.sh
    scripts/run_task_d_d5b_campaign.sh
    scripts/run_task_d_d5e_experiment.sh
    scripts/run_task_d_d5h0_experiment.sh
    scripts/run_task_d_d5h0_campaign.sh
    scripts/run_task_d_d5h1_experiment.sh
    scripts/run_task_d_d5h1_gate_smoke_campaign.sh
    scripts/run_task_d_d5h1_formal_campaign.sh
    scripts/run_task_d_d5i2_smoke.sh
    scripts/run_task_d_d5i3_calibration.sh
    scripts/run_task_d_d5i3_campaign.sh
    scripts/run_task_d_d5i4_smoke.sh
    scripts/run_task_d_d5i4_smoke_set.sh
    scripts/run_rbf_closed_loop_validation.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
