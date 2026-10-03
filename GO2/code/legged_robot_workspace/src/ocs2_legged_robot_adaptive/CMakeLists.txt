cmake_minimum_required(VERSION 3.0.2)
project(ocs2_legged_robot_adaptive)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CATKIN_PACKAGE_DEPENDENCIES
  ocs2_core
  ocs2_oc
  ocs2_ddp
  ocs2_mpc
  ocs2_sqp
  ocs2_ipm
  ocs2_robotic_tools
  ocs2_pinocchio_interface
  ocs2_centroidal_model
  ocs2_robotic_assets
  ocs2_legged_robot
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
    ${EIGEN3_INCLUDE_DIRS}
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
  DEPENDS
    Boost
    pinocchio
)

configure_file(
  "${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/package_path.h.in"
  "${PROJECT_BINARY_DIR}/include/${PROJECT_NAME}/package_path.h" @ONLY
)

set(FLAGS
  ${OCS2_CXX_FLAGS}
  ${pinocchio_CFLAGS_OTHER}
  -Wno-ignored-attributes
  -Wno-invalid-partial-specialization
  -DPINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR
  -DPINOCCHIO_URDFDOM_USE_STD_SHARED_PTR
)

include_directories(
  include
  ${PROJECT_BINARY_DIR}/include
  ${pinocchio_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  ${pinocchio_LIBRARY_DIRS}
)

add_library(${PROJECT_NAME}
  src/adaptive/AdaptiveTypes.cpp
  src/adaptive/PayloadMass1D.cpp
  src/adaptive/PayloadInertial10D.cpp
  src/adaptive/QuaternionTrackingError.cpp
  src/adaptive/SlidingVariable.cpp
  src/adaptive/AdaptiveModelSnapshot.cpp
  src/adaptive/AdaptiveParameterEstimator.cpp
  src/adaptive/GaitPhaseBaselineCorrector.cpp
  src/adaptive/CompositePhaseBaselineCorrector.cpp
  src/adaptive/AdaptiveRigidBodyRegressor.cpp
  src/adaptive/AdaptiveClf.cpp
  src/adaptive/AdaptiveClfEvaluator.cpp
  src/adaptive/AdaptiveReferenceKinematics.cpp
  src/adaptive/IdentificationReferenceProfile.cpp
  src/adaptive/AdaptiveReferenceStateEncoder.cpp
  src/adaptive/AdaptiveStateAdapter.cpp
  src/adaptive/AdaptiveModelData.cpp
  src/adaptive/AdaptiveParameterUpdateModule.cpp
  src/adaptive/CompositeRbfWrenchEstimator.cpp
  src/adaptive/WrenchResidualEstimator.cpp
  src/adaptive/RbfWrenchNetwork.cpp
  src/adaptive/RbfWrenchInputFeature.cpp
  src/adaptive/RbfWrenchClosedLoopUpdateModule.cpp
  src/adaptive/RandomFourierWrenchEstimator.cpp
  src/adaptive/SixSubnetworkRbfWrenchNetwork.cpp
  src/adaptive/TwoFeatureMinnitiRbfWrenchEstimator.cpp
  src/adaptive/TwoFeatureRbfWrenchInput.cpp
  src/adaptive/TwoFeatureSixSubnetworkRbfWrenchNetwork.cpp
  src/adaptive/TwoFeatureSixSubnetworkRbfWeightAdapter.cpp
  src/dynamics/AdaptiveLeggedRobotDynamics.cpp
  src/dynamics/MassMismatchGroundTruth.cpp
  src/dynamics/NominalCoordinatePlantDynamics.cpp
  src/dynamics/PayloadInertialRuntimeDiagnostics.cpp
  src/dynamics/PayloadInertialGroundTruth.cpp
  src/constraint/AdaptiveClfConstraint.cpp
  src/AdaptiveLeggedRobotInterface.cpp
)
add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${pinocchio_LIBRARIES}
  dl
)
target_compile_options(${PROJECT_NAME} PUBLIC ${FLAGS})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_adaptive_math test/testAdaptiveMath.cpp)
  if(TARGET test_adaptive_math)
    target_link_libraries(test_adaptive_math ${PROJECT_NAME} ${catkin_LIBRARIES} gtest_main)
    target_compile_options(test_adaptive_math PRIVATE ${FLAGS})
  endif()
  catkin_add_gtest(test_payload_inertial_10d test/testPayloadInertial10D.cpp)
  if(TARGET test_payload_inertial_10d)
    target_link_libraries(test_payload_inertial_10d ${PROJECT_NAME} ${catkin_LIBRARIES} gtest_main)
    target_compile_options(test_payload_inertial_10d PRIVATE ${FLAGS})
  endif()
  catkin_add_gtest(test_payload_inertial_plus_wrench_16d
    test/testPayloadInertialPlusWrench16D.cpp)
  if(TARGET test_payload_inertial_plus_wrench_16d)
    target_link_libraries(test_payload_inertial_plus_wrench_16d ${PROJECT_NAME} ${catkin_LIBRARIES} gtest_main)
    target_compile_options(test_payload_inertial_plus_wrench_16d PRIVATE ${FLAGS})
  endif()
  catkin_add_gtest(test_gait_phase_baseline_corrector
    test/testGaitPhaseBaselineCorrector.cpp
    test/testCompositePhaseBaselineCorrector.cpp)
  if(TARGET test_gait_phase_baseline_corrector)
    target_link_libraries(test_gait_phase_baseline_corrector
      ${PROJECT_NAME} ${catkin_LIBRARIES} gtest_main)
    target_compile_options(test_gait_phase_baseline_corrector PRIVATE ${FLAGS})
  endif()
  catkin_add_gtest(test_wrench_residual_estimator
    test/testWrenchResidualEstimator.cpp)
  if(TARGET test_wrench_residual_estimator)
    target_link_libraries(test_wrench_residual_estimator ${PROJECT_NAME} ${catkin_LIBRARIES} gtest_main)
    target_compile_options(test_wrench_residual_estimator PRIVATE ${FLAGS})
  endif()
  catkin_add_gtest(test_rbf_wrench_network test/testRbfWrenchNetwork.cpp)
  if(TARGET test_rbf_wrench_network)
    target_link_libraries(test_rbf_wrench_network ${PROJECT_NAME} ${catkin_LIBRARIES} gtest_main)
    target_compile_options(test_rbf_wrench_network PRIVATE ${FLAGS})
  endif()
  catkin_add_gtest(test_rbf_wrench_input_feature test/testRbfWrenchInputFeature.cpp)
  if(TARGET test_rbf_wrench_input_feature)
    target_link_libraries(test_rbf_wrench_input_feature ${PROJECT_NAME} ${catkin_LIBRARIES} gtest_main)
    target_compile_options(test_rbf_wrench_input_feature PRIVATE ${FLAGS})
  endif()
  catkin_add_gtest(test_six_subnetwork_rbf_wrench_network test/testSixSubnetworkRbfWrenchNetwork.cpp)
  if(TARGET test_six_subnetwork_rbf_wrench_network)
    target_link_libraries(test_six_subnetwork_rbf_wrench_network ${PROJECT_NAME} ${catkin_LIBRARIES} gtest_main)
    target_compile_options(test_six_subnetwork_rbf_wrench_network PRIVATE ${FLAGS})
  endif()
  catkin_add_gtest(test_two_feature_rbf_wrench_input test/testTwoFeatureRbfWrenchInput.cpp)
  if(TARGET test_two_feature_rbf_wrench_input)
    target_link_libraries(test_two_feature_rbf_wrench_input ${PROJECT_NAME} ${catkin_LIBRARIES} gtest_main)
    target_compile_options(test_two_feature_rbf_wrench_input PRIVATE ${FLAGS})
  endif()
  catkin_add_gtest(test_two_feature_six_subnetwork_rbf_wrench_network test/testTwoFeatureSixSubnetworkRbfWrenchNetwork.cpp)
  if(TARGET test_two_feature_six_subnetwork_rbf_wrench_network)
    target_link_libraries(test_two_feature_six_subnetwork_rbf_wrench_network ${PROJECT_NAME} ${catkin_LIBRARIES} gtest_main)
    target_compile_options(test_two_feature_six_subnetwork_rbf_wrench_network PRIVATE ${FLAGS})
  endif()
  catkin_add_gtest(test_two_feature_six_subnetwork_rbf_weight_adapter test/testTwoFeatureSixSubnetworkRbfWeightAdapter.cpp)
  if(TARGET test_two_feature_six_subnetwork_rbf_weight_adapter)
    target_link_libraries(test_two_feature_six_subnetwork_rbf_weight_adapter ${PROJECT_NAME} ${catkin_LIBRARIES} gtest_main)
    target_compile_options(test_two_feature_six_subnetwork_rbf_weight_adapter PRIVATE ${FLAGS})
  endif()
  catkin_add_gtest(test_two_feature_minniti_rbf_wrench_estimator test/testTwoFeatureMinnitiRbfWrenchEstimator.cpp)
  if(TARGET test_two_feature_minniti_rbf_wrench_estimator)
    target_link_libraries(test_two_feature_minniti_rbf_wrench_estimator ${PROJECT_NAME} ${catkin_LIBRARIES} gtest_main)
    target_compile_options(test_two_feature_minniti_rbf_wrench_estimator PRIVATE ${FLAGS})
  endif()
  catkin_add_gtest(test_random_fourier_wrench_estimator test/testRandomFourierWrenchEstimator.cpp)
  if(TARGET test_random_fourier_wrench_estimator)
    target_link_libraries(test_random_fourier_wrench_estimator ${PROJECT_NAME} ${catkin_LIBRARIES} gtest_main)
    target_compile_options(test_random_fourier_wrench_estimator PRIVATE ${FLAGS})
  endif()
  catkin_add_gtest(test_composite_rbf_wrench_estimator test/testCompositeRbfWrenchEstimator.cpp)
  if(TARGET test_composite_rbf_wrench_estimator)
    target_link_libraries(test_composite_rbf_wrench_estimator ${PROJECT_NAME} ${catkin_LIBRARIES} gtest_main)
    target_compile_options(test_composite_rbf_wrench_estimator PRIVATE ${FLAGS})
  endif()
  catkin_add_gtest(test_rbf_wrench_closed_loop_update_module test/testRbfWrenchClosedLoopUpdateModule.cpp)
  if(TARGET test_rbf_wrench_closed_loop_update_module)
    target_link_libraries(test_rbf_wrench_closed_loop_update_module ${PROJECT_NAME} ${catkin_LIBRARIES} gtest_main)
    target_compile_options(test_rbf_wrench_closed_loop_update_module PRIVATE ${FLAGS})
  endif()
endif()

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
