cmake_minimum_required(VERSION 3.10)
project(legged_interface)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        ocs2_legged_robot
        ocs2_self_collision
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
        ${PROJECT_NAME}
        CATKIN_DEPENDS
        roscpp
        ocs2_legged_robot
        ocs2_self_collision
        DEPENDS
        pinocchio
)

###########
## Build ##
###########

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        ${pinocchio_INCLUDE_DIRS}
)

link_directories(
        ${pinocchio_LIBRARY_DIRS}
)

add_library(${PROJECT_NAME}
        src/adaptive/MyRbfPredictionResidualObserver.cpp
        src/adaptive/MyRbfFyCompensation.cpp
        src/adaptive/MyRbfWrenchCompensation.cpp
        src/adaptive/MyRbfTrackingError.cpp
        src/adaptive/MyRbfV2Core.cpp
        src/adaptive/MyRbfV5DynamicsCore.cpp
        src/adaptive/MyRbfAugmentedLeggedDynamics.cpp
        src/adaptive/MyRbfV2Pipeline.cpp
        src/adaptive/ComparisonWrenchEstimator.cpp
        src/adaptive/MyRbfShadowEstimator.cpp
        src/adaptive/RandomFourierWrenchEstimator.cpp
        src/adaptive/MyRbfShadowPipeline.cpp
        src/adaptive/modules/PeriodicResidualModule.cpp
        src/adaptive/modules/PayloadIdModule.cpp
        src/adaptive/modules/ResidualDynamicsModule.cpp
        src/adaptive/modules/AdaptiveFeatureModule.cpp
        src/adaptive/modules/ResidualConfidenceModule.cpp
        src/adaptive/modules/AdaptiveModuleShadowPipeline.cpp
        src/constraint/EndEffectorLinearConstraint.cpp
        src/constraint/FrictionConeConstraint.cpp
        src/constraint/ZeroForceConstraint.cpp
        src/constraint/NormalVelocityConstraintCppAd.cpp
        src/constraint/ZeroVelocityConstraintCppAd.cpp
        src/constraint/SwingTrajectoryPlanner.cpp
        src/initialization/LeggedRobotInitializer.cpp
        src/SwitchedModelReferenceManager.cpp
        src/LeggedRobotPreComputation.cpp
        src/LeggedInterface.cpp
        )

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${pinocchio_LIBRARIES}
        )

target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

if (CATKIN_ENABLE_TESTING)
    catkin_add_gtest(${PROJECT_NAME}_myrbf_v2_augmented_dynamics_test test/adaptive/TestMyRbfV2AugmentedDynamics.cpp)
    target_link_libraries(${PROJECT_NAME}_myrbf_v2_augmented_dynamics_test ${PROJECT_NAME} ${catkin_LIBRARIES} ${pinocchio_LIBRARIES} gtest_main)
    target_compile_options(${PROJECT_NAME}_myrbf_v2_augmented_dynamics_test PUBLIC ${OCS2_CXX_FLAGS})
    add_executable(myrbf_v2_core_test test/adaptive/TestMyRbfV2Core.cpp src/adaptive/MyRbfV2Core.cpp)
    target_compile_features(myrbf_v2_core_test PRIVATE cxx_std_14)
    add_test(NAME myrbf_v2_core_test COMMAND myrbf_v2_core_test)
    add_executable(myrbf_v2_theory_core_test test/adaptive/TestMyRbfV2TheoryCore.cpp src/adaptive/MyRbfV2Core.cpp)
    target_compile_features(myrbf_v2_theory_core_test PRIVATE cxx_std_14)
    add_test(NAME myrbf_v2_theory_core_test COMMAND myrbf_v2_theory_core_test)
    add_executable(myrbf_v5_dynamics_core_test
            test/adaptive/TestMyRbfV5DynamicsCore.cpp
            src/adaptive/MyRbfV5DynamicsCore.cpp)
    target_compile_features(myrbf_v5_dynamics_core_test PRIVATE cxx_std_14)
    add_test(NAME myrbf_v5_dynamics_core_test COMMAND myrbf_v5_dynamics_core_test)
    catkin_add_gtest(${PROJECT_NAME}_myrbf_v2_external_pipeline_test test/adaptive/TestMyRbfV2ExternalPipeline.cpp)
    target_link_libraries(${PROJECT_NAME}_myrbf_v2_external_pipeline_test ${PROJECT_NAME} ${catkin_LIBRARIES} ${pinocchio_LIBRARIES} gtest_main)
    target_compile_options(${PROJECT_NAME}_myrbf_v2_external_pipeline_test PUBLIC ${OCS2_CXX_FLAGS})
    catkin_add_gtest(${PROJECT_NAME}_myrbf_v2_pipeline_test
            test/adaptive/TestMyRbfV2Pipeline.cpp)
    target_link_libraries(${PROJECT_NAME}_myrbf_v2_pipeline_test
            ${PROJECT_NAME} ${catkin_LIBRARIES} ${pinocchio_LIBRARIES} gtest_main)
    target_compile_options(${PROJECT_NAME}_myrbf_v2_pipeline_test PUBLIC ${OCS2_CXX_FLAGS})
    catkin_add_gtest(${PROJECT_NAME}_myrbf_shadow_estimator_test
            test/adaptive/TestMyRbfShadowEstimator.cpp)
    target_link_libraries(${PROJECT_NAME}_myrbf_shadow_estimator_test
            ${PROJECT_NAME} ${catkin_LIBRARIES} ${pinocchio_LIBRARIES} gtest_main)
    target_compile_options(${PROJECT_NAME}_myrbf_shadow_estimator_test PUBLIC ${OCS2_CXX_FLAGS})

    catkin_add_gtest(${PROJECT_NAME}_random_fourier_wrench_estimator_test
            test/adaptive/TestRandomFourierWrenchEstimator.cpp)
    target_link_libraries(${PROJECT_NAME}_random_fourier_wrench_estimator_test
            ${PROJECT_NAME} ${catkin_LIBRARIES} ${pinocchio_LIBRARIES} gtest_main)
    target_compile_options(${PROJECT_NAME}_random_fourier_wrench_estimator_test PUBLIC ${OCS2_CXX_FLAGS})

    catkin_add_gtest(${PROJECT_NAME}_comparison_wrench_estimator_test
            test/adaptive/TestComparisonWrenchEstimator.cpp)
    target_link_libraries(${PROJECT_NAME}_comparison_wrench_estimator_test
            ${PROJECT_NAME} ${catkin_LIBRARIES} ${pinocchio_LIBRARIES} gtest_main)
    target_compile_options(${PROJECT_NAME}_comparison_wrench_estimator_test PUBLIC ${OCS2_CXX_FLAGS})

    catkin_add_gtest(${PROJECT_NAME}_myrbf_prediction_residual_observer_test
            test/adaptive/TestMyRbfPredictionResidualObserver.cpp)
    target_link_libraries(${PROJECT_NAME}_myrbf_prediction_residual_observer_test
            ${PROJECT_NAME} ${catkin_LIBRARIES} ${pinocchio_LIBRARIES} gtest_main)
    target_compile_options(${PROJECT_NAME}_myrbf_prediction_residual_observer_test PUBLIC ${OCS2_CXX_FLAGS})

    catkin_add_gtest(${PROJECT_NAME}_myrbf_shadow_pipeline_test
            test/adaptive/TestMyRbfShadowPipeline.cpp)
    target_link_libraries(${PROJECT_NAME}_myrbf_shadow_pipeline_test
            ${PROJECT_NAME} ${catkin_LIBRARIES} ${pinocchio_LIBRARIES} gtest_main)
    target_compile_options(${PROJECT_NAME}_myrbf_shadow_pipeline_test PUBLIC ${OCS2_CXX_FLAGS})

    catkin_add_gtest(${PROJECT_NAME}_myrbf_tracking_error_test
            test/adaptive/TestMyRbfTrackingError.cpp)
    target_link_libraries(${PROJECT_NAME}_myrbf_tracking_error_test
            ${PROJECT_NAME} ${catkin_LIBRARIES} ${pinocchio_LIBRARIES} gtest_main)
    target_compile_options(${PROJECT_NAME}_myrbf_tracking_error_test PUBLIC ${OCS2_CXX_FLAGS})

    catkin_add_gtest(${PROJECT_NAME}_myrbf_fy_compensation_test
            test/adaptive/TestMyRbfFyCompensation.cpp)
    target_link_libraries(${PROJECT_NAME}_myrbf_fy_compensation_test
            ${PROJECT_NAME} ${catkin_LIBRARIES} ${pinocchio_LIBRARIES} gtest_main)
    target_compile_options(${PROJECT_NAME}_myrbf_fy_compensation_test PUBLIC ${OCS2_CXX_FLAGS})

    catkin_add_gtest(${PROJECT_NAME}_myrbf_wrench_compensation_test
            test/adaptive/TestMyRbfWrenchCompensation.cpp)
    target_link_libraries(${PROJECT_NAME}_myrbf_wrench_compensation_test
            ${PROJECT_NAME} ${catkin_LIBRARIES} ${pinocchio_LIBRARIES} gtest_main)
    target_compile_options(${PROJECT_NAME}_myrbf_wrench_compensation_test PUBLIC ${OCS2_CXX_FLAGS})

    catkin_add_gtest(${PROJECT_NAME}_periodic_residual_module_test
            test/adaptive/modules/TestPeriodicResidualModule.cpp)
    target_link_libraries(${PROJECT_NAME}_periodic_residual_module_test
            ${PROJECT_NAME} ${catkin_LIBRARIES} ${pinocchio_LIBRARIES} gtest_main)
    target_compile_options(${PROJECT_NAME}_periodic_residual_module_test PUBLIC ${OCS2_CXX_FLAGS})

    catkin_add_gtest(${PROJECT_NAME}_payload_id_module_test
            test/adaptive/modules/TestPayloadIdModule.cpp)
    target_link_libraries(${PROJECT_NAME}_payload_id_module_test
            ${PROJECT_NAME} ${catkin_LIBRARIES} ${pinocchio_LIBRARIES} gtest_main)
    target_compile_options(${PROJECT_NAME}_payload_id_module_test PUBLIC ${OCS2_CXX_FLAGS})

    catkin_add_gtest(${PROJECT_NAME}_residual_dynamics_module_test
            test/adaptive/modules/TestResidualDynamicsModule.cpp)
    target_link_libraries(${PROJECT_NAME}_residual_dynamics_module_test
            ${PROJECT_NAME} ${catkin_LIBRARIES} ${pinocchio_LIBRARIES} gtest_main)
    target_compile_options(${PROJECT_NAME}_residual_dynamics_module_test PUBLIC ${OCS2_CXX_FLAGS})

    catkin_add_gtest(${PROJECT_NAME}_adaptive_feature_module_test
            test/adaptive/modules/TestAdaptiveFeatureModule.cpp)
    target_link_libraries(${PROJECT_NAME}_adaptive_feature_module_test
            ${PROJECT_NAME} ${catkin_LIBRARIES} ${pinocchio_LIBRARIES} gtest_main)
    target_compile_options(${PROJECT_NAME}_adaptive_feature_module_test PUBLIC ${OCS2_CXX_FLAGS})

    catkin_add_gtest(${PROJECT_NAME}_residual_confidence_module_test
            test/adaptive/modules/TestResidualConfidenceModule.cpp)
    target_link_libraries(${PROJECT_NAME}_residual_confidence_module_test
            ${PROJECT_NAME} ${catkin_LIBRARIES} ${pinocchio_LIBRARIES} gtest_main)
    target_compile_options(${PROJECT_NAME}_residual_confidence_module_test PUBLIC ${OCS2_CXX_FLAGS})

    catkin_add_gtest(${PROJECT_NAME}_adaptive_module_shadow_pipeline_test
            test/adaptive/modules/TestAdaptiveModuleShadowPipeline.cpp)
    target_link_libraries(${PROJECT_NAME}_adaptive_module_shadow_pipeline_test
            ${PROJECT_NAME} ${catkin_LIBRARIES} ${pinocchio_LIBRARIES} gtest_main)
    target_compile_options(${PROJECT_NAME}_adaptive_module_shadow_pipeline_test PUBLIC ${OCS2_CXX_FLAGS})
endif ()

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
    message(STATUS "Run clang tooling for target " ${PROJECT_NAME})
    add_clang_tooling(
            TARGETS ${PROJECT_NAME}
            SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
            CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
            CF_WERROR
    )
endif (cmake_clang_tools_FOUND)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        )
