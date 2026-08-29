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
        src/adaptive/MyRbfShadowEstimator.cpp
        src/adaptive/MyRbfShadowPipeline.cpp
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
    catkin_add_gtest(${PROJECT_NAME}_myrbf_shadow_estimator_test
            test/adaptive/TestMyRbfShadowEstimator.cpp)
    target_link_libraries(${PROJECT_NAME}_myrbf_shadow_estimator_test
            ${PROJECT_NAME} ${catkin_LIBRARIES} ${pinocchio_LIBRARIES} gtest_main)
    target_compile_options(${PROJECT_NAME}_myrbf_shadow_estimator_test PUBLIC ${OCS2_CXX_FLAGS})

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
