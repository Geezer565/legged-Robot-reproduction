//
// Created by qiayuan on 2022/6/24.
//

#pragma once

#include <limits>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <legged_common/hardware_interface/ContactSensorInterface.h>
#include <realtime_tools/realtime_publisher.h>
#include <std_msgs/Float64MultiArray.h>

#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ocs2_core/misc/Benchmark.h>
#include <ocs2_legged_robot_ros/visualization/LeggedRobotVisualizer.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>

#include <legged_estimation/StateEstimateBase.h>
#include <legged_interface/LeggedInterface.h>
#include <legged_interface/adaptive/MyRbfShadowPipeline.h>
#include <legged_interface/adaptive/MyRbfFyCompensation.h>
#include <legged_interface/adaptive/MyRbfTrackingError.h>
#include <legged_interface/adaptive/MyRbfWrenchCompensation.h>
#include <legged_wbc/WbcBase.h>

#include "legged_controllers/SafetyChecker.h"
#include "legged_controllers/visualization/LeggedSelfCollisionVisualization.h"

namespace legged {
using namespace ocs2;
using namespace legged_robot;

class LeggedController : public controller_interface::MultiInterfaceController<HybridJointInterface, hardware_interface::ImuSensorInterface,
                                                                               ContactSensorInterface> {
 public:
  LeggedController() = default;
  ~LeggedController() override;
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  void starting(const ros::Time& time) override;
  void stopping(const ros::Time& /*time*/) override { mpcRunning_ = false; }

 protected:
  virtual void updateStateEstimation(const ros::Time& time, const ros::Duration& period);

  virtual void setupLeggedInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                    bool verbose);
  virtual void setupMpc();
  virtual void setupMrt();
  virtual void setupStateEstimate(const std::string& taskFile, bool verbose);
  void updateMyRbfShadowObservation(const vector_t& optimizedState,
                                    const vector_t& optimizedInput);
  void queueMyRbfShadowPrediction();
  void applyMyRbfControlCompensation(vector_t& optimizedInput,
                                     std::size_t plannedMode, scalar_t dt);
  void applyMyRbfFyCompensation(vector_t& optimizedInput,
                                std::size_t plannedMode, scalar_t dt);
  void applyMyRbfWrenchCompensation(vector_t& optimizedInput,
                                    std::size_t plannedMode, scalar_t dt);
  void publishMyRbfShadowDiagnostics(
      const adaptive::MyRbfShadowPipelineResult& result,
      const vector_t& desiredRbdState);

  // Interface
  std::shared_ptr<LeggedInterface> leggedInterface_;
  std::shared_ptr<PinocchioEndEffectorKinematics> eeKinematicsPtr_;
  std::vector<HybridJointHandle> hybridJointHandles_;
  std::vector<ContactSensorHandle> contactHandles_;
  hardware_interface::ImuSensorHandle imuSensorHandle_;

  // State Estimation
  SystemObservation currentObservation_;
  vector_t measuredRbdState_;
  std::shared_ptr<StateEstimateBase> stateEstimate_;
  std::shared_ptr<CentroidalModelRbdConversions> rbdConversions_;

  // Whole Body Control
  std::shared_ptr<WbcBase> wbc_;
  std::shared_ptr<SafetyChecker> safetyChecker_;

  // Nonlinear MPC
  std::shared_ptr<MPC_BASE> mpc_;
  std::shared_ptr<MPC_MRT_Interface> mpcMrtInterface_;

  // Visualization
  std::shared_ptr<LeggedRobotVisualizer> robotVisualizer_;
  std::shared_ptr<LeggedSelfCollisionVisualization> selfCollisionVisualization_;
  ros::Publisher observationPublisher_;

  // Estimation and control injection are independently gated. Injection is
  // disabled by default; an explicitly selected adapter may apply either the
  // legacy Fy-only correction or the bounded six-axis wrench allocation.
  bool enableMyRbfShadow_ = false;
  bool enableMyRbfControlInjection_ = false;
  scalar_t myRbfShadowUpdatePeriod_ = 0.01;
  scalar_t myRbfShadowTimeTolerance_ = 0.003;
  scalar_t myRbfShadowLambdaLinear_ = 2.0;
  scalar_t myRbfShadowLambdaAngular_ = 1.5;
  scalar_t myRbfControlGain_ = 0.1;
  scalar_t myRbfControlForceLimit_ = 3.0;
  scalar_t myRbfControlRateLimit_ = 5.0;
  scalar_t myRbfAppliedFy_ = 0.0;
  std::string myRbfControlInterface_ = "legacy_fy";
  bool myRbfCompensationAwareObserver_ = false;
  adaptive::vector6_t myRbfControlGainVector_ =
      adaptive::vector6_t::Constant(0.1);
  adaptive::vector6_t myRbfControlMagnitudeLimit_ =
      (adaptive::vector6_t() << 3.0, 3.0, 6.0, 0.5, 0.5, 0.5).finished();
  adaptive::vector6_t myRbfControlRateLimitVector_ =
      (adaptive::vector6_t() << 5.0, 5.0, 10.0, 1.0, 1.0, 1.0).finished();
  scalar_t myRbfAllocatorFrictionCoefficient_ = 0.5;
  scalar_t myRbfAllocatorMinimumNormalForce_ = 2.0;
  scalar_t myRbfAllocatorMaximumForceDelta_ = 15.0;
  scalar_t myRbfAllocatorDamping_ = 1e-5;
  scalar_t myRbfAllocatorScale_ = 0.0;
  adaptive::vector6_t myRbfRequestedWrench_ = adaptive::vector6_t::Zero();
  adaptive::vector6_t myRbfAppliedWrench_ = adaptive::vector6_t::Zero();
  bool myRbfShadowPredictionPending_ = false;
  scalar_t myRbfShadowPredictionTime_ =
      std::numeric_limits<scalar_t>::quiet_NaN();
  std::size_t myRbfShadowValidSampleCount_ = 0;
  std::size_t myRbfShadowRejectedSampleCount_ = 0;
  std::size_t myRbfShadowRolloutFailureCount_ = 0;
  std::unique_ptr<adaptive::MyRbfShadowPipeline> myRbfShadowPipeline_;
  std::unique_ptr<PinocchioInterface> myRbfPinocchioInterface_;
  std::unique_ptr<CentroidalModelPinocchioMapping> myRbfPinocchioMapping_;
  std::unique_ptr<PinocchioEndEffectorKinematics> myRbfEeKinematics_;
  std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::Float64MultiArray>>
      myRbfShadowPublisher_;

 private:
  std::thread mpcThread_;
  std::atomic_bool controllerRunning_{}, mpcRunning_{};
  benchmark::RepeatedTimer mpcTimer_;
  benchmark::RepeatedTimer wbcTimer_;
};

class LeggedCheaterController : public LeggedController {
 protected:
  void setupStateEstimate(const std::string& taskFile, bool verbose) override;
};

}  // namespace legged
