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
#include <legged_interface/adaptive/MyRbfV2Pipeline.h>
#include <legged_interface/adaptive/MyRbfV5DynamicsCore.h>
#include <legged_interface/adaptive/MyRbfFyCompensation.h>
#include <legged_interface/adaptive/MyRbfTrackingError.h>
#include <legged_interface/adaptive/MyRbfWrenchCompensation.h>
#include <legged_wbc/WbcBase.h>
#include <legged_wbc/MyRbfV2Wbc.h>
#include "legged_controllers/MyRbfV2PolicyBinding.h"

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
  void stopping(const ros::Time& /*time*/) override {
    mpcRunning_ = false;
    if(myRbfV2Lyapunov_)for(auto& joint:hybridJointHandles_)joint.setCommand(0,0,0,0,0);
  }

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
  void publishMyRbfV2Diagnostics(
      const adaptive::MyRbfV2PipelineResult& result,
      const vector_t& desiredRbdState);
  bool hasMyRbfPipeline() const {
    return static_cast<bool>(myRbfShadowPipeline_) ||
           static_cast<bool>(myRbfV2Pipeline_) ||
           static_cast<bool>(myRbfV5Core_);
  }

  void configureMyRbfV5(ros::NodeHandle& nh);
  void configureMyRbfV11(ros::NodeHandle& nh);
  void updateMyRbfV11(const ros::Time& time,double dt,const vector_t& stateDesired,
                      const vector_t& inputDesired,std::size_t mode);
  bool myRbfV11_=false, myRbfV11PredictionValid_=false;
  std::unique_ptr<adaptive::v11::Core> myRbfV11Core_;
  adaptive::v11::Vector6 myRbfV11PreviousVelocity_=adaptive::v11::Vector6::Zero();
  adaptive::v11::Vector6 myRbfV11PreviousRate_=adaptive::v11::Vector6::Zero();
  adaptive::v11::Vector6 myRbfV11PreviousNominalRate_=adaptive::v11::Vector6::Zero();
  adaptive::v11::Matrix6 myRbfV11PreviousG_=adaptive::v11::Matrix6::Identity();
  double myRbfV11PreviousTime_=0;
  std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::Float64MultiArray>> myRbfV11Publisher_;
  void observeMyRbfV5();
  void queueMyRbfV5Prediction();
  adaptive::vector6_t myRbfV5EquivalentWrench(
      const adaptive::v5::Axes& acceleration) const;
  adaptive::v5::Axes myRbfV5EquivalentAcceleration(
      const adaptive::vector6_t& wrench) const;
  void publishMyRbfV5Diagnostics(
      const adaptive::v5::UpdateResult& result,
      const adaptive::vector6_t& externalWrenchEstimate);

  void configureMyRbfV2Lyapunov(ros::NodeHandle& nh);
  void updateMyRbfV2Control(const ros::Time& time, double dt,
      const vector_t& stateDesired, const vector_t& inputDesired, std::size_t mode);
  bool updateMyRbfV2Warmup(const ros::Time& time, double dt,
      const vector_t& stateDesired, const vector_t& inputDesired, std::size_t mode);
  void publishMyRbfV2Lyapunov(const MyRbfV2WbcDiagnostics& diagnostic,
      const adaptive::MyRbfV2Snapshot& active, double holdMismatch);
  bool myRbfV2Lyapunov_ = false;
  bool myRbfV3Teacher_ = false;
  bool myRbfV4Convergence_ = false;
  bool myRbfV5Dynamics_ = false;
  bool myRbfV6Dynamics_ = false;
  bool myRbfV3SharedAllocator_ = false;
  double myRbfV3CompositeRatio_ = 0.75;
  double myRbfV3UpdateDt_ = 0.0;
  bool myRbfV3TeacherErrorValid_ = false;
  adaptive::vector6_t myRbfV3PredictionError_ = adaptive::vector6_t::Zero();
  adaptive::vector6_t myRbfV3TrackingError_ = adaptive::vector6_t::Zero();
  adaptive::vector6_t myRbfV3CompositeError_ = adaptive::vector6_t::Zero();
  vector_t myRbfV3PredictionStateError_;
  vector_t myRbfV3TrackingStateError_;
  vector_t myRbfV3CompositeStateError_;
  std::string myRbfV2FeatureMode_ = "contact_aware", myRbfV2RobustMode_ = "sign";
  std::shared_ptr<adaptive::MyRbfV2SnapshotStore> myRbfV2Snapshots_;
  std::shared_ptr<MyRbfV2PolicyBinding> myRbfV2PolicyBinding_;
  std::mutex myRbfV2MpcLifecycleMutex_;
  std::atomic_bool myRbfV2SolverFault_{false};
  MyRbfV2WbcInput myRbfV2WbcInput_;
  std::uint64_t myRbfV2Sequence_ = 0;
  std::uint64_t myRbfV2DiagnosticSequence_ = 0, myRbfV2DiagnosticDrops_ = 0;
  bool myRbfV2MomentumValid_ = false, myRbfV2Armed_ = false;
  double myRbfV2PreviousTime_ = 0.0, myRbfV2ActivationStartTime_ = 0.0;
  double myRbfV2ActivationWarmup_ = 1.0;
  adaptive::vector6_t myRbfV2PreviousMomentum_ = adaptive::vector6_t::Zero();
  adaptive::vector6_t myRbfV2PreviousNominalRate_ = adaptive::vector6_t::Zero();
  adaptive::vector6_t myRbfV2ExternalResidual_ = adaptive::vector6_t::Zero();
  adaptive::vector6_t myRbfV2AppliedCorrection_ = adaptive::vector6_t::Zero();
  Eigen::Vector3d myRbfV2PreviousComOffset_ = Eigen::Vector3d::Zero();
  std::unique_ptr<PinocchioInterface> myRbfV2ResidualModel_;
  std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::Float64MultiArray>> myRbfV2Publisher_;
  std::unique_ptr<adaptive::v5::DynamicsCore> myRbfV5Core_;
  bool myRbfV5RbfEnabled_ = true;
  bool myRbfV5PredictionValid_ = false;
  adaptive::v5::Axes myRbfV5BaseVelocity_{};
  adaptive::v5::Axes myRbfV5NominalRate_{};
  adaptive::v5::Axes myRbfV5KnownAppliedAcceleration_{};
  adaptive::v5::Axes myRbfV5ObservedRate_{};
  adaptive::v5::Axes myRbfV5Correction_{};
  adaptive::v5::Axes myRbfV5RawTargetResidual_{};
  adaptive::v5::Axes myRbfV5ResidualMemory_{};
  adaptive::v5::Axes myRbfV5ResidualTrend_{};
  adaptive::v5::Features myRbfV5QueuedFeatures_{};
  bool myRbfV5ResidualMemoryValid_ = false;
  double myRbfV5ResidualFilterTau_ = 0.08;
  double myRbfV5PredictionDt_ = 0.0;
  std::uint64_t myRbfV5DiagnosticSequence_ = 0;
  std::uint64_t myRbfV5DiagnosticDrops_ = 0;
  std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::Float64MultiArray>>
      myRbfV5Publisher_;

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
  std::string myRbfAlgorithm_ = "v1";
  std::string myRbfEstimatorVersion_ = "legacy";
  std::string myRbfComparisonMethod_;
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
  adaptive::vector6_t myRbfAdaptiveWrench_ = adaptive::vector6_t::Zero();
  bool myRbfShadowPredictionPending_ = false;
  scalar_t myRbfShadowPredictionTime_ =
      std::numeric_limits<scalar_t>::quiet_NaN();
  std::size_t myRbfShadowValidSampleCount_ = 0;
  std::size_t myRbfShadowRejectedSampleCount_ = 0;
  std::size_t myRbfShadowRolloutFailureCount_ = 0;
  std::unique_ptr<adaptive::MyRbfShadowPipeline> myRbfShadowPipeline_;
  std::unique_ptr<adaptive::MyRbfV2Pipeline> myRbfV2Pipeline_;
  adaptive::vector6_t myRbfResidualScale_ = adaptive::vector6_t::Ones();
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
