//
// Created by qiayuan on 2022/6/24.
//

#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include "legged_controllers/LeggedController.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_core/thread_support/ExecuteAndSleep.h>
#include <ocs2_core/thread_support/SetThreadPriority.h>
#include <ocs2_legged_robot_ros/gait/GaitReceiver.h>
#include <ocs2_msgs/mpc_observation.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>
#include <ocs2_sqp/SqpMpc.h>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <angles/angles.h>
#include <cmath>
#include <stdexcept>
#include <legged_estimation/FromTopiceEstimate.h>
#include <legged_estimation/LinearKalmanFilter.h>
#include <legged_wbc/HierarchicalWbc.h>
#include <legged_wbc/WeightedWbc.h>
#include <pluginlib/class_list_macros.hpp>

namespace legged {
namespace {

adaptive::vector6_t loadMyRbfVector6(
    ros::NodeHandle& nodeHandle, const std::string& key,
    const adaptive::vector6_t& defaultValue) {
  std::vector<double> values;
  if (!nodeHandle.getParam(key, values)) {
    return defaultValue;
  }
  if (values.size() != 6) {
    throw std::invalid_argument(
        "[MyRBF shadow] " + key + " must contain exactly six values.");
  }
  adaptive::vector6_t result;
  for (Eigen::Index index = 0; index < 6; ++index) {
    result(index) = values[static_cast<std::size_t>(index)];
  }
  if (!result.allFinite()) {
    throw std::invalid_argument(
        "[MyRBF shadow] " + key + " must contain finite values.");
  }
  return result;
}

}  // namespace

bool LeggedController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh) {
  // Initialize OCS2
  std::string urdfFile;
  std::string taskFile;
  std::string referenceFile;
  controller_nh.getParam("/urdfFile", urdfFile);
  controller_nh.getParam("/taskFile", taskFile);
  controller_nh.getParam("/referenceFile", referenceFile);
  controller_nh.param("enable_myrbf_shadow", enableMyRbfShadow_, false);
  controller_nh.param(
      "enable_myrbf_control_injection", enableMyRbfControlInjection_, false);
  controller_nh.param(
      "myrbf_shadow/update_period", myRbfShadowUpdatePeriod_, 0.01);
  controller_nh.param(
      "myrbf_shadow/time_tolerance", myRbfShadowTimeTolerance_, 0.003);
  bool verbose = false;
  loadData::loadCppDataType(taskFile, "legged_robot_interface.verbose", verbose);

  setupLeggedInterface(taskFile, urdfFile, referenceFile, verbose);
  setupMpc();
  setupMrt();

  if (enableMyRbfShadow_) {
    scalar_t filterAlpha = 0.5;
    scalar_t linearExtent = 2.0;
    scalar_t angularExtent = 4.0;
    scalar_t widthRatio = 0.85;
    bool normalizeUpdate = false;
    bool trackingBiasOnly = false;
    scalar_t updateEpsilon = 1e-9;
    scalar_t minimumCoverage = 0.0;
    controller_nh.param("myrbf_shadow/filter_alpha", filterAlpha, 0.5);
    controller_nh.param("myrbf_shadow/linear_extent", linearExtent, 2.0);
    controller_nh.param("myrbf_shadow/angular_extent", angularExtent, 4.0);
    controller_nh.param("myrbf_shadow/width_ratio", widthRatio, 0.85);
    controller_nh.param("myrbf_shadow/normalize_update", normalizeUpdate,
                        false);
    controller_nh.param("myrbf_shadow/tracking_bias_only", trackingBiasOnly,
                        false);
    controller_nh.param("myrbf_shadow/update_epsilon", updateEpsilon, 1e-9);
    controller_nh.param("myrbf_shadow/minimum_coverage", minimumCoverage, 0.0);
    controller_nh.param(
        "myrbf_shadow/lambda_linear", myRbfShadowLambdaLinear_, 2.0);
    controller_nh.param(
        "myrbf_shadow/lambda_angular", myRbfShadowLambdaAngular_, 1.5);
    controller_nh.param("myrbf_shadow/control_gain", myRbfControlGain_, 0.1);
    controller_nh.param(
        "myrbf_shadow/control_force_limit", myRbfControlForceLimit_, 3.0);
    controller_nh.param(
        "myrbf_shadow/control_rate_limit", myRbfControlRateLimit_, 5.0);
    controller_nh.param("myrbf_shadow/control_interface",
                        myRbfControlInterface_, std::string("legacy_fy"));
    controller_nh.param("myrbf_shadow/compensation_aware_observer",
                        myRbfCompensationAwareObserver_, false);
    controller_nh.param("myrbf_shadow/allocator_friction_coefficient",
                        myRbfAllocatorFrictionCoefficient_, 0.5);
    controller_nh.param("myrbf_shadow/allocator_minimum_normal_force",
                        myRbfAllocatorMinimumNormalForce_, 2.0);
    controller_nh.param("myrbf_shadow/allocator_maximum_force_delta",
                        myRbfAllocatorMaximumForceDelta_, 15.0);
    controller_nh.param("myrbf_shadow/allocator_damping",
                        myRbfAllocatorDamping_, 1e-5);

    const adaptive::vector6_t defaultScale =
        (adaptive::vector6_t() <<
             18.40147689318011, 18.757520233566048,
             17.534482646627026, 2.3370077074651903,
             5.4660415889306133, 1.7261893548458138)
            .finished();
    const adaptive::vector6_t defaultPredictionGain =
        (adaptive::vector6_t() <<
             0.00100409, 0.00096634, 0.00110584,
             0.0622523, 0.004, 0.114105)
            .finished();
    const auto residualScale = loadMyRbfVector6(
        controller_nh, "myrbf_shadow/residual_scale", defaultScale);
    const auto discrepancyScale = loadMyRbfVector6(
        controller_nh, "myrbf_shadow/discrepancy_scale", defaultScale);
    const auto outputScale = loadMyRbfVector6(
        controller_nh, "myrbf_shadow/output_scale", defaultScale);
    const auto predictionGain = loadMyRbfVector6(
        controller_nh, "myrbf_shadow/prediction_gain", defaultPredictionGain);
    const auto trackingGain = loadMyRbfVector6(
        controller_nh, "myrbf_shadow/tracking_gain",
        adaptive::vector6_t::Zero());
    const auto leakage = loadMyRbfVector6(
        controller_nh, "myrbf_shadow/leakage",
        adaptive::vector6_t::Constant(0.02));
    const auto weightLimit = loadMyRbfVector6(
        controller_nh, "myrbf_shadow/weight_limit",
        adaptive::vector6_t::Constant(
            std::numeric_limits<scalar_t>::infinity()));
    myRbfControlGainVector_ = loadMyRbfVector6(
        controller_nh, "myrbf_shadow/control_gain_vector",
        adaptive::vector6_t::Constant(0.1));
    myRbfControlMagnitudeLimit_ = loadMyRbfVector6(
        controller_nh, "myrbf_shadow/control_magnitude_limit",
        (adaptive::vector6_t() << 3.0, 3.0, 6.0, 0.5, 0.5, 0.5)
            .finished());
    myRbfControlRateLimitVector_ = loadMyRbfVector6(
        controller_nh, "myrbf_shadow/control_rate_limit_vector",
        (adaptive::vector6_t() << 5.0, 5.0, 10.0, 1.0, 1.0, 1.0)
            .finished());

    if (!std::isfinite(myRbfShadowUpdatePeriod_) ||
        myRbfShadowUpdatePeriod_ <= 0.0 ||
        !std::isfinite(myRbfShadowTimeTolerance_) ||
        myRbfShadowTimeTolerance_ < 0.0) {
      ROS_ERROR("[MyRBF shadow] update period or time tolerance is invalid.");
      return false;
    }
    if (!std::isfinite(myRbfShadowLambdaLinear_) ||
        myRbfShadowLambdaLinear_ < 0.0 ||
        !std::isfinite(myRbfShadowLambdaAngular_) ||
        myRbfShadowLambdaAngular_ < 0.0) {
      ROS_ERROR("[MyRBF shadow] tracking-error gains are invalid.");
      return false;
    }
    if (!std::isfinite(myRbfControlGain_) || myRbfControlGain_ < 0.0 ||
        myRbfControlGain_ > 1.0 ||
        !std::isfinite(myRbfControlForceLimit_) ||
        myRbfControlForceLimit_ <= 0.0 ||
        !std::isfinite(myRbfControlRateLimit_) ||
        myRbfControlRateLimit_ <= 0.0) {
      ROS_ERROR("[MyRBF control] gain or safety limit is invalid.");
      return false;
    }
    if (myRbfControlInterface_ != "legacy_fy" &&
        myRbfControlInterface_ != "wrench_allocator") {
      ROS_ERROR_STREAM("[MyRBF control] Unknown control_interface: "
                       << myRbfControlInterface_);
      return false;
    }
    if ((myRbfControlGainVector_.array() < 0.0).any() ||
        (myRbfControlMagnitudeLimit_.array() <= 0.0).any() ||
        (myRbfControlRateLimitVector_.array() <= 0.0).any() ||
        !std::isfinite(myRbfAllocatorFrictionCoefficient_) ||
        myRbfAllocatorFrictionCoefficient_ <= 0.0 ||
        !std::isfinite(myRbfAllocatorMinimumNormalForce_) ||
        myRbfAllocatorMinimumNormalForce_ < 0.0 ||
        !std::isfinite(myRbfAllocatorMaximumForceDelta_) ||
        myRbfAllocatorMaximumForceDelta_ <= 0.0 ||
        !std::isfinite(myRbfAllocatorDamping_) ||
        myRbfAllocatorDamping_ <= 0.0) {
      ROS_ERROR("[MyRBF control] Six-dimensional safety limits are invalid.");
      return false;
    }

    try {
      auto settings = adaptive::createSymmetricGridMyRbfShadowSettings(
          linearExtent, angularExtent, widthRatio,
          residualScale, discrepancyScale, outputScale,
          trackingGain, predictionGain, leakage);
      settings.normalizeUpdate = normalizeUpdate;
      settings.trackingBiasOnly = trackingBiasOnly;
      settings.updateEpsilon = updateEpsilon;
      settings.minimumCoverage = minimumCoverage;
      settings.weightLimit = weightLimit;
      const auto& modelInfo = leggedInterface_->getCentroidalModelInfo();
      myRbfShadowPipeline_ = std::make_unique<adaptive::MyRbfShadowPipeline>(
          modelInfo.robotMass, modelInfo.stateDim, true,
          filterAlpha, myRbfShadowTimeTolerance_, std::move(settings));
      if (myRbfControlInterface_ == "wrench_allocator") {
        myRbfPinocchioInterface_ = std::make_unique<PinocchioInterface>(
            leggedInterface_->getPinocchioInterface());
        myRbfPinocchioMapping_ =
            std::make_unique<CentroidalModelPinocchioMapping>(modelInfo);
        myRbfEeKinematics_ =
            std::make_unique<PinocchioEndEffectorKinematics>(
                *myRbfPinocchioInterface_, *myRbfPinocchioMapping_,
                leggedInterface_->modelSettings().contactNames3DoF);
      }
    } catch (const std::exception& error) {
      ROS_ERROR_STREAM("[MyRBF shadow] Invalid configuration: " << error.what());
      return false;
    }

    ros::NodeHandle shadowNodeHandle;
    myRbfShadowPublisher_ = std::make_unique<
        realtime_tools::RealtimePublisher<std_msgs::Float64MultiArray>>(
        shadowNodeHandle, "/myrbf_shadow_diagnostics", 100);
    myRbfShadowPublisher_->msg_.data.resize(111, 0.0);
    if (enableMyRbfControlInjection_) {
      ROS_WARN_STREAM("[MyRBF control] Injection enabled through "
                      << myRbfControlInterface_
                      << "; compensation-aware observer="
                      << (myRbfCompensationAwareObserver_ ? "true" : "false"));
    } else {
      ROS_WARN_STREAM(
          "[MyRBF shadow] Diagnostics only: estimate is not connected to control. update_period="
          << myRbfShadowUpdatePeriod_);
    }
  } else if (enableMyRbfControlInjection_) {
    ROS_ERROR("[MyRBF control] Control injection requires enable_myrbf_shadow=true.");
    return false;
  }
  // Visualization
  ros::NodeHandle nh;
  CentroidalModelPinocchioMapping pinocchioMapping(leggedInterface_->getCentroidalModelInfo());
  eeKinematicsPtr_ = std::make_shared<PinocchioEndEffectorKinematics>(leggedInterface_->getPinocchioInterface(), pinocchioMapping,
                                                                      leggedInterface_->modelSettings().contactNames3DoF);
  robotVisualizer_ = std::make_shared<LeggedRobotVisualizer>(leggedInterface_->getPinocchioInterface(),
                                                             leggedInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_, nh);
  selfCollisionVisualization_.reset(new LeggedSelfCollisionVisualization(leggedInterface_->getPinocchioInterface(),
                                                                         leggedInterface_->getGeometryInterface(), pinocchioMapping, nh));

  // Hardware interface
  auto* hybridJointInterface = robot_hw->get<HybridJointInterface>();
  std::vector<std::string> joint_names{"LF_HAA", "LF_HFE", "LF_KFE", "LH_HAA", "LH_HFE", "LH_KFE",
                                       "RF_HAA", "RF_HFE", "RF_KFE", "RH_HAA", "RH_HFE", "RH_KFE"};
  for (const auto& joint_name : joint_names) {
    hybridJointHandles_.push_back(hybridJointInterface->getHandle(joint_name));
  }
  auto* contactInterface = robot_hw->get<ContactSensorInterface>();
  for (const auto& name : leggedInterface_->modelSettings().contactNames3DoF) {
    contactHandles_.push_back(contactInterface->getHandle(name));
  }
  imuSensorHandle_ = robot_hw->get<hardware_interface::ImuSensorInterface>()->getHandle("unitree_imu");

  // State estimation
  setupStateEstimate(taskFile, verbose);

  // Whole body control
  wbc_ = std::make_shared<WeightedWbc>(leggedInterface_->getPinocchioInterface(), leggedInterface_->getCentroidalModelInfo(),
                                       *eeKinematicsPtr_);
  wbc_->loadTasksSetting(taskFile, verbose);

  // Safety Checker
  safetyChecker_ = std::make_shared<SafetyChecker>(leggedInterface_->getCentroidalModelInfo());

  return true;
}

void LeggedController::starting(const ros::Time& time) {
  myRbfShadowPredictionPending_ = false;
  myRbfShadowPredictionTime_ = std::numeric_limits<scalar_t>::quiet_NaN();
  myRbfShadowValidSampleCount_ = 0;
  myRbfShadowRejectedSampleCount_ = 0;
  myRbfShadowRolloutFailureCount_ = 0;
  myRbfAppliedFy_ = 0.0;
  myRbfRequestedWrench_.setZero();
  myRbfAppliedWrench_.setZero();
  myRbfAllocatorScale_ = 0.0;
  if (myRbfShadowPipeline_) {
    myRbfShadowPipeline_->reset();
  }
  // Initial state
  currentObservation_.state.setZero(leggedInterface_->getCentroidalModelInfo().stateDim);
  updateStateEstimation(time, ros::Duration(0.002));
  currentObservation_.input.setZero(leggedInterface_->getCentroidalModelInfo().inputDim);
  currentObservation_.mode = ModeNumber::STANCE;

  TargetTrajectories target_trajectories({currentObservation_.time}, {currentObservation_.state}, {currentObservation_.input});

  // Set the first observation and command and wait for optimization to finish
  mpcMrtInterface_->setCurrentObservation(currentObservation_);
  mpcMrtInterface_->getReferenceManager().setTargetTrajectories(target_trajectories);
  ROS_INFO_STREAM("Waiting for the initial policy ...");
  while (!mpcMrtInterface_->initialPolicyReceived() && ros::ok()) {
    mpcMrtInterface_->advanceMpc();
    ros::WallRate(leggedInterface_->mpcSettings().mrtDesiredFrequency_).sleep();
  }
  ROS_INFO_STREAM("Initial policy has been received.");

  mpcRunning_ = true;
}

void LeggedController::update(const ros::Time& time, const ros::Duration& period) {
  // State Estimate
  updateStateEstimation(time, period);

  // Update the current state of the system
  mpcMrtInterface_->setCurrentObservation(currentObservation_);

  // Load the latest MPC policy
  mpcMrtInterface_->updatePolicy();

  // Evaluate the current policy
  vector_t optimizedState, optimizedInput;
  size_t plannedMode = 0;  // The mode that is active at the time the policy is evaluated at.
  mpcMrtInterface_->evaluatePolicy(currentObservation_.time, currentObservation_.state, optimizedState, optimizedInput, plannedMode);
  updateMyRbfShadowObservation(optimizedState, optimizedInput);
  applyMyRbfControlCompensation(optimizedInput, plannedMode, period.toSec());
  queueMyRbfShadowPrediction();

  // Whole body control
  currentObservation_.input = optimizedInput;

  wbcTimer_.startTimer();
  vector_t x = wbc_->update(optimizedState, optimizedInput, measuredRbdState_, plannedMode, period.toSec());
  wbcTimer_.endTimer();

  vector_t torque = x.tail(12);

  vector_t posDes = centroidal_model::getJointAngles(optimizedState, leggedInterface_->getCentroidalModelInfo());
  vector_t velDes = centroidal_model::getJointVelocities(optimizedInput, leggedInterface_->getCentroidalModelInfo());

  // Safety check, if failed, stop the controller
  if (!safetyChecker_->check(currentObservation_, optimizedState, optimizedInput)) {
    ROS_ERROR_STREAM("[Legged Controller] Safety check failed, stopping the controller.");
    stopRequest(time);
  }
  float joint_kd_list[12]={4,1,10,4,1,10,4,1,10,4,1,10};
  float joint_kp_list[12]={0,0,0,0,0,0,0,0,0,0,0,0};

  for (size_t j = 0; j < leggedInterface_->getCentroidalModelInfo().actuatedDofNum; ++j) {
    hybridJointHandles_[j].setCommand(posDes(j), velDes(j), 0,3, torque(j));
    
  }

  // Visualization
  robotVisualizer_->update(currentObservation_, mpcMrtInterface_->getPolicy(), mpcMrtInterface_->getCommand());
  selfCollisionVisualization_->update(currentObservation_);

  // Publish the observation. Only needed for the command interface
  observationPublisher_.publish(ros_msg_conversions::createObservationMsg(currentObservation_));
}

void LeggedController::updateMyRbfShadowObservation(
    const vector_t& optimizedState, const vector_t& optimizedInput) {
  if (!enableMyRbfShadow_ || !myRbfShadowPipeline_ ||
      !myRbfShadowPredictionPending_) {
    return;
  }
  if (currentObservation_.time <
      myRbfShadowPredictionTime_ - myRbfShadowTimeTolerance_) {
    return;
  }

  myRbfShadowPredictionPending_ = false;
  adaptive::MyRbfShadowPipelineResult result;
  vector_t desiredRbdState;
  try {
    desiredRbdState =
        rbdConversions_->computeRbdStateFromCentroidalModel(
            optimizedState, optimizedInput);
    const auto& modelInfo = leggedInterface_->getCentroidalModelInfo();
    const adaptive::vector6_t trackingError =
        adaptive::computeMyRbfTrackingError(
            measuredRbdState_, desiredRbdState,
            modelInfo.generalizedCoordinatesNum,
            myRbfShadowLambdaLinear_, myRbfShadowLambdaAngular_);
    result = myRbfShadowPipeline_->observeActual(
        currentObservation_.time, currentObservation_.state, trackingError);
  } catch (const std::exception& error) {
    ++myRbfShadowRejectedSampleCount_;
    ROS_WARN_STREAM_THROTTLE(
        1.0, "[MyRBF shadow] Tracking observation failed: " << error.what());
    return;
  }
  if (!result.valid) {
    ++myRbfShadowRejectedSampleCount_;
    return;
  }
  ++myRbfShadowValidSampleCount_;
  publishMyRbfShadowDiagnostics(result, desiredRbdState);
}

void LeggedController::queueMyRbfShadowPrediction() {
  if (!enableMyRbfShadow_ || !myRbfShadowPipeline_ ||
      myRbfShadowPredictionPending_) {
    return;
  }

  vector_t predictedState;
  vector_t predictedInput;
  std::size_t predictedMode = 0;
  try {
    mpcMrtInterface_->rolloutPolicy(
        currentObservation_.time, currentObservation_.state,
        myRbfShadowUpdatePeriod_, predictedState, predictedInput, predictedMode);
    myRbfShadowPredictionTime_ =
        currentObservation_.time + myRbfShadowUpdatePeriod_;
    const adaptive::vector6_t knownAppliedCompensation =
        myRbfCompensationAwareObserver_
            ? myRbfAppliedWrench_
            : adaptive::vector6_t::Zero();
    myRbfShadowPipeline_->setPrediction(
        myRbfShadowPredictionTime_, myRbfShadowUpdatePeriod_, predictedState,
        knownAppliedCompensation);
    myRbfShadowPredictionPending_ = true;
  } catch (const std::exception& error) {
    ++myRbfShadowRolloutFailureCount_;
    ROS_WARN_STREAM_THROTTLE(
        1.0, "[MyRBF shadow] One-step prediction failed: " << error.what());
  }
}

void LeggedController::applyMyRbfControlCompensation(
    vector_t& optimizedInput, std::size_t plannedMode, scalar_t dt) {
  if (!enableMyRbfControlInjection_ || !myRbfShadowPipeline_) {
    myRbfAppliedFy_ = 0.0;
    myRbfRequestedWrench_.setZero();
    myRbfAppliedWrench_.setZero();
    myRbfAllocatorScale_ = 0.0;
    return;
  }
  if (myRbfControlInterface_ == "wrench_allocator") {
    applyMyRbfWrenchCompensation(optimizedInput, plannedMode, dt);
  } else {
    applyMyRbfFyCompensation(optimizedInput, plannedMode, dt);
    myRbfRequestedWrench_.setZero();
    myRbfRequestedWrench_(1) = myRbfAppliedFy_;
    myRbfAppliedWrench_ = myRbfRequestedWrench_;
    myRbfAllocatorScale_ = 1.0;
  }
}

void LeggedController::applyMyRbfFyCompensation(
    vector_t& optimizedInput, std::size_t plannedMode, scalar_t dt) {
  if (!enableMyRbfControlInjection_ || !myRbfShadowPipeline_) {
    myRbfAppliedFy_ = 0.0;
    return;
  }

  const contact_flag_t contactFlags = modeNumber2StanceLeg(plannedMode);
  std::size_t stanceCount = 0;
  for (bool inContact : contactFlags) {
    stanceCount += inContact ? 1U : 0U;
  }
  const scalar_t estimate = stanceCount > 0
                                ? myRbfShadowPipeline_->estimate()(1)
                                : 0.0;
  myRbfAppliedFy_ = adaptive::computeLimitedFyCompensation(
      myRbfAppliedFy_, estimate, myRbfControlGain_,
      myRbfControlForceLimit_, myRbfControlRateLimit_, dt);
  if (stanceCount == 0) {
    return;
  }

  const scalar_t perFootFy =
      myRbfAppliedFy_ / static_cast<scalar_t>(stanceCount);
  for (std::size_t contact = 0; contact < contactFlags.size(); ++contact) {
    if (contactFlags[contact]) {
      optimizedInput(3 * contact + 1) += perFootFy;
    }
  }
}

void LeggedController::applyMyRbfWrenchCompensation(
    vector_t& optimizedInput, std::size_t plannedMode, scalar_t dt) {
  if (!myRbfPinocchioInterface_ || !myRbfPinocchioMapping_ ||
      !myRbfEeKinematics_) {
    myRbfAppliedWrench_.setZero();
    myRbfAppliedFy_ = 0.0;
    myRbfAllocatorScale_ = 0.0;
    return;
  }

  const contact_flag_t contactFlags = modeNumber2StanceLeg(plannedMode);
  if (!myRbfShadowPredictionPending_) {
    myRbfRequestedWrench_ = adaptive::computeLimitedWrenchCompensation(
        myRbfRequestedWrench_, myRbfShadowPipeline_->estimate(),
        myRbfControlGainVector_, myRbfControlMagnitudeLimit_,
        myRbfControlRateLimitVector_,
        std::max(dt, myRbfShadowUpdatePeriod_));
  }

  adaptive::WrenchAllocationInput allocationInput;
  allocationInput.requestedWrench = myRbfRequestedWrench_;
  allocationInput.frictionCoefficient = myRbfAllocatorFrictionCoefficient_;
  allocationInput.minimumNormalForce = myRbfAllocatorMinimumNormalForce_;
  allocationInput.maximumForceDelta = myRbfAllocatorMaximumForceDelta_;
  allocationInput.damping = myRbfAllocatorDamping_;

  try {
    myRbfPinocchioMapping_->setPinocchioInterface(
        *myRbfPinocchioInterface_);
    const vector_t q = myRbfPinocchioMapping_->getPinocchioJointPosition(
        currentObservation_.state);
    const auto& model = myRbfPinocchioInterface_->getModel();
    auto& data = myRbfPinocchioInterface_->getData();
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacements(model, data);
    const vector3_t centerOfMass = pinocchio::centerOfMass(model, data, q);
    myRbfEeKinematics_->setPinocchioInterface(*myRbfPinocchioInterface_);
    const auto contactPositions =
        myRbfEeKinematics_->getPosition(vector_t());
    for (std::size_t contact = 0;
         contact < adaptive::kMyRbfContactCount; ++contact) {
      allocationInput.contacts[contact] = contactFlags[contact];
      allocationInput.contactPositions[contact] =
          contactPositions[contact] - centerOfMass;
      allocationInput.nominalForces[contact] =
          optimizedInput.segment<3>(3 * contact);
    }

    const auto allocation =
        adaptive::allocateBodyWrenchToContacts(allocationInput);
    if (!allocation.valid) {
      myRbfAppliedWrench_.setZero();
      myRbfAppliedFy_ = 0.0;
      myRbfAllocatorScale_ = 0.0;
      return;
    }
    for (std::size_t contact = 0;
         contact < adaptive::kMyRbfContactCount; ++contact) {
      optimizedInput.segment<3>(3 * contact) +=
          allocation.forceDeltas[contact];
    }
    myRbfAppliedWrench_ = allocation.achievedWrench;
    myRbfAppliedFy_ = myRbfAppliedWrench_(1);
    myRbfAllocatorScale_ = allocation.scale;
  } catch (const std::exception& error) {
    myRbfAppliedWrench_.setZero();
    myRbfAppliedFy_ = 0.0;
    myRbfAllocatorScale_ = 0.0;
    ROS_WARN_STREAM_THROTTLE(
        1.0, "[MyRBF control] Wrench allocation failed: " << error.what());
  }
}

void LeggedController::publishMyRbfShadowDiagnostics(
    const adaptive::MyRbfShadowPipelineResult& result,
    const vector_t& desiredRbdState) {
  if (!myRbfShadowPublisher_ || !myRbfShadowPublisher_->trylock()) {
    return;
  }

  auto& values = myRbfShadowPublisher_->msg_.data;
  std::size_t index = 0;
  values[index++] = currentObservation_.time;
  values[index++] = result.residual.dt;
  for (Eigen::Index axis = 0; axis < 6; ++axis) {
    values[index++] = result.residual.rawResidual(axis);
  }
  for (Eigen::Index axis = 0; axis < 6; ++axis) {
    values[index++] = result.residual.filteredResidual(axis);
  }
  for (Eigen::Index axis = 0; axis < 6; ++axis) {
    values[index++] = result.estimator.estimateBeforeUpdate(axis);
  }
  for (Eigen::Index axis = 0; axis < 6; ++axis) {
    values[index++] = result.estimator.estimateAfterUpdate(axis);
  }
  for (Eigen::Index axis = 0; axis < 6; ++axis) {
    values[index++] = result.estimator.feature(0, axis);
  }
  for (Eigen::Index axis = 0; axis < 6; ++axis) {
    values[index++] = result.estimator.feature(1, axis);
  }
  for (Eigen::Index axis = 0; axis < 6; ++axis) {
    values[index++] = result.trackingError(axis);
  }
  for (Eigen::Index coordinate = 0; coordinate < 6; ++coordinate) {
    values[index++] = measuredRbdState_(coordinate);
  }
  for (Eigen::Index coordinate = 0; coordinate < 6; ++coordinate) {
    values[index++] = desiredRbdState(coordinate);
  }
  values[index++] = myRbfAppliedFy_;
  values[index++] = enableMyRbfControlInjection_ ? 1.0 : 0.0;
  values[index++] = static_cast<double>(myRbfShadowValidSampleCount_);
  values[index++] = static_cast<double>(myRbfShadowRejectedSampleCount_);
  for (Eigen::Index axis = 0; axis < 6; ++axis) {
    values[index++] = result.residual.netResidual(axis);
  }
  for (Eigen::Index axis = 0; axis < 6; ++axis) {
    values[index++] = myRbfRequestedWrench_(axis);
  }
  for (Eigen::Index axis = 0; axis < 6; ++axis) {
    values[index++] = myRbfAppliedWrench_(axis);
  }
  values[index++] = myRbfAllocatorScale_;
  values[index++] = myRbfControlInterface_ == "wrench_allocator" ? 2.0 : 1.0;
  bool updateFrozen = false;
  for (Eigen::Index axis = 0; axis < 6; ++axis) {
    values[index++] = result.estimator.coverage(axis);
    updateFrozen = updateFrozen ||
        result.estimator.updateFrozen[static_cast<std::size_t>(axis)];
  }
  values[index++] = updateFrozen ? 1.0 : 0.0;
  for (Eigen::Index axis = 0; axis < 6; ++axis) {
    values[index++] = result.estimator.predictionDerivativeNorm(axis);
  }
  for (Eigen::Index axis = 0; axis < 6; ++axis) {
    values[index++] = result.estimator.trackingBiasDerivative(axis);
  }
  for (Eigen::Index axis = 0; axis < 6; ++axis) {
    values[index++] = result.estimator.leakageDerivativeNorm(axis);
  }
  for (Eigen::Index axis = 0; axis < 6; ++axis) {
    values[index++] = result.estimator.projectionActive[
        static_cast<std::size_t>(axis)] ? 1.0 : 0.0;
  }
  myRbfShadowPublisher_->unlockAndPublish();
}

void LeggedController::updateStateEstimation(const ros::Time& time, const ros::Duration& period) {
  vector_t jointPos(hybridJointHandles_.size()), jointVel(hybridJointHandles_.size());
  contact_flag_t contacts;
  Eigen::Quaternion<scalar_t> quat;
  contact_flag_t contactFlag;
  vector3_t angularVel, linearAccel;
  matrix3_t orientationCovariance, angularVelCovariance, linearAccelCovariance;

  for (size_t i = 0; i < hybridJointHandles_.size(); ++i) {
    jointPos(i) = hybridJointHandles_[i].getPosition();
    jointVel(i) = hybridJointHandles_[i].getVelocity();
  }

  
  for (size_t i = 0; i < contacts.size(); ++i) {
    //contactFlag[i] = leggedInterface_->getSwitchedModelReferenceManagerPtr()->getContactFlags(currentObservation_.time)[i]; //使用时间序列估计接触
    contactFlag[i] = contactHandles_[i].isContact();
  }
  for (size_t i = 0; i < 4; ++i) {
    quat.coeffs()(i) = imuSensorHandle_.getOrientation()[i];
  }
  for (size_t i = 0; i < 3; ++i) {
    angularVel(i) = imuSensorHandle_.getAngularVelocity()[i];
    linearAccel(i) = imuSensorHandle_.getLinearAcceleration()[i];
  }
  for (size_t i = 0; i < 9; ++i) {
    orientationCovariance(i) = imuSensorHandle_.getOrientationCovariance()[i];
    angularVelCovariance(i) = imuSensorHandle_.getAngularVelocityCovariance()[i];
    linearAccelCovariance(i) = imuSensorHandle_.getLinearAccelerationCovariance()[i];
  }

  stateEstimate_->updateJointStates(jointPos, jointVel);
  stateEstimate_->updateContact(contactFlag);
  stateEstimate_->updateImu(quat, angularVel, linearAccel, orientationCovariance, angularVelCovariance, linearAccelCovariance);
  measuredRbdState_ = stateEstimate_->update(time, period);
  currentObservation_.time += period.toSec();
  scalar_t yawLast = currentObservation_.state(9);
  currentObservation_.state = rbdConversions_->computeCentroidalStateFromRbdModel(measuredRbdState_);
  currentObservation_.state(9) = yawLast + angles::shortest_angular_distance(yawLast, currentObservation_.state(9));
  currentObservation_.mode = stateEstimate_->getMode();
}

LeggedController::~LeggedController() {
  controllerRunning_ = false;
  if (mpcThread_.joinable()) {
    mpcThread_.join();
  }
  std::cerr << "########################################################################";
  std::cerr << "\n### MPC Benchmarking";
  std::cerr << "\n###   Maximum : " << mpcTimer_.getMaxIntervalInMilliseconds() << "[ms].";
  std::cerr << "\n###   Average : " << mpcTimer_.getAverageInMilliseconds() << "[ms]." << std::endl;
  std::cerr << "########################################################################";
  std::cerr << "\n### WBC Benchmarking";
  std::cerr << "\n###   Maximum : " << wbcTimer_.getMaxIntervalInMilliseconds() << "[ms].";
  std::cerr << "\n###   Average : " << wbcTimer_.getAverageInMilliseconds() << "[ms].";
}

void LeggedController::setupLeggedInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                            bool verbose) {
  leggedInterface_ = std::make_shared<LeggedInterface>(taskFile, urdfFile, referenceFile);
  leggedInterface_->setupOptimalControlProblem(taskFile, urdfFile, referenceFile, verbose);
}

void LeggedController::setupMpc() {
  mpc_ = std::make_shared<SqpMpc>(leggedInterface_->mpcSettings(), leggedInterface_->sqpSettings(),
                                  leggedInterface_->getOptimalControlProblem(), leggedInterface_->getInitializer());
  rbdConversions_ = std::make_shared<CentroidalModelRbdConversions>(leggedInterface_->getPinocchioInterface(),
                                                                    leggedInterface_->getCentroidalModelInfo());

  const std::string robotName = "legged_robot";
  ros::NodeHandle nh;
  // Gait receiver
  auto gaitReceiverPtr =
      std::make_shared<GaitReceiver>(nh, leggedInterface_->getSwitchedModelReferenceManagerPtr()->getGaitSchedule(), robotName);
  // ROS ReferenceManager
  auto rosReferenceManagerPtr = std::make_shared<RosReferenceManager>(robotName, leggedInterface_->getReferenceManagerPtr());
  rosReferenceManagerPtr->subscribe(nh);
  mpc_->getSolverPtr()->addSynchronizedModule(gaitReceiverPtr);
  mpc_->getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);
  observationPublisher_ = nh.advertise<ocs2_msgs::mpc_observation>(robotName + "_mpc_observation", 1);
}

void LeggedController::setupMrt() {
  mpcMrtInterface_ = std::make_shared<MPC_MRT_Interface>(*mpc_);
  mpcMrtInterface_->initRollout(&leggedInterface_->getRollout());
  mpcTimer_.reset();

  controllerRunning_ = true;
  mpcThread_ = std::thread([&]() {
    while (controllerRunning_) {
      try {
        executeAndSleep(
            [&]() {
              if (mpcRunning_) {
                mpcTimer_.startTimer();
                mpcMrtInterface_->advanceMpc();
                mpcTimer_.endTimer();
              }
            },
            leggedInterface_->mpcSettings().mpcDesiredFrequency_);
      } catch (const std::exception& e) {
        controllerRunning_ = false;
        ROS_ERROR_STREAM("[Ocs2 MPC thread] Error : " << e.what());
        stopRequest(ros::Time());
      }
    }
  });
  setThreadPriority(leggedInterface_->sqpSettings().threadPriority, mpcThread_);
}

void LeggedController::setupStateEstimate(const std::string& taskFile, bool verbose) {
  stateEstimate_ = std::make_shared<KalmanFilterEstimate>(leggedInterface_->getPinocchioInterface(),
                                                          leggedInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_);
  dynamic_cast<KalmanFilterEstimate&>(*stateEstimate_).loadSettings(taskFile, verbose);
  currentObservation_.time = 0;
}

void LeggedCheaterController::setupStateEstimate(const std::string& /*taskFile*/, bool /*verbose*/) {
  stateEstimate_ = std::make_shared<FromTopicStateEstimate>(leggedInterface_->getPinocchioInterface(),
                                                            leggedInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_);
}

}  // namespace legged

PLUGINLIB_EXPORT_CLASS(legged::LeggedController, controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(legged::LeggedCheaterController, controller_interface::ControllerBase)
