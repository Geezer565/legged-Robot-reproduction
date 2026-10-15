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
#include <ros/package.h>
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

adaptive::v2::Axes toMyRbfV2Axes(const adaptive::vector6_t& value) {
  adaptive::v2::Axes result{};
  for (Eigen::Index axis = 0; axis < 6; ++axis) {
    result[static_cast<std::size_t>(axis)] = value(axis);
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
  controller_nh.param("enable_eth_adaptive_clf_mpc", ethAdaptiveEnabled_, false);
  controller_nh.param("enable_eth_adaptive_wbc_compensation", ethAdaptiveWbcEnabled_, true);
  controller_nh.param("eth_adaptive_settings_file", ethAdaptiveSettingsFile_, std::string());
  controller_nh.param(
      "myrbf_shadow/algorithm", myRbfAlgorithm_, std::string("v1"));
  controller_nh.param("myrbf_shadow/estimator_version",
                      myRbfEstimatorVersion_, std::string("legacy"));
  controller_nh.param("myrbf_shadow/comparison_method",
                      myRbfComparisonMethod_, std::string(""));
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
    scalar_t compositeWindow = 0.2;
    bool normalizeUpdate = false;
    bool trackingBiasOnly = false;
    scalar_t updateEpsilon = 1e-9;
    scalar_t minimumCoverage = 0.0;
    int rffFeatureCount = 50;
    int rffRandomSeed = 20260824;
    scalar_t rffFrequencyStd = 1.0;
    scalar_t rffProjectionRadius = 0.0;
    scalar_t comparisonGaussianWidth = 0.5;
    scalar_t comparisonCompositeRatio = 0.75;
    scalar_t comparisonMinimumStep = 0.2;
    scalar_t comparisonMaximumStep = 1.0;
    scalar_t neuralFlyLambda = 0.1;
    scalar_t neuralFlyProcessNoise = 0.2;
    scalar_t neuralFlyMeasurementNoise = 1.0;
    scalar_t neuralFlyInitialCovariance = 1.0;
    controller_nh.param("myrbf_shadow/filter_alpha", filterAlpha, 0.5);
    controller_nh.param("myrbf_shadow/linear_extent", linearExtent, 2.0);
    controller_nh.param("myrbf_shadow/angular_extent", angularExtent, 4.0);
    controller_nh.param("myrbf_shadow/width_ratio", widthRatio, 0.85);
    controller_nh.param("myrbf_shadow/composite_window", compositeWindow,
                        0.2);
    controller_nh.param("myrbf_shadow/normalize_update", normalizeUpdate,
                        false);
    controller_nh.param("myrbf_shadow/tracking_bias_only", trackingBiasOnly,
                        false);
    controller_nh.param("myrbf_shadow/update_epsilon", updateEpsilon, 1e-9);
    controller_nh.param("myrbf_shadow/minimum_coverage", minimumCoverage, 0.0);
    controller_nh.param("myrbf_shadow/rff_feature_count", rffFeatureCount, 50);
    controller_nh.param("myrbf_shadow/rff_random_seed", rffRandomSeed, 20260824);
    controller_nh.param("myrbf_shadow/rff_frequency_std", rffFrequencyStd, 1.0);
    controller_nh.param("myrbf_shadow/rff_projection_radius", rffProjectionRadius, 0.0);
    controller_nh.param("myrbf_shadow/comparison_gaussian_width",
                        comparisonGaussianWidth, 0.5);
    controller_nh.param("myrbf_shadow/comparison_composite_ratio",
                        comparisonCompositeRatio, 0.75);
    controller_nh.param("myrbf_shadow/comparison_minimum_step",
                        comparisonMinimumStep, 0.2);
    controller_nh.param("myrbf_shadow/comparison_maximum_step",
                        comparisonMaximumStep, 1.0);
    controller_nh.param("myrbf_shadow/neural_fly_lambda",
                        neuralFlyLambda, 0.1);
    controller_nh.param("myrbf_shadow/neural_fly_process_noise",
                        neuralFlyProcessNoise, 0.2);
    controller_nh.param("myrbf_shadow/neural_fly_measurement_noise",
                        neuralFlyMeasurementNoise, 1.0);
    controller_nh.param("myrbf_shadow/neural_fly_initial_covariance",
                        neuralFlyInitialCovariance, 1.0);
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
    myRbfResidualScale_ = residualScale;
    const auto discrepancyScale = loadMyRbfVector6(
        controller_nh, "myrbf_shadow/discrepancy_scale", defaultScale);
    const auto outputScale = loadMyRbfVector6(
        controller_nh, "myrbf_shadow/output_scale", defaultScale);
    const auto predictionGain = loadMyRbfVector6(
        controller_nh, "myrbf_shadow/prediction_gain",
        myRbfV2Lyapunov_ ? adaptive::vector6_t::Zero().eval() : defaultPredictionGain);
    const auto trackingGain = loadMyRbfVector6(
        controller_nh, "myrbf_shadow/tracking_gain",
        adaptive::vector6_t::Zero());
    const auto rffLearningRate = loadMyRbfVector6(
        controller_nh, "myrbf_shadow/rff_learning_rate",
        adaptive::vector6_t::Constant(0.003));
    const auto comparisonAdaptationGain = loadMyRbfVector6(
        controller_nh, "myrbf_shadow/comparison_adaptation_gain",
        adaptive::vector6_t::Constant(0.25));
    const auto comparisonTrackingScale = loadMyRbfVector6(
        controller_nh, "myrbf_shadow/comparison_tracking_scale",
        adaptive::vector6_t::Ones());
    const auto leakage = loadMyRbfVector6(
        controller_nh, "myrbf_shadow/leakage",
        adaptive::vector6_t::Constant(0.02));
    const auto residualBias = loadMyRbfVector6(
        controller_nh, "myrbf_shadow/residual_bias",
        adaptive::vector6_t::Zero());
    const auto residualDeadzone = loadMyRbfVector6(
        controller_nh, "myrbf_shadow/residual_deadzone",
        adaptive::vector6_t::Zero());
    const auto weightLimit = loadMyRbfVector6(
        controller_nh, "myrbf_shadow/weight_limit",
        adaptive::vector6_t::Constant(
            std::numeric_limits<scalar_t>::infinity()));
    const auto v2Gamma = loadMyRbfVector6(
        controller_nh, "myrbf_shadow/v2_gamma",
        adaptive::vector6_t::Constant(0.002));
    const auto v2RobustGamma = loadMyRbfVector6(
        controller_nh, "myrbf_shadow/v2_robust_gamma",
        adaptive::vector6_t::Constant(0.2));
    const auto v2Nu0 = loadMyRbfVector6(
        controller_nh, "myrbf_shadow/v2_nu0",
        (adaptive::vector6_t() << 0.08, 0.08, 0.08, 0.04, 0.04, 0.04)
            .finished());
    const auto v2NuTime = loadMyRbfVector6(
        controller_nh, "myrbf_shadow/v2_nu_time",
        adaptive::vector6_t::Constant(20.0));
    const auto v2NuExponent = loadMyRbfVector6(
        controller_nh, "myrbf_shadow/v2_nu_exponent",
        adaptive::vector6_t::Constant(0.5));
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

    if (myRbfAlgorithm_ != "v1" && myRbfAlgorithm_ != "v2" &&
        myRbfAlgorithm_ != "v2_lyapunov" && myRbfAlgorithm_ != "v3_teacher" &&
        myRbfAlgorithm_ != "v4_convergence" &&
        myRbfAlgorithm_ != "v5_residual_dynamics" &&
        myRbfAlgorithm_ != "v6_variable_gain" && myRbfAlgorithm_ != "v11_equivalent_wrench") {
      ROS_ERROR_STREAM("[MyRBF] Unknown algorithm: " << myRbfAlgorithm_);
      return false;
    }
    if (myRbfEstimatorVersion_ != "legacy" &&
        myRbfEstimatorVersion_ != "rff" &&
        myRbfEstimatorVersion_ != "eth_direct" &&
        myRbfEstimatorVersion_ != "liu_dual_rbf" &&
        myRbfEstimatorVersion_ != "neural_fly_constant") {
      ROS_ERROR_STREAM("[MyRBF] Unknown estimator_version: " << myRbfEstimatorVersion_);
      return false;
    }
    if (myRbfEstimatorVersion_ == "rff" && myRbfAlgorithm_ != "v1") {
      ROS_ERROR("[MyRBF] RFF estimator is only available through the V1-compatible comparison interface.");
      return false;
    }
    const bool comparisonEstimator =
        myRbfEstimatorVersion_ == "eth_direct" ||
        myRbfEstimatorVersion_ == "liu_dual_rbf" ||
        myRbfEstimatorVersion_ == "neural_fly_constant";
    if (comparisonEstimator && myRbfAlgorithm_ != "v1") {
      ROS_ERROR("[MyRBF] Comparison estimators require the shared V1-compatible control adapter.");
      return false;
    }
    if ((myRbfEstimatorVersion_ == "eth_direct" &&
         myRbfComparisonMethod_ != "eth") ||
        (myRbfEstimatorVersion_ == "liu_dual_rbf" &&
         myRbfComparisonMethod_ != "liu_van_mpc_port") ||
        (myRbfEstimatorVersion_ == "neural_fly_constant" &&
         myRbfComparisonMethod_ != "neural_fly_constant") ||
        (myRbfEstimatorVersion_ == "rff" &&
         myRbfComparisonMethod_ != "zhou_ssi")) {
      ROS_ERROR("[MyRBF] comparison_method and estimator_version do not match.");
      return false;
    }
    if (rffFeatureCount <= 0 || rffFeatureCount > 10000 ||
        rffRandomSeed < 0 || !std::isfinite(rffFrequencyStd) ||
        rffFrequencyStd <= 0.0 || !std::isfinite(rffProjectionRadius) ||
        rffProjectionRadius < 0.0 || !rffLearningRate.allFinite() ||
        (rffLearningRate.array() < 0.0).any()) {
      ROS_ERROR("[MyRBF RFF] Random-feature settings are invalid.");
      return false;
    }
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
    if (!(myRbfV11_ && myRbfControlInterface_ == "model_and_wbc") && myRbfControlInterface_ != "legacy_fy" &&
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
    if (!myRbfV11_ && (myRbfAlgorithm_ == "v2" || myRbfV2Lyapunov_) &&
        ((!v2Gamma.allFinite()) || (v2Gamma.array() <= 0.0).any() ||
         (!v2RobustGamma.allFinite()) ||
         (v2RobustGamma.array() <= 0.0).any() ||
         (!v2Nu0.allFinite()) || (v2Nu0.array() <= 0.0).any() ||
         (!v2NuTime.allFinite()) || (v2NuTime.array() <= 0.0).any() ||
         (!v2NuExponent.allFinite()) ||
         (v2NuExponent.array() <= 0.0).any() ||
         (!weightLimit.allFinite()) ||
         (weightLimit.array() <= 0.0).any())) {
      ROS_ERROR("[MyRBF V2] Adaptive parameters must be finite and positive.");
      return false;
    }
    if ((residualDeadzone.array() < 0.0).any() ||
        (leakage.array() < 0.0).any()) {
      ROS_ERROR("[MyRBF V4] Calibration deadzones and leakage must be nonnegative.");
      return false;
    }

    try {
      const auto& modelInfo = leggedInterface_->getCentroidalModelInfo();
      if (myRbfV11_) {
        configureMyRbfV11(controller_nh);
      } else if (myRbfV5Dynamics_ || myRbfV6Dynamics_) {
        configureMyRbfV5(controller_nh);
      } else if (myRbfAlgorithm_ == "v2" || myRbfV2Lyapunov_) {
        adaptive::v2::Settings settings;
        settings.residualScale = toMyRbfV2Axes(residualScale);
        settings.outputScale = toMyRbfV2Axes(outputScale);
        settings.extent = {{linearExtent, linearExtent, linearExtent,
                            angularExtent, angularExtent, angularExtent}};
        settings.gamma = toMyRbfV2Axes(v2Gamma);
        settings.predictionGain = toMyRbfV2Axes(predictionGain);
        settings.weightLimit = toMyRbfV2Axes(weightLimit);
        settings.residualBias = toMyRbfV2Axes(residualBias);
        settings.residualDeadzone = toMyRbfV2Axes(residualDeadzone);
        settings.convergenceLeakage = toMyRbfV2Axes(leakage);
        settings.robustGamma = toMyRbfV2Axes(v2RobustGamma);
        settings.nu0 = toMyRbfV2Axes(v2Nu0);
        settings.nuTime = toMyRbfV2Axes(v2NuTime);
        settings.nuExponent = toMyRbfV2Axes(v2NuExponent);
        settings.widthRatio = widthRatio;
        settings.epsilon = updateEpsilon;
        settings.compositeWindow = compositeWindow;
        if (myRbfV2Lyapunov_) {
          configureMyRbfV2Lyapunov(controller_nh);
          if(!myRbfV3Teacher_ && !predictionGain.isZero(0))
            throw std::invalid_argument("V2 resolved prediction gain must be zero");
          settings.gamma = toMyRbfV2Axes(loadMyRbfVector6(controller_nh,"myrbf_shadow/weight_gamma",v2Gamma));
          settings.robustGamma = toMyRbfV2Axes(loadMyRbfVector6(controller_nh,"myrbf_shadow/robust_gamma",v2RobustGamma));
          settings.robustMode = myRbfV2RobustMode_ == "sign" ? adaptive::v2::RobustMode::Sign : adaptive::v2::RobustMode::Smooth;
          settings.featureMode = myRbfV2FeatureMode_ == "contact_aware" ? adaptive::v2::FeatureMode::ContactAware : adaptive::v2::FeatureMode::ResidualOnly;
          settings.updateLaw = myRbfV3Teacher_
              ? adaptive::v2::UpdateLaw::TeacherComposite
              : (myRbfV4Convergence_
                    ? adaptive::v2::UpdateLaw::ResidualConvergence
                    : adaptive::v2::UpdateLaw::V2Tracking);
          settings.robotMass = modelInfo.robotMass;
          if (myRbfV2RobustMode_ == "smooth") for (int i=1;i<6;++i) {
            if (settings.nu0[i]!=settings.nu0[0] || settings.nuTime[i]!=settings.nuTime[0] || settings.nuExponent[i]!=settings.nuExponent[0])
              throw std::invalid_argument("V2 smooth WBC currently requires equal six-axis boundary-layer widths");
          }
        }
        myRbfV2Pipeline_ = std::make_unique<adaptive::MyRbfV2Pipeline>(
            modelInfo.robotMass, modelInfo.stateDim, true,
            filterAlpha, myRbfShadowTimeTolerance_, std::move(settings));
      } else if (myRbfEstimatorVersion_ == "rff") {
        adaptive::RandomFourierWrenchEstimatorSettings settings;
        settings.residualScale = residualScale;
        settings.discrepancyScale = discrepancyScale;
        settings.outputScale = outputScale;
        settings.learningRate = rffLearningRate;
        settings.featureCount = rffFeatureCount;
        settings.randomSeed = static_cast<std::uint32_t>(rffRandomSeed);
        settings.frequencyStd = rffFrequencyStd;
        settings.projectionRadius = rffProjectionRadius;
        myRbfShadowPipeline_ = std::make_unique<adaptive::MyRbfShadowPipeline>(
            modelInfo.robotMass, modelInfo.stateDim, true,
            filterAlpha, myRbfShadowTimeTolerance_, std::move(settings));
      } else if (comparisonEstimator) {
        adaptive::ComparisonWrenchEstimatorSettings settings;
        if (myRbfEstimatorVersion_ == "eth_direct") {
          settings.method = adaptive::ComparisonMethod::EthDirect;
        } else if (myRbfEstimatorVersion_ == "liu_dual_rbf") {
          settings.method = adaptive::ComparisonMethod::LiuDualRbfPort;
        } else {
          settings.method = adaptive::ComparisonMethod::NeuralFlyConstant;
        }
        settings.residualScale = residualScale;
        settings.trackingScale = comparisonTrackingScale;
        settings.outputScale = outputScale;
        settings.adaptationGain = comparisonAdaptationGain;
        settings.weightLimit = weightLimit;
        settings.gaussianWidth = comparisonGaussianWidth;
        settings.compositeRatio = comparisonCompositeRatio;
        settings.minimumStep = comparisonMinimumStep;
        settings.maximumStep = comparisonMaximumStep;
        settings.neuralFlyLambda = neuralFlyLambda;
        settings.neuralFlyProcessNoise = neuralFlyProcessNoise;
        settings.neuralFlyMeasurementNoise = neuralFlyMeasurementNoise;
        settings.neuralFlyInitialCovariance = neuralFlyInitialCovariance;
        myRbfShadowPipeline_ = std::make_unique<adaptive::MyRbfShadowPipeline>(
            modelInfo.robotMass, modelInfo.stateDim, true,
            filterAlpha, myRbfShadowTimeTolerance_, std::move(settings));
      } else {
        auto settings = adaptive::createSymmetricGridMyRbfShadowSettings(
            linearExtent, angularExtent, widthRatio,
            residualScale, outputScale,
            trackingGain, predictionGain, leakage);
        settings.normalizeUpdate = normalizeUpdate;
        settings.trackingBiasOnly = trackingBiasOnly;
        settings.updateEpsilon = updateEpsilon;
        settings.minimumCoverage = minimumCoverage;
        settings.weightLimit = weightLimit;
        myRbfShadowPipeline_ = std::make_unique<adaptive::MyRbfShadowPipeline>(
            modelInfo.robotMass, modelInfo.stateDim, true,
            filterAlpha, myRbfShadowTimeTolerance_, std::move(settings));
      }
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
      ROS_ERROR_STREAM("[MyRBF] Invalid configuration: " << error.what());
      return false;
    }

    ros::NodeHandle shadowNodeHandle;
    myRbfShadowPublisher_ = std::make_unique<
        realtime_tools::RealtimePublisher<std_msgs::Float64MultiArray>>(
        shadowNodeHandle, "/myrbf_shadow_diagnostics", 100);
    myRbfShadowPublisher_->msg_.data.resize(124, 0.0);
    if (enableMyRbfControlInjection_) {
      ROS_WARN_STREAM("[MyRBF " << myRbfAlgorithm_
                      << " control] Injection enabled through "
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
  std::unique_lock<std::mutex> lifecycle(myRbfV2MpcLifecycleMutex_,std::defer_lock);
  if(myRbfV2Lyapunov_) {
    mpcRunning_=false;
    lifecycle.lock();  // Wait for any previous-run solve before clearing its policy.
    myRbfV2SolverFault_=false;
    mpcMrtInterface_->reset();
    myRbfV2PolicyBinding_->resetQuiescent();
  }
  myRbfV2MomentumValid_ = false;
  myRbfV2Armed_ = false;
  myRbfV2DiagnosticSequence_ = myRbfV2DiagnosticDrops_ = 0;
  myRbfV2ExternalResidual_.setZero();
  myRbfV2AppliedCorrection_.setZero();
  myRbfV3TeacherErrorValid_ = false;
  myRbfV3UpdateDt_ = 0.0;
  myRbfV3PredictionError_.setZero();
  myRbfV3TrackingError_.setZero();
  myRbfV3CompositeError_.setZero();
  const auto teacherStateDim=leggedInterface_->getCentroidalModelInfo().stateDim;
  myRbfV3PredictionStateError_.setZero(teacherStateDim);
  myRbfV3TrackingStateError_.setZero(teacherStateDim);
  myRbfV3CompositeStateError_.setZero(teacherStateDim);
  myRbfV2WbcInput_.previousContactForces.resize(0);
  if (myRbfV2Lyapunov_) {
    auto resetInput = myRbfV2WbcInput_;
    resetInput.enabled = false;
    wbc_->configureMyRbfV2(resetInput);
    adaptive::MyRbfV2Snapshot initial;
    initial.valid = true;
    initial.v11Enabled = myRbfV11_;
    initial.modelUsesSubtractedCorrection = myRbfV3Teacher_ && !myRbfV3SharedAllocator_;
    initial.sequenceId = ++myRbfV2Sequence_;
    initial.smoothingWidth.fill(myRbfV2WbcInput_.smoothWidth);
    myRbfV2Snapshots_->tryPublish(initial);
  }
  myRbfShadowPredictionPending_ = false;
  if(myRbfV11Core_)myRbfV11Core_->reset();
  myRbfV11PredictionValid_=false;
  myRbfShadowPredictionTime_ = std::numeric_limits<scalar_t>::quiet_NaN();
  myRbfShadowValidSampleCount_ = 0;
  myRbfShadowRejectedSampleCount_ = 0;
  myRbfShadowRolloutFailureCount_ = 0;
  myRbfAppliedFy_ = 0.0;
  myRbfRequestedWrench_.setZero();
  myRbfAppliedWrench_.setZero();
  myRbfAdaptiveWrench_.setZero();
  myRbfAllocatorScale_ = 0.0;
  if (myRbfShadowPipeline_) {
    myRbfShadowPipeline_->reset();
  }
  if (myRbfV2Pipeline_) {
    myRbfV2Pipeline_->reset();
  }
  if (myRbfV5Core_) {
    myRbfV5Core_->reset();
  }
  myRbfV5PredictionValid_ = false;
  myRbfV5PredictionDt_ = 0.0;
  myRbfV5DiagnosticSequence_ = myRbfV5DiagnosticDrops_ = 0;
  myRbfV5BaseVelocity_.fill(0.0);
  myRbfV5NominalRate_.fill(0.0);
  myRbfV5KnownAppliedAcceleration_.fill(0.0);
  myRbfV5ObservedRate_.fill(0.0);
  myRbfV5Correction_.fill(0.0);
  // Initial state
  currentObservation_.state.setZero(leggedInterface_->getCentroidalModelInfo().stateDim);
  updateStateEstimation(time, ros::Duration(0.002));
  myRbfV2ActivationStartTime_ = currentObservation_.time;
  currentObservation_.input.setZero(leggedInterface_->getCentroidalModelInfo().inputDim);
  currentObservation_.mode = ModeNumber::STANCE;

  TargetTrajectories target_trajectories({currentObservation_.time}, {currentObservation_.state}, {currentObservation_.input});
  if(myRbfV2Lyapunov_)mpcMrtInterface_->resetMpcNode(target_trajectories);

  // Set the first observation and command and wait for optimization to finish
  mpcMrtInterface_->setCurrentObservation(currentObservation_);
  mpcMrtInterface_->getReferenceManager().setTargetTrajectories(target_trajectories);
  ROS_INFO_STREAM("Waiting for the initial policy ...");
  while (!mpcMrtInterface_->initialPolicyReceived() && ros::ok()) {
    mpcMrtInterface_->advanceMpc();
    ros::WallRate(leggedInterface_->mpcSettings().mrtDesiredFrequency_).sleep();
  }
  ROS_INFO_STREAM("Initial policy has been received.");
  if(myRbfV2Lyapunov_) {
    // The background solver remains quiescent under lifecycle: adoption cannot
    // lose MRT's try-lock race to a second result before the first update.
    if(!mpcMrtInterface_->updatePolicy() || !myRbfV2PolicyBinding_->active() ||
       !myRbfV2PolicyBinding_->active()->valid)
      throw std::runtime_error("V2 initial policy has no valid matching snapshot");
  }

  mpcRunning_ = true;
}

void LeggedController::update(const ros::Time& time, const ros::Duration& period) {
  if(myRbfV2Lyapunov_) {
    // Only the control thread writes/clears actuator commands. An asynchronous
    // solver failure is latched, not handled by racing stopRequest from MPC.
    if(myRbfV2SolverFault_) {
      ROS_ERROR("[V2] MPC solver failed; stopping on control thread");
      stopRequest(time);return;
    }
    try {
      updateStateEstimation(time,period);
      mpcMrtInterface_->setCurrentObservation(currentObservation_);
      mpcMrtInterface_->updatePolicy();
      vector_t optimizedState,optimizedInput;
      size_t plannedMode=0;
      mpcMrtInterface_->evaluatePolicy(currentObservation_.time,currentObservation_.state,optimizedState,optimizedInput,plannedMode);
      if(myRbfV11_){
        updateMyRbfV11(time,period.toSec(),optimizedState,optimizedInput,plannedMode);return;
      }
      if(updateMyRbfV2Warmup(time,period.toSec(),optimizedState,optimizedInput,plannedMode))return;
      updateMyRbfV2Control(time,period.toSec(),optimizedState,optimizedInput,plannedMode);
    } catch(const std::exception& e) {
      ROS_ERROR_STREAM("[V2] Control cycle exception: "<<e.what());
      stopRequest(time);
    }
    return;
  }
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
  if (ethAdaptiveEnabled_) {
    try {
      updateEthAdaptiveWbc(optimizedState, optimizedInput, plannedMode, period.toSec());
    } catch (const std::exception& error) {
      ROS_ERROR_STREAM("[ETH Adaptive CLF-MPC] WBC bridge failure: " << error.what());
      stopRequest(time);
      return;
    }
  }
  updateMyRbfShadowObservation(optimizedState, optimizedInput);
  applyMyRbfControlCompensation(optimizedInput, plannedMode, period.toSec());
  queueMyRbfShadowPrediction();

  // Whole body control
  currentObservation_.input = optimizedInput;

  wbcTimer_.startTimer();
  vector_t x = wbc_->update(optimizedState, optimizedInput, measuredRbdState_, plannedMode, period.toSec());
  wbcTimer_.endTimer();
  if (x.size() != leggedInterface_->getCentroidalModelInfo().generalizedCoordinatesNum + 24 || !x.allFinite()) {
    ROS_ERROR("[Legged Controller] WBC failed; stopping before sending joint commands");
    stopRequest(time);
    return;
  }

  vector_t torque = x.tail(12);

  vector_t posDes = centroidal_model::getJointAngles(optimizedState, leggedInterface_->getCentroidalModelInfo());
  vector_t velDes = centroidal_model::getJointVelocities(optimizedInput, leggedInterface_->getCentroidalModelInfo());

  // Safety check, if failed, stop the controller
  if (!safetyChecker_->check(currentObservation_, optimizedState, optimizedInput)) {
    ROS_ERROR_STREAM("[Legged Controller] Safety check failed, stopping the controller.");
    stopRequest(time);
    return;
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
  if (!enableMyRbfShadow_ || !hasMyRbfPipeline() ||
      !myRbfShadowPredictionPending_) {
    return;
  }
  if (currentObservation_.time <
      myRbfShadowPredictionTime_ - myRbfShadowTimeTolerance_) {
    return;
  }

  myRbfShadowPredictionPending_ = false;
  if (myRbfV5Dynamics_ || myRbfV6Dynamics_) {
    try {
      observeMyRbfV5();
    } catch (const std::exception& error) {
      ++myRbfShadowRejectedSampleCount_;
      ROS_WARN_STREAM_THROTTLE(
          1.0, "[MyRBF V5] Observation failed: " << error.what());
    }
    return;
  }
  vector_t desiredRbdState;
  bool valid = false;
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
    if (myRbfAlgorithm_ == "v2") {
      // computeMyRbfTrackingError returns desired minus actual. V2 uses the
      // opposite convention: chi = actual minus desired.
      const auto result = myRbfV2Pipeline_->observeActual(
          currentObservation_.time, currentObservation_.state, -trackingError);
      valid = result.valid;
      if (valid) {
        myRbfAdaptiveWrench_ = myRbfV2Pipeline_->compensationWrench();
        ++myRbfShadowValidSampleCount_;
        publishMyRbfV2Diagnostics(result, desiredRbdState);
      }
    } else {
      const auto result = myRbfShadowPipeline_->observeActual(
          currentObservation_.time, currentObservation_.state, trackingError);
      valid = result.valid;
      if (valid) {
        myRbfAdaptiveWrench_ = myRbfShadowPipeline_->estimate();
        ++myRbfShadowValidSampleCount_;
        publishMyRbfShadowDiagnostics(result, desiredRbdState);
      }
    }
  } catch (const std::exception& error) {
    ++myRbfShadowRejectedSampleCount_;
    ROS_WARN_STREAM_THROTTLE(
        1.0, "[MyRBF shadow] Tracking observation failed: " << error.what());
    return;
  }
  if (!valid) {
    ++myRbfShadowRejectedSampleCount_;
  }
}

void LeggedController::queueMyRbfShadowPrediction() {
  if (!enableMyRbfShadow_ || !hasMyRbfPipeline() ||
      myRbfShadowPredictionPending_) {
    return;
  }

  if (myRbfV5Dynamics_ || myRbfV6Dynamics_) {
    try {
      queueMyRbfV5Prediction();
    } catch (const std::exception& error) {
      ++myRbfShadowRolloutFailureCount_;
      myRbfV5PredictionValid_ = false;
      ROS_WARN_STREAM_THROTTLE(
          1.0, "[MyRBF V5] One-step prediction failed: " << error.what());
    }
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
    if (myRbfAlgorithm_ == "v2") {
      myRbfV2Pipeline_->setPrediction(
          myRbfShadowPredictionTime_, myRbfShadowUpdatePeriod_, predictedState,
          knownAppliedCompensation);
    } else {
      myRbfShadowPipeline_->setPrediction(
          myRbfShadowPredictionTime_, myRbfShadowUpdatePeriod_, predictedState,
          knownAppliedCompensation);
    }
    myRbfShadowPredictionPending_ = true;
  } catch (const std::exception& error) {
    ++myRbfShadowRolloutFailureCount_;
    ROS_WARN_STREAM_THROTTLE(
        1.0, "[MyRBF shadow] One-step prediction failed: " << error.what());
  }
}

void LeggedController::applyMyRbfControlCompensation(
    vector_t& optimizedInput, std::size_t plannedMode, scalar_t dt) {
  if (!enableMyRbfControlInjection_ || !hasMyRbfPipeline()) {
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
  if (!enableMyRbfControlInjection_ || !hasMyRbfPipeline()) {
    myRbfAppliedFy_ = 0.0;
    return;
  }

  const contact_flag_t contactFlags = modeNumber2StanceLeg(plannedMode);
  std::size_t stanceCount = 0;
  for (bool inContact : contactFlags) {
    stanceCount += inContact ? 1U : 0U;
  }
  const scalar_t estimate =
      stanceCount > 0 ? myRbfAdaptiveWrench_(1) : 0.0;
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
        myRbfRequestedWrench_, myRbfAdaptiveWrench_,
        myRbfControlGainVector_, myRbfControlMagnitudeLimit_,
        myRbfControlRateLimitVector_,
        std::max(dt, myRbfShadowUpdatePeriod_));
  }

  adaptive::WrenchAllocationInput allocationInput;
  allocationInput.requestedWrench = myRbfRequestedWrench_;
  // The common controller convention is a world-frame wrench about the base origin.
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
    allocationInput.wrenchReferencePosition = q.head<3>();
    myRbfEeKinematics_->setPinocchioInterface(*myRbfPinocchioInterface_);
    const auto contactPositions =
        myRbfEeKinematics_->getPosition(vector_t());
    for (std::size_t contact = 0;
         contact < adaptive::kMyRbfContactCount; ++contact) {
      allocationInput.contacts[contact] = contactFlags[contact];
      allocationInput.contactPositions[contact] =
          contactPositions[contact];
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
  values[index++] = 1.0;
  for (Eigen::Index axis = 0; axis < 18; ++axis) {
    values[index++] = 0.0;
  }
  myRbfShadowPublisher_->unlockAndPublish();
}

void LeggedController::publishMyRbfV2Diagnostics(
    const adaptive::MyRbfV2PipelineResult& result,
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
    values[index++] = result.estimateBeforeUpdate(axis);
  }
  for (Eigen::Index axis = 0; axis < 6; ++axis) {
    values[index++] = result.estimateAfterUpdate(axis);
  }
  for (Eigen::Index axis = 0; axis < 6; ++axis) {
    values[index++] =
        result.residual.filteredResidual(axis) / myRbfResidualScale_(axis);
  }
  for (Eigen::Index axis = 0; axis < 6; ++axis) {
    values[index++] = -result.chi(axis);
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
  for (Eigen::Index axis = 0; axis < 6; ++axis) {
    values[index++] = result.coverage(axis);
  }
  values[index++] = 0.0;
  for (Eigen::Index axis = 0; axis < 6; ++axis) {
    values[index++] = 0.0;
  }
  for (Eigen::Index axis = 0; axis < 6; ++axis) {
    values[index++] = result.weightDerivativeNorm(axis);
  }
  for (Eigen::Index axis = 0; axis < 6; ++axis) {
    values[index++] = 0.0;
  }
  for (Eigen::Index axis = 0; axis < 6; ++axis) {
    values[index++] = result.projectionActive[
        static_cast<std::size_t>(axis)] ? 1.0 : 0.0;
  }
  values[index++] = 2.0;
  for (Eigen::Index axis = 0; axis < 6; ++axis) {
    values[index++] = result.robustWrench(axis);
  }
  for (Eigen::Index axis = 0; axis < 6; ++axis) {
    values[index++] = result.robustBound(axis);
  }
  for (Eigen::Index axis = 0; axis < 6; ++axis) {
    values[index++] = result.smoothingWidth(axis);
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
  if (ethAdaptiveEnabled_) {
    if (ethAdaptiveSettingsFile_.empty()) {
      ethAdaptiveSettingsFile_ = ros::package::getPath("ocs2_legged_robot_adaptive") +
          "/config/adaptive_settings_eth_paper_full_16d.info";
    }
    if (ethAdaptiveSettingsFile_.empty()) {
      throw std::runtime_error("ETH adaptive settings package path is unavailable");
    }
    leggedInterface_->setEthAdaptiveSettingsFile(ethAdaptiveSettingsFile_);
  }
  myRbfV11_ = myRbfAlgorithm_ == "v11_equivalent_wrench" && enableMyRbfShadow_ && enableMyRbfControlInjection_;
  myRbfV3Teacher_ = myRbfAlgorithm_ == "v3_teacher" && enableMyRbfShadow_ && enableMyRbfControlInjection_;
  myRbfV4Convergence_ = myRbfAlgorithm_ == "v4_convergence" && enableMyRbfShadow_ && enableMyRbfControlInjection_;
  myRbfV5Dynamics_ = myRbfAlgorithm_ == "v5_residual_dynamics" &&
                     enableMyRbfShadow_ && enableMyRbfControlInjection_;
  myRbfV6Dynamics_ = myRbfAlgorithm_ == "v6_variable_gain" &&
                      enableMyRbfShadow_ && enableMyRbfControlInjection_;
  myRbfV2Lyapunov_ = (myRbfAlgorithm_ == "v2_lyapunov" || myRbfV3Teacher_ ||
                      myRbfV4Convergence_ || myRbfV11_) &&
                     enableMyRbfShadow_ && enableMyRbfControlInjection_;
  if (myRbfV2Lyapunov_) {
    myRbfV2Snapshots_ = std::make_shared<adaptive::MyRbfV2SnapshotStore>();
    myRbfV2Snapshots_->v11Mode=myRbfV11_;
    leggedInterface_->setMyRbfV2Snapshots(myRbfV2Snapshots_);
    myRbfV2PolicyBinding_ = std::make_shared<MyRbfV2PolicyBinding>(myRbfV2Snapshots_);
  }
  leggedInterface_->setupOptimalControlProblem(taskFile, urdfFile, referenceFile, verbose);
  if (ethAdaptiveEnabled_) {
    ethAdaptivePolicyBinding_ = std::make_shared<EthAdaptivePolicyBinding>(
        leggedInterface_->getEthAdaptiveParameterUpdateModule(),
        leggedInterface_->getEthAdaptiveModelData());
    ros::NodeHandle node;
    node.setParam("/eth_adaptive_columns", std::vector<std::string>{
        "time", "snapshot_time", "snapshot_version",
        "pi_hat_m", "pi_hat_hx", "pi_hat_hy", "pi_hat_hz",
        "pi_hat_jxx", "pi_hat_jxy", "pi_hat_jxz", "pi_hat_jyy", "pi_hat_jyz", "pi_hat_jzz",
        "pi_hat_fx_base", "pi_hat_fy_base", "pi_hat_fz_base", "pi_hat_tx_base", "pi_hat_ty_base", "pi_hat_tz_base",
        "estimated_yupi_world_com_fx", "estimated_yupi_world_com_fy", "estimated_yupi_world_com_fz",
        "estimated_yupi_world_com_tx", "estimated_yupi_world_com_ty", "estimated_yupi_world_com_tz",
        "physical_external_world_base_fx", "physical_external_world_base_fy", "physical_external_world_base_fz",
        "physical_external_world_base_tx", "physical_external_world_base_ty", "physical_external_world_base_tz",
        "true_yupi_world_com_fx", "true_yupi_world_com_fy", "true_yupi_world_com_fz",
        "true_yupi_world_com_tx", "true_yupi_world_com_ty", "true_yupi_world_com_tz",
        "diag_valid", "diag_snapshot_time", "diag_observation_time", "diag_snapshot_version",
        "desired_com_z", "actual_com_z", "desired_com_vz", "actual_com_vz",
        "sigma_x", "sigma_y", "sigma_z",
        "reference_accel_x", "reference_accel_y", "reference_accel_z",
        "mass_raw_drive", "mass_applied_drive", "mass_parameter_derivative",
        "baseline_correction_applied", "parameter_update_enabled", "encoded_reference_derivatives"});
    ethAdaptivePublisher_ = std::make_unique<realtime_tools::RealtimePublisher<std_msgs::Float64MultiArray>>(
        node, "/eth_adaptive_diagnostics", 100);
    ethAdaptivePublisher_->msg_.data.resize(57);
  }
  // Freeze the actual nominal model used to convert logged momentum to physical velocity.
  const auto& model=leggedInterface_->getCentroidalModelInfo();
  ros::NodeHandle modelNode;
  modelNode.setParam("/go2_nominal_model/mass",model.robotMass);
  modelNode.setParam("/go2_nominal_model/type",static_cast<int>(model.centroidalModelType));
  std::vector<double> inertia,comToBase;
  for(int i=0;i<3;++i){comToBase.push_back(model.comToBasePositionNominal(i));
    for(int j=0;j<3;++j)inertia.push_back(model.centroidalInertiaNominal(i,j));}
  modelNode.setParam("/go2_nominal_model/inertia_body_row_major",inertia);
  modelNode.setParam("/go2_nominal_model/com_to_base_body",comToBase);
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
  if (ethAdaptivePolicyBinding_) mpc_->getSolverPtr()->addSynchronizedModule(ethAdaptivePolicyBinding_);
  if (myRbfV2PolicyBinding_) mpc_->getSolverPtr()->addSynchronizedModule(myRbfV2PolicyBinding_);
  mpc_->getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);
  observationPublisher_ = nh.advertise<ocs2_msgs::mpc_observation>(robotName + "_mpc_observation", 1);
}

void LeggedController::setupMrt() {
  mpcMrtInterface_ = std::make_shared<MPC_MRT_Interface>(*mpc_);
  if (ethAdaptivePolicyBinding_) mpcMrtInterface_->addMrtObserver(ethAdaptivePolicyBinding_);
  if (myRbfV2PolicyBinding_) mpcMrtInterface_->addMrtObserver(myRbfV2PolicyBinding_);
  mpcMrtInterface_->initRollout(&leggedInterface_->getRollout());
  mpcTimer_.reset();

  controllerRunning_ = true;
  mpcThread_ = std::thread([&]() {
    while (controllerRunning_) {
      // Cover fault publication as well as solve work: a previous-run exception
      // must not publish its fault after starting() has reset the next run.
      std::unique_lock<std::mutex> lifecycle(myRbfV2MpcLifecycleMutex_,std::defer_lock);
      if(myRbfV2Lyapunov_)lifecycle.lock();
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
        ROS_ERROR_STREAM("[Ocs2 MPC thread] Error : " << e.what());
        if(myRbfV2Lyapunov_) {
          mpcRunning_=false;
          myRbfV2SolverFault_=true;
        } else {
          controllerRunning_ = false;
          stopRequest(ros::Time());
        }
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
