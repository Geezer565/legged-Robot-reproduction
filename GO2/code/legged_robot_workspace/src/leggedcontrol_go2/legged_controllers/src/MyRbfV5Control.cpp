#include "legged_controllers/LeggedController.h"

#include <Eigen/LU>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace legged {
namespace {

adaptive::v5::Axes v5Axes(const adaptive::vector6_t& value) {
  adaptive::v5::Axes result{};
  for (std::size_t axis = 0; axis < 6; ++axis) result[axis] = value(axis);
  return result;
}

adaptive::v5::Axes physicalVelocity(const vector_t& rbdState,
                                    Eigen::Index generalizedCoordinates) {
  if (rbdState.size() < generalizedCoordinates + 6 || !rbdState.allFinite()) {
    throw std::invalid_argument("V5 requires a finite physical RBD state");
  }
  adaptive::v5::Axes result{};
  for (std::size_t axis = 0; axis < 3; ++axis) {
    result[axis] = rbdState(generalizedCoordinates + 3 + axis);
    result[3 + axis] = rbdState(generalizedCoordinates + axis);
  }
  return result;
}

template <std::size_t N>
std::array<double, N> arrayParameter(
    ros::NodeHandle& nh, const std::string& key,
    const std::array<double, N>& fallback) {
  std::vector<double> values;
  if (!nh.hasParam("myrbf_shadow/" + key)) return fallback;
  if (!nh.getParam("myrbf_shadow/" + key, values) || values.size() != N) {
    throw std::invalid_argument(key + " requires " + std::to_string(N) +
                                " numeric entries");
  }
  std::array<double, N> result{};
  for (std::size_t index = 0; index < N; ++index) {
    if (!std::isfinite(values[index])) {
      throw std::invalid_argument(key + " must contain finite values");
    }
    result[index] = values[index];
  }
  return result;
}

std::vector<std::string> v5Columns(bool v6) {
  std::vector<std::string> names{
      "time", "diagnostic_sequence", "diagnostic_publish_drops",
      v6 ? "v6_enabled" : "v5_enabled",
      v6 ? "v6_rbf_enabled" : "v5_rbf_enabled", "actual_update_dt"};
  const std::array<std::string, 6> suffix{{"fx", "fy", "fz", "tx", "ty", "tz"}};
  for (const std::string& group :
       {"velocity", "nominal_rate", "observed_rate",
        "known_applied_acceleration", "raw_target_residual",
        "target_residual", "residual_memory", "residual_trend",
        "prediction_error",
        "correction_acceleration", "total_rate",
        "external_wrench_estimate", "compensation_wrench"}) {
    for (const auto& axis : suffix) names.push_back(group + "_" + axis);
  }
  for (std::size_t contact = 0; contact < adaptive::v5::kContactInputs;
       ++contact) {
    names.push_back("contact_" + std::to_string(contact));
  }
  for (std::size_t node = 0; node < adaptive::v5::kBasisSize; ++node) {
    names.push_back("feature_" + std::to_string(node));
  }
  for (std::size_t axis = 0; axis < adaptive::v5::kAxes; ++axis) {
    for (std::size_t node = 0; node < adaptive::v5::kBasisSize; ++node) {
      names.push_back("weight_" + std::to_string(axis) + "_" +
                      std::to_string(node));
    }
  }
  if (v6) {
    for (const auto& group : {"gain_evidence", "gain_smoothed_evidence",
                              "gain_multiplier", "effective_gamma"}) {
      for (const auto& axis : suffix) names.push_back(std::string(group) + "_" + axis);
    }
  }
  return names;
}

}  // namespace

void LeggedController::configureMyRbfV5(ros::NodeHandle& nh) {
  adaptive::v5::Settings settings;
  nh.param("myrbf_shadow/v5_rbf_enabled", myRbfV5RbfEnabled_, true);
  settings.inputScale =
      arrayParameter(nh, "v5_input_scale", settings.inputScale);
  settings.historyScale =
      arrayParameter(nh, "v5_history_scale", settings.historyScale);
  settings.gamma = arrayParameter(nh, "v5_gamma", settings.gamma);
  settings.leakage = arrayParameter(nh, "v5_leakage", settings.leakage);
  settings.weightLimit =
      arrayParameter(nh, "v5_weight_limit", settings.weightLimit);
  settings.outputLimit =
      arrayParameter(nh, "v5_output_limit", settings.outputLimit);
  nh.param("myrbf_shadow/v5_center_extent", settings.centerExtent,
           settings.centerExtent);
  nh.param("myrbf_shadow/v5_motion_width", settings.motionWidth,
           settings.motionWidth);
  nh.param("myrbf_shadow/v5_history_width", settings.historyWidth,
           settings.historyWidth);
  nh.param("myrbf_shadow/v5_contact_width", settings.contactWidth,
           settings.contactWidth);
  nh.param("myrbf_shadow/v5_input_clip", settings.inputClip,
           settings.inputClip);
  nh.param("myrbf_shadow/v5_update_epsilon", settings.epsilon,
           settings.epsilon);
  if (myRbfV6Dynamics_) {
    settings.gainMultiplierMax = arrayParameter(
        nh, "v6_gain_multiplier_max", settings.gainMultiplierMax);
    settings.uncertaintyScale = arrayParameter(
        nh, "v6_uncertainty_scale", settings.uncertaintyScale);
    nh.param("myrbf_shadow/v6_gain_attack_tau", settings.gainAttackTau,
             settings.gainAttackTau);
    nh.param("myrbf_shadow/v6_gain_release_tau", settings.gainReleaseTau,
             settings.gainReleaseTau);
  }
  nh.param("myrbf_shadow/v5_residual_filter_tau",
           myRbfV5ResidualFilterTau_, myRbfV5ResidualFilterTau_);
  if (!std::isfinite(myRbfV5ResidualFilterTau_) ||
      myRbfV5ResidualFilterTau_ <= 0.0) {
    throw std::invalid_argument(
        "v5_residual_filter_tau must be finite and positive");
  }
  myRbfV5Core_ =
      std::make_unique<adaptive::v5::DynamicsCore>(std::move(settings));

  const auto names = v5Columns(myRbfV6Dynamics_);
  ros::NodeHandle node;
  const std::string prefix = myRbfV6Dynamics_ ? "/myrbf_v6_dynamics" :
                                             "/myrbf_v5_dynamics";
  node.setParam(prefix + "_columns", names);
  myRbfV5Publisher_ = std::make_unique<
      realtime_tools::RealtimePublisher<std_msgs::Float64MultiArray>>(
      node, prefix + "_diagnostics", 100);
  myRbfV5Publisher_->msg_.data.resize(names.size(), 0.0);
}

adaptive::vector6_t LeggedController::myRbfV5EquivalentWrench(
    const adaptive::v5::Axes& acceleration) const {
  if (!rbdConversions_ || measuredRbdState_.size() == 0) {
    throw std::runtime_error("V5 physical-state conversion is unavailable");
  }
  const auto& info = leggedInterface_->getCentroidalModelInfo();
  const double dt = std::max(myRbfV5PredictionDt_, myRbfShadowUpdatePeriod_);
  vector_t next = measuredRbdState_;
  for (std::size_t axis = 0; axis < 3; ++axis) {
    next(info.generalizedCoordinatesNum + 3 + axis) +=
        dt * acceleration[axis];
    next(info.generalizedCoordinatesNum + axis) +=
        dt * acceleration[3 + axis];
  }
  const vector_t currentCentroidal =
      rbdConversions_->computeCentroidalStateFromRbdModel(measuredRbdState_);
  const vector_t nextCentroidal =
      rbdConversions_->computeCentroidalStateFromRbdModel(next);
  const adaptive::vector6_t wrench =
      info.robotMass *
      (nextCentroidal.head<6>() - currentCentroidal.head<6>()) / dt;
  if (!wrench.allFinite()) {
    throw std::runtime_error("V5 acceleration-to-wrench conversion failed");
  }
  return wrench;
}

adaptive::v5::Axes LeggedController::myRbfV5EquivalentAcceleration(
    const adaptive::vector6_t& wrench) const {
  Eigen::Matrix<double, 6, 6> map;
  for (std::size_t axis = 0; axis < 6; ++axis) {
    adaptive::v5::Axes unit{};
    unit[axis] = 1.0;
    map.col(axis) = myRbfV5EquivalentWrench(unit);
  }
  const adaptive::vector6_t acceleration =
      map.fullPivLu().solve(wrench);
  if (!acceleration.allFinite() ||
      (map * acceleration - wrench).norm() >
          1.0e-6 * std::max(1.0, wrench.norm())) {
    throw std::runtime_error("V5 wrench-to-acceleration conversion failed");
  }
  return v5Axes(acceleration);
}

void LeggedController::queueMyRbfV5Prediction() {
  vector_t predictedState;
  vector_t predictedInput;
  std::size_t predictedMode = 0;
  mpcMrtInterface_->rolloutPolicy(
      currentObservation_.time, currentObservation_.state,
      myRbfShadowUpdatePeriod_, predictedState, predictedInput, predictedMode);
  const vector_t predictedRbd =
      rbdConversions_->computeRbdStateFromCentroidalModel(
          predictedState, predictedInput);
  const auto& info = leggedInterface_->getCentroidalModelInfo();
  myRbfV5BaseVelocity_ =
      physicalVelocity(measuredRbdState_, info.generalizedCoordinatesNum);
  const auto predictedVelocity =
      physicalVelocity(predictedRbd, info.generalizedCoordinatesNum);
  myRbfV5PredictionDt_ = myRbfShadowUpdatePeriod_;
  for (std::size_t axis = 0; axis < 6; ++axis) {
    myRbfV5NominalRate_[axis] =
        (predictedVelocity[axis] - myRbfV5BaseVelocity_[axis]) /
        myRbfV5PredictionDt_;
  }
  myRbfV5QueuedFeatures_.velocity = myRbfV5BaseVelocity_;
  myRbfV5QueuedFeatures_.nominalRate = myRbfV5NominalRate_;
  myRbfV5QueuedFeatures_.residualMemory = myRbfV5ResidualMemory_;
  myRbfV5QueuedFeatures_.residualTrend = myRbfV5ResidualTrend_;
  const contact_flag_t contactFlags = modeNumber2StanceLeg(predictedMode);
  for (std::size_t contact = 0; contact < contactFlags.size(); ++contact) {
    myRbfV5QueuedFeatures_.contacts[contact] =
        contactFlags[contact] ? 1.0 : 0.0;
  }
  myRbfV5KnownAppliedAcceleration_.fill(0.0);
  if (myRbfCompensationAwareObserver_ && myRbfAppliedWrench_.norm() > 0.0) {
    myRbfV5KnownAppliedAcceleration_ =
        myRbfV5EquivalentAcceleration(myRbfAppliedWrench_);
  }
  myRbfShadowPredictionTime_ =
      currentObservation_.time + myRbfV5PredictionDt_;
  myRbfShadowPredictionPending_ = true;
  myRbfV5PredictionValid_ = true;
}

void LeggedController::observeMyRbfV5() {
  if (!myRbfV5PredictionValid_) return;
  const auto& info = leggedInterface_->getCentroidalModelInfo();
  const auto actualVelocity =
      physicalVelocity(measuredRbdState_, info.generalizedCoordinatesNum);
  adaptive::v5::Axes adjustedObservedRate{};
  for (std::size_t axis = 0; axis < 6; ++axis) {
    myRbfV5ObservedRate_[axis] =
        (actualVelocity[axis] - myRbfV5BaseVelocity_[axis]) /
        myRbfV5PredictionDt_;
    adjustedObservedRate[axis] =
        myRbfV5ObservedRate_[axis] -
        myRbfV5KnownAppliedAcceleration_[axis];
    myRbfV5RawTargetResidual_[axis] =
        adjustedObservedRate[axis] - myRbfV5NominalRate_[axis];
  }
  adaptive::v5::Axes filteredResidual{};
  adaptive::v5::Axes residualTrend{};
  const double alpha = myRbfV5PredictionDt_ /
      (myRbfV5ResidualFilterTau_ + myRbfV5PredictionDt_);
  for (std::size_t axis = 0; axis < 6; ++axis) {
    filteredResidual[axis] = myRbfV5ResidualMemoryValid_
        ? myRbfV5ResidualMemory_[axis] +
              alpha * (myRbfV5RawTargetResidual_[axis] -
                       myRbfV5ResidualMemory_[axis])
        : myRbfV5RawTargetResidual_[axis];
    residualTrend[axis] = myRbfV5ResidualMemoryValid_
        ? (filteredResidual[axis] - myRbfV5ResidualMemory_[axis]) /
              myRbfV5PredictionDt_
        : 0.0;
  }
  auto result = myRbfV5Core_->update(
      myRbfV5QueuedFeatures_, filteredResidual,
      myRbfV5PredictionDt_, myRbfV5RbfEnabled_);
  myRbfV5ResidualMemory_ = filteredResidual;
  myRbfV5ResidualTrend_ = residualTrend;
  myRbfV5ResidualMemoryValid_ = true;
  adaptive::v5::Features currentFeatures = myRbfV5QueuedFeatures_;
  currentFeatures.velocity = actualVelocity;
  currentFeatures.residualMemory = myRbfV5ResidualMemory_;
  currentFeatures.residualTrend = myRbfV5ResidualTrend_;
  result.after = myRbfV5Core_->evaluate(currentFeatures, myRbfV5RbfEnabled_);
  myRbfV5Correction_ = result.after.correction;
  const adaptive::vector6_t externalWrench =
      myRbfV5EquivalentWrench(myRbfV5Correction_);
  myRbfAdaptiveWrench_ = -externalWrench;
  publishMyRbfV5Diagnostics(result, externalWrench);
  ++myRbfShadowValidSampleCount_;
  myRbfV5PredictionValid_ = false;
}

void LeggedController::publishMyRbfV5Diagnostics(
    const adaptive::v5::UpdateResult& result,
    const adaptive::vector6_t& externalWrenchEstimate) {
  ++myRbfV5DiagnosticSequence_;
  if (!myRbfV5Publisher_ || !myRbfV5Publisher_->trylock()) {
    ++myRbfV5DiagnosticDrops_;
    return;
  }
  auto& out = myRbfV5Publisher_->msg_.data;
  std::size_t index = 0;
  const auto add = [&](double value) { out.at(index++) = value; };
  add(currentObservation_.time);
  add(myRbfV5DiagnosticSequence_);
  add(myRbfV5DiagnosticDrops_);
  add(1.0);
  add(myRbfV5RbfEnabled_ ? 1.0 : 0.0);
  add(myRbfV5PredictionDt_);
  for (const auto& group :
       {myRbfV5BaseVelocity_, myRbfV5NominalRate_, myRbfV5ObservedRate_,
        myRbfV5KnownAppliedAcceleration_, myRbfV5RawTargetResidual_,
        result.targetResidual, myRbfV5ResidualMemory_,
        myRbfV5ResidualTrend_, result.predictionError,
        result.after.correction,
        result.after.totalRate}) {
    for (double value : group) add(value);
  }
  for (std::size_t axis = 0; axis < 6; ++axis) {
    add(externalWrenchEstimate(axis));
  }
  for (std::size_t axis = 0; axis < 6; ++axis) {
    add(-externalWrenchEstimate(axis));
  }
  for (double value : myRbfV5QueuedFeatures_.contacts) add(value);
  for (double value : result.after.basis) add(value);
  for (const auto& axisWeights : myRbfV5Core_->weights()) {
    for (double value : axisWeights) add(value);
  }
  if (myRbfV6Dynamics_) {
    for (const auto& group : {result.gainEvidence,
                              result.gainSmoothedEvidence,
                              result.gainMultiplier, result.effectiveGamma}) {
      for (double value : group) add(value);
    }
  }
  myRbfV5Publisher_->unlockAndPublish();
}

}  // namespace legged
