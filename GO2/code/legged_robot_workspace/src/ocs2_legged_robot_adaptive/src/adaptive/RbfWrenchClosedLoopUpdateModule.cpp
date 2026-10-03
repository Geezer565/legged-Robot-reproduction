#include "ocs2_legged_robot_adaptive/adaptive/RbfWrenchClosedLoopUpdateModule.h"

#include <cmath>
#include <stdexcept>
#include <utility>

#include <boost/filesystem.hpp>
#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <ocs2_core/misc/LoadData.h>

namespace ocs2 { namespace legged_robot_adaptive { namespace adaptive {
namespace {

WrenchEstimatorMethod parseMethod(const std::string& value) {
  if (value == "minniti_rbf" || value == "current_rbf") {
    return WrenchEstimatorMethod::MinnitiRbf;
  }
  if (value == "composite_rbf") {
    return WrenchEstimatorMethod::CompositeRbf;
  }
  if (value == "rff_ogd" || value == "random_fourier_ogd") {
    return WrenchEstimatorMethod::RandomFourierOgd;
  }
  throw std::invalid_argument(
      "[RbfWrenchClosedLoopSettings] unknown method: " + value);
}

void validateNonnegative(const vector6_t& value, const char* name) {
  if (!value.allFinite() || (value.array() < 0.0).any()) {
    throw std::invalid_argument(
        std::string("[RbfWrenchClosedLoopSettings] ") + name +
        " must be finite and nonnegative.");
  }
}

}  // namespace

RbfWrenchClosedLoopSettings loadRbfWrenchClosedLoopSettings(
    const std::string& filename, const std::string& prefix) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(filename, pt);

  RbfWrenchClosedLoopSettings settings;
  settings.enabled = pt.get<bool>(prefix + ".enabled", false);
  if (!settings.enabled) {
    return settings;
  }

  settings.method = parseMethod(
      pt.get<std::string>(prefix + ".method", "minniti_rbf"));

  settings.enableResidualFilter =
      pt.get<bool>(prefix + ".enableResidualFilter", true);
  settings.residualFilterAlpha =
      pt.get<scalar_t>(prefix + ".residualFilterAlpha", 0.9);
  settings.observationTimeTolerance =
      pt.get<scalar_t>(prefix + ".observationTimeTolerance", 1.0e-6);

  boost::filesystem::path basisFile(
      pt.get<std::string>(prefix + ".basisFile"));
  if (basisFile.is_relative()) {
    basisFile = boost::filesystem::path(filename).parent_path() / basisFile;
  }
  const auto frozenBasis = loadTwoFeatureSixSubnetworkRbfWrenchSettings(
      basisFile.lexically_normal().string());
  if (settings.method == WrenchEstimatorMethod::MinnitiRbf) {
    settings.estimator.network = frozenBasis;
    loadData::loadEigenMatrix(filename, prefix + ".gamma",
                              settings.estimator.gamma);
    validateNonnegative(settings.estimator.gamma, "gamma");
  } else if (settings.method == WrenchEstimatorMethod::CompositeRbf) {
    settings.compositeEstimator.network = frozenBasis;
    loadData::loadEigenMatrix(filename, prefix + ".trackingGain",
                              settings.compositeEstimator.trackingGain);
    loadData::loadEigenMatrix(filename, prefix + ".predictionGain",
                              settings.compositeEstimator.predictionGain);
    loadData::loadEigenMatrix(filename, prefix + ".leakage",
                              settings.compositeEstimator.leakage);
    validateNonnegative(settings.compositeEstimator.trackingGain,
                        "trackingGain");
    validateNonnegative(settings.compositeEstimator.predictionGain,
                        "predictionGain");
    validateNonnegative(settings.compositeEstimator.leakage, "leakage");
  } else {
    settings.randomFourierEstimator.residualScale = frozenBasis.residualScale;
    settings.randomFourierEstimator.discrepancyScale =
        frozenBasis.discrepancyScale;
    settings.randomFourierEstimator.outputScale = frozenBasis.outputScale;
    settings.randomFourierEstimator.featureCount =
        pt.get<Eigen::Index>(prefix + ".featureCount", 50);
    settings.randomFourierEstimator.randomSeed =
        pt.get<std::uint32_t>(prefix + ".randomSeed", 20260824);
    settings.randomFourierEstimator.frequencyStd =
        pt.get<scalar_t>(prefix + ".frequencyStd", 1.0);
    settings.randomFourierEstimator.projectionRadius =
        pt.get<scalar_t>(prefix + ".projectionRadius", 0.0);
    loadData::loadEigenMatrix(filename, prefix + ".learningRate",
                              settings.randomFourierEstimator.learningRate);
    validateNonnegative(settings.randomFourierEstimator.learningRate,
                        "learningRate");
  }

  if (!std::isfinite(settings.residualFilterAlpha) ||
      settings.residualFilterAlpha < 0.0 ||
      settings.residualFilterAlpha >= 1.0) {
    throw std::invalid_argument(
        "[RbfWrenchClosedLoopSettings] residual filter alpha must be in [0, 1). ");
  }
  if (!std::isfinite(settings.observationTimeTolerance) ||
      settings.observationTimeTolerance < 0.0) {
    throw std::invalid_argument(
        "[RbfWrenchClosedLoopSettings] observation tolerance must be finite and nonnegative.");
  }
  return settings;
}

RbfWrenchClosedLoopUpdateModule::RbfWrenchClosedLoopUpdateModule(
    const PinocchioInterface& pinocchioInterface,
    CentroidalModelInfo centroidalModelInfo,
    AdaptiveClfSettings adaptiveSettings,
    RbfWrenchClosedLoopSettings rbfSettings,
    std::shared_ptr<AdaptiveModelData> modelData)
    : adaptiveSettings_(std::move(adaptiveSettings)),
      rbfSettings_(std::move(rbfSettings)),
      referenceKinematics_(pinocchioInterface, centroidalModelInfo,
                           adaptiveSettings_),
      slidingVariable_(adaptiveSettings_),
      residualEstimator_(pinocchioInterface, centroidalModelInfo,
                         rbfSettings_.enableResidualFilter,
                         rbfSettings_.residualFilterAlpha),
      modelData_(std::move(modelData)) {
  validateAdaptiveClfSettings(adaptiveSettings_);
  if (!rbfSettings_.enabled) {
    throw std::invalid_argument(
        "[RbfWrenchClosedLoopUpdateModule] closed-loop RBF must be enabled.");
  }
  if (adaptiveSettings_.parameterization !=
      AdaptiveParameterization::ConstantWrench6D) {
    throw std::invalid_argument(
        "[RbfWrenchClosedLoopUpdateModule] RBF output requires constant_wrench_6d semantics.");
  }
  if (!std::isfinite(rbfSettings_.observationTimeTolerance) ||
      rbfSettings_.observationTimeTolerance < 0.0) {
    throw std::invalid_argument(
        "[RbfWrenchClosedLoopUpdateModule] observation tolerance must be finite and nonnegative.");
  }
  if (!modelData_ || !modelData_->snapshot() ||
      modelData_->snapshot()->estimate().size() != 6) {
    throw std::invalid_argument(
        "[RbfWrenchClosedLoopUpdateModule] valid six-axis model data is required.");
  }
  snapshotVersion_ = modelData_->snapshot()->version();
  if (rbfSettings_.method == WrenchEstimatorMethod::MinnitiRbf) {
    rbfEstimator_ = std::make_unique<TwoFeatureMinnitiRbfWrenchEstimator>(
        rbfSettings_.estimator);
  } else if (rbfSettings_.method == WrenchEstimatorMethod::CompositeRbf) {
    compositeRbfEstimator_ = std::make_unique<CompositeRbfWrenchEstimator>(
        rbfSettings_.compositeEstimator);
  } else {
    randomFourierEstimator_ =
        std::make_unique<RandomFourierWrenchEstimator>(
            rbfSettings_.randomFourierEstimator);
  }
  diagnostics_.method = rbfSettings_.method;
}

void RbfWrenchClosedLoopUpdateModule::submitObservation(
    scalar_t time, const vector_t& state, const vector_t& input) {
  SubmittedObservation observation;
  observation.time = time;
  observation.state = state;
  observation.input = input;
  observation.valid = std::isfinite(time) && state.allFinite() && input.allFinite();
  std::lock_guard<std::mutex> lock(observationMutex_);
  submittedObservation_ = std::move(observation);
}

RbfWrenchClosedLoopDiagnostics
RbfWrenchClosedLoopUpdateModule::diagnostics() const {
  std::lock_guard<std::mutex> lock(diagnosticsMutex_);
  return diagnostics_;
}

void RbfWrenchClosedLoopUpdateModule::preSolverRun(
    scalar_t initTime, scalar_t finalTime, const vector_t& initState,
    const ReferenceManagerInterface& referenceManager) {
  static_cast<void>(finalTime);
  if (!std::isfinite(initTime)) {
    throw std::invalid_argument(
        "[RbfWrenchClosedLoopUpdateModule] initTime must be finite.");
  }
  if (std::isfinite(lastProcessedTime_)) {
    const scalar_t elapsed = initTime - lastProcessedTime_;
    if (elapsed < -adaptiveSettings_.timestampTolerance) {
      throw std::runtime_error(
          "[RbfWrenchClosedLoopUpdateModule] observation timestamp moved backward.");
    }
    if (std::abs(initTime - lastProcessedTime_) <=
        adaptiveSettings_.timestampTolerance) {
      return;
    }
  }

  SubmittedObservation observation;
  {
    std::lock_guard<std::mutex> lock(observationMutex_);
    observation = submittedObservation_;
  }
  const bool observationMatched = observation.valid &&
      std::abs(observation.time - initTime) <=
          rbfSettings_.observationTimeTolerance;

  const auto reference = referenceKinematics_.compute(
      initTime, initState, referenceManager.getTargetTrajectories());
  const auto sliding = slidingVariable_.compute(reference.actual, reference.desired);
  WrenchResidualResult residual;
  if (observationMatched) {
    residual = residualEstimator_.update(
        observation.time, observation.state, observation.input);
  }

  vector6_t estimateBeforeUpdate = vector6_t::Zero();
  if (rbfEstimator_) {
    estimateBeforeUpdate = rbfEstimator_->previousWrenchEstimate();
  } else if (compositeRbfEstimator_) {
    estimateBeforeUpdate = compositeRbfEstimator_->previousWrenchEstimate();
  } else {
    estimateBeforeUpdate = randomFourierEstimator_->previousWrenchEstimate();
  }
  vector6_t estimateAfterUpdate = estimateBeforeUpdate;
  matrix_t weightDerivative;
  bool weightsUpdated = false;
  if (residual.valid) {
    const scalar_t updateDt = adaptiveSettings_.enableParameterUpdate
                                  ? residual.dt
                                  : 0.0;
    if (rbfEstimator_) {
      const auto result = rbfEstimator_->step(
          residual.filteredResidual, sliding.sigma, updateDt);
      estimateBeforeUpdate = result.estimateBeforeUpdate;
      estimateAfterUpdate = result.estimateAfterUpdate;
      weightDerivative = result.weightDerivative;
    } else if (compositeRbfEstimator_) {
      const auto result = compositeRbfEstimator_->step(
          residual.filteredResidual, sliding.sigma, updateDt);
      estimateBeforeUpdate = result.estimateBeforeUpdate;
      estimateAfterUpdate = result.estimateAfterUpdate;
      weightDerivative = result.weightDerivative;
    } else {
      const auto result = randomFourierEstimator_->step(
          residual.filteredResidual, sliding.sigma, updateDt);
      estimateBeforeUpdate = result.estimateBeforeUpdate;
      estimateAfterUpdate = result.estimateAfterUpdate;
      weightDerivative = result.weightDerivative;
    }
    weightsUpdated = adaptiveSettings_.enableParameterUpdate;
  }

  ++snapshotVersion_;
  const auto snapshot = AdaptiveModelSnapshot::createValid(
      estimateAfterUpdate, initTime, snapshotVersion_);
  modelData_->publish(snapshot);
  lastProcessedTime_ = initTime;

  RbfWrenchClosedLoopDiagnostics diagnostics;
  diagnostics.method = rbfSettings_.method;
  diagnostics.valid = true;
  diagnostics.observationMatched = observationMatched;
  diagnostics.residualValid = residual.valid;
  diagnostics.parameterUpdateEnabled = adaptiveSettings_.enableParameterUpdate;
  diagnostics.weightsUpdated = weightsUpdated;
  diagnostics.observationTime = initTime;
  diagnostics.snapshotTime = snapshot->time();
  diagnostics.residualDt = residual.dt;
  diagnostics.snapshotVersion = snapshot->version();
  diagnostics.rawResidual = residual.rawResidual;
  diagnostics.filteredResidual = residual.filteredResidual;
  diagnostics.sigma = sliding.sigma;
  diagnostics.estimateBeforeUpdate = estimateBeforeUpdate;
  diagnostics.estimateAfterUpdate = estimateAfterUpdate;
  if (rbfEstimator_) {
    diagnostics.weightNorm = rbfEstimator_->weights().norm();
  } else if (compositeRbfEstimator_) {
    diagnostics.weightNorm = compositeRbfEstimator_->weights().norm();
  } else {
    diagnostics.weightNorm = randomFourierEstimator_->weights().norm();
  }
  diagnostics.weightDerivativeNorm = weightDerivative.size() > 0
                                         ? weightDerivative.norm()
                                         : 0.0;
  {
    std::lock_guard<std::mutex> lock(diagnosticsMutex_);
    diagnostics_ = std::move(diagnostics);
  }
}

}}}  // namespace ocs2::legged_robot_adaptive::adaptive
