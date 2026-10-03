#include "legged_interface/adaptive/MyRbfV2Pipeline.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace legged {
namespace adaptive {
namespace {

v2::Axes toAxes(const vector6_t& value) {
  v2::Axes result{};
  for (Eigen::Index axis = 0; axis < 6; ++axis) {
    result[static_cast<std::size_t>(axis)] = value(axis);
  }
  return result;
}

vector6_t toVector6(const v2::Axes& value) {
  vector6_t result;
  for (Eigen::Index axis = 0; axis < 6; ++axis) {
    result(axis) = value[static_cast<std::size_t>(axis)];
  }
  return result;
}

}  // namespace

MyRbfV2Pipeline::MyRbfV2Pipeline(
    scalar_t robotMass, Eigen::Index stateDimension, bool enableFilter,
    scalar_t filterAlpha, scalar_t timeTolerance, v2::Settings settings)
    : residualObserver_(robotMass, stateDimension, enableFilter, filterAlpha,
                        timeTolerance),
      core_(std::move(settings)),
      externalResidualFilterEnabled_(enableFilter),
      externalResidualFilterAlpha_(filterAlpha) {}

void MyRbfV2Pipeline::setPrediction(
    scalar_t predictedTime, scalar_t intervalDt,
    const vector_t& predictedState,
    const vector6_t& knownAppliedCompensation) {
  residualObserver_.setPrediction(predictedTime, intervalDt, predictedState,
                                  knownAppliedCompensation);
}

MyRbfV2PipelineResult MyRbfV2Pipeline::observeActual(
    scalar_t actualTime, const vector_t& actualState, const vector6_t& chi) {
  v2::FeatureInput input;
  input.chi = toAxes(chi);
  return observeActual(actualTime, actualState, input);
}

MyRbfV2PipelineResult MyRbfV2Pipeline::observeActual(
    scalar_t actualTime, const vector_t& actualState,
    const v2::FeatureInput& featureInput) {
  MyRbfV2PipelineResult result;
  result.chi = toVector6(featureInput.chi);
  result.residual = residualObserver_.observe(actualTime, actualState);
  if (!result.residual.valid) {
    return result;
  }
  auto input = featureInput;
  input.causalResidual = toAxes(result.residual.filteredResidual);
  return advance(actualTime, result.residual.dt, input, std::move(result));
}

MyRbfV2PipelineResult MyRbfV2Pipeline::advanceExternalResidual(
    scalar_t actualTime, scalar_t dt, const vector6_t& externalResidualBase,
    const v2::FeatureInput& featureInput) {
  if (!externalResidualBase.allFinite() || !std::isfinite(dt) || dt <= 0.0 ||
      !std::isfinite(actualTime) || (hasEvaluation_ && actualTime <= lastTime_)) {
    throw std::invalid_argument("[MyRBF V2 pipeline] invalid external residual, dt, or non-increasing time.");
  }
  vector6_t modelResidual = externalResidualBase;
  if (core_.settings().updateLaw == v2::UpdateLaw::TeacherComposite) {
    modelResidual = -externalResidualBase;
  } else if (core_.settings().updateLaw ==
             v2::UpdateLaw::ResidualConvergence) {
    // The calibration is learned only from an independent no-force run.  No
    // applied-force command or ground-truth disturbance enters this path.
    for (Eigen::Index axis = 0; axis < 6; ++axis) {
      const auto index = static_cast<std::size_t>(axis);
      const double centered =
          externalResidualBase(axis) - core_.settings().residualBias[index];
      const double magnitude = std::max(
          std::abs(centered) - core_.settings().residualDeadzone[index], 0.0);
      // A continuous soft deadzone avoids injecting a threshold-sized step
      // when gait/model residuals first leave the calibrated noise envelope.
      modelResidual(axis) = std::copysign(magnitude, centered);
    }
  }
  vector6_t filteredCandidate = modelResidual;
  if (externalResidualFilterEnabled_ && externalResidualFilterInitialized_) {
    filteredCandidate =
        externalResidualFilterAlpha_ * filteredExternalResidual_ +
        (1.0 - externalResidualFilterAlpha_) * modelResidual;
  }

  MyRbfV2PipelineResult result;
  result.residual.valid = true;
  result.residual.dt = dt;
  result.residual.netResidual = modelResidual;
  result.residual.rawResidual = modelResidual;
  result.residual.filteredResidual = filteredCandidate;
  auto input = featureInput;
  input.causalResidual = toAxes(filteredCandidate);
  auto advanced = advance(actualTime, dt, input, std::move(result));
  // Commit the filter memory only after the complete adaptive update succeeds.
  // An invalid feature frame must not bias the next valid control frame.
  filteredExternalResidual_ = filteredCandidate;
  externalResidualFilterInitialized_ = true;
  return advanced;
}

MyRbfV2PipelineResult MyRbfV2Pipeline::advance(
    scalar_t actualTime, scalar_t dt, const v2::FeatureInput& input,
    MyRbfV2PipelineResult result) {
  result.chi = toVector6(input.chi);
  result.featureInput = input;
  if (!result.chi.allFinite() || !std::isfinite(actualTime) ||
      !std::isfinite(dt) || dt <= 0.0) {
    throw std::invalid_argument(
        "[MyRBF V2 pipeline] chi and actual time must be finite.");
  }
  const scalar_t elapsedTime = timeInitialized_ ? actualTime - startTime_ : 0.0;
  if (!std::isfinite(elapsedTime) || elapsedTime < 0.0) {
    throw std::invalid_argument(
        "[MyRBF V2 pipeline] observation time moved backwards.");
  }

  auto nextHistory = history_;
  auto nextHistoryGradientIntegral = historyGradientIntegral_;
  auto nextHistoryLabelIntegral = historyLabelIntegral_;
  scalar_t nextHistoryWindowElapsed = historyWindowElapsed_;
  bool historySampleAdded = false;
  if (core_.settings().updateLaw == v2::UpdateLaw::TeacherComposite) {
    const auto currentGradient = core_.physicalGradient(input);
    for (std::size_t axis = 0; axis < 6; ++axis) {
      nextHistoryLabelIntegral[axis] += dt * input.causalResidual[axis];
      for (std::size_t basis = 0; basis < 26; ++basis) {
        nextHistoryGradientIntegral[axis][basis] +=
            dt * currentGradient[axis][basis];
      }
    }
    nextHistoryWindowElapsed += dt;
    if (nextHistoryWindowElapsed >= core_.settings().compositeWindow) {
      v2::Sample sample;
      sample.certified = true;
      for (std::size_t axis = 0; axis < 6; ++axis) {
        sample.importance[axis] = 1.0;
        sample.label[axis] = nextHistoryLabelIntegral[axis] /
            nextHistoryWindowElapsed;
        for (std::size_t basis = 0; basis < 26; ++basis) {
          sample.physicalGradient[axis][basis] =
              nextHistoryGradientIntegral[axis][basis] /
              nextHistoryWindowElapsed;
        }
      }
      if (nextHistory.size() == core_.settings().maxHistorySamples)
        nextHistory.erase(nextHistory.begin());
      nextHistory.push_back(sample);
      nextHistoryGradientIntegral = v2::Weights{};
      nextHistoryLabelIntegral = v2::Axes{};
      nextHistoryWindowElapsed = 0.0;
      historySampleAdded = true;
    }
  }

  const auto before = core_.evaluate(elapsedTime, state_, input, nextHistory);
  result.evaluationBeforeUpdate = before;
  result.estimateBeforeUpdate = toVector6(before.estimate);

  auto nextState = state_;
  const auto& settings = core_.settings();
  for (std::size_t axis = 0; axis < 6; ++axis) {
    scalar_t derivativeSquaredNorm = 0.0;
    bool projected = false;
    for (std::size_t basis = 0; basis < 26; ++basis) {
      const scalar_t derivative = before.weightDerivative[axis][basis];
      derivativeSquaredNorm += derivative * derivative;
      const scalar_t candidate =
          state_.weights[axis][basis] + dt * derivative;
      if (!std::isfinite(candidate))
        throw std::overflow_error("[MyRBF V2 pipeline] weight integration overflow.");
      const scalar_t limited = std::max(
          -settings.weightLimit[axis],
          std::min(settings.weightLimit[axis], candidate));
      projected = projected || limited != candidate;
      nextState.weights[axis][basis] = limited;
    }
    nextState.robustBound[axis] = std::max(
        0.0, state_.robustBound[axis] + dt * before.boundDerivative[axis]);
    result.weightDerivativeNorm(static_cast<Eigen::Index>(axis)) =
        std::sqrt(derivativeSquaredNorm);
    result.projectionActive[axis] = projected;
  }

  const auto after = core_.evaluate(elapsedTime, nextState, input, nextHistory);
  result.evaluationAfterUpdate = after;
  result.estimateAfterUpdate = toVector6(after.estimate);
  result.robustWrench = toVector6(after.robustWrench);
  result.robustBound = toVector6(nextState.robustBound);
  result.smoothingWidth = toVector6(after.smoothingWidth);
  result.coverage = toVector6(after.coverage);
  result.externalWrenchEstimate = toVector6(after.externalWrenchEstimate);
  result.leftHandDisturbanceEstimate = toVector6(after.leftHandDisturbanceEstimate);
  result.requiredControlCorrection = toVector6(after.requiredControlCorrection);
  if (!result.requiredControlCorrection.allFinite() || !result.weightDerivativeNorm.allFinite()) {
    throw std::overflow_error(
        "[MyRBF V2 pipeline] compensation wrench is not finite.");
  }
  // Commit only after all calculations succeeded; failed inputs cannot publish
  // a partially integrated state/evaluation pair to a snapshot consumer.
  state_ = nextState;
  evaluation_ = after;
  featureInput_ = input;
  compensationWrench_ = result.requiredControlCorrection;
  hasEvaluation_ = true;
  lastTime_ = actualTime;
  history_ = std::move(nextHistory);
  historyGradientIntegral_ = nextHistoryGradientIntegral;
  historyLabelIntegral_ = nextHistoryLabelIntegral;
  historyWindowElapsed_ = nextHistoryWindowElapsed;
  result.historySize = static_cast<unsigned>(history_.size());
  result.historySampleAdded = historySampleAdded;
  if (!timeInitialized_) { startTime_ = actualTime; timeInitialized_ = true; }
  result.valid = true;
  return result;
}

void MyRbfV2Pipeline::reset() {
  residualObserver_.reset();
  state_ = v2::State{};
  externalResidualFilterInitialized_ = false;
  filteredExternalResidual_.setZero();
  timeInitialized_ = false;
  startTime_ = 0.0;
  compensationWrench_.setZero();
  evaluation_ = v2::Evaluation{};
  featureInput_ = v2::FeatureInput{};
  hasEvaluation_ = false;
  lastTime_ = 0.0;
  history_.clear();
  historyGradientIntegral_ = v2::Weights{};
  historyLabelIntegral_ = v2::Axes{};
  historyWindowElapsed_ = 0.0;
}

}  // namespace adaptive
}  // namespace legged
