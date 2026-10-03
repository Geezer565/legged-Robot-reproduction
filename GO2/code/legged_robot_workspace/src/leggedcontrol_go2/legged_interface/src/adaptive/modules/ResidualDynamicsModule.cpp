#include "legged_interface/adaptive/modules/ResidualDynamicsModule.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

namespace legged {
namespace adaptive {
namespace modules {
namespace {

void requirePositiveFinite(const vector6_t& value, const char* name) {
  if (!value.allFinite() || (value.array() <= 0.0).any()) {
    throw std::invalid_argument(std::string("[ResidualDynamicsModule] ") + name +
                                " must be finite and positive.");
  }
}

}  // namespace

ResidualDynamicsModule::ResidualDynamicsModule(ResidualDynamicsSettings settings)
    : settings_(std::move(settings)) {
  validateSettings();
}

void ResidualDynamicsModule::validateSettings() const {
  requirePositiveFinite(settings_.learningRate, "learningRate");
  requirePositiveFinite(settings_.parameterLimit, "parameterLimit");
  requirePositiveFinite(settings_.residualScale, "residualScale");
  requirePositiveFinite(settings_.trackingScale, "trackingScale");
  if (!std::isfinite(settings_.normalizationEpsilon) || settings_.normalizationEpsilon <= 0.0) {
    throw std::invalid_argument("[ResidualDynamicsModule] normalizationEpsilon must be finite and positive.");
  }
  if (settings_.confidenceRiseSamples == 0) {
    throw std::invalid_argument("[ResidualDynamicsModule] confidenceRiseSamples must be positive.");
  }
}

Eigen::Matrix<scalar_t, 3, 1> ResidualDynamicsModule::feature(
    Eigen::Index axis, const vector6_t& residualForLag,
    const vector6_t& trackingSignal) const {
  Eigen::Matrix<scalar_t, 3, 1> result;
  result << 1.0, residualForLag[axis] / settings_.residualScale[axis],
      trackingSignal[axis] / settings_.trackingScale[axis];
  return result;
}

vector6_t ResidualDynamicsModule::evaluate(const vector6_t& residualForLag,
                                            const vector6_t& trackingSignal) const {
  vector6_t result;
  for (Eigen::Index axis = 0; axis < 6; ++axis) {
    result[axis] = weights_.col(axis).dot(feature(axis, residualForLag, trackingSignal));
  }
  return result;
}

void ResidualDynamicsModule::projectAxis(Eigen::Index axis) {
  const scalar_t norm = weights_.col(axis).norm();
  if (norm > settings_.parameterLimit[axis]) {
    weights_.col(axis) *= settings_.parameterLimit[axis] / norm;
  }
}

ResidualDynamicsResult ResidualDynamicsModule::step(const vector6_t& residual,
                                                     const vector6_t& trackingSignal,
                                                     scalar_t dt, bool enableLearning) {
  if (!residual.allFinite() || !trackingSignal.allFinite()) {
    throw std::invalid_argument("[ResidualDynamicsModule] inputs must be finite.");
  }
  if (!std::isfinite(dt) || dt <= 0.0) {
    throw std::invalid_argument("[ResidualDynamicsModule] dt must be finite and positive.");
  }

  ResidualDynamicsResult result;
  result.predictedCurrent = evaluate(previousResidual_, trackingSignal);
  result.innovation = residual - result.predictedCurrent;
  result.learningEnabled = enableLearning;

  if (enableLearning) {
    for (Eigen::Index axis = 0; axis < 6; ++axis) {
      const auto phi = feature(axis, previousResidual_, trackingSignal);
      const scalar_t denominator = settings_.normalizationEpsilon + phi.squaredNorm();
      weights_.col(axis).noalias() +=
          settings_.learningRate[axis] * result.innovation[axis] * phi / denominator;
      projectAxis(axis);
      ++sampleCount_[static_cast<std::size_t>(axis)];
    }
  }

  if (!weights_.allFinite()) {
    throw std::overflow_error("[ResidualDynamicsModule] update produced non-finite weights.");
  }

  result.predictedAfterUpdate = evaluate(previousResidual_, trackingSignal);
  result.predictedNext = evaluate(residual, trackingSignal);
  for (Eigen::Index axis = 0; axis < 6; ++axis) {
    const unsigned samples = sampleCount_[static_cast<std::size_t>(axis)];
    if (samples > settings_.minimumSamplesForConfidence) {
      result.confidence[axis] = std::min<scalar_t>(
          1.0, static_cast<scalar_t>(samples - settings_.minimumSamplesForConfidence) /
                   static_cast<scalar_t>(settings_.confidenceRiseSamples));
    }
  }
  previousResidual_ = residual;
  return result;
}

void ResidualDynamicsModule::reset() {
  weights_.setZero();
  previousResidual_.setZero();
  sampleCount_.fill(0);
}

}  // namespace modules
}  // namespace adaptive
}  // namespace legged

