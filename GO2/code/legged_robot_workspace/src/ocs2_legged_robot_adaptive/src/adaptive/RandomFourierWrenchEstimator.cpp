#include "ocs2_legged_robot_adaptive/adaptive/RandomFourierWrenchEstimator.h"

#include <cmath>
#include <random>
#include <stdexcept>
#include <utility>

namespace ocs2 { namespace legged_robot_adaptive { namespace adaptive {

RandomFourierWrenchEstimator::RandomFourierWrenchEstimator(
    RandomFourierWrenchEstimatorSettings settings)
    : settings_(std::move(settings)),
      input_(settings_.residualScale, settings_.discrepancyScale) {
  if (settings_.featureCount <= 0 || settings_.featureCount > 10000) {
    throw std::invalid_argument(
        "[RandomFourierWrenchEstimator] feature count must be in [1, 10000].");
  }
  if (!settings_.outputScale.allFinite() ||
      (settings_.outputScale.array() <= 0.0).any()) {
    throw std::invalid_argument(
        "[RandomFourierWrenchEstimator] output scales must be finite and positive.");
  }
  if (!settings_.learningRate.allFinite() ||
      (settings_.learningRate.array() < 0.0).any()) {
    throw std::invalid_argument(
        "[RandomFourierWrenchEstimator] learning rates must be finite and nonnegative.");
  }
  if (!std::isfinite(settings_.frequencyStd) ||
      settings_.frequencyStd <= 0.0) {
    throw std::invalid_argument(
        "[RandomFourierWrenchEstimator] frequency std must be finite and positive.");
  }
  if (!std::isfinite(settings_.projectionRadius) ||
      settings_.projectionRadius < 0.0) {
    throw std::invalid_argument(
        "[RandomFourierWrenchEstimator] projection radius must be finite and nonnegative.");
  }

  frequencies_.resize(settings_.featureCount, 2);
  phases_.resize(settings_.featureCount);
  weights_ = matrix_t::Zero(settings_.featureCount, 6);
  std::mt19937 generator(settings_.randomSeed);
  std::normal_distribution<scalar_t> frequency(0.0, settings_.frequencyStd);
  std::uniform_real_distribution<scalar_t> phase(
      0.0, 2.0 * std::acos(-1.0));
  for (Eigen::Index node = 0; node < settings_.featureCount; ++node) {
    frequencies_(node, 0) = frequency(generator);
    frequencies_(node, 1) = frequency(generator);
    phases_(node) = phase(generator);
  }
}

matrix_t RandomFourierWrenchEstimator::computeBasis(
    const matrix_t& feature) const {
  if (feature.rows() != 2 || feature.cols() != 6 || !feature.allFinite()) {
    throw std::invalid_argument(
        "[RandomFourierWrenchEstimator] feature must be finite 2x6.");
  }
  matrix_t basis(settings_.featureCount, 6);
  const scalar_t scale = 1.0 / std::sqrt(
      static_cast<scalar_t>(settings_.featureCount));
  for (Eigen::Index axis = 0; axis < 6; ++axis) {
    for (Eigen::Index node = 0; node < settings_.featureCount; ++node) {
      basis(node, axis) = scale * std::cos(
          frequencies_(node, 0) * feature(0, axis) +
          frequencies_(node, 1) * feature(1, axis) + phases_(node));
    }
  }
  return basis;
}

vector6_t RandomFourierWrenchEstimator::evaluate(
    const matrix_t& basis) const {
  if (basis.rows() != settings_.featureCount || basis.cols() != 6 ||
      !basis.allFinite()) {
    throw std::invalid_argument(
        "[RandomFourierWrenchEstimator] basis has invalid shape or values.");
  }
  vector6_t output;
  for (Eigen::Index axis = 0; axis < 6; ++axis) {
    output(axis) = settings_.outputScale(axis) *
        weights_.col(axis).dot(basis.col(axis));
  }
  return output;
}

void RandomFourierWrenchEstimator::projectRows() {
  if (settings_.projectionRadius <= 0.0) {
    return;
  }
  for (Eigen::Index node = 0; node < weights_.rows(); ++node) {
    const scalar_t norm = weights_.row(node).norm();
    if (norm > settings_.projectionRadius) {
      weights_.row(node) *= settings_.projectionRadius / norm;
    }
  }
}

RandomFourierWrenchStepResult RandomFourierWrenchEstimator::step(
    const vector6_t& residualFiltered, const vector6_t& sigma, scalar_t dt) {
  if (!residualFiltered.allFinite() || !sigma.allFinite()) {
    throw std::invalid_argument(
        "[RandomFourierWrenchEstimator] residual and sigma must be finite.");
  }
  if (!std::isfinite(dt) || dt < 0.0) {
    throw std::invalid_argument(
        "[RandomFourierWrenchEstimator] dt must be finite and nonnegative.");
  }

  RandomFourierWrenchStepResult result;
  result.feature = input_.compute(
      residualFiltered, previousWrenchEstimate_).feature;
  result.basis = computeBasis(result.feature);
  result.estimateBeforeUpdate = evaluate(result.basis);
  result.weightDerivative = matrix_t::Zero(settings_.featureCount, 6);

  if (dt > 0.0) {
    for (Eigen::Index axis = 0; axis < 6; ++axis) {
      const scalar_t normalizedError =
          (residualFiltered(axis) - result.estimateBeforeUpdate(axis)) /
          settings_.outputScale(axis);
      const vector_t weightUpdate =
          2.0 * settings_.learningRate(axis) * normalizedError *
          result.basis.col(axis);
      weights_.col(axis) += weightUpdate;
      result.weightDerivative.col(axis) = weightUpdate / dt;
    }
    projectRows();
    if (!weights_.allFinite()) {
      throw std::overflow_error(
          "[RandomFourierWrenchEstimator] online update produced non-finite weights.");
    }
  }

  result.weightsAfterUpdate = weights_;
  result.estimateAfterUpdate = evaluate(result.basis);
  previousWrenchEstimate_ = result.estimateAfterUpdate;
  return result;
}

}}}  // namespace ocs2::legged_robot_adaptive::adaptive
