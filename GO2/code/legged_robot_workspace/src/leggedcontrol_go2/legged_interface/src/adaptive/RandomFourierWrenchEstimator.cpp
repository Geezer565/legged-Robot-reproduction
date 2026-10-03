#include "legged_interface/adaptive/RandomFourierWrenchEstimator.h"

#include <cmath>
#include <random>
#include <stdexcept>
#include <utility>

namespace legged {
namespace adaptive {

constexpr Eigen::Index RandomFourierWrenchEstimator::kAxisCount;
constexpr Eigen::Index RandomFourierWrenchEstimator::kInputDimension;

namespace {
void requirePositiveFinite(const vector6_t& values, const char* name) {
  if (!values.allFinite() || (values.array() <= 0.0).any()) {
    throw std::invalid_argument(std::string("[RandomFourierWrenchEstimator] ") + name + " must be finite and positive.");
  }
}
void requireNonnegativeFinite(const vector6_t& values, const char* name) {
  if (!values.allFinite() || (values.array() < 0.0).any()) {
    throw std::invalid_argument(std::string("[RandomFourierWrenchEstimator] ") + name + " must be finite and nonnegative.");
  }
}
}  // namespace

RandomFourierWrenchEstimator::RandomFourierWrenchEstimator(RandomFourierWrenchEstimatorSettings settings)
    : settings_(std::move(settings)) {
  validateSettings();
  frequencies_.resize(settings_.featureCount, kInputDimension);
  phases_.resize(settings_.featureCount);
  weights_ = matrix_t::Zero(settings_.featureCount, kAxisCount);
  std::mt19937 generator(settings_.randomSeed);
  std::normal_distribution<scalar_t> frequency(0.0, settings_.frequencyStd);
  std::uniform_real_distribution<scalar_t> phase(0.0, 2.0 * std::acos(-1.0));
  for (Eigen::Index node = 0; node < settings_.featureCount; ++node) {
    frequencies_(node, 0) = frequency(generator);
    frequencies_(node, 1) = frequency(generator);
    phases_(node) = phase(generator);
  }
}

void RandomFourierWrenchEstimator::validateSettings() const {
  requirePositiveFinite(settings_.residualScale, "residualScale");
  requirePositiveFinite(settings_.discrepancyScale, "discrepancyScale");
  requirePositiveFinite(settings_.outputScale, "outputScale");
  requireNonnegativeFinite(settings_.learningRate, "learningRate");
  if (settings_.featureCount <= 0 || settings_.featureCount > 10000) throw std::invalid_argument("[RandomFourierWrenchEstimator] featureCount must lie in [1, 10000].");
  if (!std::isfinite(settings_.frequencyStd) || settings_.frequencyStd <= 0.0) throw std::invalid_argument("[RandomFourierWrenchEstimator] frequencyStd must be finite and positive.");
  if (!std::isfinite(settings_.projectionRadius) || settings_.projectionRadius < 0.0) throw std::invalid_argument("[RandomFourierWrenchEstimator] projectionRadius must be finite and nonnegative.");
}

matrix_t RandomFourierWrenchEstimator::computeFeature(const vector6_t& residualFiltered) const {
  matrix_t feature(kInputDimension, kAxisCount);
  feature.row(0) = residualFiltered.cwiseQuotient(settings_.residualScale).transpose();
  feature.row(1) = (residualFiltered - previousEstimate_).cwiseQuotient(settings_.discrepancyScale).transpose();
  return feature;
}

matrix_t RandomFourierWrenchEstimator::computeBasis(const matrix_t& feature) const {
  if (feature.rows() != kInputDimension || feature.cols() != kAxisCount || !feature.allFinite()) throw std::invalid_argument("[RandomFourierWrenchEstimator] feature must be finite 2x6.");
  matrix_t basis(settings_.featureCount, kAxisCount);
  const scalar_t scale = 1.0 / std::sqrt(static_cast<scalar_t>(settings_.featureCount));
  for (Eigen::Index axis = 0; axis < kAxisCount; ++axis)
    for (Eigen::Index node = 0; node < settings_.featureCount; ++node)
      basis(node, axis) = scale * std::cos(frequencies_(node, 0) * feature(0, axis) + frequencies_(node, 1) * feature(1, axis) + phases_(node));
  return basis;
}

vector6_t RandomFourierWrenchEstimator::evaluate(const matrix_t& basis) const {
  if (basis.rows() != settings_.featureCount || basis.cols() != kAxisCount || !basis.allFinite()) throw std::invalid_argument("[RandomFourierWrenchEstimator] basis has invalid shape or values.");
  vector6_t output;
  for (Eigen::Index axis = 0; axis < kAxisCount; ++axis) output(axis) = settings_.outputScale(axis) * weights_.col(axis).dot(basis.col(axis));
  return output;
}

void RandomFourierWrenchEstimator::projectRows() {
  if (settings_.projectionRadius <= 0.0) return;
  for (Eigen::Index node = 0; node < weights_.rows(); ++node) {
    const scalar_t norm = weights_.row(node).norm();
    if (norm > settings_.projectionRadius) weights_.row(node) *= settings_.projectionRadius / norm;
  }
}

RandomFourierWrenchStepResult RandomFourierWrenchEstimator::step(const vector6_t& residualFiltered, scalar_t dt) {
  if (!residualFiltered.allFinite()) throw std::invalid_argument("[RandomFourierWrenchEstimator] residual must be finite.");
  if (!std::isfinite(dt) || dt < 0.0) throw std::invalid_argument("[RandomFourierWrenchEstimator] dt must be finite and nonnegative.");
  RandomFourierWrenchStepResult result;
  result.feature = computeFeature(residualFiltered);
  result.basis = computeBasis(result.feature);
  result.estimateBeforeUpdate = evaluate(result.basis);
  result.weightDerivative = matrix_t::Zero(settings_.featureCount, kAxisCount);
  if (dt > 0.0) {
    for (Eigen::Index axis = 0; axis < kAxisCount; ++axis) {
      const scalar_t normalizedError = (residualFiltered(axis) - result.estimateBeforeUpdate(axis)) / settings_.outputScale(axis);
      const vector_t update = 2.0 * settings_.learningRate(axis) * normalizedError * result.basis.col(axis);
      weights_.col(axis) += update;
      result.weightDerivative.col(axis) = update / dt;
    }
    projectRows();
    if (!weights_.allFinite()) throw std::overflow_error("[RandomFourierWrenchEstimator] update produced non-finite weights.");
  }
  result.weightsAfterUpdate = weights_;
  result.estimateAfterUpdate = evaluate(result.basis);
  previousEstimate_ = result.estimateAfterUpdate;
  return result;
}

void RandomFourierWrenchEstimator::reset() {
  weights_.setZero();
  previousEstimate_.setZero();
}

}  // namespace adaptive
}  // namespace legged
