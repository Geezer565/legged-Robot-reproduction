#include "legged_interface/adaptive/ComparisonWrenchEstimator.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace legged {
namespace adaptive {

constexpr Eigen::Index ComparisonWrenchEstimator::kAxisCount;
constexpr Eigen::Index ComparisonWrenchEstimator::kLiuGaussianCount;
constexpr Eigen::Index ComparisonWrenchEstimator::kLiuBasisCount;

namespace {

void requirePositive(const vector6_t& value, const char* name) {
  if (!value.allFinite() || (value.array() <= 0.0).any()) {
    throw std::invalid_argument(std::string("[ComparisonWrenchEstimator] ") +
                                name + " must be finite and positive.");
  }
}

}  // namespace

ComparisonWrenchEstimator::ComparisonWrenchEstimator(
    ComparisonWrenchEstimatorSettings settings)
    : settings_(std::move(settings)),
      covariance_(vector6_t::Constant(settings_.neuralFlyInitialCovariance)) {
  validateSettings();
}

void ComparisonWrenchEstimator::validateSettings() const {
  requirePositive(settings_.residualScale, "residualScale");
  requirePositive(settings_.trackingScale, "trackingScale");
  requirePositive(settings_.outputScale, "outputScale");
  requirePositive(settings_.adaptationGain, "adaptationGain");
  requirePositive(settings_.weightLimit, "weightLimit");
  if (!std::isfinite(settings_.gaussianWidth) ||
      settings_.gaussianWidth <= 0.0 ||
      !std::isfinite(settings_.compositeRatio) ||
      settings_.compositeRatio <= 0.5 || settings_.compositeRatio >= 1.0 ||
      !std::isfinite(settings_.minimumStep) ||
      !std::isfinite(settings_.maximumStep) ||
      settings_.minimumStep <= 0.0 ||
      settings_.maximumStep < settings_.minimumStep ||
      !std::isfinite(settings_.neuralFlyLambda) ||
      settings_.neuralFlyLambda < 0.0 ||
      !std::isfinite(settings_.neuralFlyProcessNoise) ||
      settings_.neuralFlyProcessNoise < 0.0 ||
      !std::isfinite(settings_.neuralFlyMeasurementNoise) ||
      settings_.neuralFlyMeasurementNoise <= 0.0 ||
      !std::isfinite(settings_.neuralFlyInitialCovariance) ||
      settings_.neuralFlyInitialCovariance <= 0.0) {
    throw std::invalid_argument(
        "[ComparisonWrenchEstimator] invalid method-specific settings.");
  }
}

void ComparisonWrenchEstimator::projectVector(vector6_t& value) const {
  for (Eigen::Index axis = 0; axis < kAxisCount; ++axis) {
    value(axis) = std::max(-settings_.weightLimit(axis),
                           std::min(settings_.weightLimit(axis), value(axis)));
  }
}

void ComparisonWrenchEstimator::projectColumns(matrix_t& value) const {
  for (Eigen::Index axis = 0; axis < kAxisCount; ++axis) {
    const scalar_t norm = value.col(axis).norm();
    if (norm > settings_.weightLimit(axis)) {
      value.col(axis) *= settings_.weightLimit(axis) / norm;
    }
  }
}

ComparisonWrenchStepResult ComparisonWrenchEstimator::step(
    const vector6_t& residualFiltered, const vector6_t& trackingError,
    scalar_t dt) {
  if (!residualFiltered.allFinite() || !trackingError.allFinite() ||
      !std::isfinite(dt) || dt < 0.0) {
    throw std::invalid_argument(
        "[ComparisonWrenchEstimator] samples must be finite and dt nonnegative.");
  }
  switch (settings_.method) {
    case ComparisonMethod::EthDirect:
      return stepEth(trackingError, dt);
    case ComparisonMethod::LiuDualRbfPort:
      return stepLiu(residualFiltered, trackingError, dt);
    case ComparisonMethod::NeuralFlyConstant:
      return stepNeuralFly(residualFiltered, trackingError, dt);
  }
  throw std::logic_error("[ComparisonWrenchEstimator] unknown method.");
}

ComparisonWrenchStepResult ComparisonWrenchEstimator::stepEth(
    const vector6_t& trackingError, scalar_t dt) {
  ComparisonWrenchStepResult result;
  result.representation = ComparisonRepresentation::DirectParameter;
  result.feature =
      trackingError.cwiseQuotient(settings_.trackingScale).transpose();
  result.basis = matrix_t::Ones(1, kAxisCount);
  result.compensationBeforeUpdate = compensationEstimate_;
  const vector6_t derivative =
      settings_.adaptationGain.cwiseProduct(trackingError);
  result.weightDerivative = derivative.transpose();
  compensationEstimate_ += dt * derivative;
  projectVector(compensationEstimate_);
  result.weightsAfterUpdate = compensationEstimate_.transpose();
  result.compensationAfterUpdate = compensationEstimate_;
  result.externalEstimateAfterUpdate = -compensationEstimate_;
  result.effectiveStep = settings_.adaptationGain;
  return result;
}

matrix_t ComparisonWrenchEstimator::liuBasis(
    const vector6_t& residualFiltered,
    const vector6_t& trackingError) const {
  matrix_t basis(kLiuBasisCount, kAxisCount);
  basis.row(0).setOnes();
  for (Eigen::Index axis = 0; axis < kAxisCount; ++axis) {
    const scalar_t residual =
        residualFiltered(axis) / settings_.residualScale(axis);
    const scalar_t tracking =
        trackingError(axis) / settings_.trackingScale(axis);
    for (Eigen::Index node = 0; node < kLiuGaussianCount; ++node) {
      const scalar_t center =
          -1.0 + 2.0 * static_cast<scalar_t>(node) /
                     static_cast<scalar_t>(kLiuGaussianCount - 1);
      const scalar_t dr = residual - center;
      const scalar_t dt = tracking - center;
      basis(node + 1, axis) =
          std::exp(-0.5 * (dr * dr + dt * dt) /
                   (settings_.gaussianWidth * settings_.gaussianWidth));
    }
    const scalar_t norm = basis.col(axis).norm();
    if (norm > 0.0) basis.col(axis) /= norm;
  }
  return basis;
}

ComparisonWrenchStepResult ComparisonWrenchEstimator::stepLiu(
    const vector6_t& residualFiltered, const vector6_t& trackingError,
    scalar_t dt) {
  ComparisonWrenchStepResult result;
  result.representation = ComparisonRepresentation::DualGaussianRbf;
  result.feature.resize(2, kAxisCount);
  result.feature.row(0) =
      residualFiltered.cwiseQuotient(settings_.residualScale).transpose();
  result.feature.row(1) =
      trackingError.cwiseQuotient(settings_.trackingScale).transpose();
  result.basis = liuBasis(residualFiltered, trackingError);
  result.compensationBeforeUpdate = compensationEstimate_;
  const vector6_t predictionError =
      residualFiltered - compensationEstimate_;
  result.compositeError =
      settings_.compositeRatio * predictionError +
      (1.0 - settings_.compositeRatio) * trackingError;
  result.weightDerivative =
      matrix_t::Zero(2 * kLiuBasisCount, kAxisCount);
  for (Eigen::Index axis = 0; axis < kAxisCount; ++axis) {
    const scalar_t normalizedMagnitude = std::min(
        1.0, std::abs(residualFiltered(axis) / settings_.residualScale(axis)));
    const scalar_t step =
        settings_.minimumStep +
        (settings_.maximumStep - settings_.minimumStep) *
            normalizedMagnitude * normalizedMagnitude;
    result.effectiveStep(axis) = step;
    const vector_t predictionDerivative =
        settings_.adaptationGain(axis) * step *
        settings_.compositeRatio * result.compositeError(axis) *
        result.basis.col(axis);
    const vector_t trackingDerivative =
        settings_.adaptationGain(axis) * step *
        (1.0 - settings_.compositeRatio) * result.compositeError(axis) *
        result.basis.col(axis);
    liuPredictionWeights_.col(axis) += dt * predictionDerivative;
    liuTrackingWeights_.col(axis) += dt * trackingDerivative;
    result.weightDerivative.block(0, axis, kLiuBasisCount, 1) =
        predictionDerivative;
    result.weightDerivative.block(kLiuBasisCount, axis, kLiuBasisCount, 1) =
        trackingDerivative;
  }
  projectColumns(liuPredictionWeights_);
  projectColumns(liuTrackingWeights_);
  for (Eigen::Index axis = 0; axis < kAxisCount; ++axis) {
    compensationEstimate_(axis) =
        settings_.outputScale(axis) *
        (liuPredictionWeights_.col(axis) + liuTrackingWeights_.col(axis))
            .dot(result.basis.col(axis));
  }
  result.weightsAfterUpdate.resize(2 * kLiuBasisCount, kAxisCount);
  result.weightsAfterUpdate.topRows(kLiuBasisCount) = liuPredictionWeights_;
  result.weightsAfterUpdate.bottomRows(kLiuBasisCount) = liuTrackingWeights_;
  result.compensationAfterUpdate = compensationEstimate_;
  result.externalEstimateAfterUpdate = -compensationEstimate_;
  return result;
}

ComparisonWrenchStepResult ComparisonWrenchEstimator::stepNeuralFly(
    const vector6_t& residualFiltered, const vector6_t& trackingError,
    scalar_t dt) {
  ComparisonWrenchStepResult result;
  result.representation = ComparisonRepresentation::ConstantFeature;
  result.feature = matrix_t::Ones(1, kAxisCount);
  result.basis = result.feature;
  result.compensationBeforeUpdate = compensationEstimate_;
  vector6_t derivative = vector6_t::Zero();
  for (Eigen::Index axis = 0; axis < kAxisCount; ++axis) {
    const scalar_t p = covariance_(axis);
    derivative(axis) =
        -settings_.neuralFlyLambda * compensationEstimate_(axis) -
        p / settings_.neuralFlyMeasurementNoise *
            (compensationEstimate_(axis) - residualFiltered(axis)) +
        p * trackingError(axis);
    const scalar_t pDerivative =
        -2.0 * settings_.neuralFlyLambda * p +
        settings_.neuralFlyProcessNoise -
        p * p / settings_.neuralFlyMeasurementNoise;
    covariance_(axis) = std::max(1e-9, p + dt * pDerivative);
  }
  compensationEstimate_ += dt *
      settings_.adaptationGain.cwiseProduct(derivative);
  projectVector(compensationEstimate_);
  result.weightDerivative = derivative.transpose();
  result.weightsAfterUpdate = compensationEstimate_.transpose();
  result.compensationAfterUpdate = compensationEstimate_;
  result.externalEstimateAfterUpdate = -compensationEstimate_;
  result.covarianceAfterUpdate = covariance_;
  result.effectiveStep = settings_.adaptationGain;
  return result;
}

void ComparisonWrenchEstimator::reset() {
  compensationEstimate_.setZero();
  covariance_.setConstant(settings_.neuralFlyInitialCovariance);
  liuPredictionWeights_.setZero();
  liuTrackingWeights_.setZero();
}

}  // namespace adaptive
}  // namespace legged
