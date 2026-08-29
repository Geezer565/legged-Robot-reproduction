#include "legged_interface/adaptive/MyRbfShadowEstimator.h"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace legged {
namespace adaptive {

constexpr Eigen::Index MyRbfShadowEstimator::kAxisCount;
constexpr Eigen::Index MyRbfShadowEstimator::kInputDimension;
constexpr Eigen::Index MyRbfShadowEstimator::kGaussianNodeCount;
constexpr Eigen::Index MyRbfShadowEstimator::kBasisCount;
constexpr bool MyRbfShadowEstimator::kControlInjectionEnabled;

namespace {

void requirePositiveFinite(const vector6_t& values, const char* name) {
  if (!values.allFinite() || (values.array() <= 0.0).any()) {
    throw std::invalid_argument(std::string("[MyRbfShadowEstimator] ") + name +
                                " must be finite and positive.");
  }
}

void requireNonnegativeFinite(const vector6_t& values, const char* name) {
  if (!values.allFinite() || (values.array() < 0.0).any()) {
    throw std::invalid_argument(std::string("[MyRbfShadowEstimator] ") + name +
                                " must be finite and nonnegative.");
  }
}

}  // namespace

MyRbfShadowSettings createSymmetricGridMyRbfShadowSettings(
    scalar_t linearExtent, scalar_t angularExtent, scalar_t widthRatio,
    const vector6_t& residualScale, const vector6_t& discrepancyScale,
    const vector6_t& outputScale, const vector6_t& trackingGain,
    const vector6_t& predictionGain, const vector6_t& leakage) {
  if (!std::isfinite(linearExtent) || !std::isfinite(angularExtent) ||
      !std::isfinite(widthRatio) || linearExtent <= 0.0 ||
      angularExtent <= 0.0 || widthRatio <= 0.0) {
    throw std::invalid_argument(
        "[MyRbfShadowEstimator] symmetric grid extents and width ratio must be positive and finite.");
  }

  MyRbfShadowSettings settings;
  settings.residualScale = residualScale;
  settings.discrepancyScale = discrepancyScale;
  settings.outputScale = outputScale;
  settings.trackingGain = trackingGain;
  settings.predictionGain = predictionGain;
  settings.leakage = leakage;
  settings.centers = matrix_t::Zero(
      MyRbfShadowEstimator::kAxisCount,
      MyRbfShadowEstimator::kInputDimension *
          MyRbfShadowEstimator::kGaussianNodeCount);
  settings.widths = matrix_t::Zero(
      MyRbfShadowEstimator::kAxisCount,
      MyRbfShadowEstimator::kGaussianNodeCount);

  for (Eigen::Index axis = 0;
       axis < MyRbfShadowEstimator::kAxisCount; ++axis) {
    const scalar_t extent = axis < 3 ? linearExtent : angularExtent;
    const scalar_t spacing = extent / 2.0;
    for (Eigen::Index row = 0; row < 5; ++row) {
      for (Eigen::Index column = 0; column < 5; ++column) {
        const Eigen::Index node = 5 * row + column;
        settings.centers(axis, 2 * node) =
            -extent + spacing * column;
        settings.centers(axis, 2 * node + 1) =
            -extent + spacing * row;
        settings.widths(axis, node) = widthRatio * spacing;
      }
    }
  }

  static_cast<void>(MyRbfShadowEstimator(settings));
  return settings;
}

MyRbfShadowEstimator::MyRbfShadowEstimator(MyRbfShadowSettings settings)
    : settings_(std::move(settings)) {
  validateSettings();
}

void MyRbfShadowEstimator::validateSettings() const {
  requirePositiveFinite(settings_.residualScale, "residualScale");
  requirePositiveFinite(settings_.discrepancyScale, "discrepancyScale");
  requirePositiveFinite(settings_.outputScale, "outputScale");
  requireNonnegativeFinite(settings_.trackingGain, "trackingGain");
  requireNonnegativeFinite(settings_.predictionGain, "predictionGain");
  requireNonnegativeFinite(settings_.leakage, "leakage");

  if (settings_.centers.rows() != kAxisCount ||
      settings_.centers.cols() != kInputDimension * kGaussianNodeCount ||
      !settings_.centers.allFinite()) {
    throw std::invalid_argument(
        "[MyRbfShadowEstimator] centers must be a finite 6x50 matrix.");
  }
  if (settings_.widths.rows() != kAxisCount ||
      settings_.widths.cols() != kGaussianNodeCount ||
      !settings_.widths.allFinite() || (settings_.widths.array() <= 0.0).any()) {
    throw std::invalid_argument(
        "[MyRbfShadowEstimator] widths must be a positive finite 6x25 matrix.");
  }
}

matrix_t MyRbfShadowEstimator::computeFeature(
    const vector6_t& residualFiltered) const {
  matrix_t feature(kInputDimension, kAxisCount);
  feature.row(0) =
      residualFiltered.cwiseQuotient(settings_.residualScale).transpose();
  feature.row(1) =
      (residualFiltered - previousEstimate_)
          .cwiseQuotient(settings_.discrepancyScale)
          .transpose();
  return feature;
}

matrix_t MyRbfShadowEstimator::computeBasis(const matrix_t& feature) const {
  matrix_t basis(kBasisCount, kAxisCount);
  basis.row(0).setOnes();
  for (Eigen::Index axis = 0; axis < kAxisCount; ++axis) {
    for (Eigen::Index node = 0; node < kGaussianNodeCount; ++node) {
      const scalar_t dx =
          feature(0, axis) - settings_.centers(axis, 2 * node);
      const scalar_t dy =
          feature(1, axis) - settings_.centers(axis, 2 * node + 1);
      const scalar_t width = settings_.widths(axis, node);
      basis(node + 1, axis) =
          std::exp(-(dx * dx + dy * dy) / (2.0 * width * width));
    }
  }
  return basis;
}

vector6_t MyRbfShadowEstimator::evaluate(const matrix_t& basis) const {
  vector6_t estimate;
  for (Eigen::Index axis = 0; axis < kAxisCount; ++axis) {
    estimate(axis) = settings_.outputScale(axis) *
                     weights_.col(axis).dot(basis.col(axis));
  }
  return estimate;
}

MyRbfShadowStepResult MyRbfShadowEstimator::step(
    const vector6_t& residualFiltered, const vector6_t& sigma, scalar_t dt) {
  if (!residualFiltered.allFinite() || !sigma.allFinite()) {
    throw std::invalid_argument(
        "[MyRbfShadowEstimator] residual and sigma must be finite.");
  }
  if (!std::isfinite(dt) || dt < 0.0) {
    throw std::invalid_argument(
        "[MyRbfShadowEstimator] dt must be finite and nonnegative.");
  }

  MyRbfShadowStepResult result;
  result.feature = computeFeature(residualFiltered);
  result.basis = computeBasis(result.feature);
  result.estimateBeforeUpdate = evaluate(result.basis);
  result.weightDerivative = matrix_t::Zero(kBasisCount, kAxisCount);

  const vector6_t predictionError =
      residualFiltered - result.estimateBeforeUpdate;
  for (Eigen::Index axis = 0; axis < kAxisCount; ++axis) {
    const vector_t physicalGradient =
        settings_.outputScale(axis) * result.basis.col(axis);
    result.weightDerivative.col(axis) =
        settings_.trackingGain(axis) * sigma(axis) * physicalGradient +
        settings_.predictionGain(axis) * predictionError(axis) *
            physicalGradient -
        settings_.leakage(axis) * weights_.col(axis);
  }

  result.weightsAfterUpdate = weights_ + dt * result.weightDerivative;
  if (!result.weightsAfterUpdate.allFinite()) {
    throw std::overflow_error(
        "[MyRbfShadowEstimator] update produced non-finite weights.");
  }
  weights_ = result.weightsAfterUpdate;
  result.estimateAfterUpdate = evaluate(result.basis);
  previousEstimate_ = result.estimateAfterUpdate;
  return result;
}

void MyRbfShadowEstimator::reset() {
  weights_.setZero();
  previousEstimate_.setZero();
}

}  // namespace adaptive
}  // namespace legged
