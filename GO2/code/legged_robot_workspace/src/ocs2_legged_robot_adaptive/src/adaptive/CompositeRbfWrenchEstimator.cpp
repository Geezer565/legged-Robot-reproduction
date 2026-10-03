#include "ocs2_legged_robot_adaptive/adaptive/CompositeRbfWrenchEstimator.h"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace ocs2 { namespace legged_robot_adaptive { namespace adaptive {
namespace {

void validateGain(const vector6_t& gain, const char* name) {
  if (!gain.allFinite() || (gain.array() < 0.0).any()) {
    throw std::invalid_argument(
        std::string("[CompositeRbfWrenchEstimator] ") + name +
        " must be finite and nonnegative.");
  }
}

}  // namespace

CompositeRbfWrenchEstimator::CompositeRbfWrenchEstimator(
    CompositeRbfWrenchEstimatorSettings settings)
    : settings_(std::move(settings)),
      input_(settings_.network.residualScale,
             settings_.network.discrepancyScale),
      network_(settings_.network) {
  validateGain(settings_.trackingGain, "tracking gain");
  validateGain(settings_.predictionGain, "prediction gain");
  validateGain(settings_.leakage, "leakage");
}

CompositeRbfWrenchStepResult CompositeRbfWrenchEstimator::step(
    const vector6_t& residualFiltered, const vector6_t& sigma, scalar_t dt) {
  if (!residualFiltered.allFinite() || !sigma.allFinite()) {
    throw std::invalid_argument(
        "[CompositeRbfWrenchEstimator] residual and sigma must be finite.");
  }
  if (!std::isfinite(dt) || dt < 0.0) {
    throw std::invalid_argument(
        "[CompositeRbfWrenchEstimator] dt must be finite and nonnegative.");
  }

  CompositeRbfWrenchStepResult result;
  result.feature = input_.compute(
      residualFiltered, previousWrenchEstimate_).feature;
  const auto beforeUpdate = network_.evaluate(result.feature);
  result.basis = beforeUpdate.basis;
  result.estimateBeforeUpdate = beforeUpdate.physicalOutput;
  result.weightDerivative = matrix_t::Zero(
      TwoFeatureSixSubnetworkRbfWrenchNetwork::kBasisCount,
      TwoFeatureSixSubnetworkRbfWrenchNetwork::kAxisCount);

  const vector6_t predictionError =
      residualFiltered - result.estimateBeforeUpdate;
  for (Eigen::Index axis = 0;
       axis < TwoFeatureSixSubnetworkRbfWrenchNetwork::kAxisCount; ++axis) {
    const vector_t physicalGradient =
        settings_.network.outputScale(axis) * result.basis.col(axis);
    result.weightDerivative.col(axis) =
        settings_.trackingGain(axis) * sigma(axis) * physicalGradient +
        settings_.predictionGain(axis) * predictionError(axis) *
            physicalGradient -
        settings_.leakage(axis) * network_.weights().col(axis);
  }

  result.weightsAfterUpdate =
      network_.weights() + dt * result.weightDerivative;
  if (!result.weightsAfterUpdate.allFinite()) {
    throw std::overflow_error(
        "[CompositeRbfWrenchEstimator] Euler update produced non-finite weights.");
  }
  network_.setWeights(result.weightsAfterUpdate);
  result.estimateAfterUpdate =
      network_.evaluate(result.feature).physicalOutput;
  previousWrenchEstimate_ = result.estimateAfterUpdate;
  return result;
}

}}}  // namespace ocs2::legged_robot_adaptive::adaptive
