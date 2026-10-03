#include "ocs2_legged_robot_adaptive/adaptive/TwoFeatureMinnitiRbfWrenchEstimator.h"

namespace ocs2 { namespace legged_robot_adaptive { namespace adaptive {
namespace {

TwoFeatureRbfWeightAdapterSettings makeAdapterSettings(
    const TwoFeatureMinnitiRbfWrenchEstimatorSettings& settings) {
  TwoFeatureRbfWeightAdapterSettings adapterSettings;
  adapterSettings.gamma = settings.gamma;
  adapterSettings.outputScale = settings.network.outputScale;
  return adapterSettings;
}

}  // namespace

TwoFeatureMinnitiRbfWrenchEstimator::TwoFeatureMinnitiRbfWrenchEstimator(
    TwoFeatureMinnitiRbfWrenchEstimatorSettings settings)
    : input_(settings.network.residualScale, settings.network.discrepancyScale),
      network_(settings.network),
      adapter_(makeAdapterSettings(settings)) {}

TwoFeatureMinnitiRbfWrenchStepResult TwoFeatureMinnitiRbfWrenchEstimator::step(
    const vector6_t& residualFiltered, const vector6_t& sigma, scalar_t dt) {
  TwoFeatureMinnitiRbfWrenchStepResult result;
  result.feature = input_.compute(residualFiltered, previousWrenchEstimate_).feature;

  const auto beforeUpdate = network_.evaluate(result.feature);
  result.basis = beforeUpdate.basis;
  result.estimateBeforeUpdate = beforeUpdate.physicalOutput;
  result.weightDerivative = adapter_.weightDerivative(result.basis, sigma);
  result.weightsAfterUpdate = adapter_.eulerStep(network_.weights(), result.weightDerivative, dt);
  network_.setWeights(result.weightsAfterUpdate);

  result.estimateAfterUpdate = network_.evaluate(result.feature).physicalOutput;
  previousWrenchEstimate_ = result.estimateAfterUpdate;
  return result;
}

}}}  // namespace ocs2::legged_robot_adaptive::adaptive
