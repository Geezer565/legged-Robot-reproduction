#include "legged_interface/adaptive/MyRbfShadowPipeline.h"

#include <utility>

namespace legged {
namespace adaptive {

constexpr bool MyRbfShadowPipeline::kControlInjectionEnabled;

MyRbfShadowPipeline::MyRbfShadowPipeline(
    scalar_t robotMass, Eigen::Index stateDimension, bool enableFilter,
    scalar_t filterAlpha, scalar_t timeTolerance,
    MyRbfShadowSettings estimatorSettings)
    : residualObserver_(robotMass, stateDimension, enableFilter, filterAlpha,
                        timeTolerance),
      rbfEstimator_(new MyRbfShadowEstimator(std::move(estimatorSettings))) {}

MyRbfShadowPipeline::MyRbfShadowPipeline(
    scalar_t robotMass, Eigen::Index stateDimension, bool enableFilter,
    scalar_t filterAlpha, scalar_t timeTolerance,
    RandomFourierWrenchEstimatorSettings estimatorSettings)
    : residualObserver_(robotMass, stateDimension, enableFilter, filterAlpha,
                        timeTolerance),
      rffEstimator_(new RandomFourierWrenchEstimator(std::move(estimatorSettings))) {}

MyRbfShadowPipeline::MyRbfShadowPipeline(
    scalar_t robotMass, Eigen::Index stateDimension, bool enableFilter,
    scalar_t filterAlpha, scalar_t timeTolerance,
    ComparisonWrenchEstimatorSettings estimatorSettings)
    : residualObserver_(robotMass, stateDimension, enableFilter, filterAlpha,
                        timeTolerance),
      comparisonEstimator_(
          new ComparisonWrenchEstimator(std::move(estimatorSettings))) {}

void MyRbfShadowPipeline::setPrediction(
    scalar_t predictedTime, scalar_t intervalDt,
    const vector_t& predictedState,
    const vector6_t& knownAppliedCompensation) {
  residualObserver_.setPrediction(predictedTime, intervalDt, predictedState,
                                  knownAppliedCompensation);
}

MyRbfShadowPipelineResult MyRbfShadowPipeline::observeActual(
    scalar_t actualTime, const vector_t& actualState,
    const vector6_t& trackingError) {
  MyRbfShadowPipelineResult result;
  result.trackingError = trackingError;
  result.residual = residualObserver_.observe(actualTime, actualState);
  if (!result.residual.valid) {
    return result;
  }
  if (rbfEstimator_) {
    result.estimator = rbfEstimator_->step(result.residual.filteredResidual,
                                           trackingError, result.residual.dt);
  } else if (rffEstimator_) {
    const auto rff = rffEstimator_->step(result.residual.filteredResidual,
                                         result.residual.dt);
    result.estimator.feature = rff.feature;
    result.estimator.basis = rff.basis;
    result.estimator.estimateBeforeUpdate = rff.estimateBeforeUpdate;
    result.estimator.estimateAfterUpdate = rff.estimateAfterUpdate;
    result.estimator.weightDerivative = rff.weightDerivative;
    result.estimator.weightsAfterUpdate = rff.weightsAfterUpdate;
  } else {
    const auto comparison = comparisonEstimator_->step(
        result.residual.filteredResidual, trackingError, result.residual.dt);
    result.estimator.feature = comparison.feature;
    result.estimator.basis = comparison.basis;
    result.estimator.estimateBeforeUpdate =
        comparison.compensationBeforeUpdate;
    result.estimator.estimateAfterUpdate =
        comparison.compensationAfterUpdate;
    result.estimator.weightDerivative = comparison.weightDerivative;
    result.estimator.weightsAfterUpdate = comparison.weightsAfterUpdate;
    result.estimator.trackingBiasDerivative =
        comparison.compensationAfterUpdate -
        comparison.compensationBeforeUpdate;
  }
  result.valid = true;
  return result;
}

const vector6_t& MyRbfShadowPipeline::estimate() const {
  if (rbfEstimator_) return rbfEstimator_->estimate();
  if (rffEstimator_) return rffEstimator_->estimate();
  return comparisonEstimator_->estimate();
}

void MyRbfShadowPipeline::reset() {
  residualObserver_.reset();
  if (rbfEstimator_) rbfEstimator_->reset();
  if (rffEstimator_) rffEstimator_->reset();
  if (comparisonEstimator_) comparisonEstimator_->reset();
}

}  // namespace adaptive
}  // namespace legged
