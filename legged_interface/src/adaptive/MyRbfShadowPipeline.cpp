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
      estimator_(std::move(estimatorSettings)) {}

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
  result.estimator = estimator_.step(result.residual.filteredResidual,
                                     trackingError,
                                     result.residual.dt);
  result.valid = true;
  return result;
}

void MyRbfShadowPipeline::reset() {
  residualObserver_.reset();
  estimator_.reset();
}

}  // namespace adaptive
}  // namespace legged
