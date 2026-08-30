#pragma once

#include "legged_interface/adaptive/MyRbfPredictionResidualObserver.h"
#include "legged_interface/adaptive/MyRbfShadowEstimator.h"

namespace legged {
namespace adaptive {

struct MyRbfShadowPipelineResult {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  bool valid = false;
  MyRbfPredictionResidualResult residual;
  vector6_t trackingError = vector6_t::Zero();
  MyRbfShadowStepResult estimator;
};

/**
 * Composition of prediction-residual observation and MyRBF update. The
 * pipeline remains independent of the controller; its estimate can be logged
 * only or consumed by a separately gated control adapter.
 */
class MyRbfShadowPipeline final {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static constexpr bool kControlInjectionEnabled = false;

  MyRbfShadowPipeline(scalar_t robotMass, Eigen::Index stateDimension,
                      bool enableFilter, scalar_t filterAlpha,
                      scalar_t timeTolerance,
                      MyRbfShadowSettings estimatorSettings);

  void setPrediction(scalar_t predictedTime, scalar_t intervalDt,
                     const vector_t& predictedState,
                     const vector6_t& knownAppliedCompensation =
                         vector6_t::Zero());
  MyRbfShadowPipelineResult observeActual(scalar_t actualTime,
                                          const vector_t& actualState,
                                          const vector6_t& trackingError);
  const vector6_t& estimate() const { return estimator_.estimate(); }
  bool controlInjectionEnabled() const { return kControlInjectionEnabled; }
  void reset();

 private:
  MyRbfPredictionResidualObserver residualObserver_;
  MyRbfShadowEstimator estimator_;
};

}  // namespace adaptive
}  // namespace legged
