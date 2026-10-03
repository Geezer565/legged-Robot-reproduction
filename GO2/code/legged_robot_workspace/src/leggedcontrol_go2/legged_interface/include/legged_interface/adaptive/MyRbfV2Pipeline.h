#pragma once

#include "legged_interface/adaptive/AdaptiveTypes.h"
#include "legged_interface/adaptive/MyRbfPredictionResidualObserver.h"
#include "legged_interface/adaptive/MyRbfV2Core.h"

namespace legged {
namespace adaptive {

struct MyRbfV2PipelineResult {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  bool valid = false;
  MyRbfPredictionResidualResult residual;
  vector6_t chi = vector6_t::Zero();
  vector6_t estimateBeforeUpdate = vector6_t::Zero();
  vector6_t estimateAfterUpdate = vector6_t::Zero();
  vector6_t robustWrench = vector6_t::Zero();
  vector6_t robustBound = vector6_t::Zero();
  vector6_t smoothingWidth = vector6_t::Zero();
  vector6_t coverage = vector6_t::Zero();
  vector6_t weightDerivativeNorm = vector6_t::Zero();
  std::array<bool, 6> projectionActive{{false, false, false, false, false,
                                        false}};
  v2::FeatureInput featureInput{};
  v2::Evaluation evaluationBeforeUpdate{};
  v2::Evaluation evaluationAfterUpdate{};
  vector6_t externalWrenchEstimate = vector6_t::Zero();
  vector6_t leftHandDisturbanceEstimate = vector6_t::Zero();
  vector6_t requiredControlCorrection = vector6_t::Zero();
  unsigned historySize = 0;
  bool historySampleAdded = false;
};

class MyRbfV2Pipeline final {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  MyRbfV2Pipeline(scalar_t robotMass, Eigen::Index stateDimension,
                  bool enableFilter, scalar_t filterAlpha,
                  scalar_t timeTolerance, v2::Settings settings);
  void setPrediction(scalar_t predictedTime, scalar_t intervalDt,
                     const vector_t& predictedState,
                     const vector6_t& knownAppliedCompensation =
                         vector6_t::Zero());
  MyRbfV2PipelineResult observeActual(scalar_t actualTime,
                                      const vector_t& actualState,
                                      const vector6_t& chi);
  // Legacy observer residual direction is retained by observeActual. In this
  // overload observer residual replaces featureInput.causalResidual only.
  MyRbfV2PipelineResult observeActual(scalar_t actualTime,
                                      const vector_t& actualState,
                                      const v2::FeatureInput& featureInput);
  // Direct physical path: no observer/prediction dependency.
  // externalResidualBase is the measured EXTERNAL wrench in world axes at the
  // base reference point; chi/velocities must be conjugate there. V2 keeps
  // this physical direction. TeacherComposite converts it to the Word-model
  // left-hand residual before applying the configured first-order filter.
  // The filtered residual replaces featureInput.causalResidual. chi is kept.
  MyRbfV2PipelineResult advanceExternalResidual(
      scalar_t actualTime, scalar_t dt, const vector6_t& externalResidualBase,
      const v2::FeatureInput& featureInput);
  const vector6_t& compensationWrench() const { return compensationWrench_; }
  const vector6_t& requiredControlCorrection() const { return compensationWrench_; }
  const v2::Evaluation& evaluation() const { return evaluation_; }
  const v2::FeatureInput& featureInput() const { return featureInput_; }
  const v2::Core& core() const { return core_; }
  bool hasEvaluation() const { return hasEvaluation_; }
  const v2::State& state() const { return state_; }
  std::size_t historySize() const { return history_.size(); }
  void reset();

 private:
  MyRbfV2PipelineResult advance(scalar_t actualTime, scalar_t dt,
                                const v2::FeatureInput& input,
                                MyRbfV2PipelineResult result);
  MyRbfPredictionResidualObserver residualObserver_;
  v2::Core core_;
  v2::State state_;
  bool externalResidualFilterEnabled_ = false;
  scalar_t externalResidualFilterAlpha_ = 0.0;
  bool externalResidualFilterInitialized_ = false;
  vector6_t filteredExternalResidual_ = vector6_t::Zero();
  bool timeInitialized_ = false;
  scalar_t startTime_ = 0.0;
  vector6_t compensationWrench_ = vector6_t::Zero();
  v2::Evaluation evaluation_{};
  v2::FeatureInput featureInput_{};
  bool hasEvaluation_ = false;
  scalar_t lastTime_ = 0.0;
  std::vector<v2::Sample> history_;
  v2::Weights historyGradientIntegral_{};
  v2::Axes historyLabelIntegral_{};
  scalar_t historyWindowElapsed_ = 0.0;
};

}  // namespace adaptive
}  // namespace legged
