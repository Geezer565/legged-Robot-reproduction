#pragma once

#include "ocs2_legged_robot_adaptive/adaptive/TwoFeatureRbfWrenchInput.h"
#include "ocs2_legged_robot_adaptive/adaptive/TwoFeatureSixSubnetworkRbfWrenchNetwork.h"

namespace ocs2 { namespace legged_robot_adaptive { namespace adaptive {

struct CompositeRbfWrenchEstimatorSettings { EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  TwoFeatureSixSubnetworkRbfWrenchSettings network;
  vector6_t trackingGain = vector6_t::Zero();
  vector6_t predictionGain = vector6_t::Zero();
  vector6_t leakage = vector6_t::Zero();
};

struct CompositeRbfWrenchStepResult { EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  matrix_t feature;
  matrix_t basis;
  vector6_t estimateBeforeUpdate = vector6_t::Zero();
  matrix_t weightDerivative;
  matrix_t weightsAfterUpdate;
  vector6_t estimateAfterUpdate = vector6_t::Zero();
};

/**
 * Neural-Fly-inspired composite update on the existing frozen Gaussian basis.
 * It deliberately omits Neural-Fly's offline DAIML representation and adaptive
 * covariance matrix; the isolated ablation adds prediction-error and leakage
 * terms to the already validated tracking-error update.
 */
class CompositeRbfWrenchEstimator final {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  explicit CompositeRbfWrenchEstimator(
      CompositeRbfWrenchEstimatorSettings settings);

  CompositeRbfWrenchStepResult step(const vector6_t& residualFiltered,
                                     const vector6_t& sigma,
                                     scalar_t dt);

  const matrix_t& weights() const { return network_.weights(); }
  const vector6_t& previousWrenchEstimate() const {
    return previousWrenchEstimate_;
  }
  void setWeights(const matrix_t& weights) { network_.setWeights(weights); }

 private:
  CompositeRbfWrenchEstimatorSettings settings_;
  TwoFeatureRbfWrenchInput input_;
  TwoFeatureSixSubnetworkRbfWrenchNetwork network_;
  vector6_t previousWrenchEstimate_ = vector6_t::Zero();
};

}}}  // namespace ocs2::legged_robot_adaptive::adaptive
