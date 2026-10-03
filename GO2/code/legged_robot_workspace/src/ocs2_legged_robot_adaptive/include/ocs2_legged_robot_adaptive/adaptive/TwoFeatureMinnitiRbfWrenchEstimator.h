#pragma once

#include "ocs2_legged_robot_adaptive/adaptive/TwoFeatureRbfWrenchInput.h"
#include "ocs2_legged_robot_adaptive/adaptive/TwoFeatureSixSubnetworkRbfWeightAdapter.h"
#include "ocs2_legged_robot_adaptive/adaptive/TwoFeatureSixSubnetworkRbfWrenchNetwork.h"

namespace ocs2 { namespace legged_robot_adaptive { namespace adaptive {

struct TwoFeatureMinnitiRbfWrenchEstimatorSettings { EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  TwoFeatureSixSubnetworkRbfWrenchSettings network;
  vector6_t gamma = vector6_t::Zero();
};

struct TwoFeatureMinnitiRbfWrenchStepResult { EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  matrix_t feature;
  matrix_t basis;
  vector6_t estimateBeforeUpdate = vector6_t::Zero();
  matrix_t weightDerivative;
  matrix_t weightsAfterUpdate;
  vector6_t estimateAfterUpdate = vector6_t::Zero();
};

/**
 * Minimal stateful entry point for the two-feature, six-subnetwork RBF estimator.
 * Each step forms features from the previous cycle's estimate, applies the fixed
 * Minniti-consistent weight update, then stores the updated estimate for the next cycle.
 */
class TwoFeatureMinnitiRbfWrenchEstimator final {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  explicit TwoFeatureMinnitiRbfWrenchEstimator(TwoFeatureMinnitiRbfWrenchEstimatorSettings settings);

  TwoFeatureMinnitiRbfWrenchStepResult step(const vector6_t& residualFiltered,
                                            const vector6_t& sigma,
                                            scalar_t dt);

  const matrix_t& weights() const { return network_.weights(); }
  const vector6_t& previousWrenchEstimate() const { return previousWrenchEstimate_; }

 private:
  TwoFeatureRbfWrenchInput input_;
  TwoFeatureSixSubnetworkRbfWrenchNetwork network_;
  TwoFeatureSixSubnetworkRbfWeightAdapter adapter_;
  vector6_t previousWrenchEstimate_ = vector6_t::Zero();
};

}}}  // namespace ocs2::legged_robot_adaptive::adaptive
