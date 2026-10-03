#pragma once

#include <cstdint>

#include "ocs2_legged_robot_adaptive/adaptive/AdaptiveTypes.h"
#include "ocs2_legged_robot_adaptive/adaptive/TwoFeatureRbfWrenchInput.h"

namespace ocs2 { namespace legged_robot_adaptive { namespace adaptive {

struct RandomFourierWrenchEstimatorSettings { EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  vector6_t residualScale = vector6_t::Ones();
  vector6_t discrepancyScale = vector6_t::Ones();
  vector6_t outputScale = vector6_t::Ones();
  vector6_t learningRate = vector6_t::Zero();
  Eigen::Index featureCount = 50;
  std::uint32_t randomSeed = 20260824;
  scalar_t frequencyStd = 1.0;
  scalar_t projectionRadius = 0.0;
};

struct RandomFourierWrenchStepResult { EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  matrix_t feature;
  matrix_t basis;
  vector6_t estimateBeforeUpdate = vector6_t::Zero();
  matrix_t weightDerivative;
  matrix_t weightsAfterUpdate;
  vector6_t estimateAfterUpdate = vector6_t::Zero();
};

/**
 * Interface-matched SSI-MPC random Fourier estimator.
 *
 * The official implementation uses phi(z)=cos(omega*z+b)/sqrt(M) and
 * alpha += 2*eta*error*phi.  Here the same online update is applied to six
 * independent scalar outputs while retaining the already validated two-feature
 * input used by the current OCS2 RBF experiment.
 */
class RandomFourierWrenchEstimator final {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  explicit RandomFourierWrenchEstimator(
      RandomFourierWrenchEstimatorSettings settings);

  RandomFourierWrenchStepResult step(const vector6_t& residualFiltered,
                                      const vector6_t& sigma,
                                      scalar_t dt);

  const matrix_t& weights() const { return weights_; }
  const matrix_t& frequencies() const { return frequencies_; }
  const vector_t& phases() const { return phases_; }
  const vector6_t& previousWrenchEstimate() const {
    return previousWrenchEstimate_;
  }

 private:
  matrix_t computeBasis(const matrix_t& feature) const;
  vector6_t evaluate(const matrix_t& basis) const;
  void projectRows();

  RandomFourierWrenchEstimatorSettings settings_;
  TwoFeatureRbfWrenchInput input_;
  matrix_t frequencies_;  // M x 2, shared by all six scalar outputs.
  vector_t phases_;       // M x 1.
  matrix_t weights_;      // M x 6.
  vector6_t previousWrenchEstimate_ = vector6_t::Zero();
};

}}}  // namespace ocs2::legged_robot_adaptive::adaptive
