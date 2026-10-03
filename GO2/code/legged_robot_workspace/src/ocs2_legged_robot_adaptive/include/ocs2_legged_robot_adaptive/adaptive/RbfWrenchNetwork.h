#pragma once

#include "ocs2_legged_robot_adaptive/adaptive/AdaptiveTypes.h"

namespace ocs2 { namespace legged_robot_adaptive { namespace adaptive {

struct RbfWrenchSettings { EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  vector6_t residualScale = vector6_t::Ones();
  vector6_t sigmaScale = vector6_t::Ones();
  vector6_t learningGain = vector6_t::Ones();
  scalar_t beta = 0.7;
};
struct RbfWrenchEvaluation { EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  vector6_t normalizedInput = vector6_t::Zero(); vector_t basis;
  vector6_t normalizedOutput = vector6_t::Zero(); vector6_t physicalOutput = vector6_t::Zero();
};
struct RbfWrenchUpdateResult { EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  RbfWrenchEvaluation evaluation; vector6_t normalizedResidualError = vector6_t::Zero();
  vector6_t normalizedSigma = vector6_t::Zero(); vector6_t compositeError = vector6_t::Zero(); matrix_t weightDerivative;
};

/** Six independent scalar RBF outputs sharing one explicitly supplied Gaussian basis. */
class RbfWrenchNetwork final {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  RbfWrenchNetwork(RbfWrenchSettings settings, matrix_t centers, vector_t widths);
  vector6_t normalizeResidual(const vector6_t& residual) const;
  vector_t computeBasis(const vector6_t& chi) const;
  RbfWrenchEvaluation evaluate(const vector6_t& residual) const;
  vector6_t computeCompositeError(const vector6_t& residual, const vector6_t& estimatedWrench, const vector6_t& sigma) const;
  matrix_t computeWeightDerivative(const vector_t& basis, const vector6_t& compositeError) const;
  RbfWrenchUpdateResult updateWeights(const vector6_t& residual, const vector6_t& sigma, scalar_t dt);
  const matrix_t& weights() const { return weights_; }
  void setWeights(const matrix_t& weights); void resetWeights();
 private:
  void validateSettings() const; void validateVector(const vector6_t& value, const char* name) const;
  RbfWrenchSettings settings_; matrix_t centers_; vector_t widths_; matrix_t weights_;
};

}}}  // namespace ocs2::legged_robot_adaptive::adaptive
