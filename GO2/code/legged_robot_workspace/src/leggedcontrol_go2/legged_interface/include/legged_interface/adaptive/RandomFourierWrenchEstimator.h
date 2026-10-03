#pragma once

#include <cstdint>

#include "legged_interface/adaptive/AdaptiveTypes.h"

namespace legged {
namespace adaptive {

struct RandomFourierWrenchEstimatorSettings {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  vector6_t residualScale = vector6_t::Ones();
  vector6_t discrepancyScale = vector6_t::Ones();
  vector6_t outputScale = vector6_t::Ones();
  vector6_t learningRate = vector6_t::Zero();
  Eigen::Index featureCount = 50;
  std::uint32_t randomSeed = 20260824;
  scalar_t frequencyStd = 1.0;
  scalar_t projectionRadius = 0.0;
};

struct RandomFourierWrenchStepResult {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  matrix_t feature;
  matrix_t basis;
  vector6_t estimateBeforeUpdate = vector6_t::Zero();
  matrix_t weightDerivative;
  matrix_t weightsAfterUpdate;
  vector6_t estimateAfterUpdate = vector6_t::Zero();
};

class RandomFourierWrenchEstimator final {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  static constexpr Eigen::Index kAxisCount = 6;
  static constexpr Eigen::Index kInputDimension = 2;

  explicit RandomFourierWrenchEstimator(RandomFourierWrenchEstimatorSettings settings);
  RandomFourierWrenchStepResult step(const vector6_t& residualFiltered, scalar_t dt);
  const vector6_t& estimate() const { return previousEstimate_; }
  const matrix_t& weights() const { return weights_; }
  const matrix_t& frequencies() const { return frequencies_; }
  const vector_t& phases() const { return phases_; }
  void reset();

 private:
  matrix_t computeFeature(const vector6_t& residualFiltered) const;
  matrix_t computeBasis(const matrix_t& feature) const;
  vector6_t evaluate(const matrix_t& basis) const;
  void projectRows();
  void validateSettings() const;

  RandomFourierWrenchEstimatorSettings settings_;
  matrix_t frequencies_;
  vector_t phases_;
  matrix_t weights_;
  vector6_t previousEstimate_ = vector6_t::Zero();
};

}  // namespace adaptive
}  // namespace legged
