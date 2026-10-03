#pragma once

#include <array>
#include <limits>

#include "legged_interface/adaptive/AdaptiveTypes.h"

namespace legged {
namespace adaptive {

struct MyRbfShadowSettings {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  vector6_t residualScale = vector6_t::Ones();
  vector6_t outputScale = vector6_t::Ones();
  matrix_t centers;
  matrix_t widths;
  vector6_t trackingGain = vector6_t::Zero();
  vector6_t predictionGain = vector6_t::Zero();
  vector6_t leakage = vector6_t::Zero();
  bool normalizeUpdate = false;
  bool trackingBiasOnly = false;
  scalar_t updateEpsilon = 1e-9;
  scalar_t minimumCoverage = 0.0;
  vector6_t weightLimit = vector6_t::Constant(
      std::numeric_limits<scalar_t>::infinity());
};

struct MyRbfShadowStepResult {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  matrix_t feature;
  matrix_t basis;
  vector6_t estimateBeforeUpdate = vector6_t::Zero();
  vector6_t estimateAfterUpdate = vector6_t::Zero();
  vector6_t coverage = vector6_t::Zero();
  std::array<bool, 6> updateFrozen{};
  vector6_t predictionDerivativeNorm = vector6_t::Zero();
  vector6_t trackingBiasDerivative = vector6_t::Zero();
  vector6_t leakageDerivativeNorm = vector6_t::Zero();
  std::array<bool, 6> projectionActive{{false, false, false, false, false,
                                        false}};
  matrix_t weightDerivative;
  matrix_t weightsAfterUpdate;
};

MyRbfShadowSettings createSymmetricGridMyRbfShadowSettings(
    scalar_t linearExtent, scalar_t angularExtent, scalar_t widthRatio,
    const vector6_t& residualScale,
    const vector6_t& outputScale, const vector6_t& trackingGain,
    const vector6_t& predictionGain, const vector6_t& leakage);

/**
 * Pure six-axis RBF estimator. It never writes controller outputs directly;
 * the separately gated controller adapter may consume estimate() for closed-loop
 * compensation.
 */
class MyRbfShadowEstimator final {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static constexpr Eigen::Index kAxisCount = 6;
  static constexpr Eigen::Index kInputDimension = 1;
  static constexpr Eigen::Index kGaussianNodeCount = 25;
  static constexpr Eigen::Index kBasisCount = 26;
  static constexpr bool kControlInjectionEnabled = false;

  explicit MyRbfShadowEstimator(MyRbfShadowSettings settings);

  MyRbfShadowStepResult step(const vector6_t& residualFiltered,
                             const vector6_t& sigma, scalar_t dt,
                             scalar_t updateGate = 1.0);

  const vector6_t& estimate() const { return previousEstimate_; }
  const matrix_t& weights() const { return weights_; }
  bool controlInjectionEnabled() const { return kControlInjectionEnabled; }
  void reset();

 private:
  matrix_t computeFeature(const vector6_t& residualFiltered) const;
  matrix_t computeBasis(const matrix_t& feature) const;
  vector6_t evaluate(const matrix_t& basis) const;
  void validateSettings() const;

  MyRbfShadowSettings settings_;
  matrix_t weights_ = matrix_t::Zero(kBasisCount, kAxisCount);
  vector6_t previousEstimate_ = vector6_t::Zero();
};

}  // namespace adaptive
}  // namespace legged
