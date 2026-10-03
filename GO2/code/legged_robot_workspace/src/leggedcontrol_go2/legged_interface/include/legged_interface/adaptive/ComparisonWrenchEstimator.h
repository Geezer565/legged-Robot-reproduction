#pragma once

#include "legged_interface/adaptive/AdaptiveTypes.h"

namespace legged {
namespace adaptive {

enum class ComparisonMethod {
  EthDirect,
  LiuDualRbfPort,
  NeuralFlyConstant,
};

enum class ComparisonRepresentation {
  DirectParameter,
  DualGaussianRbf,
  ConstantFeature,
};

struct ComparisonWrenchEstimatorSettings {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ComparisonMethod method = ComparisonMethod::EthDirect;
  vector6_t residualScale = vector6_t::Ones();
  vector6_t trackingScale = vector6_t::Ones();
  vector6_t outputScale = vector6_t::Ones();
  vector6_t adaptationGain = vector6_t::Ones();
  vector6_t weightLimit = vector6_t::Constant(100.0);
  scalar_t gaussianWidth = 0.5;
  scalar_t compositeRatio = 0.75;
  scalar_t minimumStep = 0.2;
  scalar_t maximumStep = 1.0;
  scalar_t neuralFlyLambda = 0.1;
  scalar_t neuralFlyProcessNoise = 0.2;
  scalar_t neuralFlyMeasurementNoise = 1.0;
  scalar_t neuralFlyInitialCovariance = 1.0;
};

struct ComparisonWrenchStepResult {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ComparisonRepresentation representation =
      ComparisonRepresentation::DirectParameter;
  matrix_t feature;
  matrix_t basis;
  vector6_t compensationBeforeUpdate = vector6_t::Zero();
  vector6_t compensationAfterUpdate = vector6_t::Zero();
  vector6_t externalEstimateAfterUpdate = vector6_t::Zero();
  vector6_t compositeError = vector6_t::Zero();
  vector6_t effectiveStep = vector6_t::Zero();
  vector6_t covarianceAfterUpdate = vector6_t::Zero();
  matrix_t weightDerivative;
  matrix_t weightsAfterUpdate;
};

/**
 * Method-specific online estimators adapted to the common Go2 six-axis
 * compensation interface.
 *
 * ETH uses the direct identity regressor Y=I and has no RBF representation.
 * Liu preserves the dual Gaussian-network, composite-error and variable-step
 * structure as a clearly labelled Go2 wrench-interface port.
 * Neural-Fly uses the paper's constant-feature online ablation with covariance
 * adaptation; it is not the offline-trained full Neural-Fly representation.
 */
class ComparisonWrenchEstimator final {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static constexpr Eigen::Index kAxisCount = 6;
  static constexpr Eigen::Index kLiuGaussianCount = 25;
  static constexpr Eigen::Index kLiuBasisCount = 26;

  explicit ComparisonWrenchEstimator(
      ComparisonWrenchEstimatorSettings settings);

  ComparisonWrenchStepResult step(const vector6_t& residualFiltered,
                                  const vector6_t& trackingError,
                                  scalar_t dt);
  const vector6_t& estimate() const { return compensationEstimate_; }
  void reset();

 private:
  ComparisonWrenchStepResult stepEth(const vector6_t& trackingError,
                                     scalar_t dt);
  ComparisonWrenchStepResult stepLiu(const vector6_t& residualFiltered,
                                     const vector6_t& trackingError,
                                     scalar_t dt);
  ComparisonWrenchStepResult stepNeuralFly(
      const vector6_t& residualFiltered, const vector6_t& trackingError,
      scalar_t dt);
  matrix_t liuBasis(const vector6_t& residualFiltered,
                    const vector6_t& trackingError) const;
  void validateSettings() const;
  void projectVector(vector6_t& value) const;
  void projectColumns(matrix_t& value) const;

  ComparisonWrenchEstimatorSettings settings_;
  vector6_t compensationEstimate_ = vector6_t::Zero();
  vector6_t covariance_ = vector6_t::Ones();
  matrix_t liuPredictionWeights_ =
      matrix_t::Zero(kLiuBasisCount, kAxisCount);
  matrix_t liuTrackingWeights_ =
      matrix_t::Zero(kLiuBasisCount, kAxisCount);
};

}  // namespace adaptive
}  // namespace legged
