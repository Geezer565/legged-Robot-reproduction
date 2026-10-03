#include <gtest/gtest.h>

#include "legged_interface/adaptive/ComparisonWrenchEstimator.h"

namespace legged {
namespace adaptive {
namespace {

ComparisonWrenchEstimatorSettings baseSettings(ComparisonMethod method) {
  ComparisonWrenchEstimatorSettings settings;
  settings.method = method;
  settings.outputScale.setOnes();
  settings.residualScale.setOnes();
  settings.trackingScale.setOnes();
  settings.adaptationGain.setOnes();
  settings.weightLimit.setConstant(100.0);
  settings.gaussianWidth = 0.5;
  settings.compositeRatio = 0.75;
  settings.minimumStep = 0.2;
  settings.maximumStep = 1.0;
  settings.neuralFlyLambda = 0.1;
  settings.neuralFlyProcessNoise = 0.2;
  settings.neuralFlyMeasurementNoise = 1.0;
  settings.neuralFlyInitialCovariance = 1.0;
  return settings;
}

TEST(ComparisonWrenchEstimator, EthUsesDirectAdaptiveParameterWithoutRbfBasis) {
  ComparisonWrenchEstimator estimator(baseSettings(ComparisonMethod::EthDirect));
  vector6_t tracking = vector6_t::Zero();
  tracking(1) = -2.0;
  const auto result = estimator.step(vector6_t::Zero(), tracking, 0.1);

  EXPECT_EQ(result.representation, ComparisonRepresentation::DirectParameter);
  EXPECT_EQ(result.basis.rows(), 1);
  EXPECT_NEAR(result.compensationAfterUpdate(1), -0.2, 1e-12);
  EXPECT_TRUE(result.compensationAfterUpdate.head<1>().isZero(1e-12));
  EXPECT_TRUE(result.compensationAfterUpdate.tail<4>().isZero(1e-12));
}

TEST(ComparisonWrenchEstimator, LiuUsesCompositeErrorAndVariableStepRbfUpdate) {
  auto settings = baseSettings(ComparisonMethod::LiuDualRbfPort);
  ComparisonWrenchEstimator estimator(settings);
  vector6_t residual = vector6_t::Zero();
  vector6_t tracking = vector6_t::Zero();
  residual(0) = 1.0;
  tracking(0) = -1.0;
  const auto result = estimator.step(residual, tracking, 0.1);

  EXPECT_EQ(result.representation, ComparisonRepresentation::DualGaussianRbf);
  EXPECT_GT(result.basis.rows(), 1);
  EXPECT_NEAR(result.compositeError(0), 0.5, 1e-12);
  EXPECT_GE(result.effectiveStep(0), settings.minimumStep);
  EXPECT_LE(result.effectiveStep(0), settings.maximumStep);
  EXPECT_GT(result.compensationAfterUpdate(0), 0.0);
}

TEST(ComparisonWrenchEstimator, NeuralFlyConstantUpdatesCovarianceAndParameter) {
  ComparisonWrenchEstimator estimator(baseSettings(ComparisonMethod::NeuralFlyConstant));
  vector6_t residual = vector6_t::Zero();
  vector6_t tracking = vector6_t::Zero();
  residual(2) = 2.0;
  tracking(2) = 0.5;
  const auto result = estimator.step(residual, tracking, 0.1);

  EXPECT_EQ(result.representation, ComparisonRepresentation::ConstantFeature);
  EXPECT_GT(result.compensationAfterUpdate(2), 0.0);
  EXPECT_LT(result.covarianceAfterUpdate(2), 1.0);
  EXPECT_TRUE(result.compensationAfterUpdate.head<2>().isZero(1e-12));
  EXPECT_TRUE(result.compensationAfterUpdate.tail<3>().isZero(1e-12));
}

TEST(ComparisonWrenchEstimator, ExternalEstimateIsNegativeCompensation) {
  ComparisonWrenchEstimator estimator(baseSettings(ComparisonMethod::EthDirect));
  vector6_t tracking = vector6_t::Zero();
  tracking(4) = 3.0;
  const auto result = estimator.step(vector6_t::Zero(), tracking, 0.2);

  EXPECT_TRUE(result.externalEstimateAfterUpdate.isApprox(
      -result.compensationAfterUpdate, 1e-12));
}

}  // namespace
}  // namespace adaptive
}  // namespace legged
