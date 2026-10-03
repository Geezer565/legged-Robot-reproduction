#include <limits>

#include <gtest/gtest.h>

#include "ocs2_legged_robot_adaptive/adaptive/CompositeRbfWrenchEstimator.h"

namespace ocs2 { namespace legged_robot_adaptive { namespace adaptive { namespace {

CompositeRbfWrenchEstimatorSettings settings() {
  CompositeRbfWrenchEstimatorSettings value;
  value.network.residualScale.setOnes();
  value.network.discrepancyScale.setOnes();
  value.network.outputScale.setOnes();
  value.network.centers = matrix_t::Zero(6, 50);
  value.network.widths = matrix_t::Ones(6, 25);
  value.trackingGain.setZero();
  value.predictionGain.setOnes();
  value.leakage.setZero();
  return value;
}

TEST(CompositeRbfWrenchEstimator, PredictionErrorUpdatesWhenSigmaIsZero) {
  CompositeRbfWrenchEstimator estimator(settings());
  vector6_t residual = vector6_t::Zero();
  residual(0) = 0.5;

  const auto result = estimator.step(residual, vector6_t::Zero(), 0.01);

  EXPECT_TRUE(result.estimateBeforeUpdate.isZero(0.0));
  EXPECT_GT(result.weightDerivative.col(0).norm(), 0.0);
  EXPECT_GT(result.estimateAfterUpdate(0), 0.0);
}

TEST(CompositeRbfWrenchEstimator, ZeroPredictionAndTrackingErrorDoNotUpdate) {
  CompositeRbfWrenchEstimator estimator(settings());
  const auto result = estimator.step(
      vector6_t::Zero(), vector6_t::Zero(), 0.1);
  EXPECT_TRUE(result.weightDerivative.isZero(0.0));
  EXPECT_TRUE(estimator.weights().isZero(0.0));
}

TEST(CompositeRbfWrenchEstimator, LeakageReducesExistingWeightNorm) {
  auto value = settings();
  value.predictionGain.setZero();
  value.leakage.setConstant(0.5);
  CompositeRbfWrenchEstimator estimator(value);
  estimator.setWeights(matrix_t::Ones(26, 6));
  const scalar_t before = estimator.weights().norm();

  estimator.step(vector6_t::Zero(), vector6_t::Zero(), 0.1);

  EXPECT_LT(estimator.weights().norm(), before);
}

TEST(CompositeRbfWrenchEstimator, SixOutputAxesStayIndependent) {
  CompositeRbfWrenchEstimator estimator(settings());
  vector6_t residual = vector6_t::Zero();
  residual(0) = 0.4;
  for (int sample = 0; sample < 10; ++sample) {
    estimator.step(residual, vector6_t::Zero(), 0.01);
  }

  EXPECT_GT(estimator.weights().col(0).norm(), 0.0);
  EXPECT_TRUE(estimator.weights().rightCols<5>().isZero(0.0));
  EXPECT_TRUE(estimator.previousWrenchEstimate().tail<5>().isZero(0.0));
}

TEST(CompositeRbfWrenchEstimator, TrackingTermRemainsAvailable) {
  auto value = settings();
  value.predictionGain.setZero();
  value.trackingGain.setConstant(0.25);
  CompositeRbfWrenchEstimator estimator(value);
  vector6_t sigma = vector6_t::Zero();
  sigma(2) = 0.4;

  const auto result = estimator.step(vector6_t::Zero(), sigma, 0.01);

  EXPECT_GT(result.weightDerivative.col(2).norm(), 0.0);
  EXPECT_TRUE(result.weightDerivative.leftCols<2>().isZero(0.0));
  EXPECT_TRUE(result.weightDerivative.rightCols<3>().isZero(0.0));
}

TEST(CompositeRbfWrenchEstimator, RejectsInvalidGains) {
  auto value = settings();
  value.predictionGain(0) = -1.0;
  EXPECT_THROW(CompositeRbfWrenchEstimator{value}, std::invalid_argument);

  value = settings();
  value.trackingGain(0) = std::numeric_limits<scalar_t>::quiet_NaN();
  EXPECT_THROW(CompositeRbfWrenchEstimator{value}, std::invalid_argument);

  value = settings();
  value.leakage(0) = -0.1;
  EXPECT_THROW(CompositeRbfWrenchEstimator{value}, std::invalid_argument);
}

}}}}  // namespace ocs2::legged_robot_adaptive::adaptive
