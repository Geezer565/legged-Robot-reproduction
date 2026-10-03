#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "ocs2_legged_robot_adaptive/adaptive/RandomFourierWrenchEstimator.h"

namespace ocs2 { namespace legged_robot_adaptive { namespace adaptive { namespace {

RandomFourierWrenchEstimatorSettings settings() {
  RandomFourierWrenchEstimatorSettings value;
  value.residualScale << 20.0, 20.0, 50.0, 5.0, 5.0, 5.0;
  value.discrepancyScale = value.residualScale;
  value.outputScale = value.residualScale;
  value.learningRate.setConstant(0.003);
  value.featureCount = 50;
  value.randomSeed = 20260824;
  value.frequencyStd = 1.0;
  value.projectionRadius = 0.0;
  return value;
}

TEST(RandomFourierWrenchEstimator, FixedSeedIsExactlyRepeatable) {
  RandomFourierWrenchEstimator left(settings());
  RandomFourierWrenchEstimator right(settings());
  vector6_t residual;
  residual << 10.0, -2.0, 3.0, 0.2, -0.1, 0.3;
  vector6_t sigma;
  sigma << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0;

  const auto a = left.step(residual, sigma, 0.005);
  const auto b = right.step(residual, sigma, 0.005);

  EXPECT_TRUE(a.feature.isApprox(b.feature, 0.0));
  EXPECT_TRUE(a.basis.isApprox(b.basis, 0.0));
  EXPECT_TRUE(a.estimateAfterUpdate.isApprox(b.estimateAfterUpdate, 0.0));
  EXPECT_TRUE(left.weights().isApprox(right.weights(), 0.0));
}

TEST(RandomFourierWrenchEstimator, RepeatedOnlineGradientUpdatesReduceError) {
  RandomFourierWrenchEstimator estimator(settings());
  vector6_t residual = vector6_t::Zero();
  residual(0) = 10.0;
  const scalar_t initialError = std::abs(residual(0));
  scalar_t finalError = initialError;

  for (int sample = 0; sample < 600; ++sample) {
    const auto result = estimator.step(residual, vector6_t::Zero(), 0.005);
    finalError = std::abs(residual(0) - result.estimateAfterUpdate(0));
  }

  EXPECT_LT(finalError, 0.25 * initialError);
  EXPECT_GT(estimator.previousWrenchEstimate()(0), 0.0);
}

TEST(RandomFourierWrenchEstimator, SixOutputAxesStayIndependent) {
  RandomFourierWrenchEstimator estimator(settings());
  vector6_t residual = vector6_t::Zero();
  residual(0) = 8.0;
  for (int sample = 0; sample < 200; ++sample) {
    estimator.step(residual, vector6_t::Zero(), 0.005);
  }

  EXPECT_GT(std::abs(estimator.previousWrenchEstimate()(0)), 1.0e-6);
  EXPECT_TRUE(estimator.previousWrenchEstimate().tail<5>().isZero(0.0));
  EXPECT_TRUE(estimator.weights().rightCols<5>().isZero(0.0));
}

TEST(RandomFourierWrenchEstimator, ZeroDtDoesNotUpdateWeights) {
  RandomFourierWrenchEstimator estimator(settings());
  vector6_t residual = vector6_t::Ones();
  const auto result = estimator.step(residual, vector6_t::Ones(), 0.0);
  EXPECT_TRUE(estimator.weights().isZero(0.0));
  EXPECT_TRUE(result.estimateAfterUpdate.isZero(0.0));
}

TEST(RandomFourierWrenchEstimator, RejectsInvalidSettings) {
  auto value = settings();
  value.featureCount = 0;
  EXPECT_THROW(RandomFourierWrenchEstimator{value}, std::invalid_argument);

  value = settings();
  value.frequencyStd = 0.0;
  EXPECT_THROW(RandomFourierWrenchEstimator{value}, std::invalid_argument);

  value = settings();
  value.learningRate(2) = -0.1;
  EXPECT_THROW(RandomFourierWrenchEstimator{value}, std::invalid_argument);

  value = settings();
  value.outputScale(0) = std::numeric_limits<scalar_t>::quiet_NaN();
  EXPECT_THROW(RandomFourierWrenchEstimator{value}, std::invalid_argument);
}

}}}}  // namespace ocs2::legged_robot_adaptive::adaptive
