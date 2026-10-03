#include <gtest/gtest.h>

#include "ocs2_legged_robot_adaptive/adaptive/TwoFeatureMinnitiRbfWrenchEstimator.h"

namespace ocs2 { namespace legged_robot_adaptive { namespace adaptive { namespace {

TwoFeatureMinnitiRbfWrenchEstimatorSettings settings() {
  TwoFeatureMinnitiRbfWrenchEstimatorSettings settings;
  settings.network.residualScale.setOnes();
  settings.network.discrepancyScale.setOnes();
  settings.network.outputScale.setOnes();
  settings.network.centers = matrix_t::Zero(6, 50);
  settings.network.widths = matrix_t::Ones(6, 25);
  settings.gamma.setConstant(0.5);
  return settings;
}

TEST(TwoFeatureMinnitiRbfWrenchEstimator, PerformsOneIndependentFixedStepUpdate) {
  TwoFeatureMinnitiRbfWrenchEstimator estimator(settings());
  vector6_t sigma = vector6_t::Zero();
  sigma(0) = 2.0;

  const auto result = estimator.step(vector6_t::Zero(), sigma, 0.1);

  EXPECT_TRUE(result.feature.isZero());
  EXPECT_TRUE(result.estimateBeforeUpdate.isZero());
  EXPECT_TRUE(result.weightsAfterUpdate.col(0).isConstant(0.1));
  EXPECT_TRUE(result.weightsAfterUpdate.rightCols(5).isZero());
  EXPECT_NEAR(result.estimateAfterUpdate(0), 2.6, 1e-12);
  EXPECT_TRUE(result.estimateAfterUpdate.tail<5>().isZero());
}

TEST(TwoFeatureMinnitiRbfWrenchEstimator, ReusesPreviousEstimateWithoutSameCycleLoop) {
  TwoFeatureMinnitiRbfWrenchEstimator estimator(settings());
  vector6_t sigma = vector6_t::Zero();
  sigma(0) = 2.0;
  estimator.step(vector6_t::Zero(), sigma, 0.1);

  const auto result = estimator.step(vector6_t::Zero(), vector6_t::Zero(), 0.0);

  EXPECT_NEAR(result.feature(1, 0), -2.6, 1e-12);
  EXPECT_TRUE(result.feature.row(0).isZero());
  EXPECT_TRUE(result.feature.block(1, 1, 1, 5).isZero());
}

}}}}  // namespace ocs2::legged_robot_adaptive::adaptive
