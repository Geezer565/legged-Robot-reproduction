#include <gtest/gtest.h>

#include "legged_interface/adaptive/RandomFourierWrenchEstimator.h"

namespace legged {
namespace adaptive {
namespace {

RandomFourierWrenchEstimatorSettings settings() {
  RandomFourierWrenchEstimatorSettings value;
  value.residualScale.setOnes();
  value.discrepancyScale.setOnes();
  value.outputScale.setOnes();
  value.learningRate.setConstant(0.05);
  value.featureCount = 16;
  value.randomSeed = 42;
  value.frequencyStd = 1.0;
  value.projectionRadius = 0.0;
  return value;
}

TEST(RandomFourierWrenchEstimator, FixedSeedProducesIdenticalOnlineUpdates) {
  RandomFourierWrenchEstimator left(settings());
  RandomFourierWrenchEstimator right(settings());
  vector6_t residual;
  residual << 0.4, -0.3, 0.2, -0.1, 0.05, -0.02;

  const auto a = left.step(residual, 0.01);
  const auto b = right.step(residual, 0.01);

  EXPECT_TRUE(a.feature.isApprox(b.feature, 0.0));
  EXPECT_TRUE(a.estimateAfterUpdate.isApprox(b.estimateAfterUpdate, 0.0));
}

TEST(RandomFourierWrenchEstimator, PositiveSingleAxisResidualDoesNotUpdateOtherAxes) {
  RandomFourierWrenchEstimator estimator(settings());
  vector6_t residual = vector6_t::Zero();
  residual[1] = 1.0;

  const auto result = estimator.step(residual, 0.02);

  EXPECT_GT(result.estimateAfterUpdate[1], 0.0);
  for (int axis = 0; axis < 6; ++axis) {
    if (axis != 1) {
      EXPECT_DOUBLE_EQ(result.estimateAfterUpdate[axis], 0.0);
    }
  }
}

}  // namespace
}  // namespace adaptive
}  // namespace legged
