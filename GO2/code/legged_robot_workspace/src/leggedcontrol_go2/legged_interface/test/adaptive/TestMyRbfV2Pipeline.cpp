#include <gtest/gtest.h>

#include "legged_interface/adaptive/MyRbfV2Pipeline.h"

namespace legged {
namespace adaptive {
namespace {

TEST(MyRbfV2Pipeline, IntegratesAllNodesAndReturnsOpposingCompensation) {
  v2::Settings settings;
  settings.gamma.fill(0.1);
  settings.robustGamma.fill(0.2);
  settings.weightLimit.fill(2.0);
  MyRbfV2Pipeline pipeline(10.0, 12, false, 0.0, 1e-6, settings);

  vector_t predicted = vector_t::Zero(12);
  pipeline.setPrediction(0.01, 0.01, predicted);
  const auto result = pipeline.observeActual(
      0.01, predicted, vector6_t::Constant(0.2));

  ASSERT_TRUE(result.valid);
  EXPECT_GT(result.estimateAfterUpdate(0), 0.0);
  EXPECT_GT(result.robustWrench(0), 0.0);
  EXPECT_GT(result.robustBound(0), 0.0);
  EXPECT_LT(pipeline.compensationWrench()(0), 0.0);
  EXPECT_GT(result.weightDerivativeNorm(0), 0.0);
  EXPECT_GT(result.coverage(0), 0.0);
}

TEST(MyRbfV2Pipeline, ResetClearsAdaptiveStateAndPendingPrediction) {
  MyRbfV2Pipeline pipeline(10.0, 12, false, 0.0, 1e-6, v2::Settings{});
  vector_t state = vector_t::Zero(12);
  pipeline.setPrediction(0.01, 0.01, state);
  ASSERT_TRUE(pipeline.observeActual(
      0.01, state, vector6_t::Constant(0.2)).valid);

  pipeline.reset();

  EXPECT_TRUE(pipeline.compensationWrench().isZero(1e-12));
  for (const auto& axis : pipeline.state().weights) {
    for (double weight : axis) {
      EXPECT_DOUBLE_EQ(weight, 0.0);
    }
  }
  for (double bound : pipeline.state().robustBound) {
    EXPECT_DOUBLE_EQ(bound, 0.0);
  }
  EXPECT_FALSE(pipeline.observeActual(
      0.02, state, vector6_t::Zero()).valid);
}

}  // namespace
}  // namespace adaptive
}  // namespace legged
