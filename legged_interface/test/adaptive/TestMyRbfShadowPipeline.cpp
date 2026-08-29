#include <gtest/gtest.h>

#include "legged_interface/adaptive/MyRbfShadowPipeline.h"

namespace legged {
namespace adaptive {
namespace {

MyRbfShadowSettings makePipelineEstimatorSettings() {
  MyRbfShadowSettings settings;
  settings.residualScale = vector6_t::Constant(10.0);
  settings.discrepancyScale = vector6_t::Constant(10.0);
  settings.outputScale = vector6_t::Ones();
  settings.centers = matrix_t::Zero(
      MyRbfShadowEstimator::kAxisCount,
      MyRbfShadowEstimator::kInputDimension *
          MyRbfShadowEstimator::kGaussianNodeCount);
  settings.widths = matrix_t::Ones(
      MyRbfShadowEstimator::kAxisCount,
      MyRbfShadowEstimator::kGaussianNodeCount);
  settings.predictionGain = vector6_t::Ones();
  return settings;
}

TEST(MyRbfShadowPipeline, UpdatesOnlyAfterAnAlignedPredictionIsObserved) {
  MyRbfShadowPipeline pipeline(10.0, 24, true, 0.5, 1e-6,
                               makePipelineEstimatorSettings());
  vector_t actualState = vector_t::Zero(24);
  EXPECT_FALSE(
      pipeline.observeActual(1.0, actualState, vector6_t::Zero()).valid);

  pipeline.setPrediction(1.1, 0.1, vector_t::Zero(24));
  actualState(0) = 0.1;
  const auto result =
      pipeline.observeActual(1.1, actualState, vector6_t::Zero());

  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.residual.rawResidual(0), -10.0, 1e-12);
  EXPECT_LT(result.estimator.estimateAfterUpdate(0), 0.0);
  EXPECT_TRUE(result.estimator.feature.allFinite());
}

TEST(MyRbfShadowPipeline, ResetDropsPredictionAndLearnedEstimate) {
  MyRbfShadowPipeline pipeline(10.0, 24, false, 0.0, 1e-6,
                               makePipelineEstimatorSettings());
  vector_t actualState = vector_t::Zero(24);
  actualState(0) = 0.1;
  pipeline.setPrediction(1.1, 0.1, vector_t::Zero(24));
  ASSERT_TRUE(
      pipeline.observeActual(1.1, actualState, vector6_t::Zero()).valid);
  ASSERT_FALSE(pipeline.estimate().isZero());

  pipeline.reset();

  EXPECT_TRUE(pipeline.estimate().isZero());
  EXPECT_FALSE(
      pipeline.observeActual(1.2, actualState, vector6_t::Zero()).valid);
  EXPECT_FALSE(pipeline.controlInjectionEnabled());
}

TEST(MyRbfShadowPipeline, UsesSuppliedTrackingErrorForEthUpdate) {
  auto settings = makePipelineEstimatorSettings();
  settings.predictionGain.setZero();
  settings.trackingGain.setOnes();
  MyRbfShadowPipeline pipeline(10.0, 24, false, 0.0, 1e-6,
                               std::move(settings));
  const vector_t actualState = vector_t::Zero(24);
  pipeline.setPrediction(1.1, 0.1, vector_t::Zero(24));
  vector6_t sigma = vector6_t::Zero();
  sigma(1) = 2.0;

  const auto result = pipeline.observeActual(1.1, actualState, sigma);

  ASSERT_TRUE(result.valid);
  EXPECT_GT(result.estimator.estimateAfterUpdate(1), 0.0);
  EXPECT_TRUE(result.estimator.estimateAfterUpdate.head<1>().isZero(1e-12));
  EXPECT_TRUE(result.estimator.estimateAfterUpdate.tail<4>().isZero(1e-12));
}

TEST(MyRbfShadowPipeline,
     LearnsExternalRatherThanNetWrenchWithKnownCompensation) {
  MyRbfShadowPipeline pipeline(10.0, 24, false, 0.0, 1e-6,
                               makePipelineEstimatorSettings());
  vector_t actualState = vector_t::Zero(24);
  actualState(1) = 0.16;
  vector6_t appliedCompensation = vector6_t::Zero();
  appliedCompensation(1) = -4.0;
  pipeline.setPrediction(1.1, 0.1, vector_t::Zero(24),
                         appliedCompensation);

  const auto result =
      pipeline.observeActual(1.1, actualState, vector6_t::Zero());

  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.residual.netResidual(1), -16.0, 1e-12);
  EXPECT_NEAR(result.residual.rawResidual(1), -20.0, 1e-12);
  EXPECT_LT(result.estimator.estimateAfterUpdate(1), 0.0);
}

}  // namespace
}  // namespace adaptive
}  // namespace legged
