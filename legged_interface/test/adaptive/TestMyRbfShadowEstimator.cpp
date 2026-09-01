#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>

#include "legged_interface/adaptive/MyRbfShadowEstimator.h"

namespace legged {
namespace adaptive {
namespace {

MyRbfShadowSettings makeSettings() {
  MyRbfShadowSettings settings;
  settings.residualScale = vector6_t::Constant(2.0);
  settings.discrepancyScale = vector6_t::Constant(4.0);
  settings.outputScale = vector6_t::Ones();
  settings.centers = matrix_t::Zero(
      MyRbfShadowEstimator::kAxisCount,
      MyRbfShadowEstimator::kInputDimension *
          MyRbfShadowEstimator::kGaussianNodeCount);
  settings.widths = matrix_t::Ones(
      MyRbfShadowEstimator::kAxisCount,
      MyRbfShadowEstimator::kGaussianNodeCount);
  settings.trackingGain = vector6_t::Zero();
  settings.predictionGain = vector6_t::Ones();
  settings.leakage = vector6_t::Zero();
  return settings;
}

TEST(MyRbfShadowEstimator, BuildsTheTwoFrozenInputFeaturesPerAxis) {
  MyRbfShadowEstimator estimator(makeSettings());
  vector6_t residual;
  residual << 2.0, -4.0, 6.0, -8.0, 10.0, -12.0;

  const auto first = estimator.step(residual, vector6_t::Zero(), 0.0);

  EXPECT_TRUE(first.feature.row(0).isApprox((residual / 2.0).transpose()));
  EXPECT_TRUE(first.feature.row(1).isApprox((residual / 4.0).transpose()));
  EXPECT_TRUE(first.estimateBeforeUpdate.isZero());

  estimator.step(residual, vector6_t::Zero(), 0.1);
  const vector6_t previousEstimate = estimator.estimate();
  const auto next = estimator.step(residual, vector6_t::Zero(), 0.0);
  EXPECT_TRUE(next.feature.row(1).isApprox(
      ((residual - previousEstimate) / 4.0).transpose()));
}

TEST(MyRbfShadowEstimator, SeparatesEthTrackingAndV3PredictionUpdates) {
  auto v3Settings = makeSettings();
  v3Settings.predictionGain = vector6_t::Constant(2.0);
  MyRbfShadowEstimator v3(v3Settings);

  auto ethSettings = makeSettings();
  ethSettings.predictionGain.setZero();
  ethSettings.trackingGain = vector6_t::Constant(3.0);
  MyRbfShadowEstimator eth(ethSettings);

  const vector6_t residual = vector6_t::Constant(2.0);
  const vector6_t sigma = vector6_t::Constant(0.5);
  const auto v3Step = v3.step(residual, sigma, 0.1);
  const auto ethStep = eth.step(residual, sigma, 0.1);

  EXPECT_NEAR(v3Step.weightDerivative(0, 0), 4.0, 1e-12);
  EXPECT_NEAR(ethStep.weightDerivative(0, 0), 1.5, 1e-12);
  EXPECT_GT(v3Step.estimateAfterUpdate(0), ethStep.estimateAfterUpdate(0));
}

TEST(MyRbfShadowEstimator, BiasOnlyTrackingUpdatesOnlyTheConstantNode) {
  auto settings = makeSettings();
  settings.predictionGain.setZero();
  settings.trackingGain = vector6_t::Constant(2.0);
  settings.trackingBiasOnly = true;
  MyRbfShadowEstimator estimator(settings);

  const auto result = estimator.step(vector6_t::Zero(),
                                     vector6_t::Constant(0.5), 0.1);

  EXPECT_NEAR(result.weightDerivative(0, 0), 1.0, 1e-12);
  EXPECT_TRUE(result.weightDerivative.block(
      1, 0, MyRbfShadowEstimator::kGaussianNodeCount,
      MyRbfShadowEstimator::kAxisCount).isZero(1e-12));
}

TEST(MyRbfShadowEstimator, PredictionStillUpdatesGaussianNodes) {
  auto settings = makeSettings();
  settings.trackingBiasOnly = true;
  settings.predictionGain = vector6_t::Constant(2.0);
  MyRbfShadowEstimator estimator(settings);

  const auto result = estimator.step(vector6_t::Constant(2.0),
                                     vector6_t::Zero(), 0.1);

  EXPECT_GT(result.weightDerivative(1, 0), 0.0);
  EXPECT_GT(result.weightsAfterUpdate(1, 0), 0.0);
}

TEST(MyRbfShadowEstimator, NormalizedUpdateUsesTheFullPhysicalGradientNorm) {
  auto settings = makeSettings();
  settings.normalizeUpdate = true;
  settings.updateEpsilon = 0.5;
  MyRbfShadowEstimator estimator(settings);

  const vector6_t residual = vector6_t::Constant(2.0);
  const auto result = estimator.step(residual, vector6_t::Zero(), 0.0);
  const vector_t physicalGradient = result.basis.col(0);
  const vector_t expected =
      residual(0) * physicalGradient /
      (settings.updateEpsilon + physicalGradient.squaredNorm());

  EXPECT_TRUE(result.weightDerivative.col(0).isApprox(expected, 1e-12));
}

TEST(MyRbfShadowEstimator, GateAndCoverageCanFreezeLearning) {
  auto gatedSettings = makeSettings();
  gatedSettings.trackingGain = vector6_t::Ones();
  MyRbfShadowEstimator gated(gatedSettings);
  const auto gateResult = gated.step(vector6_t::Ones(), vector6_t::Ones(),
                                     0.1, 0.0);
  EXPECT_TRUE(gateResult.weightDerivative.isZero(1e-12));
  EXPECT_TRUE(gateResult.weightsAfterUpdate.isZero(1e-12));
  EXPECT_TRUE(gateResult.updateFrozen[0]);

  auto coverageSettings = makeSettings();
  coverageSettings.minimumCoverage = 0.9;
  MyRbfShadowEstimator coverageLimited(coverageSettings);
  const auto coverageResult = coverageLimited.step(
      vector6_t::Constant(100.0), vector6_t::Zero(), 0.1);
  EXPECT_LT(coverageResult.coverage(0), coverageSettings.minimumCoverage);
  EXPECT_TRUE(coverageResult.weightDerivative.isZero(1e-12));
  EXPECT_TRUE(coverageResult.updateFrozen[0]);
}

TEST(MyRbfShadowEstimator, ProjectionBoundsEveryWeight) {
  auto settings = makeSettings();
  settings.predictionGain = vector6_t::Constant(100.0);
  settings.weightLimit = vector6_t::Constant(0.05);
  MyRbfShadowEstimator estimator(settings);

  const auto result = estimator.step(vector6_t::Constant(10.0),
                                     vector6_t::Zero(), 1.0);

  EXPECT_LE(result.weightsAfterUpdate.cwiseAbs().maxCoeff(), 0.05 + 1e-12);
  EXPECT_NEAR(result.weightsAfterUpdate(0, 0), 0.05, 1e-12);
  EXPECT_TRUE(result.projectionActive[1]);
}

TEST(MyRbfShadowEstimator, SeparatesPredictionTrackingAndLeakageDiagnostics) {
  auto predictionSettings = makeSettings();
  predictionSettings.trackingGain.setZero();
  predictionSettings.leakage.setZero();
  MyRbfShadowEstimator predictionOnly(predictionSettings);
  const auto prediction = predictionOnly.step(vector6_t::Constant(2.0),
                                               vector6_t::Zero(), 0.0);
  EXPECT_GT(prediction.predictionDerivativeNorm(1), 0.0);
  EXPECT_DOUBLE_EQ(prediction.trackingBiasDerivative(1), 0.0);
  EXPECT_DOUBLE_EQ(prediction.leakageDerivativeNorm(1), 0.0);

  auto trackingSettings = makeSettings();
  trackingSettings.predictionGain.setZero();
  trackingSettings.trackingGain = vector6_t::Constant(2.0);
  trackingSettings.trackingBiasOnly = true;
  MyRbfShadowEstimator trackingOnly(trackingSettings);
  const auto tracking = trackingOnly.step(vector6_t::Zero(),
                                           vector6_t::Constant(0.5), 0.0);
  EXPECT_NE(tracking.trackingBiasDerivative(1), 0.0);
  EXPECT_DOUBLE_EQ(tracking.predictionDerivativeNorm(1), 0.0);
  EXPECT_DOUBLE_EQ(tracking.leakageDerivativeNorm(1), 0.0);

  auto leakageSettings = makeSettings();
  leakageSettings.predictionGain.setZero();
  leakageSettings.trackingGain = vector6_t::Ones();
  leakageSettings.leakage = vector6_t::Constant(0.5);
  MyRbfShadowEstimator leakageOnly(leakageSettings);
  leakageOnly.step(vector6_t::Zero(), vector6_t::Ones(), 0.1);
  const auto leakage = leakageOnly.step(vector6_t::Zero(),
                                         vector6_t::Zero(), 0.0);
  EXPECT_GT(leakage.leakageDerivativeNorm(1), 0.0);
  EXPECT_DOUBLE_EQ(leakage.predictionDerivativeNorm(1), 0.0);
  EXPECT_DOUBLE_EQ(leakage.trackingBiasDerivative(1), 0.0);
}

TEST(MyRbfShadowEstimator, DiagnosticsLeaveTheFrozenUpdateEquationUnchanged) {
  auto settings = makeSettings();
  settings.outputScale << 1.0, 1.2, 0.8, 1.5, 0.9, 1.1;
  settings.trackingGain = vector6_t::Constant(0.7);
  settings.predictionGain = vector6_t::Constant(1.3);
  settings.leakage = vector6_t::Constant(0.04);
  settings.normalizeUpdate = true;
  settings.trackingBiasOnly = true;
  settings.updateEpsilon = 0.2;
  settings.weightLimit = vector6_t::Constant(0.35);
  MyRbfShadowEstimator estimator(settings);

  matrix_t expectedWeights = matrix_t::Zero(
      MyRbfShadowEstimator::kBasisCount, MyRbfShadowEstimator::kAxisCount);
  vector6_t expectedPreviousEstimate = vector6_t::Zero();
  for (int step = 0; step < 100; ++step) {
    vector6_t residual;
    vector6_t sigma;
    for (Eigen::Index axis = 0; axis < MyRbfShadowEstimator::kAxisCount;
         ++axis) {
      residual(axis) = 0.2 * (step + 1) * (axis + 1);
      sigma(axis) = 0.05 * (axis + 1);
    }
    const scalar_t updateGate = step % 4 == 0 ? 0.0 : 0.75;
    const scalar_t dt = 0.002;
    const auto result = estimator.step(residual, sigma, dt, updateGate);

    vector6_t expectedBefore = vector6_t::Zero();
    matrix_t expectedDerivative = matrix_t::Zero(
        MyRbfShadowEstimator::kBasisCount, MyRbfShadowEstimator::kAxisCount);
    for (Eigen::Index axis = 0; axis < MyRbfShadowEstimator::kAxisCount;
         ++axis) {
      expectedBefore(axis) = settings.outputScale(axis) *
          expectedWeights.col(axis).dot(result.basis.col(axis));
      const vector_t physicalGradient =
          settings.outputScale(axis) * result.basis.col(axis);
      vector_t updateGradient = physicalGradient /
          (settings.updateEpsilon + physicalGradient.squaredNorm());
      const bool learningEnabled = updateGate > 0.0 &&
          result.coverage(axis) >= settings.minimumCoverage;
      if (learningEnabled) {
        const scalar_t biasGradient = settings.outputScale(axis) /
            (settings.updateEpsilon + settings.outputScale(axis) *
                settings.outputScale(axis));
        expectedDerivative(0, axis) += updateGate *
            settings.trackingGain(axis) * sigma(axis) * biasGradient;
        expectedDerivative.col(axis) += updateGate *
            settings.predictionGain(axis) *
            (residual(axis) - expectedBefore(axis)) * updateGradient;
      }
      expectedDerivative.col(axis) -=
          settings.leakage(axis) * expectedWeights.col(axis);
    }
    expectedWeights += dt * expectedDerivative;
    for (Eigen::Index axis = 0; axis < MyRbfShadowEstimator::kAxisCount;
         ++axis) {
      expectedWeights.col(axis) = expectedWeights.col(axis)
          .cwiseMax(-settings.weightLimit(axis))
          .cwiseMin(settings.weightLimit(axis));
    }

    vector6_t expectedAfter = vector6_t::Zero();
    for (Eigen::Index axis = 0; axis < MyRbfShadowEstimator::kAxisCount;
         ++axis) {
      expectedAfter(axis) = settings.outputScale(axis) *
          expectedWeights.col(axis).dot(result.basis.col(axis));
      EXPECT_DOUBLE_EQ(result.coverage(axis), result.basis
          .block(1, axis, MyRbfShadowEstimator::kGaussianNodeCount, 1)
          .maxCoeff());
      EXPECT_EQ(result.updateFrozen[static_cast<std::size_t>(axis)],
                updateGate == 0.0);
    }
    EXPECT_TRUE(result.estimateBeforeUpdate.isApprox(expectedBefore, 1e-12));
    EXPECT_TRUE(result.weightsAfterUpdate.isApprox(expectedWeights, 1e-12));
    EXPECT_TRUE(result.estimateAfterUpdate.isApprox(expectedAfter, 1e-12));
    EXPECT_TRUE(estimator.weights().isApprox(expectedWeights, 1e-12));
    EXPECT_TRUE(estimator.estimate().isApprox(expectedAfter, 1e-12));
    expectedPreviousEstimate = expectedAfter;
  }
  EXPECT_FALSE(expectedPreviousEstimate.isZero());
}

TEST(MyRbfShadowEstimator, LegacyDefaultsReproduceTheV4Derivative) {
  auto settings = makeSettings();
  settings.trackingGain = vector6_t::Constant(3.0);
  settings.predictionGain = vector6_t::Constant(2.0);
  settings.leakage = vector6_t::Constant(0.1);
  MyRbfShadowEstimator estimator(settings);

  estimator.step(vector6_t::Constant(1.0), vector6_t::Zero(), 0.1);
  const matrix_t weightsBefore = estimator.weights();
  const vector6_t residual = vector6_t::Constant(2.0);
  const vector6_t sigma = vector6_t::Constant(0.5);
  const auto result = estimator.step(residual, sigma, 0.0);

  for (Eigen::Index axis = 0; axis < MyRbfShadowEstimator::kAxisCount;
       ++axis) {
    const vector_t physicalGradient =
        settings.outputScale(axis) * result.basis.col(axis);
    const scalar_t predictionError =
        residual(axis) - result.estimateBeforeUpdate(axis);
    const vector_t expected =
        settings.trackingGain(axis) * sigma(axis) * physicalGradient +
        settings.predictionGain(axis) * predictionError * physicalGradient -
        settings.leakage(axis) * weightsBefore.col(axis);
    EXPECT_TRUE(result.weightDerivative.col(axis).isApprox(expected, 1e-12));
  }
}

TEST(MyRbfShadowEstimator, RejectsUnsafeSettingsAndSamples) {
  auto settings = makeSettings();
  settings.discrepancyScale(2) = 0.0;
  EXPECT_THROW(MyRbfShadowEstimator{settings}, std::invalid_argument);

  MyRbfShadowEstimator estimator(makeSettings());
  EXPECT_THROW(estimator.step(vector6_t::Zero(), vector6_t::Zero(), -0.01),
               std::invalid_argument);
  vector6_t invalid = vector6_t::Zero();
  invalid(0) = std::numeric_limits<scalar_t>::quiet_NaN();
  EXPECT_THROW(estimator.step(invalid, vector6_t::Zero(), 0.01),
               std::invalid_argument);

  settings = makeSettings();
  settings.updateEpsilon = 0.0;
  EXPECT_THROW(MyRbfShadowEstimator{settings}, std::invalid_argument);

  settings = makeSettings();
  settings.minimumCoverage = 1.01;
  EXPECT_THROW(MyRbfShadowEstimator{settings}, std::invalid_argument);

  settings = makeSettings();
  settings.weightLimit(4) = 0.0;
  EXPECT_THROW(MyRbfShadowEstimator{settings}, std::invalid_argument);

  EXPECT_THROW(estimator.step(vector6_t::Zero(), vector6_t::Zero(), 0.01,
                              1.01),
               std::invalid_argument);
}

TEST(MyRbfShadowEstimator, IsDiagnosticsOnlyAndCannotInjectControl) {
  static_assert(!MyRbfShadowEstimator::kControlInjectionEnabled,
                "The first Go2 migration must remain shadow-only.");
  MyRbfShadowEstimator estimator(makeSettings());
  EXPECT_FALSE(estimator.controlInjectionEnabled());
}

TEST(MyRbfShadowEstimator, BuildsTheFrozenFiveByFiveSymmetricBasis) {
  const vector6_t scales = vector6_t::Ones();
  const auto settings = createSymmetricGridMyRbfShadowSettings(
      2.0, 4.0, 0.85, scales, scales, scales,
      vector6_t::Zero(), vector6_t::Ones(), vector6_t::Zero());

  EXPECT_NEAR(settings.centers(0, 0), -2.0, 1e-12);
  EXPECT_NEAR(settings.centers(0, 1), -2.0, 1e-12);
  EXPECT_NEAR(settings.centers(0, 48), 2.0, 1e-12);
  EXPECT_NEAR(settings.centers(0, 49), 2.0, 1e-12);
  EXPECT_NEAR(settings.widths(0, 0), 0.85, 1e-12);
  EXPECT_NEAR(settings.centers(3, 48), 4.0, 1e-12);
  EXPECT_NEAR(settings.widths(3, 0), 1.7, 1e-12);
}

}  // namespace
}  // namespace adaptive
}  // namespace legged
