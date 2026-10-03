#include <gtest/gtest.h>

#include <limits>

#include "legged_interface/adaptive/modules/AdaptiveModuleShadowPipeline.h"

namespace legged {
namespace adaptive {
namespace modules {
namespace {

AdaptiveModuleShadowSettings isolatedSettings() {
  AdaptiveModuleShadowSettings settings;
  settings.enablePeriodic = false;
  settings.enablePayload = false;
  settings.enableResidualDynamics = false;
  settings.enableFeatureAdvisor = false;
  settings.slowWrenchMode = SlowWrenchMode::MyRbfOnly;
  settings.confidence.riseTime = 0.0;
  settings.confidence.fallTime = 0.0;
  settings.confidence.contactTransitionHoldTime = 0.0;
  settings.confidence.minimumInjectionConfidence = 0.0;
  settings.confidence.fullInjectionConfidence = 0.0;
  settings.confidence.minimumCoverage = 0.0;
  settings.confidence.fullCoverage = 0.0;
  settings.confidence.residualDeadband.setZero();
  settings.confidence.maximumWrench.setConstant(1000.0);
  settings.confidence.maximumEstimateRate.setConstant(1.0e9);
  settings.confidence.maximumGatedWrenchRate.setConstant(1.0e9);
  return settings;
}

AdaptiveModuleShadowInput validInput() {
  AdaptiveModuleShadowInput input;
  input.time = 1.0;
  input.dt = 0.01;
  input.dataValid = true;
  input.coverage.setOnes();
  return input;
}

TEST(AdaptiveModuleShadowPipeline, MyRbfOnlyPathIsShadowAndGatesCandidate) {
  AdaptiveModuleShadowPipeline pipeline(isolatedSettings());
  auto input = validInput();
  input.physicalResidual << 1.0, -2.0, 3.0, -0.4, 0.5, -0.6;
  input.myRbfRawEstimate = input.physicalResidual;

  const auto result = pipeline.step(input);

  EXPECT_TRUE(result.inputValid);
  EXPECT_TRUE(result.shadowOnly);
  EXPECT_TRUE(result.slowWrench.isApprox(input.myRbfRawEstimate, 0.0));
  EXPECT_TRUE(result.candidateWrench.isApprox(input.myRbfRawEstimate, 0.0));
  EXPECT_TRUE(result.gatedWrench.isApprox(input.myRbfRawEstimate, 1e-12));
  EXPECT_TRUE(result.confidenceResult.gatedWrench.isApprox(result.gatedWrench,
                                                           0.0));
}

TEST(AdaptiveModuleShadowPipeline, InvalidCoreDataResetsAndReturnsZero) {
  AdaptiveModuleShadowPipeline pipeline(isolatedSettings());
  auto input = validInput();
  input.physicalResidual.setConstant(5.0);
  input.myRbfRawEstimate.setConstant(5.0);
  ASSERT_TRUE(pipeline.step(input).inputValid);

  input.physicalResidual(2) = std::numeric_limits<double>::quiet_NaN();
  const auto result = pipeline.step(input);

  EXPECT_FALSE(result.inputValid);
  EXPECT_TRUE(result.shadowOnly);
  EXPECT_TRUE(result.candidateWrench.isZero(0.0));
  EXPECT_TRUE(result.gatedWrench.isZero(0.0));
  EXPECT_FALSE(result.confidenceResult.valid);
}

TEST(AdaptiveModuleShadowPipeline,
     PayloadReplacesRatherThanDuplicatesSlowWrench) {
  auto settings = isolatedSettings();
  settings.enablePayload = true;
  settings.slowWrenchMode = SlowWrenchMode::PayloadOnly;
  settings.payload.adaptationGain.setConstant(100.0);
  settings.payload.forgettingRate = 0.0;
  settings.payload.informationForgettingRate = 0.0;
  AdaptiveModuleShadowPipeline pipeline(settings);

  auto input = validInput();
  input.physicalResidual(2) = -2.0 * 9.81;
  input.myRbfRawEstimate = input.physicalResidual;
  input.payloadInput.gravityDirection << 0.0, 0.0, -1.0;

  const auto result = pipeline.step(input);

  ASSERT_TRUE(result.payloadResult.inputValid);
  EXPECT_NEAR(result.payloadResult.deltaMass, 2.0, 1e-6);
  EXPECT_TRUE(result.slowWrench.isApprox(input.physicalResidual, 1e-6));
  EXPECT_TRUE(result.candidateWrench.isApprox(input.physicalResidual, 1e-6));
  EXPECT_FALSE(result.candidateWrench.isApprox(
      2.0 * input.physicalResidual, 1e-6));
}

TEST(AdaptiveModuleShadowPipeline,
     ConfidenceBlendFallsBackToMyRbfWhenPayloadIsInvalid) {
  auto settings = isolatedSettings();
  settings.enablePayload = true;
  settings.slowWrenchMode = SlowWrenchMode::ConfidenceBlend;
  AdaptiveModuleShadowPipeline pipeline(settings);

  auto input = validInput();
  input.physicalResidual.setConstant(3.0);
  input.myRbfRawEstimate.setConstant(2.0);
  input.payloadInput.hasAngularVelocity = true;
  input.payloadInput.angularVelocity.setConstant(10.0);

  const auto result = pipeline.step(input);

  EXPECT_FALSE(result.payloadResult.inputValid);
  EXPECT_NEAR(result.payloadBlend, 0.0, 0.0);
  EXPECT_TRUE(result.slowWrench.isApprox(input.myRbfRawEstimate, 0.0));
}

TEST(AdaptiveModuleShadowPipeline, FeatureAdviceIsRecordedButNeverAdded) {
  auto settings = isolatedSettings();
  settings.enableFeatureAdvisor = true;
  settings.feature.minimumSamples = 1;
  settings.feature.publishEverySamples = 1;
  AdaptiveModuleShadowPipeline pipeline(settings);

  auto input = validInput();
  input.physicalResidual.setConstant(4.0);
  input.myRbfRawEstimate.setConstant(4.0);
  input.featureInput.causalResidual.fill(100.0);

  const auto result = pipeline.step(input);

  EXPECT_TRUE(result.featureSuggestion.ready);
  EXPECT_EQ(result.featureSuggestion.sampleCount, 1u);
  EXPECT_NEAR(result.featureSuggestion.mean[0], 100.0, 0.0);
  EXPECT_TRUE(result.candidateWrench.isApprox(input.myRbfRawEstimate, 0.0));
}

TEST(AdaptiveModuleShadowPipeline,
     PeriodicModuleReceivesResidualAfterSlowPart) {
  auto settings = isolatedSettings();
  settings.enablePeriodic = true;
  settings.periodic.frequencyHz.fill(1.0);
  settings.periodic.minimumObservedPeriods = 0.5;
  AdaptiveModuleShadowPipeline pipeline(settings);

  auto input = validInput();
  input.time = 0.0;
  input.physicalResidual.setConstant(6.0);
  input.myRbfRawEstimate.setConstant(6.0);
  input.featureInput.causalResidual.fill(50.0);
  pipeline.step(input);
  input.time = 1.0;
  const auto result = pipeline.step(input);

  EXPECT_TRUE(result.periodicResult.inputValid);
  EXPECT_TRUE(result.residualAfterSlow.isZero(1e-12));
  EXPECT_TRUE(result.periodicComponent.isZero(1e-9));
  EXPECT_TRUE(result.candidateWrench.isApprox(input.myRbfRawEstimate, 1e-9));
}

TEST(AdaptiveModuleShadowPipeline,
     ResidualDynamicsContributesOnlyItsTrustedPrediction) {
  auto settings = isolatedSettings();
  settings.enableResidualDynamics = true;
  settings.residualDynamics.learningRate.setOnes();
  settings.residualDynamics.minimumSamplesForConfidence = 0;
  settings.residualDynamics.confidenceRiseSamples = 1;
  AdaptiveModuleShadowPipeline pipeline(settings);

  auto input = validInput();
  input.physicalResidual << 1.0, 2.0, 3.0, 0.1, 0.2, 0.3;

  const auto result = pipeline.step(input);

  EXPECT_TRUE(result.dynamicsResult.learningEnabled);
  EXPECT_TRUE(result.dynamicsResult.confidence.isApprox(vector6_t::Ones(),
                                                        0.0));
  EXPECT_TRUE(result.dynamicsComponent.isApprox(
      result.dynamicsResult.predictedNext, 1e-12));
  EXPECT_TRUE(result.candidateWrench.isApprox(result.dynamicsComponent,
                                              1e-12));
}

TEST(AdaptiveModuleShadowPipeline, ResetClearsAllLearnedContributions) {
  auto settings = isolatedSettings();
  settings.enableResidualDynamics = true;
  settings.residualDynamics.minimumSamplesForConfidence = 0;
  settings.residualDynamics.confidenceRiseSamples = 1;
  AdaptiveModuleShadowPipeline pipeline(settings);
  auto input = validInput();
  input.physicalResidual.setConstant(2.0);
  ASSERT_FALSE(pipeline.step(input).dynamicsComponent.isZero(1e-9));

  pipeline.reset(0.0);
  input.time = 0.0;
  input.physicalResidual.setZero();
  const auto result = pipeline.step(input);

  EXPECT_TRUE(result.dynamicsResult.predictedCurrent.isZero(0.0));
  EXPECT_TRUE(result.candidateWrench.isZero(0.0));
  EXPECT_TRUE(result.gatedWrench.isZero(0.0));
}

}  // namespace
}  // namespace modules
}  // namespace adaptive
}  // namespace legged
