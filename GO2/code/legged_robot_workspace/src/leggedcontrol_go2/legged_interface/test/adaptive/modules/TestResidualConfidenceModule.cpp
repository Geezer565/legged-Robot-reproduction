#include <gtest/gtest.h>

#include "legged_interface/adaptive/modules/ResidualConfidenceModule.h"

#include <algorithm>
#include <cmath>

namespace legged {
namespace adaptive {
namespace modules {
namespace {

ResidualConfidenceSettings makeSettings() {
  ResidualConfidenceSettings settings;
  settings.riseTime = 1.0;
  settings.fallTime = 0.02;
  settings.contactTransitionHoldTime = 0.0;
  settings.minimumInjectionConfidence = 0.0;
  settings.fullInjectionConfidence = 0.9;
  settings.minimumCoverage = 0.2;
  settings.fullCoverage = 0.8;
  settings.residualDeadband.setConstant(0.1);
  settings.maximumWrench.setConstant(100.0);
  settings.maximumEstimateRate.setConstant(1.0e6);
  settings.maximumGatedWrenchRate.setConstant(1.0e6);
  return settings;
}

ResidualConfidenceInput makeNominalInput(double dt = 0.01) {
  ResidualConfidenceInput input;
  input.rawEstimate.setConstant(10.0);
  input.residual.setConstant(10.0);
  input.coverage.setOnes();
  input.contactChanged = false;
  input.dataValid = true;
  input.dt = dt;
  return input;
}

void settle(ResidualConfidenceModule& module,
            const ResidualConfidenceInput& input,
            int steps = 1000) {
  for (int step = 0; step < steps; ++step) {
    module.step(input);
  }
}

TEST(ResidualConfidenceModule, InvalidDataImmediatelyZerosAllAxes) {
  ResidualConfidenceModule module(makeSettings());
  auto input = makeNominalInput();
  settle(module, input);
  ASSERT_GT(module.confidence().minCoeff(), 0.99);

  input.dataValid = false;
  const auto output = module.step(input);

  EXPECT_FALSE(output.valid);
  EXPECT_TRUE(output.confidence.isZero(0.0));
  EXPECT_TRUE(output.gatedWrench.isZero(0.0));
}

TEST(ResidualConfidenceModule, ContactChangeFallsFasterThanReliableDataRecovers) {
  ResidualConfidenceModule module(makeSettings());
  auto input = makeNominalInput();
  settle(module, input);

  const double before = module.confidence()(0);
  input.contactChanged = true;
  input.dt = 0.05;
  const double afterDrop = module.step(input).confidence(0);

  input.contactChanged = false;
  const double afterRecovery = module.step(input).confidence(0);

  const double drop = before - afterDrop;
  const double recovery = afterRecovery - afterDrop;
  EXPECT_GT(drop, recovery * 10.0);
  EXPECT_GT(afterRecovery, afterDrop);
  EXPECT_LT(afterRecovery, before);
}

TEST(ResidualConfidenceModule, CoverageAndDirectionChecksArePerAxis) {
  ResidualConfidenceModule module(makeSettings());
  auto input = makeNominalInput();
  settle(module, input);

  input.dt = 0.05;
  input.coverage(0) = 0.0;
  input.residual(1) = -input.rawEstimate(1);
  const auto output = module.step(input);

  EXPECT_LT(output.confidence(0), 0.2);
  EXPECT_LT(output.confidence(1), 0.2);
  EXPECT_GT(output.confidence(2), 0.99);
  EXPECT_LT(std::abs(output.gatedWrench(0)),
            std::abs(output.gatedWrench(2)));
  EXPECT_LT(std::abs(output.gatedWrench(1)),
            std::abs(output.gatedWrench(2)));
}

TEST(ResidualConfidenceModule, SuddenEstimateJumpLowersConfidence) {
  auto settings = makeSettings();
  settings.maximumEstimateRate.setConstant(10.0);
  ResidualConfidenceModule module(settings);
  auto input = makeNominalInput(0.01);
  input.rawEstimate.setConstant(1.0);
  input.residual = input.rawEstimate;
  settle(module, input);
  const double before = module.confidence()(0);

  input.dt = 0.1;
  input.rawEstimate.setConstant(50.0);
  input.residual = input.rawEstimate;
  const auto output = module.step(input);

  EXPECT_LT(output.confidence(0), before * 0.2);
  EXPECT_TRUE(output.rateLimited[0]);
}

TEST(ResidualConfidenceModule, GatedWrenchObeysMagnitudeAndSlewLimits) {
  auto settings = makeSettings();
  settings.riseTime = 1.0e-6;
  settings.fallTime = 1.0e-6;
  settings.maximumWrench.setConstant(10.0);
  settings.maximumEstimateRate.setConstant(1.0e9);
  settings.maximumGatedWrenchRate.setConstant(2.0);
  ResidualConfidenceModule module(settings);
  auto input = makeNominalInput(0.1);
  input.rawEstimate.setConstant(50.0);
  input.residual = input.rawEstimate;

  vector6_t previous = vector6_t::Zero();
  for (int step = 0; step < 100; ++step) {
    const auto output = module.step(input);
    EXPECT_LE(output.gatedWrench.cwiseAbs().maxCoeff(), 10.0 + 1.0e-12);
    EXPECT_LE((output.gatedWrench - previous).cwiseAbs().maxCoeff(),
              0.2 + 1.0e-12);
    previous = output.gatedWrench;
  }

  input.rawEstimate.setConstant(-50.0);
  input.residual = input.rawEstimate;
  for (int step = 0; step < 100; ++step) {
    const auto output = module.step(input);
    EXPECT_LE(output.gatedWrench.cwiseAbs().maxCoeff(), 10.0 + 1.0e-12);
    EXPECT_LE((output.gatedWrench - previous).cwiseAbs().maxCoeff(),
              0.2 + 1.0e-12);
    previous = output.gatedWrench;
  }
}

TEST(ResidualConfidenceModule, ResetClearsConfidenceAndOutputHistory) {
  ResidualConfidenceModule module(makeSettings());
  const auto input = makeNominalInput();
  settle(module, input);
  ASSERT_GT(module.confidence().minCoeff(), 0.99);

  module.reset();

  EXPECT_TRUE(module.confidence().isZero(0.0));
  EXPECT_TRUE(module.gatedWrench().isZero(0.0));
  const auto first = module.step(input);
  EXPECT_LT(first.confidence.maxCoeff(), 0.02);
}

TEST(ResidualConfidenceModule, NonFiniteInputIsRejectedImmediately) {
  ResidualConfidenceModule module(makeSettings());
  auto input = makeNominalInput();
  settle(module, input);
  input.residual(3) = std::numeric_limits<double>::quiet_NaN();

  const auto output = module.step(input);

  EXPECT_FALSE(output.valid);
  EXPECT_TRUE(output.confidence.isZero(0.0));
  EXPECT_TRUE(output.gatedWrench.isZero(0.0));
}

}  // namespace
}  // namespace modules
}  // namespace adaptive
}  // namespace legged
