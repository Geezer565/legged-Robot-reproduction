#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "legged_interface/adaptive/modules/PayloadIdModule.h"

namespace legged {
namespace adaptive {
namespace modules {
namespace {

PayloadIdSettings testSettings() {
  PayloadIdSettings settings;
  settings.adaptationGain << 4.0, 4.0, 4.0, 4.0;
  settings.forgettingRate = 0.0;
  settings.gravityMagnitude = 9.81;
  settings.minDeltaMass = 0.0;
  settings.maxDeltaMass = 8.0;
  settings.maxAbsDeltaCom << 0.15, 0.15, 0.20;
  settings.normalizationEpsilon = 1e-9;
  settings.confidenceInformationScale << 20.0, 20.0, 20.0, 20.0;
  settings.maximumAngularSpeedForAutomaticRegressor = 1.0;
  return settings;
}

PayloadIdInput staticInput(const vector6_t& wrench) {
  PayloadIdInput input;
  input.externalResidual = wrench;
  input.gravityDirection << 0.0, 0.0, -1.0;
  input.updateAllowed = true;
  return input;
}

TEST(PayloadIdModule, CenteredStaticLoadConvergesToMass) {
  PayloadIdModule module(testSettings());
  vector6_t wrench = vector6_t::Zero();
  wrench(2) = -2.0 * 9.81;

  PayloadIdOutput result;
  for (int sample = 0; sample < 1000; ++sample) {
    result = module.step(staticInput(wrench), 0.005);
  }

  EXPECT_TRUE(result.inputValid);
  EXPECT_TRUE(result.updated);
  EXPECT_NEAR(result.deltaMass, 2.0, 2e-3);
  EXPECT_TRUE(result.deltaCom.isZero(1e-9));
  EXPECT_TRUE(result.equivalentWrench.isApprox(wrench, 2e-2));
  EXPECT_GT(result.parameterConfidence(0), 0.9);
}

TEST(PayloadIdModule, StaticTorqueIdentifiesHorizontalCenterOfMass) {
  PayloadIdModule module(testSettings());
  const double mass = 2.0;
  vector3_t com;
  com << 0.10, -0.05, 0.0;
  vector3_t gravity;
  gravity << 0.0, 0.0, -9.81;
  vector6_t wrench = vector6_t::Zero();
  wrench.head<3>() = mass * gravity;
  wrench.tail<3>() = (mass * com).cross(gravity);

  PayloadIdOutput result;
  for (int sample = 0; sample < 1000; ++sample) {
    result = module.step(staticInput(wrench), 0.005);
  }

  EXPECT_NEAR(result.deltaMass, mass, 2e-3);
  EXPECT_NEAR(result.deltaCom.x(), com.x(), 2e-3);
  EXPECT_NEAR(result.deltaCom.y(), com.y(), 2e-3);
  EXPECT_NEAR(result.deltaCom.z(), 0.0, 1e-12);
  EXPECT_GT(result.parameterConfidence(1), 0.9);
  EXPECT_GT(result.parameterConfidence(2), 0.9);
  EXPECT_NEAR(result.parameterConfidence(3), 0.0, 1e-12);
}

TEST(PayloadIdModule, CustomRegressorCanIdentifyAllFourParameters) {
  PayloadIdModule module(testSettings());
  PayloadIdInput input;
  input.useCustomRegressor = true;
  input.customRegressor.setZero();
  input.customRegressor.topRows<4>().setIdentity();
  Eigen::Matrix<scalar_t, 4, 1> expected;
  expected << 3.0, 0.30, -0.15, 0.45;
  input.externalResidual = input.customRegressor * expected;

  PayloadIdOutput result;
  for (int sample = 0; sample < 1000; ++sample) {
    result = module.step(input, 0.005);
  }

  EXPECT_TRUE(result.usedCustomRegressor);
  EXPECT_NEAR(result.deltaMass, 3.0, 2e-3);
  EXPECT_NEAR(result.deltaCom.x(), 0.10, 2e-3);
  EXPECT_NEAR(result.deltaCom.y(), -0.05, 2e-3);
  EXPECT_NEAR(result.deltaCom.z(), 0.15, 2e-3);
  EXPECT_GT(result.parameterConfidence.minCoeff(), 0.19);
}

TEST(PayloadIdModule, InvalidSampleDoesNotChangeEstimateOrEmitWrench) {
  PayloadIdModule module(testSettings());
  vector6_t wrench = vector6_t::Zero();
  wrench(2) = -9.81;
  for (int sample = 0; sample < 200; ++sample) {
    module.step(staticInput(wrench), 0.005);
  }
  const auto before = module.parameters();

  auto input = staticInput(wrench);
  input.externalResidual(0) = std::numeric_limits<double>::quiet_NaN();
  const auto result = module.step(input, 0.005);

  EXPECT_FALSE(result.inputValid);
  EXPECT_FALSE(result.updated);
  EXPECT_TRUE(result.equivalentWrench.isZero(0.0));
  EXPECT_TRUE(module.parameters().isApprox(before, 0.0));
}

TEST(PayloadIdModule, AutomaticRegressorRejectsFastRotationButCustomOneCanRun) {
  PayloadIdModule module(testSettings());
  auto input = staticInput(vector6_t::Zero());
  input.hasAngularVelocity = true;
  input.angularVelocity << 0.0, 0.0, 1.1;
  EXPECT_FALSE(module.step(input, 0.01).inputValid);

  input.useCustomRegressor = true;
  input.customRegressor.setZero();
  EXPECT_TRUE(module.step(input, 0.01).inputValid);
}

TEST(PayloadIdModule, ProjectionAndResetKeepTheEstimateSafe) {
  auto settings = testSettings();
  settings.maxDeltaMass = 4.0;
  PayloadIdModule module(settings);
  vector6_t wrench = vector6_t::Zero();
  wrench(2) = -1000.0;
  for (int sample = 0; sample < 500; ++sample) {
    module.step(staticInput(wrench), 0.01);
  }
  EXPECT_NEAR(module.parameters()(0), 4.0, 1e-12);

  module.reset();
  const auto result = module.step(staticInput(vector6_t::Zero()), 0.0);
  EXPECT_TRUE(module.parameters().isZero(0.0));
  EXPECT_NEAR(result.deltaMass, 0.0, 0.0);
  EXPECT_TRUE(result.deltaCom.isZero(0.0));
  EXPECT_TRUE(result.equivalentWrench.isZero(0.0));
  EXPECT_NEAR(result.confidence, 0.0, 0.0);
}

TEST(PayloadIdModule, UpdateGateFreezesParametersWhileKeepingPrediction) {
  PayloadIdModule module(testSettings());
  vector6_t wrench = vector6_t::Zero();
  wrench(2) = -9.81;
  for (int sample = 0; sample < 200; ++sample) {
    module.step(staticInput(wrench), 0.005);
  }
  const auto before = module.parameters();

  auto input = staticInput(wrench);
  input.updateAllowed = false;
  const auto result = module.step(input, 0.01);

  EXPECT_TRUE(result.inputValid);
  EXPECT_FALSE(result.updated);
  EXPECT_TRUE(module.parameters().isApprox(before, 0.0));
  EXPECT_FALSE(result.equivalentWrench.isZero(1e-6));
}

}  // namespace
}  // namespace modules
}  // namespace adaptive
}  // namespace legged
