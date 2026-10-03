#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>

#include "legged_interface/adaptive/modules/PeriodicResidualModule.h"

namespace legged {
namespace adaptive {
namespace modules {
namespace {

PeriodicResidualModule::Settings oneHertzSettings() {
  PeriodicResidualModule::Settings settings;
  settings.harmonics = 2;
  settings.frequencyHz.fill(1.0);
  settings.forgettingFactor = 0.999;
  settings.initialCovariance = 100.0;
  settings.minimumObservedPeriods = 1.5;
  settings.outputLimit.fill(100.0);
  return settings;
}

MyRbfV2FeatureInput sampleWithFy(double value) {
  MyRbfV2FeatureInput input;
  input.causalResidual[1] = value;
  return input;
}

TEST(PeriodicResidualModule, LearnsTwoHarmonicsWithoutCrossAxisLeakage) {
  PeriodicResidualModule module(oneHertzSettings());
  constexpr double dt = 0.002;
  constexpr double pi = 3.14159265358979323846;
  PeriodicResidualResult result;

  for (int step = 0; step < 2500; ++step) {
    const double time = step * dt;
    const double fy = 12.0 * std::sin(2.0 * pi * time + 0.25) +
                      3.0 * std::sin(4.0 * pi * time - 0.4);
    result = module.step(time, dt, sampleWithFy(fy));
  }

  EXPECT_TRUE(result.inputValid);
  EXPECT_TRUE(result.valid);
  EXPECT_NEAR(result.fundamentalAmplitude[1], 12.0, 0.15);
  EXPECT_NEAR(result.secondHarmonicAmplitude[1], 3.0, 0.15);
  EXPECT_NEAR(result.periodicWrench[1],
              12.0 * std::sin(2.0 * pi * 4.998 + 0.25) +
                  3.0 * std::sin(4.0 * pi * 4.998 - 0.4),
              0.15);
  for (int axis = 0; axis < 6; ++axis) {
    if (axis != 1) {
      EXPECT_NEAR(result.periodicWrench[axis], 0.0, 1e-12);
      EXPECT_NEAR(result.fundamentalAmplitude[axis], 0.0, 1e-12);
      EXPECT_NEAR(result.secondHarmonicAmplitude[axis], 0.0, 1e-12);
    }
  }
}

TEST(PeriodicResidualModule, FirstSampleAtLargeClockStartsConfidenceAtZero) {
  PeriodicResidualModule module(oneHertzSettings());

  const auto first = module.step(1234.0, 0.001, sampleWithFy(2.0));

  EXPECT_TRUE(first.inputValid);
  EXPECT_FALSE(first.valid);
  EXPECT_DOUBLE_EQ(first.confidence[1], 0.0);
}

TEST(PeriodicResidualModule, ResetClearsLearnedSignalAndRestartsConfidence) {
  PeriodicResidualModule module(oneHertzSettings());
  constexpr double dt = 0.01;
  constexpr double pi = 3.14159265358979323846;
  PeriodicResidualResult learned;
  for (int step = 0; step < 300; ++step) {
    const double time = step * dt;
    learned = module.step(time, dt, sampleWithFy(8.0 * std::sin(2.0 * pi * time)));
  }
  ASSERT_TRUE(learned.valid);
  ASSERT_GT(learned.fundamentalAmplitude[1], 7.5);

  module.reset(10.0);
  const auto reset = module.step(10.0, dt, sampleWithFy(0.0));

  EXPECT_TRUE(reset.inputValid);
  EXPECT_FALSE(reset.valid);
  EXPECT_DOUBLE_EQ(reset.confidence[1], 0.0);
  EXPECT_DOUBLE_EQ(reset.periodicWrench[1], 0.0);
  EXPECT_DOUBLE_EQ(reset.fundamentalAmplitude[1], 0.0);
}

TEST(PeriodicResidualModule, NonFiniteSampleReturnsSafeZeroAndDoesNotPoisonState) {
  PeriodicResidualModule module(oneHertzSettings());
  auto invalidInput = sampleWithFy(std::numeric_limits<double>::quiet_NaN());

  const auto rejected = module.step(0.0, 0.01, invalidInput);

  EXPECT_FALSE(rejected.inputValid);
  EXPECT_FALSE(rejected.valid);
  EXPECT_TRUE(rejected.periodicWrench.allFinite());
  EXPECT_DOUBLE_EQ(rejected.periodicWrench.norm(), 0.0);

  const auto recovered = module.step(0.01, 0.01, sampleWithFy(1.0));
  EXPECT_TRUE(recovered.inputValid);
  EXPECT_TRUE(recovered.periodicWrench.allFinite());
}

TEST(PeriodicResidualModule, RejectsMoreThanTwoHarmonics) {
  auto settings = oneHertzSettings();
  settings.harmonics = 3;
  EXPECT_THROW(PeriodicResidualModule module(settings), std::invalid_argument);
}

}  // namespace
}  // namespace modules
}  // namespace adaptive
}  // namespace legged
