#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>

#include "legged_interface/adaptive/modules/ResidualDynamicsModule.h"

namespace legged {
namespace adaptive {
namespace modules {
namespace {

ResidualDynamicsSettings settings() {
  ResidualDynamicsSettings value;
  value.learningRate.setConstant(0.35);
  value.parameterLimit.setConstant(4.0);
  value.minimumSamplesForConfidence = 20;
  value.confidenceRiseSamples = 40;
  return value;
}

TEST(ResidualDynamicsModule, StartsFromZeroAndRejectsInvalidInput) {
  ResidualDynamicsModule module(settings());
  vector6_t zero = vector6_t::Zero();
  const auto first = module.step(zero, zero, 0.01, true);
  EXPECT_TRUE(first.predictedCurrent.isZero(0.0));
  EXPECT_TRUE(first.predictedNext.isZero(0.0));
  EXPECT_TRUE(first.confidence.isZero(0.0));

  vector6_t bad = zero;
  bad[2] = std::numeric_limits<scalar_t>::quiet_NaN();
  EXPECT_THROW(module.step(bad, zero, 0.01, true), std::invalid_argument);
  EXPECT_THROW(module.step(zero, zero, 0.0, true), std::invalid_argument);
}

TEST(ResidualDynamicsModule, LearnsConstantSingleAxisWithoutCrossAxisLeakage) {
  ResidualDynamicsModule module(settings());
  vector6_t residual = vector6_t::Zero();
  vector6_t chi = vector6_t::Zero();
  residual[1] = 2.0;

  ResidualDynamicsResult result;
  for (int sample = 0; sample < 120; ++sample) {
    result = module.step(residual, chi, 0.01, true);
  }

  EXPECT_NEAR(result.predictedNext[1], 2.0, 0.08);
  EXPECT_GT(result.confidence[1], 0.95);
  for (int axis = 0; axis < 6; ++axis) {
    if (axis != 1) {
      EXPECT_DOUBLE_EQ(result.predictedNext[axis], 0.0);
    }
  }
}

TEST(ResidualDynamicsModule, FreezeDoesNotChangeWeightsAndResetClearsState) {
  ResidualDynamicsModule module(settings());
  vector6_t residual = vector6_t::Ones();
  vector6_t chi = vector6_t::Zero();
  for (int sample = 0; sample < 40; ++sample) module.step(residual, chi, 0.01, true);
  const matrix_t before = module.weights();
  module.step(2.0 * residual, chi, 0.01, false);
  EXPECT_TRUE(module.weights().isApprox(before, 0.0));

  module.reset();
  EXPECT_TRUE(module.weights().isZero(0.0));
  const auto after = module.step(vector6_t::Zero(), chi, 0.01, false);
  EXPECT_TRUE(after.predictedNext.isZero(0.0));
}

}  // namespace
}  // namespace modules
}  // namespace adaptive
}  // namespace legged

