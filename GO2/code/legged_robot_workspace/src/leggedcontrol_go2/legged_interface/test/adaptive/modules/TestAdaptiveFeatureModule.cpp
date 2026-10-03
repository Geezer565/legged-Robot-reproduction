#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>

#include "legged_interface/adaptive/modules/AdaptiveFeatureModule.h"

namespace legged {
namespace adaptive {
namespace modules {
namespace {

AdaptiveFeatureSettings settings() {
  AdaptiveFeatureSettings value;
  value.initialExtent.fill(1.0);
  value.minimumExtent.fill(0.5);
  value.maximumExtent.fill(8.0);
  value.extentSigmaMultiplier = 2.5;
  value.minimumSamples = 20;
  value.publishEverySamples = 10;
  value.maximumRelativeChange = 0.10;
  return value;
}

TEST(AdaptiveFeatureModule, KeepsInitialMapUntilEnoughValidSamples) {
  AdaptiveFeatureModule module(settings());
  v2::FeatureInput input;
  input.causalResidual.fill(0.0);

  AdaptiveFeatureSuggestion result;
  for (int sample = 0; sample < 19; ++sample) result = module.observe(input, true);

  EXPECT_FALSE(result.ready);
  EXPECT_EQ(result.version, 0u);
  for (double extent : result.extent) EXPECT_DOUBLE_EQ(extent, 1.0);
}

TEST(AdaptiveFeatureModule, ExpandsOnlyExcitedAxisAndLimitsEachPublicationStep) {
  AdaptiveFeatureModule module(settings());
  v2::FeatureInput input;
  AdaptiveFeatureSuggestion result;
  for (int sample = 0; sample < 40; ++sample) {
    input.causalResidual.fill(0.0);
    input.causalResidual[0] = (sample % 2 == 0) ? -4.0 : 4.0;
    result = module.observe(input, true);
  }

  EXPECT_TRUE(result.ready);
  EXPECT_GT(result.extent[0], 1.0);
  EXPECT_LE(result.extent[0], 1.1 + 1e-12);
  for (int axis = 1; axis < 6; ++axis) EXPECT_NEAR(result.extent[axis], 0.9, 1e-12);
}

TEST(AdaptiveFeatureModule, InvalidOrRejectedSamplesDoNotUpdateAndResetRestoresDefaults) {
  AdaptiveFeatureModule module(settings());
  v2::FeatureInput input;
  input.causalResidual.fill(1.0);
  const auto ignored = module.observe(input, false);
  EXPECT_EQ(ignored.sampleCount, 0u);

  input.causalResidual[3] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(module.observe(input, true), std::invalid_argument);

  module.reset();
  const auto reset = module.current();
  EXPECT_EQ(reset.sampleCount, 0u);
  EXPECT_EQ(reset.version, 0u);
  for (double extent : reset.extent) EXPECT_DOUBLE_EQ(extent, 1.0);
}

}  // namespace
}  // namespace modules
}  // namespace adaptive
}  // namespace legged

