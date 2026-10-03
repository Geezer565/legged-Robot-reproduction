#include <gtest/gtest.h>

#include <limits>

#include <ocs2_legged_robot_adaptive_ros/MatchedWrenchDisturbance.h>

namespace ocs2 {
namespace legged_robot_adaptive_ros {
namespace {

MatchedWrenchDisturbanceConfig enabledConfig() {
  MatchedWrenchDisturbanceConfig config;
  config.enabled = true;
  config.startTime = 1.0;
  config.endTime = 2.0;
  config.wrench << 12.0, -6.0, 3.0, 24.0, -12.0, 6.0;
  return config;
}

TEST(MatchedWrenchDisturbance, UsesOpenStartAndClosedEndForCompletedIntegrationSteps) {
  MatchedWrenchDisturbance disturbance(enabledConfig());
  EXPECT_FALSE(disturbance.isActive(1.0));
  EXPECT_TRUE(disturbance.isActive(1.01));
  EXPECT_TRUE(disturbance.isActive(2.0));
  EXPECT_FALSE(disturbance.isActive(2.01));
}

TEST(MatchedWrenchDisturbance, SupportsSignReversalSineAndPulseTrainProfiles) {
  auto reversalConfig = enabledConfig();
  reversalConfig.profile = MatchedWrenchDisturbanceProfile::SignReversal;
  reversalConfig.secondaryWrench = -reversalConfig.wrench;
  MatchedWrenchDisturbance reversal(reversalConfig);
  EXPECT_TRUE(reversal.appliedWrench(1.25).isApprox(reversalConfig.wrench));
  EXPECT_TRUE(reversal.appliedWrench(1.75).isApprox(-reversalConfig.wrench));

  auto sineConfig = enabledConfig();
  sineConfig.profile = MatchedWrenchDisturbanceProfile::Sine;
  sineConfig.frequencyHz = 0.25;
  MatchedWrenchDisturbance sine(sineConfig);
  EXPECT_TRUE(sine.appliedWrench(2.0).isApprox(sineConfig.wrench, 1.0e-12));

  auto pulseConfig = enabledConfig();
  pulseConfig.profile = MatchedWrenchDisturbanceProfile::PulseTrain;
  pulseConfig.frequencyHz = 2.0;
  pulseConfig.secondaryWrench = -0.5 * pulseConfig.wrench;
  MatchedWrenchDisturbance pulseTrain(pulseConfig);
  EXPECT_TRUE(pulseTrain.appliedWrench(1.10).isApprox(pulseConfig.wrench));
  EXPECT_TRUE(
      pulseTrain.appliedWrench(1.30).isApprox(pulseConfig.secondaryWrench));
  EXPECT_TRUE(pulseTrain.appliedWrench(1.60).isApprox(pulseConfig.wrench));

  auto sineCosineConfig = enabledConfig();
  sineCosineConfig.profile = MatchedWrenchDisturbanceProfile::SineCosine;
  sineCosineConfig.frequencyHz = 0.25;
  sineCosineConfig.secondaryWrench = -0.5 * sineCosineConfig.wrench;
  MatchedWrenchDisturbance sineCosine(sineCosineConfig);
  EXPECT_TRUE(sineCosine.appliedWrench(1.0 + 1.0e-6).isApprox(
      sineCosineConfig.secondaryWrench, 1.0e-4));
  EXPECT_TRUE(sineCosine.appliedWrench(2.0).isApprox(
      sineCosineConfig.wrench, 1.0e-12));
}

TEST(MatchedWrenchDisturbance, ParsesProfileNames) {
  EXPECT_EQ(matchedWrenchDisturbanceProfileFromString("constant"),
            MatchedWrenchDisturbanceProfile::Constant);
  EXPECT_EQ(matchedWrenchDisturbanceProfileFromString("sign_reversal"),
            MatchedWrenchDisturbanceProfile::SignReversal);
  EXPECT_EQ(matchedWrenchDisturbanceProfileFromString("sine"),
            MatchedWrenchDisturbanceProfile::Sine);
  EXPECT_EQ(matchedWrenchDisturbanceProfileFromString("pulse_train"),
            MatchedWrenchDisturbanceProfile::PulseTrain);
  EXPECT_EQ(matchedWrenchDisturbanceProfileFromString("sine_cosine"),
            MatchedWrenchDisturbanceProfile::SineCosine);
  EXPECT_THROW(matchedWrenchDisturbanceProfileFromString("unknown"),
               std::invalid_argument);
}

TEST(MatchedWrenchDisturbance, AppliesPaperSignInNormalizedMomentumChannelOnly) {
  MatchedWrenchDisturbance disturbance(enabledConfig());
  vector_t state = vector_t::LinSpaced(12, 1.0, 12.0);
  const vector_t before = state;

  disturbance.applyToState(1.02, 0.02, 12.0, state);

  vector_t expected = before;
  expected.head<6>() += (legged_robot_adaptive::adaptive::vector6_t()
                          << -0.02, 0.01, -0.005, -0.04, 0.02, -0.01).finished();
  EXPECT_TRUE(state.isApprox(expected, 1.0e-14));
  EXPECT_TRUE(state.tail(6).isApprox(before.tail(6), 1.0e-14));
}

TEST(MatchedWrenchDisturbance, DisabledOrInactiveConfigurationLeavesStateUnchanged) {
  auto config = enabledConfig();
  config.enabled = false;
  MatchedWrenchDisturbance disabled(config);
  vector_t state = vector_t::Random(12);
  const vector_t before = state;
  disabled.applyToState(1.5, 0.02, 12.0, state);
  EXPECT_TRUE(state.isApprox(before, 0.0));

  MatchedWrenchDisturbance enabled(enabledConfig());
  enabled.applyToState(0.5, 0.02, 12.0, state);
  EXPECT_TRUE(state.isApprox(before, 0.0));
}

TEST(MatchedWrenchDisturbance, RejectsInvalidConfigurationAndIntegrationInputs) {
  auto config = enabledConfig();
  config.endTime = config.startTime;
  EXPECT_THROW({ MatchedWrenchDisturbance disturbance(config); }, std::invalid_argument);
  config = enabledConfig();
  config.wrench(0) = std::numeric_limits<scalar_t>::quiet_NaN();
  EXPECT_THROW({ MatchedWrenchDisturbance disturbance(config); }, std::invalid_argument);
  config = enabledConfig();
  config.frequencyHz = 0.0;
  EXPECT_THROW({ MatchedWrenchDisturbance disturbance(config); }, std::invalid_argument);
  config = enabledConfig();
  config.profile = MatchedWrenchDisturbanceProfile::SignReversal;
  config.endTime = -1.0;
  EXPECT_THROW({ MatchedWrenchDisturbance disturbance(config); }, std::invalid_argument);

  MatchedWrenchDisturbance disturbance(enabledConfig());
  vector_t shortState = vector_t::Zero(5);
  EXPECT_THROW(disturbance.applyToState(1.5, 0.02, 12.0, shortState), std::invalid_argument);
  vector_t state = vector_t::Zero(12);
  EXPECT_THROW(disturbance.applyToState(1.5, 0.0, 12.0, state), std::invalid_argument);
}

}  // namespace
}  // namespace legged_robot_adaptive_ros
}  // namespace ocs2
