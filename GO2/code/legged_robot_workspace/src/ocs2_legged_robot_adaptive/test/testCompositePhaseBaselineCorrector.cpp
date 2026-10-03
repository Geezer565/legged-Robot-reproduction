#include <gtest/gtest.h>

#include <boost/filesystem.hpp>

#include <fstream>

#include "ocs2_legged_robot_adaptive/adaptive/CompositePhaseBaselineCorrector.h"
#include "ocs2_legged_robot_adaptive/package_path.h"

namespace ocs2 {
namespace legged_robot_adaptive {
namespace {

std::string compositeFixturePath() {
  return (boost::filesystem::path(getPath()) / "test" / "data" /
          "d5i4_composite_baseline_fixture.csv").string();
}

TEST(CompositePhaseBaselineCorrector,
     LoadsSparseKeyAndAppliesOnlyDuringFrozenSteadyWindow) {
  adaptive::CompositePhaseBaselineCorrector corrector(
      compositeFixturePath(), 120, 24, 12.0, 2.4, 2.4, 24.0, 1.2);
  EXPECT_EQ(corrector.occupiedKeys(), 1u);
  const std::array<std::pair<scalar_t, std::size_t>, 17> startup = {{
      {0.0, 13}, {0.3, 7}, {0.6, 14}, {0.9, 11},
      {1.2, 13}, {1.5, 7}, {1.8, 14}, {2.1, 11},
      {2.4, 13}, {2.7, 7}, {3.0, 14}, {3.3, 11},
      {3.6, 13}, {3.9, 7}, {4.2, 14}, {4.5, 11}, {4.8, 13}}};
  adaptive::CompositePhaseBaselineSample sample;
  for (const auto& value : startup) sample = corrector.sample(value.first, value.second);
  EXPECT_TRUE(sample.steadyStageActive);
  EXPECT_TRUE(sample.cycleBoundary);
  EXPECT_FALSE(sample.keySupported);
  EXPECT_FALSE(sample.correctionActive);
  EXPECT_TRUE(sample.baseline.isZero(0.0));

  sample = corrector.sample(4.82, 13);
  ASSERT_TRUE(sample.steadyStageActive);
  ASSERT_TRUE(sample.gaitPhaseValid);
  ASSERT_TRUE(sample.keySupported);
  ASSERT_TRUE(sample.correctionActive);
  EXPECT_EQ(sample.profilePhaseBin, 24);
  EXPECT_EQ(sample.gaitPhaseBin, 0);
  for (Eigen::Index index = 0; index < 10; ++index) {
    EXPECT_DOUBLE_EQ(sample.baseline(index), static_cast<double>(index + 1));
  }
}

TEST(CompositePhaseBaselineCorrector,
     UnsupportedAndRampKeysFailClosedWithoutFutureInformation) {
  adaptive::CompositePhaseBaselineCorrector corrector(
      compositeFixturePath(), 120, 24, 12.0, 2.4, 2.4, 24.0, 1.2);
  auto sample = corrector.sample(2.4, 13);
  EXPECT_FALSE(sample.steadyStageActive);
  EXPECT_FALSE(sample.correctionActive);
  EXPECT_TRUE(sample.baseline.isZero(0.0));
  sample = corrector.sample(2.7, 7);
  EXPECT_TRUE(sample.gaitPhaseValid);
  EXPECT_FALSE(sample.keySupported);
  EXPECT_TRUE(sample.baseline.isZero(0.0));
  sample = corrector.sample(3.0, 11);
  EXPECT_FALSE(sample.gaitPhaseValid);
  EXPECT_FALSE(sample.correctionActive);
  EXPECT_THROW(corrector.sample(3.0, 11), std::invalid_argument);
}

TEST(CompositePhaseBaselineCorrector, SettingsDefaultOffAndBranchesAreExclusive) {
  adaptive::AdaptiveClfSettings settings;
  EXPECT_FALSE(settings.enableCompositePhaseBaselineCorrection);
  EXPECT_TRUE(settings.compositePhaseBaselineFile.empty());
  settings.parameterization = adaptive::AdaptiveParameterization::PayloadInertial10D;
  settings.gamma = matrix_t::Identity(10, 10);
  settings.initialParameterEstimate = vector_t::Zero(10);
  settings.enableCompositePhaseBaselineCorrection = true;
  settings.compositePhaseBaselineFile = compositeFixturePath();
  EXPECT_NO_THROW(adaptive::validateAdaptiveClfSettings(settings));
  settings.enableGaitPhaseBaselineCorrection = true;
  settings.gaitPhaseBaselineFile = compositeFixturePath();
  EXPECT_THROW(adaptive::validateAdaptiveClfSettings(settings),
               std::invalid_argument);
}

TEST(CompositePhaseBaselineCorrector, AcceptsPythonCsvCrlfLineEndings) {
  const auto temporaryPath =
      boost::filesystem::temp_directory_path() /
      boost::filesystem::unique_path("d5i4-composite-%%%%-%%%%.csv");
  {
    std::ifstream input(compositeFixturePath());
    ASSERT_TRUE(input.good());
    std::ofstream output(temporaryPath.string(), std::ios::binary);
    ASSERT_TRUE(output.good());
    std::string line;
    while (std::getline(input, line)) output << line << "\r\n";
  }

  EXPECT_NO_THROW(adaptive::CompositePhaseBaselineCorrector(
      temporaryPath.string(), 120, 24, 12.0, 2.4, 2.4, 24.0, 1.2));
  boost::filesystem::remove(temporaryPath);
}

}  // namespace
}  // namespace legged_robot_adaptive
}  // namespace ocs2
