#include <gtest/gtest.h>

#include <boost/filesystem.hpp>

#include "ocs2_legged_robot_adaptive/adaptive/AdaptiveParameterEstimator.h"
#include "ocs2_legged_robot_adaptive/adaptive/GaitPhaseBaselineCorrector.h"
#include "ocs2_legged_robot_adaptive/package_path.h"

namespace ocs2 {
namespace legged_robot_adaptive {
namespace {

using adaptive::AdaptiveClfSettings;
using adaptive::AdaptiveEstimatorUpdate;
using adaptive::AdaptiveParameterEstimator;
using adaptive::AdaptiveParameterization;
using adaptive::GaitPhaseBaselineCorrector;
using adaptive::vector6_t;

std::string baselinePath() {
  const boost::filesystem::path adaptivePath(getPath());
  return (adaptivePath.parent_path() / "ocs2_legged_robot_adaptive_ros" /
          "config" / "d5h0_nominal_gait_phase_baseline_v1.csv").string();
}

AdaptiveClfSettings tenDimensionalSettings() {
  AdaptiveClfSettings settings;
  settings.parameterization = AdaptiveParameterization::PayloadInertial10D;
  settings.gamma = matrix_t::Identity(10, 10);
  settings.initialParameterEstimate = vector_t::Zero(10);
  return settings;
}

TEST(GaitPhaseBaselineCorrector, LoadsFrozenCsvAndMatchesD5H0EndpointConvention) {
  GaitPhaseBaselineCorrector corrector(baselinePath(), 24, 1.2, 2.2);
  const std::array<std::pair<scalar_t, std::size_t>, 10> startup = {{
      {0.0, 13}, {0.3, 7}, {0.6, 14}, {0.9, 11}, {1.2, 13},
      {1.22, 13}, {1.5, 7}, {1.8, 14}, {2.1, 11}, {2.4, 13}}};
  adaptive::GaitPhaseBaselineSample sample;
  for (const auto& item : startup) {
    sample = corrector.sample(item.first, item.second);
  }
  EXPECT_TRUE(sample.cycleBoundary);
  EXPECT_FALSE(sample.correctionActive);
  EXPECT_EQ(sample.phaseBin, 23);

  sample = corrector.sample(2.42, 13);
  ASSERT_TRUE(sample.phaseValid);
  ASSERT_TRUE(sample.correctionActive);
  ASSERT_EQ(sample.phaseBin, 0);
  EXPECT_NEAR(sample.baseline(0), 0.08834797189376141, 1.0e-14);
  EXPECT_NEAR(sample.baseline(1), 3.342828898755842, 1.0e-14);

  corrector.sample(2.7, 7);
  corrector.sample(3.0, 14);
  corrector.sample(3.3, 11);
  sample = corrector.sample(3.6, 13);
  EXPECT_TRUE(sample.cycleBoundary);
  EXPECT_TRUE(sample.correctionActive);
  EXPECT_EQ(sample.phaseBin, 23);
  EXPECT_TRUE(sample.baseline.allFinite());
}

TEST(GaitPhaseBaselineCorrector, InvalidSequenceFailsClosedAndRestartsCausally) {
  GaitPhaseBaselineCorrector corrector(baselinePath(), 24, 1.2, 0.0);
  EXPECT_TRUE(corrector.sample(0.0, 13).phaseValid);
  EXPECT_FALSE(corrector.sample(0.3, 14).phaseValid);
  EXPECT_FALSE(corrector.sample(0.6, 11).phaseValid);
  const auto restart = corrector.sample(1.2, 13);
  EXPECT_TRUE(restart.phaseValid);
  EXPECT_TRUE(restart.correctionActive);
  EXPECT_EQ(restart.phaseBin, 0);
  EXPECT_THROW(corrector.sample(1.2, 13), std::invalid_argument);
}

TEST(GaitPhaseBaselineCorrector, EstimatorSubtractsBaselineBeforeGammaExactly) {
  auto settings = tenDimensionalSettings();
  settings.gamma = 2.0 * matrix_t::Identity(10, 10);
  AdaptiveParameterEstimator estimator(settings);
  AdaptiveEstimatorUpdate update;
  update.sigma = vector6_t::Ones();
  update.regressor = matrix_t::Zero(6, 10);
  update.regressor.block(0, 0, 6, 6).setIdentity();
  update.nominalDriveBaseline = vector_t::Constant(10, 0.25);
  update.dt = 0.1;
  update.time = 1.0;

  vector_t expectedDrive = vector_t::Constant(10, -0.25);
  expectedDrive.head(6).array() += 1.0;
  const vector_t expectedDerivative = 2.0 * expectedDrive;
  EXPECT_TRUE(estimator.computeParameterDerivative(update).isApprox(
      expectedDerivative, 0.0));
  const auto snapshot = estimator.update(update);
  EXPECT_TRUE(snapshot->estimate().isApprox(
      0.1 * expectedDerivative, 1.0e-15));
}

TEST(GaitPhaseBaselineCorrector, EmptyBaselinePreservesHistoricalEq8Exactly) {
  auto settings = tenDimensionalSettings();
  AdaptiveParameterEstimator estimator(settings);
  AdaptiveEstimatorUpdate update;
  update.sigma << 1.0, -2.0, 3.0, -4.0, 5.0, -6.0;
  update.regressor = matrix_t::Random(6, 10);
  update.dt = 0.02;
  update.time = 1.0;
  const vector_t expected = update.regressor.transpose() * update.sigma;
  EXPECT_TRUE(estimator.computeParameterDerivative(update).isApprox(
      expected, 0.0));
}

TEST(GaitPhaseBaselineCorrector, BaselineCannotLeakIntoLegacyParameterizations) {
  AdaptiveClfSettings settings;
  AdaptiveParameterEstimator estimator(settings);
  AdaptiveEstimatorUpdate update;
  update.sigma.setOnes();
  update.regressor.setIdentity(6, 6);
  update.nominalDriveBaseline = vector_t::Zero(6);
  update.dt = 0.02;
  update.time = 1.0;
  EXPECT_THROW(estimator.computeParameterDerivative(update),
               std::invalid_argument);

  settings.enableGaitPhaseBaselineCorrection = true;
  settings.gaitPhaseBaselineFile = baselinePath();
  EXPECT_THROW(adaptive::validateAdaptiveClfSettings(settings),
               std::invalid_argument);
}

TEST(GaitPhaseBaselineCorrector, ExistingConfigDefaultsCorrectionOff) {
  const boost::filesystem::path adaptivePath(getPath());
  const auto settings = adaptive::loadAdaptiveClfSettings(
      (adaptivePath / "config" /
       "adaptive_settings_payload_inertial_10d_smoke.info").string());
  EXPECT_FALSE(settings.enableGaitPhaseBaselineCorrection);
  EXPECT_TRUE(settings.gaitPhaseBaselineFile.empty());
  EXPECT_EQ(settings.gaitPhaseBins, 24);
  EXPECT_DOUBLE_EQ(settings.gaitCycleDuration, 1.2);
  EXPECT_DOUBLE_EQ(settings.gaitBaselineMinimumCycleStartTime, 2.2);
}

}  // namespace
}  // namespace legged_robot_adaptive
}  // namespace ocs2
