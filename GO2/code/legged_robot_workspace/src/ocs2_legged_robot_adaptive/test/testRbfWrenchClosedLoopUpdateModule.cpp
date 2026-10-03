#include <gtest/gtest.h>

#include <boost/filesystem.hpp>
#include <ocs2_legged_robot/LeggedRobotInterface.h>

#include "ocs2_legged_robot_adaptive/AdaptiveLeggedRobotInterface.h"
#include "ocs2_legged_robot_adaptive/adaptive/RbfWrenchClosedLoopUpdateModule.h"
#include "ocs2_legged_robot_adaptive/package_path.h"

namespace ocs2 { namespace legged_robot_adaptive { namespace adaptive { namespace {

struct Fixture {
  Fixture()
      : adaptivePath(getPath()),
        sourcePath(adaptivePath.parent_path()),
        leggedPath(sourcePath / "ocs2/ocs2_robotic_examples/ocs2_legged_robot"),
        base((leggedPath / "config/mpc/task.info").string(),
             (sourcePath / "ocs2_robotic_assets/resources/anymal_c/urdf/anymal.urdf").string(),
             (leggedPath / "config/command/reference.info").string()) {}

  RbfWrenchClosedLoopSettings rbfSettings() const {
    RbfWrenchClosedLoopSettings settings;
    settings.enabled = true;
    settings.method = WrenchEstimatorMethod::MinnitiRbf;
    settings.enableResidualFilter = false;
    settings.observationTimeTolerance = 1.0e-9;
    settings.estimator.network.residualScale.setOnes();
    settings.estimator.network.discrepancyScale.setOnes();
    settings.estimator.network.outputScale.setOnes();
    settings.estimator.network.centers = matrix_t::Zero(6, 50);
    settings.estimator.network.widths = matrix_t::Ones(6, 25);
    settings.estimator.gamma.setConstant(0.5);
    return settings;
  }

  boost::filesystem::path adaptivePath;
  boost::filesystem::path sourcePath;
  boost::filesystem::path leggedPath;
  legged_robot::LeggedRobotInterface base;
};

TEST(RbfWrenchClosedLoopUpdateModule, PublishesUpdatedSixAxisSnapshotBeforeSolve) {
  Fixture fixture;
  auto adaptiveSettings = loadAdaptiveClfSettings(
      (fixture.adaptivePath / "config/adaptive_settings.info").string());
  auto modelData = std::make_shared<AdaptiveModelData>(
      AdaptiveModelSnapshot::createValid(vector6_t::Zero(), 0.0, 1));
  RbfWrenchClosedLoopUpdateModule module(
      fixture.base.getPinocchioInterface(), fixture.base.getCentroidalModelInfo(),
      adaptiveSettings, fixture.rbfSettings(), modelData);

  vector_t state0 = fixture.base.getInitialState();
  state0.head<6>().setZero();
  const vector_t input = vector_t::Zero(fixture.base.getCentroidalModelInfo().inputDim);
  auto manager = fixture.base.getSwitchedModelReferenceManagerPtr();
  manager->setTargetTrajectories(TargetTrajectories({0.0}, {state0}, {input}));
  manager->preSolverRun(0.0, 0.1, state0);
  module.submitObservation(0.0, state0, input);
  module.preSolverRun(0.0, 0.1, state0, *manager);

  vector_t state1 = state0;
  state1(0) = 0.05;
  state1(6) += 0.02;
  manager->preSolverRun(0.05, 0.15, state1);
  module.submitObservation(0.05, state1, input);
  module.preSolverRun(0.05, 0.15, state1, *manager);

  const auto diagnostics = module.diagnostics();
  const auto snapshot = modelData->snapshot();
  EXPECT_TRUE(diagnostics.observationMatched);
  EXPECT_TRUE(diagnostics.residualValid);
  EXPECT_TRUE(diagnostics.weightsUpdated);
  EXPECT_DOUBLE_EQ(snapshot->time(), 0.05);
  EXPECT_EQ(snapshot->estimate().size(), 6);
  EXPECT_GT(snapshot->estimate().norm(), 0.0);
  EXPECT_TRUE(snapshot->estimate().isApprox(diagnostics.estimateAfterUpdate, 1.0e-14));
}

TEST(RbfWrenchClosedLoopUpdateModule, DuplicateSolverCallDoesNotUpdateTwice) {
  Fixture fixture;
  auto adaptiveSettings = loadAdaptiveClfSettings(
      (fixture.adaptivePath / "config/adaptive_settings.info").string());
  auto modelData = std::make_shared<AdaptiveModelData>(
      AdaptiveModelSnapshot::createValid(vector6_t::Zero(), 0.0, 1));
  RbfWrenchClosedLoopUpdateModule module(
      fixture.base.getPinocchioInterface(), fixture.base.getCentroidalModelInfo(),
      adaptiveSettings, fixture.rbfSettings(), modelData);
  vector_t state = fixture.base.getInitialState();
  state.head<6>().setZero();
  const vector_t input = vector_t::Zero(fixture.base.getCentroidalModelInfo().inputDim);
  auto manager = fixture.base.getSwitchedModelReferenceManagerPtr();
  manager->setTargetTrajectories(TargetTrajectories({0.0}, {state}, {input}));
  manager->preSolverRun(0.0, 0.1, state);
  module.submitObservation(0.0, state, input);
  module.preSolverRun(0.0, 0.1, state, *manager);
  const auto version = modelData->snapshot()->version();

  module.preSolverRun(0.0, 0.1, state, *manager);

  EXPECT_EQ(modelData->snapshot()->version(), version);
}

TEST(RbfWrenchClosedLoopSettings, DefaultStaysOffAndExplicitConfigLoadsFrozenBasis) {
  const boost::filesystem::path adaptivePath(getPath());
  const auto baseline = loadRbfWrenchClosedLoopSettings(
      (adaptivePath / "config/adaptive_settings.info").string());
  const auto enabled = loadRbfWrenchClosedLoopSettings(
      (adaptivePath / "config/adaptive_settings_rbf_wrench_v1.info").string());
  EXPECT_FALSE(baseline.enabled);
  EXPECT_TRUE(enabled.enabled);
  EXPECT_EQ(enabled.method, WrenchEstimatorMethod::MinnitiRbf);
  EXPECT_DOUBLE_EQ(enabled.residualFilterAlpha, 0.5);
  EXPECT_DOUBLE_EQ(enabled.estimator.gamma(0), 0.25);
  EXPECT_NEAR(enabled.estimator.network.centers(0, 6),
              1.0027117584559631, 1.0e-14);
}

TEST(RbfWrenchClosedLoopSettings, ExplicitConfigsSelectThreeIsolatedMethods) {
  const boost::filesystem::path adaptivePath(getPath());
  const auto current = loadRbfWrenchClosedLoopSettings(
      (adaptivePath / "config/adaptive_settings_rbf_wrench_v1.info").string());
  const auto composite = loadRbfWrenchClosedLoopSettings(
      (adaptivePath /
       "config/adaptive_settings_composite_rbf_wrench_v1.info").string());
  const auto rff = loadRbfWrenchClosedLoopSettings(
      (adaptivePath / "config/adaptive_settings_rff_wrench_v1.info").string());

  EXPECT_EQ(current.method, WrenchEstimatorMethod::MinnitiRbf);
  EXPECT_EQ(composite.method, WrenchEstimatorMethod::CompositeRbf);
  EXPECT_EQ(rff.method, WrenchEstimatorMethod::RandomFourierOgd);
  EXPECT_GT(composite.compositeEstimator.predictionGain(0), 0.0);
  EXPECT_EQ(rff.randomFourierEstimator.featureCount, 50);
  EXPECT_DOUBLE_EQ(rff.randomFourierEstimator.learningRate(0), 0.003);
}

TEST(RbfWrenchClosedLoopSettings, InterfaceSelectsRbfOnlyForExplicitConfig) {
  Fixture fixture;
  AdaptiveLeggedRobotInterface baseline(
      (fixture.leggedPath / "config/mpc/task.info").string(),
      (fixture.sourcePath / "ocs2_robotic_assets/resources/anymal_c/urdf/anymal.urdf").string(),
      (fixture.leggedPath / "config/command/reference.info").string());
  AdaptiveLeggedRobotInterface enabled(
      (fixture.leggedPath / "config/mpc/task.info").string(),
      (fixture.sourcePath / "ocs2_robotic_assets/resources/anymal_c/urdf/anymal.urdf").string(),
      (fixture.leggedPath / "config/command/reference.info").string(), false,
      (fixture.adaptivePath / "config/adaptive_settings_rbf_wrench_v1.info").string());
  EXPECT_EQ(baseline.getRbfWrenchClosedLoopUpdateModulePtr(), nullptr);
  EXPECT_NE(enabled.getRbfWrenchClosedLoopUpdateModulePtr(), nullptr);
}

TEST(RbfWrenchClosedLoopSettings, InterfaceAcceptsBothNewExplicitConfigs) {
  Fixture fixture;
  AdaptiveLeggedRobotInterface composite(
      (fixture.leggedPath / "config/mpc/task.info").string(),
      (fixture.sourcePath / "ocs2_robotic_assets/resources/anymal_c/urdf/anymal.urdf").string(),
      (fixture.leggedPath / "config/command/reference.info").string(), false,
      (fixture.adaptivePath /
       "config/adaptive_settings_composite_rbf_wrench_v1.info").string());
  AdaptiveLeggedRobotInterface rff(
      (fixture.leggedPath / "config/mpc/task.info").string(),
      (fixture.sourcePath / "ocs2_robotic_assets/resources/anymal_c/urdf/anymal.urdf").string(),
      (fixture.leggedPath / "config/command/reference.info").string(), false,
      (fixture.adaptivePath / "config/adaptive_settings_rff_wrench_v1.info").string());

  ASSERT_NE(composite.getRbfWrenchClosedLoopUpdateModulePtr(), nullptr);
  ASSERT_NE(rff.getRbfWrenchClosedLoopUpdateModulePtr(), nullptr);
  EXPECT_EQ(composite.getRbfWrenchClosedLoopUpdateModulePtr()->diagnostics().method,
            WrenchEstimatorMethod::CompositeRbf);
  EXPECT_EQ(rff.getRbfWrenchClosedLoopUpdateModulePtr()->diagnostics().method,
            WrenchEstimatorMethod::RandomFourierOgd);
}

}}}}  // namespace ocs2::legged_robot_adaptive::adaptive
