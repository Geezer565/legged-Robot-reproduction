#include <gtest/gtest.h>
#include <boost/filesystem/path.hpp>
#include <algorithm>
#include <cmath>

#include <ocs2_core/PreComputation.h>
#include <ocs2_legged_robot/LeggedRobotInterface.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include "ocs2_legged_robot_adaptive/package_path.h"
#include "ocs2_legged_robot_adaptive/adaptive/AdaptiveParameterEstimator.h"
#include "ocs2_legged_robot_adaptive/adaptive/AdaptiveParameterUpdateModule.h"
#include "ocs2_legged_robot_adaptive/adaptive/AdaptiveClf.h"
#include "ocs2_legged_robot_adaptive/adaptive/AdaptiveClfEvaluator.h"
#include "ocs2_legged_robot_adaptive/adaptive/AdaptiveReferenceKinematics.h"
#include "ocs2_legged_robot_adaptive/adaptive/AdaptiveReferenceStateEncoder.h"
#include "ocs2_legged_robot_adaptive/adaptive/AdaptiveRigidBodyRegressor.h"
#include "ocs2_legged_robot_adaptive/adaptive/IdentificationReferenceProfile.h"
#include "ocs2_legged_robot_adaptive/adaptive/PayloadMass1D.h"
#include "ocs2_legged_robot_adaptive/adaptive/QuaternionTrackingError.h"
#include "ocs2_legged_robot_adaptive/dynamics/AdaptiveLeggedRobotDynamics.h"
#include "ocs2_legged_robot_adaptive/dynamics/MassMismatchGroundTruth.h"
#include "ocs2_legged_robot_adaptive/AdaptiveLeggedRobotInterface.h"

namespace ocs2 {
namespace legged_robot_adaptive {
namespace {

adaptive::AdaptiveClfSettings makeSettings() {
  adaptive::AdaptiveClfSettings settings;
  settings.gamma.setIdentity();
  settings.initialParameterEstimate.setZero();
  return settings;
}

TEST(SlidingVariable, DiagnosticSubtermsCloseExactlyToSigma) {
  auto settings = makeSettings();
  settings.lambdaLinear.diagonal() << 2.0, 3.0, 4.0;
  settings.lambdaAngular.diagonal() << 5.0, 6.0, 7.0;
  adaptive::SlidingVariable sliding(settings);

  adaptive::RigidBodyTrackingState actual;
  adaptive::RigidBodyTrackingState desired;
  actual.comPositionWorld << 1.0, -2.0, 0.5;
  desired.comPositionWorld << 1.2, -1.7, 0.4;
  actual.comVelocityWorld << 0.3, -0.2, 0.1;
  desired.comVelocityWorld << 0.5, -0.1, -0.2;
  actual.angularVelocityWorld << -0.4, 0.2, 0.3;
  desired.angularVelocityWorld << -0.1, -0.2, 0.7;
  desired.orientationWorldFromBase =
      Eigen::AngleAxis<scalar_t>(0.1, adaptive::vector3_t::UnitZ());

  const auto result = sliding.compute(actual, desired);
  adaptive::vector6_t expectedVelocity;
  expectedVelocity << 0.2, 0.1, -0.3, 0.3, -0.4, 0.4;
  EXPECT_TRUE(result.velocityErrorWorld.isApprox(expectedVelocity, 1.0e-14));
  EXPECT_TRUE(result.lambdaPoseErrorWorld.head<3>().isApprox(
      settings.lambdaLinear *
          (desired.comPositionWorld - actual.comPositionWorld),
      1.0e-14));
  EXPECT_TRUE(result.lambdaPoseErrorWorld.tail<3>().isApprox(
      settings.lambdaAngular * result.orientationErrorWorld, 1.0e-14));
  EXPECT_TRUE(result.sigma.isApprox(
      result.velocityErrorWorld + result.lambdaPoseErrorWorld, 1.0e-14));
}

TEST(AdaptiveDynamics, ZeroEstimateMatchesNominalCorrection) {
  EXPECT_TRUE(AdaptiveLeggedRobotDynamics::normalizedMomentumRateCorrection(adaptive::vector6_t::Zero(), 12.0)
                  .isZero(1.0e-14));
}

TEST(AdaptiveDynamics, ConstantWrenchHasPaperSignInNormalizedMomentumRate) {
  adaptive::vector6_t estimate;
  estimate << 12.0, -6.0, 3.0, 24.0, -12.0, 6.0;
  adaptive::vector6_t expected;
  expected << -1.0, 0.5, -0.25, -2.0, 1.0, -0.5;
  EXPECT_TRUE(AdaptiveLeggedRobotDynamics::normalizedMomentumRateCorrection(estimate, 12.0).isApprox(expected));
}

TEST(PayloadMass1D, ReferenceRegressorAndMassConversionsAreExact) {
  const adaptive::vector3_t referenceAcceleration(1.0, -2.0, 3.0);
  const adaptive::vector6_t column =
      adaptive::PayloadMass1D::referenceRegressorColumn(
          referenceAcceleration);
  adaptive::vector6_t expected = adaptive::vector6_t::Zero();
  expected.head<3>() << 1.0, -2.0, 12.81;
  EXPECT_TRUE(column.isApprox(expected, 1.0e-14));

  constexpr scalar_t nominalMass = 52.13485;
  constexpr scalar_t payload = 5.213485;
  EXPECT_NEAR(adaptive::PayloadMass1D::estimatedTotalMass(
                  nominalMass, payload, 1.0),
              57.348335, 1.0e-12);
  EXPECT_NEAR(adaptive::PayloadMass1D::estimatedMassRatio(
                  nominalMass, payload, 1.0),
              1.0 - 1.0 / 1.10, 1.0e-14);
  EXPECT_THROW(adaptive::PayloadMass1D::estimatedTotalMass(
                   nominalMass, -nominalMass, 1.0),
               std::domain_error);
}

TEST(PayloadMass1D, SlotineLiRegressorUsesOnlyScalarMassColumn) {
  adaptive::AdaptiveRigidBodyRegressor::NominalParameters parameters;
  parameters.mass = 52.13485;
  adaptive::AdaptiveRigidBodyRegressor regressor(parameters);
  adaptive::AdaptiveRigidBodyRegressor::EvaluationPoint point;
  point.referenceLinearAccelerationWorld << 0.5, -0.25, 1.0;
  const adaptive::matrix6_t payloadRegressor =
      regressor.computeUnknownReferenceRegressor(
          point, adaptive::AdaptiveParameterization::PayloadMass1D);
  EXPECT_TRUE(payloadRegressor.rightCols<5>().isZero(0.0));
  EXPECT_TRUE(payloadRegressor.col(0).isApprox(
      adaptive::PayloadMass1D::referenceRegressorColumn(
          point.referenceLinearAccelerationWorld),
      1.0e-14));
  EXPECT_TRUE(regressor.computeUnknownReferenceRegressor().isIdentity());
}

TEST(MassMismatchGroundTruth, ExistingConstantWrenchSignAnchorIsExact) {
  constexpr scalar_t nominalMass = 52.13485;
  adaptive::vector6_t wrench;
  wrench << 20.0, -3.0, 5.0, 2.0, -4.0, 1.0;
  const adaptive::vector6_t nominalFlow = adaptive::vector6_t::Zero();
  const adaptive::vector6_t trueFlow =
      nominalFlow + AdaptiveLeggedRobotDynamics::normalizedMomentumRateCorrection(
                        wrench, nominalMass);

  const auto piStar = MassMismatchGroundTruth::computeEquivalentUncertainty(
      nominalFlow, trueFlow, nominalMass);
  EXPECT_TRUE(piStar.isApprox(wrench, 1.0e-14));
  EXPECT_TRUE(MassMismatchGroundTruth::reconstructTrueFlow(
                  nominalFlow, piStar, nominalMass)
                  .isApprox(trueFlow, 1.0e-14));
}

TEST(MassMismatchGroundTruth, EqualMassProducesZeroResidualOnRealSrbdFlow) {
  const boost::filesystem::path adaptivePath(getPath());
  const auto sourcePath = adaptivePath.parent_path();
  const auto leggedPath =
      sourcePath / "ocs2/ocs2_robotic_examples/ocs2_legged_robot";
  legged_robot::LeggedRobotInterface interface(
      (leggedPath / "config/mpc/task.info").string(),
      (sourcePath /
       "ocs2_robotic_assets/resources/anymal_c/urdf/anymal.urdf")
          .string(),
      (leggedPath / "config/command/reference.info").string());
  const auto& info = interface.getCentroidalModelInfo();
  EXPECT_NEAR(info.robotMass, 52.13485, 1.0e-10);
  MassMismatchGroundTruth groundTruth(interface.getPinocchioInterface(), info,
                                      info.robotMass);
  const vector_t state = interface.getInitialState();
  vector_t input = vector_t::Zero(info.inputDim);
  input.head<3>() << 120.0, -15.0, 30.0;

  const auto sample = groundTruth.evaluate(0.0, state, input);
  EXPECT_TRUE(sample.normalizedDynamicsResidual.isZero(1.0e-14));
  EXPECT_TRUE(sample.piStar.isZero(1.0e-12));
  EXPECT_TRUE(sample.reconstructionError.isZero(1.0e-14));
  EXPECT_NEAR(sample.fullFlowResidualNorm, 0.0, 1.0e-14);
  EXPECT_NEAR(sample.unmatchedFlowResidualNorm, 0.0, 1.0e-14);
}

TEST(MassMismatchGroundTruth,
     NonzeroMassResidualAndEquivalentWrenchCloseTheRealFlowLoop) {
  const boost::filesystem::path adaptivePath(getPath());
  const auto sourcePath = adaptivePath.parent_path();
  const auto leggedPath =
      sourcePath / "ocs2/ocs2_robotic_examples/ocs2_legged_robot";
  legged_robot::LeggedRobotInterface interface(
      (leggedPath / "config/mpc/task.info").string(),
      (sourcePath /
       "ocs2_robotic_assets/resources/anymal_c/urdf/anymal.urdf")
          .string(),
      (leggedPath / "config/command/reference.info").string());
  const auto& info = interface.getCentroidalModelInfo();
  constexpr scalar_t unitTestScale = 1.10;  // Test-only; not the formal campaign value.
  MassMismatchGroundTruth groundTruth(
      interface.getPinocchioInterface(), info,
      unitTestScale * info.robotMass);
  const vector_t state = interface.getInitialState();
  vector_t input = vector_t::Zero(info.inputDim);
  const adaptive::vector3_t firstContactForce(120.0, -15.0, 30.0);
  input.head<3>() = firstContactForce;

  const auto sample = groundTruth.evaluate(0.0, state, input);
  const adaptive::vector3_t expectedEquivalentForce =
      (1.0 - 1.0 / unitTestScale) * firstContactForce;
  EXPECT_GT(sample.normalizedDynamicsResidual.norm(), 1.0e-10);
  EXPECT_GT(sample.piStar.norm(), 1.0e-8);
  EXPECT_TRUE(sample.piStar.head<3>().isApprox(expectedEquivalentForce,
                                               1.0e-10));
  EXPECT_TRUE(sample.reconstructionError.isZero(1.0e-12));
  EXPECT_TRUE((sample.nominalFlow - sample.piStar / info.robotMass)
                  .isApprox(sample.trueFlow, 1.0e-12));
  EXPECT_TRUE(groundTruth.trueInfo().comToBasePositionNominal.isApprox(
      groundTruth.nominalInfo().comToBasePositionNominal, 0.0));
  EXPECT_TRUE(groundTruth.trueInfo().centroidalInertiaNominal.isApprox(
      groundTruth.nominalInfo().centroidalInertiaNominal, 0.0));

  // TimeTriggeredRollout clones this exact dynamics object. Verify that clone
  // evaluates the same true flow used to generate pi_star.
  std::unique_ptr<SystemDynamicsBase> plantClone(
      groundTruth.truePlantDynamics().clone());
  PreComputation preComputation;
  const vector_t clonedTrueFlow =
      plantClone->computeFlowMap(0.0, state, input, preComputation);
  EXPECT_TRUE(clonedTrueFlow.head<6>().isApprox(sample.trueFlow, 1.0e-12));
}

TEST(PayloadMass1D, EstimatedPayloadExactlyClosesRealMassMismatchFlow) {
  const boost::filesystem::path adaptivePath(getPath());
  const auto sourcePath = adaptivePath.parent_path();
  const auto leggedPath =
      sourcePath / "ocs2/ocs2_robotic_examples/ocs2_legged_robot";
  legged_robot::LeggedRobotInterface interface(
      (leggedPath / "config/mpc/task.info").string(),
      (sourcePath /
       "ocs2_robotic_assets/resources/anymal_c/urdf/anymal.urdf")
          .string(),
      (leggedPath / "config/command/reference.info").string());
  const auto& info = interface.getCentroidalModelInfo();
  constexpr scalar_t massScale = 1.10;
  const scalar_t payload = (massScale - 1.0) * info.robotMass;
  MassMismatchGroundTruth groundTruth(interface.getPinocchioInterface(), info,
                                      massScale * info.robotMass);
  const vector_t state = interface.getInitialState();
  vector_t input = vector_t::Zero(info.inputDim);
  input.segment<3>(0) << 120.0, -15.0, 160.0;
  input.segment<3>(3) << -20.0, 5.0, 110.0;

  const auto sample = groundTruth.evaluate(0.0, state, input);
  adaptive::vector6_t estimate = adaptive::vector6_t::Zero();
  estimate(0) = payload;
  const adaptive::vector6_t correction =
      AdaptiveLeggedRobotDynamics::normalizedMomentumRateCorrection(
          adaptive::AdaptiveParameterization::PayloadMass1D, estimate,
          sample.nominalFlow, info.robotMass, 1.0);
  EXPECT_TRUE((sample.nominalFlow + correction)
                  .isApprox(sample.trueFlow, 1.0e-12));

  adaptive::vector6_t gravity = adaptive::vector6_t::Zero();
  gravity.z() = -9.81;
  const adaptive::vector6_t contactWrench =
      info.robotMass * (sample.nominalFlow - gravity);
  EXPECT_TRUE(adaptive::PayloadMass1D::dynamicsEquivalentWrench(
                  contactWrench, info.robotMass, payload, 1.0)
                  .isApprox(sample.piStar, 1.0e-10));
}

TEST(AdaptiveEstimator, ExplicitEulerMatchesEquationEight) {
  auto settings = makeSettings();
  settings.gamma.diagonal() << 3.0, 3.0, 3.0, 1.5, 1.5, 1.5;
  adaptive::AdaptiveParameterEstimator estimator(settings);
  estimator.reset(adaptive::vector6_t::Zero(), 0.0);
  adaptive::AdaptiveEstimatorUpdate update;
  update.sigma << 1.0, -2.0, 0.5, 4.0, -1.0, 2.0;
  update.dt = 0.1;
  update.time = 0.1;
  const auto snapshot = estimator.update(update);
  EXPECT_TRUE(snapshot->estimate().isApprox(0.1 * settings.gamma * update.sigma));
}

TEST(AdaptiveEstimator, PayloadMassScalarEulerMatchesEquationEight) {
  auto settings = makeSettings();
  settings.parameterization =
      adaptive::AdaptiveParameterization::PayloadMass1D;
  settings.payloadMassAdaptationGain = 5.0;
  adaptive::AdaptiveParameterEstimator estimator(settings);
  estimator.reset(adaptive::vector6_t::Zero(), 0.0);

  adaptive::AdaptiveEstimatorUpdate update;
  update.sigma << 0.2, -0.1, 0.3, 9.0, 8.0, 7.0;
  update.regressor.setZero();
  update.regressor.col(0) << 1.0, 2.0, 9.81, 0.0, 0.0, 0.0;
  update.dt = 0.02;
  update.time = 0.02;
  const auto snapshot = estimator.update(update);
  const scalar_t expected =
      update.dt * settings.payloadMassAdaptationGain *
      update.regressor.col(0).dot(update.sigma);
  EXPECT_NEAR(snapshot->estimate()(0), expected, 1.0e-14);
  EXPECT_TRUE(snapshot->estimate().tail<5>().isZero(0.0));
}

TEST(AdaptiveSettings, PayloadMassConfigLoadsWithoutChangingBaselineConfig) {
  const boost::filesystem::path adaptivePath(getPath());
  const auto payload = adaptive::loadAdaptiveClfSettings(
      (adaptivePath / "config/adaptive_settings_payload_mass_1d.info")
          .string());
  const auto payloadOff = adaptive::loadAdaptiveClfSettings(
      (adaptivePath / "config/adaptive_settings_payload_mass_1d_off.info")
          .string());
  const auto baseline = adaptive::loadAdaptiveClfSettings(
      (adaptivePath / "config/adaptive_settings.info").string());
  EXPECT_EQ(payload.parameterization,
            adaptive::AdaptiveParameterization::PayloadMass1D);
  EXPECT_EQ(baseline.parameterization,
            adaptive::AdaptiveParameterization::ConstantWrench6D);
  EXPECT_DOUBLE_EQ(payload.payloadMassAdaptationGain, 5.0);
  EXPECT_TRUE(payload.initialParameterEstimate.isZero(0.0));
  EXPECT_EQ(payloadOff.parameterization,
            adaptive::AdaptiveParameterization::PayloadMass1D);
  EXPECT_FALSE(payloadOff.enableParameterUpdate);
  EXPECT_FALSE(payloadOff.enableAdaptiveDynamics);
  EXPECT_TRUE(payloadOff.enableClfConstraint);
  EXPECT_FALSE(payloadOff.enableInputBiasCost);
  EXPECT_DOUBLE_EQ(payloadOff.payloadMassAdaptationGain,
                   payload.payloadMassAdaptationGain);
  EXPECT_TRUE(payloadOff.initialParameterEstimate.isApprox(
      payload.initialParameterEstimate));
  EXPECT_STREQ(adaptive::adaptiveParameterizationName(
                   payload.parameterization),
               "payload_mass_1d");
}

TEST(AdaptiveUpdateModule, SameTimestampIsRejectedAsSecondEulerStep) {
  EXPECT_TRUE(adaptive::AdaptiveParameterUpdateModule::isDuplicateTimestamp(1.0, 1.0, 1.0e-9));
  EXPECT_TRUE(adaptive::AdaptiveParameterUpdateModule::isDuplicateTimestamp(1.0 + 5.0e-10, 1.0, 1.0e-9));
  EXPECT_FALSE(adaptive::AdaptiveParameterUpdateModule::isDuplicateTimestamp(1.1, 1.0, 1.0e-9));
}

TEST(AdaptiveClf, EquationElevenMatchesHandCalculation) {
  auto settings = makeSettings();
  settings.kd.setIdentity();
  adaptive::AdaptiveClf clf(settings);
  adaptive::AdaptiveClfInput input;
  input.sigma << 1., -2., 0.5, 0., 1., -1.;
  input.contactWrenchWorld << 10., 3., 1., 2., -1., 4.;
  input.nominalReferenceWrenchWorld << 2., 1., 0., 1., 0., 1.;
  input.parameterEstimate << 1., 0., -1., 0., 2., 1.;
  const scalar_t expected = input.sigma.dot(input.contactWrenchWorld - input.nominalReferenceWrenchWorld - input.parameterEstimate) - 0.5 * input.sigma.squaredNorm();
  EXPECT_DOUBLE_EQ(clf.computeValue(input), expected);
}

TEST(AdaptiveClf, EquationElevenDiagnosticsDecompositionReconstructsValue) {
  auto settings = makeSettings();
  settings.kd.setIdentity();
  adaptive::AdaptiveClf clf(settings);
  adaptive::AdaptiveClfInput input;
  input.sigma << 0.4, -0.2, 0.1, 0.3, -0.5, 0.25;
  input.contactWrenchWorld << 5.0, -2.0, 3.0, 1.0, 4.0, -1.0;
  input.nominalReferenceWrenchWorld << 1.0, 0.5, 2.0, -1.0, 0.0, 0.5;
  input.parameterEstimate << 0.2, -0.3, 0.1, 0.4, -0.2, 0.6;

  // 与 constraint diagnostics 完全一致的 Eq.(11) 分解：Y_u=I_6，
  // contribution_i 保留每个 world force / about-COM torque 轴的符号贡献。
  const adaptive::vector6_t residual = input.contactWrenchWorld - input.nominalReferenceWrenchWorld -
                                       input.unknownRegressor * input.parameterEstimate;
  const adaptive::vector6_t contribution = input.sigma.cwiseProduct(residual);
  const scalar_t linearTerm = contribution.sum();
  const scalar_t dampingTerm = 0.5 * input.sigma.dot(settings.kd * input.sigma);
  const scalar_t reconstructedH = linearTerm - dampingTerm;

  EXPECT_NEAR(contribution.sum(), linearTerm, 1.0e-14);
  EXPECT_NEAR(reconstructedH, clf.computeValue(input), 1.0e-14);
}

TEST(AdaptiveClf, ZeroSigmaHasNoEpsilonOffset) {
  adaptive::AdaptiveClf clf(makeSettings());
  adaptive::AdaptiveClfInput input;
  input.sigma.setZero(); input.contactWrenchWorld.setRandom(); input.nominalReferenceWrenchWorld.setRandom(); input.parameterEstimate.setRandom();
  EXPECT_DOUBLE_EQ(clf.computeValue(input), 0.0);
}

TEST(AdaptiveClf, ZeroEstimateRemovesUnknownWrenchTermAndKdIdentityDamping) {
  auto settings = makeSettings(); settings.kd.setIdentity(); adaptive::AdaptiveClf clf(settings);
  adaptive::AdaptiveClfInput input; input.sigma << 1., 2., -1., 0.5, 0., -2.; input.contactWrenchWorld.setZero(); input.nominalReferenceWrenchWorld.setZero(); input.parameterEstimate.setZero();
  EXPECT_DOUBLE_EQ(clf.computeValue(input), -0.5 * input.sigma.squaredNorm());
}

TEST(AdaptiveClf, ContactForceInputFiniteDifferenceIsNonzero) {
  auto settings = makeSettings(); settings.kd.setIdentity(); adaptive::AdaptiveClf clf(settings);
  adaptive::AdaptiveClfInput input; input.sigma << 1., 0., 0., 0., 0., -1.; input.nominalReferenceWrenchWorld.setZero(); input.parameterEstimate.setZero();
  // 单接触点 r=[0,1,0]：Fx 同时改变 world force 与 about-COM torque，代表 MPC input 的一个 force 分量。
  const adaptive::vector3_t r(0., 1., 0.); const scalar_t eps=1e-6;
  const auto valueAt = [&](scalar_t fx) { input.contactWrenchWorld << fx, 0., 0., r.cross(adaptive::vector3_t(fx,0.,0.)); return clf.computeValue(input); };
  const scalar_t derivative=(valueAt(3.+eps)-valueAt(3.-eps))/(2.*eps);
  EXPECT_NEAR(valueAt(3.), 5., 1e-12); EXPECT_GT(std::abs(derivative), 1e-8);
}

TEST(AdaptiveClf, FrozenSnapshotChangeChangesNextRunValue) {
  auto settings = makeSettings(); adaptive::AdaptiveClf clf(settings); adaptive::AdaptiveClfInput input; input.sigma << 1.,0.,0.,0.,0.,0.; input.contactWrenchWorld.setZero(); input.nominalReferenceWrenchWorld.setZero();
  auto data=std::make_shared<adaptive::AdaptiveModelData>(adaptive::AdaptiveModelSnapshot::createValid(adaptive::vector6_t::Zero(),0.,1)); input.parameterEstimate=data->snapshot()->estimate(); const scalar_t first=clf.computeValue(input);
  adaptive::vector6_t next=adaptive::vector6_t::Zero(); next(0)=2.; data->publish(adaptive::AdaptiveModelSnapshot::createValid(next,1.,2)); input.parameterEstimate=data->snapshot()->estimate(); EXPECT_NE(clf.computeValue(input),first);
}

TEST(AdaptiveClfConstraint, RealAnimalDummyInputHasNonzeroDfdu) {
  // 由本包的 generated package_path 定位同一 workspace 的原生 OCS2 资源；
  // 不依赖 ocs2_legged_robot 的 generated 头文件，以保持本测试仅链接本包。
  const boost::filesystem::path adaptivePath(getPath());
  const auto sourcePath = adaptivePath.parent_path();
  const auto ocs2Path = sourcePath / "ocs2";
  const auto leggedPath = ocs2Path / "ocs2_robotic_examples/ocs2_legged_robot";
  const std::string taskFile = (leggedPath / "config/mpc/task.info").string();
  const std::string referenceFile = (leggedPath / "config/command/reference.info").string();
  const std::string urdfFile = (sourcePath / "ocs2_robotic_assets/resources/anymal_c/urdf/anymal.urdf").string();
  AdaptiveLeggedRobotInterface interface(taskFile, urdfFile, referenceFile);
  const vector_t state = interface.getInitialState();
  vector_t desired = state;
  desired(6) += 0.10;  // 有限、物理有效的 base x tracking error，确保 sigma_l 非零。
  vector_t input = vector_t::Zero(interface.getCentroidalModelInfo().inputDim);
  auto manager = interface.getSwitchedModelReferenceManagerPtr();
  manager->setTargetTrajectories(TargetTrajectories({0.0}, {desired}, {input}));
  manager->preSolverRun(0.0, 0.1, state);
  PreComputation preComputation;
  const auto approximation = interface.getAdaptiveClfConstraintPtr()->getLinearApproximation(0.0, state, input, preComputation);
  ASSERT_TRUE(approximation.dfdu.allFinite());
  ASSERT_GT(approximation.dfdu.norm(), 1.0e-8);
  Eigen::Index index = 0;
  approximation.dfdu.row(0).cwiseAbs().maxCoeff(&index);
  std::cout << "real_constraint_dfdu_norm=" << approximation.dfdu.norm()
            << ", index=" << index << ", value=" << approximation.dfdu(0, index) << std::endl;
}

TEST(AdaptiveClfDiagnostics, CloneSharesLatestStorage) {
  const boost::filesystem::path adaptivePath(getPath());
  const auto ocs2Path = adaptivePath.parent_path() / "ocs2";
  const auto leggedPath = ocs2Path / "ocs2_robotic_examples/ocs2_legged_robot";
  AdaptiveLeggedRobotInterface interface((leggedPath / "config/mpc/task.info").string(),
                                         (adaptivePath.parent_path() / "ocs2_robotic_assets/resources/anymal_c/urdf/anymal.urdf").string(),
                                         (leggedPath / "config/command/reference.info").string());
  const auto constraint = interface.getAdaptiveClfConstraintPtr();
  std::unique_ptr<StateInputConstraint> clone(constraint->clone());
  const vector_t state = interface.getInitialState();
  const vector_t input = vector_t::Zero(interface.getCentroidalModelInfo().inputDim);
  interface.getSwitchedModelReferenceManagerPtr()->setTargetTrajectories(
      TargetTrajectories({0.0}, {state}, {input}));
  interface.getSwitchedModelReferenceManagerPtr()->preSolverRun(0.0, 0.1, state);
  PreComputation preComputation;

  // 模拟 OCS2 soft constraint 调用 clone 的真实路径：clone 写 diagnostics，
  // interface 保存的原对象必须读到同一份最新结果。
  const vector_t cloneValue = clone->getValue(0.0, state, input, preComputation);
  const auto fromOriginal = constraint->diagnostics();
  ASSERT_EQ(cloneValue.size(), 1);
  EXPECT_DOUBLE_EQ(fromOriginal.hClf, cloneValue(0));
  EXPECT_TRUE(fromOriginal.sigma.allFinite());
}

TEST(AdaptiveClfEvaluator, MatchesConstraintAtSameStateInputSnapshotAndReference) {
  const boost::filesystem::path adaptivePath(getPath());
  const auto sourcePath = adaptivePath.parent_path();
  const auto leggedPath = sourcePath / "ocs2/ocs2_robotic_examples/ocs2_legged_robot";
  AdaptiveLeggedRobotInterface interface(
      (leggedPath / "config/mpc/task.info").string(),
      (sourcePath / "ocs2_robotic_assets/resources/anymal_c/urdf/anymal.urdf").string(),
      (leggedPath / "config/command/reference.info").string());
  const vector_t state = interface.getInitialState();
  vector_t desired = state;
  desired(6) += 0.1;  // 强制非零 sigma，避免一致性测试退化为零值比较。
  const vector_t input = vector_t::Zero(interface.getCentroidalModelInfo().inputDim);
  auto manager = interface.getSwitchedModelReferenceManagerPtr();
  manager->setTargetTrajectories(TargetTrajectories({0.0}, {desired}, {input}));
  manager->preSolverRun(0.0, 0.1, state);
  PreComputation preComputation;

  const auto constraint = interface.getAdaptiveClfConstraintPtr();
  ASSERT_NE(constraint, nullptr);
  const scalar_t hConstraint = constraint->getValue(0.0, state, input, preComputation)(0);
  const auto fromConstraint = constraint->diagnostics();
  const auto fromEvaluator = interface.getAdaptiveClfEvaluatorPtr()->evaluate(
      0.0, state, input, interface.getAdaptiveModelDataPtr()->snapshot());

  EXPECT_DOUBLE_EQ(fromEvaluator.hClf, hConstraint);
  EXPECT_TRUE(fromEvaluator.sigma.isApprox(fromConstraint.sigma, 1.0e-12));
  EXPECT_TRUE(fromEvaluator.clfContribution.isApprox(fromConstraint.clfContribution, 1.0e-12));
  EXPECT_TRUE(fromEvaluator.residualWrench.isApprox(fromConstraint.residualWrench, 1.0e-12));
  EXPECT_NEAR(fromEvaluator.clfContribution.sum() - fromEvaluator.dampingTerm, fromEvaluator.hClf, 1.0e-12);
}

TEST(AdaptiveClfEvaluator, SnapshotSynchronizationRejectsPreviousOrFutureFrame) {
  // 这项是 executed monitor 的工程保护，而非论文 Eq.(11) 的新条件：真实
  // observation 必须匹配 Eq.(8) 发布的同一轮 frozen pi_hat，不能静默跨帧评价。
  EXPECT_TRUE(adaptive::AdaptiveClfEvaluator::isSnapshotTimeSynchronized(1.0, 1.0 + 5.0e-7, 1.0e-6));
  EXPECT_FALSE(adaptive::AdaptiveClfEvaluator::isSnapshotTimeSynchronized(1.0, 0.99, 1.0e-6));
  EXPECT_FALSE(adaptive::AdaptiveClfEvaluator::isSnapshotTimeSynchronized(1.0, 1.01, 1.0e-6));
}

TEST(AdaptiveClfEvaluator, DoesNotRequireConstraintAttachment) {
  // evaluator 自身不依赖 StateInputSoftConstraint。故 CLF OFF 时只要 Eq.(8)
  // snapshot 与 reference 存在，即可旁路计算 hypothetical Eq.(11)。
  const boost::filesystem::path adaptivePath(getPath());
  const auto sourcePath = adaptivePath.parent_path();
  const auto leggedPath = sourcePath / "ocs2/ocs2_robotic_examples/ocs2_legged_robot";
  legged_robot::LeggedRobotInterface base(
      (leggedPath / "config/mpc/task.info").string(),
      (sourcePath / "ocs2_robotic_assets/resources/anymal_c/urdf/anymal.urdf").string(),
      (leggedPath / "config/command/reference.info").string());
  const auto settings = adaptive::loadAdaptiveClfSettings(adaptivePath.string() + "/config/adaptive_settings.info");
  adaptive::AdaptiveClfEvaluator evaluator(base.getPinocchioInterface(), base.getCentroidalModelInfo(),
                                            *base.getSwitchedModelReferenceManagerPtr(), settings);
  const vector_t state = base.getInitialState();
  const vector_t input = vector_t::Zero(base.getCentroidalModelInfo().inputDim);
  const auto baseManager = base.getSwitchedModelReferenceManagerPtr();
  baseManager->setTargetTrajectories(TargetTrajectories({0.0}, {state}, {input}));
  baseManager->preSolverRun(0.0, 0.1, state);
  const auto snapshot = adaptive::AdaptiveModelSnapshot::createValid(adaptive::vector6_t::Zero(), 0.0, 1);
  const auto diagnostics = evaluator.evaluate(0.0, state, input, snapshot);
  EXPECT_TRUE(std::isfinite(diagnostics.hClf));
  EXPECT_TRUE(diagnostics.sigma.allFinite());
}

TEST(AdaptiveReferenceKinematics, MovingPoseReferenceProducesKinematicallyConsistentVelocity) {
  const boost::filesystem::path adaptivePath(getPath());
  const auto sourcePath = adaptivePath.parent_path();
  const auto leggedPath = sourcePath / "ocs2/ocs2_robotic_examples/ocs2_legged_robot";
  legged_robot::LeggedRobotInterface base((leggedPath / "config/mpc/task.info").string(),
      (sourcePath / "ocs2_robotic_assets/resources/anymal_c/urdf/anymal.urdf").string(),
      (leggedPath / "config/command/reference.info").string());
  const auto settings = adaptive::loadAdaptiveClfSettings(adaptivePath.string() + "/config/adaptive_settings.info");
  adaptive::AdaptiveReferenceKinematicsAdapter reference(base.getPinocchioInterface(), base.getCentroidalModelInfo(), settings);
  vector_t start = base.getInitialState(); start.head<6>().setZero();
  vector_t finish = start; finish(6) += 0.10;
  const vector_t input = vector_t::Zero(base.getCentroidalModelInfo().inputDim);
  const TargetTrajectories target({0.0, 0.2}, {start, finish}, {input, input});
  for (const scalar_t time : {0.0, 0.05, 0.10, 0.15}) {
    const auto r = reference.compute(time, start, target);
    EXPECT_NEAR(r.desired.comPositionWorld.x() - reference.compute(0.0, start, target).desired.comPositionWorld.x(), 0.5 * time, 1e-10);
    EXPECT_NEAR(r.desired.comVelocityWorld.x(), 0.5, 1e-10);
    EXPECT_NEAR(r.desiredComAccelerationWorld.x(), 0.0, 1e-14);
  }
  EXPECT_NEAR(reference.compute(0.2, start, target).desired.comVelocityWorld.x(), 0.0, 1e-14);
  EXPECT_NEAR(reference.compute(0.3, start, target).desired.comVelocityWorld.x(), 0.0, 1e-14);
}

TEST(IdentificationReferenceProfile, AnalyticDerivativesAndPeriodicityAreConsistent) {
  const adaptive::IdentificationReferenceProfile profile;
  const scalar_t time = 2.137;
  const scalar_t h = 1.0e-6;
  for (const auto candidate : {
           adaptive::IdentificationExcitationCandidate::E1Translation,
           adaptive::IdentificationExcitationCandidate::E2Rotation,
           adaptive::IdentificationExcitationCandidate::E3Combined}) {
    const auto center = profile.sample(candidate, time);
    const auto plus = profile.sample(candidate, time + h);
    const auto minus = profile.sample(candidate, time - h);
    EXPECT_LT((((plus.comPositionOffsetWorld - minus.comPositionOffsetWorld) /
                (2.0 * h)) - center.comVelocityWorld).norm(), 1.0e-9);
    EXPECT_LT((((plus.comVelocityWorld - minus.comVelocityWorld) /
                (2.0 * h)) - center.comAccelerationWorld).norm(), 1.0e-9);
    EXPECT_LT((((plus.eulerZyx - minus.eulerZyx) / (2.0 * h)) -
               center.eulerZyxDerivative).norm(), 1.0e-9);
    EXPECT_LT((((plus.eulerZyxDerivative - minus.eulerZyxDerivative) /
                (2.0 * h)) - center.eulerZyxSecondDerivative).norm(), 1.0e-9);
    EXPECT_LT((((plus.angularVelocityWorld - minus.angularVelocityWorld) /
                (2.0 * h)) - center.angularAccelerationWorld).norm(), 1.0e-8);

    const auto periodic =
        profile.sample(candidate, time + profile.settings().period);
    EXPECT_TRUE(periodic.comPositionOffsetWorld.isApprox(
        center.comPositionOffsetWorld, 1.0e-12));
    EXPECT_TRUE(periodic.comVelocityWorld.isApprox(center.comVelocityWorld,
                                                   1.0e-12));
    EXPECT_TRUE(periodic.comAccelerationWorld.isApprox(
        center.comAccelerationWorld, 1.0e-12));
    EXPECT_TRUE(periodic.eulerZyx.isApprox(center.eulerZyx, 1.0e-12));
    EXPECT_TRUE(periodic.angularVelocityWorld.isApprox(
        center.angularVelocityWorld, 1.0e-12));
    EXPECT_TRUE(periodic.angularAccelerationWorld.isApprox(
        center.angularAccelerationWorld, 1.0e-12));
  }
}

TEST(IdentificationReferenceProfile, SmallEnvelopeRespectsD5I0Limits) {
  const adaptive::IdentificationReferenceProfile profile;
  scalar_t maxTranslationSpeed = 0.0;
  scalar_t maxAngularSpeed = 0.0;
  for (std::size_t i = 0; i <= 12000; ++i) {
    const scalar_t time = profile.settings().period *
                          static_cast<scalar_t>(i) / 12000.0;
    const auto sample = profile.sample(
        adaptive::IdentificationExcitationCandidate::E3Combined, time);
    maxTranslationSpeed =
        std::max(maxTranslationSpeed, sample.comVelocityWorld.norm());
    maxAngularSpeed =
        std::max(maxAngularSpeed, sample.angularVelocityWorld.norm());
    EXPECT_LE(sample.comPositionOffsetWorld.cwiseAbs().x(), 0.020 + 1e-14);
    EXPECT_LE(sample.comPositionOffsetWorld.cwiseAbs().y(), 0.015 + 1e-14);
    EXPECT_LE(sample.comPositionOffsetWorld.cwiseAbs().z(), 0.008 + 1e-14);
    EXPECT_LE(sample.eulerZyx.cwiseAbs().x(), 0.040 + 1e-14);
    EXPECT_LE(sample.eulerZyx.cwiseAbs().y(), 0.025 + 1e-14);
    EXPECT_LE(sample.eulerZyx.cwiseAbs().z(), 0.025 + 1e-14);
  }
  EXPECT_LT(maxTranslationSpeed, 0.5);
  EXPECT_LT(maxAngularSpeed, 0.3);
}

TEST(IdentificationReferenceSchedule, C2RampAndStageBoundariesAreExact) {
  const adaptive::IdentificationReferenceProfile profile;
  const adaptive::IdentificationReferenceSchedule schedule(profile);
  const auto before = schedule.sample(
      adaptive::IdentificationExcitationCandidate::E3Combined, 2.399);
  EXPECT_EQ(before.stage, adaptive::IdentificationReferenceStage::PreStart);
  EXPECT_TRUE(before.reference.comPositionOffsetWorld.isZero(0.0));
  EXPECT_TRUE(before.reference.eulerZyx.isZero(0.0));

  const auto start = schedule.sample(
      adaptive::IdentificationExcitationCandidate::E3Combined, 2.4);
  EXPECT_EQ(start.stage, adaptive::IdentificationReferenceStage::RampIn);
  EXPECT_DOUBLE_EQ(start.envelope, 0.0);
  EXPECT_DOUBLE_EQ(start.envelopeDerivative, 0.0);
  EXPECT_DOUBLE_EQ(start.envelopeSecondDerivative, 0.0);
  EXPECT_TRUE(start.reference.comVelocityWorld.isZero(0.0));
  EXPECT_TRUE(start.reference.angularVelocityWorld.isZero(0.0));

  const auto steady = schedule.sample(
      adaptive::IdentificationExcitationCandidate::E3Combined,
      schedule.rampInEndTime());
  EXPECT_EQ(steady.stage, adaptive::IdentificationReferenceStage::Steady);
  EXPECT_DOUBLE_EQ(steady.envelope, 1.0);
  EXPECT_DOUBLE_EQ(steady.envelopeDerivative, 0.0);
  EXPECT_DOUBLE_EQ(steady.envelopeSecondDerivative, 0.0);

  const auto settle = schedule.sample(
      adaptive::IdentificationExcitationCandidate::E3Combined,
      schedule.rampOutEndTime());
  EXPECT_EQ(settle.stage, adaptive::IdentificationReferenceStage::Settle);
  EXPECT_TRUE(settle.reference.comPositionOffsetWorld.isZero(0.0));
  EXPECT_TRUE(settle.reference.comVelocityWorld.isZero(0.0));
  EXPECT_TRUE(settle.reference.eulerZyx.isZero(0.0));
  EXPECT_TRUE(settle.reference.angularVelocityWorld.isZero(0.0));
}

TEST(IdentificationReferenceSchedule, ProductRuleDerivativesMatchFiniteDifferences) {
  const adaptive::IdentificationReferenceSchedule schedule;
  const scalar_t h = 1.0e-6;
  for (const scalar_t time : {3.1, 5.7, 17.3}) {
    const auto center = schedule.sample(
        adaptive::IdentificationExcitationCandidate::E3Combined, time);
    const auto plus = schedule.sample(
        adaptive::IdentificationExcitationCandidate::E3Combined, time + h);
    const auto minus = schedule.sample(
        adaptive::IdentificationExcitationCandidate::E3Combined, time - h);
    EXPECT_LT((((plus.reference.comPositionOffsetWorld -
                 minus.reference.comPositionOffsetWorld) / (2.0 * h)) -
               center.reference.comVelocityWorld).norm(), 1.0e-8);
    EXPECT_LT((((plus.reference.comVelocityWorld -
                 minus.reference.comVelocityWorld) / (2.0 * h)) -
               center.reference.comAccelerationWorld).norm(), 1.0e-7);
    EXPECT_LT((((plus.reference.eulerZyx - minus.reference.eulerZyx) /
                (2.0 * h)) - center.reference.eulerZyxDerivative).norm(),
              1.0e-8);
    EXPECT_LT((((plus.reference.angularVelocityWorld -
                 minus.reference.angularVelocityWorld) / (2.0 * h)) -
               center.reference.angularAccelerationWorld).norm(), 1.0e-6);
  }
}

TEST(AdaptiveReferenceStateEncoder, PoseAndTwistRoundTripUsesNativeCentroidalSemantics) {
  const boost::filesystem::path adaptivePath(getPath());
  const auto sourcePath = adaptivePath.parent_path();
  const auto leggedPath = sourcePath / "ocs2/ocs2_robotic_examples/ocs2_legged_robot";
  legged_robot::LeggedRobotInterface base(
      (leggedPath / "config/mpc/task.info").string(),
      (sourcePath / "ocs2_robotic_assets/resources/anymal_c/urdf/anymal.urdf").string(),
      (leggedPath / "config/command/reference.info").string());
  adaptive::AdaptiveStateAdapter adapter(base.getPinocchioInterface(),
                                         base.getCentroidalModelInfo());
  adaptive::AdaptiveReferenceStateEncoder encoder(
      base.getPinocchioInterface(), base.getCentroidalModelInfo());
  const vector_t initial = base.getInitialState();
  const auto initialPhysical = adapter.adaptActual(initial);
  const adaptive::IdentificationReferenceProfile profile;
  const auto sample = profile.sample(
      adaptive::IdentificationExcitationCandidate::E3Combined, 0.731);
  const vector_t encoded = encoder.encode(
      initialPhysical.comPositionWorld + sample.comPositionOffsetWorld,
      sample.eulerZyx, sample.comVelocityWorld,
      sample.angularVelocityWorld,
      initial.tail(base.getCentroidalModelInfo().actuatedDofNum));
  const auto decoded = adapter.adaptDesiredWithEncodedVelocity(encoded);
  EXPECT_TRUE(decoded.comPositionWorld.isApprox(
      initialPhysical.comPositionWorld + sample.comPositionOffsetWorld,
      1.0e-10));
  EXPECT_TRUE(decoded.comVelocityWorld.isApprox(sample.comVelocityWorld,
                                                1.0e-10));
  EXPECT_TRUE(decoded.angularVelocityWorld.isApprox(
      sample.angularVelocityWorld, 1.0e-10));
  const adaptive::quaternion_t expected =
      getQuaternionFromEulerAnglesZyx<scalar_t>(sample.eulerZyx);
  EXPECT_NEAR(decoded.orientationWorldFromBase.angularDistance(expected), 0.0,
              1.0e-12);
}

TEST(AdaptiveReferenceKinematics, EncodedDerivativeModeIsExplicitAndSegmentConsistent) {
  const boost::filesystem::path adaptivePath(getPath());
  const auto sourcePath = adaptivePath.parent_path();
  const auto leggedPath = sourcePath / "ocs2/ocs2_robotic_examples/ocs2_legged_robot";
  legged_robot::LeggedRobotInterface base(
      (leggedPath / "config/mpc/task.info").string(),
      (sourcePath / "ocs2_robotic_assets/resources/anymal_c/urdf/anymal.urdf").string(),
      (leggedPath / "config/command/reference.info").string());
  auto settings = adaptive::loadAdaptiveClfSettings(
      adaptivePath.string() + "/config/adaptive_settings.info");
  EXPECT_FALSE(settings.enableEncodedReferenceDerivatives);
  settings.enableEncodedReferenceDerivatives = true;
  adaptive::AdaptiveReferenceKinematicsAdapter reference(
      base.getPinocchioInterface(), base.getCentroidalModelInfo(), settings);
  adaptive::AdaptiveStateAdapter adapter(base.getPinocchioInterface(),
                                         base.getCentroidalModelInfo());
  adaptive::AdaptiveReferenceStateEncoder encoder(
      base.getPinocchioInterface(), base.getCentroidalModelInfo());
  const vector_t initial = base.getInitialState();
  const auto initialPhysical = adapter.adaptActual(initial);
  const adaptive::IdentificationReferenceProfile profile;
  const scalar_t t0 = 0.40;
  const scalar_t t1 = 0.41;
  const auto s0 = profile.sample(
      adaptive::IdentificationExcitationCandidate::E3Combined, t0);
  const auto s1 = profile.sample(
      adaptive::IdentificationExcitationCandidate::E3Combined, t1);
  const vector_t joints =
      initial.tail(base.getCentroidalModelInfo().actuatedDofNum);
  const auto encode = [&](const adaptive::IdentificationReferenceSample& s) {
    return encoder.encode(initialPhysical.comPositionWorld +
                              s.comPositionOffsetWorld,
                          s.eulerZyx, s.comVelocityWorld,
                          s.angularVelocityWorld, joints);
  };
  const vector_t state0 = encode(s0);
  const vector_t state1 = encode(s1);
  const vector_t input =
      vector_t::Zero(base.getCentroidalModelInfo().inputDim);
  const TargetTrajectories target({t0, t1}, {state0, state1},
                                  {input, input});
  const auto applied = reference.compute(0.5 * (t0 + t1), initial, target);
  const auto decoded0 = adapter.adaptDesiredWithEncodedVelocity(state0);
  const auto decoded1 = adapter.adaptDesiredWithEncodedVelocity(state1);
  EXPECT_TRUE(applied.desiredComAccelerationWorld.isApprox(
      (decoded1.comVelocityWorld - decoded0.comVelocityWorld) / (t1 - t0),
      1.0e-10));
  EXPECT_TRUE(applied.desiredAngularAccelerationWorld.isApprox(
      (decoded1.angularVelocityWorld - decoded0.angularVelocityWorld) /
          (t1 - t0),
      1.0e-10));
  EXPECT_GT(applied.desired.angularVelocityWorld.norm(), 1.0e-4);
}

TEST(AdaptiveReferenceKinematics, ReferenceVelocityIdentityMatchesSlidingVariable) {
  const boost::filesystem::path adaptivePath(getPath());
  const auto sourcePath = adaptivePath.parent_path();
  const auto leggedPath = sourcePath / "ocs2/ocs2_robotic_examples/ocs2_legged_robot";
  legged_robot::LeggedRobotInterface base((leggedPath / "config/mpc/task.info").string(),
      (sourcePath / "ocs2_robotic_assets/resources/anymal_c/urdf/anymal.urdf").string(),
      (leggedPath / "config/command/reference.info").string());
  const auto settings = adaptive::loadAdaptiveClfSettings(adaptivePath.string() + "/config/adaptive_settings.info");
  adaptive::AdaptiveReferenceKinematicsAdapter reference(base.getPinocchioInterface(), base.getCentroidalModelInfo(), settings);
  adaptive::SlidingVariable sliding(settings);
  vector_t start = base.getInitialState(); start.head<6>().setZero();
  vector_t finish = start; finish(6) += 0.10;
  vector_t actual = start; actual(0) = 0.2;  // world x actual linear velocity。
  const vector_t input = vector_t::Zero(base.getCentroidalModelInfo().inputDim);
  const TargetTrajectories target({0.0, 0.2}, {start, finish}, {input, input});
  const auto r = reference.compute(0.10, actual, target);
  const auto sigma = sliding.compute(r.actual, r.desired).sigma;
  EXPECT_NEAR(sigma(0), r.referenceLinearVelocityWorld.x() - r.actual.comVelocityWorld.x(), 1e-12);
  EXPECT_NEAR(r.referenceLinearAccelerationWorld.x(), 2.0 * (0.5 - 0.2), 1e-12);
}

TEST(AdaptiveReferenceKinematics, ParameterUpdateAndClfShareReferenceKinematics) {
  const boost::filesystem::path adaptivePath(getPath());
  const auto sourcePath = adaptivePath.parent_path();
  const auto leggedPath = sourcePath / "ocs2/ocs2_robotic_examples/ocs2_legged_robot";
  AdaptiveLeggedRobotInterface interface((leggedPath / "config/mpc/task.info").string(),
      (sourcePath / "ocs2_robotic_assets/resources/anymal_c/urdf/anymal.urdf").string(),
      (leggedPath / "config/command/reference.info").string());
  vector_t start = interface.getInitialState(); start.head<6>().setZero();
  vector_t finish = start; finish(6) += 0.10;
  vector_t actual = start; actual(0) = 0.2;
  const vector_t input = vector_t::Zero(interface.getCentroidalModelInfo().inputDim);
  const TargetTrajectories target({0.0, 0.2}, {start, finish}, {input, input});
  const auto manager = interface.getSwitchedModelReferenceManagerPtr();
  manager->setTargetTrajectories(target); manager->preSolverRun(0.0, 0.2, start);
  const auto module = interface.getAdaptiveParameterUpdateModulePtr();
  module->preSolverRun(0.0, 0.2, start, *manager);
  module->preSolverRun(0.05, 0.2, actual, *manager);
  const auto d = interface.getAdaptiveClfEvaluatorPtr()->evaluate(0.05, actual, input, module->snapshot());
  adaptive::matrix6_t gamma = adaptive::matrix6_t::Zero(); gamma.diagonal() << 3., 3., 3., 1.5, 1.5, 1.5;
  EXPECT_TRUE(module->snapshot()->estimate().isApprox(0.05 * gamma * d.sigma, 1e-10));
}

TEST(AdaptiveReferenceKinematics, StandingReferenceRegressionUnchanged) {
  const boost::filesystem::path adaptivePath(getPath());
  const auto sourcePath = adaptivePath.parent_path();
  const auto leggedPath = sourcePath / "ocs2/ocs2_robotic_examples/ocs2_legged_robot";
  legged_robot::LeggedRobotInterface base((leggedPath / "config/mpc/task.info").string(),
      (sourcePath / "ocs2_robotic_assets/resources/anymal_c/urdf/anymal.urdf").string(),
      (leggedPath / "config/command/reference.info").string());
  const auto settings = adaptive::loadAdaptiveClfSettings(adaptivePath.string() + "/config/adaptive_settings.info");
  adaptive::AdaptiveReferenceKinematicsAdapter reference(base.getPinocchioInterface(), base.getCentroidalModelInfo(), settings);
  vector_t state = base.getInitialState(); state.head<6>().setZero();
  const vector_t input = vector_t::Zero(base.getCentroidalModelInfo().inputDim);
  const TargetTrajectories target({0.0}, {state}, {input});
  const auto r = reference.compute(0.0, state, target);
  EXPECT_TRUE(r.desired.comVelocityWorld.isZero(1e-14));
  EXPECT_TRUE(r.desiredComAccelerationWorld.isZero(1e-14));
  EXPECT_TRUE(r.referenceLinearVelocityWorld.isZero(1e-14));
  EXPECT_TRUE(r.referenceLinearAccelerationWorld.isZero(1e-14));

  // 静止 target 下 dot(v_r)=omega_r=dot(omega_r)=0。因此论文 Eq.(11) 的
  // Y_n*pi_n 必须退化为抵消 world gravity 所需的 [0,0,m*g,0,0,0]，这是
  // 本轮 pose 导数恢复不能破坏的 standing baseline 回归条件。
  adaptive::AdaptiveRigidBodyRegressor::NominalParameters parameters;
  parameters.mass = base.getCentroidalModelInfo().robotMass;
  parameters.inertiaBaseAboutCom = base.getCentroidalModelInfo().centroidalInertiaNominal;
  parameters.gravityWorld << 0.0, 0.0, -9.81;
  adaptive::AdaptiveRigidBodyRegressor regressor(parameters);
  adaptive::AdaptiveRigidBodyRegressor::EvaluationPoint point;
  point.rotationWorldFromBase = r.actual.orientationWorldFromBase.toRotationMatrix();
  point.actualAngularVelocityWorld = r.actual.angularVelocityWorld;
  point.referenceLinearAccelerationWorld = r.referenceLinearAccelerationWorld;
  point.referenceAngularVelocityWorld = r.referenceAngularVelocityWorld;
  point.referenceAngularAccelerationWorld = r.referenceAngularAccelerationWorld;
  adaptive::vector6_t expectedWrench = adaptive::vector6_t::Zero();
  expectedWrench.z() = parameters.mass * 9.81;
  EXPECT_TRUE(regressor.computeNominalReferenceWrench(point).isApprox(expectedWrench, 1e-10));
}

TEST(QuaternionTrackingError, DerivativeMatchesFiniteDifference) {
  const adaptive::quaternion_t actual(Eigen::AngleAxis<scalar_t>(0.10, adaptive::vector3_t::UnitY()));
  const adaptive::quaternion_t desired(Eigen::AngleAxis<scalar_t>(-0.08, adaptive::vector3_t::UnitX()));
  const adaptive::vector3_t omegaActual(0.2, -0.1, 0.3);
  const adaptive::vector3_t omegaDesired(-0.15, 0.25, -0.05);
  const scalar_t h = 1.0e-7;
  const auto advance = [](const adaptive::quaternion_t& q, const adaptive::vector3_t& omega, scalar_t dt) {
    const scalar_t norm = omega.norm();
    return norm > 0.0 ? adaptive::quaternion_t(Eigen::AngleAxis<scalar_t>(norm * dt, omega / norm)) * q : q;
  };
  const auto finiteDifference = (adaptive::computeQuaternionTrackingError(advance(actual, omegaActual, h), advance(desired, omegaDesired, h)) -
                                 adaptive::computeQuaternionTrackingError(advance(actual, omegaActual, -h), advance(desired, omegaDesired, -h))) / (2.0 * h);
  const auto analytic = adaptive::computeQuaternionTrackingErrorDerivative(actual, desired, omegaActual, omegaDesired);
  EXPECT_TRUE(finiteDifference.isApprox(analytic, 1e-7));
  EXPECT_TRUE(adaptive::computeQuaternionTrackingErrorDerivative(adaptive::quaternion_t::Identity(), adaptive::quaternion_t::Identity(),
      adaptive::vector3_t::Zero(), adaptive::vector3_t::Zero()).isZero(1e-14));
  EXPECT_TRUE(adaptive::computeQuaternionTrackingErrorDerivative(actual, actual, omegaActual, omegaActual).isZero(1e-14));
}

}  // namespace
}  // namespace legged_robot_adaptive
}  // namespace ocs2
