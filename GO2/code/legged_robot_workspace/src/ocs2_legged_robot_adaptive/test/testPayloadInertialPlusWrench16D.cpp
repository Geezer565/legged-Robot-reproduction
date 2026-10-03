#include <gtest/gtest.h>
#include <boost/filesystem/path.hpp>
#include <memory>

#include <ocs2_core/PreComputation.h>
#include <ocs2_legged_robot/LeggedRobotInterface.h>

#include "ocs2_legged_robot_adaptive/adaptive/AdaptiveParameterEstimator.h"
#include "ocs2_legged_robot_adaptive/adaptive/AdaptiveClf.h"
#include "ocs2_legged_robot_adaptive/adaptive/AdaptiveRigidBodyRegressor.h"
#include "ocs2_legged_robot_adaptive/adaptive/AdaptiveReferenceKinematics.h"
#include "ocs2_legged_robot_adaptive/adaptive/PayloadInertial10D.h"
#include "ocs2_legged_robot_adaptive/adaptive/AdaptiveModelData.h"
#include "ocs2_legged_robot_adaptive/dynamics/AdaptiveLeggedRobotDynamics.h"
#include "ocs2_legged_robot_adaptive/dynamics/NominalCoordinatePlantDynamics.h"
#include "ocs2_legged_robot_adaptive/package_path.h"

namespace ocs2 { namespace legged_robot_adaptive { namespace {

using adaptive::AdaptiveParameterization;

adaptive::AdaptiveClfSettings jointSettings() {
  adaptive::AdaptiveClfSettings s;
  s.parameterization = AdaptiveParameterization::PayloadInertial10DPlusWrench6D;
  s.gamma = matrix_t::Zero(16, 16);
  s.gamma.topLeftCorner(10, 10).diagonal().setConstant(2.0);
  s.gamma.bottomRightCorner(6, 6).diagonal().setConstant(0.5);
  s.initialParameterEstimate = vector_t::Zero(16);
  adaptive::validateAdaptiveClfSettings(s);
  return s;
}

std::unique_ptr<legged_robot::LeggedRobotInterface> makeInterface() {
  const boost::filesystem::path root(getPath());
  const auto src = root.parent_path();
  return std::unique_ptr<legged_robot::LeggedRobotInterface>(
      new legged_robot::LeggedRobotInterface(
          (src / "ocs2/ocs2_robotic_examples/ocs2_legged_robot/config/mpc/task.info").string(),
          (src / "ocs2_robotic_assets/resources/anymal_c/urdf/anymal.urdf").string(),
          (src / "ocs2/ocs2_robotic_examples/ocs2_legged_robot/config/command/reference.info").string()));
}

TEST(PayloadInertialPlusWrench16D, RegressorIsExactBlockConcatenation) {
  adaptive::AdaptiveRigidBodyRegressor r;
  adaptive::AdaptiveRigidBodyRegressor::EvaluationPoint p;
  p.referenceLinearAccelerationWorld << 0.3, -0.2, 0.4;
  p.referenceAngularVelocityWorld << -0.1, 0.2, 0.3;
  p.referenceAngularAccelerationWorld << 0.4, 0.1, -0.2;
  const auto yin = r.computeUnknownReferenceRegressor(p, AdaptiveParameterization::PayloadInertial10D);
  const auto y = r.computeUnknownReferenceRegressor(p, AdaptiveParameterization::PayloadInertial10DPlusWrench6D);
  EXPECT_EQ(y.rows(), 6); EXPECT_EQ(y.cols(), 16);
  EXPECT_TRUE(y.leftCols(10).isApprox(yin, 1e-14));
  EXPECT_TRUE(y.rightCols(6).isApprox(adaptive::matrix6_t::Identity(), 1e-14));
}

TEST(PayloadInertialPlusWrench16D,
     PaperWrenchBlockMapsBaseForceAndTorqueToOcs2WorldCoordinates) {
  adaptive::AdaptiveRigidBodyRegressor r;
  adaptive::AdaptiveRigidBodyRegressor::EvaluationPoint p;
  p.rotationWorldFromBase =
      Eigen::AngleAxis<scalar_t>(
          0.42, adaptive::vector3_t(0.3, -0.5, 0.2).normalized())
          .toRotationMatrix();
  const auto y = r.computeUnknownReferenceRegressor(
      p, AdaptiveParameterization::PayloadInertial10DPlusWrench6D,
      adaptive::AdaptiveDynamicsForm::PaperRegressorCompensation);
  adaptive::matrix6_t expected = adaptive::matrix6_t::Zero();
  expected.topLeftCorner<3, 3>() = p.rotationWorldFromBase;
  expected.bottomRightCorner<3, 3>() = p.rotationWorldFromBase;
  EXPECT_TRUE(y.rightCols<6>().isApprox(expected, 1.0e-14));

  adaptive::vector6_t baseWrench;
  baseWrench << 1.0, -2.0, 3.0, -0.4, 0.5, -0.6;
  adaptive::vector6_t worldWrench;
  worldWrench.head<3>() =
      p.rotationWorldFromBase * baseWrench.head<3>();
  worldWrench.tail<3>() =
      p.rotationWorldFromBase * baseWrench.tail<3>();
  EXPECT_TRUE((y.rightCols<6>() * baseWrench)
                  .isApprox(worldWrench, 1.0e-14));
}

TEST(PayloadInertialPlusWrench16D, ConfigurationLoadsSixteenDimensionalBlockGamma) {
  const auto s = adaptive::loadAdaptiveClfSettings(
      std::string(getPath()) +
      "/config/adaptive_settings_payload_inertial_10d_plus_wrench_6d.info");
  EXPECT_EQ(s.parameterization, AdaptiveParameterization::PayloadInertial10DPlusWrench6D);
  EXPECT_EQ(s.gamma.rows(), 16); EXPECT_EQ(s.gamma.cols(), 16);
  EXPECT_EQ(s.initialParameterEstimate.size(), 16);
  EXPECT_DOUBLE_EQ(s.gamma(0, 0), 1.0);
  EXPECT_DOUBLE_EQ(s.gamma(10, 10), 2.0);
  EXPECT_DOUBLE_EQ(s.gamma(13, 13), 0.5);
  EXPECT_TRUE(s.gamma.topRightCorner(10, 6).isZero(0.0));
  EXPECT_TRUE(s.gamma.bottomLeftCorner(6, 10).isZero(0.0));
}

TEST(PayloadInertialPlusWrench16D, EquationEightUsesBlockDiagonalGammaAndEuler) {
  const auto s = jointSettings();
  adaptive::AdaptiveParameterEstimator estimator(s);
  adaptive::AdaptiveEstimatorUpdate u;
  u.sigma << 0.2, -0.3, 0.4, -0.5, 0.6, -0.7;
  u.regressor = adaptive::regressor_t::Zero(6, 16);
  u.regressor.leftCols(6).setIdentity();
  u.regressor.rightCols(6).setIdentity();
  u.dt = 0.1; u.time = 0.1;
  const vector_t dot = estimator.computeParameterDerivative(u);
  EXPECT_TRUE(dot.isApprox(s.gamma * u.regressor.transpose() * u.sigma, 1e-14));
  const auto next = estimator.update(u);
  EXPECT_TRUE(next->estimate().isApprox(0.1 * dot, 1e-14));
}

TEST(PayloadInertialPlusWrench16D, ZeroAndDegenerateWrenchesAreExact) {
  adaptive::AdaptiveRigidBodyRegressor r;
  adaptive::AdaptiveRigidBodyRegressor::EvaluationPoint p;
  const auto y = r.computeUnknownReferenceRegressor(p, AdaptiveParameterization::PayloadInertial10DPlusWrench6D);
  vector_t pi = vector_t::Zero(16);
  EXPECT_TRUE((y * pi).isZero(0.0)); // zero uncertainty
  pi.tail(6) << 1., -2., 3., -4., 5., -6.;
  EXPECT_TRUE((y * pi).isApprox(pi.tail(6), 1e-14)); // wrench-only Eq.(11)
  pi.setZero(); pi.head(10) = vector_t::LinSpaced(10, -0.2, 0.3);
  EXPECT_TRUE((y * pi).isApprox(y.leftCols(10) * pi.head(10), 1e-14)); // inertial-only Eq.(11)
}

TEST(PayloadInertialPlusWrench16D, EquationElevenUsesBothUncertaintyTerms) {
  const auto s = jointSettings();
  adaptive::AdaptiveRigidBodyRegressor r;
  adaptive::AdaptiveRigidBodyRegressor::EvaluationPoint p;
  p.referenceLinearAccelerationWorld << 0.2, -0.1, 0.4;
  const auto y = r.computeUnknownReferenceRegressor(
      p, AdaptiveParameterization::PayloadInertial10DPlusWrench6D);
  vector_t pi = vector_t::LinSpaced(16, -0.3, 0.45);
  adaptive::AdaptiveClfInput in;
  in.sigma << 0.1, -0.2, 0.3, -0.4, 0.5, -0.6;
  in.contactWrenchWorld = adaptive::vector6_t::Constant(1.7);
  in.nominalReferenceWrenchWorld = adaptive::vector6_t::Constant(-0.2);
  in.unknownRegressor = y; in.parameterEstimate = pi;
  const scalar_t expected = in.sigma.dot(in.contactWrenchWorld -
      in.nominalReferenceWrenchWorld - y.leftCols(10) * pi.head(10) -
      pi.tail(6)) - 0.5 * in.sigma.squaredNorm();
  EXPECT_NEAR(adaptive::AdaptiveClf(s).computeValue(in), expected, 1e-14);
}

TEST(PayloadInertialPlusWrench16D, SnapshotFreezesAllSixteenEntriesTogether) {
  const auto s = jointSettings();
  adaptive::AdaptiveParameterEstimator estimator(s);
  vector_t pi = vector_t::LinSpaced(16, -0.8, 0.7);
  const auto frozen = estimator.reset(pi, 2.0);
  EXPECT_TRUE(frozen->valid()); EXPECT_EQ(frozen->estimate().size(), 16);
  EXPECT_EQ(frozen->version(), estimator.snapshot()->version());
  EXPECT_TRUE(frozen->estimate().head(10).isApprox(pi.head(10), 0.0));
  EXPECT_TRUE(frozen->estimate().tail(6).isApprox(pi.tail(6), 0.0));
  EXPECT_DOUBLE_EQ(frozen->time(), 2.0);
}

TEST(PayloadInertialPlusWrench16D, EquationTenSeparatesInertialFlowAndWrench) {
  auto interface = makeInterface();
  const auto& nominal = interface->getCentroidalModelInfo();
  adaptive::PayloadInertial10D::Parameters payload;
  payload.mass = 1.2; payload.firstMomentNominalComBase << 0.12, -0.03, 0.08;
  payload.inertiaAboutNominalComBase.diagonal() << 0.03, 0.04, 0.05;
  const adaptive::vector10_t inertial = adaptive::PayloadInertial10D::toVector(payload);
  adaptive::vector6_t wrench; wrench << 2., -1., 0.5, 0.3, -0.2, 0.1;
  vector_t joint = vector_t::Zero(16); joint.head(10) = inertial; joint.tail(6) = wrench;
  auto data = std::make_shared<adaptive::AdaptiveModelData>(
      adaptive::AdaptiveModelSnapshot::createValid(joint, 0.2, 7));
  auto settings = jointSettings(); settings.enableAdaptiveDynamics = true;
  AdaptiveLeggedRobotDynamics dynamics(interface->getPinocchioInterface(), nominal,
      "joint_dynamics", interface->modelSettings(), settings, data);
  vector_t state = interface->getInitialState();
  vector_t input = vector_t::Zero(nominal.inputDim); PreComputation pre;
  NominalCoordinatePlantDynamics nominalDynamics(interface->getPinocchioInterface(), nominal, nominal);
  NominalCoordinatePlantDynamics inertialDynamics(interface->getPinocchioInterface(), nominal,
      adaptive::PayloadInertial10D::makeEstimatedModelInfo(nominal, inertial));
  const vector_t nominalFlow = nominalDynamics.computeFlowMap(0.2, state, input, pre);
  const vector_t inertialFlow = inertialDynamics.computeFlowMap(0.2, state, input, pre);
  vector_t expected = inertialFlow;
  expected.head(6) += AdaptiveLeggedRobotDynamics::normalizedMomentumRateCorrection(wrench, nominal.robotMass);
  EXPECT_TRUE(dynamics.computeFlowMap(0.2, state, input, pre).isApprox(expected, 1e-13));

  // A/B/C reductions: no uncertainty -> nominal; each zeroed block recovers
  // exactly the already-established 10D or 6D Eq.(10) semantics.
  joint.setZero(); data->publish(adaptive::AdaptiveModelSnapshot::createValid(joint, 0.3, 8));
  EXPECT_TRUE(dynamics.computeFlowMap(0.3, state, input, pre).isApprox(nominalFlow, 1e-13));
  joint.head(10) = inertial; data->publish(adaptive::AdaptiveModelSnapshot::createValid(joint, 0.4, 9));
  EXPECT_TRUE(dynamics.computeFlowMap(0.4, state, input, pre).isApprox(inertialFlow, 1e-13));
  joint.setZero(); joint.tail(6) = wrench; data->publish(adaptive::AdaptiveModelSnapshot::createValid(joint, 0.5, 10));
  vector_t wrenchOnly = nominalFlow;
  wrenchOnly.head(6) += AdaptiveLeggedRobotDynamics::normalizedMomentumRateCorrection(wrench, nominal.robotMass);
  EXPECT_TRUE(dynamics.computeFlowMap(0.5, state, input, pre).isApprox(wrenchOnly, 1e-13));
}

TEST(PayloadInertialPlusWrench16D,
     ComparativeConfigurationMatchesEveryPublishedNumericGain) {
  const auto settings = adaptive::loadAdaptiveClfSettings(
      std::string(getPath()) +
      "/config/adaptive_settings_eth_paper_comparative_10d.info");
  const vector_t expectedGamma =
      (vector_t(10) << 5.0, 1.0, 1.0, 1.0,
       0.01, 0.01, 0.01, 0.01, 0.01, 0.01).finished();
  EXPECT_EQ(settings.parameterization,
            AdaptiveParameterization::PayloadInertial10D);
  EXPECT_EQ(settings.adaptiveDynamicsForm,
            adaptive::AdaptiveDynamicsForm::PaperRegressorCompensation);
  EXPECT_TRUE(settings.lambdaLinear.isApprox(
      5.0 * adaptive::matrix3_t::Identity(), 0.0));
  EXPECT_TRUE(settings.lambdaAngular.isApprox(
      5.0 * adaptive::matrix3_t::Identity(), 0.0));
  EXPECT_TRUE(settings.gamma.diagonal().isApprox(expectedGamma, 0.0));
  EXPECT_TRUE(settings.kd.diagonal().isApprox(
      (adaptive::vector6_t() << 50.0, 50.0, 50.0,
       80.0, 80.0, 80.0).finished(), 0.0));
  EXPECT_TRUE(settings.initialParameterEstimate.isZero(0.0));
  EXPECT_TRUE(settings.enableParameterUpdate);
  EXPECT_TRUE(settings.enableAdaptiveDynamics);
  EXPECT_TRUE(settings.enableClfConstraint);
  EXPECT_FALSE(settings.enableGaitPhaseBaselineCorrection);
  EXPECT_FALSE(settings.enableCompositePhaseBaselineCorrection);
}

TEST(PayloadInertialPlusWrench16D,
     FullPaperStructureKeepsAllSixteenParameters) {
  const auto settings = adaptive::loadAdaptiveClfSettings(
      std::string(getPath()) +
      "/config/adaptive_settings_eth_paper_full_16d.info");
  EXPECT_EQ(settings.parameterization,
            AdaptiveParameterization::PayloadInertial10DPlusWrench6D);
  EXPECT_EQ(settings.adaptiveDynamicsForm,
            adaptive::AdaptiveDynamicsForm::PaperRegressorCompensation);
  EXPECT_EQ(settings.gamma.rows(), 16);
  EXPECT_TRUE(settings.gamma.diagonal().head<10>().isApprox(
      (vector_t(10) << 5.0, 1.0, 1.0, 1.0,
       0.01, 0.01, 0.01, 0.01, 0.01, 0.01).finished(), 0.0));
  EXPECT_TRUE(settings.gamma.topRightCorner(10, 6).isZero(0.0));
  EXPECT_TRUE(settings.gamma.bottomLeftCorner(6, 10).isZero(0.0));
}

TEST(PayloadInertialPlusWrench16D,
     EquationTenSubtractsTheCompleteRegressorProduct) {
  adaptive::regressor_t y(6, 16);
  y = adaptive::regressor_t::Random(6, 16);
  const vector_t estimate = vector_t::LinSpaced(16, -0.4, 0.35);
  constexpr scalar_t nominalMass = 52.0;
  const adaptive::vector6_t correction =
      AdaptiveLeggedRobotDynamics::normalizedMomentumRateCorrection(
          y, estimate, nominalMass);
  EXPECT_TRUE(correction.isApprox(-(y * estimate) / nominalMass, 1.0e-14));
  EXPECT_FALSE(correction.isApprox(
      AdaptiveLeggedRobotDynamics::normalizedMomentumRateCorrection(
          estimate.tail<6>(), nominalMass), 1.0e-10));
}

TEST(PayloadInertialPlusWrench16D,
     PaperDynamicsUsesTheSameReferenceRegressorEndToEnd) {
  auto interface = makeInterface();
  const auto& nominal = interface->getCentroidalModelInfo();
  auto manager = interface->getSwitchedModelReferenceManagerPtr();

  vector_t state = interface->getInitialState();
  state.head<6>() << 0.12, -0.08, 0.05, 0.015, -0.01, 0.02;
  state.segment<3>(9) << 0.06, -0.04, 0.08;
  vector_t input = vector_t::Zero(nominal.inputDim);
  input.head<12>() = vector_t::LinSpaced(12, 75.0, 135.0);
  manager->setTargetTrajectories(
      TargetTrajectories({0.0}, {state}, {input}));
  manager->preSolverRun(0.0, 0.1, state);

  vector_t estimate = vector_t::LinSpaced(16, -0.3, 0.45);
  auto data = std::make_shared<adaptive::AdaptiveModelData>(
      adaptive::AdaptiveModelSnapshot::createValid(estimate, 0.0, 1));
  auto settings = jointSettings();
  settings.adaptiveDynamicsForm =
      adaptive::AdaptiveDynamicsForm::PaperRegressorCompensation;
  settings.enableEncodedReferenceDerivatives = true;

  AdaptiveLeggedRobotDynamics dynamics(
      interface->getPinocchioInterface(), nominal, "paper_joint_dynamics",
      interface->modelSettings(), settings, data, manager.get());
  NominalCoordinatePlantDynamics nominalDynamics(
      interface->getPinocchioInterface(), nominal, nominal);
  PreComputation pre;
  vector_t expected =
      nominalDynamics.computeFlowMap(0.0, state, input, pre);

  adaptive::AdaptiveReferenceKinematicsAdapter referenceAdapter(
      interface->getPinocchioInterface(), nominal, settings);
  const auto reference = referenceAdapter.compute(
      0.0, state, manager->getTargetTrajectories());
  adaptive::AdaptiveRigidBodyRegressor regressor(
      {nominal.robotMass, nominal.centroidalInertiaNominal,
       adaptive::vector3_t(0.0, 0.0, -9.81)});
  adaptive::AdaptiveRigidBodyRegressor::EvaluationPoint point;
  point.rotationWorldFromBase =
      reference.actual.orientationWorldFromBase.toRotationMatrix();
  point.actualAngularVelocityWorld =
      reference.actual.angularVelocityWorld;
  point.referenceLinearAccelerationWorld =
      reference.referenceLinearAccelerationWorld;
  point.referenceAngularVelocityWorld =
      reference.referenceAngularVelocityWorld;
  point.referenceAngularAccelerationWorld =
      reference.referenceAngularAccelerationWorld;
  const adaptive::regressor_t y =
      regressor.computeUnknownDynamicsRegressor(
          point,
          AdaptiveParameterization::PayloadInertial10DPlusWrench6D,
          adaptive::AdaptiveDynamicsForm::PaperRegressorCompensation);
  expected.head<6>() -= y * estimate / nominal.robotMass;

  EXPECT_TRUE(dynamics.computeFlowMap(0.0, state, input, pre)
                  .isApprox(expected, 1.0e-12));
  std::unique_ptr<SystemDynamicsBase> clone(dynamics.clone());
  EXPECT_TRUE(clone->computeFlowMap(0.0, state, input, pre)
                  .isApprox(expected, 1.0e-12));

  const vector_t stateDirection =
      vector_t::LinSpaced(state.size(), -0.15, 0.18);
  const vector_t inputDirection =
      vector_t::LinSpaced(input.size(), 0.7, -0.5);
  constexpr scalar_t epsilon = 2.0e-6;
  const vector_t plus = dynamics.computeFlowMap(
      0.0, state + epsilon * stateDirection,
      input + epsilon * inputDirection, pre);
  const vector_t minus = dynamics.computeFlowMap(
      0.0, state - epsilon * stateDirection,
      input - epsilon * inputDirection, pre);
  const vector_t finiteDifference = (plus - minus) / (2.0 * epsilon);
  const auto approximation =
      dynamics.linearApproximation(0.0, state, input, pre);
  const vector_t analytic =
      approximation.dfdx * stateDirection +
      approximation.dfdu * inputDirection;
  EXPECT_LT((finiteDifference - analytic).norm() /
                (1.0 + finiteDifference.norm()),
            2.0e-4);
}
TEST(PayloadInertialPlusWrench16D,
     PaperDynamicsCloneRetainsEncodedReferenceDerivatives) {
  auto interface = makeInterface();
  const auto& nominal = interface->getCentroidalModelInfo();
  auto manager = interface->getSwitchedModelReferenceManagerPtr();

  vector_t state = interface->getInitialState();
  vector_t target0 = state;
  vector_t target1 = state;
  target0.head<6>().setZero();
  target1.head<6>() << 0.35, -0.18, 0.12, 0.08, -0.05, 0.04;
  vector_t input = vector_t::Zero(nominal.inputDim);
  manager->setTargetTrajectories(
      TargetTrajectories({0.0, 1.0}, {target0, target1}, {input, input}));
  manager->preSolverRun(0.5, 0.6, state);

  vector_t estimate = vector_t::LinSpaced(16, -0.3, 0.45);
  auto data = std::make_shared<adaptive::AdaptiveModelData>(
      adaptive::AdaptiveModelSnapshot::createValid(estimate, 0.5, 1));
  auto settings = jointSettings();
  settings.adaptiveDynamicsForm =
      adaptive::AdaptiveDynamicsForm::PaperRegressorCompensation;
  settings.enableEncodedReferenceDerivatives = true;

  AdaptiveLeggedRobotDynamics dynamics(
      interface->getPinocchioInterface(), nominal,
      "paper_clone_reference_contract", interface->modelSettings(),
      settings, data, manager.get());
  std::unique_ptr<SystemDynamicsBase> clone(dynamics.clone());
  PreComputation pre;
  EXPECT_TRUE(clone->computeFlowMap(0.5, state, input, pre).isApprox(
      dynamics.computeFlowMap(0.5, state, input, pre), 1.0e-12));
}
} } }  // namespace ocs2::legged_robot_adaptive
