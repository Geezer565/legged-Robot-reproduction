#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <boost/filesystem/path.hpp>
#include <memory>

#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ocs2_core/PreComputation.h>
#include <ocs2_legged_robot/LeggedRobotInterface.h>

#include "ocs2_legged_robot_adaptive/adaptive/PayloadInertial10D.h"
#include "ocs2_legged_robot_adaptive/adaptive/PayloadMass1D.h"
#include "ocs2_legged_robot_adaptive/adaptive/AdaptiveClf.h"
#include "ocs2_legged_robot_adaptive/adaptive/AdaptiveModelData.h"
#include "ocs2_legged_robot_adaptive/adaptive/AdaptiveParameterEstimator.h"
#include "ocs2_legged_robot_adaptive/dynamics/AdaptiveLeggedRobotDynamics.h"
#include "ocs2_legged_robot_adaptive/dynamics/NominalCoordinatePlantDynamics.h"
#include "ocs2_legged_robot_adaptive/dynamics/PayloadInertialRuntimeDiagnostics.h"
#include "ocs2_legged_robot_adaptive/dynamics/PayloadInertialGroundTruth.h"
#include "ocs2_legged_robot_adaptive/AdaptiveLeggedRobotInterface.h"
#include "ocs2_legged_robot_adaptive/package_path.h"

namespace ocs2 {
namespace legged_robot_adaptive {
namespace {

using adaptive::PayloadInertial10D;
using adaptive::matrix3_t;
using adaptive::vector3_t;

matrix3_t parallelAxis(const vector3_t& displacement) {
  return displacement.squaredNorm() * matrix3_t::Identity() -
         displacement * displacement.transpose();
}

PayloadInertial10D::Parameters makePhysicalPayload() {
  const scalar_t mass = 5.213485;
  const vector3_t position(0.16, -0.03, 0.09);
  matrix3_t inertiaAtPayloadCom = matrix3_t::Zero();
  inertiaAtPayloadCom.diagonal() << 0.08, 0.10, 0.12;
  return PayloadInertial10D::fromPhysicalPayload(
      mass, position, inertiaAtPayloadCom);
}

std::unique_ptr<legged_robot::LeggedRobotInterface> makeInterface() {
  const boost::filesystem::path adaptivePath(getPath());
  const auto sourcePath = adaptivePath.parent_path();
  const auto leggedPath =
      sourcePath / "ocs2/ocs2_robotic_examples/ocs2_legged_robot";
  return std::unique_ptr<legged_robot::LeggedRobotInterface>(
      new legged_robot::LeggedRobotInterface(
      (leggedPath / "config/mpc/task.info").string(),
      (sourcePath /
       "ocs2_robotic_assets/resources/anymal_c/urdf/anymal.urdf")
          .string(),
      (leggedPath / "config/command/reference.info").string()));
}

adaptive::AdaptiveClfSettings makePayloadInertialSettings() {
  adaptive::AdaptiveClfSettings settings;
  settings.parameterization =
      adaptive::AdaptiveParameterization::PayloadInertial10D;
  settings.gamma = matrix_t::Identity(10, 10);
  settings.initialParameterEstimate = vector_t::Zero(10);
  settings.enableParameterUpdate = false;
  settings.enableAdaptiveDynamics = false;
  adaptive::validateAdaptiveClfSettings(settings);
  return settings;
}

TEST(PayloadInertial10D, PhysicalPayloadRoundTripUsesFirstMomentAndParallelAxis) {
  const scalar_t mass = 4.5;
  const vector3_t position(0.20, -0.05, 0.12);
  matrix3_t inertiaAtPayloadCom = matrix3_t::Zero();
  inertiaAtPayloadCom.diagonal() << 0.06, 0.08, 0.10;

  const auto parameters = PayloadInertial10D::fromPhysicalPayload(
      mass, position, inertiaAtPayloadCom);
  EXPECT_TRUE(parameters.firstMomentNominalComBase.isApprox(
      mass * position, 1.0e-14));
  EXPECT_TRUE(parameters.inertiaAboutNominalComBase.isApprox(
      inertiaAtPayloadCom + mass * parallelAxis(position), 1.0e-14));
  EXPECT_TRUE(PayloadInertial10D::inertiaAboutPayloadCom(parameters).isApprox(
      inertiaAtPayloadCom, 1.0e-13));
  EXPECT_TRUE(PayloadInertial10D::isPhysicallyConsistent(parameters));

  const adaptive::vector10_t vector = PayloadInertial10D::toVector(parameters);
  EXPECT_EQ(vector.size(), 10);
  const auto roundTrip = PayloadInertial10D::fromVector(vector);
  EXPECT_NEAR(roundTrip.mass, mass, 0.0);
  EXPECT_TRUE(roundTrip.firstMomentNominalComBase.isApprox(
      parameters.firstMomentNominalComBase, 0.0));
  EXPECT_TRUE(roundTrip.inertiaAboutNominalComBase.isApprox(
      parameters.inertiaAboutNominalComBase, 0.0));
}

TEST(PayloadInertial10D, PhysicalValidationAcceptsZeroAndPointMassButRejectsInvalidInertia) {
  EXPECT_TRUE(PayloadInertial10D::isPhysicallyConsistent(
      PayloadInertial10D::Parameters{}));

  matrix3_t zeroInertia = matrix3_t::Zero();
  const auto pointMass = PayloadInertial10D::fromPhysicalPayload(
      2.0, vector3_t::Zero(), zeroInertia);
  EXPECT_TRUE(PayloadInertial10D::isPhysicallyConsistent(pointMass));

  matrix3_t invalidInertia = matrix3_t::Zero();
  invalidInertia.diagonal() << 0.01, 0.01, 1.0;
  EXPECT_THROW(PayloadInertial10D::fromPhysicalPayload(
                   2.0, vector3_t::Zero(), invalidInertia),
               std::invalid_argument);

  auto negativeMass = pointMass;
  negativeMass.mass = -1.0;
  EXPECT_FALSE(PayloadInertial10D::isPhysicallyConsistent(negativeMass));
  EXPECT_THROW(PayloadInertial10D::validatePhysicalPayload(negativeMass),
               std::domain_error);
}

TEST(PayloadInertial10D, MassColumnExactlyReducesToPayloadMass1D) {
  PayloadInertial10D::ReferencePoint point;
  point.referenceLinearAccelerationNominalComWorld << 0.5, -0.25, 1.0;
  const adaptive::matrix6x10_t regressor =
      PayloadInertial10D::referenceRegressor(point);
  EXPECT_EQ(regressor.rows(), 6);
  EXPECT_EQ(regressor.cols(), 10);
  EXPECT_TRUE(regressor.col(0).isApprox(
      adaptive::PayloadMass1D::referenceRegressorColumn(
          point.referenceLinearAccelerationNominalComWorld,
          point.gravityWorld),
      1.0e-14));
}

TEST(PayloadInertial10D, ReferenceRegressorMatchesDirectRigidPayloadWrench) {
  PayloadInertial10D::ReferencePoint point;
  point.rotationWorldFromBase =
      Eigen::AngleAxis<scalar_t>(0.31, vector3_t(0.2, -0.7, 0.4).normalized())
          .toRotationMatrix();
  point.actualAngularVelocityWorld << 0.4, -0.3, 0.2;
  point.referenceAngularVelocityWorld << -0.1, 0.25, 0.35;
  point.referenceAngularAccelerationWorld << 0.6, -0.2, 0.15;
  point.referenceLinearAccelerationNominalComWorld << 0.7, -0.4, 0.3;

  const auto parameters = makePhysicalPayload();
  const adaptive::vector10_t pi = PayloadInertial10D::toVector(parameters);
  const adaptive::vector6_t fromRegressor =
      PayloadInertial10D::referenceWrench(point, pi);

  const vector3_t hWorld = point.rotationWorldFromBase *
                           parameters.firstMomentNominalComBase;
  const matrix3_t inertiaWorld =
      point.rotationWorldFromBase *
      parameters.inertiaAboutNominalComBase *
      point.rotationWorldFromBase.transpose();
  const vector3_t effectiveAcceleration =
      point.referenceLinearAccelerationNominalComWorld - point.gravityWorld;
  adaptive::vector6_t direct;
  direct.head<3>() =
      parameters.mass * effectiveAcceleration +
      point.referenceAngularAccelerationWorld.cross(hWorld) +
      point.actualAngularVelocityWorld.cross(
          point.referenceAngularVelocityWorld.cross(hWorld));
  direct.tail<3>() =
      inertiaWorld * point.referenceAngularAccelerationWorld +
      point.actualAngularVelocityWorld.cross(
          inertiaWorld * point.referenceAngularVelocityWorld) +
      hWorld.cross(effectiveAcceleration);
  EXPECT_TRUE(fromRegressor.isApprox(direct, 1.0e-12));

  // The map must be linear even when the supplied vector is not itself a
  // physically valid standalone rigid body.
  adaptive::vector10_t piA = adaptive::vector10_t::LinSpaced(10, -0.3, 0.6);
  adaptive::vector10_t piB = adaptive::vector10_t::LinSpaced(10, 0.2, 1.1);
  EXPECT_TRUE(PayloadInertial10D::referenceWrench(point, piA + piB).isApprox(
      PayloadInertial10D::referenceWrench(point, piA) +
          PayloadInertial10D::referenceWrench(point, piB),
      1.0e-12));
}

TEST(PayloadInertial10D, ReferenceRegressorMatchesIndependentNewtonEulerWrench) {
  PayloadInertial10D::ReferencePoint point;
  point.rotationWorldFromBase =
      Eigen::AngleAxis<scalar_t>(-0.27,
                                 vector3_t(0.5, 0.1, -0.3).normalized())
          .toRotationMatrix();
  point.actualAngularVelocityWorld << 0.32, -0.18, 0.41;
  point.referenceAngularVelocityWorld = point.actualAngularVelocityWorld;
  point.referenceAngularAccelerationWorld << -0.22, 0.37, 0.14;
  point.referenceLinearAccelerationNominalComWorld << 0.45, -0.28, 0.19;

  constexpr scalar_t mass = 3.7;
  const vector3_t payloadPositionBase(0.18, -0.06, 0.11);
  matrix3_t payloadComInertiaBase;
  payloadComInertiaBase << 0.075, 0.004, -0.003,
                           0.004, 0.091, 0.002,
                          -0.003, 0.002, 0.104;
  const auto payload = PayloadInertial10D::fromPhysicalPayload(
      mass, payloadPositionBase, payloadComInertiaBase);
  const adaptive::vector6_t regressorWrench =
      PayloadInertial10D::referenceWrench(
          point, PayloadInertial10D::toVector(payload));

  // Independent Newton-Euler construction at the physical payload CoM.
  const vector3_t rWorld =
      point.rotationWorldFromBase * payloadPositionBase;
  const matrix3_t payloadComInertiaWorld =
      point.rotationWorldFromBase * payloadComInertiaBase *
      point.rotationWorldFromBase.transpose();
  const vector3_t payloadComAcceleration =
      point.referenceLinearAccelerationNominalComWorld +
      point.referenceAngularAccelerationWorld.cross(rWorld) +
      point.actualAngularVelocityWorld.cross(
          point.actualAngularVelocityWorld.cross(rWorld));
  const vector3_t force =
      mass * (payloadComAcceleration - point.gravityWorld);
  const vector3_t torqueAtPayloadCom =
      payloadComInertiaWorld * point.referenceAngularAccelerationWorld +
      point.actualAngularVelocityWorld.cross(
          payloadComInertiaWorld * point.actualAngularVelocityWorld);
  adaptive::vector6_t newtonEulerWrench;
  newtonEulerWrench << force, torqueAtPayloadCom + rWorld.cross(force);
  EXPECT_TRUE(regressorWrench.isApprox(newtonEulerWrench, 1.0e-12));
}

TEST(PayloadInertial10D, StaticOffCenterPayloadHasExpectedGravityForceAndTorqueSign) {
  constexpr scalar_t mass = 5.0;
  const vector3_t position(0.20, 0.0, 0.10);
  const auto payload = PayloadInertial10D::fromPhysicalPayload(
      mass, position, matrix3_t::Zero());
  PayloadInertial10D::ReferencePoint point;
  const adaptive::vector6_t wrench = PayloadInertial10D::referenceWrench(
      point, PayloadInertial10D::toVector(payload));

  adaptive::vector6_t expected = adaptive::vector6_t::Zero();
  const vector3_t effectiveAcceleration = -point.gravityWorld;
  expected.head<3>() = mass * effectiveAcceleration;
  expected.tail<3>() = (mass * position).cross(effectiveAcceleration);
  EXPECT_TRUE(wrench.isApprox(expected, 1.0e-13));
  EXPECT_GT(wrench(2), 0.0);
  EXPECT_LT(wrench(4), 0.0);
}

TEST(PayloadInertial10D, PlantCompositionPreservesNominalAndMovesTotalComConsistently) {
  auto interface = makeInterface();
  const auto& nominal = interface->getCentroidalModelInfo();
  const auto zeroInfo = PayloadInertial10D::makePlantModelInfo(
      nominal, PayloadInertial10D::Parameters{});
  EXPECT_NEAR(zeroInfo.robotMass, nominal.robotMass, 0.0);
  EXPECT_TRUE(zeroInfo.comToBasePositionNominal.isApprox(
      nominal.comToBasePositionNominal, 0.0));
  EXPECT_TRUE(zeroInfo.centroidalInertiaNominal.isApprox(
      nominal.centroidalInertiaNominal, 0.0));

  const auto payload = makePhysicalPayload();
  const auto trueInfo =
      PayloadInertial10D::makePlantModelInfo(nominal, payload);
  EXPECT_NEAR(trueInfo.robotMass, nominal.robotMass + payload.mass, 1.0e-13);
  const vector3_t expectedTotalComShift =
      payload.firstMomentNominalComBase / trueInfo.robotMass;
  const vector3_t actualTotalComShift =
      nominal.comToBasePositionNominal -
      trueInfo.comToBasePositionNominal;
  EXPECT_TRUE(actualTotalComShift.isApprox(expectedTotalComShift, 1.0e-14));
  EXPECT_GT((trueInfo.centroidalInertiaNominal -
             nominal.centroidalInertiaNominal)
                .norm(),
            1.0e-6);
}

TEST(PayloadInertial10D,
     EstimatedIncrementNeedsOnlyAValidCombinedModelAndIsNeverProjected) {
  auto interface = makeInterface();
  const auto& nominal = interface->getCentroidalModelInfo();

  // A first moment without standalone payload mass is not a physical payload,
  // but it is a valid local inertial increment when the combined model remains
  // physical. The online path must retain it exactly rather than project it.
  adaptive::vector10_t estimate = adaptive::vector10_t::Zero();
  estimate(1) = 0.05;
  EXPECT_FALSE(PayloadInertial10D::isPhysicallyConsistent(
      PayloadInertial10D::fromVector(estimate)));
  const auto estimatedInfo = PayloadInertial10D::makeEstimatedModelInfo(
      nominal, estimate, 1.0);
  EXPECT_NEAR(estimatedInfo.robotMass, nominal.robotMass, 0.0);
  EXPECT_GT((estimatedInfo.comToBasePositionNominal -
             nominal.comToBasePositionNominal).norm(),
            1.0e-8);

  adaptive::vector10_t invalid = adaptive::vector10_t::Zero();
  invalid(0) = -nominal.robotMass;
  EXPECT_THROW(PayloadInertial10D::makeEstimatedModelInfo(
                   nominal, invalid, 1.0),
               std::domain_error);
}

TEST(PayloadInertial10D, RuntimeOffSettingsLoadWithExactTenDimensionalSchema) {
  const boost::filesystem::path adaptivePath(getPath());
  const auto settings = adaptive::loadAdaptiveClfSettings(
      (adaptivePath /
       "config/adaptive_settings_payload_inertial_10d_runtime_off.info")
          .string());
  EXPECT_EQ(settings.parameterization,
            adaptive::AdaptiveParameterization::PayloadInertial10D);
  EXPECT_EQ(adaptive::parameterDimension(settings.parameterization), 10);
  EXPECT_EQ(settings.gamma.rows(), 10);
  EXPECT_EQ(settings.gamma.cols(), 10);
  EXPECT_EQ(settings.initialParameterEstimate.size(), 10);
  EXPECT_TRUE(settings.initialParameterEstimate.isZero(0.0));
  EXPECT_FALSE(settings.enableParameterUpdate);
  EXPECT_FALSE(settings.enableAdaptiveDynamics);
  EXPECT_STREQ(adaptive::adaptiveParameterizationName(
                   settings.parameterization),
               "payload_inertial_10d");

  auto invalid = settings;
  invalid.gamma = matrix_t::Identity(6, 6);
  EXPECT_THROW(adaptive::validateAdaptiveClfSettings(invalid),
               std::invalid_argument);
}

TEST(PayloadInertial10D,
     RuntimeOffInterfaceSharesTenDimensionalSnapshotAcrossEstimatorAndClf) {
  const boost::filesystem::path adaptivePath(getPath());
  const auto sourcePath = adaptivePath.parent_path();
  const auto leggedPath =
      sourcePath / "ocs2/ocs2_robotic_examples/ocs2_legged_robot";
  AdaptiveLeggedRobotInterface interface(
      (leggedPath / "config/mpc/task.info").string(),
      (sourcePath /
       "ocs2_robotic_assets/resources/anymal_c/urdf/anymal.urdf")
          .string(),
      (leggedPath / "config/command/reference.info").string(), false,
      (adaptivePath /
       "config/adaptive_settings_payload_inertial_10d_runtime_off.info")
          .string());
  EXPECT_FALSE(interface.getAdaptiveClfSettings().enableParameterUpdate);
  EXPECT_FALSE(interface.getAdaptiveClfSettings().enableAdaptiveDynamics);
  ASSERT_NE(interface.getAdaptiveParameterUpdateModulePtr(), nullptr);
  ASSERT_NE(interface.getAdaptiveClfEvaluatorPtr(), nullptr);
  ASSERT_NE(interface.getAdaptiveClfConstraintPtr(), nullptr);
  const auto snapshot = interface.getAdaptiveModelDataPtr()->snapshot();
  ASSERT_NE(snapshot, nullptr);
  EXPECT_EQ(snapshot->estimate().size(), 10);

  const vector_t state = interface.getInitialState();
  const vector_t input =
      vector_t::Zero(interface.getCentroidalModelInfo().inputDim);
  const auto manager = interface.getSwitchedModelReferenceManagerPtr();
  manager->setTargetTrajectories(
      TargetTrajectories({0.0}, {state}, {input}));
  manager->preSolverRun(0.0, 0.1, state);
  const auto diagnostics = interface.getAdaptiveClfEvaluatorPtr()->evaluate(
      0.0, state, input, snapshot);
  EXPECT_EQ(diagnostics.piHat.size(), 10);
  EXPECT_EQ(diagnostics.parameterDerivative.size(), 10);
  EXPECT_EQ(diagnostics.unknownReferenceRegressor.rows(), 6);
  EXPECT_EQ(diagnostics.unknownReferenceRegressor.cols(), 10);
  EXPECT_TRUE(diagnostics.usesFullFlowAdaptiveDynamics);
  EXPECT_TRUE(diagnostics.estimatedUnknownWrench.isZero(0.0));
}

TEST(PayloadInertial10D, EquationEightUsesFullSixByTenRegressor) {
  auto settings = makePayloadInertialSettings();
  settings.gamma.diagonal() = vector_t::LinSpaced(10, 0.5, 2.0);
  adaptive::AdaptiveParameterEstimator estimator(settings);
  estimator.reset(vector_t::Zero(10), 0.0);

  PayloadInertial10D::ReferencePoint point;
  point.rotationWorldFromBase =
      Eigen::AngleAxis<scalar_t>(0.23, vector3_t(0.3, -0.4, 0.2).normalized())
          .toRotationMatrix();
  point.actualAngularVelocityWorld << 0.2, -0.3, 0.1;
  point.referenceLinearAccelerationNominalComWorld << 0.4, -0.2, 0.3;
  point.referenceAngularVelocityWorld << -0.1, 0.25, 0.2;
  point.referenceAngularAccelerationWorld << 0.5, -0.15, 0.35;

  adaptive::AdaptiveEstimatorUpdate update;
  update.sigma << 0.3, -0.1, 0.25, -0.2, 0.4, 0.15;
  update.regressor = PayloadInertial10D::referenceRegressor(point);
  update.dt = 0.02;
  update.time = 0.02;
  const vector_t expected =
      update.dt * settings.gamma * update.regressor.transpose() *
      update.sigma;
  const auto snapshot = estimator.update(update);
  ASSERT_EQ(snapshot->estimate().size(), 10);
  EXPECT_TRUE(snapshot->estimate().isApprox(expected, 1.0e-14));
  EXPECT_EQ(snapshot->version(), 2);
}

TEST(PayloadInertial10D, EquationElevenAcceptsSixByTenRegressorExactly) {
  auto settings = makePayloadInertialSettings();
  adaptive::AdaptiveClf clf(settings);
  adaptive::AdaptiveClfInput input;
  input.sigma << 0.2, -0.3, 0.1, 0.4, -0.2, 0.5;
  input.contactWrenchWorld << 12.0, -3.0, 8.0, 1.0, -2.0, 0.5;
  input.nominalReferenceWrenchWorld << 2.0, 1.0, 5.0, -1.0, 0.2, 0.1;
  input.unknownRegressor = adaptive::regressor_t::Random(6, 10);
  input.parameterEstimate = vector_t::LinSpaced(10, -0.4, 0.7);
  const scalar_t expected = input.sigma.dot(
      input.contactWrenchWorld - input.nominalReferenceWrenchWorld -
      input.unknownRegressor * input.parameterEstimate) -
      0.5 * input.sigma.dot(settings.kd * input.sigma);
  EXPECT_NEAR(clf.computeValue(input), expected, 1.0e-14);

  input.parameterEstimate.conservativeResize(9);
  EXPECT_THROW(clf.computeValue(input), std::invalid_argument);
}

TEST(PayloadInertial10D, FullFlowAndPhysicalCoordinatePrototypeCloseExactly) {
  auto interface = makeInterface();
  const auto& nominalInfo = interface->getCentroidalModelInfo();
  const auto payload = makePhysicalPayload();
  PayloadInertialGroundTruth groundTruth(
      interface->getPinocchioInterface(), nominalInfo, payload);

  vector_t state = interface->getInitialState();
  state.head<6>() << 0.35, -0.12, 0.08, 0.025, -0.018, 0.031;
  state.segment<3>(9) << 0.08, -0.06, 0.12;
  vector_t input = vector_t::Zero(nominalInfo.inputDim);
  input.segment<3>(0) << 130.0, -12.0, 170.0;
  input.segment<3>(3) << -25.0, 8.0, 115.0;
  input.segment<3>(6) << 15.0, 4.0, 90.0;
  input.segment<3>(9) << -8.0, -3.0, 105.0;
  input.tail(nominalInfo.actuatedDofNum) =
      vector_t::LinSpaced(nominalInfo.actuatedDofNum, -0.15, 0.18);

  const auto sample = groundTruth.evaluate(0.4, state, input);
  EXPECT_GT(sample.normalizedMomentumRateResidualNorm, 1.0e-8);
  EXPECT_GT(sample.coordinateFlowResidualNorm, 1.0e-8);
  EXPECT_NEAR(sample.fullFlowResidualNorm,
              sample.fullFlowResidual.norm(), 0.0);

  // Frozen estimate == physical truth: an estimated-info full flow is exactly
  // the true flow. This is the D1A closure target that a head-6 wrench cannot
  // satisfy when the coordinate-flow residual is nonzero.
  MassMismatchPlantDynamics estimatedDynamics(
      interface->getPinocchioInterface(),
      PayloadInertial10D::makePlantModelInfo(nominalInfo, payload));
  PreComputation preComputation;
  const vector_t estimatedFlow = estimatedDynamics.computeFlowMap(
      0.4, state, input, preComputation);
  EXPECT_TRUE(estimatedFlow.isApprox(sample.trueFlow, 1.0e-13));

  vector_t headOnlyReconstruction = sample.nominalFlow;
  headOnlyReconstruction.head<6>() +=
      sample.normalizedMomentumRateResidual;
  EXPECT_NEAR((sample.trueFlow - headOnlyReconstruction).norm(),
              sample.coordinateFlowResidualNorm, 1.0e-13);
  EXPECT_GT((sample.trueFlow - headOnlyReconstruction).norm(), 1.0e-8);

  // Verify the analytic full-flow linearization of the estimated-info model.
  const auto approximation = estimatedDynamics.linearApproximation(
      0.4, state, input, preComputation);
  vector_t stateDirection = vector_t::Zero(state.size());
  stateDirection.head<6>() << 0.13, -0.08, 0.04, 0.02, -0.03, 0.01;
  stateDirection.segment<3>(6) << 0.05, -0.02, 0.03;
  stateDirection.segment<3>(9) << -0.04, 0.03, 0.02;
  vector_t inputDirection = vector_t::Zero(input.size());
  inputDirection.head<12>() = vector_t::LinSpaced(12, -0.7, 0.9);
  inputDirection.tail(nominalInfo.actuatedDofNum) =
      vector_t::LinSpaced(nominalInfo.actuatedDofNum, 0.05, -0.06);
  constexpr scalar_t epsilon = 1.0e-6;
  const vector_t plus = estimatedDynamics.computeFlowMap(
      0.4, state + epsilon * stateDirection,
      input + epsilon * inputDirection, preComputation);
  const vector_t minus = estimatedDynamics.computeFlowMap(
      0.4, state - epsilon * stateDirection,
      input - epsilon * inputDirection, preComputation);
  const vector_t finiteDifference = (plus - minus) / (2.0 * epsilon);
  const vector_t analytic = approximation.dfdx * stateDirection +
                            approximation.dfdu * inputDirection;
  EXPECT_LT((finiteDifference - analytic).norm() /
                (1.0 + finiteDifference.norm()),
            2.0e-5);

  // Same physical RBD state maps to different nominal/true normalized
  // momentum coordinates, while each inverse conversion recovers the same
  // physical pose and velocity. This proves why plant and controller states
  // must be isolated before a 10D dummy experiment.
  const std::size_t nq = nominalInfo.generalizedCoordinatesNum;
  vector_t rbdState = vector_t::Zero(2 * nq);
  rbdState.head<3>() = state.segment<3>(9);       // orientation
  rbdState.segment<3>(3) = state.segment<3>(6);  // position
  rbdState.segment(6, nominalInfo.actuatedDofNum) =
      state.tail(nominalInfo.actuatedDofNum);
  rbdState.segment<3>(nq) << 0.22, -0.17, 0.11;      // angular velocity
  rbdState.segment<3>(nq + 3) << 0.31, 0.07, -0.04; // linear velocity
  rbdState.tail(nominalInfo.actuatedDofNum) =
      input.tail(nominalInfo.actuatedDofNum);

  CentroidalModelRbdConversions nominalConversions(
      interface->getPinocchioInterface(), nominalInfo);
  CentroidalModelRbdConversions trueConversions(
      interface->getPinocchioInterface(), groundTruth.trueInfo());
  const vector_t nominalState =
      nominalConversions.computeCentroidalStateFromRbdModel(rbdState);
  const vector_t trueState =
      trueConversions.computeCentroidalStateFromRbdModel(rbdState);
  EXPECT_TRUE(nominalState.tail(nominalInfo.generalizedCoordinatesNum)
                  .isApprox(trueState.tail(nominalInfo.generalizedCoordinatesNum),
                            1.0e-14));
  EXPECT_GT((nominalState.head<6>() - trueState.head<6>()).norm(), 1.0e-8);

  const vector_t nominalRecovered =
      nominalConversions.computeRbdStateFromCentroidalModel(nominalState,
                                                             input);
  const vector_t trueRecovered =
      trueConversions.computeRbdStateFromCentroidalModel(trueState, input);
  EXPECT_TRUE(nominalRecovered.isApprox(rbdState, 1.0e-10));
  EXPECT_TRUE(trueRecovered.isApprox(rbdState, 1.0e-10));
}

TEST(PayloadInertial10D,
     NominalCoordinatePlantFlowRepresentsTheSamePhysicalDerivative) {
  auto interface = makeInterface();
  const auto& nominalInfo = interface->getCentroidalModelInfo();
  const auto payload = makePhysicalPayload();
  const auto trueInfo =
      PayloadInertial10D::makePlantModelInfo(nominalInfo, payload);
  NominalCoordinatePlantDynamics coordinateDynamics(
      interface->getPinocchioInterface(), nominalInfo, trueInfo);
  MassMismatchPlantDynamics trueDynamics(
      interface->getPinocchioInterface(), trueInfo);

  vector_t nominalState = interface->getInitialState();
  nominalState.head<6>() << 0.31, -0.14, 0.09, 0.028, -0.019, 0.033;
  nominalState.segment<3>(9) << 0.09, -0.07, 0.11;
  vector_t input = vector_t::Zero(nominalInfo.inputDim);
  input.head<12>() = vector_t::LinSpaced(12, 85.0, 145.0);
  input.tail(nominalInfo.actuatedDofNum) =
      vector_t::LinSpaced(nominalInfo.actuatedDofNum, -0.12, 0.16);
  PreComputation preComputation;

  const vector_t trueState =
      coordinateDynamics.plantStateFromNominalState(nominalState);
  EXPECT_GT((trueState.head<6>() - nominalState.head<6>()).norm(), 1.0e-8);
  EXPECT_TRUE(coordinateDynamics.nominalStateFromPlantState(trueState)
                  .isApprox(nominalState, 1.0e-12));

  const vector_t trueFlow = trueDynamics.computeFlowMap(
      0.35, trueState, input, preComputation);
  const vector_t nominalCoordinateFlow = coordinateDynamics.computeFlowMap(
      0.35, nominalState, input, preComputation);
  constexpr scalar_t epsilon = 1.0e-7;
  const vector_t finiteDifference =
      (coordinateDynamics.nominalStateFromPlantState(
           trueState + epsilon * trueFlow) -
       coordinateDynamics.nominalStateFromPlantState(
           trueState - epsilon * trueFlow)) /
      (2.0 * epsilon);
  EXPECT_LT((finiteDifference - nominalCoordinateFlow).norm() /
                (1.0 + finiteDifference.norm()),
            2.0e-7);

  const auto approximation = coordinateDynamics.linearApproximation(
      0.35, nominalState, input, preComputation);
  vector_t stateDirection = vector_t::LinSpaced(
      nominalState.size(), -0.15, 0.18);
  vector_t inputDirection = vector_t::LinSpaced(
      input.size(), 0.8, -0.6);
  constexpr scalar_t linearizationStep = 2.0e-6;
  const vector_t plus = coordinateDynamics.computeFlowMap(
      0.35, nominalState + linearizationStep * stateDirection,
      input + linearizationStep * inputDirection, preComputation);
  const vector_t minus = coordinateDynamics.computeFlowMap(
      0.35, nominalState - linearizationStep * stateDirection,
      input - linearizationStep * inputDirection, preComputation);
  const vector_t finiteDifferenceLinearization =
      (plus - minus) / (2.0 * linearizationStep);
  const vector_t analytic = approximation.dfdx * stateDirection +
                            approximation.dfdu * inputDirection;
  EXPECT_LT((finiteDifferenceLinearization - analytic).norm() /
                (1.0 + finiteDifferenceLinearization.norm()),
            3.0e-5);
}

TEST(PayloadInertial10D,
     FrozenSnapshotDrivesCompleteAdaptiveFlowLinearizationAndClone) {
  auto interface = makeInterface();
  const auto& nominalInfo = interface->getCentroidalModelInfo();
  const auto payload = makePhysicalPayload();
  const adaptive::vector10_t truth = PayloadInertial10D::toVector(payload);
  auto modelData = std::make_shared<adaptive::AdaptiveModelData>(
      adaptive::AdaptiveModelSnapshot::createValid(truth, 0.2, 1));
  auto settings = makePayloadInertialSettings();
  settings.enableAdaptiveDynamics = true;
  AdaptiveLeggedRobotDynamics adaptiveDynamics(
      interface->getPinocchioInterface(), nominalInfo, "dynamics",
      interface->modelSettings(), settings, modelData);
  NominalCoordinatePlantDynamics expectedDynamics(
      interface->getPinocchioInterface(), nominalInfo,
      PayloadInertial10D::makePlantModelInfo(nominalInfo, payload));

  vector_t state = interface->getInitialState();
  state.head<6>() << 0.23, -0.11, 0.07, 0.02, -0.015, 0.025;
  state.segment<3>(9) << 0.06, -0.04, 0.09;
  vector_t input = vector_t::Zero(nominalInfo.inputDim);
  input.head<12>() = vector_t::LinSpaced(12, 70.0, 150.0);
  input.tail(nominalInfo.actuatedDofNum) =
      vector_t::LinSpaced(nominalInfo.actuatedDofNum, -0.1, 0.12);
  PreComputation preComputation;

  const vector_t expectedFlow = expectedDynamics.computeFlowMap(
      0.2, state, input, preComputation);
  EXPECT_TRUE(adaptiveDynamics.computeFlowMap(
                  0.2, state, input, preComputation)
                  .isApprox(expectedFlow, 1.0e-13));
  const auto expectedApproximation = expectedDynamics.linearApproximation(
      0.2, state, input, preComputation);
  const auto adaptiveApproximation = adaptiveDynamics.linearApproximation(
      0.2, state, input, preComputation);
  EXPECT_TRUE(adaptiveApproximation.f.isApprox(
      expectedApproximation.f, 1.0e-13));
  EXPECT_TRUE(adaptiveApproximation.dfdx.isApprox(
      expectedApproximation.dfdx, 1.0e-12));
  EXPECT_TRUE(adaptiveApproximation.dfdu.isApprox(
      expectedApproximation.dfdu, 1.0e-12));

  std::unique_ptr<SystemDynamicsBase> clone(adaptiveDynamics.clone());
  EXPECT_TRUE(clone->computeFlowMap(0.2, state, input, preComputation)
                  .isApprox(expectedFlow, 1.0e-13));

  // A new immutable version invalidates each object's cache. Zero estimate
  // must recover the complete nominal scalar flow, including coordinate rows.
  modelData->publish(adaptive::AdaptiveModelSnapshot::createValid(
      vector_t::Zero(10), 0.3, 2));
  NominalCoordinatePlantDynamics nominalDynamics(
      interface->getPinocchioInterface(), nominalInfo, nominalInfo);
  const vector_t nominalFlow = nominalDynamics.computeFlowMap(
      0.3, state, input, preComputation);
  EXPECT_TRUE(adaptiveDynamics.computeFlowMap(
                  0.3, state, input, preComputation)
                  .isApprox(nominalFlow, 1.0e-13));
  EXPECT_TRUE(clone->computeFlowMap(0.3, state, input, preComputation)
                  .isApprox(nominalFlow, 1.0e-13));

  vector_t invalid = vector_t::Zero(10);
  invalid(0) = -nominalInfo.robotMass;
  modelData->publish(adaptive::AdaptiveModelSnapshot::createValid(
      invalid, 0.4, 3));
  EXPECT_THROW(adaptiveDynamics.computeFlowMap(
                   0.4, state, input, preComputation),
               std::domain_error);
}

TEST(PayloadInertial10D,
     RuntimeDiagnosticsExposeVersionedCandidateAndAppliedFullFlow) {
  auto interface = makeInterface();
  const auto& nominalInfo = interface->getCentroidalModelInfo();
  const auto payload = makePhysicalPayload();
  const adaptive::vector10_t truth = PayloadInertial10D::toVector(payload);
  PayloadInertialRuntimeDiagnostics diagnostics(
      interface->getPinocchioInterface(), nominalInfo, 1.0);
  NominalCoordinatePlantDynamics nominalDynamics(
      interface->getPinocchioInterface(), nominalInfo, nominalInfo);
  NominalCoordinatePlantDynamics estimatedDynamics(
      interface->getPinocchioInterface(), nominalInfo,
      PayloadInertial10D::makePlantModelInfo(nominalInfo, payload));

  vector_t state = interface->getInitialState();
  state.head<6>() << 0.19, -0.08, 0.05, 0.014, -0.021, 0.017;
  state.segment<3>(9) << 0.04, -0.06, 0.08;
  vector_t input = vector_t::Zero(nominalInfo.inputDim);
  input.head<12>() = vector_t::LinSpaced(12, 80.0, 140.0);
  PreComputation preComputation;
  const auto snapshot = adaptive::AdaptiveModelSnapshot::createValid(
      truth, 0.7, 11);

  const auto off = diagnostics.evaluate(0.7, state, input, snapshot, false);
  const vector_t expectedNominal = nominalDynamics.computeFlowMap(
      0.7, state, input, preComputation);
  const vector_t expectedEstimated = estimatedDynamics.computeFlowMap(
      0.7, state, input, preComputation);
  EXPECT_TRUE(off.estimatedModelValid);
  EXPECT_FALSE(off.adaptiveDynamicsApplied);
  EXPECT_TRUE(off.nominalFlow.isApprox(expectedNominal, 1.0e-13));
  EXPECT_TRUE(off.candidateFullFlowCorrection.isApprox(
      expectedEstimated - expectedNominal, 1.0e-13));
  EXPECT_TRUE(off.appliedFullFlowCorrection.isZero(0.0));

  const auto on = diagnostics.evaluate(0.7, state, input, snapshot, true);
  EXPECT_TRUE(on.estimatedModelValid);
  EXPECT_TRUE(on.adaptiveDynamicsApplied);
  EXPECT_TRUE(on.appliedFullFlowCorrection.isApprox(
      on.candidateFullFlowCorrection, 0.0));
  EXPECT_DOUBLE_EQ(on.estimatedTotalMass,
                   nominalInfo.robotMass + payload.mass);

  vector_t invalid = vector_t::Zero(10);
  invalid(0) = -nominalInfo.robotMass;
  const auto invalidSample = diagnostics.evaluate(
      0.8, state, input,
      adaptive::AdaptiveModelSnapshot::createValid(invalid, 0.8, 12),
      true);
  EXPECT_FALSE(invalidSample.estimatedModelValid);
  EXPECT_FALSE(invalidSample.adaptiveDynamicsApplied);
  EXPECT_TRUE(invalidSample.candidateFullFlowCorrection.isZero(0.0));
  EXPECT_TRUE(std::isnan(invalidSample.estimatedTotalMass));
}

}  // namespace
}  // namespace legged_robot_adaptive
}  // namespace ocs2
