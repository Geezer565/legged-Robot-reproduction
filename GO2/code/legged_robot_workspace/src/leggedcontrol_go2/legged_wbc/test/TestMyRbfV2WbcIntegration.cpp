#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/model.hpp>
#include "legged_wbc/WeightedWbc.h"
#include "legged_wbc/HierarchicalWbc.h"
#include <gtest/gtest.h>
#include <cmath>

namespace legged {
namespace {

pinocchio::Model integrationModel() {
  pinocchio::Model model;
  pinocchio::JointModelComposite base(2);
  base.addJoint(pinocchio::JointModelTranslation());
  base.addJoint(pinocchio::JointModelSphericalZYX());
  auto baseId = model.addJoint(0, base, pinocchio::SE3::Identity(), "base");
  model.appendBodyToJoint(baseId, pinocchio::Inertia(8., vector3_t(0.02, -0.01, 0.03), matrix3_t::Identity()));
  for (int leg = 0; leg < 4; ++leg) {
    auto parent = baseId;
    for (int joint = 0; joint < 3; ++joint) {
      vector3_t r(0., 0., -0.2);
      if (joint == 0) r = vector3_t(leg < 2 ? 0.3 : -0.3, leg % 2 ? -0.15 : 0.15, 0.);
      pinocchio::JointModel jointModel = pinocchio::JointModelRY();
      if (joint == 0) jointModel = pinocchio::JointModelRX();
      parent = model.addJoint(parent, jointModel, pinocchio::SE3(matrix3_t::Identity(), r),
                              "leg" + std::to_string(leg) + "joint" + std::to_string(joint));
      model.appendBodyToJoint(parent, pinocchio::Inertia(0.3, vector3_t(0., 0., -0.1), 0.01 * matrix3_t::Identity()));
    }
    model.addFrame(pinocchio::Frame("foot" + std::to_string(leg), parent, 0,
                   pinocchio::SE3(matrix3_t::Identity(), vector3_t(0., 0., -0.2)), pinocchio::OP_FRAME));
  }
  return model;
}

CentroidalModelInfo integrationInfo(const pinocchio::Model& model) {
  CentroidalModelInfo info;
  info.centroidalModelType = CentroidalModelType::FullCentroidalDynamics;
  info.numThreeDofContacts = 4; info.numSixDofContacts = 0;
  info.generalizedCoordinatesNum = 18; info.actuatedDofNum = 12;
  info.stateDim = 24; info.inputDim = 24; info.robotMass = 11.6;
  info.qPinocchioNominal = vector_t::Zero(18);
  info.centroidalInertiaNominal.setIdentity(); info.comToBasePositionNominal.setZero();
  for (int i = 0; i < 4; ++i) info.endEffectorFrameIndices.push_back(model.getFrameId("foot" + std::to_string(i)));
  return info;
}

// Only the soft objective is replaced. update(), all constraint assembly,
// reference preparation, solver failure handling and diagnostics are production code.
class IntegrationWbc : public WeightedWbc {
 public:
  IntegrationWbc(const PinocchioInterface& pin, const CentroidalModelInfo& info,
                 const PinocchioEndEffectorKinematics& kinematics) : WeightedWbc(pin, info, kinematics) {
    torqueLimits_ = vector3_t::Constant(500.);
    frictionCoeff_ = 0.6; swingKp_ = 1.; swingKd_ = 1.;
  }
 protected:
  Task formulateWeightedTasks(const vector_t&, const vector_t& input, scalar_t) override {
    vector_t target = vector_t::Zero(numDecisionVars_);
    target.segment(18, 12) = input.head(12);
    return {matrix_t::Identity(numDecisionVars_, numDecisionVars_), target, matrix_t(), vector_t()};
  }
};

struct MyRbfV2WbcIntegration : public ::testing::Test {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  PinocchioInterface pin{integrationModel()};
  CentroidalModelInfo info{integrationInfo(pin.getModel())};
  CentroidalModelPinocchioMapping mapping{info};
  PinocchioEndEffectorKinematics kinematics{pin, mapping, {"foot0", "foot1", "foot2", "foot3"}};
  IntegrationWbc wbc{pin, info, kinematics};
  vector_t state = vector_t::Zero(24), input = vector_t::Zero(24), measured = vector_t::Zero(36);
  MyRbfV2WbcInput frozen;
  void SetUp() override {
    frozen.enabled = true; frozen.snapshotSequenceId = 37;
    frozen.maxContactForceRate.setConstant(100000.);
    for (int i = 0; i < 4; ++i) {
      input[3 * i + 2] = info.robotMass * 9.81 / 4.;
      // Bent knees keep the three-axis leg Jacobian nonsingular in the solve fixture.
      state[12 + 3 * i + 1] = measured[6 + 3 * i + 1] = 0.5;
      state[12 + 3 * i + 2] = measured[6 + 3 * i + 2] = -1.;
    }
    frozen.previousContactForces = input.head(12);
    wbc.configureMyRbfV2(frozen);
  }
};

TEST_F(MyRbfV2WbcIntegration, FinalProductionSolveAndPreparationUsesCurrentState) {
  state[6] = -0.01;
  ASSERT_TRUE(wbc.prepareMyRbfV2Tracking(state, input, measured, 15, 0.01));
  const auto prepared = wbc.getMyRbfV2Diagnostics();
  EXPECT_GT(prepared.chi[0], 0.);
  EXPECT_FALSE(prepared.referenceWrenchIsFinal);
  EXPECT_FALSE(prepared.desiredAccelerationInitialized);
  ASSERT_TRUE(wbc.prepareMyRbfV2Tracking(state, input, measured, 15, 0.01));
  EXPECT_FALSE(wbc.getMyRbfV2Diagnostics().desiredAccelerationInitialized);
  frozen.externalWrenchEstimate[0] = 2.;
  frozen.snapshotSequenceId = 38;
  wbc.configureMyRbfV2(frozen);
  const vector_t solution = wbc.update(state, input, measured, 15, 0.01);
  const auto& diag = wbc.getMyRbfV2Diagnostics();
  ASSERT_EQ(solution.size(), 42) << static_cast<int>(diag.status) << ":" << diag.solverReturnCode;
  EXPECT_TRUE(diag.feasible); EXPECT_TRUE(diag.referenceWrenchIsFinal);
  EXPECT_EQ(diag.snapshotSequenceId, 38u); EXPECT_EQ(diag.contactWrenchRank, 6);
  EXPECT_LT((diag.chi - prepared.chi).norm(), 1e-12);
  EXPECT_LE(diag.clfViolation, frozen.feasibilityTolerance);
  EXPECT_LE(diag.maxEqualityViolation, frozen.feasibilityTolerance);
  EXPECT_LE(diag.maxInequalityViolation, frozen.feasibilityTolerance);
  EXPECT_GE(diag.torqueLimitMargin, -frozen.feasibilityTolerance);
  EXPECT_GE(diag.frictionMargin, -frozen.feasibilityTolerance);
  EXPECT_GE(diag.normalForceMargin, -frozen.feasibilityTolerance);
  EXPECT_GE(diag.contactForceRateMargin, -frozen.feasibilityTolerance);
  EXPECT_TRUE(diag.finalContactForces.isApprox(solution.segment(18, 12), 0.));
  ASSERT_TRUE(wbc.prepareMyRbfV2Tracking(state, input, measured, 15, 0.01));
  EXPECT_TRUE(wbc.getMyRbfV2Diagnostics().desiredAccelerationInitialized);
}

TEST_F(MyRbfV2WbcIntegration, InfeasibleHardClfReturnsNoPrimalSolution) {
  state[6] = -0.1;
  frozen.robustBound.setConstant(1e9);
  wbc.configureMyRbfV2(frozen);
  EXPECT_EQ(wbc.update(state, input, measured, 15, 0.01).size(), 0);
  EXPECT_FALSE(wbc.getMyRbfV2Diagnostics().feasible);
  EXPECT_NE(wbc.getMyRbfV2Diagnostics().status, MyRbfV2WbcStatus::Solved);
}

TEST_F(MyRbfV2WbcIntegration, WordV3ModeOmitsInfeasibleClfButKeepsPhysicalChecks) {
  state[6] = -0.1;
  frozen.robustBound.setConstant(1e9);
  frozen.enforceClfConstraint = false;
  wbc.configureMyRbfV2(frozen);
  const vector_t solution = wbc.update(state, input, measured, 15, 0.01);
  const auto& diag = wbc.getMyRbfV2Diagnostics();
  ASSERT_EQ(solution.size(), 42) << static_cast<int>(diag.status) << ":" << diag.solverReturnCode;
  EXPECT_TRUE(diag.feasible);
  EXPECT_EQ(diag.status, MyRbfV2WbcStatus::Solved);
  EXPECT_GT(diag.clfViolation, frozen.feasibilityTolerance);
  EXPECT_LE(diag.maxEqualityViolation, frozen.feasibilityTolerance);
  EXPECT_LE(diag.maxInequalityViolation, frozen.feasibilityTolerance);
  EXPECT_GE(diag.frictionMargin, -frozen.feasibilityTolerance);
  EXPECT_GE(diag.normalForceMargin, -frozen.feasibilityTolerance);
}

TEST_F(MyRbfV2WbcIntegration, WordV3ModeDoesNotInjectEstimateDirectlyIntoWholeBodyDynamics) {
  IntegrationWbc v3WithEstimate(pin, info, kinematics);
  IntegrationWbc v3WithZeroEstimate(pin, info, kinematics);
  frozen.enforceClfConstraint = false;
  frozen.injectExternalWrenchInDynamics = false;
  frozen.externalWrenchEstimate << 4., -3., 2., .5, -.4, .3;
  v3WithEstimate.configureMyRbfV2(frozen);
  auto zero = frozen;
  zero.externalWrenchEstimate.setZero();
  v3WithZeroEstimate.configureMyRbfV2(zero);
  const vector_t actual = v3WithEstimate.update(state, input, measured, 15, .01);
  const vector_t expected = v3WithZeroEstimate.update(state, input, measured, 15, .01);
  ASSERT_EQ(actual.size(), 42);
  ASSERT_EQ(expected.size(), 42);
  EXPECT_TRUE(actual.isApprox(expected, 1e-10));
}

TEST_F(MyRbfV2WbcIntegration, WordV3ModePreservesNominalWholeBodyControl) {
  IntegrationWbc nominal(pin, info, kinematics);
  IntegrationWbc v3(pin, info, kinematics);
  frozen.enforceClfConstraint = false;
  frozen.injectExternalWrenchInDynamics = false;
  frozen.externalWrenchEstimate.setZero();
  v3.configureMyRbfV2(frozen);

  const vector_t expected = nominal.update(state, input, measured, 15, .01);
  const vector_t actual = v3.update(state, input, measured, 15, .01);
  ASSERT_EQ(expected.size(), 42);
  ASSERT_EQ(actual.size(), 42);
  EXPECT_TRUE(actual.isApprox(expected, 1e-10));
}

TEST_F(MyRbfV2WbcIntegration, RejectsMalformedStateAndClearsFeasibleStatus) {
  ASSERT_EQ(wbc.update(state, input, measured, 15, 0.01).size(), 42);
  EXPECT_EQ(wbc.update(state, input, measured, 16, 0.01).size(), 0);
  EXPECT_FALSE(wbc.getMyRbfV2Diagnostics().feasible);
  EXPECT_EQ(wbc.getMyRbfV2Diagnostics().status, MyRbfV2WbcStatus::InvalidInput);
  EXPECT_EQ(wbc.update(state, input, vector_t::Zero(6), 15, 0.01).size(), 0);
  measured[1] = std::acos(-1.) / 2.;
  EXPECT_EQ(wbc.update(state, input, measured, 15, 0.01).size(), 0);
  measured.setZero(); measured[30] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(wbc.update(state, input, measured, 15, 0.01).size(), 0);
  measured.setZero();
  EXPECT_EQ(wbc.update(state, input, measured, 15, 0.).size(), 0);
  frozen.robustBound[0] = -1.; wbc.configureMyRbfV2(frozen);
  EXPECT_EQ(wbc.update(state, input, measured, 15, 0.01).size(), 0);
  frozen.robustBound.setZero(); frozen.surfaceNormal.setZero(); wbc.configureMyRbfV2(frozen);
  EXPECT_EQ(wbc.update(state, input, measured, 15, 0.01).size(), 0);
  frozen.surfaceNormal[0] = std::numeric_limits<double>::quiet_NaN(); wbc.configureMyRbfV2(frozen);
  EXPECT_EQ(wbc.update(state, input, measured, 15, 0.01).size(), 0);
}

TEST_F(MyRbfV2WbcIntegration, ForceRateAtContactReleaseRemainsHard) {
  frozen.maxContactForceRate.setConstant(1.);
  wbc.configureMyRbfV2(frozen);
  EXPECT_EQ(wbc.update(state, input, measured, 0, 0.01).size(), 0);
  EXPECT_FALSE(wbc.getMyRbfV2Diagnostics().feasible);
}

TEST_F(MyRbfV2WbcIntegration, DisabledInputDoesNotChangeLegacySolution) {
  IntegrationWbc legacy(pin, info, kinematics);
  frozen.enabled = false;
  frozen.externalWrenchEstimate.setConstant(1e5);
  frozen.robustBound.setConstant(-1.);
  frozen.maxNormalForce = 0.;
  wbc.configureMyRbfV2(frozen);
  for (int cycle = 0; cycle < 3; ++cycle) {
    const vector_t actual = wbc.update(state, input, measured, 15, 0.01);
    const vector_t expected = legacy.update(state, input, measured, 15, 0.01);
    ASSERT_EQ(actual.size(), 42); ASSERT_EQ(expected.size(), 42);
    EXPECT_TRUE((actual.array() == expected.array()).all());
    EXPECT_EQ(wbc.getMyRbfV2Diagnostics().status, MyRbfV2WbcStatus::Disabled);
  }
}

TEST_F(MyRbfV2WbcIntegration, UnsupportedHierarchyRejectsBeforeSolving) {
  HierarchicalWbc hierarchical(pin, info, kinematics);
  hierarchical.configureMyRbfV2(frozen);
  EXPECT_EQ(hierarchical.update(state, input, measured, 15, 0.01).size(), 0);
  EXPECT_EQ(hierarchical.getMyRbfV2Diagnostics().status, MyRbfV2WbcStatus::Unsupported);
  EXPECT_FALSE(hierarchical.getMyRbfV2Diagnostics().feasible);
}

TEST_F(MyRbfV2WbcIntegration, StaticFiveDegreeCommonPlane) {
  const double angle = 5. * std::acos(-1.) / 180.;
  frozen.surfaceNormal = vector3_t(-std::sin(angle), 0., std::cos(angle));
  state[10] = measured[1] = -angle;
  wbc.configureMyRbfV2(frozen);
  const vector_t solution = wbc.update(state, input, measured, 15, 0.01);
  const auto& diag = wbc.getMyRbfV2Diagnostics();
  ASSERT_EQ(solution.size(), 42) << static_cast<int>(diag.status) << ":" << diag.solverReturnCode;
  EXPECT_TRUE(diag.feasible);
  EXPECT_GE(diag.frictionMargin, -frozen.feasibilityTolerance);
  EXPECT_GE(diag.normalForceMargin, -frozen.feasibilityTolerance);
}

}  // namespace
}  // namespace legged
