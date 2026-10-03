// Exercise the real wrapper and OCS2 model helpers, with an independent
// body-mass CoM oracle. No generated robot files or ROS runtime are required.
#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <gtest/gtest.h>
#include <cmath>
#include <memory>
#include <thread>
#include <atomic>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include "legged_interface/adaptive/MyRbfAugmentedLeggedDynamics.h"
#include "legged_interface/adaptive/MyRbfV11Clf.h"

namespace legged {
namespace adaptive {
namespace {
using ocs2::vector_t;
using ocs2::matrix_t;
using ocs2::CentroidalModelType;

pinocchio::Model articulatedModel() {
  pinocchio::Model model;
  pinocchio::JointModelComposite base(2);
  base.addJoint(pinocchio::JointModelTranslation());
  base.addJoint(pinocchio::JointModelSphericalZYX());
  const auto b = model.addJoint(0, base, pinocchio::SE3::Identity(), "base");
  model.appendBodyToJoint(b, pinocchio::Inertia(8., Eigen::Vector3d(.12, -.07, .05), Eigen::Matrix3d::Identity()));
  const auto hip = model.addJoint(b, pinocchio::JointModelRY(),
      pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(.3, .15, -.1)), "hip");
  model.appendBodyToJoint(hip, pinocchio::Inertia(3., Eigen::Vector3d(.04, .03, -.2), .1 * Eigen::Matrix3d::Identity()));
  const auto knee = model.addJoint(hip, pinocchio::JointModelRX(),
      pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(.02, -.04, -.4)), "knee");
  model.appendBodyToJoint(knee, pinocchio::Inertia(1., Eigen::Vector3d(-.06, .05, -.17), .03 * Eigen::Matrix3d::Identity()));
  return model;
}

// Deliberately not data.com, Ag, or updateCentroidalDynamics: sum the world
// positions of every body's mass. Translation + ZYX coordinates have nq=nv.
Eigen::Vector3d physicalCom(const pinocchio::Model& model, const vector_t& q) {
  pinocchio::Data data(model);
  pinocchio::forwardKinematics(model, data, q);
  Eigen::Vector3d weighted = Eigen::Vector3d::Zero();
  double mass = 0.;
  for (pinocchio::JointIndex j = 1; j < model.njoints; ++j) {
    const auto& inertia = model.inertias[j];
    weighted += inertia.mass() * data.oMi[j].act(inertia.lever());
    mass += inertia.mass();
  }
  return weighted / mass;
}

// Nontrivial nominal dynamics make dropped input/state derivatives, altered
// time forwarding, and clone loss observable independently of the addition.
class AffineNominal final : public ocs2::SystemDynamicsBase {
 public:
  AffineNominal(int nx, int nu) : a(nx, nx), b(nx, nu), bias(nx) {
    for (int i = 0; i < nx; ++i) {
      bias(i) = .13 * (i + 1);
      for (int j = 0; j < nx; ++j) a(i, j) = .007 * (1 + i - 2 * j);
      for (int j = 0; j < nu; ++j) b(i, j) = .011 * (2 + 3 * i + j);
    }
  }
  AffineNominal* clone() const override { return new AffineNominal(*this); }
  vector_t computeFlowMap(double t, const vector_t& x, const vector_t& u, const ocs2::PreComputation&) override {
    return a * x + b * u + t * bias;
  }
  ocs2::VectorFunctionLinearApproximation linearApproximation(double t, const vector_t& x,
      const vector_t& u, const ocs2::PreComputation& pre) override {
    ocs2::VectorFunctionLinearApproximation result;
    result.f = computeFlowMap(t, x, u, pre); result.dfdx = a; result.dfdu = b;
    return result;
  }
  matrix_t a, b;
  vector_t bias;
};

class MyRbfV2AugmentedDynamicsTest : public ::testing::TestWithParam<CentroidalModelType> {
 protected:
  pinocchio::Model model = articulatedModel();
  ocs2::PinocchioInterface pinocchio{model};
  ocs2::CentroidalModelInfo info{};
  std::shared_ptr<MyRbfV2SnapshotStore> store = std::make_shared<MyRbfV2SnapshotStore>();
  AffineNominal nominal{14, 2};
  std::unique_ptr<MyRbfAugmentedLeggedDynamics> wrapper;
  ocs2::PreComputation pre;

  void SetUp() override {
    ASSERT_EQ(model.nq, 8); ASSERT_EQ(model.nv, 8);
    info.centroidalModelType = GetParam();
    info.generalizedCoordinatesNum = 8; info.actuatedDofNum = 2;
    info.stateDim = 14; info.inputDim = 2; info.robotMass = 12.;
    info.qPinocchioNominal = vector_t::Zero(8);
    info.comToBasePositionNominal = -physicalCom(model, info.qPinocchioNominal);
    info.centroidalInertiaNominal = Eigen::Matrix3d::Identity();
    wrapper.reset(new MyRbfAugmentedLeggedDynamics(nominal, pinocchio, info, store));
  }
  vector_t state(int sample = 0) const {
    vector_t x(14);
    for (int j = 0; j < 14; ++j) x(j) = .4 * std::sin(.7 * j + .3 * sample);
    x.segment<3>(6) << .8, -.4, .65;
    x.segment<3>(9) << .31 + .09 * sample, -.43 + .04 * sample, .27 - .06 * sample;
    return x;
  }
  vector_t input() const { vector_t u(2); u << .3, -.7; return u; }
  MyRbfV2Snapshot snapshot(std::uint64_t id = 11) const {
    MyRbfV2Snapshot s;
    s.valid = true; s.sequenceId = id; s.timestamp = 1.25;
    s.externalWrenchEstimate = {{13., -7., 19., 2.5, -3., 4.}};
    s.modelWrenchEstimate = s.externalWrenchEstimate;
    // Unrelated fields must never be added or used with the opposite sign.
    s.leftHandDisturbanceEstimate.fill(81.); s.requiredControlCorrection.fill(-93.);
    s.robustWrench.fill(127.); s.referenceWrench.fill(-221.);
    return s;
  }
  void freeze(const MyRbfV2Snapshot& s) {
    ASSERT_TRUE(store->tryPublish(s)); store->freezeForSolver();
  }
  Eigen::Vector3d expectedCom(const vector_t& x) const {
    vector_t q = x.tail(8);
    if (GetParam() == CentroidalModelType::SingleRigidBodyDynamics) q.tail(2).setZero();
    return physicalCom(model, q);
  }
  vector_t expectedAddition(const vector_t& x, const MyRbfV2Snapshot& s) const {
    vector_t result = vector_t::Zero(14);
    if (!s.valid) return result;
    const double sign = s.modelUsesSubtractedCorrection ? -1. : 1.;
    const Eigen::Vector3d force(s.modelWrenchEstimate[0], s.modelWrenchEstimate[1], s.modelWrenchEstimate[2]);
    const Eigen::Vector3d torque(s.modelWrenchEstimate[3], s.modelWrenchEstimate[4], s.modelWrenchEstimate[5]);
    const Eigen::Vector3d base = x.segment<3>(6);
    result.head<3>() = sign * force / 12.;
    result.segment<3>(3) = sign * (torque + (base - expectedCom(x)).cross(force)) / 12.;
    return result;
  }
};

TEST_P(MyRbfV2AugmentedDynamicsTest, AbsentAndInvalidSnapshotExactlyPreserveNominalFlowAndJacobians) {
  for (int kind = 0; kind < 2; ++kind) {
    if (kind) { auto invalid = snapshot(); invalid.valid = false; freeze(invalid); }
    for (int sample = 0; sample < 5; ++sample) {
      const auto x = state(sample), u = input();
      const auto expected = nominal.linearApproximation(.73, x, u, pre);
      const auto actual = wrapper->linearApproximation(.73, x, u, pre);
      EXPECT_TRUE((wrapper->computeFlowMap(.73, x, u, pre).array() == expected.f.array()).all());
      EXPECT_TRUE((actual.f.array() == expected.f.array()).all());
      EXPECT_TRUE((actual.dfdx.array() == expected.dfdx.array()).all());
      EXPECT_TRUE((actual.dfdu.array() == expected.dfdu.array()).all());
    }
  }
}

TEST(V11WordDefinition, ExactAllNodeUpdateAndCompositeCancellation) {
  v11::Settings settings;v11::Core core(settings);
  v11::Vector6 ee,er;ee<<.01,-.02,.03,-.04,.05,-.06;er<<-.1,.2,.1,.3,-.2,.1;
  Eigen::Matrix3d inertia;inertia<<.4,.02,0,.02,.6,.01,0,.01,.8;
  const auto R=v11::rotation(Eigen::Vector3d(.4,-.2,.1));
  const auto G=v11::mapping(15.,inertia,R);
  const auto out=core.update(ee,er,G,.001,true);
  for(int i=0;i<6;++i){
    EXPECT_NEAR(out.xi(i),std::max(-1.,std::min(1.,ee(i)/settings.scale(i))),1e-14);
    for(int j=0;j<26;++j)
      EXPECT_NEAR(core.weights()(i,j),-.001*settings.gamma(i)*out.features(i,j)*G.col(i).dot(out.ec),1e-14);
    const Eigen::Matrix<double,1,26> tilde=Eigen::Matrix<double,1,26>::Constant(.13*(i+1));
    EXPECT_NEAR(tilde.dot(out.features.row(i))*G.col(i).dot(out.ec)+
                tilde.dot(out.derivative.row(i))/settings.gamma(i),0.,1e-13);
  }
  EXPECT_TRUE((G.topLeftCorner<3,3>()-R/15.).norm()<1e-13);
  EXPECT_TRUE((G.bottomRightCorner<3,3>()*inertia-Eigen::Matrix3d::Identity()).norm()<1e-13);
  const auto before=core.weights();core.update(ee,er,G,.001,false);
  EXPECT_TRUE(core.weights().isApprox(before));core.reset();EXPECT_EQ(core.weights().norm(),0.);
}

TEST(V11WordDefinition, OneStepPredictionDoesNotExtrapolateAccelerationAcrossMpcHorizon) {
  v11::Vector6 value,rate;
  const v11::Vector6 origin=v11::Vector6::Zero(),acceleration=v11::Vector6::Constant(200.);
  v11::oneStepPrediction(origin,acceleration,1.,1.001,1.0005,value,rate);
  EXPECT_NEAR(value(0),.1,1e-10);EXPECT_NEAR(rate(0),200.,1e-10);
  v11::oneStepPrediction(origin,acceleration,1.,1.001,2.,value,rate);
  EXPECT_NEAR(value(0),.2,1e-10);EXPECT_EQ(rate.norm(),0.);
}

TEST_P(MyRbfV2AugmentedDynamicsTest, V11UsesBodyWrenchAboutComAndLeftHandSign) {
  auto s=snapshot();s.v11Enabled=true;s.v11Disturbance<<3,-4,5,.3,-.2,.1;freeze(s);
  const auto x=state();const auto R=v11::rotation(x.segment<3>(9));
  const auto difference=(wrapper->computeFlowMap(0.,x,input(),pre)-nominal.computeFlowMap(0.,x,input(),pre)).eval();
  EXPECT_TRUE(difference.head<3>().isApprox(-R*s.v11Disturbance.head<3>()/info.robotMass,1e-12));
  EXPECT_TRUE(difference.segment<3>(3).isApprox(-R*s.v11Disturbance.tail<3>()/info.robotMass,1e-12));
  const auto a=wrapper->linearApproximation(0.,x,input(),pre);
  for(int j=9;j<12;++j){auto xp=x,xm=x;xp(j)+=1e-5;xm(j)-=1e-5;
    const auto numerical=((wrapper->computeFlowMap(0.,xp,input(),pre)-wrapper->computeFlowMap(0.,xm,input(),pre))/2e-5).eval();
    EXPECT_TRUE(a.dfdx.col(j).isApprox(numerical,1e-7));
  }
}

TEST_P(MyRbfV2AugmentedDynamicsTest, V11CompositeConstraintMatchesFiniteDifferences) {
  info.centroidalInertiaNominal<<.4,.02,0,.02,.6,.01,0,.01,.8;
  auto s=snapshot();s.v11Enabled=true;s.v11PredictionValid=true;
  s.v11PredictionOrigin<<.1,-.2,.03,.04,-.05,.06;
  s.v11PredictionRate<<.02,-.01,.03,-.04,.01,.02;s.v11PredictionTime=.7;
  s.v11Disturbance<<2,-3,4,.1,-.2,.3;
  s.v11Target=ocs2::TargetTrajectories({0.},{state(1)},{input()});freeze(s);
  MyRbfV11Clf constraint(*wrapper,info,store);
  const auto x=state(),u=input();const double t=.73,eps=1e-6;
  const auto a=constraint.getLinearApproximation(t,x,u,pre);
  EXPECT_TRUE(a.f.isApprox(constraint.getValue(t,x,u,pre),1e-12));
  for(int j=0;j<x.size();++j){auto xp=x,xm=x;xp(j)+=eps;xm(j)-=eps;
    const double numerical=(constraint.getValue(t,xp,u,pre)(0)-constraint.getValue(t,xm,u,pre)(0))/(2*eps);
    EXPECT_NEAR(a.dfdx(0,j),numerical,2e-5)<<"state "<<j;
  }
  for(int j=0;j<u.size();++j){auto up=u,um=u;up(j)+=eps;um(j)-=eps;
    const double numerical=(constraint.getValue(t,x,up,pre)(0)-constraint.getValue(t,x,um,pre)(0))/(2*eps);
    EXPECT_NEAR(a.dfdu(0,j),numerical,2e-5)<<"input "<<j;
  }
}

TEST_P(MyRbfV2AugmentedDynamicsTest, WorldBaseWrenchHasCorrectSignMassMomentShiftAndOnlyFirstSixRates) {
  auto s = snapshot();
  for (int kind = 0; kind < 3; ++kind) {
    s = snapshot();
    if (kind == 1) for (int i = 0; i < 3; ++i) s.externalWrenchEstimate[i] = 0.;
    if (kind == 2) for (int i = 3; i < 6; ++i) s.externalWrenchEstimate[i] = 0.;
    freeze(s);
    for (int sample = 0; sample < 8; ++sample) {
      const auto x = state(sample), u = input();
      const vector_t delta = wrapper->computeFlowMap(.73, x, u, pre) - nominal.computeFlowMap(.73, x, u, pre);
      EXPECT_LT((delta - expectedAddition(x, s)).norm(), 2.e-13);
      EXPECT_TRUE(delta.tail(8).isZero(0.));
      const auto linear = wrapper->linearApproximation(.73, x, u, pre);
      EXPECT_LT((linear.f - wrapper->computeFlowMap(.73, x, u, pre)).norm(), 1.e-14);
      EXPECT_TRUE((linear.dfdu.array() == nominal.b.array()).all());
    }
  }
}

TEST_P(MyRbfV2AugmentedDynamicsTest, V3CorrectionIsSubtractedWhileLegacyV2EstimateRemainsAdded) {
  const auto x = state(), u = input();
  auto legacy = snapshot();
  freeze(legacy);
  const vector_t legacyDelta =
      wrapper->computeFlowMap(.73, x, u, pre) - nominal.computeFlowMap(.73, x, u, pre);
  EXPECT_LT((legacyDelta - expectedAddition(x, legacy)).norm(), 2.e-13);

  auto v3 = legacy;
  v3.modelUsesSubtractedCorrection = true;
  for (int i = 0; i < 6; ++i)
    v3.externalWrenchEstimate[i] = -v3.modelWrenchEstimate[i];
  freeze(v3);
  const vector_t v3Delta =
      wrapper->computeFlowMap(.73, x, u, pre) - nominal.computeFlowMap(.73, x, u, pre);
  EXPECT_LT((v3Delta - expectedAddition(x, v3)).norm(), 2.e-13);
  EXPECT_LT((v3Delta + legacyDelta).norm(), 2.e-13);
  for (int i = 0; i < 6; ++i)
    EXPECT_DOUBLE_EQ(v3.externalWrenchEstimate[i], -legacy.externalWrenchEstimate[i]);
}

TEST_P(MyRbfV2AugmentedDynamicsTest, ModelComAndMomentumMapMatchIndependentGeometry) {
  const double eps = 1.e-6;
  for (int sample = 0; sample < 8; ++sample) {
    const vector_t x = state(sample), q = x.tail(8);
    ocs2::updateCentroidalDynamics(pinocchio, info, q);
    EXPECT_LT((pinocchio.getData().com[0] - expectedCom(x)).norm(), 1.e-13);
    const matrix_t jac = pinocchio.getData().Ag.topRows(3) / 12.;
    for (int j = 0; j < 8; ++j) {
      vector_t plus = x, minus = x; plus(6 + j) += eps; minus(6 + j) -= eps;
      const Eigen::Vector3d fd = (expectedCom(plus) - expectedCom(minus)) / (2. * eps);
      EXPECT_LT((jac.col(j) - fd).norm(), 3.e-9) << "sample=" << sample << " q column=" << j;
    }
  }
}

TEST_P(MyRbfV2AugmentedDynamicsTest, GeometryAndCompleteFlowJacobiansMatchCentralDifferences) {
  const auto s = snapshot(); freeze(s);
  const double eps = 1.e-6;
  for (int sample = 0; sample < 8; ++sample) {
    const auto x = state(sample), u = input();
    const auto linear = wrapper->linearApproximation(.73, x, u, pre);
    matrix_t geometry(14, 14);
    for (int j = 0; j < 14; ++j) {
      vector_t plus = x, minus = x; plus(j) += eps; minus(j) -= eps;
      const vector_t fd = (wrapper->computeFlowMap(.73, plus, u, pre) - wrapper->computeFlowMap(.73, minus, u, pre)) / (2. * eps);
      geometry.col(j) = (expectedAddition(plus, s) - expectedAddition(minus, s)) / (2. * eps);
      EXPECT_LT((linear.dfdx.col(j) - fd).norm(), 4.e-9) << "sample=" << sample << " state column=" << j;
    }
    EXPECT_LT((linear.dfdx - nominal.a - geometry).norm(), 8.e-9);
    EXPECT_GT(geometry.block(3, 9, 3, 3).norm(), .01);
    if (GetParam() == CentroidalModelType::SingleRigidBodyDynamics) EXPECT_TRUE(geometry.rightCols(2).isZero(0.));
    else EXPECT_GT(geometry.block(3, 12, 3, 2).norm(), .01);
    for (int j = 0; j < 2; ++j) {
      vector_t plus = u, minus = u; plus(j) += eps; minus(j) -= eps;
      const vector_t fd = (wrapper->computeFlowMap(.73, x, plus, pre) - wrapper->computeFlowMap(.73, x, minus, pre)) / (2. * eps);
      EXPECT_LT((linear.dfdu.col(j) - fd).norm(), 4.e-9);
    }
  }
}

TEST_P(MyRbfV2AugmentedDynamicsTest, CloneRetainsEstimateAndPendingUpdatesCannotLeakIntoSolve) {
  const auto first = snapshot(); freeze(first);
  const auto frozen = store->frozenForSolver;
  std::unique_ptr<MyRbfAugmentedLeggedDynamics> clone(wrapper->clone());
  const auto x = state(), u = input();
  const auto before = wrapper->linearApproximation(.73, x, u, pre);
  auto pending = snapshot(22);
  pending.externalWrenchEstimate.fill(-39.);
  pending.modelWrenchEstimate = pending.externalWrenchEstimate;
  ASSERT_TRUE(store->tryPublish(pending));
  pending.externalWrenchEstimate.fill(600.);  // Publication must own a copy.
  for (auto* dynamics : {wrapper.get(), clone.get()}) {
    const auto actual = dynamics->linearApproximation(.73, x, u, pre);
    EXPECT_TRUE((actual.f.array() == before.f.array()).all());
    EXPECT_TRUE((actual.dfdx.array() == before.dfdx.array()).all());
    EXPECT_TRUE((actual.dfdu.array() == before.dfdu.array()).all());
  }
  // Separate worker-owned instances may evaluate while the control thread
  // publishes. freezeForSolver is intentionally never called during workers.
  std::atomic<int> published{0};
  std::thread publisher([&] {
    for (int i = 0; i < 200; ++i) if (store->tryPublish(pending)) ++published;
  });
  for (int i = 0; i < 100; ++i) {
    EXPECT_TRUE((clone->computeFlowMap(.73, x, u, pre).array() == before.f.array()).all());
  }
  publisher.join();
  EXPECT_GT(published.load(), 0);
  EXPECT_EQ(store->frozenForSolver, frozen);
  EXPECT_EQ(frozen->sequenceId, 11u);
  freeze(pending);  // Next solve boundary: both instances must see new data.
  const vector_t expected = nominal.computeFlowMap(.73, x, u, pre) + expectedAddition(x, pending);
  EXPECT_LT((wrapper->computeFlowMap(.73, x, u, pre) - expected).norm(), 1.e-12);
  wrapper.reset();
  EXPECT_LT((clone->computeFlowMap(.73, x, u, pre) - expected).norm(), 1.e-12);
  EXPECT_EQ(frozen->externalWrenchEstimate, first.externalWrenchEstimate);
}

INSTANTIATE_TEST_SUITE_P(BothCentroidalModels, MyRbfV2AugmentedDynamicsTest,
    ::testing::Values(CentroidalModelType::SingleRigidBodyDynamics, CentroidalModelType::FullCentroidalDynamics),
    [](const ::testing::TestParamInfo<CentroidalModelType>& p) {
      return p.param == CentroidalModelType::SingleRigidBodyDynamics ? "SRBD" : "FullCentroidal";
    });

}  // namespace
}  // namespace adaptive
}  // namespace legged
