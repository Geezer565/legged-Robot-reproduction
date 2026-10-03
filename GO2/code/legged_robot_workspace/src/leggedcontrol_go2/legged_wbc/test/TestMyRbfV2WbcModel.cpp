#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include "legged_wbc/MyRbfV2WbcMath.h"
#include "legged_wbc/MyRbfV2WbcDynamics.h"
#include <gtest/gtest.h>

namespace legged {
namespace {

pinocchio::Model articulatedEulerModel() {
  pinocchio::Model model;
  pinocchio::JointModelComposite base(2);
  base.addJoint(pinocchio::JointModelTranslation());
  base.addJoint(pinocchio::JointModelSphericalZYX());
  auto baseId = model.addJoint(0, base, pinocchio::SE3::Identity(), "base");
  model.appendBodyToJoint(baseId, pinocchio::Inertia(8., Eigen::Vector3d(0.02, -0.01, 0.03), Eigen::Matrix3d::Identity()));
  for (int leg = 0; leg < 4; ++leg) {
    auto parent = baseId;
    for (int joint = 0; joint < 3; ++joint) {
      Eigen::Vector3d r(0., 0., -0.2);
      if (joint == 0) r = Eigen::Vector3d(leg < 2 ? 0.3 : -0.3, leg % 2 ? -0.15 : 0.15, 0.);
      parent = model.addJoint(parent, pinocchio::JointModelRY(), pinocchio::SE3(Eigen::Matrix3d::Identity(), r),
                              "leg" + std::to_string(leg) + "joint" + std::to_string(joint));
      model.appendBodyToJoint(parent, pinocchio::Inertia(0.3, Eigen::Vector3d(0., 0., -0.1), 0.01 * Eigen::Matrix3d::Identity()));
    }
    model.addFrame(pinocchio::Frame("foot" + std::to_string(leg), parent, 0,
                   pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0., 0., -0.2)), pinocchio::OP_FRAME));
  }
  return model;
}

Eigen::MatrixXd massAt(const pinocchio::Model& model, const Eigen::VectorXd& q) {
  pinocchio::Data data(model);
  Eigen::MatrixXd mass = pinocchio::crba(model, data, q);
  mass.triangularView<Eigen::StrictlyLower>() = mass.transpose().triangularView<Eigen::StrictlyLower>();
  return mass;
}

TEST(MyRbfV2WbcModel, ActualEulerCoriolisAndMovingLegReference) {
  const auto model = articulatedEulerModel();
  ASSERT_EQ(model.nq, 18); ASSERT_EQ(model.nv, 18);
  std::srand(387);
  for (int sample = 0; sample < 30; ++sample) {
    const Eigen::VectorXd q = 0.6 * Eigen::VectorXd::Random(18);
    const Eigen::VectorXd v = Eigen::VectorXd::Random(18);
    pinocchio::Data data(model);
    const Eigen::MatrixXd mass = massAt(model, q);
    const Eigen::VectorXd gravity = pinocchio::computeGeneralizedGravity(model, data, q);
    // Canonical Euler-coordinate Christoffel matrix, independently exercised by
    // every basis direction. Its production use needs only one product with vr.
    Eigen::MatrixXd c(18, 18);
    for (int col = 0; col < 18; ++col) {
      const Eigen::VectorXd basis = Eigen::VectorXd::Unit(18, col);
      c.col(col) = myrbf_wbc::eulerReferenceBias(model, data, q, v, basis) - gravity;
    }
    const Eigen::VectorXd nle = pinocchio::nonLinearEffects(model, data, q, v);
    EXPECT_LT((c * v - (nle - gravity)).norm(), 1e-9);
    const double h = 1e-6;
    const Eigen::MatrixXd mdot = (massAt(model, q + h * v) - massAt(model, q - h * v)) / (2. * h);
    EXPECT_LT((mdot - c - c.transpose()).norm(), 1e-7);

    const WbcVector6 qd = 0.6 * WbcVector6::Random(), vd = WbcVector6::Random(), ad = WbcVector6::Random();
    const WbcVector6 lambda = WbcVector6::Constant(1.2);
    const Eigen::VectorXd aj = Eigen::VectorXd::Random(12);
    const auto reference = myrbf_wbc::reference(mass, c, gravity, q, v, qd, vd, ad, lambda);
    Eigen::VectorXd vr = v, ar(18);
    vr.head<6>() = vd - lambda.cwiseProduct(myrbf_wbc::localError(q.head<6>(), qd));
    ar.head<6>() = ad - lambda.cwiseProduct(v.head<6>() - vd);
    ar.tail(12) = aj;
    const Eigen::VectorXd bias = myrbf_wbc::eulerReferenceBias(model, data, q, v, vr);
    EXPECT_LT((bias - c * vr - gravity).norm(), 1e-9);
    const auto productionRef = myrbf_wbc::referenceFromBias(mass, bias, q, v, qd, vd, ad, lambda);
    EXPECT_LT((productionRef.constantWrench - reference.constantWrench).norm(), 1e-9);
    const Eigen::VectorXd y = mass * ar + c * vr + gravity;
    const auto t = myrbf_wbc::baseTransform(q.segment<3>(3));
    const WbcVector6 wrench = reference.constantWrench + reference.jointAccelerationWrench * aj;
    EXPECT_LT((t.transpose() * wrench - y.head<6>()).norm(), 1e-10);
    Eigen::VectorXd s = v - vr;
    EXPECT_LT(s.tail(12).norm(), 1e-14);
    EXPECT_NEAR(reference.chi.dot(wrench), s.dot(y), 1e-9);

    // The full mechanical storage identity remains exact with s_joint=0.
    Eigen::VectorXd actualAcceleration = Eigen::VectorXd::Random(18);
    actualAcceleration.tail(12) = aj;  // s_joint and its derivative are both zero in the ideal execution identity.
    const Eigen::VectorXd sdot = actualAcceleration - ar;
    const double storageDerivative = s.dot(mass * sdot) + 0.5 * s.dot(mdot * s);
    const double errorPower = s.dot(mass * sdot + c * s);
    EXPECT_NEAR(storageDerivative, errorPower, 1e-7);

    pinocchio::computeJointJacobians(model, data, q);
    pinocchio::updateFramePlacements(model, data);
    for (int leg = 0; leg < 4; ++leg) {
      const auto frame = model.getFrameId("foot" + std::to_string(leg));
      Eigen::Matrix<double, 6, Eigen::Dynamic> jac(6, model.nv); jac.setZero();
      pinocchio::getFrameJacobian(model, data, frame, pinocchio::LOCAL_WORLD_ALIGNED, jac);
      Eigen::Matrix<double, 6, 3> map;
      map.topRows<3>().setIdentity();
      map.bottomRows<3>() = myrbf_wbc::skew(data.oMf[frame].translation() - q.head<3>());
      EXPECT_LT((t.transpose() * map - jac.topLeftCorner(3, 6).transpose()).norm(), 1e-10);
    }
  }
}

}  // namespace
}  // namespace legged
