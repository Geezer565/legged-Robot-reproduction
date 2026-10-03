#include "ocs2_legged_robot_adaptive/adaptive/AdaptiveRigidBodyRegressor.h"
#include "ocs2_legged_robot_adaptive/adaptive/PayloadInertial10D.h"
#include "ocs2_legged_robot_adaptive/adaptive/PayloadMass1D.h"
#include <stdexcept>
#include <utility>
namespace ocs2 { namespace legged_robot_adaptive { namespace adaptive {
AdaptiveRigidBodyRegressor::AdaptiveRigidBodyRegressor() : AdaptiveRigidBodyRegressor(NominalParameters()) {}
AdaptiveRigidBodyRegressor::AdaptiveRigidBodyRegressor(NominalParameters parameters) : parameters_(std::move(parameters)) {
  if (!(parameters_.mass > 0.0) || !parameters_.inertiaBaseAboutCom.allFinite() || !parameters_.gravityWorld.allFinite())
    throw std::invalid_argument("[AdaptiveRigidBodyRegressor] 名义刚体参数非法。");
}
vector6_t AdaptiveRigidBodyRegressor::computeNominalReferenceWrench(const EvaluationPoint& p) const {
  if (!p.rotationWorldFromBase.allFinite() || !p.actualAngularVelocityWorld.allFinite() || !p.referenceLinearAccelerationWorld.allFinite() || !p.referenceAngularVelocityWorld.allFinite() || !p.referenceAngularAccelerationWorld.allFinite())
    throw std::invalid_argument("[AdaptiveRigidBodyRegressor] world 系评估点非法。");
  const matrix3_t inertiaWorld = p.rotationWorldFromBase * parameters_.inertiaBaseAboutCom * p.rotationWorldFromBase.transpose();
  vector6_t wrench;
  // Eq.(11) 的 nominal_reference_wrench：前三维 M(a_ref-g)，后三维 I*alpha_ref + omega_actual x (I*omega_ref)。
  wrench.head<3>() = parameters_.mass * (p.referenceLinearAccelerationWorld - parameters_.gravityWorld);
  wrench.tail<3>() = inertiaWorld * p.referenceAngularAccelerationWorld + p.actualAngularVelocityWorld.cross(inertiaWorld * p.referenceAngularVelocityWorld);
  return wrench;
}
regressor_t AdaptiveRigidBodyRegressor::computeUnknownReferenceRegressor(
    const EvaluationPoint& point,
    AdaptiveParameterization parameterization,
    AdaptiveDynamicsForm dynamicsForm) const {
  if (parameterization == AdaptiveParameterization::ConstantWrench6D) {
    return matrix6_t::Identity();
  }
  if (parameterization == AdaptiveParameterization::PayloadInertial10D) {
    PayloadInertial10D::ReferencePoint payloadPoint;
    payloadPoint.rotationWorldFromBase = point.rotationWorldFromBase;
    payloadPoint.actualAngularVelocityWorld =
        point.actualAngularVelocityWorld;
    payloadPoint.referenceLinearAccelerationNominalComWorld =
        point.referenceLinearAccelerationWorld;
    payloadPoint.referenceAngularVelocityWorld =
        point.referenceAngularVelocityWorld;
    payloadPoint.referenceAngularAccelerationWorld =
        point.referenceAngularAccelerationWorld;
    payloadPoint.gravityWorld = parameters_.gravityWorld;
    return PayloadInertial10D::referenceRegressor(payloadPoint);
  }
  if (parameterization == AdaptiveParameterization::PayloadInertial10DPlusWrench6D) {
    // Joint Eq. (8)/(11) regressor.  The first ten columns are the
    // payload inertial block.  The final six are selected below according to
    // the explicit coordinate contract of the chosen dynamics form.
    matrix6x16_t regressor;
    PayloadInertial10D::ReferencePoint payloadPoint;
    payloadPoint.rotationWorldFromBase = point.rotationWorldFromBase;
    payloadPoint.actualAngularVelocityWorld = point.actualAngularVelocityWorld;
    payloadPoint.referenceLinearAccelerationNominalComWorld = point.referenceLinearAccelerationWorld;
    payloadPoint.referenceAngularVelocityWorld = point.referenceAngularVelocityWorld;
    payloadPoint.referenceAngularAccelerationWorld = point.referenceAngularAccelerationWorld;
    payloadPoint.gravityWorld = parameters_.gravityWorld;
    regressor.leftCols<10>() = PayloadInertial10D::referenceRegressor(payloadPoint);
    if (dynamicsForm ==
        AdaptiveDynamicsForm::PaperRegressorCompensation) {
      // The paper defines the constant force and torque in base frame.
      // OCS2 stores both normalized linear and angular momentum in world
      // coordinates, so the exact coordinate-equivalent map is diag(R,R).
      regressor.rightCols<6>().setZero();
      regressor.block<3, 3>(0, 10) = point.rotationWorldFromBase;
      regressor.block<3, 3>(3, 13) = point.rotationWorldFromBase;
    } else {
      regressor.rightCols<6>().setIdentity();
    }
    return regressor;
  }
  matrix6_t regressor = matrix6_t::Zero();
  regressor.col(0) = PayloadMass1D::referenceRegressorColumn(
      point.referenceLinearAccelerationWorld, parameters_.gravityWorld);
  return regressor;
}
matrix6_t AdaptiveRigidBodyRegressor::computeUnknownReferenceRegressor() const {
  return matrix6_t::Identity();
}
regressor_t AdaptiveRigidBodyRegressor::computeUnknownDynamicsRegressor(
    const EvaluationPoint& point,
    AdaptiveParameterization parameterization,
    AdaptiveDynamicsForm dynamicsForm) const {
  // Eq.(8) uses the modified reference regressor of paper Eq.(9).
  return computeUnknownReferenceRegressor(
      point, parameterization, dynamicsForm);
}
matrix6_t AdaptiveRigidBodyRegressor::computeUnknownDynamicsRegressor() const {
  return matrix6_t::Identity();
}
} } }
