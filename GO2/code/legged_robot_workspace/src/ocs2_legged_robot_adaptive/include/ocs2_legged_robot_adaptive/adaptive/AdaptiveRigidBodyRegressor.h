#pragma once

#include "ocs2_legged_robot_adaptive/adaptive/AdaptiveTypes.h"

namespace ocs2 {
namespace legged_robot_adaptive {
namespace adaptive {
/**
 * Eq. (8)/(11) unknown-term regressor in OCS2 world-wrench coordinates.
 * ConstantWrench6D remains the historical world-frame I6 model.  The complete
 * paper parameterization uses [Y_in, diag(R_WB,R_WB)] because the paper stores
 * its final [force; torque] parameters in the base frame.
 */
class AdaptiveRigidBodyRegressor final {
 public:
  struct NominalParameters {
    scalar_t mass = 1.0;
    matrix3_t inertiaBaseAboutCom = matrix3_t::Identity();
    vector3_t gravityWorld = (vector3_t() << 0.0, 0.0, -9.81).finished();
  };
  struct EvaluationPoint {
    matrix3_t rotationWorldFromBase = matrix3_t::Identity();
    vector3_t actualAngularVelocityWorld = vector3_t::Zero();
    vector3_t referenceLinearAccelerationWorld = vector3_t::Zero();
    vector3_t referenceAngularVelocityWorld = vector3_t::Zero();
    vector3_t referenceAngularAccelerationWorld = vector3_t::Zero();
  };
  AdaptiveRigidBodyRegressor();
  explicit AdaptiveRigidBodyRegressor(NominalParameters parameters);
  vector6_t computeNominalReferenceWrench(const EvaluationPoint& point) const;
  regressor_t computeUnknownReferenceRegressor(
      const EvaluationPoint& point,
      AdaptiveParameterization parameterization,
      AdaptiveDynamicsForm dynamicsForm =
          AdaptiveDynamicsForm::LegacyEstimatedModel) const;
  /** Backward-compatible ConstantWrench6D anchor. */
  matrix6_t computeUnknownReferenceRegressor() const;
  regressor_t computeUnknownDynamicsRegressor(
      const EvaluationPoint& point,
      AdaptiveParameterization parameterization,
      AdaptiveDynamicsForm dynamicsForm =
          AdaptiveDynamicsForm::LegacyEstimatedModel) const;
  /** Backward-compatible ConstantWrench6D anchor. */
  matrix6_t computeUnknownDynamicsRegressor() const;
 private:
  NominalParameters parameters_;
};
}  // namespace adaptive
}  // namespace legged_robot_adaptive
}  // namespace ocs2
