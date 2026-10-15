#pragma once

#include <cstdint>
#include <memory>

#include <ocs2_core/dynamics/SystemDynamicsBase.h>
#include <ocs2_legged_robot/dynamics/LeggedRobotDynamicsAD.h>
#include <ocs2_oc/synchronized_module/ReferenceManagerInterface.h>

#include "ocs2_legged_robot_adaptive/adaptive/AdaptiveModelData.h"
#include "ocs2_legged_robot_adaptive/adaptive/AdaptiveReferenceKinematics.h"
#include "ocs2_legged_robot_adaptive/adaptive/AdaptiveRigidBodyRegressor.h"
#include "ocs2_legged_robot_adaptive/adaptive/PayloadMass1D.h"
#include "ocs2_legged_robot_adaptive/dynamics/NominalCoordinatePlantDynamics.h"

namespace ocs2 {
namespace legged_robot_adaptive {

/**
 * Adaptive SRBD prediction dynamics.
 *
 * The paper mode implements Eq. (10) literally:
 *   xdot = xdot_nominal + [-Y_u(q,v,v_r,dot(v_r))*pi_hat/m_nominal; 0].
 * Historical model-rebuild behavior remains available only through the
 * explicit legacy setting.
 */
class AdaptiveLeggedRobotDynamics final : public SystemDynamicsBase {
 public:
  AdaptiveLeggedRobotDynamics(
      const PinocchioInterface& pinocchioInterface,
      CentroidalModelInfo centroidalModelInfo,
      const std::string& modelName,
      const legged_robot::ModelSettings& modelSettings,
      adaptive::AdaptiveClfSettings settings,
      std::shared_ptr<const adaptive::AdaptiveModelData> modelData,
      const ReferenceManagerInterface* referenceManager =
          nullptr);

  AdaptiveLeggedRobotDynamics* clone() const override;
  vector_t computeFlowMap(scalar_t time, const vector_t& state,
                          const vector_t& input,
                          const PreComputation& preComputation) override;
  VectorFunctionLinearApproximation linearApproximation(
      scalar_t time, const vector_t& state, const vector_t& input,
      const PreComputation& preComputation) override;

  static adaptive::vector6_t normalizedMomentumRateCorrection(
      const adaptive::vector6_t& parameterEstimate, scalar_t robotMass);
  static adaptive::vector6_t normalizedMomentumRateCorrection(
      const adaptive::regressor_t& regressor,
      const adaptive::parameter_vector_t& parameterEstimate,
      scalar_t robotMass);
  static adaptive::vector6_t normalizedMomentumRateCorrection(
      adaptive::AdaptiveParameterization parameterization,
      const adaptive::vector6_t& parameterEstimate,
      const adaptive::vector6_t& nominalNormalizedMomentumRate,
      scalar_t robotMass, scalar_t minimumEstimatedTotalMass);

 private:
  adaptive::vector6_t paperRegressorCorrection(
      scalar_t time, const vector_t& state,
      const adaptive::AdaptiveModelSnapshotPtr& snapshot) const;
  NominalCoordinatePlantDynamics& payloadInertialDynamics(
      const adaptive::AdaptiveModelSnapshotPtr& snapshot);
  NominalCoordinatePlantDynamics& payloadInertialDynamics(
      const vector_t& inertialEstimate, std::uint64_t snapshotVersion);

  legged_robot::LeggedRobotDynamicsAD nominalDynamics_;
  PinocchioInterface pinocchioInterface_;
  CentroidalModelInfo info_;
  std::string modelName_;
  legged_robot::ModelSettings modelSettings_;
  adaptive::AdaptiveClfSettings settings_;
  adaptive::AdaptiveParameterization parameterization_ =
      adaptive::AdaptiveParameterization::ConstantWrench6D;
  adaptive::AdaptiveDynamicsForm adaptiveDynamicsForm_ =
      adaptive::AdaptiveDynamicsForm::LegacyEstimatedModel;
  scalar_t minimumEstimatedTotalMass_ = 1.0;
  std::shared_ptr<const adaptive::AdaptiveModelData> modelData_;
  const ReferenceManagerInterface* referenceManagerPtr_ =
      nullptr;
  std::unique_ptr<adaptive::AdaptiveReferenceKinematicsAdapter>
      referenceKinematicsPtr_;
  adaptive::AdaptiveRigidBodyRegressor regressor_;
  std::unique_ptr<NominalCoordinatePlantDynamics>
      payloadInertialDynamicsPtr_;
  std::uint64_t payloadInertialSnapshotVersion_ = 0;
};

}  // namespace legged_robot_adaptive
}  // namespace ocs2
