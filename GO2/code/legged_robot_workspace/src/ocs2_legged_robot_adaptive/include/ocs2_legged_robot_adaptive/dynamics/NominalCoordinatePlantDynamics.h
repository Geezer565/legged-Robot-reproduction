#pragma once

#include <ocs2_core/dynamics/SystemDynamicsBase.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include "ocs2_legged_robot_adaptive/dynamics/MassMismatchGroundTruth.h"

namespace ocs2 {
namespace legged_robot_adaptive {

/**
 * Expresses a plant model's SRBD flow in the controller's nominal centroidal
 * coordinates. Both states represent the same physical base pose/twist; only
 * normalized centroidal momentum coordinates differ.
 */
class NominalCoordinatePlantDynamics final : public SystemDynamicsBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  NominalCoordinatePlantDynamics(const PinocchioInterface& pinocchioInterface,
                                 CentroidalModelInfo nominalInfo,
                                 CentroidalModelInfo plantInfo);
  NominalCoordinatePlantDynamics(const NominalCoordinatePlantDynamics& other);

  NominalCoordinatePlantDynamics* clone() const override;
  vector_t computeFlowMap(scalar_t time, const vector_t& nominalState,
                          const vector_t& input,
                          const PreComputation& preComputation) override;
  VectorFunctionLinearApproximation linearApproximation(
      scalar_t time, const vector_t& nominalState, const vector_t& input,
      const PreComputation& preComputation) override;

  vector_t plantStateFromNominalState(const vector_t& nominalState) const;
  vector_t nominalStateFromPlantState(const vector_t& plantState) const;
  matrix_t plantStateJacobianFromNominalState(
      const vector_t& nominalState) const;
  matrix_t nominalStateJacobianFromPlantState(
      const vector_t& plantState) const;

  const CentroidalModelInfo& nominalInfo() const { return nominalInfo_; }
  const CentroidalModelInfo& plantInfo() const { return plantInfo_; }

 private:
  using matrix6_t = Eigen::Matrix<scalar_t, 6, 6>;
  using vector6_t = Eigen::Matrix<scalar_t, 6, 1>;
  using vector3_t = Eigen::Matrix<scalar_t, 3, 1>;

  static void validateInfoPair(const CentroidalModelInfo& nominalInfo,
                               const CentroidalModelInfo& plantInfo);
  static matrix6_t normalizedMomentumMap(
      const CentroidalModelInfo& info,
      const vector3_t& eulerAnglesZyx);
  static matrix6_t momentumCoordinateMap(
      const CentroidalModelInfo& fromInfo,
      const CentroidalModelInfo& toInfo,
      const vector3_t& eulerAnglesZyx);
  static matrix_t stateJacobian(const CentroidalModelInfo& fromInfo,
                                const CentroidalModelInfo& toInfo,
                                const vector_t& fromState);
  static vector_t transformState(const CentroidalModelInfo& fromInfo,
                                 const CentroidalModelInfo& toInfo,
                                 const vector_t& fromState);

  PinocchioInterface pinocchioInterface_;
  CentroidalModelInfo nominalInfo_;
  CentroidalModelInfo plantInfo_;
  MassMismatchPlantDynamics plantDynamics_;
};

}  // namespace legged_robot_adaptive
}  // namespace ocs2
