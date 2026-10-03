#pragma once

#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_centroidal_model/PinocchioCentroidalDynamics.h>
#include <ocs2_core/dynamics/SystemDynamicsBase.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include "ocs2_legged_robot_adaptive/adaptive/AdaptiveTypes.h"

namespace ocs2 {
namespace legged_robot_adaptive {

/**
 * Non-AD centroidal dynamics used only by the animal_dummy true-plant rollout
 * and Task C diagnostics. The equations are the scalar version of the same
 * Pinocchio centroidal flow map used to generate LeggedRobotDynamicsAD.
 */
class MassMismatchPlantDynamics final : public SystemDynamicsBase {
 public:
  MassMismatchPlantDynamics(const PinocchioInterface& pinocchioInterface,
                            CentroidalModelInfo info);

  MassMismatchPlantDynamics* clone() const override;

  vector_t computeFlowMap(scalar_t time, const vector_t& state,
                          const vector_t& input,
                          const PreComputation& preComputation) override;

  VectorFunctionLinearApproximation linearApproximation(
      scalar_t time, const vector_t& state, const vector_t& input,
      const PreComputation& preComputation) override;

  const CentroidalModelInfo& modelInfo() const { return info_; }

 private:
  MassMismatchPlantDynamics(const MassMismatchPlantDynamics& other);

  void validateStateInput(const vector_t& state, const vector_t& input) const;
  void updateKinematics(const vector_t& state);

  PinocchioInterface pinocchioInterface_;
  CentroidalModelInfo info_;
  CentroidalModelPinocchioMapping mapping_;
  PinocchioCentroidalDynamics dynamics_;
};

struct MassMismatchGroundTruthSample {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  adaptive::vector6_t nominalFlow = adaptive::vector6_t::Zero();
  adaptive::vector6_t trueFlow = adaptive::vector6_t::Zero();
  adaptive::vector6_t normalizedDynamicsResidual = adaptive::vector6_t::Zero();
  adaptive::vector6_t piStar = adaptive::vector6_t::Zero();
  adaptive::vector6_t reconstructionError = adaptive::vector6_t::Zero();
  scalar_t fullFlowResidualNorm = 0.0;
  scalar_t unmatchedFlowResidualNorm = 0.0;
};

/**
 * Task C mass-only ground truth evaluated in one common OCS2 state/input
 * coordinate. It constructs both models from the same nominal model info and
 * changes only trueInfo.robotMass.
 *
 * With Eq. (10), f_adaptive_6 = f_nominal_6 - pi_hat / m_nominal.
 * Therefore pi_star = -m_nominal * (f_true_6 - f_nominal_6).
 */
class MassMismatchGroundTruth final {
 public:
  MassMismatchGroundTruth(const PinocchioInterface& pinocchioInterface,
                          CentroidalModelInfo nominalInfo,
                          scalar_t truePlantMass);

  MassMismatchGroundTruthSample evaluate(scalar_t time, const vector_t& state,
                                         const vector_t& input);

  const MassMismatchPlantDynamics& truePlantDynamics() const {
    return trueDynamics_;
  }
  scalar_t nominalMass() const { return nominalInfo_.robotMass; }
  scalar_t trueMass() const { return trueInfo_.robotMass; }
  const CentroidalModelInfo& nominalInfo() const { return nominalInfo_; }
  const CentroidalModelInfo& trueInfo() const { return trueInfo_; }

  static adaptive::vector6_t computeEquivalentUncertainty(
      const adaptive::vector6_t& nominalFlow,
      const adaptive::vector6_t& trueFlow, scalar_t nominalMass);

  static adaptive::vector6_t reconstructTrueFlow(
      const adaptive::vector6_t& nominalFlow,
      const adaptive::vector6_t& piStar, scalar_t nominalMass);

 private:
  static CentroidalModelInfo makeTrueInfo(const CentroidalModelInfo& nominalInfo,
                                         scalar_t truePlantMass);

  CentroidalModelInfo nominalInfo_;
  CentroidalModelInfo trueInfo_;
  MassMismatchPlantDynamics nominalDynamics_;
  MassMismatchPlantDynamics trueDynamics_;
};

}  // namespace legged_robot_adaptive
}  // namespace ocs2
