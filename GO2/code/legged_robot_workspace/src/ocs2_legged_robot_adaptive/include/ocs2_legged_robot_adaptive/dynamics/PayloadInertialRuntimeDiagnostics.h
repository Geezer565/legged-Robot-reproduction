#pragma once

#include <cstdint>
#include <memory>

#include <ocs2_core/Types.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include "ocs2_legged_robot_adaptive/adaptive/AdaptiveModelSnapshot.h"
#include "ocs2_legged_robot_adaptive/dynamics/NominalCoordinatePlantDynamics.h"

namespace ocs2 {
namespace legged_robot_adaptive {

/** One executed-state diagnostic sample for the 10D full-model Eq. (10). */
struct PayloadInertialRuntimeSample {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  bool estimatedModelValid = false;
  bool adaptiveDynamicsApplied = false;
  scalar_t estimatedTotalMass = 0.0;
  adaptive::vector3_t estimatedTotalComShiftNominalComBase =
      adaptive::vector3_t::Zero();
  adaptive::matrix3_t estimatedTotalInertiaComBase =
      adaptive::matrix3_t::Zero();
  vector_t nominalFlow;
  vector_t candidateFullFlowCorrection;
  vector_t appliedFullFlowCorrection;
};

/**
 * Reconstructs the exact versioned frozen-model flow used by the 10D
 * AdaptiveLeggedRobotDynamics, but only on the executed state for logging.
 * It never changes the estimator, snapshot, OCP, or policy.
 */
class PayloadInertialRuntimeDiagnostics final {
 public:
  PayloadInertialRuntimeDiagnostics(const PinocchioInterface& pinocchioInterface,
                                    CentroidalModelInfo nominalInfo,
                                    scalar_t minimumEstimatedTotalMass);

  PayloadInertialRuntimeSample evaluate(
      scalar_t time, const vector_t& nominalState, const vector_t& input,
      const adaptive::AdaptiveModelSnapshotPtr& snapshot,
      bool enableAdaptiveDynamics);

 private:
  PinocchioInterface pinocchioInterface_;
  CentroidalModelInfo nominalInfo_;
  scalar_t minimumEstimatedTotalMass_;
  NominalCoordinatePlantDynamics nominalDynamics_;
  std::unique_ptr<NominalCoordinatePlantDynamics> estimatedDynamicsPtr_;
  std::uint64_t estimatedSnapshotVersion_ = 0;
};

}  // namespace legged_robot_adaptive
}  // namespace ocs2
