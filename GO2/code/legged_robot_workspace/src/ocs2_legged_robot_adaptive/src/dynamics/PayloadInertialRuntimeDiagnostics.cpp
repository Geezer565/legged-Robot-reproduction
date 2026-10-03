#include "ocs2_legged_robot_adaptive/dynamics/PayloadInertialRuntimeDiagnostics.h"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include <ocs2_core/PreComputation.h>

#include "ocs2_legged_robot_adaptive/adaptive/PayloadInertial10D.h"

namespace ocs2 {
namespace legged_robot_adaptive {

PayloadInertialRuntimeDiagnostics::PayloadInertialRuntimeDiagnostics(
    const PinocchioInterface& pinocchioInterface,
    CentroidalModelInfo nominalInfo, scalar_t minimumEstimatedTotalMass)
    : pinocchioInterface_(pinocchioInterface),
      nominalInfo_(std::move(nominalInfo)),
      minimumEstimatedTotalMass_(minimumEstimatedTotalMass),
      nominalDynamics_(pinocchioInterface_, nominalInfo_, nominalInfo_) {
  if (!(minimumEstimatedTotalMass_ > 0.0) ||
      !std::isfinite(minimumEstimatedTotalMass_)) {
    throw std::invalid_argument(
        "[PayloadInertialRuntimeDiagnostics] invalid total-mass floor.");
  }
}

PayloadInertialRuntimeSample PayloadInertialRuntimeDiagnostics::evaluate(
    scalar_t time, const vector_t& nominalState, const vector_t& input,
    const adaptive::AdaptiveModelSnapshotPtr& snapshot,
    bool enableAdaptiveDynamics) {
  if (!snapshot || !snapshot->valid() || snapshot->estimate().size() != 10) {
    throw std::invalid_argument(
        "[PayloadInertialRuntimeDiagnostics] valid 10D snapshot required.");
  }

  PreComputation preComputation;
  PayloadInertialRuntimeSample sample;
  sample.nominalFlow = nominalDynamics_.computeFlowMap(
      time, nominalState, input, preComputation);
  sample.candidateFullFlowCorrection = vector_t::Zero(nominalInfo_.stateDim);
  sample.appliedFullFlowCorrection = vector_t::Zero(nominalInfo_.stateDim);

  try {
    adaptive::vector10_t estimate = snapshot->estimate();
    const CentroidalModelInfo estimatedInfo =
        adaptive::PayloadInertial10D::makeEstimatedModelInfo(
            nominalInfo_, estimate, minimumEstimatedTotalMass_);
    if (!estimatedDynamicsPtr_ ||
        estimatedSnapshotVersion_ != snapshot->version()) {
      estimatedDynamicsPtr_.reset(new NominalCoordinatePlantDynamics(
          pinocchioInterface_, nominalInfo_, estimatedInfo));
      estimatedSnapshotVersion_ = snapshot->version();
    }

    const vector_t estimatedFlow = estimatedDynamicsPtr_->computeFlowMap(
        time, nominalState, input, preComputation);
    sample.candidateFullFlowCorrection = estimatedFlow - sample.nominalFlow;
    if (enableAdaptiveDynamics) {
      sample.appliedFullFlowCorrection = sample.candidateFullFlowCorrection;
      sample.adaptiveDynamicsApplied = true;
    }
    sample.estimatedTotalMass = estimatedInfo.robotMass;
    // Both vectors are base-to-CoM, expressed in base axes. Their difference
    // is therefore the estimated total-CoM shift from nominal C0.
    sample.estimatedTotalComShiftNominalComBase =
        nominalInfo_.comToBasePositionNominal -
        estimatedInfo.comToBasePositionNominal;
    sample.estimatedTotalInertiaComBase =
        estimatedInfo.centroidalInertiaNominal;
    sample.estimatedModelValid = true;
  } catch (const std::domain_error&) {
    // Diagnostics report invalidity without hiding it behind clipping or
    // projection. The solver-side 10D dynamics retains its fail-fast policy.
    sample.estimatedTotalMass =
        std::numeric_limits<scalar_t>::quiet_NaN();
    sample.estimatedTotalComShiftNominalComBase.setConstant(
        std::numeric_limits<scalar_t>::quiet_NaN());
    sample.estimatedTotalInertiaComBase.setConstant(
        std::numeric_limits<scalar_t>::quiet_NaN());
  }
  return sample;
}

}  // namespace legged_robot_adaptive
}  // namespace ocs2
