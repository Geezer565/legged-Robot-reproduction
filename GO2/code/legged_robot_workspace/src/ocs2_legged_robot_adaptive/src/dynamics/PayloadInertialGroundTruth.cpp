#include "ocs2_legged_robot_adaptive/dynamics/PayloadInertialGroundTruth.h"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace ocs2 {
namespace legged_robot_adaptive {

PayloadInertialGroundTruth::PayloadInertialGroundTruth(
    const PinocchioInterface& pinocchioInterface,
    CentroidalModelInfo nominalInfo,
    adaptive::PayloadInertial10D::Parameters payloadParameters)
    : nominalInfo_(std::move(nominalInfo)),
      trueInfo_(adaptive::PayloadInertial10D::makePlantModelInfo(
          nominalInfo_, payloadParameters)),
      payloadParameters_(std::move(payloadParameters)),
      nominalDynamics_(pinocchioInterface, nominalInfo_),
      trueDynamics_(pinocchioInterface, trueInfo_) {}

PayloadInertialGroundTruthSample PayloadInertialGroundTruth::evaluate(
    scalar_t time, const vector_t& state, const vector_t& input) {
  if (!std::isfinite(time)) {
    throw std::invalid_argument(
        "[PayloadInertialGroundTruth] evaluation time must be finite.");
  }
  PayloadInertialGroundTruthSample sample;
  sample.nominalFlow = nominalDynamics_.computeFlowMap(
      time, state, input, PreComputation());
  sample.trueFlow = trueDynamics_.computeFlowMap(
      time, state, input, PreComputation());
  sample.fullFlowResidual = sample.trueFlow - sample.nominalFlow;
  sample.normalizedMomentumRateResidual =
      sample.fullFlowResidual.head<6>();
  sample.coordinateFlowResidual = sample.fullFlowResidual.tail(
      sample.fullFlowResidual.size() - 6);
  sample.normalizedMomentumRateResidualNorm =
      sample.normalizedMomentumRateResidual.norm();
  sample.coordinateFlowResidualNorm = sample.coordinateFlowResidual.norm();
  sample.fullFlowResidualNorm = sample.fullFlowResidual.norm();
  return sample;
}

}  // namespace legged_robot_adaptive
}  // namespace ocs2

