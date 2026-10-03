#include "ocs2_legged_robot_adaptive/dynamics/MassMismatchGroundTruth.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>

namespace ocs2 {
namespace legged_robot_adaptive {

MassMismatchPlantDynamics::MassMismatchPlantDynamics(
    const PinocchioInterface& pinocchioInterface, CentroidalModelInfo info)
    : pinocchioInterface_(pinocchioInterface),
      info_(std::move(info)),
      mapping_(info_),
      dynamics_(info_) {
  if (info_.centroidalModelType !=
          CentroidalModelType::SingleRigidBodyDynamics ||
      !(info_.robotMass > 0.0) || !std::isfinite(info_.robotMass) ||
      info_.stateDim < 6 || info_.inputDim <= 0) {
    throw std::invalid_argument(
        "[MassMismatchPlantDynamics] Task C requires finite positive mass and SRBD dimensions.");
  }
  mapping_.setPinocchioInterface(pinocchioInterface_);
  dynamics_.setPinocchioInterface(pinocchioInterface_);
}

MassMismatchPlantDynamics::MassMismatchPlantDynamics(
    const MassMismatchPlantDynamics& other)
    : SystemDynamicsBase(other),
      pinocchioInterface_(other.pinocchioInterface_),
      info_(other.info_),
      mapping_(info_),
      dynamics_(info_) {
  mapping_.setPinocchioInterface(pinocchioInterface_);
  dynamics_.setPinocchioInterface(pinocchioInterface_);
}

MassMismatchPlantDynamics* MassMismatchPlantDynamics::clone() const {
  return new MassMismatchPlantDynamics(*this);
}

void MassMismatchPlantDynamics::validateStateInput(
    const vector_t& state, const vector_t& input) const {
  if (state.size() != info_.stateDim || input.size() != info_.inputDim ||
      !state.allFinite() || !input.allFinite()) {
    throw std::invalid_argument(
        "[MassMismatchPlantDynamics] State/input dimensions or values are invalid.");
  }
}

void MassMismatchPlantDynamics::updateKinematics(const vector_t& state) {
  const vector_t q = mapping_.getPinocchioJointPosition(state);
  updateCentroidalDynamics(pinocchioInterface_, info_, q);
}

vector_t MassMismatchPlantDynamics::computeFlowMap(
    scalar_t time, const vector_t& state, const vector_t& input,
    const PreComputation&) {
  validateStateInput(state, input);
  updateKinematics(state);
  return dynamics_.getValue(time, state, input);
}

VectorFunctionLinearApproximation
MassMismatchPlantDynamics::linearApproximation(
    scalar_t time, const vector_t& state, const vector_t& input,
    const PreComputation&) {
  validateStateInput(state, input);
  updateKinematics(state);
  const vector_t q = mapping_.getPinocchioJointPosition(state);
  const vector_t v = mapping_.getPinocchioJointVelocity(state, input);
  updateCentroidalDynamicsDerivatives(pinocchioInterface_, info_, q, v);
  return dynamics_.getLinearApproximation(time, state, input);
}

CentroidalModelInfo MassMismatchGroundTruth::makeTrueInfo(
    const CentroidalModelInfo& nominalInfo, scalar_t truePlantMass) {
  if (!(nominalInfo.robotMass > 0.0) ||
      !std::isfinite(nominalInfo.robotMass) || !(truePlantMass > 0.0) ||
      !std::isfinite(truePlantMass)) {
    throw std::invalid_argument(
        "[MassMismatchGroundTruth] Nominal and true plant masses must be positive and finite.");
  }
  CentroidalModelInfo trueInfo = nominalInfo;
  trueInfo.robotMass = truePlantMass;
  return trueInfo;
}

MassMismatchGroundTruth::MassMismatchGroundTruth(
    const PinocchioInterface& pinocchioInterface,
    CentroidalModelInfo nominalInfo, scalar_t truePlantMass)
    : nominalInfo_(std::move(nominalInfo)),
      trueInfo_(makeTrueInfo(nominalInfo_, truePlantMass)),
      nominalDynamics_(pinocchioInterface, nominalInfo_),
      trueDynamics_(pinocchioInterface, trueInfo_) {}

adaptive::vector6_t MassMismatchGroundTruth::computeEquivalentUncertainty(
    const adaptive::vector6_t& nominalFlow,
    const adaptive::vector6_t& trueFlow, scalar_t nominalMass) {
  if (!nominalFlow.allFinite() || !trueFlow.allFinite() ||
      !(nominalMass > 0.0) || !std::isfinite(nominalMass)) {
    throw std::invalid_argument(
        "[MassMismatchGroundTruth] Flow samples and nominal mass must be valid.");
  }
  return -nominalMass * (trueFlow - nominalFlow);
}

adaptive::vector6_t MassMismatchGroundTruth::reconstructTrueFlow(
    const adaptive::vector6_t& nominalFlow,
    const adaptive::vector6_t& piStar, scalar_t nominalMass) {
  if (!nominalFlow.allFinite() || !piStar.allFinite() ||
      !(nominalMass > 0.0) || !std::isfinite(nominalMass)) {
    throw std::invalid_argument(
        "[MassMismatchGroundTruth] Flow, pi_star and nominal mass must be valid.");
  }
  return nominalFlow - piStar / nominalMass;
}

MassMismatchGroundTruthSample MassMismatchGroundTruth::evaluate(
    scalar_t time, const vector_t& state, const vector_t& input) {
  if (!std::isfinite(time)) {
    throw std::invalid_argument(
        "[MassMismatchGroundTruth] Evaluation time must be finite.");
  }

  const vector_t nominalFull =
      nominalDynamics_.computeFlowMap(time, state, input, PreComputation());
  const vector_t trueFull =
      trueDynamics_.computeFlowMap(time, state, input, PreComputation());

  MassMismatchGroundTruthSample sample;
  sample.nominalFlow = nominalFull.head<6>();
  sample.trueFlow = trueFull.head<6>();
  sample.normalizedDynamicsResidual = sample.trueFlow - sample.nominalFlow;
  sample.piStar = computeEquivalentUncertainty(
      sample.nominalFlow, sample.trueFlow, nominalInfo_.robotMass);
  sample.reconstructionError =
      reconstructTrueFlow(sample.nominalFlow, sample.piStar,
                          nominalInfo_.robotMass) -
      sample.trueFlow;
  sample.fullFlowResidualNorm = (trueFull - nominalFull).norm();
  sample.unmatchedFlowResidualNorm =
      state.size() > 6 ? (trueFull - nominalFull).tail(state.size() - 6).norm()
                       : 0.0;
  return sample;
}

}  // namespace legged_robot_adaptive
}  // namespace ocs2
