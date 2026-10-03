#include "ocs2_legged_robot_adaptive/dynamics/NominalCoordinatePlantDynamics.h"

#include <array>
#include <cmath>
#include <stdexcept>
#include <utility>

#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <ocs2_robotic_tools/common/SkewSymmetricMatrix.h>

namespace ocs2 {
namespace legged_robot_adaptive {
namespace {

constexpr scalar_t kCoordinateDerivativeStep = 1.0e-6;

}  // namespace

NominalCoordinatePlantDynamics::NominalCoordinatePlantDynamics(
    const PinocchioInterface& pinocchioInterface,
    CentroidalModelInfo nominalInfo, CentroidalModelInfo plantInfo)
    : pinocchioInterface_(pinocchioInterface),
      nominalInfo_(std::move(nominalInfo)),
      plantInfo_(std::move(plantInfo)),
      plantDynamics_(pinocchioInterface_, plantInfo_) {
  validateInfoPair(nominalInfo_, plantInfo_);
}

NominalCoordinatePlantDynamics::NominalCoordinatePlantDynamics(
    const NominalCoordinatePlantDynamics& other)
    : SystemDynamicsBase(other),
      pinocchioInterface_(other.pinocchioInterface_),
      nominalInfo_(other.nominalInfo_),
      plantInfo_(other.plantInfo_),
      plantDynamics_(pinocchioInterface_, plantInfo_) {}

NominalCoordinatePlantDynamics* NominalCoordinatePlantDynamics::clone() const {
  return new NominalCoordinatePlantDynamics(*this);
}

void NominalCoordinatePlantDynamics::validateInfoPair(
    const CentroidalModelInfo& nominalInfo,
    const CentroidalModelInfo& plantInfo) {
  if (nominalInfo.centroidalModelType !=
          CentroidalModelType::SingleRigidBodyDynamics ||
      plantInfo.centroidalModelType !=
          CentroidalModelType::SingleRigidBodyDynamics ||
      nominalInfo.stateDim != plantInfo.stateDim ||
      nominalInfo.inputDim != plantInfo.inputDim ||
      nominalInfo.generalizedCoordinatesNum !=
          plantInfo.generalizedCoordinatesNum ||
      nominalInfo.stateDim < 12 || !(nominalInfo.robotMass > 0.0) ||
      !(plantInfo.robotMass > 0.0)) {
    throw std::invalid_argument(
        "[NominalCoordinatePlantDynamics] compatible finite SRBD model infos are required.");
  }
}

NominalCoordinatePlantDynamics::matrix6_t
NominalCoordinatePlantDynamics::normalizedMomentumMap(
    const CentroidalModelInfo& info,
    const vector3_t& eulerAnglesZyx) {
  const auto rotationBaseToWorld =
      getRotationMatrixFromZyxEulerAngles(eulerAnglesZyx);
  const auto angularVelocityMap =
      getMappingFromEulerAnglesZyxDerivativeToGlobalAngularVelocity(
          eulerAnglesZyx);
  const vector3_t comToBaseWorld =
      rotationBaseToWorld * info.comToBasePositionNominal;
  const Eigen::Matrix<scalar_t, 3, 3> inertiaWorld =
      rotationBaseToWorld * info.centroidalInertiaNominal *
      rotationBaseToWorld.transpose();

  matrix6_t map = matrix6_t::Zero();
  map.topLeftCorner<3, 3>().setIdentity();
  map.topRightCorner<3, 3>() =
      skewSymmetricMatrix(comToBaseWorld) * angularVelocityMap;
  map.bottomRightCorner<3, 3>() =
      inertiaWorld * angularVelocityMap / info.robotMass;
  if (!map.allFinite() || std::abs(map.determinant()) < 1.0e-12) {
    throw std::domain_error(
        "[NominalCoordinatePlantDynamics] normalized momentum map is singular.");
  }
  return map;
}

NominalCoordinatePlantDynamics::matrix6_t
NominalCoordinatePlantDynamics::momentumCoordinateMap(
    const CentroidalModelInfo& fromInfo,
    const CentroidalModelInfo& toInfo,
    const vector3_t& eulerAnglesZyx) {
  const matrix6_t fromMap = normalizedMomentumMap(fromInfo, eulerAnglesZyx);
  const matrix6_t toMap = normalizedMomentumMap(toInfo, eulerAnglesZyx);
  return toMap * fromMap.inverse();
}

vector_t NominalCoordinatePlantDynamics::transformState(
    const CentroidalModelInfo& fromInfo,
    const CentroidalModelInfo& toInfo, const vector_t& fromState) {
  if (fromState.size() != fromInfo.stateDim || !fromState.allFinite()) {
    throw std::invalid_argument(
        "[NominalCoordinatePlantDynamics] state dimension/value is invalid.");
  }
  vector_t toState = fromState;
  const vector3_t eulerAngles = fromState.segment<3>(9);
  toState.head<6>() =
      momentumCoordinateMap(fromInfo, toInfo, eulerAngles) *
      fromState.head<6>();
  return toState;
}

matrix_t NominalCoordinatePlantDynamics::stateJacobian(
    const CentroidalModelInfo& fromInfo,
    const CentroidalModelInfo& toInfo, const vector_t& fromState) {
  const Eigen::Index stateDim = fromInfo.stateDim;
  matrix_t jacobian = matrix_t::Identity(stateDim, stateDim);
  const vector3_t eulerAngles = fromState.segment<3>(9);
  jacobian.topLeftCorner<6, 6>() =
      momentumCoordinateMap(fromInfo, toInfo, eulerAngles);
  for (Eigen::Index axis = 0; axis < 3; ++axis) {
    vector3_t plus = eulerAngles;
    vector3_t minus = eulerAngles;
    plus(axis) += kCoordinateDerivativeStep;
    minus(axis) -= kCoordinateDerivativeStep;
    const matrix6_t derivative =
        (momentumCoordinateMap(fromInfo, toInfo, plus) -
         momentumCoordinateMap(fromInfo, toInfo, minus)) /
        (2.0 * kCoordinateDerivativeStep);
    jacobian.block<6, 1>(0, 9 + axis) =
        derivative * fromState.head<6>();
  }
  return jacobian;
}

vector_t NominalCoordinatePlantDynamics::plantStateFromNominalState(
    const vector_t& nominalState) const {
  return transformState(nominalInfo_, plantInfo_, nominalState);
}

vector_t NominalCoordinatePlantDynamics::nominalStateFromPlantState(
    const vector_t& plantState) const {
  return transformState(plantInfo_, nominalInfo_, plantState);
}

matrix_t
NominalCoordinatePlantDynamics::plantStateJacobianFromNominalState(
    const vector_t& nominalState) const {
  return stateJacobian(nominalInfo_, plantInfo_, nominalState);
}

matrix_t
NominalCoordinatePlantDynamics::nominalStateJacobianFromPlantState(
    const vector_t& plantState) const {
  return stateJacobian(plantInfo_, nominalInfo_, plantState);
}

vector_t NominalCoordinatePlantDynamics::computeFlowMap(
    scalar_t time, const vector_t& nominalState, const vector_t& input,
    const PreComputation& preComputation) {
  const vector_t plantState = plantStateFromNominalState(nominalState);
  const vector_t plantFlow = plantDynamics_.computeFlowMap(
      time, plantState, input, preComputation);
  return nominalStateJacobianFromPlantState(plantState) * plantFlow;
}

VectorFunctionLinearApproximation
NominalCoordinatePlantDynamics::linearApproximation(
    scalar_t time, const vector_t& nominalState, const vector_t& input,
    const PreComputation& preComputation) {
  const vector_t plantState = plantStateFromNominalState(nominalState);
  const auto plantApproximation = plantDynamics_.linearApproximation(
      time, plantState, input, preComputation);
  const matrix_t plantFromNominal =
      plantStateJacobianFromNominalState(nominalState);
  const matrix_t nominalFromPlant =
      nominalStateJacobianFromPlantState(plantState);

  auto approximation = VectorFunctionLinearApproximation::Zero(
      nominalInfo_.stateDim, nominalInfo_.stateDim, nominalInfo_.inputDim);
  approximation.f = nominalFromPlant * plantApproximation.f;
  approximation.dfdx = nominalFromPlant * plantApproximation.dfdx *
                       plantFromNominal;
  approximation.dfdu = nominalFromPlant * plantApproximation.dfdu;

  // Coordinate-curvature term: d(J_phi)/dx_nominal * f_plant. J_phi only
  // varies with normalized momentum and base orientation for SRBD.
  constexpr std::array<Eigen::Index, 9> activeColumns{{
      0, 1, 2, 3, 4, 5, 9, 10, 11}};
  for (const Eigen::Index column : activeColumns) {
    vector_t plus = nominalState;
    vector_t minus = nominalState;
    plus(column) += kCoordinateDerivativeStep;
    minus(column) -= kCoordinateDerivativeStep;
    const vector_t plantPlus = plantStateFromNominalState(plus);
    const vector_t plantMinus = plantStateFromNominalState(minus);
    const matrix_t jacobianPlus =
        nominalStateJacobianFromPlantState(plantPlus);
    const matrix_t jacobianMinus =
        nominalStateJacobianFromPlantState(plantMinus);
    approximation.dfdx.col(column) +=
        ((jacobianPlus - jacobianMinus) /
         (2.0 * kCoordinateDerivativeStep)) *
        plantApproximation.f;
  }
  return approximation;
}

}  // namespace legged_robot_adaptive
}  // namespace ocs2
