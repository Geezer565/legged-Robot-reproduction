#include "ocs2_legged_robot_adaptive/adaptive/PayloadInertial10D.h"

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ocs2 {
namespace legged_robot_adaptive {
namespace adaptive {
namespace {

matrix3_t symmetricInertiaBasis(std::size_t index) {
  matrix3_t basis = matrix3_t::Zero();
  switch (index) {
    case 0:
      basis(0, 0) = 1.0;
      break;
    case 1:
      basis(0, 1) = basis(1, 0) = 1.0;
      break;
    case 2:
      basis(0, 2) = basis(2, 0) = 1.0;
      break;
    case 3:
      basis(1, 1) = 1.0;
      break;
    case 4:
      basis(1, 2) = basis(2, 1) = 1.0;
      break;
    case 5:
      basis(2, 2) = 1.0;
      break;
    default:
      throw std::out_of_range("[PayloadInertial10D] inertia basis index must be in [0,5].");
  }
  return basis;
}

bool isSymmetric(const matrix3_t& matrix, scalar_t tolerance) {
  return (matrix - matrix.transpose()).cwiseAbs().maxCoeff() <= tolerance;
}

bool isValidRigidBodyComInertia(const matrix3_t& inertia, scalar_t tolerance) {
  if (!inertia.allFinite() || !isSymmetric(inertia, tolerance)) {
    return false;
  }
  Eigen::SelfAdjointEigenSolver<matrix3_t> solver(
      0.5 * (inertia + inertia.transpose()));
  if (solver.info() != Eigen::Success) {
    return false;
  }
  const vector3_t principal = solver.eigenvalues();
  if (principal.minCoeff() < -tolerance) {
    return false;
  }
  // Principal moments of a physical rigid body obey the triangle inequality.
  return principal(2) <= principal(0) + principal(1) + tolerance;
}

}  // namespace

PayloadInertial10D::Parameters PayloadInertial10D::fromVector(
    const vector10_t& parameterVector) {
  if (!parameterVector.allFinite()) {
    throw std::invalid_argument(
        "[PayloadInertial10D] parameter vector contains NaN/Inf.");
  }
  Parameters parameters;
  parameters.mass = parameterVector(0);
  parameters.firstMomentNominalComBase = parameterVector.segment<3>(1);
  parameters.inertiaAboutNominalComBase <<
      parameterVector(4), parameterVector(5), parameterVector(6),
      parameterVector(5), parameterVector(7), parameterVector(8),
      parameterVector(6), parameterVector(8), parameterVector(9);
  return parameters;
}

vector10_t PayloadInertial10D::toVector(const Parameters& parameters) {
  if (!std::isfinite(parameters.mass) ||
      !parameters.firstMomentNominalComBase.allFinite() ||
      !parameters.inertiaAboutNominalComBase.allFinite()) {
    throw std::invalid_argument(
        "[PayloadInertial10D] payload parameters contain NaN/Inf.");
  }
  if (!isSymmetric(parameters.inertiaAboutNominalComBase, 1.0e-12)) {
    throw std::invalid_argument(
        "[PayloadInertial10D] inertia about nominal CoM must be symmetric.");
  }
  vector10_t result;
  result << parameters.mass, parameters.firstMomentNominalComBase,
      parameters.inertiaAboutNominalComBase(0, 0),
      parameters.inertiaAboutNominalComBase(0, 1),
      parameters.inertiaAboutNominalComBase(0, 2),
      parameters.inertiaAboutNominalComBase(1, 1),
      parameters.inertiaAboutNominalComBase(1, 2),
      parameters.inertiaAboutNominalComBase(2, 2);
  return result;
}

PayloadInertial10D::Parameters PayloadInertial10D::fromPhysicalPayload(
    scalar_t mass, const vector3_t& positionNominalComToPayloadComBase,
    const matrix3_t& inertiaPayloadComBase) {
  if (!(mass > 0.0) || !std::isfinite(mass) ||
      !positionNominalComToPayloadComBase.allFinite() ||
      !isValidRigidBodyComInertia(inertiaPayloadComBase, 1.0e-10)) {
    throw std::invalid_argument(
        "[PayloadInertial10D] physical payload mass/CoM/inertia is invalid.");
  }
  Parameters parameters;
  parameters.mass = mass;
  parameters.firstMomentNominalComBase =
      mass * positionNominalComToPayloadComBase;
  const matrix3_t symmetricPayloadComInertia =
      0.5 * (inertiaPayloadComBase + inertiaPayloadComBase.transpose());
  parameters.inertiaAboutNominalComBase =
      symmetricPayloadComInertia +
      mass * parallelAxisMatrix(positionNominalComToPayloadComBase);
  validatePhysicalPayload(parameters);
  return parameters;
}

matrix3_t PayloadInertial10D::inertiaAboutPayloadCom(
    const Parameters& parameters) {
  if (!(parameters.mass > 0.0) || !std::isfinite(parameters.mass)) {
    throw std::invalid_argument(
        "[PayloadInertial10D] positive mass is required to recover payload CoM inertia.");
  }
  const vector3_t displacement =
      parameters.firstMomentNominalComBase / parameters.mass;
  return 0.5 *
         (parameters.inertiaAboutNominalComBase +
          parameters.inertiaAboutNominalComBase.transpose() -
          2.0 * parameters.mass * parallelAxisMatrix(displacement));
}

bool PayloadInertial10D::isPhysicallyConsistent(
    const Parameters& parameters, scalar_t tolerance) {
  if (!(tolerance >= 0.0) || !std::isfinite(tolerance) ||
      !std::isfinite(parameters.mass) ||
      !parameters.firstMomentNominalComBase.allFinite() ||
      !parameters.inertiaAboutNominalComBase.allFinite() ||
      !isSymmetric(parameters.inertiaAboutNominalComBase, tolerance)) {
    return false;
  }
  if (std::abs(parameters.mass) <= tolerance) {
    return parameters.firstMomentNominalComBase.norm() <= tolerance &&
           parameters.inertiaAboutNominalComBase.norm() <= tolerance;
  }
  if (parameters.mass < tolerance) {
    return false;
  }
  return isValidRigidBodyComInertia(inertiaAboutPayloadCom(parameters),
                                    tolerance);
}

void PayloadInertial10D::validatePhysicalPayload(
    const Parameters& parameters, scalar_t tolerance) {
  if (!isPhysicallyConsistent(parameters, tolerance)) {
    throw std::domain_error(
        "[PayloadInertial10D] payload spatial inertia is not physically consistent.");
  }
}

matrix6x10_t PayloadInertial10D::referenceRegressor(
    const ReferencePoint& point) {
  validateReferencePoint(point);
  const vector3_t effectiveAcceleration =
      point.referenceLinearAccelerationNominalComWorld - point.gravityWorld;

  matrix6x10_t regressor = matrix6x10_t::Zero();
  regressor.block<3, 1>(0, 0) = effectiveAcceleration;

  for (std::size_t i = 0; i < 3; ++i) {
    const vector3_t firstMomentWorld =
        point.rotationWorldFromBase.col(static_cast<Eigen::Index>(i));
    regressor.block<3, 1>(0, 1 + i) =
        point.referenceAngularAccelerationWorld.cross(firstMomentWorld) +
        point.actualAngularVelocityWorld.cross(
            point.referenceAngularVelocityWorld.cross(firstMomentWorld));
    regressor.block<3, 1>(3, 1 + i) =
        firstMomentWorld.cross(effectiveAcceleration);
  }

  for (std::size_t i = 0; i < 6; ++i) {
    const matrix3_t inertiaWorld =
        point.rotationWorldFromBase * symmetricInertiaBasis(i) *
        point.rotationWorldFromBase.transpose();
    regressor.block<3, 1>(3, 4 + i) =
        inertiaWorld * point.referenceAngularAccelerationWorld +
        point.actualAngularVelocityWorld.cross(
            inertiaWorld * point.referenceAngularVelocityWorld);
  }
  return regressor;
}

vector6_t PayloadInertial10D::referenceWrench(
    const ReferencePoint& point, const vector10_t& parameterVector) {
  if (!parameterVector.allFinite()) {
    throw std::invalid_argument(
        "[PayloadInertial10D] parameter vector contains NaN/Inf.");
  }
  return referenceRegressor(point) * parameterVector;
}

CentroidalModelInfo PayloadInertial10D::makePlantModelInfo(
    const CentroidalModelInfo& nominalInfo,
    const Parameters& payloadParameters) {
  if (nominalInfo.centroidalModelType !=
          CentroidalModelType::SingleRigidBodyDynamics ||
      !(nominalInfo.robotMass > 0.0) ||
      !std::isfinite(nominalInfo.robotMass) ||
      !nominalInfo.comToBasePositionNominal.allFinite() ||
      !isValidRigidBodyComInertia(nominalInfo.centroidalInertiaNominal,
                                  1.0e-10)) {
    throw std::invalid_argument(
        "[PayloadInertial10D] nominal model must be a valid SRBD model.");
  }
  validatePhysicalPayload(payloadParameters);

  return makeEstimatedModelInfo(nominalInfo, toVector(payloadParameters),
                                1.0e-12);
}

CentroidalModelInfo PayloadInertial10D::makeEstimatedModelInfo(
    const CentroidalModelInfo& nominalInfo,
    const vector10_t& estimatedParameterVector,
    scalar_t minimumEstimatedTotalMass) {
  if (nominalInfo.centroidalModelType !=
          CentroidalModelType::SingleRigidBodyDynamics ||
      !(nominalInfo.robotMass > 0.0) ||
      !std::isfinite(nominalInfo.robotMass) ||
      !nominalInfo.comToBasePositionNominal.allFinite() ||
      !isValidRigidBodyComInertia(nominalInfo.centroidalInertiaNominal,
                                  1.0e-10)) {
    throw std::invalid_argument(
        "[PayloadInertial10D] nominal model must be a valid SRBD model.");
  }
  if (!estimatedParameterVector.allFinite() ||
      !(minimumEstimatedTotalMass > 0.0) ||
      !std::isfinite(minimumEstimatedTotalMass)) {
    throw std::invalid_argument(
        "[PayloadInertial10D] estimated parameters/mass floor are invalid.");
  }
  const Parameters estimatedParameters = fromVector(estimatedParameterVector);

  if (estimatedParameterVector.isZero(0.0)) {
    return nominalInfo;
  }

  const scalar_t totalMass = nominalInfo.robotMass + estimatedParameters.mass;
  if (!(totalMass >= minimumEstimatedTotalMass) || !std::isfinite(totalMass)) {
    throw std::domain_error(
        "[PayloadInertial10D] combined total mass is invalid.");
  }

  // C0 is the nominal robot CoM. The nominal robot has zero first moment
  // about C0, so the total-CoM shift is exactly h_payload / m_total.
  const vector3_t totalComFromNominalComBase =
      estimatedParameters.firstMomentNominalComBase / totalMass;
  const matrix3_t totalInertiaAboutNominalComBase =
      nominalInfo.centroidalInertiaNominal +
      estimatedParameters.inertiaAboutNominalComBase;
  const matrix3_t totalInertiaAboutTotalComBase =
      totalInertiaAboutNominalComBase -
      totalMass * parallelAxisMatrix(totalComFromNominalComBase);
  if (!isValidRigidBodyComInertia(totalInertiaAboutTotalComBase, 1.0e-9)) {
    throw std::domain_error(
        "[PayloadInertial10D] combined centroidal inertia is invalid.");
  }

  const vector3_t baseToNominalComBase =
      -nominalInfo.comToBasePositionNominal;
  const vector3_t baseToTotalComBase =
      baseToNominalComBase + totalComFromNominalComBase;

  CentroidalModelInfo trueInfo = nominalInfo;
  trueInfo.robotMass = totalMass;
  trueInfo.comToBasePositionNominal = -baseToTotalComBase;
  trueInfo.centroidalInertiaNominal =
      0.5 * (totalInertiaAboutTotalComBase +
             totalInertiaAboutTotalComBase.transpose());
  return trueInfo;
}

matrix3_t PayloadInertial10D::parallelAxisMatrix(
    const vector3_t& displacement) {
  return displacement.squaredNorm() * matrix3_t::Identity() -
         displacement * displacement.transpose();
}

void PayloadInertial10D::validateReferencePoint(
    const ReferencePoint& point) {
  if (!point.rotationWorldFromBase.allFinite() ||
      !point.actualAngularVelocityWorld.allFinite() ||
      !point.referenceLinearAccelerationNominalComWorld.allFinite() ||
      !point.referenceAngularVelocityWorld.allFinite() ||
      !point.referenceAngularAccelerationWorld.allFinite() ||
      !point.gravityWorld.allFinite()) {
    throw std::invalid_argument(
        "[PayloadInertial10D] reference point contains NaN/Inf.");
  }
  const matrix3_t orthogonalityError =
      point.rotationWorldFromBase.transpose() *
          point.rotationWorldFromBase -
      matrix3_t::Identity();
  if (orthogonalityError.cwiseAbs().maxCoeff() > 1.0e-9 ||
      std::abs(point.rotationWorldFromBase.determinant() - 1.0) > 1.0e-9) {
    throw std::invalid_argument(
        "[PayloadInertial10D] rotationWorldFromBase is not a rotation matrix.");
  }
}

}  // namespace adaptive
}  // namespace legged_robot_adaptive
}  // namespace ocs2
