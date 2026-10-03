#include "ocs2_legged_robot_adaptive/adaptive/AdaptiveReferenceStateEncoder.h"

#include <stdexcept>
#include <utility>

#include <ocs2_robotic_tools/common/RotationTransforms.h>

namespace ocs2 {
namespace legged_robot_adaptive {
namespace adaptive {

AdaptiveReferenceStateEncoder::AdaptiveReferenceStateEncoder(
    const PinocchioInterface& pinocchioInterface, CentroidalModelInfo info)
    : info_(std::move(info)), conversions_(pinocchioInterface, info_) {
  if (info_.centroidalModelType !=
      CentroidalModelType::SingleRigidBodyDynamics) {
    throw std::invalid_argument(
        "[AdaptiveReferenceStateEncoder] only SRBD is supported.");
  }
}

vector_t AdaptiveReferenceStateEncoder::encode(
    const vector3_t& desiredComPositionWorld,
    const vector3_t& desiredEulerZyx,
    const vector3_t& desiredComVelocityWorld,
    const vector3_t& desiredAngularVelocityWorld,
    const vector_t& desiredJointPosition) {
  if (desiredJointPosition.size() !=
          static_cast<Eigen::Index>(info_.actuatedDofNum) ||
      !desiredComPositionWorld.allFinite() || !desiredEulerZyx.allFinite() ||
      !desiredComVelocityWorld.allFinite() ||
      !desiredAngularVelocityWorld.allFinite() ||
      !desiredJointPosition.allFinite()) {
    throw std::invalid_argument(
        "[AdaptiveReferenceStateEncoder] desired pose/twist is invalid.");
  }

  const matrix3_t rotationWorldFromBase =
      getRotationMatrixFromZyxEulerAngles<scalar_t>(desiredEulerZyx);
  const vector3_t comToBaseWorld =
      rotationWorldFromBase * info_.comToBasePositionNominal;
  const vector3_t desiredBasePositionWorld =
      desiredComPositionWorld + comToBaseWorld;
  const vector3_t desiredBaseVelocityWorld =
      desiredComVelocityWorld +
      desiredAngularVelocityWorld.cross(comToBaseWorld);

  vector_t rbdState = vector_t::Zero(2 * info_.generalizedCoordinatesNum);
  rbdState.head<3>() = desiredEulerZyx;
  rbdState.segment<3>(3) = desiredBasePositionWorld;
  rbdState.segment(6, info_.actuatedDofNum) = desiredJointPosition;
  rbdState.segment<3>(info_.generalizedCoordinatesNum) =
      desiredAngularVelocityWorld;
  rbdState.segment<3>(info_.generalizedCoordinatesNum + 3) =
      desiredBaseVelocityWorld;
  return conversions_.computeCentroidalStateFromRbdModel(rbdState);
}

}  // namespace adaptive
}  // namespace legged_robot_adaptive
}  // namespace ocs2
