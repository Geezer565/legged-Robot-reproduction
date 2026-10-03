#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/data.hpp>

#include "ocs2_legged_robot_adaptive/adaptive/AdaptiveStateAdapter.h"

#include <stdexcept>
#include <utility>

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

namespace ocs2 {
namespace legged_robot_adaptive {
namespace adaptive {

AdaptiveStateAdapter::AdaptiveStateAdapter(const PinocchioInterface& pinocchioInterface,
                                           CentroidalModelInfo centroidalModelInfo)
    : pinocchioInterface_(pinocchioInterface),
      info_(std::move(centroidalModelInfo)),
      mapping_(info_) {
  if (info_.centroidalModelType != CentroidalModelType::SingleRigidBodyDynamics) {
    throw std::invalid_argument("[AdaptiveStateAdapter] 本阶段只支持 SingleRigidBodyDynamics。");
  }
  mapping_.setPinocchioInterface(pinocchioInterface_);
}

RigidBodyTrackingState AdaptiveStateAdapter::adaptActual(const vector_t& state) {
  return adapt(state, false);
}

RigidBodyTrackingState AdaptiveStateAdapter::adaptDesired(const vector_t& desiredState) {
  return adapt(desiredState, true);
}

RigidBodyTrackingState AdaptiveStateAdapter::adaptDesiredWithEncodedVelocity(
    const vector_t& desiredState) {
  return adapt(desiredState, false);
}

RigidBodyTrackingState AdaptiveStateAdapter::adapt(const vector_t& state, bool isDesired) {
  if (state.size() != static_cast<Eigen::Index>(info_.stateDim) || !state.allFinite()) {
    throw std::invalid_argument("[AdaptiveStateAdapter] OCS2 centroidal state 维度或数值非法。");
  }

  const vector_t q = mapping_.getPinocchioJointPosition(state);
  updateCentroidalDynamics(pinocchioInterface_, info_, q);

  RigidBodyTrackingState result;
  result.comPositionWorld = pinocchioInterface_.getData().com[0];
  result.comVelocityWorld = centroidal_model::getNormalizedMomentum(state, info_).head<3>();

  const vector3_t eulerZyx = q.segment<3>(3);
  result.orientationWorldFromBase = getQuaternionFromEulerAnglesZyx<scalar_t>(eulerZyx);
  result.orientationWorldFromBase.normalize();

  if (isDesired) {
    // TargetTrajectories 的 state[3:6] 同样是归一化质心角动量，不是 omega。
    // 第一阶段的 reference 约定未提供角速度，因此 desired omega 保守置零，绝不错误转换该动量。
    result.angularVelocityWorld.setZero();
  } else {
    const vector_t zeroInput = vector_t::Zero(info_.inputDim);
    const vector_t generalizedVelocity = mapping_.getPinocchioJointVelocity(state, zeroInput);
    result.angularVelocityWorld = getGlobalAngularVelocityFromEulerAnglesZyxDerivatives<scalar_t>(
        eulerZyx, generalizedVelocity.segment<3>(3));
  }

  if (!result.comPositionWorld.allFinite() || !result.comVelocityWorld.allFinite() ||
      !result.orientationWorldFromBase.coeffs().allFinite() || !result.angularVelocityWorld.allFinite()) {
    throw std::runtime_error("[AdaptiveStateAdapter] 物理 world-frame 状态恢复失败。");
  }
  return result;
}

}  // namespace adaptive
}  // namespace legged_robot_adaptive
}  // namespace ocs2
