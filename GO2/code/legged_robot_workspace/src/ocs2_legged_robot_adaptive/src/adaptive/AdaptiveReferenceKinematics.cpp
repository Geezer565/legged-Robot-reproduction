#include "ocs2_legged_robot_adaptive/adaptive/AdaptiveReferenceKinematics.h"

#include <stdexcept>

#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>

namespace ocs2 {
namespace legged_robot_adaptive {
namespace adaptive {

AdaptiveReferenceKinematicsAdapter::AdaptiveReferenceKinematicsAdapter(
    const PinocchioInterface& pinocchioInterface, CentroidalModelInfo info, AdaptiveClfSettings settings)
    : pinocchioInterface_(pinocchioInterface),
      info_(std::move(info)),
      stateAdapter_(pinocchioInterface_, info_),
      lambdaLinear_(settings.lambdaLinear),
      lambdaAngular_(settings.lambdaAngular),
      enableEncodedReferenceDerivatives_(settings.enableEncodedReferenceDerivatives) {
  validateAdaptiveClfSettings(settings);
}

AdaptiveReferenceKinematicsAdapter::AdaptiveReferenceKinematicsAdapter(
    const AdaptiveReferenceKinematicsAdapter& other)
    : pinocchioInterface_(other.pinocchioInterface_),
      info_(other.info_),
      stateAdapter_(pinocchioInterface_, info_),
      lambdaLinear_(other.lambdaLinear_),
      lambdaAngular_(other.lambdaAngular_),
      enableEncodedReferenceDerivatives_(other.enableEncodedReferenceDerivatives_) {}

AdaptiveReferenceKinematicsAdapter::Segment AdaptiveReferenceKinematicsAdapter::selectSegment(
    scalar_t time, const scalar_array_t& times) {
  if (times.empty()) {
    throw std::invalid_argument("[AdaptiveReferenceKinematics] TargetTrajectories 为空。");
  }
  if (times.size() < 2 || time >= times.back()) {
    // 约定：最后 knot 及其后为 stationary reference。这样 t_end 的 x_d 为终点，
    // v_d=0；不把 piecewise-linear 终点的不可微冲量送进 Eq.(8)/(11)。
    return {0, false};
  }
  if (time <= times.front()) {
    // 约定：起始 knot 使用右导数，因此新 pose target 一开始 v_d 即为首段斜率。
    return {0, true};
  }
  for (size_t i = 0; i + 1 < times.size(); ++i) {
    if (time < times[i + 1]) {
      return {i, true};
    }
  }
  return {0, false};
}

RigidBodyTrackingState AdaptiveReferenceKinematicsAdapter::desiredPoseState(const vector_t& state) const {
  // AdaptiveStateAdapter 正确从 Pinocchio 恢复 desired COM position 与 Q_WB；
  // 它旧有的 desired velocity 来自 state momentum，故由 compute() 覆盖为 pose
  // trajectory 的物理导数，绝不在这里当作 moving v_d 使用。
  return stateAdapter_.adaptDesired(state);
}

AdaptiveReferenceKinematics AdaptiveReferenceKinematicsAdapter::compute(
    scalar_t time, const vector_t& actualState, const TargetTrajectories& targetTrajectories) const {
  if (targetTrajectories.empty() || targetTrajectories.timeTrajectory.size() != targetTrajectories.stateTrajectory.size()) {
    throw std::invalid_argument("[AdaptiveReferenceKinematics] target trajectory 的 time/state 非法。");
  }

  AdaptiveReferenceKinematics result;
  result.actual = stateAdapter_.adaptActual(actualState);
  result.desired = enableEncodedReferenceDerivatives_
                       ? stateAdapter_.adaptDesiredWithEncodedVelocity(
                             targetTrajectories.getDesiredState(time))
                       : desiredPoseState(targetTrajectories.getDesiredState(time));

  const Segment segment = selectSegment(time, targetTrajectories.timeTrajectory);
  if (segment.active) {
    const scalar_t t0 = targetTrajectories.timeTrajectory[segment.index];
    const scalar_t t1 = targetTrajectories.timeTrajectory[segment.index + 1];
    const scalar_t dt = t1 - t0;
    if (!(dt > 0.0)) {
      throw std::invalid_argument("[AdaptiveReferenceKinematics] target knot 时间必须严格递增。");
    }
    const auto start = enableEncodedReferenceDerivatives_
                           ? stateAdapter_.adaptDesiredWithEncodedVelocity(
                                 targetTrajectories.stateTrajectory[segment.index])
                           : desiredPoseState(targetTrajectories.stateTrajectory[segment.index]);
    const auto finish = enableEncodedReferenceDerivatives_
                            ? stateAdapter_.adaptDesiredWithEncodedVelocity(
                                  targetTrajectories.stateTrajectory[segment.index + 1])
                            : desiredPoseState(targetTrajectories.stateTrajectory[segment.index + 1]);

    if (enableEncodedReferenceDerivatives_) {
      // The target knots carry explicit v_d/omega_d in normalized centroidal
      // momentum.  Linear interpolation therefore gives the desired velocity;
      // its segment derivative is the applied a_d/alpha_d.  This is a
      // transport contract, not a second estimator or controller law.
      result.desiredComAccelerationWorld =
          (finish.comVelocityWorld - start.comVelocityWorld) / dt;
      result.desiredAngularAccelerationWorld =
          (finish.angularVelocityWorld - start.angularVelocityWorld) / dt;
    } else {
      // 对当前原生 piecewise-linear pose target，COM endpoint secant 正是该段的
      // 物理 v_d。x-only 且 orientation/joints 固定时严格得到 0.10/0.20=0.5 m/s。
      result.desired.comVelocityWorld =
          (finish.comPositionWorld - start.comPositionWorld) / dt;
      result.desiredComAccelerationWorld.setZero();  // 每个线性段内部 a_d=0。

      // 原生 pose 用 ZYX Euler 分量线性插值。当前 x-only target 的 EulerDot=0，
      // 因而 omega_d=alpha_d=0；一般旋转轨迹的 omega_d 由当前 Q_WB Euler
      // velocity mapping 恢复。alpha_d 在本阶段只在分段内按零加速度近似。
      const vector3_t eulerDot =
          (targetTrajectories.stateTrajectory[segment.index + 1]
                   .segment<3>(9) -
           targetTrajectories.stateTrajectory[segment.index].segment<3>(9)) /
          dt;
      const vector3_t euler =
          targetTrajectories.getDesiredState(time).segment<3>(9);
      result.desired.angularVelocityWorld =
          getGlobalAngularVelocityFromEulerAnglesZyxDerivatives<scalar_t>(
              euler, eulerDot);
      result.desiredAngularAccelerationWorld.setZero();
    }
  } else {
    // target 区间外为常值 pose，物理速度和加速度都为零。
    result.desired.comVelocityWorld.setZero();
    result.desiredComAccelerationWorld.setZero();
    result.desired.angularVelocityWorld.setZero();
    result.desiredAngularAccelerationWorld.setZero();
  }

  const vector3_t positionError = result.desired.comPositionWorld - result.actual.comPositionWorld;
  const vector3_t velocityError = result.desired.comVelocityWorld - result.actual.comVelocityWorld;
  result.orientationErrorWorld = computeQuaternionTrackingError(
      result.actual.orientationWorldFromBase, result.desired.orientationWorldFromBase);
  result.orientationErrorDerivativeWorld = computeQuaternionTrackingErrorDerivative(
      result.actual.orientationWorldFromBase, result.desired.orientationWorldFromBase,
      result.actual.angularVelocityWorld, result.desired.angularVelocityWorld);

  // 论文 tracking reference，且与 desired-actual SlidingVariable 严格同号。
  result.referenceLinearVelocityWorld = result.desired.comVelocityWorld + lambdaLinear_ * positionError;
  result.referenceLinearAccelerationWorld = result.desiredComAccelerationWorld + lambdaLinear_ * velocityError;
  result.referenceAngularVelocityWorld = result.desired.angularVelocityWorld + lambdaAngular_ * result.orientationErrorWorld;
  result.referenceAngularAccelerationWorld =
      result.desiredAngularAccelerationWorld + lambdaAngular_ * result.orientationErrorDerivativeWorld;
  return result;
}

}  // namespace adaptive
}  // namespace legged_robot_adaptive
}  // namespace ocs2
