#include "ocs2_legged_robot_adaptive/adaptive/SlidingVariable.h"
#include "ocs2_legged_robot_adaptive/adaptive/QuaternionTrackingError.h"
#include <stdexcept>
namespace ocs2 { namespace legged_robot_adaptive { namespace adaptive {
SlidingVariable::SlidingVariable(const AdaptiveClfSettings& settings) : lambdaLinear_(settings.lambdaLinear), lambdaAngular_(settings.lambdaAngular) { validateAdaptiveClfSettings(settings); }
SlidingVariableResult SlidingVariable::compute(const RigidBodyTrackingState& actual, const RigidBodyTrackingState& desired) const {
  if (!actual.comPositionWorld.allFinite() || !actual.comVelocityWorld.allFinite() || !actual.angularVelocityWorld.allFinite() || !desired.comPositionWorld.allFinite() || !desired.comVelocityWorld.allFinite() || !desired.angularVelocityWorld.allFinite()) throw std::invalid_argument("[SlidingVariable] world 状态含 NaN/Inf。");
  SlidingVariableResult r;
  r.orientationErrorWorld = computeQuaternionTrackingError(actual.orientationWorldFromBase, desired.orientationWorldFromBase);
  // ============================================================
  // 论文的统一滑模变量：误差方向固定为 desired - actual。
  // sigma_l = v_d-v + Lambda_l(p_d-p)，sigma_o = omega_d-omega + Lambda_o e_o。
  // p/v 是 OCS2 所表示的质心 world 量；omega 必须由运动学恢复，不能误用 state[3:6] 质心角动量。
  // Lambda 是后续调参位置，完全由配置文件给定。
  // ============================================================
  r.velocityErrorWorld.head<3>() =
      desired.comVelocityWorld - actual.comVelocityWorld;
  r.velocityErrorWorld.tail<3>() =
      desired.angularVelocityWorld - actual.angularVelocityWorld;
  r.lambdaPoseErrorWorld.head<3>() =
      lambdaLinear_ * (desired.comPositionWorld - actual.comPositionWorld);
  r.lambdaPoseErrorWorld.tail<3>() =
      lambdaAngular_ * r.orientationErrorWorld;
  r.sigma = r.velocityErrorWorld + r.lambdaPoseErrorWorld;
  return r;
}
} } }
