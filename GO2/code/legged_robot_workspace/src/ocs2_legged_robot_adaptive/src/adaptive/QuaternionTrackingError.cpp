#include "ocs2_legged_robot_adaptive/adaptive/QuaternionTrackingError.h"
#include <stdexcept>
namespace ocs2 { namespace legged_robot_adaptive { namespace adaptive {
vector3_t computeQuaternionTrackingError(const quaternion_t& actualInput, const quaternion_t& desiredInput) {
  if (!actualInput.coeffs().allFinite() || !desiredInput.coeffs().allFinite() || actualInput.norm() < 1e-12 || desiredInput.norm() < 1e-12)
    throw std::invalid_argument("[QuaternionTrackingError] Q_WB 必须有限且非退化。");
  quaternion_t actual = actualInput.normalized(), desired = desiredInput.normalized();
  // q 与 -q 物理等价；先对齐半球，避免参考轨迹跨符号面时出现假误差。
  if (actual.coeffs().dot(desired.coeffs()) < 0.0) desired.coeffs() *= -1.0;
  // desired * actual^{-1} 的向量部：论文采用 desired - actual，且 Q_WB 令此量在 world 系。
  return actual.w() * desired.vec() - desired.w() * actual.vec() - desired.vec().cross(actual.vec());
}

vector3_t computeQuaternionTrackingErrorDerivative(const quaternion_t& actualInput,
                                                    const quaternion_t& desiredInput,
                                                    const vector3_t& actualAngularVelocityWorld,
                                                    const vector3_t& desiredAngularVelocityWorld) {
  if (!actualAngularVelocityWorld.allFinite() || !desiredAngularVelocityWorld.allFinite()) {
    throw std::invalid_argument("[QuaternionTrackingError] world angular velocity 必须有限。");
  }
  quaternion_t actual = actualInput.normalized();
  quaternion_t desired = desiredInput.normalized();
  // 与 error 本身严格采用同一 hemisphere，避免 q/-q 等价表示让解析导数跳变。
  if (actual.coeffs().dot(desired.coeffs()) < 0.0) {
    desired.coeffs() *= -1.0;
  }
  const quaternion_t error = desired * actual.conjugate();  // Q_d * Q^{-1}
  const vector3_t& e = error.vec();
  return 0.5 * (error.w() * (desiredAngularVelocityWorld - actualAngularVelocityWorld) +
                desiredAngularVelocityWorld.cross(e) - e.cross(actualAngularVelocityWorld));
}
} } }
