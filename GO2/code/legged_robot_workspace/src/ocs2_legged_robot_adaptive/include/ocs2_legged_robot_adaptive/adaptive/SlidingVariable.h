#pragma once

#include "ocs2_legged_robot_adaptive/adaptive/AdaptiveTypes.h"

namespace ocs2 {
namespace legged_robot_adaptive {
namespace adaptive {

/**
 * 供 estimator 与 CLF 共用的物理量；均为 world frame。
 * 这里 angularVelocityWorld 是实际角速度，刻意不是 OCS2 state[3:6] 的质心角动量。
 */
struct RigidBodyTrackingState {
  vector3_t comPositionWorld = vector3_t::Zero();
  vector3_t comVelocityWorld = vector3_t::Zero();
  quaternion_t orientationWorldFromBase = quaternion_t::Identity();
  vector3_t angularVelocityWorld = vector3_t::Zero();
};

struct SlidingVariableResult {
  vector3_t orientationErrorWorld = vector3_t::Zero();
  /** [v_d-v; omega_d-omega], world frame. */
  vector6_t velocityErrorWorld = vector6_t::Zero();
  /** [Lambda_l(p_d-p); Lambda_o e_o], world frame. */
  vector6_t lambdaPoseErrorWorld = vector6_t::Zero();
  vector6_t sigma = vector6_t::Zero();  // [sigma_l; sigma_o]
};

class SlidingVariable final {
 public:
  explicit SlidingVariable(const AdaptiveClfSettings& settings);
  SlidingVariableResult compute(const RigidBodyTrackingState& actual,
                                const RigidBodyTrackingState& desired) const;
 private:
  matrix3_t lambdaLinear_;
  matrix3_t lambdaAngular_;
};
}  // namespace adaptive
}  // namespace legged_robot_adaptive
}  // namespace ocs2
