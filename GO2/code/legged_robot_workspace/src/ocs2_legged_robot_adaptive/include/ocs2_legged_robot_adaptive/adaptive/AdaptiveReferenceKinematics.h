#pragma once

#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include "ocs2_legged_robot_adaptive/adaptive/AdaptiveStateAdapter.h"
#include "ocs2_legged_robot_adaptive/adaptive/QuaternionTrackingError.h"

namespace ocs2 {
namespace legged_robot_adaptive {
namespace adaptive {

// ============================================================
// 论文 Eq.(8)/(11) 共用的 reference kinematics / reference dynamics。
//
// desired* 是由 TargetTrajectories 的 pose/configuration trajectory 恢复的
// 物理 world-frame 量，不能把 state[0:3] 的零 momentum reference 当作
// moving pose trajectory 的速度。reference* 是论文 tracking reference：
// v_r = v_d + Lambda_l(p_d-p)，dot(v_r)=a_d+Lambda_l(v_d-v)。
// ============================================================
struct AdaptiveReferenceKinematics {
  RigidBodyTrackingState actual;
  RigidBodyTrackingState desired;
  vector3_t desiredComAccelerationWorld = vector3_t::Zero();
  vector3_t desiredAngularAccelerationWorld = vector3_t::Zero();
  vector3_t orientationErrorWorld = vector3_t::Zero();
  vector3_t orientationErrorDerivativeWorld = vector3_t::Zero();
  vector3_t referenceLinearVelocityWorld = vector3_t::Zero();
  vector3_t referenceLinearAccelerationWorld = vector3_t::Zero();
  vector3_t referenceAngularVelocityWorld = vector3_t::Zero();
  vector3_t referenceAngularAccelerationWorld = vector3_t::Zero();
};

class AdaptiveReferenceKinematicsAdapter final {
 public:
  AdaptiveReferenceKinematicsAdapter(const PinocchioInterface& pinocchioInterface, CentroidalModelInfo info,
                                     AdaptiveClfSettings settings);
  AdaptiveReferenceKinematicsAdapter(const AdaptiveReferenceKinematicsAdapter& other);

  /**
   * 从原生 TargetTrajectories 的 configuration/pose curve 构造论文 reference。
   * 该函数不修改 TargetTrajectories，只为 adaptive Eq.(8)/(11) 旁路恢复其导数。
   */
  AdaptiveReferenceKinematics compute(scalar_t time, const vector_t& actualState,
                                      const TargetTrajectories& targetTrajectories) const;

 private:
  struct Segment {
    size_t index = 0;
    bool active = false;
  };
  static Segment selectSegment(scalar_t time, const scalar_array_t& times);
  RigidBodyTrackingState desiredPoseState(const vector_t& state) const;

  PinocchioInterface pinocchioInterface_;
  CentroidalModelInfo info_;
  mutable AdaptiveStateAdapter stateAdapter_;
  matrix3_t lambdaLinear_;
  matrix3_t lambdaAngular_;
  bool enableEncodedReferenceDerivatives_ = false;
};

}  // namespace adaptive
}  // namespace legged_robot_adaptive
}  // namespace ocs2
