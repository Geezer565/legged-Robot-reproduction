#pragma once

#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include "ocs2_legged_robot_adaptive/adaptive/SlidingVariable.h"

namespace ocs2 {
namespace legged_robot_adaptive {
namespace adaptive {

/** 将 OCS2 SRBD 状态转换为统一 sigma 需要的 world-frame 物理状态。 */
class AdaptiveStateAdapter final {
 public:
  AdaptiveStateAdapter(const PinocchioInterface& pinocchioInterface,
                       CentroidalModelInfo centroidalModelInfo);

  RigidBodyTrackingState adaptActual(const vector_t& state);
  RigidBodyTrackingState adaptDesired(const vector_t& desiredState);
  /**
   * Decode a desired state whose normalized centroidal momentum was generated
   * from an explicit desired base twist.  The caller must use zero desired
   * joint velocity; TargetTrajectories does not carry a desired input here.
   */
  RigidBodyTrackingState adaptDesiredWithEncodedVelocity(
      const vector_t& desiredState);

 private:
  RigidBodyTrackingState adapt(const vector_t& state, bool isDesired);

  PinocchioInterface pinocchioInterface_;
  CentroidalModelInfo info_;
  CentroidalModelPinocchioMapping mapping_;
};

}  // namespace adaptive
}  // namespace legged_robot_adaptive
}  // namespace ocs2
