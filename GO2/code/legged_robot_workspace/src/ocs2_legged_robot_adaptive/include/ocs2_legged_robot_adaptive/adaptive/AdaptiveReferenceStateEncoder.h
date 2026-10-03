#pragma once

#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include "ocs2_legged_robot_adaptive/adaptive/AdaptiveTypes.h"

namespace ocs2 {
namespace legged_robot_adaptive {
namespace adaptive {

/**
 * Encodes a fixed-joint desired COM pose/twist into native OCS2 centroidal
 * state semantics.  This is a target-construction utility; it does not touch
 * Eq. (8), Eq. (10), Eq. (11), or the plant model.
 */
class AdaptiveReferenceStateEncoder final {
 public:
  AdaptiveReferenceStateEncoder(const PinocchioInterface& pinocchioInterface,
                                CentroidalModelInfo info);

  vector_t encode(const vector3_t& desiredComPositionWorld,
                  const vector3_t& desiredEulerZyx,
                  const vector3_t& desiredComVelocityWorld,
                  const vector3_t& desiredAngularVelocityWorld,
                  const vector_t& desiredJointPosition);

 private:
  CentroidalModelInfo info_;
  CentroidalModelRbdConversions conversions_;
};

}  // namespace adaptive
}  // namespace legged_robot_adaptive
}  // namespace ocs2
