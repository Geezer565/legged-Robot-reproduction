#pragma once

#include "legged_interface/adaptive/AdaptiveTypes.h"

namespace legged {
namespace adaptive {

/**
 * Computes the six-axis ETH sliding variable from actual and desired RBD
 * states. RBD layout is [EulerZYX, base position, joints, world angular
 * velocity, world linear velocity, joint velocity].
 */
vector6_t computeMyRbfTrackingError(
    const vector_t& actualRbdState, const vector_t& desiredRbdState,
    Eigen::Index generalizedCoordinatesNum, scalar_t lambdaLinear,
    scalar_t lambdaAngular);

}  // namespace adaptive
}  // namespace legged
