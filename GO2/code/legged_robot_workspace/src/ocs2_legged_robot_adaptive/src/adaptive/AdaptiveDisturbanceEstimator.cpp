#include "ocs2_legged_robot_adaptive/adaptive/AdaptiveDisturbanceEstimator.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace ocs2 {
namespace legged_robot_adaptive {

AdaptiveDisturbanceEstimator::AdaptiveDisturbanceEstimator() : AdaptiveDisturbanceEstimator(Config()) {}

AdaptiveDisturbanceEstimator::AdaptiveDisturbanceEstimator(Config config)
    : config_(std::move(config)),
      activeEstimate_(),
      estimateBuffer_(std::unique_ptr<AdaptiveDisturbanceEstimate>(new AdaptiveDisturbanceEstimate(activeEstimate_))) {}

void AdaptiveDisturbanceEstimator::preSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t& initState,
                                                const ReferenceManagerInterface& referenceManager) {
  auto lockedEstimate = estimateBuffer_.lock();

  if (std::isfinite(lastUpdateTime_)) {
    const scalar_t dt = std::max<scalar_t>(0.0, initTime - lastUpdateTime_);
    const auto desiredState = referenceManager.getTargetTrajectories().getDesiredState(initTime);

    const legged_robot::vector3_t positionError = initState.segment<3>(6) - desiredState.segment<3>(6);
    const legged_robot::vector3_t comVelocityError = initState.segment<3>(0) - desiredState.segment<3>(0);
    const legged_robot::vector3_t orientationError = initState.segment<3>(9) - desiredState.segment<3>(9);
    const legged_robot::vector3_t angularMomentumError = initState.segment<3>(3) - desiredState.segment<3>(3);

    const legged_robot::vector3_t sigmaPosition = comVelocityError + config_.lambdaPosition * positionError;
    const legged_robot::vector3_t sigmaOrientation = angularMomentumError + config_.lambdaOrientation * orientationError;

    // Online disturbance adaptation: d_hat_dot = -Gamma * sigma
    lockedEstimate->externalForceInWorld -= config_.forceAdaptationGain * dt * sigmaPosition;
    lockedEstimate->externalTorqueInWorld -= config_.torqueAdaptationGain * dt * sigmaOrientation;

    lockedEstimate->externalForceInWorld = clampNorm(lockedEstimate->externalForceInWorld, config_.maxForceEstimate);
    lockedEstimate->externalTorqueInWorld = clampNorm(lockedEstimate->externalTorqueInWorld, config_.maxTorqueEstimate);
  }

  lastUpdateTime_ = initTime;
  activeEstimate_ = *lockedEstimate;
}

legged_robot::vector3_t AdaptiveDisturbanceEstimator::clampNorm(const legged_robot::vector3_t& value, scalar_t maxNorm) {
  const scalar_t norm = value.norm();
  if (norm <= maxNorm || norm < 1e-8) {
    return value;
  }
  return value * (maxNorm / norm);
}

}  // namespace legged_robot_adaptive
}  // namespace ocs2
