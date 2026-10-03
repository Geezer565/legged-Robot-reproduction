#include "ocs2_legged_robot_adaptive/adaptive/PayloadMass1D.h"

#include <cmath>
#include <stdexcept>

namespace ocs2 {
namespace legged_robot_adaptive {
namespace adaptive {

vector6_t PayloadMass1D::referenceRegressorColumn(
    const vector3_t& referenceLinearAccelerationWorld,
    const vector3_t& gravityWorld) {
  if (!referenceLinearAccelerationWorld.allFinite() ||
      !gravityWorld.allFinite()) {
    throw std::invalid_argument(
        "[PayloadMass1D] Reference acceleration/gravity must be finite.");
  }
  vector6_t column = vector6_t::Zero();
  column.head<3>() = referenceLinearAccelerationWorld - gravityWorld;
  return column;
}

scalar_t PayloadMass1D::estimatedTotalMass(
    scalar_t nominalMass, scalar_t payloadMassEstimate,
    scalar_t minimumEstimatedTotalMass) {
  if (!(nominalMass > 0.0) || !std::isfinite(nominalMass) ||
      !std::isfinite(payloadMassEstimate) ||
      !(minimumEstimatedTotalMass > 0.0) ||
      !std::isfinite(minimumEstimatedTotalMass)) {
    throw std::invalid_argument("[PayloadMass1D] Mass arguments are invalid.");
  }
  const scalar_t totalMass = nominalMass + payloadMassEstimate;
  if (totalMass < minimumEstimatedTotalMass) {
    throw std::domain_error(
        "[PayloadMass1D] Estimated total mass is below the configured safety floor.");
  }
  return totalMass;
}

scalar_t PayloadMass1D::estimatedMassRatio(
    scalar_t nominalMass, scalar_t payloadMassEstimate,
    scalar_t minimumEstimatedTotalMass) {
  const scalar_t totalMass = estimatedTotalMass(
      nominalMass, payloadMassEstimate, minimumEstimatedTotalMass);
  return payloadMassEstimate / totalMass;
}

vector6_t PayloadMass1D::dynamicsEquivalentWrench(
    const vector6_t& contactWrenchWorld, scalar_t nominalMass,
    scalar_t payloadMassEstimate, scalar_t minimumEstimatedTotalMass) {
  if (!contactWrenchWorld.allFinite()) {
    throw std::invalid_argument(
        "[PayloadMass1D] Contact wrench must be finite.");
  }
  return estimatedMassRatio(nominalMass, payloadMassEstimate,
                            minimumEstimatedTotalMass) *
         contactWrenchWorld;
}

vector6_t PayloadMass1D::normalizedMomentumRateCorrection(
    const vector6_t& nominalNormalizedMomentumRate, scalar_t nominalMass,
    scalar_t payloadMassEstimate, scalar_t minimumEstimatedTotalMass) {
  if (!nominalNormalizedMomentumRate.allFinite()) {
    throw std::invalid_argument(
        "[PayloadMass1D] Nominal momentum rate must be finite.");
  }
  const scalar_t totalMass = estimatedTotalMass(
      nominalMass, payloadMassEstimate, minimumEstimatedTotalMass);
  const scalar_t scale = nominalMass / totalMass;
  vector6_t normalizedGravity = vector6_t::Zero();
  normalizedGravity.z() = -9.81;
  return (scale - 1.0) *
         (nominalNormalizedMomentumRate - normalizedGravity);
}

}  // namespace adaptive
}  // namespace legged_robot_adaptive
}  // namespace ocs2
