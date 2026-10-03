#pragma once

#include <array>

#include "legged_interface/adaptive/AdaptiveTypes.h"

namespace legged {
namespace adaptive {

constexpr std::size_t kMyRbfContactCount = 4;

struct WrenchAllocationInput {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // All vectors are expressed in world axes. requestedWrench is about
  // wrenchReferencePosition; contactPositions are absolute world positions.
  vector6_t requestedWrench = vector6_t::Zero();
  vector3_t wrenchReferencePosition = vector3_t::Zero();
  std::array<vector3_t, kMyRbfContactCount> contactPositions;
  std::array<vector3_t, kMyRbfContactCount> nominalForces;
  std::array<bool, kMyRbfContactCount> contacts{{false, false, false, false}};
  scalar_t frictionCoefficient = 0.5;
  scalar_t minimumNormalForce = 1.0;
  scalar_t maximumForceDelta = 20.0;
  scalar_t damping = 1e-6;
};

struct WrenchAllocationResult {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  bool valid = false;
  scalar_t scale = 0.0;
  std::array<vector3_t, kMyRbfContactCount> forceDeltas;
  vector6_t achievedWrench = vector6_t::Zero();
};

vector6_t computeLimitedWrenchCompensation(
    const vector6_t& previousCompensation, const vector6_t& estimatedWrench,
    const vector6_t& controlGain, const vector6_t& magnitudeLimit,
    const vector6_t& rateLimit, scalar_t dt);

WrenchAllocationResult allocateBodyWrenchToContacts(
    const WrenchAllocationInput& input);

}  // namespace adaptive
}  // namespace legged
