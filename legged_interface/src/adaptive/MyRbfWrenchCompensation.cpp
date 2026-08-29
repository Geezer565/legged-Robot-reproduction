#include "legged_interface/adaptive/MyRbfWrenchCompensation.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

#include <Eigen/Cholesky>

namespace legged {
namespace adaptive {
namespace {

Eigen::Matrix<scalar_t, 3, 3> skew(const vector3_t& value) {
  Eigen::Matrix<scalar_t, 3, 3> result;
  result << 0.0, -value(2), value(1),
      value(2), 0.0, -value(0),
      -value(1), value(0), 0.0;
  return result;
}

bool isFeasible(const WrenchAllocationInput& input,
                const std::array<vector3_t, kMyRbfContactCount>& deltas,
                scalar_t scale) {
  constexpr scalar_t tolerance = 1e-9;
  for (std::size_t index = 0; index < kMyRbfContactCount; ++index) {
    if (!input.contacts[index]) {
      continue;
    }
    const vector3_t delta = scale * deltas[index];
    const vector3_t finalForce = input.nominalForces[index] + delta;
    if (delta.norm() > input.maximumForceDelta + tolerance ||
        finalForce(2) < input.minimumNormalForce - tolerance ||
        finalForce.head<2>().norm() >
            input.frictionCoefficient * finalForce(2) + tolerance) {
      return false;
    }
  }
  return true;
}

}  // namespace

vector6_t computeLimitedWrenchCompensation(
    const vector6_t& previousCompensation, const vector6_t& estimatedWrench,
    const vector6_t& controlGain, const vector6_t& magnitudeLimit,
    const vector6_t& rateLimit, scalar_t dt) {
  if (!previousCompensation.allFinite() || !estimatedWrench.allFinite() ||
      !controlGain.allFinite() || !magnitudeLimit.allFinite() ||
      !rateLimit.allFinite() || !std::isfinite(dt) || dt <= 0.0 ||
      (controlGain.array() < 0.0).any() ||
      (magnitudeLimit.array() <= 0.0).any() ||
      (rateLimit.array() <= 0.0).any()) {
    throw std::invalid_argument(
        "[MyRBF wrench] compensation limits must be finite and positive.");
  }

  vector6_t result;
  for (Eigen::Index axis = 0; axis < 6; ++axis) {
    const scalar_t target = std::max(
        -magnitudeLimit(axis),
        std::min(magnitudeLimit(axis),
                 controlGain(axis) * estimatedWrench(axis)));
    const scalar_t maximumStep = rateLimit(axis) * dt;
    const scalar_t step = std::max(
        -maximumStep,
        std::min(maximumStep, target - previousCompensation(axis)));
    result(axis) = previousCompensation(axis) + step;
  }
  return result;
}

WrenchAllocationResult allocateBodyWrenchToContacts(
    const WrenchAllocationInput& input) {
  if (!input.requestedWrench.allFinite() ||
      !std::isfinite(input.frictionCoefficient) ||
      input.frictionCoefficient <= 0.0 ||
      !std::isfinite(input.minimumNormalForce) ||
      input.minimumNormalForce < 0.0 ||
      !std::isfinite(input.maximumForceDelta) ||
      input.maximumForceDelta <= 0.0 || !std::isfinite(input.damping) ||
      input.damping <= 0.0) {
    throw std::invalid_argument(
        "[MyRBF wrench] allocator safety parameters are invalid.");
  }

  WrenchAllocationResult result;
  for (auto& delta : result.forceDeltas) {
    delta.setZero();
  }

  std::vector<std::size_t> activeContacts;
  for (std::size_t index = 0; index < kMyRbfContactCount; ++index) {
    if (!input.contactPositions[index].allFinite() ||
        !input.nominalForces[index].allFinite()) {
      throw std::invalid_argument(
          "[MyRBF wrench] contact data must be finite.");
    }
    if (input.contacts[index]) {
      activeContacts.push_back(index);
    }
  }
  if (activeContacts.empty()) {
    return result;
  }

  matrix_t allocation = matrix_t::Zero(
      6, 3 * static_cast<Eigen::Index>(activeContacts.size()));
  for (std::size_t column = 0; column < activeContacts.size(); ++column) {
    const Eigen::Index start = 3 * static_cast<Eigen::Index>(column);
    allocation.block<3, 3>(0, start).setIdentity();
    allocation.block<3, 3>(3, start) =
        skew(input.contactPositions[activeContacts[column]]);
  }
  const Eigen::Matrix<scalar_t, 6, 6> regularized =
      allocation * allocation.transpose() +
      input.damping * Eigen::Matrix<scalar_t, 6, 6>::Identity();
  const vector_t stackedDeltas =
      allocation.transpose() * regularized.ldlt().solve(input.requestedWrench);
  if (!stackedDeltas.allFinite()) {
    return result;
  }
  for (std::size_t column = 0; column < activeContacts.size(); ++column) {
    result.forceDeltas[activeContacts[column]] =
        stackedDeltas.segment<3>(3 * static_cast<Eigen::Index>(column));
  }

  scalar_t scale = 1.0;
  if (!isFeasible(input, result.forceDeltas, scale)) {
    scalar_t lower = 0.0;
    scalar_t upper = 1.0;
    for (int iteration = 0; iteration < 60; ++iteration) {
      const scalar_t candidate = 0.5 * (lower + upper);
      if (isFeasible(input, result.forceDeltas, candidate)) {
        lower = candidate;
      } else {
        upper = candidate;
      }
    }
    scale = lower;
  }

  result.achievedWrench.setZero();
  for (std::size_t index = 0; index < kMyRbfContactCount; ++index) {
    result.forceDeltas[index] *= scale;
    if (!input.contacts[index]) {
      continue;
    }
    result.achievedWrench.head<3>() += result.forceDeltas[index];
    result.achievedWrench.tail<3>() +=
        input.contactPositions[index].cross(result.forceDeltas[index]);
  }
  result.scale = scale;
  result.valid = true;
  return result;
}

}  // namespace adaptive
}  // namespace legged
