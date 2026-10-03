#include "legged_interface/adaptive/MyRbfTrackingError.h"

#include <cmath>
#include <stdexcept>

namespace legged {
namespace adaptive {
namespace {

quaternion_t quaternionFromEulerZyx(const vector3_t& eulerZyx) {
  const Eigen::AngleAxis<scalar_t> yaw(eulerZyx(0), vector3_t::UnitZ());
  const Eigen::AngleAxis<scalar_t> pitch(eulerZyx(1), vector3_t::UnitY());
  const Eigen::AngleAxis<scalar_t> roll(eulerZyx(2), vector3_t::UnitX());
  return quaternion_t(yaw * pitch * roll).normalized();
}

vector3_t quaternionTrackingError(quaternion_t actual,
                                  quaternion_t desired) {
  actual.normalize();
  desired.normalize();
  if (actual.coeffs().dot(desired.coeffs()) < 0.0) {
    desired.coeffs() *= -1.0;
  }
  return actual.w() * desired.vec() - desired.w() * actual.vec() -
         desired.vec().cross(actual.vec());
}

}  // namespace

vector6_t computeMyRbfTrackingError(
    const vector_t& actualRbdState, const vector_t& desiredRbdState,
    Eigen::Index generalizedCoordinatesNum, scalar_t lambdaLinear,
    scalar_t lambdaAngular) {
  const Eigen::Index expectedSize = 2 * generalizedCoordinatesNum;
  if (generalizedCoordinatesNum < 6 || actualRbdState.size() != expectedSize ||
      desiredRbdState.size() != expectedSize) {
    throw std::invalid_argument(
        "[MyRbfTrackingError] RBD state dimensions are inconsistent.");
  }
  if (!actualRbdState.allFinite() || !desiredRbdState.allFinite() ||
      !std::isfinite(lambdaLinear) || !std::isfinite(lambdaAngular) ||
      lambdaLinear < 0.0 || lambdaAngular < 0.0) {
    throw std::invalid_argument(
        "[MyRbfTrackingError] Inputs and gains must be finite and gains nonnegative.");
  }

  vector6_t sigma;
  const vector3_t actualPosition = actualRbdState.segment<3>(3);
  const vector3_t desiredPosition = desiredRbdState.segment<3>(3);
  const vector3_t actualAngularVelocity =
      actualRbdState.segment<3>(generalizedCoordinatesNum);
  const vector3_t desiredAngularVelocity =
      desiredRbdState.segment<3>(generalizedCoordinatesNum);
  const vector3_t actualLinearVelocity =
      actualRbdState.segment<3>(generalizedCoordinatesNum + 3);
  const vector3_t desiredLinearVelocity =
      desiredRbdState.segment<3>(generalizedCoordinatesNum + 3);

  sigma.head<3>() = desiredLinearVelocity - actualLinearVelocity +
                    lambdaLinear * (desiredPosition - actualPosition);
  sigma.tail<3>() =
      desiredAngularVelocity - actualAngularVelocity +
      lambdaAngular * quaternionTrackingError(
                          quaternionFromEulerZyx(actualRbdState.head<3>()),
                          quaternionFromEulerZyx(desiredRbdState.head<3>()));
  return sigma;
}

}  // namespace adaptive
}  // namespace legged
