#include <gtest/gtest.h>

#include <limits>

#include "legged_interface/adaptive/MyRbfTrackingError.h"

namespace legged {
namespace adaptive {
namespace {

constexpr Eigen::Index kGeneralizedCoordinates = 18;

TEST(MyRbfTrackingError, ComputesDesiredMinusActualLinearSlidingVariable) {
  vector_t actual = vector_t::Zero(2 * kGeneralizedCoordinates);
  vector_t desired = vector_t::Zero(2 * kGeneralizedCoordinates);
  actual.segment<3>(3) << 0.2, -0.1, 0.4;
  desired.segment<3>(3) << 0.5, -0.3, 0.6;
  actual.segment<3>(kGeneralizedCoordinates + 3) << 0.1, 0.2, -0.3;
  desired.segment<3>(kGeneralizedCoordinates + 3) << 0.4, -0.1, 0.2;

  const auto sigma = computeMyRbfTrackingError(
      actual, desired, kGeneralizedCoordinates, 2.0, 1.5);

  const vector3_t expected =
      (vector3_t() << 0.9, -0.7, 0.9).finished();
  EXPECT_TRUE(sigma.head<3>().isApprox(expected, 1e-12));
  EXPECT_TRUE(sigma.tail<3>().isZero(1e-12));
}

TEST(MyRbfTrackingError, UsesQuaternionOrientationErrorAndWorldAngularRate) {
  vector_t actual = vector_t::Zero(2 * kGeneralizedCoordinates);
  vector_t desired = vector_t::Zero(2 * kGeneralizedCoordinates);
  desired(0) = 0.2;  // Euler ZYX yaw.
  actual.segment<3>(kGeneralizedCoordinates) << 0.1, -0.2, 0.3;
  desired.segment<3>(kGeneralizedCoordinates) << 0.3, -0.1, 0.5;

  const auto sigma = computeMyRbfTrackingError(
      actual, desired, kGeneralizedCoordinates, 2.0, 1.5);

  const vector3_t expected =
      (vector3_t() << 0.2, 0.1, 0.2 + 1.5 * std::sin(0.1)).finished();
  EXPECT_TRUE(sigma.tail<3>().isApprox(expected, 1e-12));
}

TEST(MyRbfTrackingError, RejectsInvalidInputs) {
  vector_t valid = vector_t::Zero(2 * kGeneralizedCoordinates);
  EXPECT_THROW(computeMyRbfTrackingError(
                   valid.head(valid.size() - 1), valid,
                   kGeneralizedCoordinates, 2.0, 1.5),
               std::invalid_argument);
  valid(0) = std::numeric_limits<scalar_t>::quiet_NaN();
  EXPECT_THROW(computeMyRbfTrackingError(
                   valid, vector_t::Zero(2 * kGeneralizedCoordinates),
                   kGeneralizedCoordinates, 2.0, 1.5),
               std::invalid_argument);
  EXPECT_THROW(computeMyRbfTrackingError(
                   vector_t::Zero(2 * kGeneralizedCoordinates),
                   vector_t::Zero(2 * kGeneralizedCoordinates),
                   kGeneralizedCoordinates, -1.0, 1.5),
               std::invalid_argument);
}

}  // namespace
}  // namespace adaptive
}  // namespace legged
