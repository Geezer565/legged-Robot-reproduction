#include <gtest/gtest.h>

#include "legged_interface/adaptive/MyRbfWrenchCompensation.h"

namespace legged {
namespace adaptive {
namespace {

TEST(MyRbfWrenchCompensation,
     LimitsEveryAxisByMagnitudeAndSlewRate) {
  const vector6_t previous = vector6_t::Zero();
  const vector6_t estimate =
      (vector6_t() << -20.0, 20.0, -30.0, 4.0, -5.0, 6.0).finished();
  const vector6_t gain = vector6_t::Constant(0.5);
  const vector6_t magnitude =
      (vector6_t() << 8.0, 8.0, 8.0, 1.0, 1.0, 1.0).finished();
  const vector6_t slew =
      (vector6_t() << 10.0, 10.0, 10.0, 2.0, 2.0, 2.0).finished();

  const vector6_t actual = computeLimitedWrenchCompensation(
      previous, estimate, gain, magnitude, slew, 0.1);

  const vector6_t expected =
      (vector6_t() << -1.0, 1.0, -1.0, 0.2, -0.2, 0.2).finished();
  EXPECT_TRUE(actual.isApprox(expected, 1e-12));
}

TEST(MyRbfWrenchCompensation,
     AllocatesForceWithoutCreatingAnUnrequestedBodyTorque) {
  WrenchAllocationInput input;
  input.contactPositions = {
      (vector3_t() << 0.30, 0.20, -0.30).finished(),
      (vector3_t() << -0.30, 0.20, -0.30).finished(),
      (vector3_t() << 0.30, -0.20, -0.30).finished(),
      (vector3_t() << -0.30, -0.20, -0.30).finished()};
  input.nominalForces.fill((vector3_t() << 0.0, 0.0, 50.0).finished());
  input.contacts = {true, true, true, true};
  input.requestedWrench(1) = -8.0;
  input.frictionCoefficient = 0.7;
  input.minimumNormalForce = 2.0;
  input.maximumForceDelta = 20.0;
  input.damping = 1e-8;

  const auto result = allocateBodyWrenchToContacts(input);

  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.scale, 1.0, 1e-8);
  EXPECT_TRUE(result.achievedWrench.isApprox(input.requestedWrench, 1e-6));
  EXPECT_NEAR(result.achievedWrench.tail<3>().norm(), 0.0, 1e-6);
}

TEST(MyRbfWrenchCompensation,
     AllocatesWrenchAboutAnExplicitWorldReferencePoint) {
  WrenchAllocationInput input;
  const vector3_t reference =
      (vector3_t() << 1.20, -0.70, 0.45).finished();
  input.wrenchReferencePosition = reference;
  input.contactPositions = {
      reference + (vector3_t() << 0.30, 0.20, -0.30).finished(),
      reference + (vector3_t() << -0.30, 0.20, -0.30).finished(),
      reference + (vector3_t() << 0.30, -0.20, -0.30).finished(),
      reference + (vector3_t() << -0.30, -0.20, -0.30).finished()};
  input.nominalForces.fill((vector3_t() << 0.0, 0.0, 50.0).finished());
  input.contacts = {true, true, true, true};
  input.requestedWrench(1) = -8.0;
  input.frictionCoefficient = 0.7;
  input.minimumNormalForce = 2.0;
  input.maximumForceDelta = 20.0;
  input.damping = 1e-8;

  const auto result = allocateBodyWrenchToContacts(input);

  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.achievedWrench.isApprox(input.requestedWrench, 1e-6));
  EXPECT_NEAR(result.achievedWrench.tail<3>().norm(), 0.0, 1e-6);
}

TEST(MyRbfWrenchCompensation,
     LeavesSwingFeetUntouchedAndScalesUnsafeRequests) {
  WrenchAllocationInput input;
  input.contactPositions = {
      (vector3_t() << 0.30, 0.20, -0.30).finished(),
      (vector3_t() << -0.30, 0.20, -0.30).finished(),
      (vector3_t() << 0.30, -0.20, -0.30).finished(),
      (vector3_t() << -0.30, -0.20, -0.30).finished()};
  input.nominalForces.fill((vector3_t() << 0.0, 0.0, 30.0).finished());
  input.contacts = {true, false, true, false};
  input.requestedWrench(0) = 100.0;
  input.frictionCoefficient = 0.3;
  input.minimumNormalForce = 2.0;
  input.maximumForceDelta = 8.0;
  input.damping = 1e-8;

  const auto result = allocateBodyWrenchToContacts(input);

  ASSERT_TRUE(result.valid);
  EXPECT_LT(result.scale, 1.0);
  EXPECT_TRUE(result.forceDeltas[1].isZero(1e-12));
  EXPECT_TRUE(result.forceDeltas[3].isZero(1e-12));
  for (std::size_t index : {0U, 2U}) {
    const vector3_t finalForce =
        input.nominalForces[index] + result.forceDeltas[index];
    EXPECT_GE(finalForce(2), input.minimumNormalForce - 1e-9);
    EXPECT_LE(finalForce.head<2>().norm(),
              input.frictionCoefficient * finalForce(2) + 1e-9);
    EXPECT_LE(result.forceDeltas[index].norm(),
              input.maximumForceDelta + 1e-9);
  }
}

TEST(MyRbfWrenchCompensation, RejectsInvalidSafetyParameters) {
  WrenchAllocationInput input;
  input.contacts = {true, true, true, true};
  input.frictionCoefficient = -0.1;
  EXPECT_THROW(allocateBodyWrenchToContacts(input), std::invalid_argument);
}

}  // namespace
}  // namespace adaptive
}  // namespace legged
