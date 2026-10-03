#include <cmath>
#include <limits>
#include <gtest/gtest.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>
#include "legged_controllers/MyRbfV2PolicyBinding.h"

namespace legged {
namespace {

class MyRbfV2PolicyBindingTest : public ::testing::Test {
 protected:
  std::shared_ptr<adaptive::MyRbfV2SnapshotStore> store = std::make_shared<adaptive::MyRbfV2SnapshotStore>();
  MyRbfV2PolicyBinding binding{store};
  ocs2::ReferenceManager reference;
  ocs2::PrimalSolution policy;

  void publish(std::uint64_t sequence, double force) {
    adaptive::MyRbfV2Snapshot value;
    value.valid = true;
    value.sequenceId = sequence;
    value.timestamp = 0.25 * sequence;
    value.externalWrenchEstimate[0] = force;
    ASSERT_TRUE(store->tryPublish(value));
  }
  void solve(double time) {
    binding.preSolverRun(time, time + 1., ocs2::vector_t::Zero(24), reference);
    binding.postSolverRun(policy);
  }
  ocs2::CommandData command(double time) {
    ocs2::CommandData result;
    result.mpcInitObservation_.time = time;
    return result;
  }
  void buffer(double time) { binding.modifyBufferedSolution(command(time), policy); }
  void activate(double time) { binding.modifyActiveSolution(command(time), policy); }
};

TEST_F(MyRbfV2PolicyBindingTest, StartsEmptyAndBindsAbsentSnapshotWithoutInventingEstimate) {
  EXPECT_FALSE(binding.active());
  EXPECT_EQ(binding.activeSolveId(), 0u);
  solve(1.);
  buffer(1.);
  EXPECT_FALSE(binding.active());
  activate(1.);
  ASSERT_TRUE(binding.active());
  EXPECT_FALSE(binding.active()->valid);
  EXPECT_EQ(binding.active()->sequenceId, 0u);
  EXPECT_EQ(binding.activeSolveId(), 1u);
}

TEST_F(MyRbfV2PolicyBindingTest, V11ClfUsesTheReferenceOfTheCurrentSolve) {
  adaptive::MyRbfV2Snapshot value;
  value.valid = value.v11Enabled = true;
  value.v11Target = ocs2::TargetTrajectories({0.}, {ocs2::vector_t::Zero(24)}, {ocs2::vector_t::Zero(24)});
  ASSERT_TRUE(store->tryPublish(value));
  const auto target = ocs2::TargetTrajectories({1.}, {ocs2::vector_t::Ones(24)}, {ocs2::vector_t::Zero(24)});
  reference.setTargetTrajectories(target);
  reference.preSolverRun(1., 2., ocs2::vector_t::Zero(24));
  solve(1.);
  EXPECT_TRUE(store->frozenForSolver->v11Target.stateTrajectory.front().isApprox(target.stateTrajectory.front()));
  buffer(1.);
  activate(1.);
  EXPECT_TRUE(binding.active()->v11Target.stateTrajectory.front().isApprox(target.stateTrajectory.front()));
}

TEST_F(MyRbfV2PolicyBindingTest, PendingPublicationCannotChangeSolvedOrBufferedSnapshot) {
  publish(101, 12.);
  solve(1.);
  const auto frozen = store->frozenForSolver;
  publish(202, -45.);
  buffer(1.);
  // A subsequent solve may start before MRT adopts the previous policy.
  solve(2.);
  ASSERT_EQ(store->frozenForSolver->sequenceId, 202u);
  activate(1.);
  ASSERT_EQ(binding.active(), frozen);
  EXPECT_EQ(binding.active()->sequenceId, 101u);
  EXPECT_EQ(binding.active()->externalWrenchEstimate[0], 12.);
  EXPECT_EQ(binding.activeSolveId(), 1u);
  buffer(2.);
  EXPECT_EQ(binding.active(), frozen);
  EXPECT_EQ(binding.activeSolveId(), 1u);
  activate(2.);
  EXPECT_EQ(binding.active()->sequenceId, 202u);
  EXPECT_EQ(binding.activeSolveId(), 2u);
  EXPECT_EQ(frozen->externalWrenchEstimate[0], 12.);
}

TEST_F(MyRbfV2PolicyBindingTest, LatestBufferedPolicyWinsWhileActiveRemainsUnchanged) {
  publish(10, 10.); solve(1.); buffer(1.); activate(1.);
  const auto active = binding.active();
  publish(20, 20.); solve(2.); buffer(2.);
  publish(30, 30.); solve(3.); buffer(3.);
  EXPECT_EQ(binding.active(), active);
  EXPECT_EQ(binding.activeSolveId(), 1u);
  activate(3.);
  ASSERT_TRUE(binding.active());
  EXPECT_EQ(binding.active()->sequenceId, 30u);
  EXPECT_EQ(binding.activeSolveId(), 3u);
}

TEST_F(MyRbfV2PolicyBindingTest, StaleOrFutureCommandIsRejectedWithoutCorruptingBuffer) {
  publish(10, 10.); solve(1.); buffer(1.); activate(1.);
  const auto old = binding.active();
  publish(20, 20.); solve(2.);
  EXPECT_THROW(buffer(1.), std::runtime_error);
  EXPECT_THROW(buffer(3.), std::runtime_error);
  activate(1.);
  EXPECT_EQ(binding.active(), old);
  EXPECT_EQ(binding.activeSolveId(), 1u);
  buffer(2.); activate(2.);
  EXPECT_EQ(binding.active()->sequenceId, 20u);
  EXPECT_EQ(binding.activeSolveId(), 2u);
}

TEST_F(MyRbfV2PolicyBindingTest, TimestampToleranceAndSolveIdsAreIndependentOfLearnerIds) {
  publish(900, 4.); solve(5.);
  EXPECT_NO_THROW(buffer(5. + 0.5e-9));
  EXPECT_THROW(buffer(5. + 2.e-9), std::runtime_error);
  EXPECT_THROW(buffer(5. - 2.e-9), std::runtime_error);
  activate(5.);
  EXPECT_EQ(binding.activeSolveId(), 1u);
  EXPECT_EQ(binding.active()->sequenceId, 900u);
  // Re-solving at the same time without a learner update is legitimate.
  solve(5.); buffer(5.); activate(5.);
  EXPECT_EQ(binding.activeSolveId(), 2u);
  EXPECT_EQ(binding.active()->sequenceId, 900u);
}

TEST_F(MyRbfV2PolicyBindingTest, NonFiniteCommandTimeCannotPassStaleDetection) {
  publish(10, 10.); solve(1.); buffer(1.); activate(1.);
  const auto old = binding.active();
  publish(20, 20.); solve(2.);
  EXPECT_THROW(buffer(std::numeric_limits<double>::quiet_NaN()), std::runtime_error);
  EXPECT_THROW(buffer(std::numeric_limits<double>::infinity()), std::runtime_error);
  EXPECT_THROW(buffer(-std::numeric_limits<double>::infinity()), std::runtime_error);
  activate(1.);
  EXPECT_EQ(binding.active(), old);
  EXPECT_EQ(binding.activeSolveId(), 1u);
}

TEST_F(MyRbfV2PolicyBindingTest, QuiescentResetClearsActiveAndBufferedPolicyAndRestartsSolveIds) {
  publish(10, 10.); solve(1.); buffer(1.); activate(1.);
  const auto oldActive = binding.active();
  publish(20, 20.); solve(2.); buffer(2.);
  ASSERT_EQ(binding.activeSolveId(), 1u);
  ASSERT_EQ(binding.active(), oldActive);

  // No solver or MRT worker runs here: resetQuiescent's lifecycle precondition.
  binding.resetQuiescent();
  EXPECT_FALSE(binding.active());
  EXPECT_EQ(binding.activeSolveId(), 0u);
  // Exercise the otherwise private buffer through its adoption callback.
  // Neither the second solve's snapshot nor its solve ID may reappear.
  activate(2.);
  EXPECT_FALSE(binding.active());
  EXPECT_EQ(binding.activeSolveId(), 0u);
  EXPECT_THROW(buffer(2.), std::runtime_error);
  binding.resetQuiescent();  // Repeated reset must also leave an empty binding.
  EXPECT_FALSE(binding.active());

  // Startup publishes a fresh initial snapshot before starting a fresh solve.
  publish(30, -4.); solve(3.); buffer(3.);
  EXPECT_FALSE(binding.active());
  activate(3.);
  ASSERT_TRUE(binding.active());
  EXPECT_EQ(binding.activeSolveId(), 1u);
  EXPECT_EQ(binding.active()->sequenceId, 30u);
  EXPECT_EQ(binding.active()->externalWrenchEstimate[0], -4.);
  ASSERT_TRUE(oldActive);
  EXPECT_EQ(oldActive->sequenceId, 10u);
  EXPECT_EQ(oldActive->externalWrenchEstimate[0], 10.);
}

TEST_F(MyRbfV2PolicyBindingTest, NonFiniteSolveStartCannotBindFinitePolicy) {
  publish(10, 10.);
  EXPECT_THROW({ solve(std::numeric_limits<double>::quiet_NaN()); buffer(2.); }, std::exception);
  EXPECT_FALSE(binding.active());
  EXPECT_EQ(binding.activeSolveId(), 0u);
}

TEST_F(MyRbfV2PolicyBindingTest, PublicationOwnsMetadataAndArraysAndOldSnapshotSurvivesNextFreeze) {
  adaptive::MyRbfV2Snapshot value;
  value.valid = true; value.sequenceId = 42; value.timestamp = 3.5; value.contactMode = 9;
  value.externalWrenchEstimate[3] = -17.; value.robustBound[2] = 5.;
  value.weights[4][7] = .625; value.feature[1][8] = -.875;
  ASSERT_TRUE(store->tryPublish(value));
  value.sequenceId = 43; value.weights[4][7] = 99.; value.externalWrenchEstimate[3] = 300.;
  solve(4.); buffer(4.); activate(4.);
  const auto first = binding.active();
  ASSERT_TRUE(first);
  EXPECT_EQ(first->sequenceId, 42u); EXPECT_EQ(first->timestamp, 3.5); EXPECT_EQ(first->contactMode, 9u);
  EXPECT_EQ(first->externalWrenchEstimate[3], -17.); EXPECT_EQ(first->robustBound[2], 5.);
  EXPECT_EQ(first->weights[4][7], .625); EXPECT_EQ(first->feature[1][8], -.875);
  ASSERT_TRUE(store->tryPublish(value));
  solve(5.); buffer(5.); activate(5.);
  EXPECT_EQ(binding.active()->sequenceId, 43u);
  EXPECT_EQ(first->sequenceId, 42u); EXPECT_EQ(first->weights[4][7], .625);
}

TEST_F(MyRbfV2PolicyBindingTest, SamplesPreviousPolicyAndItsPairedTargetForTeacherErrors) {
  publish(1, 0.);
  policy.timeTrajectory_ = {1., 2., 3.};
  policy.stateTrajectory_ = {
      ocs2::vector_t::Constant(24, 0.),
      ocs2::vector_t::Constant(24, 2.),
      ocs2::vector_t::Constant(24, 4.)};
  for (auto& state : policy.stateTrajectory_) state.segment<3>(9).setZero();
  auto cmd = command(1.);
  ocs2::vector_array_t targets = {
      ocs2::vector_t::Constant(24, .5),
      ocs2::vector_t::Constant(24, 1.5),
      ocs2::vector_t::Constant(24, 2.5)};
  for (auto& state : targets) state.segment<3>(9).setZero();
  cmd.mpcTargetTrajectories_ = ocs2::TargetTrajectories({1., 2., 3.}, targets);
  binding.preSolverRun(1., 3., ocs2::vector_t::Zero(24), reference);
  binding.postSolverRun(policy);
  binding.modifyBufferedSolution(cmd, policy);
  binding.modifyActiveSolution(cmd, policy);

  ocs2::vector_t actual = ocs2::vector_t::Constant(24, 3.);
  actual.segment<3>(9).setZero();
  ocs2::vector_t predictionError, trackingError;
  ASSERT_TRUE(binding.sampleTeacherErrors(1.5, actual, predictionError,
                                          trackingError));
  ocs2::vector_t expected = ocs2::vector_t::Constant(24, 2.);
  expected.segment<3>(9).setZero();
  EXPECT_TRUE(predictionError.isApprox(expected));
  EXPECT_TRUE(trackingError.isApprox(expected));
  EXPECT_FALSE(binding.sampleTeacherErrors(.9, actual, predictionError,
                                           trackingError));
  EXPECT_FALSE(binding.sampleTeacherErrors(3.1, actual, predictionError,
                                           trackingError));
}

TEST_F(MyRbfV2PolicyBindingTest, UsesCompleteStateAndWorldRotationBoxMinusForTeacherErrors) {
  publish(1, 0.);
  ocs2::vector_t predicted = ocs2::vector_t::Zero(24);
  ocs2::vector_t target = ocs2::vector_t::Zero(24);
  predicted(9) = M_PI - 0.05;
  target(9) = M_PI - 0.05;
  policy.timeTrajectory_ = {1., 3.};
  policy.stateTrajectory_ = {predicted, predicted};
  auto cmd = command(1.);
  cmd.mpcTargetTrajectories_ = ocs2::TargetTrajectories(
      {1., 3.}, {target, target});
  binding.preSolverRun(1., 3., ocs2::vector_t::Zero(24), reference);
  binding.postSolverRun(policy);
  binding.modifyBufferedSolution(cmd, policy);
  binding.modifyActiveSolution(cmd, policy);

  ocs2::vector_t actual = ocs2::vector_t::Zero(24);
  actual.head<6>() << 1., 2., 3., 4., 5., 6.;
  actual.segment<3>(6) << .2, -.3, .4;
  actual(9) = -M_PI + 0.05;
  for (int i = 12; i < 24; ++i) actual(i) = .1 * i;

  ocs2::vector_t predictionError, trackingError;
  ASSERT_TRUE(binding.sampleTeacherErrors(2., actual, predictionError,
                                          trackingError));
  ASSERT_EQ(predictionError.size(), 24);
  ASSERT_EQ(trackingError.size(), 24);
  EXPECT_TRUE(predictionError.head<6>().isApprox(actual.head<6>(), 1e-12));
  EXPECT_TRUE(predictionError.segment<3>(6).isApprox(actual.segment<3>(6), 1e-12));
  Eigen::Vector3d expectedRotation(0., 0., .1);
  EXPECT_TRUE(predictionError.segment<3>(9).isApprox(expectedRotation, 1e-12));
  EXPECT_TRUE(predictionError.tail(12).isApprox(actual.tail(12), 1e-12));
  EXPECT_TRUE(trackingError.isApprox(predictionError, 1e-12));
}

TEST_F(MyRbfV2PolicyBindingTest, RejectsHybridDuplicateTimeForTeacherErrors) {
  publish(1, 0.);
  policy.timeTrajectory_ = {1., 2., 2., 3.};
  policy.stateTrajectory_ = {
      ocs2::vector_t::Zero(24), ocs2::vector_t::Ones(24),
      ocs2::vector_t::Constant(24, 2.), ocs2::vector_t::Constant(24, 3.)};
  auto cmd = command(1.);
  cmd.mpcTargetTrajectories_ = ocs2::TargetTrajectories(
      {1., 3.}, {ocs2::vector_t::Zero(24), ocs2::vector_t::Zero(24)});
  binding.preSolverRun(1., 3., ocs2::vector_t::Zero(24), reference);
  binding.postSolverRun(policy);
  binding.modifyBufferedSolution(cmd, policy);
  binding.modifyActiveSolution(cmd, policy);
  ocs2::vector_t predictionError, trackingError;
  EXPECT_FALSE(binding.sampleTeacherErrors(
      2., ocs2::vector_t::Zero(24), predictionError, trackingError));
}

}  // namespace
}  // namespace legged
