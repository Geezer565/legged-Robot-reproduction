#include <gtest/gtest.h>

#include <boost/filesystem/path.hpp>
#include <memory>

#include <ocs2_legged_robot/LeggedRobotInterface.h>

#include "ocs2_legged_robot_adaptive/package_path.h"
#include "ocs2_legged_robot_adaptive/adaptive/WrenchResidualEstimator.h"

namespace ocs2 {
namespace legged_robot_adaptive {
namespace adaptive {
namespace {

constexpr scalar_t kRobotMass = 52.13485;

std::unique_ptr<legged_robot::LeggedRobotInterface> makeNominalInterface() {
  const boost::filesystem::path adaptivePath(getPath());
  const auto sourcePath = adaptivePath.parent_path();
  const auto leggedPath = sourcePath / "ocs2/ocs2_robotic_examples/ocs2_legged_robot";
  return std::make_unique<legged_robot::LeggedRobotInterface>(
      (leggedPath / "config/mpc/task.info").string(),
      (sourcePath / "ocs2_robotic_assets/resources/anymal_c/urdf/anymal.urdf").string(),
      (leggedPath / "config/command/reference.info").string());
}

TEST(WrenchResidualEstimator, ReconstructsSingleAxisKnownWrench) {
  vector6_t nominalRate;
  nominalRate << 0.2, -0.1, 0.5, 0.03, -0.02, 0.04;
  vector6_t knownWrench = vector6_t::Zero();
  knownWrench(0) = 20.0;
  const vector6_t observedRate = nominalRate - knownWrench / kRobotMass;
  EXPECT_TRUE(WrenchResidualEstimator::reconstructWrenchResidual(
                  kRobotMass, observedRate, nominalRate)
                  .isApprox(knownWrench, 1.0e-14));
}

TEST(WrenchResidualEstimator, ReconstructsAllWrenchAxes) {
  vector6_t nominalRate;
  nominalRate << 0.2, -0.1, 0.5, 0.03, -0.02, 0.04;
  vector6_t knownWrench;
  knownWrench << 10.0, -5.0, 30.0, 1.0, -2.0, 3.0;
  const vector6_t observedRate = nominalRate - knownWrench / kRobotMass;
  EXPECT_TRUE(WrenchResidualEstimator::reconstructWrenchResidual(
                  kRobotMass, observedRate, nominalRate)
                  .isApprox(knownWrench, 1.0e-14));
}

TEST(WrenchResidualEstimator, RejectsInvalidFormulaInputs) {
  const vector6_t zero = vector6_t::Zero();
  EXPECT_THROW(WrenchResidualEstimator::reconstructWrenchResidual(
                   0.0, zero, zero), std::invalid_argument);
}

TEST(WrenchResidualEstimator, FirstFrameIsInvalidThenSecondFrameIsValid) {
  auto interface = makeNominalInterface();
  const auto& info = interface->getCentroidalModelInfo();
  WrenchResidualEstimator estimator(interface->getPinocchioInterface(), info);
  const vector_t state = interface->getInitialState();
  const vector_t input = vector_t::Zero(info.inputDim);

  EXPECT_FALSE(estimator.update(0.0, state, input).valid);
  const auto second = estimator.update(0.01, state, input);
  EXPECT_TRUE(second.valid);
  EXPECT_NEAR(second.dt, 0.01, 1.0e-14);
}

TEST(WrenchResidualEstimator, InvalidDtDoesNotProduceResidual) {
  auto interface = makeNominalInterface();
  const auto& info = interface->getCentroidalModelInfo();
  WrenchResidualEstimator estimator(interface->getPinocchioInterface(), info);
  const vector_t state = interface->getInitialState();
  const vector_t input = vector_t::Zero(info.inputDim);

  EXPECT_FALSE(estimator.update(1.0, state, input).valid);
  EXPECT_FALSE(estimator.update(1.0, state, input).valid);
}

}  // namespace
}  // namespace adaptive
}  // namespace legged_robot_adaptive
}  // namespace ocs2
