#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>

#include "legged_interface/adaptive/MyRbfPredictionResidualObserver.h"

namespace legged {
namespace adaptive {
namespace {

TEST(MyRbfPredictionResidualObserver,
     RejectsSamplesWithoutAnAlignedPreviousPrediction) {
  MyRbfPredictionResidualObserver observer(10.0, 24, true, 0.5, 1e-6);
  const vector_t actualState = vector_t::Zero(24);

  EXPECT_FALSE(observer.observe(1.0, actualState).valid);

  observer.setPrediction(1.1, 0.1, vector_t::Zero(24));
  EXPECT_FALSE(observer.observe(1.10001, actualState).valid);
}

TEST(MyRbfPredictionResidualObserver,
     ReconstructsWorldWrenchFromOneStepMomentumInnovation) {
  MyRbfPredictionResidualObserver observer(10.0, 24, false, 0.0, 1e-6);
  vector_t predictedState = vector_t::Zero(24);
  vector_t actualState = vector_t::Zero(24);
  actualState.head<6>() << 0.1, -0.2, 0.3, -0.04, 0.05, -0.06;
  observer.setPrediction(2.1, 0.1, predictedState);

  const auto result = observer.observe(2.1, actualState);

  ASSERT_TRUE(result.valid);
  const vector6_t expected =
      (vector6_t() << -10.0, 20.0, -30.0, 4.0, -5.0, 6.0).finished();
  EXPECT_TRUE(result.rawResidual.isApprox(expected, 1e-12));
  EXPECT_TRUE(result.filteredResidual.isApprox(expected, 1e-12));
  EXPECT_NEAR(result.dt, 0.1, 1e-12);
}

TEST(MyRbfPredictionResidualObserver,
     FiltersOnlyValidConsecutivePredictionResiduals) {
  MyRbfPredictionResidualObserver observer(10.0, 24, true, 0.5, 1e-6);
  vector_t actualState = vector_t::Zero(24);
  observer.setPrediction(1.1, 0.1, vector_t::Zero(24));
  actualState(0) = 0.1;
  const auto first = observer.observe(1.1, actualState);
  ASSERT_TRUE(first.valid);
  EXPECT_NEAR(first.filteredResidual(0), -10.0, 1e-12);

  observer.setPrediction(1.2, 0.1, vector_t::Zero(24));
  actualState(0) = 0.2;
  const auto second = observer.observe(1.2, actualState);
  ASSERT_TRUE(second.valid);
  EXPECT_NEAR(second.rawResidual(0), -20.0, 1e-12);
  EXPECT_NEAR(second.filteredResidual(0), -15.0, 1e-12);
}

TEST(MyRbfPredictionResidualObserver,
     RestoresExternalWrenchWhenThePreviousControlAppliedCompensation) {
  MyRbfPredictionResidualObserver observer(10.0, 24, false, 0.0, 1e-6);
  vector_t actualState = vector_t::Zero(24);
  actualState(1) = 0.16;
  vector6_t appliedCompensation = vector6_t::Zero();
  appliedCompensation(1) = -4.0;
  observer.setPrediction(3.1, 0.1, vector_t::Zero(24),
                         appliedCompensation);

  const auto result = observer.observe(3.1, actualState);

  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.netResidual(1), -16.0, 1e-12);
  EXPECT_NEAR(result.rawResidual(1), -20.0, 1e-12);
  EXPECT_NEAR(result.filteredResidual(1), -20.0, 1e-12);
}

TEST(MyRbfPredictionResidualObserver, RejectsUnsafeConfigurationAndData) {
  EXPECT_THROW(
      MyRbfPredictionResidualObserver(0.0, 24, true, 0.5, 1e-6),
      std::invalid_argument);
  EXPECT_THROW(
      MyRbfPredictionResidualObserver(10.0, 5, true, 0.5, 1e-6),
      std::invalid_argument);

  MyRbfPredictionResidualObserver observer(10.0, 24, true, 0.5, 1e-6);
  vector_t invalidState = vector_t::Zero(24);
  invalidState(0) = std::numeric_limits<scalar_t>::quiet_NaN();
  EXPECT_THROW(observer.setPrediction(1.1, 0.1, invalidState),
               std::invalid_argument);
}

}  // namespace
}  // namespace adaptive
}  // namespace legged
