#include <gtest/gtest.h>

#include "legged_interface/adaptive/MyRbfFyCompensation.h"

namespace legged {
namespace adaptive {
namespace {

TEST(MyRbfFyCompensation, AppliesScaleMagnitudeLimitAndRateLimit) {
  EXPECT_NEAR(computeLimitedFyCompensation(
                  0.0, -20.0, 0.1, 3.0, 5.0, 0.1),
              -0.5, 1e-12);
  EXPECT_NEAR(computeLimitedFyCompensation(
                  -1.9, -100.0, 0.1, 3.0, 5.0, 0.1),
              -2.4, 1e-12);
  EXPECT_NEAR(computeLimitedFyCompensation(
                  -2.9, -100.0, 0.1, 3.0, 50.0, 0.1),
              -3.0, 1e-12);
}

TEST(MyRbfFyCompensation, RejectsUnsafeParameters) {
  EXPECT_THROW(computeLimitedFyCompensation(
                   0.0, 1.0, -0.1, 3.0, 5.0, 0.1),
               std::invalid_argument);
  EXPECT_THROW(computeLimitedFyCompensation(
                   0.0, 1.0, 0.1, 0.0, 5.0, 0.1),
               std::invalid_argument);
  EXPECT_THROW(computeLimitedFyCompensation(
                   0.0, 1.0, 0.1, 3.0, 5.0, -0.1),
               std::invalid_argument);
}

}  // namespace
}  // namespace adaptive
}  // namespace legged
