#include <gtest/gtest.h>

#include <limits>

#include "ocs2_legged_robot_adaptive/adaptive/RbfWrenchInputFeature.h"

namespace ocs2 { namespace legged_robot_adaptive { namespace adaptive { namespace {

RbfWrenchInputFeature makeFeature() {
  vector6_t scale;
  scale << 18.4014769, 18.7575202, 17.5344826, 2.3370077, 5.4660416, 1.7261894;
  return RbfWrenchInputFeature(scale);
}

TEST(RbfWrenchInputFeature, ZeroResidualAndEstimateGiveZeroFeature) {
  const auto result = makeFeature().compute(vector6_t::Zero(), vector6_t::Zero());
  EXPECT_TRUE(result.uncompensatedDiscrepancy.isZero());
  EXPECT_TRUE(result.chi.isZero());
}
TEST(RbfWrenchInputFeature, FxResidualWithoutPreviousEstimate) {
  vector6_t residual = vector6_t::Zero(); residual(0) = 20.;
  EXPECT_DOUBLE_EQ(makeFeature().compute(residual, vector6_t::Zero()).uncompensatedDiscrepancy(0), 20.);
}
TEST(RbfWrenchInputFeature, FxPartialCompensation) {
  vector6_t residual = vector6_t::Zero(), previous = vector6_t::Zero(); residual(0) = 20.; previous(0) = 15.;
  EXPECT_DOUBLE_EQ(makeFeature().compute(residual, previous).uncompensatedDiscrepancy(0), 5.);
}
TEST(RbfWrenchInputFeature, FxExactCompensation) {
  vector6_t residual = vector6_t::Zero(), previous = vector6_t::Zero(); residual(0) = previous(0) = 20.;
  EXPECT_TRUE(makeFeature().compute(residual, previous).uncompensatedDiscrepancy.isZero());
}
TEST(RbfWrenchInputFeature, FxOvercompensationChangesDiscrepancySign) {
  vector6_t residual = vector6_t::Zero(), previous = vector6_t::Zero(); residual(0) = 18.4; previous(0) = 20.;
  EXPECT_NEAR(makeFeature().compute(residual, previous).uncompensatedDiscrepancy(0), -1.6, 1e-14);
}
TEST(RbfWrenchInputFeature, MixedSixAxisDifference) {
  vector6_t residual, previous, expected;
  residual << 10., -5., 20., 1., -2., .5; previous << 7., -4., 15., .4, -1.5, .2;
  expected << 3., -1., 5., .6, -.5, .3;
  EXPECT_TRUE(makeFeature().computeUncompensatedDiscrepancy(residual, previous).isApprox(expected, 1e-14));
}
TEST(RbfWrenchInputFeature, NormalizeScaleToOne) {
  const auto feature = makeFeature();
  EXPECT_TRUE(feature.normalize(feature.discrepancyScale()).isApprox(vector6_t::Ones(), 1e-14));
}
TEST(RbfWrenchInputFeature, RejectsInvalidScales) {
  vector6_t scale = vector6_t::Ones(); scale(0) = 0.; EXPECT_THROW(RbfWrenchInputFeature{scale}, std::invalid_argument);
  scale.setOnes(); scale(0) = -1.; EXPECT_THROW(RbfWrenchInputFeature{scale}, std::invalid_argument);
  scale.setOnes(); scale(0) = std::numeric_limits<scalar_t>::quiet_NaN(); EXPECT_THROW(RbfWrenchInputFeature{scale}, std::invalid_argument);
  scale.setOnes(); scale(0) = std::numeric_limits<scalar_t>::infinity(); EXPECT_THROW(RbfWrenchInputFeature{scale}, std::invalid_argument);
}
TEST(RbfWrenchInputFeature, RejectsNonFiniteInputs) {
  vector6_t invalid = vector6_t::Zero(); invalid(0) = std::numeric_limits<scalar_t>::quiet_NaN();
  EXPECT_THROW(makeFeature().compute(invalid, vector6_t::Zero()), std::invalid_argument);
  invalid(0) = std::numeric_limits<scalar_t>::infinity();
  EXPECT_THROW(makeFeature().compute(vector6_t::Zero(), invalid), std::invalid_argument);
}

}}}}  // namespace ocs2::legged_robot_adaptive::adaptive
