#include <gtest/gtest.h>

#include <limits>

#include "ocs2_legged_robot_adaptive/adaptive/RbfWrenchInputFeature.h"
#include "ocs2_legged_robot_adaptive/adaptive/SixSubnetworkRbfWrenchNetwork.h"

namespace ocs2 { namespace legged_robot_adaptive { namespace adaptive { namespace {

SixSubnetworkRbfWrenchSettings frozenSettings() {
  SixSubnetworkRbfWrenchSettings settings;
  settings.inputScale << 18.40147689318011, 18.757520233566048, 17.534482646627026, 2.3370077074651903, 5.466041588930613, 1.7261893548458138;
  settings.outputScale = settings.inputScale;
  vector6_t limits, widths;
  limits << 1.0043173709219944, 1.0008567179053736, 1.0027441431454545, 1.0570257160809347, 1.0046340791545685, 1.0246288237663064;
  widths << .062769835682624647, .062553544869085848, .062671508946590904, .066064107255058416, .06278962994716053, .064039301485394148;
  settings.centers.resize(6, 25);
  for (Eigen::Index axis = 0; axis < 6; ++axis) for (Eigen::Index node = 0; node < 25; ++node) settings.centers(axis, node) = limits(axis) * (node - 12.) / 12.;
  settings.widths = widths;
  return settings;
}
SixSubnetworkRbfWrenchNetwork make() { return SixSubnetworkRbfWrenchNetwork(frozenSettings()); }

TEST(SixSubnetworkRbf, ZeroWeights) {
  vector6_t chi; chi << .2, -.4, .1, .3, -.9, .8;
  EXPECT_TRUE(make().evaluate(chi).physicalOutput.isZero());
}
TEST(SixSubnetworkRbf, CenterActivation) {
  const auto network = make(); vector6_t chi = vector6_t::Zero(); chi(0) = network.centers()(0, 17);
  EXPECT_DOUBLE_EQ(network.computeBasis(chi)(18, 0), 1.);
}
TEST(SixSubnetworkRbf, Bias) {
  auto network = make(); matrix_t weights = matrix_t::Zero(26, 6); weights(0, 2) = 1.; network.setWeights(weights);
  const vector6_t output = network.evaluateNormalized(vector6_t::Zero());
  EXPECT_DOUBLE_EQ(output(2), 1.); EXPECT_TRUE(output.head<2>().isZero()); EXPECT_TRUE(output.tail<3>().isZero());
}
TEST(SixSubnetworkRbf, OutputScale) {
  auto network = make(); matrix_t weights = matrix_t::Zero(26, 6); weights(0, 2) = 1.; network.setWeights(weights);
  EXPECT_DOUBLE_EQ(network.evaluate(vector6_t::Zero()).physicalOutput(2), network.outputScale()(2));
}
TEST(SixSubnetworkRbf, AxisInputIndependence) {
  const auto network = make(); vector6_t first = vector6_t::Zero(), second = vector6_t::Zero(); second(0) = .5;
  const matrix_t a = network.computeBasis(first), b = network.computeBasis(second);
  EXPECT_FALSE(a.col(0).isApprox(b.col(0), 1e-14));
  for (Eigen::Index axis = 1; axis < 6; ++axis) EXPECT_TRUE(a.col(axis).isApprox(b.col(axis), 1e-14));
}
TEST(SixSubnetworkRbf, DifferentAxesUseIndependentBasis) {
  const auto network = make(); vector6_t chi = vector6_t::Zero(); chi(0) = chi(3) = .5;
  const matrix_t basis = network.computeBasis(chi);
  EXPECT_FALSE(basis.col(0).isApprox(basis.col(3), 1e-8));
}
TEST(SixSubnetworkRbf, WeightColumnIndependence) {
  auto network = make(); matrix_t weights = matrix_t::Zero(26, 6); weights(0, 4) = .75; network.setWeights(weights);
  const vector6_t output = network.evaluateNormalized(vector6_t::Zero());
  EXPECT_DOUBLE_EQ(output(4), .75); EXPECT_TRUE(output.head<4>().isZero()); EXPECT_DOUBLE_EQ(output(5), 0.);
}
TEST(SixSubnetworkRbf, AdaptiveWeightCountIs156) {
  EXPECT_EQ(SixSubnetworkRbfWrenchNetwork::kBasisCount * SixSubnetworkRbfWrenchNetwork::kAxisCount, 156);
  EXPECT_EQ(make().weights().size(), 156);
}
TEST(SixSubnetworkRbf, RejectsInvalidWidths) {
  auto settings = frozenSettings(); settings.widths(0) = 0.; EXPECT_THROW(SixSubnetworkRbfWrenchNetwork{settings}, std::invalid_argument);
  settings = frozenSettings(); settings.widths(0) = std::numeric_limits<scalar_t>::quiet_NaN(); EXPECT_THROW(SixSubnetworkRbfWrenchNetwork{settings}, std::invalid_argument);
  settings = frozenSettings(); settings.widths(0) = std::numeric_limits<scalar_t>::infinity(); EXPECT_THROW(SixSubnetworkRbfWrenchNetwork{settings}, std::invalid_argument);
}
TEST(SixSubnetworkRbf, RejectsInvalidScalesAndCenters) {
  auto settings = frozenSettings(); settings.inputScale(0) = 0.; EXPECT_THROW(SixSubnetworkRbfWrenchNetwork{settings}, std::invalid_argument);
  settings = frozenSettings(); settings.outputScale(1) = std::numeric_limits<scalar_t>::infinity(); EXPECT_THROW(SixSubnetworkRbfWrenchNetwork{settings}, std::invalid_argument);
  settings = frozenSettings(); settings.centers(0, 0) = std::numeric_limits<scalar_t>::quiet_NaN(); EXPECT_THROW(SixSubnetworkRbfWrenchNetwork{settings}, std::invalid_argument);
}
TEST(SixSubnetworkRbf, RejectsInvalidChi) {
  vector6_t invalid = vector6_t::Zero(); invalid(0) = std::numeric_limits<scalar_t>::infinity(); EXPECT_THROW(make().computeBasis(invalid), std::invalid_argument);
  invalid(0) = std::numeric_limits<scalar_t>::quiet_NaN(); EXPECT_THROW(make().computeBasis(invalid), std::invalid_argument);
}
TEST(SixSubnetworkRbf, FrozenBasisMatchesPythonAuditAtZeroAndHalf) {
  const auto network = make();
  for (const scalar_t value : {0., .5, -.5}) {
    const matrix_t basis = network.computeBasis(vector6_t::Constant(value));
    const vector6_t expectedMax = value == 0. ? vector6_t::Ones() : (vector6_t() << .99940882188352076, .9999765535682873, .99976037551329133, .91106904229969099, .99931936665520926, .98168125916246696).finished();
    for (Eigen::Index axis = 0; axis < 6; ++axis) {
      EXPECT_NEAR(basis.col(axis).tail(25).maxCoeff(), expectedMax(axis), 1e-13);
      EXPECT_EQ((basis.col(axis).tail(25).array() > .1).count(), 3);
    }
  }
}
TEST(SixSubnetworkRbf, InputFeatureIntegrationSanity) {
  vector6_t scale = frozenSettings().inputScale;
  RbfWrenchInputFeature feature(scale); vector6_t residual = vector6_t::Zero(), previous = vector6_t::Zero(); residual(0) = 18.4; previous(0) = 10.;
  const auto input = feature.compute(residual, previous); EXPECT_NEAR(input.uncompensatedDiscrepancy(0), 8.4, 1e-14); EXPECT_NEAR(input.chi(0), 8.4 / scale(0), 1e-14);
  auto network = make(); matrix_t weights = matrix_t::Zero(26, 6); weights(1, 0) = 1.; network.setWeights(weights);
  const auto evaluation = network.evaluate(input.chi); EXPECT_GT(evaluation.physicalOutput(0), 0.); EXPECT_TRUE(evaluation.physicalOutput.tail<5>().isZero());
}

}}}}  // namespace ocs2::legged_robot_adaptive::adaptive
