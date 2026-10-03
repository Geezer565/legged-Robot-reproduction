#pragma once

#include "ocs2_legged_robot_adaptive/adaptive/AdaptiveTypes.h"

namespace ocs2 { namespace legged_robot_adaptive { namespace adaptive {

struct TwoFeatureSixSubnetworkRbfWrenchSettings { EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  vector6_t residualScale = vector6_t::Ones();  // Config metadata; feature has already normalized it.
  vector6_t discrepancyScale = vector6_t::Ones();
  vector6_t outputScale = vector6_t::Ones();
  matrix_t centers;  // Shape 6x50: for axis i, columns [2*j,2*j+1] form center j.
  matrix_t widths;   // Shape 6x25, node-specific positive widths.
};
struct TwoFeatureSixSubnetworkRbfWrenchEvaluation { EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  matrix_t basis; vector6_t normalizedOutput = vector6_t::Zero(); vector6_t physicalOutput = vector6_t::Zero();
};

TwoFeatureSixSubnetworkRbfWrenchSettings
loadTwoFeatureSixSubnetworkRbfWrenchSettings(
    const std::string& filename,
    const std::string& prefix = "rbf_wrench_two_feature_six_subnetwork_basis");

/** Six independent 2D Gaussian bases -> six scalar wrench outputs; no weight law lives here. */
class TwoFeatureSixSubnetworkRbfWrenchNetwork final {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  static constexpr Eigen::Index kAxisCount=6, kInputDimension=2, kGaussianNodeCount=25, kBasisCount=26;
  explicit TwoFeatureSixSubnetworkRbfWrenchNetwork(TwoFeatureSixSubnetworkRbfWrenchSettings settings);
  matrix_t computeBasis(const matrix_t& feature) const;
  vector6_t evaluateNormalized(const matrix_t& feature) const;
  TwoFeatureSixSubnetworkRbfWrenchEvaluation evaluate(const matrix_t& feature) const;
  const matrix_t& weights() const { return weights_; } void setWeights(const matrix_t& weights); void resetWeights();
  const matrix_t& centers() const { return settings_.centers; } const matrix_t& widths() const { return settings_.widths; }
  const vector6_t& outputScale() const { return settings_.outputScale; }
 private:
  void validateSettings() const;
  TwoFeatureSixSubnetworkRbfWrenchSettings settings_; matrix_t weights_;
};

}}}  // namespace ocs2::legged_robot_adaptive::adaptive
