#include "ocs2_legged_robot_adaptive/adaptive/SixSubnetworkRbfWrenchNetwork.h"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace ocs2 { namespace legged_robot_adaptive { namespace adaptive {

SixSubnetworkRbfWrenchNetwork::SixSubnetworkRbfWrenchNetwork(SixSubnetworkRbfWrenchSettings settings)
    : settings_(std::move(settings)), weights_(matrix_t::Zero(kBasisCount, kAxisCount)) {
  validateSettings();
}

void SixSubnetworkRbfWrenchNetwork::validateFinite(const vector6_t& value, const char* name) {
  if (!value.allFinite()) throw std::invalid_argument(std::string("[SixSubnetworkRbfWrenchNetwork] ") + name + " must be finite.");
}

void SixSubnetworkRbfWrenchNetwork::validateSettings() const {
  validateFinite(settings_.inputScale, "inputScale");
  validateFinite(settings_.outputScale, "outputScale");
  if ((settings_.inputScale.array() <= 0.).any() || (settings_.outputScale.array() <= 0.).any() ||
      settings_.centers.rows() != kAxisCount || settings_.centers.cols() != kGaussianNodeCount ||
      !settings_.centers.allFinite() || !settings_.widths.allFinite() || (settings_.widths.array() <= 0.).any()) {
    throw std::invalid_argument("[SixSubnetworkRbfWrenchNetwork] positive finite scales/widths and finite 6x25 centers required.");
  }
}

matrix_t SixSubnetworkRbfWrenchNetwork::computeBasis(const vector6_t& chi) const {
  validateFinite(chi, "chi");
  matrix_t basis(kBasisCount, kAxisCount);
  basis.row(0).setOnes();
  for (Eigen::Index axis = 0; axis < kAxisCount; ++axis) {
    for (Eigen::Index node = 0; node < kGaussianNodeCount; ++node) {
      const scalar_t delta = chi(axis) - settings_.centers(axis, node);
      basis(node + 1, axis) = std::exp(-delta * delta / (2. * settings_.widths(axis) * settings_.widths(axis)));
    }
  }
  return basis;
}

vector6_t SixSubnetworkRbfWrenchNetwork::evaluateNormalized(const vector6_t& chi) const {
  const matrix_t basis = computeBasis(chi);
  vector6_t output;
  for (Eigen::Index axis = 0; axis < kAxisCount; ++axis) output(axis) = weights_.col(axis).dot(basis.col(axis));
  return output;
}

SixSubnetworkRbfWrenchEvaluation SixSubnetworkRbfWrenchNetwork::evaluate(const vector6_t& chi) const {
  SixSubnetworkRbfWrenchEvaluation result;
  result.basis = computeBasis(chi);
  for (Eigen::Index axis = 0; axis < kAxisCount; ++axis) result.normalizedOutput(axis) = weights_.col(axis).dot(result.basis.col(axis));
  result.physicalOutput = settings_.outputScale.cwiseProduct(result.normalizedOutput);
  return result;
}

void SixSubnetworkRbfWrenchNetwork::setWeights(const matrix_t& weights) {
  if (weights.rows() != kBasisCount || weights.cols() != kAxisCount || !weights.allFinite()) {
    throw std::invalid_argument("[SixSubnetworkRbfWrenchNetwork] weights must be finite 26x6.");
  }
  weights_ = weights;
}

void SixSubnetworkRbfWrenchNetwork::resetWeights() { weights_.setZero(); }

}}}  // namespace ocs2::legged_robot_adaptive::adaptive
