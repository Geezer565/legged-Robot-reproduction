#pragma once

#include "ocs2_legged_robot_adaptive/adaptive/AdaptiveTypes.h"

#include <array>

namespace ocs2 { namespace legged_robot_adaptive { namespace adaptive {

/** Phase2C-1A.3: six 1D subnetworks, each with one bias plus 25 Gaussian nodes. */
struct SixSubnetworkRbfWrenchSettings { EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  vector6_t inputScale = vector6_t::Ones();
  vector6_t outputScale = vector6_t::Ones();
  matrix_t centers;  // Shape 6x25: row i contains only subnetwork i's 1D centers.
  vector6_t widths = vector6_t::Ones();
};

struct SixSubnetworkRbfWrenchEvaluation { EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  matrix_t basis;  // Shape 26x6; column i is [1, phi_i1, ..., phi_i25].
  vector6_t normalizedOutput = vector6_t::Zero();
  vector6_t physicalOutput = vector6_t::Zero();
};

/**
 * 新结构：六个独立的 1D Gaussian bases 各自产生一个 scalar output。
 * 旧 RbfWrenchNetwork 是“共享 6D Gaussian basis -> 六输出”；该类刻意不复用
 * 6D 距离。原因是共享 basis 在 discrepancy envelope 上存在 coverage/locality
 * 冲突，而此 topology 已在冻结 split 的独立 holdout 上审计通过。
 *
 * 输入 chi_i 是 observable uncompensated-discrepancy proxy 的归一化分量，
 * 不是 true uncertainty 或 supervised label。该类不提取输入特征、也不实现
 * weight adaptation；RbfWrenchInputFeature 仍负责 residualFiltered-previousEstimate。
 */
class SixSubnetworkRbfWrenchNetwork final {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  static constexpr Eigen::Index kAxisCount = 6;
  static constexpr Eigen::Index kGaussianNodeCount = 25;
  static constexpr Eigen::Index kBasisCount = kGaussianNodeCount + 1;

  explicit SixSubnetworkRbfWrenchNetwork(SixSubnetworkRbfWrenchSettings settings);
  matrix_t computeBasis(const vector6_t& chi) const;
  vector6_t evaluateNormalized(const vector6_t& chi) const;
  SixSubnetworkRbfWrenchEvaluation evaluate(const vector6_t& chi) const;
  const matrix_t& weights() const { return weights_; }
  void setWeights(const matrix_t& weights);
  void resetWeights();
  const matrix_t& centers() const { return settings_.centers; }
  const vector6_t& widths() const { return settings_.widths; }
  const vector6_t& inputScale() const { return settings_.inputScale; }
  const vector6_t& outputScale() const { return settings_.outputScale; }

 private:
  void validateSettings() const;
  static void validateFinite(const vector6_t& value, const char* name);
  SixSubnetworkRbfWrenchSettings settings_;
  matrix_t weights_;
};

}}}  // namespace ocs2::legged_robot_adaptive::adaptive
