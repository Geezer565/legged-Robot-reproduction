#pragma once

#include "ocs2_legged_robot_adaptive/adaptive/AdaptiveTypes.h"

namespace ocs2 { namespace legged_robot_adaptive { namespace adaptive {

struct TwoFeatureRbfWeightAdapterSettings { EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  vector6_t gamma = vector6_t::Zero();
  vector6_t outputScale = vector6_t::Ones();
};

/**
 * Phase2C-1B.2 的纯数学 Minniti-consistent RBF 权重导数。
 * 固定 Gaussian basis 后网络对 W 线性参数化，采用正号
 * Wdot_i=gamma_i*s_out_i*Phi_i*sigma_i，对应 piHatDot=Gamma*Y_u^T*sigma。
 * input residual/discrepancy scales 仅在 feature->basis 中生效，绝不重复进入此处；
 * OCS2 normalized momentum dynamics 的 -wHat/m 也不意味着此处再乘 1/m。
 * sigma 保持物理 world velocity/angular-velocity sliding variable，不归一化。
 */
class TwoFeatureSixSubnetworkRbfWeightAdapter final {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  static constexpr Eigen::Index kBasisCount = 26;
  static constexpr Eigen::Index kAxisCount = 6;
  explicit TwoFeatureSixSubnetworkRbfWeightAdapter(TwoFeatureRbfWeightAdapterSettings settings);
  matrix_t weightDerivative(const matrix_t& basis, const vector6_t& sigma) const;
  matrix_t eulerStep(const matrix_t& weights, const matrix_t& weightDerivative, scalar_t dt) const;
  const vector6_t& gamma() const { return settings_.gamma; }
  const vector6_t& outputScale() const { return settings_.outputScale; }
 private:
  void validateMatrix(const matrix_t& value, const char* name) const;
  TwoFeatureRbfWeightAdapterSettings settings_;
};

}}}  // namespace ocs2::legged_robot_adaptive::adaptive
