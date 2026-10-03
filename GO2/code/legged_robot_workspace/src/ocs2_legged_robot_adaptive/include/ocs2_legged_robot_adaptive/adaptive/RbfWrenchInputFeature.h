#pragma once

#include "ocs2_legged_robot_adaptive/adaptive/AdaptiveTypes.h"

namespace ocs2 { namespace legged_robot_adaptive { namespace adaptive {

/** Phase2C-1A RBF 输入的中间量；所有量的顺序均为 [Fx,Fy,Fz,Tx,Ty,Tz]。 */
struct RbfWrenchInputFeatureResult { EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  vector6_t uncompensatedDiscrepancy = vector6_t::Zero();
  vector6_t chi = vector6_t::Zero();
};

/**
 * Phase2C-1A 的纯数学输入特征模块。
 *
 * residualFiltered 是动力学不一致的可观测代理；它会含有 observer bias、滤波
 * 延迟、接触/模型失配和腿部动力学污染。previousWrenchEstimate 是上一周期
 * 网络已补偿的完整 6D wrench。因此二者之差是当前“未补偿不确定性”的
 * observable discrepancy proxy，而非 true uncertainty error。
 *
 * chi 只决定 RBF basis activation，不是 ground-truth uncertainty，也不是
 * supervised training label。sigma、tracking error 和 reference error 不属于
 * 本输入向量；它们留给后续 Lyapunov-derived adaptive weight law。该职责分离
 * 刻意避免旧的 residual -> residual identity mapping。
 */
class RbfWrenchInputFeature final {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /** discrepancyScale 初版复用 Phase2B residualScale，尚未针对闭环 discrepancy 重拟合。 */
  explicit RbfWrenchInputFeature(vector6_t discrepancyScale);

  vector6_t computeUncompensatedDiscrepancy(const vector6_t& residualFiltered,
                                             const vector6_t& previousWrenchEstimate) const;
  vector6_t normalize(const vector6_t& discrepancy) const;
  RbfWrenchInputFeatureResult compute(const vector6_t& residualFiltered,
                                      const vector6_t& previousWrenchEstimate) const;
  const vector6_t& discrepancyScale() const { return discrepancyScale_; }

 private:
  void validateVector(const vector6_t& value, const char* name) const;
  vector6_t discrepancyScale_;
};

}}}  // namespace ocs2::legged_robot_adaptive::adaptive
