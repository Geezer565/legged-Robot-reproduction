#pragma once

#include "ocs2_legged_robot_adaptive/adaptive/AdaptiveTypes.h"

namespace ocs2 { namespace legged_robot_adaptive { namespace adaptive {

/** Two rows by six axes: row 0 is residual level, row 1 is uncompensated discrepancy. */
struct TwoFeatureRbfWrenchInputResult { EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  matrix_t feature;  // Shape 2x6, axis order [Fx,Fy,Fz,Tx,Ty,Tz].
};

/**
 * Phase2C-1A.5 feature extraction. 两行都是可观测代理：第一行保留 total
 * residual level，第二行表示尚未补偿部分；两者均不是 ground truth 或 label。
 * previous estimate 使用上一周期值，避免同周期 algebraic loop。
 */
class TwoFeatureRbfWrenchInput final {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  TwoFeatureRbfWrenchInput(vector6_t residualScale, vector6_t discrepancyScale);
  TwoFeatureRbfWrenchInputResult compute(const vector6_t& residualFiltered,
                                         const vector6_t& previousWrenchEstimate) const;
  const vector6_t& residualScale() const { return residualScale_; }
  const vector6_t& discrepancyScale() const { return discrepancyScale_; }
 private:
  static void validateFinite(const vector6_t& value, const char* name);
  vector6_t residualScale_;
  vector6_t discrepancyScale_;
};

}}}  // namespace ocs2::legged_robot_adaptive::adaptive
