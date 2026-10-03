#include "ocs2_legged_robot_adaptive/adaptive/AdaptiveClf.h"
#include <stdexcept>
namespace ocs2 { namespace legged_robot_adaptive { namespace adaptive {
AdaptiveClf::AdaptiveClf(const AdaptiveClfSettings& s) : kd_(s.kd) { validateAdaptiveClfSettings(s); }
scalar_t AdaptiveClf::computeValue(const AdaptiveClfInput& in) const {
  if (!in.sigma.allFinite() || !in.contactWrenchWorld.allFinite() || !in.nominalReferenceWrenchWorld.allFinite() || !in.unknownRegressor.allFinite() || !in.parameterEstimate.allFinite() || in.unknownRegressor.rows() != 6 || in.unknownRegressor.cols() != in.parameterEstimate.size()) throw std::invalid_argument("[AdaptiveClf] 输入含 NaN/Inf 或 Y/pi 维度不一致。");
  // ============================================================
  // 论文 Eq.(11)：h_clf = sigma^T(w_contact - w_nominal - Y_u*pi_hat)
  //                         - 0.5*sigma^T*K_D*sigma，要求 h_clf >= 0。
  // 全部 wrench 是 world 系，前三维为力、后三维为关于 COM 的力矩；负号与
  // desired-actual 的 sigma 及参考模板的 constant_wrench_6d 参数定义一致。
  // K_D 作为完整 6x6 配置矩阵保留，是后续调参位置，不使用 clfRate 替代。
  // ============================================================
  return in.sigma.dot(in.contactWrenchWorld - in.nominalReferenceWrenchWorld - in.unknownRegressor * in.parameterEstimate) - 0.5 * in.sigma.dot(kd_ * in.sigma);
}
} } }
