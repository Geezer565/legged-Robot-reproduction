#include "ocs2_legged_robot_adaptive/adaptive/RbfWrenchInputFeature.h"

#include <stdexcept>
#include <utility>

namespace ocs2 { namespace legged_robot_adaptive { namespace adaptive {

RbfWrenchInputFeature::RbfWrenchInputFeature(vector6_t discrepancyScale)
    : discrepancyScale_(std::move(discrepancyScale)) {
  if (!discrepancyScale_.allFinite() || (discrepancyScale_.array() <= 0.).any()) {
    throw std::invalid_argument("[RbfWrenchInputFeature] discrepancy scale must be positive and finite.");
  }
}

void RbfWrenchInputFeature::validateVector(const vector6_t& value, const char* name) const {
  if (!value.allFinite()) {
    throw std::invalid_argument(std::string("[RbfWrenchInputFeature] ") + name + " must be finite.");
  }
}

vector6_t RbfWrenchInputFeature::computeUncompensatedDiscrepancy(
    const vector6_t& residualFiltered, const vector6_t& previousWrenchEstimate) const {
  validateVector(residualFiltered, "residualFiltered");
  validateVector(previousWrenchEstimate, "previousWrenchEstimate");
  // 当前观测代理减去上一周期已补偿 wrench：不是 ground truth 误差的严格等式。
  return residualFiltered - previousWrenchEstimate;
}

vector6_t RbfWrenchInputFeature::normalize(const vector6_t& discrepancy) const {
  validateVector(discrepancy, "uncompensated discrepancy");
  // chi = S_chi^{-1} dW，仅供 Gaussian basis 的激活计算使用。
  return discrepancy.cwiseQuotient(discrepancyScale_);
}

RbfWrenchInputFeatureResult RbfWrenchInputFeature::compute(
    const vector6_t& residualFiltered, const vector6_t& previousWrenchEstimate) const {
  RbfWrenchInputFeatureResult result;
  result.uncompensatedDiscrepancy =
      computeUncompensatedDiscrepancy(residualFiltered, previousWrenchEstimate);
  result.chi = normalize(result.uncompensatedDiscrepancy);
  return result;
}

}}}  // namespace ocs2::legged_robot_adaptive::adaptive
