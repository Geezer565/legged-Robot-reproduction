#include "ocs2_legged_robot_adaptive/adaptive/TwoFeatureRbfWrenchInput.h"

#include <stdexcept>
#include <utility>

namespace ocs2 { namespace legged_robot_adaptive { namespace adaptive {

TwoFeatureRbfWrenchInput::TwoFeatureRbfWrenchInput(vector6_t residualScale, vector6_t discrepancyScale)
    : residualScale_(std::move(residualScale)), discrepancyScale_(std::move(discrepancyScale)) {
  validateFinite(residualScale_, "residualScale"); validateFinite(discrepancyScale_, "discrepancyScale");
  if ((residualScale_.array() <= 0.).any() || (discrepancyScale_.array() <= 0.).any()) throw std::invalid_argument("[TwoFeatureRbfWrenchInput] scales must be positive.");
}
void TwoFeatureRbfWrenchInput::validateFinite(const vector6_t& value, const char* name) {
  if (!value.allFinite()) throw std::invalid_argument(std::string("[TwoFeatureRbfWrenchInput] ") + name + " must be finite.");
}
TwoFeatureRbfWrenchInputResult TwoFeatureRbfWrenchInput::compute(const vector6_t& residualFiltered, const vector6_t& previousWrenchEstimate) const {
  validateFinite(residualFiltered, "residualFiltered"); validateFinite(previousWrenchEstimate, "previousWrenchEstimate");
  TwoFeatureRbfWrenchInputResult result; result.feature.resize(2, 6);
  result.feature.row(0) = residualFiltered.cwiseQuotient(residualScale_).transpose();
  result.feature.row(1) = (residualFiltered - previousWrenchEstimate).cwiseQuotient(discrepancyScale_).transpose();
  return result;
}

}}}  // namespace ocs2::legged_robot_adaptive::adaptive
