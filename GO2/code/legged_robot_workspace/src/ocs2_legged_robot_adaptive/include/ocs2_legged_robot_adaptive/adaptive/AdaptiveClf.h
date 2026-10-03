#pragma once

#include "ocs2_legged_robot_adaptive/adaptive/AdaptiveTypes.h"

namespace ocs2 {
namespace legged_robot_adaptive {
namespace adaptive {

struct AdaptiveClfInput {
  vector6_t sigma = vector6_t::Zero();
  vector6_t contactWrenchWorld = vector6_t::Zero();
  vector6_t nominalReferenceWrenchWorld = vector6_t::Zero();
  regressor_t unknownRegressor = regressor_t::Identity(6, 6);
  parameter_vector_t parameterEstimate = parameter_vector_t::Zero(6);
};

class AdaptiveClf final {
 public:
  explicit AdaptiveClf(const AdaptiveClfSettings& settings);
  scalar_t computeValue(const AdaptiveClfInput& input) const;
  const matrix6_t& kd() const { return kd_; }

 private:
  matrix6_t kd_;
};

}  // namespace adaptive
}  // namespace legged_robot_adaptive
}  // namespace ocs2
