#pragma once

#include <cstdint>
#include <memory>

#include "ocs2_legged_robot_adaptive/adaptive/AdaptiveTypes.h"

namespace ocs2 {
namespace legged_robot_adaptive {
namespace adaptive {

class AdaptiveModelSnapshot final {
 public:
  using ConstPtr = std::shared_ptr<const AdaptiveModelSnapshot>;
  static ConstPtr createInitial(const parameter_vector_t& estimate);
  static ConstPtr createValid(const parameter_vector_t& estimate, scalar_t time, std::uint64_t version);
  const parameter_vector_t& estimate() const { return estimate_; }
  scalar_t time() const { return time_; }
  std::uint64_t version() const { return version_; }
  bool valid() const { return valid_; }
 private:
  AdaptiveModelSnapshot(parameter_vector_t estimate, scalar_t time, std::uint64_t version, bool valid);
  parameter_vector_t estimate_;
  scalar_t time_;
  std::uint64_t version_;
  bool valid_;
};
using AdaptiveModelSnapshotPtr = AdaptiveModelSnapshot::ConstPtr;
}  // namespace adaptive
}  // namespace legged_robot_adaptive
}  // namespace ocs2
