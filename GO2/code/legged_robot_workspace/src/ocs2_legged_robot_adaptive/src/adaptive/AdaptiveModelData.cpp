#include "ocs2_legged_robot_adaptive/adaptive/AdaptiveModelData.h"

#include <stdexcept>

namespace ocs2 {
namespace legged_robot_adaptive {
namespace adaptive {

AdaptiveModelData::AdaptiveModelData(AdaptiveModelSnapshotPtr snapshot) {
  publish(std::move(snapshot));
}

void AdaptiveModelData::publish(AdaptiveModelSnapshotPtr snapshot) {
  if (!snapshot || !snapshot->valid()) {
    throw std::invalid_argument("[AdaptiveModelData] 只能发布有效 immutable snapshot。");
  }
  std::atomic_store(&snapshot_, std::move(snapshot));
}

AdaptiveModelSnapshotPtr AdaptiveModelData::snapshot() const {
  return std::atomic_load(&snapshot_);
}

}  // namespace adaptive
}  // namespace legged_robot_adaptive
}  // namespace ocs2
