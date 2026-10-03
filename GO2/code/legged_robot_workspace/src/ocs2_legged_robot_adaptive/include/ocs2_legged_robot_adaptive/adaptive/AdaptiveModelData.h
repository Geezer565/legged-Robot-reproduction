#pragma once

#include <memory>

#include "ocs2_legged_robot_adaptive/adaptive/AdaptiveModelSnapshot.h"

namespace ocs2 {
namespace legged_robot_adaptive {
namespace adaptive {

/**
 * 每轮 MPC 求解冻结的 immutable snapshot 通道。
 * synchronized module 是唯一写者；dynamics 只读同一 shared_ptr，保证整轮 rollout/线性化不混用估计。
 */
class AdaptiveModelData final {
 public:
  explicit AdaptiveModelData(AdaptiveModelSnapshotPtr snapshot);
  void publish(AdaptiveModelSnapshotPtr snapshot);
  AdaptiveModelSnapshotPtr snapshot() const;

 private:
  AdaptiveModelSnapshotPtr snapshot_;
};

}  // namespace adaptive
}  // namespace legged_robot_adaptive
}  // namespace ocs2
