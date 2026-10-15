#pragma once

#include <memory>
#include <mutex>

#include <ocs2_core/constraint/StateInputConstraint.h>

#include "ocs2_legged_robot_adaptive/adaptive/AdaptiveClfEvaluator.h"
#include "ocs2_legged_robot_adaptive/adaptive/AdaptiveModelData.h"

namespace ocs2 {
namespace legged_robot_adaptive {
using AdaptiveClfDiagnostics = adaptive::AdaptiveClfDiagnostics;

/**
 * OCS2 的 StateInputSoftConstraint 会 clone 原始 constraint。solver 实际计算的是 clone，
 * ROS 持有的却是 interface 创建的原对象；因此两者必须共享此 storage，否则 topic 会停在旧值。
 */
struct AdaptiveClfDiagnosticsData {
  mutable std::mutex mutex;
  AdaptiveClfDiagnostics latest;
};

class AdaptiveClfConstraint final : public StateInputConstraint {
 public:
  AdaptiveClfConstraint(const PinocchioInterface& pinocchioInterface, CentroidalModelInfo info,
                        const ReferenceManagerInterface& referenceManager,
                        adaptive::AdaptiveClfSettings settings,
                        std::shared_ptr<const adaptive::AdaptiveModelData> modelData);
  AdaptiveClfConstraint* clone() const override { return new AdaptiveClfConstraint(*this); }
  size_t getNumConstraints(scalar_t) const override { return 1; }
  vector_t getValue(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation&) const override;
  VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                           const PreComputation& preComp) const override;
  AdaptiveClfDiagnostics diagnostics() const;
 private:
  AdaptiveClfConstraint(const AdaptiveClfConstraint& other);
  scalar_t evaluate(scalar_t time, const vector_t& state, const vector_t& input, bool publish) const;
  adaptive::AdaptiveClfEvaluator evaluator_;
  std::shared_ptr<const adaptive::AdaptiveModelData> modelData_;
  std::shared_ptr<AdaptiveClfDiagnosticsData> diagnosticsData_;
};
}  // namespace legged_robot_adaptive
}  // namespace ocs2
