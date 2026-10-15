#include "ocs2_legged_robot_adaptive/constraint/AdaptiveClfConstraint.h"

#include <stdexcept>

namespace ocs2 {
namespace legged_robot_adaptive {

AdaptiveClfConstraint::AdaptiveClfConstraint(const PinocchioInterface& pinocchioInterface, CentroidalModelInfo info,
                                             const ReferenceManagerInterface& referenceManager,
                                             adaptive::AdaptiveClfSettings settings,
                                             std::shared_ptr<const adaptive::AdaptiveModelData> modelData)
    : StateInputConstraint(ConstraintOrder::Linear),
      evaluator_(pinocchioInterface, std::move(info), referenceManager, settings),
      modelData_(std::move(modelData)),
      diagnosticsData_(std::make_shared<AdaptiveClfDiagnosticsData>()) {
  if (!modelData_) {
    throw std::invalid_argument("[AdaptiveClfConstraint] AdaptiveModelData 不可为空。");
  }
}

AdaptiveClfConstraint::AdaptiveClfConstraint(const AdaptiveClfConstraint& other)
    : StateInputConstraint(other),
      evaluator_(other.evaluator_),
      modelData_(other.modelData_),
      diagnosticsData_(other.diagnosticsData_) {}

scalar_t AdaptiveClfConstraint::evaluate(scalar_t time, const vector_t& state, const vector_t& input, bool publish) const {
  // Eq.(11) 的所有实际数学计算唯一委托给 AdaptiveClfEvaluator。这样 solver
  // constraint 和 executed observation 旁路诊断不会各自维护一份公式/符号链。
  const AdaptiveClfDiagnostics diagnostics = evaluator_.evaluate(time, state, input, modelData_->snapshot());

  if (publish) {
    // 不能在 evaluate() 发布 ROS topic：DDP rollout 与有限差分会高频调用。
    // 这里只存 solver 内部 latest；executed monitor 使用独立 evaluator，绝不覆盖它。
    std::lock_guard<std::mutex> lock(diagnosticsData_->mutex);
    diagnosticsData_->latest = diagnostics;
  }
  return diagnostics.hClf;
}

vector_t AdaptiveClfConstraint::getValue(scalar_t time, const vector_t& state, const vector_t& input,
                                         const PreComputation&) const {
  vector_t value(1);
  value(0) = evaluate(time, state, input, true);
  return value;
}

VectorFunctionLinearApproximation AdaptiveClfConstraint::getLinearApproximation(
    scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) const {
  VectorFunctionLinearApproximation approximation;
  approximation.f = getValue(time, state, input, preComp);
  approximation.dfdx = matrix_t::Zero(1, state.size());
  approximation.dfdu = matrix_t::Zero(1, input.size());

  // 这仍是既有的中心有限差分线性化；扰动采样明确 publish=false，避免覆盖
  // solver 名义点 diagnostics，也避免 executed diagnostics 被 DDP 影响。
  const scalar_t step = 1.0e-6;
  for (Eigen::Index j = 0; j < state.size(); ++j) {
    vector_t plus = state;
    vector_t minus = state;
    plus(j) += step;
    minus(j) -= step;
    approximation.dfdx(0, j) = (evaluate(time, plus, input, false) - evaluate(time, minus, input, false)) / (2.0 * step);
  }
  for (Eigen::Index j = 0; j < input.size(); ++j) {
    vector_t plus = input;
    vector_t minus = input;
    plus(j) += step;
    minus(j) -= step;
    approximation.dfdu(0, j) = (evaluate(time, state, plus, false) - evaluate(time, state, minus, false)) / (2.0 * step);
  }
  return approximation;
}

AdaptiveClfDiagnostics AdaptiveClfConstraint::diagnostics() const {
  std::lock_guard<std::mutex> lock(diagnosticsData_->mutex);
  return diagnosticsData_->latest;
}

}  // namespace legged_robot_adaptive
}  // namespace ocs2
