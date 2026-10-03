#include "ocs2_legged_robot_adaptive/adaptive/WrenchResidualEstimator.h"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace ocs2 {
namespace legged_robot_adaptive {
namespace adaptive {

WrenchResidualEstimator::WrenchResidualEstimator(
    const PinocchioInterface& pinocchioInterface, CentroidalModelInfo nominalInfo,
    bool enableFilter, scalar_t filterAlpha)
    : nominalDynamics_(pinocchioInterface, nominalInfo, nominalInfo),
      robotMass_(nominalInfo.robotMass),
      enableFilter_(enableFilter),
      filterAlpha_(filterAlpha) {
  if (!(robotMass_ > 0.0) || !std::isfinite(robotMass_)) {
    throw std::invalid_argument(
        "[WrenchResidualEstimator] nominal robot mass must be positive and finite.");
  }
  validateFilterSettings(enableFilter_, filterAlpha_);
}

void WrenchResidualEstimator::validateFilterSettings(bool, scalar_t filterAlpha) {
  if (!std::isfinite(filterAlpha) || filterAlpha < 0.0 || filterAlpha >= 1.0) {
    throw std::invalid_argument(
        "[WrenchResidualEstimator] filter alpha must satisfy 0 <= alpha < 1.");
  }
}

bool WrenchResidualEstimator::validSample(scalar_t time, const vector_t& state,
                                          const vector_t& input) const {
  return std::isfinite(time) && state.size() >= 6 &&
         state.size() == nominalDynamics_.nominalInfo().stateDim &&
         input.size() == nominalDynamics_.nominalInfo().inputDim &&
         state.allFinite() && input.allFinite();
}

void WrenchResidualEstimator::storePreviousSample(scalar_t time,
                                                   const vector_t& state,
                                                   const vector_t& input) {
  previousTime_ = time;
  previousState_ = state;
  previousInput_ = input;
  hasPreviousSample_ = true;
}

vector6_t WrenchResidualEstimator::reconstructWrenchResidual(
    scalar_t robotMass, const vector6_t& observedRate,
    const vector6_t& nominalRate) {
  if (!(robotMass > 0.0) || !std::isfinite(robotMass) ||
      !observedRate.allFinite() || !nominalRate.allFinite()) {
    throw std::invalid_argument(
        "[WrenchResidualEstimator] residual inputs must be finite.");
  }
  // AdaptiveLeggedRobotDynamics 的 Eq.(10) 约定为 hdot = hdot_nom - w/m，
  // 因此此处乘 nominal robotMass 并保留负号，严格反解出 [F; T]。前三维
  // 恢复为 N，后三维恢复为关于质心的 Nm；后者不是角加速度。
  return -robotMass * (observedRate - nominalRate);
}

WrenchResidualResult WrenchResidualEstimator::update(
    scalar_t time, const vector_t& state, const vector_t& input) {
  WrenchResidualResult result;
  if (!validSample(time, state, input)) {
    return result;
  }

  if (!hasPreviousSample_) {
    // 第一帧没有时间区间：只缓存，绝不以零状态伪造 residual，也不初始化滤波器。
    storePreviousSample(time, state, input);
    return result;
  }

  const scalar_t dt = time - previousTime_;
  result.dt = dt;
  if (!std::isfinite(dt) || dt <= std::numeric_limits<scalar_t>::epsilon()) {
    // 丢弃异常区间但推进到最新的有限样本，避免后续帧反复跨越同一个异常时间点。
    // raw/filter 均保持不变，且本帧不发布有效结果。
    storePreviousSample(time, state, input);
    return result;
  }

  // OCS2 state 前6维是 normalized centroidal momentum（centroidal momentum / m），
  // 并非 [v, omega]。故其导数不是直接的线/角加速度，而是下面 residual 的动力学坐标。
  result.observedRate =
      (state.head<6>() - previousState_.head<6>()) / dt;

  // 仅以完全 nominal 模型在区间两端求 flow；绝不读取 adaptive snapshot、pi_hat、
  // payload/true plant 或任何真值扰动。端点平均与差分 observedRate 对应同一时间区间。
  const vector6_t nominalPrevious = nominalDynamics_
                                        .computeFlowMap(previousTime_, previousState_,
                                                        previousInput_, preComputation_)
                                        .head<6>();
  const vector6_t nominalCurrent = nominalDynamics_
                                       .computeFlowMap(time, state, input,
                                                       preComputation_)
                                       .head<6>();
  result.nominalRate = 0.5 * (nominalPrevious + nominalCurrent);
  if (!result.observedRate.allFinite() || !result.nominalRate.allFinite()) {
    storePreviousSample(time, state, input);
    return result;
  }

  result.rawResidual = reconstructWrenchResidual(
      robotMass_, result.observedRate, result.nominalRate);
  rawResidual_ = result.rawResidual;
  if (!enableFilter_) {
    filteredResidual_ = rawResidual_;
  } else if (!filterInitialized_) {
    filteredResidual_ = rawResidual_;
    filterInitialized_ = true;
  } else {
    filteredResidual_ = filterAlpha_ * filteredResidual_ +
                        (1.0 - filterAlpha_) * rawResidual_;
  }
  result.filteredResidual = filteredResidual_;
  result.valid = true;

  // The closed-loop RBF module consumes this six-dimensional residual; the
  // estimator itself remains independent of the adaptive snapshot and plant truth.
  storePreviousSample(time, state, input);
  return result;
}

void WrenchResidualEstimator::reset() {
  previousTime_ = 0.0;
  previousState_.resize(0);
  previousInput_.resize(0);
  hasPreviousSample_ = false;
  rawResidual_.setZero();
  filteredResidual_.setZero();
  filterInitialized_ = false;
}

}  // namespace adaptive
}  // namespace legged_robot_adaptive
}  // namespace ocs2
