#include "ocs2_legged_robot_adaptive/adaptive/AdaptiveParameterEstimator.h"

#include <cmath>
#include <stdexcept>

namespace ocs2 {
namespace legged_robot_adaptive {
namespace adaptive {

AdaptiveParameterEstimator::AdaptiveParameterEstimator(const AdaptiveClfSettings& settings)
    : gamma_(settings.gamma),
      parameterization_(settings.parameterization),
      payloadMassAdaptationGain_(settings.payloadMassAdaptationGain),
      snapshot_(AdaptiveModelSnapshot::createInitial(settings.initialParameterEstimate)) {
  validateAdaptiveClfSettings(settings);
}

parameter_vector_t AdaptiveParameterEstimator::computeParameterDerivative(
    const AdaptiveEstimatorUpdate& update) const {
  if (!update.sigma.allFinite() || !update.regressor.allFinite() || !std::isfinite(update.dt) || update.dt <= 0.0 ||
      !std::isfinite(update.time) || update.regressor.rows() != 6 ||
      update.regressor.cols() != snapshot_->estimate().size() ||
      (snapshot_->valid() && update.time <= snapshot_->time())) {
    throw std::invalid_argument("[AdaptiveParameterEstimator] Eq.(8) 更新输入非法。");
  }
  if (update.nominalDriveBaseline.size() != 0 &&
      (parameterization_ != AdaptiveParameterization::PayloadInertial10D ||
       update.nominalDriveBaseline.size() != snapshot_->estimate().size() ||
       !update.nominalDriveBaseline.allFinite())) {
    throw std::invalid_argument(
        "[AdaptiveParameterEstimator] D5H1 nominal drive baseline is invalid.");
  }
  // ============================================================
  // 论文 Eq.(8)：pi_hat_dot = Gamma * Y_u^T * sigma；显式 Euler：pi_hat_next = pi_hat + dt*pi_hat_dot。
  // sigma 唯一来自 SlidingVariable，因此 estimator 与后续 CLF 不会产生符号不一致。
  // 不做饱和/投影：那些是工程保护而非论文原公式；Gamma 是配置中的调参位置。
  // ============================================================
  parameter_vector_t parameterDerivative =
      parameter_vector_t::Zero(snapshot_->estimate().size());
  if (parameterization_ != AdaptiveParameterization::PayloadMass1D) {
    // ConstantWrench6D, PayloadInertial10D and the joint 16D mode all use
    // the paper's vector Eq.(8): pi_dot=Gamma*Y^T*sigma.  For the joint
    // mode Gamma is the configured 16x16 block-diagonal tuning matrix.
    parameter_vector_t parameterDrive =
        update.regressor.transpose() * update.sigma;
    if (update.nominalDriveBaseline.size() != 0) {
      // D5H1 robustness candidate: subtract the frozen nominal gait-phase
      // mean in pre-Gamma coordinates.  The default empty vector preserves
      // the historical paper Eq. (8) path exactly.
      parameterDrive -= update.nominalDriveBaseline;
    }
    parameterDerivative = gamma_ * parameterDrive;
  } else {
    // PayloadMass1D stores delta-mass [kg] only in element zero. Eq.(8) is
    // delta_m_hat_dot = Gamma_m * y_m(q,v,v_r,dot(v_r))^T * sigma.
    parameterDerivative(0) = payloadMassAdaptationGain_ *
                             update.regressor.col(0).dot(update.sigma);
  }
  return parameterDerivative;
}

AdaptiveModelSnapshotPtr AdaptiveParameterEstimator::update(const AdaptiveEstimatorUpdate& update) {
  const parameter_vector_t parameterDerivative =
      computeParameterDerivative(update);
  const parameter_vector_t next = snapshot_->estimate() + update.dt * parameterDerivative;
  if (!next.allFinite()) {
    throw std::overflow_error("[AdaptiveParameterEstimator] 参数估计溢出。");
  }
  snapshot_ = AdaptiveModelSnapshot::createValid(next, update.time, snapshot_->version() + 1);
  return snapshot_;
}

AdaptiveModelSnapshotPtr AdaptiveParameterEstimator::reset(const parameter_vector_t& estimate, scalar_t time) {
  if (!estimate.allFinite() || !std::isfinite(time)) {
    throw std::invalid_argument("[AdaptiveParameterEstimator] reset 参数非法。");
  }
  if (estimate.size() != static_cast<Eigen::Index>(parameterDimension(parameterization_))) {
    throw std::invalid_argument("[AdaptiveParameterEstimator] reset estimate dimension mismatch.");
  }
  if (parameterization_ == AdaptiveParameterization::PayloadMass1D &&
      !estimate.tail(5).isZero(0.0)) {
    throw std::invalid_argument(
        "[AdaptiveParameterEstimator] PayloadMass1D inactive estimate elements must be zero.");
  }
  snapshot_ = AdaptiveModelSnapshot::createValid(estimate, time, snapshot_->version() + 1);
  return snapshot_;
}

}  // namespace adaptive
}  // namespace legged_robot_adaptive
}  // namespace ocs2
