#include <pinocchio/fwd.hpp>

#include "legged_controllers/LeggedController.h"
#include "legged_controllers/ExactPayloadTruth.h"

#include <Eigen/Geometry>
#include <cmath>
#include <limits>
#include <ocs2_legged_robot_adaptive/adaptive/AdaptiveParameterUpdateModule.h>

#include <ocs2_legged_robot_adaptive/adaptive/AdaptiveReferenceKinematics.h>
#include <ocs2_legged_robot_adaptive/adaptive/AdaptiveRigidBodyRegressor.h>

namespace legged {

void LeggedController::updateEthAdaptiveWbc(const vector_t& stateDesired,
                                            const vector_t& /*inputDesired*/,
                                            std::size_t /*mode*/, scalar_t /*dt*/) {
  if (!ethAdaptivePolicyBinding_) return;
  const auto snapshot = ethAdaptivePolicyBinding_->active();
  if (!snapshot || !snapshot->valid()) {
    throw std::runtime_error("no active ETH snapshot bound to active MPC policy");
  }
  const auto& info = leggedInterface_->getCentroidalModelInfo();
  const auto& settings = leggedInterface_->getEthAdaptiveSettings();
  const auto& target = mpcMrtInterface_->getCommand().mpcTargetTrajectories_;
  namespace eth = ocs2::legged_robot_adaptive::adaptive;
  eth::AdaptiveReferenceKinematicsAdapter referenceAdapter(
      leggedInterface_->getPinocchioInterface(), info, settings);
  const auto reference = referenceAdapter.compute(currentObservation_.time,
                                                  currentObservation_.state, target);
  eth::AdaptiveRigidBodyRegressor regressor(
      {info.robotMass, info.centroidalInertiaNominal,
       eth::vector3_t(0.0, 0.0, -9.81)});
  eth::AdaptiveRigidBodyRegressor::EvaluationPoint point;
  point.rotationWorldFromBase = reference.actual.orientationWorldFromBase.toRotationMatrix();
  point.actualAngularVelocityWorld = reference.actual.angularVelocityWorld;
  point.referenceLinearAccelerationWorld = reference.referenceLinearAccelerationWorld;
  point.referenceAngularVelocityWorld = reference.referenceAngularVelocityWorld;
  point.referenceAngularAccelerationWorld = reference.referenceAngularAccelerationWorld;

  // Exact ground truth for the +5 kg mass-only experiment:
  //
  //   d_u^true = Y_in(q,v,v_r,dot(v_r)) * pi_in^true
  //
  // Uses exactly the same ETH reference kinematics and the same
  // PayloadInertial10D regressor as the estimator.
  const eth::vector6_t trueYpiWorldCom =
      exact_payload_truth::trueYpiWorldCom(reference);
  const eth::vector6_t estimatedYpiWorldCom =
      regressor.computeUnknownDynamicsRegressor(
          point, settings.parameterization, settings.adaptiveDynamicsForm) *
      snapshot->estimate();

  // Eq.(10) inserts -Y*pi_hat/m into the prediction flow. WBC instead takes
  // a physical external wrench about base origin: -Y*pi_hat translated C->B.
  const Eigen::Matrix3d Rdesired =
      (Eigen::AngleAxisd(stateDesired(9), Eigen::Vector3d::UnitZ()) *
       Eigen::AngleAxisd(stateDesired(10), Eigen::Vector3d::UnitY()) *
       Eigen::AngleAxisd(stateDesired(11), Eigen::Vector3d::UnitX())).toRotationMatrix();
  Eigen::Matrix<double, 6, 1> externalWorldBase;
  externalWorldBase.head<3>() = -estimatedYpiWorldCom.head<3>();
  const Eigen::Vector3d rBaseToCom = -Rdesired * info.comToBasePositionNominal;
  externalWorldBase.tail<3>() = -estimatedYpiWorldCom.tail<3>() +
      rBaseToCom.cross(externalWorldBase.head<3>());

  if (ethAdaptiveWbcEnabled_) {
    MyRbfV2WbcInput input;
    input.enabled = true;
    input.enforceClfConstraint = false;  // Eq.(11) is a soft MPC constraint.
    input.injectExternalWrenchInDynamics = true;
    input.v11MatchedAcceleration = false;
    input.snapshotSequenceId = snapshot->version();
    input.externalWrenchEstimate = externalWorldBase;
    input.robustBound.setZero();
    wbc_->configureMyRbfV2(input);
  }

  if (ethAdaptivePublisher_ && ethAdaptivePublisher_->trylock()) {
    auto& values = ethAdaptivePublisher_->msg_.data;
    std::size_t i = 0;
    values.at(i++) = currentObservation_.time;
    values.at(i++) = snapshot->time();
    values.at(i++) = static_cast<double>(snapshot->version());
    for (Eigen::Index j = 0; j < snapshot->estimate().size(); ++j) values.at(i++) = snapshot->estimate()(j);
    for (Eigen::Index j = 0; j < 6; ++j) values.at(i++) = estimatedYpiWorldCom(j);
    for (Eigen::Index j = 0; j < 6; ++j) values.at(i++) = externalWorldBase(j);
    for (Eigen::Index j = 0; j < 6; ++j) values.at(i++) = trueYpiWorldCom(j);

    // ==========================================================
    // ETH Eq.(8) 只读诊断
    //
    // 更新律在 MPC 求解线程执行，而此处在控制线程发布。
    // 因此必须检查当前策略快照和诊断快照是否一致。
    //
    // 不一致时：
    //   diag_valid = 0
    //   具体误差/导数写 NaN，禁止后续误配分析。
    //
    // 本段绝不修改估计值、MPC、CLF 或 WBC。
    // ==========================================================

    const auto updateModule =
        leggedInterface_->getEthAdaptiveParameterUpdateModule();

    const auto diagnostic = updateModule->diagnostics();

    // ==========================================================
    // ETH 异步诊断修复
    //
    // 参数更新运行在 MPC 求解线程；
    // 此处发布运行在控制器线程。
    //
    // 两个线程的快照版本不必相同。
    //
    // diag_valid 表示诊断样本自身有效，
    // 而不是它与当前控制策略快照完全同步。
    //
    // 时间对齐应在离线分析时，依据：
    //   diag_snapshot_time
    //   diag_snapshot_version
    //   snapshot_time
    //   snapshot_version
    // 单独完成。
    // ==========================================================

    const bool diagnosticAvailable =
        diagnostic.valid &&
        std::isfinite(diagnostic.snapshotTime) &&
        std::isfinite(diagnostic.observationTime) &&
        diagnostic.rawParameterDrive.size() > 0 &&
        diagnostic.appliedParameterDrive.size() > 0 &&
        diagnostic.appliedParameterDerivative.size() > 0 &&
        diagnostic.sigma.allFinite() &&
        diagnostic.desiredComPositionWorld.allFinite() &&
        diagnostic.actualComPositionWorld.allFinite() &&
        diagnostic.desiredComVelocityWorld.allFinite() &&
        diagnostic.actualComVelocityWorld.allFinite() &&
        diagnostic.referenceLinearAccelerationWorld.allFinite() &&
        diagnostic.rawParameterDrive.allFinite() &&
        diagnostic.appliedParameterDrive.allFinite() &&
        diagnostic.appliedParameterDerivative.allFinite();

    // 禁止发布 NaN。
    // diag_valid=0 时，数值为占位值0，离线分析必须过滤。
    const auto safeValue = [diagnosticAvailable](double x) {
        return (diagnosticAvailable && std::isfinite(x))
                   ? x : 0.0;
    };

    // 4个元数据字段：始终发布，供离线核查。
    values.at(i++) = diagnosticAvailable ? 1.0 : 0.0;
    values.at(i++) = diagnostic.snapshotTime;
    values.at(i++) = diagnostic.observationTime;
    values.at(i++) = static_cast<double>(
        diagnostic.snapshotVersion
    );

    // 1-4：期望/实际质心高度和竖直速度（WORLD）。
    values.at(i++) = safeValue(
        diagnostic.desiredComPositionWorld.z()
    );
    values.at(i++) = safeValue(
        diagnostic.actualComPositionWorld.z()
    );
    values.at(i++) = safeValue(
        diagnostic.desiredComVelocityWorld.z()
    );
    values.at(i++) = safeValue(
        diagnostic.actualComVelocityWorld.z()
    );

    // 5-7：真正进入 Eq.(8) 的 WORLD 滑模变量。
    values.at(i++) = safeValue(diagnostic.sigma(0));
    values.at(i++) = safeValue(diagnostic.sigma(1));
    values.at(i++) = safeValue(diagnostic.sigma(2));

    // 8-10：构建质量回归列的参考加速度。
    values.at(i++) = safeValue(
        diagnostic.referenceLinearAccelerationWorld.x()
    );
    values.at(i++) = safeValue(
        diagnostic.referenceLinearAccelerationWorld.y()
    );
    values.at(i++) = safeValue(
        diagnostic.referenceLinearAccelerationWorld.z()
    );

    // 11-13：质量参数更新的原始驱动、实际驱动和导数。
    //
    // pi[0] 是质量参数：
    //   raw drive       = Y_m^T * sigma
    //   applied drive   = 扣除可选基线修正后的驱动
    //   mass derivative = Gamma_m * applied drive
    values.at(i++) = safeValue(
        diagnostic.rawParameterDrive(0)
    );
    values.at(i++) = safeValue(
        diagnostic.appliedParameterDrive(0)
    );
    values.at(i++) = safeValue(
        diagnostic.appliedParameterDerivative(0)
    );

    // 14-16：判断更新开关和参考方式。
    values.at(i++) = safeValue(
        diagnostic.correctionApplied ? 1.0 : 0.0
    );
    values.at(i++) = safeValue(
        diagnostic.parameterUpdateEnabled ? 1.0 : 0.0
    );
    values.at(i++) = safeValue(
        diagnostic.encodedReferenceDerivativesEnabled ? 1.0 : 0.0
    );

    ethAdaptivePublisher_->unlockAndPublish();
  }
}

}  // namespace legged
