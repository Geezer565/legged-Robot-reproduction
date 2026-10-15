#include "ocs2_legged_robot_adaptive/adaptive/AdaptiveClfEvaluator.h"

#include <cmath>
#include <stdexcept>

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>

namespace ocs2 {
namespace legged_robot_adaptive {
namespace adaptive {

AdaptiveClfEvaluator::AdaptiveClfEvaluator(const PinocchioInterface& pinocchioInterface, CentroidalModelInfo info,
                                           const ReferenceManagerInterface& referenceManager,
                                           AdaptiveClfSettings settings)
    : pinocchioInterface_(pinocchioInterface),
      info_(std::move(info)),
      mapping_(info_),
      referenceManagerPtr_(&referenceManager),
      referenceKinematics_(pinocchioInterface, info_, settings),
      slidingVariable_(settings),
      regressor_({info_.robotMass, info_.centroidalInertiaNominal, vector3_t(0.0, 0.0, -9.81)}),
      clf_(settings),
      settings_(std::move(settings)) {
  mapping_.setPinocchioInterface(pinocchioInterface_);
}

AdaptiveClfEvaluator::AdaptiveClfEvaluator(const AdaptiveClfEvaluator& other)
    : pinocchioInterface_(other.pinocchioInterface_),
      info_(other.info_),
      mapping_(info_),
      referenceManagerPtr_(other.referenceManagerPtr_),
      // AdaptiveStateAdapter 内含不可复制的 Pinocchio mapping；clone 必须从
      // PinocchioInterface 重建它，不能共享 solver/monitor 的可变运动学数据。
      referenceKinematics_(other.referenceKinematics_),
      slidingVariable_(other.slidingVariable_),
      regressor_(other.regressor_),
      clf_(other.clf_),
      settings_(other.settings_) {
  mapping_.setPinocchioInterface(pinocchioInterface_);
}

AdaptiveClfDiagnostics AdaptiveClfEvaluator::evaluate(scalar_t time, const vector_t& state, const vector_t& input,
                                                       const AdaptiveModelSnapshotPtr& snapshot) const {
  if (!snapshot || !snapshot->valid()) {
    throw std::invalid_argument("[AdaptiveClfEvaluator] Eq.(11) 需要有效的 immutable adaptive snapshot。");
  }

  AdaptiveClfDiagnostics diagnostics;
  diagnostics.observationTime = time;
  diagnostics.snapshotTime = snapshot->time();
  diagnostics.timeError = diagnostics.snapshotTime - diagnostics.observationTime;
  diagnostics.piHat = snapshot->estimate();
  if (diagnostics.piHat.size() != static_cast<Eigen::Index>(
          parameterDimension(settings_.parameterization))) {
    throw std::invalid_argument(
        "[AdaptiveClfEvaluator] snapshot dimension does not match parameterization.");
  }
  diagnostics.parameterDerivative =
      parameter_vector_t::Zero(diagnostics.piHat.size());
  diagnostics.parameterization = settings_.parameterization;

  // Eq.(8)/Eq.(11) 共用的 pose-derivative reference。physical v_d 不读取
  // TargetTrajectories state[0:3]，而是从 q_d(t) 的 COM trajectory 恢复。
  const auto reference = referenceKinematics_.compute(time, state, referenceManagerPtr_->getTargetTrajectories());
  const auto sliding = slidingVariable_.compute(reference.actual, reference.desired);
  diagnostics.velocityErrorWorld = sliding.velocityErrorWorld;
  diagnostics.lambdaPoseErrorWorld = sliding.lambdaPoseErrorWorld;
  diagnostics.sigma = sliding.sigma;

  // Eq.(11) 的 w_contact：输入中的每个 3DoF contact force 由 OCS2 mapping
  // 读取，故不假定 four legs/12D input。lever 与 force 均为 world frame，
  // r_COM_to_contact x f 得到关于系统 COM 的 world-frame torque。
  const vector_t q = mapping_.getPinocchioJointPosition(state);
  updateCentroidalDynamics(pinocchioInterface_, info_, q);
  for (size_t i = 0; i < info_.numThreeDofContacts; ++i) {
    const auto force = centroidal_model::getContactForces(input, i, info_);
    const auto lever = getPositionComToContactPointInWorldFrame(pinocchioInterface_, info_, i);
    diagnostics.contactWrench.head<3>() += force;
    diagnostics.contactWrench.tail<3>() += lever.cross(force);
  }

  AdaptiveRigidBodyRegressor::EvaluationPoint point;
  point.rotationWorldFromBase = reference.actual.orientationWorldFromBase.toRotationMatrix();
  point.actualAngularVelocityWorld = reference.actual.angularVelocityWorld;
  // 论文 nominal term 使用统一 tracking reference：dot(v_r)、omega_r、
  // dot(omega_r)。standing 时这些量为零，自动回归 [0,0,mg,0,0,0]。
  point.referenceLinearAccelerationWorld = reference.referenceLinearAccelerationWorld;
  point.referenceAngularVelocityWorld = reference.referenceAngularVelocityWorld;
  point.referenceAngularAccelerationWorld = reference.referenceAngularAccelerationWorld;
  diagnostics.nominalReferenceWrench = regressor_.computeNominalReferenceWrench(point);
  diagnostics.unknownReferenceRegressor =
      regressor_.computeUnknownReferenceRegressor(
          point, settings_.parameterization, settings_.adaptiveDynamicsForm);
  diagnostics.estimatedUnknownWrench =
      diagnostics.unknownReferenceRegressor * diagnostics.piHat;

  if (settings_.parameterization ==
      AdaptiveParameterization::ConstantWrench6D) {
    diagnostics.adaptiveDynamicsEquivalentWrench = diagnostics.piHat.head<6>();
    diagnostics.parameterDerivative =
        settings_.gamma * diagnostics.sigma;
  } else if (settings_.parameterization ==
             AdaptiveParameterization::PayloadMass1D) {
    diagnostics.payloadMassEstimate = diagnostics.piHat(0);
    diagnostics.estimatedTotalMass = PayloadMass1D::estimatedTotalMass(
        info_.robotMass, diagnostics.payloadMassEstimate,
        settings_.minimumEstimatedTotalMass);
    diagnostics.estimatedMassRatio = PayloadMass1D::estimatedMassRatio(
        info_.robotMass, diagnostics.payloadMassEstimate,
        settings_.minimumEstimatedTotalMass);
    diagnostics.adaptiveDynamicsEquivalentWrench =
        PayloadMass1D::dynamicsEquivalentWrench(
            diagnostics.contactWrench, info_.robotMass,
            diagnostics.payloadMassEstimate,
            settings_.minimumEstimatedTotalMass);
    diagnostics.parameterDerivative(0) =
        settings_.payloadMassAdaptationGain *
        diagnostics.unknownReferenceRegressor.col(0).dot(
            diagnostics.sigma);
  } else {
    diagnostics.adaptiveDynamicsEquivalentWrench =
        diagnostics.estimatedUnknownWrench;
    diagnostics.parameterDerivative =
        settings_.gamma * diagnostics.unknownReferenceRegressor.transpose() *
        diagnostics.sigma;
    diagnostics.usesFullFlowAdaptiveDynamics =
        settings_.adaptiveDynamicsForm ==
        AdaptiveDynamicsForm::LegacyEstimatedModel;
    if (settings_.parameterization == AdaptiveParameterization::PayloadInertial10DPlusWrench6D) {
      // Eq.(11): w_hat_u=Y_in*pi_in_hat+pi_f_hat; publish both addends.
      diagnostics.inertialParameterEstimate = diagnostics.piHat.head<10>();
      diagnostics.inertialEstimatedWrench = diagnostics.unknownReferenceRegressor.leftCols(10) * diagnostics.piHat.head<10>();
      diagnostics.constantWrenchEstimate =
          diagnostics.unknownReferenceRegressor.rightCols(6) *
          diagnostics.piHat.tail<6>();
    }
  }
  if (!diagnostics.usesFullFlowAdaptiveDynamics) {
    diagnostics.candidateDynamicsCorrection =
        -diagnostics.adaptiveDynamicsEquivalentWrench / info_.robotMass;
  }

  AdaptiveClfInput inputData;
  inputData.sigma = diagnostics.sigma;
  inputData.contactWrenchWorld = diagnostics.contactWrench;
  inputData.nominalReferenceWrenchWorld = diagnostics.nominalReferenceWrench;
  inputData.parameterEstimate = diagnostics.piHat;
  inputData.unknownRegressor = diagnostics.unknownReferenceRegressor;
  diagnostics.hClf = clf_.computeValue(inputData);

  // Eq.(11) 的精确分解：linear term = sigma^T residual，damping term =
  // 0.5 sigma^T K_D sigma。这里只生成诊断量，既不改变 constraint 也不调参。
  diagnostics.residualWrench = diagnostics.contactWrench - diagnostics.nominalReferenceWrench -
                               diagnostics.estimatedUnknownWrench;
  diagnostics.clfContribution = diagnostics.sigma.cwiseProduct(diagnostics.residualWrench);
  diagnostics.clfLinearTerm = diagnostics.clfContribution.sum();
  diagnostics.dampingTerm = 0.5 * diagnostics.sigma.dot(clf_.kd() * diagnostics.sigma);
  diagnostics.reconstructedH = diagnostics.clfLinearTerm - diagnostics.dampingTerm;
  diagnostics.violation = std::max<scalar_t>(0.0, -diagnostics.hClf);

  // 非论文项：仅防止 Eq.(11) 实现与诊断分解漂移；不是额外 constraint 或 penalty。
  const scalar_t tolerance = 1.0e-12 * (1.0 + std::abs(diagnostics.hClf));
  if (std::abs(diagnostics.reconstructedH - diagnostics.hClf) > tolerance) {
    throw std::runtime_error("[AdaptiveClfEvaluator] Eq.(11) diagnostics reconstruction mismatch.");
  }
  return diagnostics;
}

bool AdaptiveClfEvaluator::isSnapshotTimeSynchronized(scalar_t observationTime, scalar_t snapshotTime,
                                                       scalar_t tolerance) {
  return std::isfinite(observationTime) && std::isfinite(snapshotTime) && std::isfinite(tolerance) && tolerance >= 0.0 &&
         std::abs(snapshotTime - observationTime) <= tolerance;
}

}  // namespace adaptive
}  // namespace legged_robot_adaptive
}  // namespace ocs2
