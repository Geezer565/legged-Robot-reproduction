#include "ocs2_legged_robot_adaptive/adaptive/AdaptiveParameterUpdateModule.h"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace ocs2 {
namespace legged_robot_adaptive {
namespace adaptive {

AdaptiveParameterUpdateModule::AdaptiveParameterUpdateModule(
    const PinocchioInterface& pinocchioInterface, CentroidalModelInfo centroidalModelInfo,
    AdaptiveClfSettings settings, std::shared_ptr<AdaptiveModelData> modelData)
    : settings_(std::move(settings)),
      referenceKinematics_(pinocchioInterface, centroidalModelInfo, settings_),
      slidingVariable_(settings_),
      regressor_({centroidalModelInfo.robotMass,
                  centroidalModelInfo.centroidalInertiaNominal,
                  vector3_t(0.0, 0.0, -9.81)}),
      estimator_(settings_),
      modelData_(std::move(modelData)) {
  validateAdaptiveClfSettings(settings_);
  if (!modelData_) {
    throw std::invalid_argument("[AdaptiveParameterUpdateModule] modelData 不可为空。");
  }
  if (settings_.enableGaitPhaseBaselineCorrection) {
    gaitPhaseBaselineCorrector_.reset(new GaitPhaseBaselineCorrector(
        settings_.gaitPhaseBaselineFile, settings_.gaitPhaseBins,
        settings_.gaitCycleDuration,
        settings_.gaitBaselineMinimumCycleStartTime));
  }
  if (settings_.enableCompositePhaseBaselineCorrection) {
    compositePhaseBaselineCorrector_.reset(
        new CompositePhaseBaselineCorrector(
            settings_.compositePhaseBaselineFile,
            settings_.compositeProfilePhaseBins,
            settings_.compositeGaitPhaseBins,
            settings_.compositeProfilePeriod,
            settings_.compositeProfileStartTime,
            settings_.compositeRampInDuration,
            settings_.compositeSteadyDuration,
            settings_.compositeGaitCycleDuration));
  }
  const Eigen::Index p = static_cast<Eigen::Index>(
      parameterDimension(settings_.parameterization));
  diagnostics_.rawParameterDrive = parameter_vector_t::Zero(p);
  diagnostics_.nominalDriveBaseline = parameter_vector_t::Zero(p);
  diagnostics_.appliedParameterDrive = parameter_vector_t::Zero(p);
  diagnostics_.appliedParameterDerivative = parameter_vector_t::Zero(p);
}

bool AdaptiveParameterUpdateModule::isDuplicateTimestamp(scalar_t currentTime, scalar_t previousTime,
                                                          scalar_t tolerance) {
  return std::isfinite(previousTime) && std::abs(currentTime - previousTime) <= tolerance;
}

AdaptiveParameterUpdateDiagnostics
AdaptiveParameterUpdateModule::diagnostics() const {
  std::lock_guard<std::mutex> lock(diagnosticsMutex_);
  return diagnostics_;
}

void AdaptiveParameterUpdateModule::preSolverRun(scalar_t initTime, scalar_t finalTime,
                                                 const vector_t& initState,
                                                 const ReferenceManagerInterface& referenceManager) {
  static_cast<void>(finalTime);
  if (!std::isfinite(initTime)) {
    throw std::invalid_argument("[AdaptiveParameterUpdateModule] initTime 非有限。");
  }

  if (std::isfinite(lastProcessedTime_)) {
    const scalar_t elapsed = initTime - lastProcessedTime_;
    if (elapsed < -settings_.timestampTolerance) {
      throw std::runtime_error("[AdaptiveParameterUpdateModule] observation timestamp 倒退，拒绝积分 Eq.(8)。");
    }
    if (isDuplicateTimestamp(initTime, lastProcessedTime_, settings_.timestampTolerance)) {
      // OCS2 可能为同一 observation 多次调用 preSolverRun；同 timestamp 必须不重复 Euler 积分。
      modelData_->publish(estimator_.snapshot());
      return;
    }
  }

  // Eq.(8) 与 Eq.(11) 共享同一 reference kinematics：moving pose 的 v_d 从
  // q_d(t) 导数恢复，不能再从原生 target 的零 momentum reference 读取。
  const auto reference = referenceKinematics_.compute(
      initTime, initState, referenceManager.getTargetTrajectories());
  const SlidingVariableResult sliding = slidingVariable_.compute(reference.actual, reference.desired);

  AdaptiveRigidBodyRegressor::EvaluationPoint point;
  point.rotationWorldFromBase =
      reference.actual.orientationWorldFromBase.toRotationMatrix();
  point.actualAngularVelocityWorld = reference.actual.angularVelocityWorld;
  point.referenceLinearAccelerationWorld =
      reference.referenceLinearAccelerationWorld;
  point.referenceAngularVelocityWorld =
      reference.referenceAngularVelocityWorld;
  point.referenceAngularAccelerationWorld =
      reference.referenceAngularAccelerationWorld;
  const regressor_t regressor = regressor_.computeUnknownDynamicsRegressor(
      point, settings_.parameterization, settings_.adaptiveDynamicsForm);
  const parameter_vector_t rawParameterDrive =
      regressor.transpose() * sliding.sigma;

  const std::size_t mode = referenceManager.getModeSchedule().modeAtTime(initTime);
  GaitPhaseBaselineSample phaseSample;
  phaseSample.mode = mode;
  if (gaitPhaseBaselineCorrector_) {
    phaseSample = gaitPhaseBaselineCorrector_->sample(initTime, mode);
  } else {
    phaseSample.baseline = parameter_vector_t::Zero(rawParameterDrive.size());
  }
  CompositePhaseBaselineSample compositeSample;
  compositeSample.mode = mode;
  if (compositePhaseBaselineCorrector_) {
    compositeSample = compositePhaseBaselineCorrector_->sample(initTime, mode);
  } else {
    compositeSample.baseline =
        parameter_vector_t::Zero(rawParameterDrive.size());
  }

  parameter_vector_t appliedParameterDrive = rawParameterDrive;
  parameter_vector_t appliedParameterDerivative =
      parameter_vector_t::Zero(rawParameterDrive.size());
  bool correctionApplied = false;

  if (!std::isfinite(lastProcessedTime_)) {
    // 首帧没有已知 dt：只把配置初值变成带 timestamp 的有效冻结快照，不能伪造 Eq.(8) 积分。
    estimator_.reset(estimator_.snapshot()->estimate(), initTime);
  } else if (settings_.enableParameterUpdate) {
    AdaptiveEstimatorUpdate update;
    update.sigma = sliding.sigma;
    update.regressor = regressor;
    update.dt = initTime - lastProcessedTime_;
    update.time = initTime;
    if (gaitPhaseBaselineCorrector_ && phaseSample.correctionActive) {
      update.nominalDriveBaseline = phaseSample.baseline;
      appliedParameterDrive -= phaseSample.baseline;
      correctionApplied = true;
    } else if (compositePhaseBaselineCorrector_ &&
               compositeSample.correctionActive) {
      update.nominalDriveBaseline = compositeSample.baseline;
      appliedParameterDrive -= compositeSample.baseline;
      correctionApplied = true;
    }
    appliedParameterDerivative = estimator_.computeParameterDerivative(update);
    estimator_.update(update);
  } else {
    estimator_.reset(estimator_.snapshot()->estimate(), initTime);
  }

  lastProcessedTime_ = initTime;
  const auto currentSnapshot = estimator_.snapshot();
  modelData_->publish(currentSnapshot);

  AdaptiveParameterUpdateDiagnostics diagnostics;
  diagnostics.valid = true;
  diagnostics.parameterUpdateEnabled = settings_.enableParameterUpdate;
  diagnostics.gaitPhaseBaselineConfigured =
      static_cast<bool>(gaitPhaseBaselineCorrector_);
  diagnostics.compositePhaseBaselineConfigured =
      static_cast<bool>(compositePhaseBaselineCorrector_);
  diagnostics.compositeSteadyStageActive =
      compositeSample.steadyStageActive;
  diagnostics.compositeKeySupported = compositeSample.keySupported;
  diagnostics.compositeGaitPhaseValid = compositeSample.gaitPhaseValid;
  diagnostics.phaseValid = phaseSample.phaseValid;
  diagnostics.correctionApplied = correctionApplied;
  diagnostics.cycleBoundary = gaitPhaseBaselineCorrector_
      ? phaseSample.cycleBoundary : compositeSample.cycleBoundary;
  diagnostics.observationTime = initTime;
  diagnostics.snapshotTime = currentSnapshot->time();
  diagnostics.normalizedPhase = phaseSample.normalizedPhase;
  diagnostics.cycleStart = phaseSample.cycleStart;
  diagnostics.snapshotVersion = currentSnapshot->version();
  diagnostics.mode = mode;
  diagnostics.phaseBin = phaseSample.phaseBin;
  diagnostics.compositeProfilePhaseBin = compositeSample.profilePhaseBin;
  diagnostics.compositeGaitPhaseBin = compositeSample.gaitPhaseBin;
  diagnostics.compositeProfilePhase = compositeSample.profilePhase;
  diagnostics.compositeGaitPhase = compositeSample.gaitPhase;
  diagnostics.compositeCycleStart = compositeSample.cycleStart;
  diagnostics.encodedReferenceDerivativesEnabled =
      settings_.enableEncodedReferenceDerivatives;
  diagnostics.sigma = sliding.sigma;
  // 只读诊断：保存同一次 Eq.(8) 更新使用的实际质心状态。
  diagnostics.actualComPositionWorld =
      reference.actual.comPositionWorld;
  diagnostics.actualComVelocityWorld =
      reference.actual.comVelocityWorld;
  diagnostics.desiredComPositionWorld =
      reference.desired.comPositionWorld;
  diagnostics.desiredOrientationWorldFromBase =
      reference.desired.orientationWorldFromBase;
  diagnostics.desiredComVelocityWorld =
      reference.desired.comVelocityWorld;
  diagnostics.desiredComAccelerationWorld =
      reference.desiredComAccelerationWorld;
  diagnostics.desiredAngularVelocityWorld =
      reference.desired.angularVelocityWorld;
  diagnostics.desiredAngularAccelerationWorld =
      reference.desiredAngularAccelerationWorld;
  diagnostics.referenceLinearVelocityWorld =
      reference.referenceLinearVelocityWorld;
  diagnostics.referenceLinearAccelerationWorld =
      reference.referenceLinearAccelerationWorld;
  diagnostics.referenceAngularVelocityWorld =
      reference.referenceAngularVelocityWorld;
  diagnostics.referenceAngularAccelerationWorld =
      reference.referenceAngularAccelerationWorld;
  diagnostics.rawParameterDrive = rawParameterDrive;
  diagnostics.nominalDriveBaseline = gaitPhaseBaselineCorrector_
      ? phaseSample.baseline : compositeSample.baseline;
  diagnostics.appliedParameterDrive = appliedParameterDrive;
  diagnostics.appliedParameterDerivative = appliedParameterDerivative;
  {
    std::lock_guard<std::mutex> lock(diagnosticsMutex_);
    diagnostics_ = std::move(diagnostics);
  }
}

}  // namespace adaptive
}  // namespace legged_robot_adaptive
}  // namespace ocs2
