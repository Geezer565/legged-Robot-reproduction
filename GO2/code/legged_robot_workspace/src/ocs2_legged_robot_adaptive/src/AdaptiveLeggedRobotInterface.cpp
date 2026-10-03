#include "ocs2_legged_robot_adaptive/AdaptiveLeggedRobotInterface.h"

#include "ocs2_legged_robot_adaptive/dynamics/AdaptiveLeggedRobotDynamics.h"
#include "ocs2_legged_robot_adaptive/package_path.h"

#include <ocs2_core/soft_constraint/StateInputSoftConstraint.h>

namespace ocs2 {
namespace legged_robot_adaptive {

AdaptiveLeggedRobotInterface::AdaptiveLeggedRobotInterface(const std::string& taskFile, const std::string& urdfFile,
                                                           const std::string& referenceFile,
                                                           bool useHardFrictionConeConstraint,
                                                           const std::string& adaptiveSettingsFile)
    : baseInterfacePtr_(new legged_robot::LeggedRobotInterface(taskFile, urdfFile, referenceFile, useHardFrictionConeConstraint)),
      problemPtr_(new OptimalControlProblem(baseInterfacePtr_->getOptimalControlProblem())) {
  adaptiveSettingsFile_ = adaptiveSettingsFile.empty()
                              ? getPath() + "/config/adaptive_settings.info"
                              : adaptiveSettingsFile;
  adaptiveSettings_ = adaptive::loadAdaptiveClfSettings(adaptiveSettingsFile_);
  const auto& settings = adaptiveSettings_;

  const auto initialSnapshot = adaptive::AdaptiveModelSnapshot::createValid(
      settings.initialParameterEstimate, 0.0, 1);
  modelDataPtr_ = std::make_shared<adaptive::AdaptiveModelData>(initialSnapshot);
  parameterUpdateModulePtr_ = std::make_shared<adaptive::AdaptiveParameterUpdateModule>(
      baseInterfacePtr_->getPinocchioInterface(), baseInterfacePtr_->getCentroidalModelInfo(), settings, modelDataPtr_);
  const auto rbfSettings =
      adaptive::loadRbfWrenchClosedLoopSettings(adaptiveSettingsFile_);
  if (rbfSettings.enabled) {
    rbfWrenchClosedLoopUpdateModulePtr_ =
        std::make_shared<adaptive::RbfWrenchClosedLoopUpdateModule>(
            baseInterfacePtr_->getPinocchioInterface(),
            baseInterfacePtr_->getCentroidalModelInfo(), settings, rbfSettings,
            modelDataPtr_);
  }

  // executed diagnostics 必须独立于 enableClfConstraint：OFF 组也需要以完全相同
  // 的 Eq.(11) 旁路评价实际 observation，才能与 ON 组公平对比。
  clfEvaluatorPtr_ = std::make_shared<adaptive::AdaptiveClfEvaluator>(
      baseInterfacePtr_->getPinocchioInterface(), baseInterfacePtr_->getCentroidalModelInfo(),
      *baseInterfacePtr_->getSwitchedModelReferenceManagerPtr(), settings);

  // Task B 的三组必须只由两个 mode bit 决定。即使 Eq.(10) 关闭，Eq.(8)
  // synchronized module、frozen snapshot、Eq.(11) 与 executed diagnostics 仍保持
  // 完全相同的装配；只有 full_adaptive 使用 adaptive dynamics wrapper。
  if (settings.enableAdaptiveDynamics) {
    problemPtr_->dynamicsPtr = std::make_unique<AdaptiveLeggedRobotDynamics>(
        baseInterfacePtr_->getPinocchioInterface(), baseInterfacePtr_->getCentroidalModelInfo(), "dynamics",
        baseInterfacePtr_->modelSettings(), settings, modelDataPtr_,
        baseInterfacePtr_->getSwitchedModelReferenceManagerPtr().get());
    rolloutPtr_ = std::make_unique<TimeTriggeredRollout>(*problemPtr_->dynamicsPtr, baseInterfacePtr_->rolloutSettings());
  }
  if (settings.enableClfConstraint) {
    clfConstraintPtr_ = std::make_shared<AdaptiveClfConstraint>(
        baseInterfacePtr_->getPinocchioInterface(), baseInterfacePtr_->getCentroidalModelInfo(),
        *baseInterfacePtr_->getSwitchedModelReferenceManagerPtr(), settings, modelDataPtr_);
    const RelaxedBarrierPenalty::Config penalty{settings.softConstraintMu, settings.softConstraintDelta};
    problemPtr_->softConstraintPtr->add(
        "adaptiveClfEq11", std::make_unique<StateInputSoftConstraint>(
            std::unique_ptr<StateInputConstraint>(clfConstraintPtr_->clone()),
            std::make_unique<RelaxedBarrierPenalty>(penalty)));
  }
}

}  // namespace legged_robot_adaptive
}  // namespace ocs2
