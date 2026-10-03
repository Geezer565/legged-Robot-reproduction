#pragma once

#include <memory>
#include <string>

#include <ocs2_core/Types.h>
#include <ocs2_core/initialization/Initializer.h>
#include <ocs2_core/penalties/Penalties.h>
#include <ocs2_ddp/DDP_Settings.h>
#include <ocs2_ipm/IpmSettings.h>
#include <ocs2_mpc/MPC_Settings.h>
#include <ocs2_oc/oc_problem/OptimalControlProblem.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>
#include <ocs2_robotic_tools/common/RobotInterface.h>
#include <ocs2_sqp/SqpSettings.h>

#include <ocs2_legged_robot/LeggedRobotInterface.h>

#include "ocs2_legged_robot_adaptive/adaptive/AdaptiveModelData.h"
#include "ocs2_legged_robot_adaptive/adaptive/AdaptiveParameterUpdateModule.h"
#include "ocs2_legged_robot_adaptive/adaptive/AdaptiveClfEvaluator.h"
#include "ocs2_legged_robot_adaptive/adaptive/AdaptiveTypes.h"
#include "ocs2_legged_robot_adaptive/adaptive/RbfWrenchClosedLoopUpdateModule.h"
#include "ocs2_legged_robot_adaptive/constraint/AdaptiveClfConstraint.h"

namespace ocs2 {
namespace legged_robot_adaptive {

class AdaptiveLeggedRobotInterface final : public RobotInterface {
 public:
  AdaptiveLeggedRobotInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                               bool useHardFrictionConeConstraint = false,
                               const std::string& adaptiveSettingsFile = "");

  ~AdaptiveLeggedRobotInterface() override = default;

  const OptimalControlProblem& getOptimalControlProblem() const override { return *problemPtr_; }
  const Initializer& getInitializer() const override { return baseInterfacePtr_->getInitializer(); }
  std::shared_ptr<ReferenceManagerInterface> getReferenceManagerPtr() const override { return baseInterfacePtr_->getReferenceManagerPtr(); }

  const ddp::Settings& ddpSettings() const { return baseInterfacePtr_->ddpSettings(); }
  const mpc::Settings& mpcSettings() const { return baseInterfacePtr_->mpcSettings(); }
  const rollout::Settings& rolloutSettings() const { return baseInterfacePtr_->rolloutSettings(); }
  const sqp::Settings& sqpSettings() const { return baseInterfacePtr_->sqpSettings(); }
  const ipm::Settings& ipmSettings() const { return baseInterfacePtr_->ipmSettings(); }

  const vector_t& getInitialState() const { return baseInterfacePtr_->getInitialState(); }
  const RolloutBase& getRollout() const { return rolloutPtr_ ? *rolloutPtr_ : baseInterfacePtr_->getRollout(); }
  const legged_robot::ModelSettings& modelSettings() const { return baseInterfacePtr_->modelSettings(); }
  const CentroidalModelInfo& getCentroidalModelInfo() const { return baseInterfacePtr_->getCentroidalModelInfo(); }
  const PinocchioInterface& getPinocchioInterface() const { return baseInterfacePtr_->getPinocchioInterface(); }
  std::shared_ptr<legged_robot::SwitchedModelReferenceManager> getSwitchedModelReferenceManagerPtr() const {
    return baseInterfacePtr_->getSwitchedModelReferenceManagerPtr();
  }
  std::shared_ptr<adaptive::AdaptiveParameterUpdateModule> getAdaptiveParameterUpdateModulePtr() const {
    return parameterUpdateModulePtr_;
  }
  std::shared_ptr<adaptive::RbfWrenchClosedLoopUpdateModule>
  getRbfWrenchClosedLoopUpdateModulePtr() const {
    return rbfWrenchClosedLoopUpdateModulePtr_;
  }
  std::shared_ptr<AdaptiveClfConstraint> getAdaptiveClfConstraintPtr() const { return clfConstraintPtr_; }
  /**
   * executed Eq.(11) 旁路评价器。即使 enableClfConstraint=false 也始终创建，
   * 因此 ON/OFF 比较只改变 constraint 是否进入 OCP，不改变观测诊断数学。
   */
  std::shared_ptr<adaptive::AdaptiveClfEvaluator> getAdaptiveClfEvaluatorPtr() const { return clfEvaluatorPtr_; }
  std::shared_ptr<const adaptive::AdaptiveModelData> getAdaptiveModelDataPtr() const { return modelDataPtr_; }
  const adaptive::AdaptiveClfSettings& getAdaptiveClfSettings() const { return adaptiveSettings_; }
  const std::string& getAdaptiveSettingsFile() const { return adaptiveSettingsFile_; }

 private:
  std::unique_ptr<legged_robot::LeggedRobotInterface> baseInterfacePtr_;
  std::unique_ptr<OptimalControlProblem> problemPtr_;
  std::unique_ptr<TimeTriggeredRollout> rolloutPtr_;
  adaptive::AdaptiveClfSettings adaptiveSettings_;
  std::string adaptiveSettingsFile_;
  std::shared_ptr<adaptive::AdaptiveModelData> modelDataPtr_;
  std::shared_ptr<adaptive::AdaptiveParameterUpdateModule> parameterUpdateModulePtr_;
  std::shared_ptr<adaptive::RbfWrenchClosedLoopUpdateModule>
      rbfWrenchClosedLoopUpdateModulePtr_;
  std::shared_ptr<adaptive::AdaptiveClfEvaluator> clfEvaluatorPtr_;
  std::shared_ptr<AdaptiveClfConstraint> clfConstraintPtr_;
};

}  // namespace legged_robot_adaptive
}  // namespace ocs2
