#pragma once

#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>

#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>

#include "ocs2_legged_robot_adaptive/adaptive/AdaptiveModelData.h"
#include "ocs2_legged_robot_adaptive/adaptive/AdaptiveParameterEstimator.h"
#include "ocs2_legged_robot_adaptive/adaptive/AdaptiveRigidBodyRegressor.h"
#include "ocs2_legged_robot_adaptive/adaptive/AdaptiveReferenceKinematics.h"
#include "ocs2_legged_robot_adaptive/adaptive/CompositePhaseBaselineCorrector.h"
#include "ocs2_legged_robot_adaptive/adaptive/GaitPhaseBaselineCorrector.h"

namespace ocs2 {
namespace legged_robot_adaptive {
namespace adaptive {

/** Exact synchronized Eq.(8)/D5H1 update sample published read-only to ROS. */
struct AdaptiveParameterUpdateDiagnostics {
  bool valid = false;
  bool parameterUpdateEnabled = false;
  bool gaitPhaseBaselineConfigured = false;
  bool compositePhaseBaselineConfigured = false;
  bool compositeSteadyStageActive = false;
  bool compositeKeySupported = false;
  bool compositeGaitPhaseValid = false;
  bool phaseValid = false;
  bool correctionApplied = false;
  bool cycleBoundary = false;
  scalar_t observationTime = 0.0;
  scalar_t snapshotTime = 0.0;
  scalar_t normalizedPhase = -1.0;
  scalar_t cycleStart = -1.0;
  std::uint64_t snapshotVersion = 0;
  std::size_t mode = 0;
  int phaseBin = -1;
  int compositeProfilePhaseBin = -1;
  int compositeGaitPhaseBin = -1;
  scalar_t compositeProfilePhase = -1.0;
  scalar_t compositeGaitPhase = -1.0;
  scalar_t compositeCycleStart = -1.0;
  bool encodedReferenceDerivativesEnabled = false;
  vector6_t sigma = vector6_t::Zero();
  vector3_t desiredComPositionWorld = vector3_t::Zero();
  // 诊断专用：与 desired 由同一次参考运动学计算得到。
  // WORLD 坐标系；不影响任何自适应或控制公式。
  vector3_t actualComPositionWorld = vector3_t::Zero();
  vector3_t actualComVelocityWorld = vector3_t::Zero();
  quaternion_t desiredOrientationWorldFromBase = quaternion_t::Identity();
  vector3_t desiredComVelocityWorld = vector3_t::Zero();
  vector3_t desiredComAccelerationWorld = vector3_t::Zero();
  vector3_t desiredAngularVelocityWorld = vector3_t::Zero();
  vector3_t desiredAngularAccelerationWorld = vector3_t::Zero();
  vector3_t referenceLinearVelocityWorld = vector3_t::Zero();
  vector3_t referenceLinearAccelerationWorld = vector3_t::Zero();
  vector3_t referenceAngularVelocityWorld = vector3_t::Zero();
  vector3_t referenceAngularAccelerationWorld = vector3_t::Zero();
  parameter_vector_t rawParameterDrive = parameter_vector_t::Zero(6);
  parameter_vector_t nominalDriveBaseline = parameter_vector_t::Zero(6);
  parameter_vector_t appliedParameterDrive = parameter_vector_t::Zero(6);
  parameter_vector_t appliedParameterDerivative = parameter_vector_t::Zero(6);
};

/** 在每个新 observation 运行论文 Eq.(8)，并将结果冻结给同轮 prediction dynamics。 */
class AdaptiveParameterUpdateModule final : public SolverSynchronizedModule {
 public:
  AdaptiveParameterUpdateModule(const PinocchioInterface& pinocchioInterface,
                                CentroidalModelInfo centroidalModelInfo,
                                AdaptiveClfSettings settings,
                                std::shared_ptr<AdaptiveModelData> modelData);

  void preSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t& initState,
                    const ReferenceManagerInterface& referenceManager) override;
  void postSolverRun(const PrimalSolution&) override {}

  AdaptiveModelSnapshotPtr snapshot() const { return estimator_.snapshot(); }
  AdaptiveParameterUpdateDiagnostics diagnostics() const;
  static bool isDuplicateTimestamp(scalar_t currentTime, scalar_t previousTime, scalar_t tolerance);

 private:
  AdaptiveClfSettings settings_;
  AdaptiveReferenceKinematicsAdapter referenceKinematics_;
  SlidingVariable slidingVariable_;
  AdaptiveRigidBodyRegressor regressor_;
  AdaptiveParameterEstimator estimator_;
  std::unique_ptr<GaitPhaseBaselineCorrector> gaitPhaseBaselineCorrector_;
  std::unique_ptr<CompositePhaseBaselineCorrector>
      compositePhaseBaselineCorrector_;
  std::shared_ptr<AdaptiveModelData> modelData_;
  scalar_t lastProcessedTime_ = std::numeric_limits<scalar_t>::quiet_NaN();
  mutable std::mutex diagnosticsMutex_;
  AdaptiveParameterUpdateDiagnostics diagnostics_;
};

}  // namespace adaptive
}  // namespace legged_robot_adaptive
}  // namespace ocs2
