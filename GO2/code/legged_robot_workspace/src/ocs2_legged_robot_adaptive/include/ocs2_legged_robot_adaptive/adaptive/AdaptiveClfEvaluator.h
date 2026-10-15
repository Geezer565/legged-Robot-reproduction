#pragma once

#include <memory>

#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_oc/synchronized_module/ReferenceManagerInterface.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include "ocs2_legged_robot_adaptive/adaptive/AdaptiveClf.h"
#include "ocs2_legged_robot_adaptive/adaptive/AdaptiveModelSnapshot.h"
#include "ocs2_legged_robot_adaptive/adaptive/AdaptiveReferenceKinematics.h"
#include "ocs2_legged_robot_adaptive/adaptive/AdaptiveRigidBodyRegressor.h"
#include "ocs2_legged_robot_adaptive/adaptive/PayloadMass1D.h"

namespace ocs2 {
namespace legged_robot_adaptive {
namespace adaptive {

// ============================================================
// 论文 Eq.(11) 的完整一次评价结果。
//
// 所有 wrench 均在 world frame 表达，顺序固定为
// [Fx, Fy, Fz, Tx, Ty, Tz]；后三维 torque 均关于系统 COM。
// observationTime/snapshotTime 不是论文变量，而是 executed 旁路诊断的
// 工程同步证据：只有两者时间匹配时，才允许把 pi_hat 用于真实 observation。
// ============================================================
struct AdaptiveClfDiagnostics {
  vector6_t velocityErrorWorld = vector6_t::Zero();
  vector6_t lambdaPoseErrorWorld = vector6_t::Zero();
  vector6_t sigma = vector6_t::Zero();
  parameter_vector_t piHat = parameter_vector_t::Zero(6);
  vector6_t contactWrench = vector6_t::Zero();
  vector6_t nominalReferenceWrench = vector6_t::Zero();
  regressor_t unknownReferenceRegressor = regressor_t::Identity(6, 6);
  vector6_t estimatedUnknownWrench = vector6_t::Zero();
  vector_t inertialParameterEstimate = vector_t::Zero(0);
  vector6_t inertialEstimatedWrench = vector6_t::Zero();
  vector6_t constantWrenchEstimate = vector6_t::Zero();
  /** Wrench whose -w/m_nominal mapping is actually applied by Eq.(10). */
  vector6_t adaptiveDynamicsEquivalentWrench = vector6_t::Zero();
  vector6_t candidateDynamicsCorrection = vector6_t::Zero();
  parameter_vector_t parameterDerivative = parameter_vector_t::Zero(6);
  vector6_t residualWrench = vector6_t::Zero();
  vector6_t clfContribution = vector6_t::Zero();
  scalar_t clfLinearTerm = 0.0;
  scalar_t dampingTerm = 0.0;
  scalar_t reconstructedH = 0.0;
  scalar_t hClf = 0.0;
  scalar_t violation = 0.0;
  scalar_t observationTime = 0.0;
  scalar_t snapshotTime = 0.0;
  scalar_t timeError = 0.0;
  scalar_t payloadMassEstimate = 0.0;
  scalar_t estimatedTotalMass = 0.0;
  scalar_t estimatedMassRatio = 0.0;
  /** PayloadInertial10D uses a full estimated model flow, not -w/m. */
  bool usesFullFlowAdaptiveDynamics = false;
  AdaptiveParameterization parameterization =
      AdaptiveParameterization::ConstantWrench6D;
};

/**
 * 唯一的 Adaptive CLF 数学评价器。
 *
 * constraint 与真实 observation monitor 都通过这里计算 Eq.(11)，避免两处
 * 分别实现 sigma/contact wrench/nominal wrench 而发生符号漂移。它不持有
 * ROS publisher，也不写 solver diagnostics；因此 executed 旁路评价不会污染
 * DDP rollout 中的 latest constraint diagnostics。
 */
class AdaptiveClfEvaluator final {
 public:
  AdaptiveClfEvaluator(const PinocchioInterface& pinocchioInterface, CentroidalModelInfo info,
                       const ReferenceManagerInterface& referenceManager,
                       AdaptiveClfSettings settings);
  /** OCS2 会 clone constraint；复制时重建每个 clone 自己的 Pinocchio state。 */
  AdaptiveClfEvaluator(const AdaptiveClfEvaluator& other);

  /** 用调用者指定的 immutable snapshot 计算 Eq.(11)。 */
  AdaptiveClfDiagnostics evaluate(scalar_t time, const vector_t& state, const vector_t& input,
                                  const AdaptiveModelSnapshotPtr& snapshot) const;

  /**
   * executed monitor 的同步保护：绝不以未来或上一帧 pi_hat 静默评价 observation。
   * 若时间不匹配返回 false，调用者应保留 observation，等待对应 Eq.(8) snapshot。
   */
  static bool isSnapshotTimeSynchronized(scalar_t observationTime, scalar_t snapshotTime, scalar_t tolerance);

 private:
  mutable PinocchioInterface pinocchioInterface_;
  CentroidalModelInfo info_;
  mutable CentroidalModelPinocchioMapping mapping_;
  const ReferenceManagerInterface* referenceManagerPtr_;
  mutable AdaptiveReferenceKinematicsAdapter referenceKinematics_;
  SlidingVariable slidingVariable_;
  AdaptiveRigidBodyRegressor regressor_;
  AdaptiveClf clf_;
  AdaptiveClfSettings settings_;
};

}  // namespace adaptive
}  // namespace legged_robot_adaptive
}  // namespace ocs2
