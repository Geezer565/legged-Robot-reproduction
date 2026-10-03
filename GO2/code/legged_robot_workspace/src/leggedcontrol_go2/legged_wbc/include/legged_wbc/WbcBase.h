//
// Created by qiayuan on 2022/7/1.
//

#pragma once

#include "legged_wbc/Task.h"
#include "legged_wbc/MyRbfV2Wbc.h"

#include <ocs2_centroidal_model/PinocchioCentroidalDynamics.h>
#include <ocs2_legged_robot/gait/MotionPhaseDefinition.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>

namespace legged {
using namespace ocs2;
using namespace legged_robot;

// Decision Variables: x = [\dot u^T, F^T, \tau^T]^T
class WbcBase {
  using Vector6 = Eigen::Matrix<scalar_t, 6, 1>;
  using Matrix6 = Eigen::Matrix<scalar_t, 6, 6>;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  WbcBase(const PinocchioInterface& pinocchioInterface, CentroidalModelInfo info, const PinocchioEndEffectorKinematics& eeKinematics);

  virtual void loadTasksSetting(const std::string& taskFile, bool verbose);

  void configureMyRbfV2(const MyRbfV2WbcInput& input);
  bool prepareMyRbfV2Tracking(const vector_t& stateDesired, const vector_t& inputDesired,
                            const vector_t& rbdStateMeasured, size_t mode, scalar_t period);
  const MyRbfV2WbcDiagnostics& getMyRbfV2Diagnostics() const { return myRbfV2Diagnostics_; }

  virtual vector_t update(const vector_t& stateDesired, const vector_t& inputDesired, const vector_t& rbdStateMeasured, size_t mode,
                          scalar_t period);

 protected:
  void updateMeasured(const vector_t& rbdStateMeasured);
  void updateDesired(const vector_t& stateDesired, const vector_t& inputDesired);

  size_t getNumDecisionVars() const { return numDecisionVars_; }

  Task formulateFloatingBaseEomTask();
  Task formulateTorqueLimitsTask();
  Task formulateNoContactMotionTask();
  Task formulateFrictionConeTask();
  Task formulateBaseAccelTask(const vector_t& stateDesired, const vector_t& inputDesired, scalar_t period);
  Task formulateSwingLegTask();
  Task formulateContactForceTask(const vector_t& inputDesired) const;

  bool prepareMyRbfV2(const vector_t& stateDesired, const vector_t& inputDesired, scalar_t period);
  bool validateMyRbfV2Cycle(const vector_t& stateDesired, const vector_t& inputDesired,
                          const vector_t& rbdStateMeasured, scalar_t period);
  Task formulateMyRbfV2ClfTask() const;
  Task formulateMyRbfV2ForceLimitsTask() const;
  bool checkMyRbfV2Solution(const vector_t& solution, const Task& constraints);
  MyRbfV2WbcInput myRbfV2Input_;
  MyRbfV2WbcDiagnostics myRbfV2Diagnostics_;
  Matrix6 myRbfV2Transform_ = Matrix6::Identity();
  matrix3_t myRbfV2ContactFrame_ = matrix3_t::Identity();
  Vector6 myRbfV2ReferenceConstant_ = Vector6::Zero();
  matrix_t myRbfV2ReferenceJointAccel_, myRbfV2WrenchMap_;
  vector_t myRbfV2PreviousDesiredVelocity_, myRbfV2CurrentDesiredVelocity_;
  std::uint64_t myRbfV2PreviousDesiredSnapshotSequenceId_ = 0;
  scalar_t myRbfV2Period_ = 0.0;
  scalar_t myRbfV2Dissipation_ = 0.0;

  size_t numDecisionVars_;
  PinocchioInterface pinocchioInterfaceMeasured_, pinocchioInterfaceDesired_;
  CentroidalModelInfo info_;

  std::unique_ptr<PinocchioEndEffectorKinematics> eeKinematics_;
  CentroidalModelPinocchioMapping mapping_;

  vector_t qMeasured_, vMeasured_, inputLast_;
  matrix_t j_, dj_;
  contact_flag_t contactFlag_{};
  size_t numContacts_{};

  // Task Parameters:
  vector_t torqueLimits_;
  scalar_t frictionCoeff_{}, swingKp_{}, swingKd_{};
};

}  // namespace legged
