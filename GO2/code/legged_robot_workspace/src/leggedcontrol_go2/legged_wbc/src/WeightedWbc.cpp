//
// Created by qiayuan on 22-12-23.
//

#include "legged_wbc/WeightedWbc.h"

#include <qpOASES.hpp>
#include <cmath>
#include <ros/console.h>

namespace legged {

vector_t WeightedWbc::update(const vector_t& stateDesired, const vector_t& inputDesired, const vector_t& rbdStateMeasured, size_t mode,
                             scalar_t period) {
  if (myRbfV2Input_.enabled) {
    if (!prepareMyRbfV2Tracking(stateDesired, inputDesired, rbdStateMeasured, mode, period)) return {};
    myRbfV2PreviousDesiredVelocity_ = myRbfV2CurrentDesiredVelocity_;
    myRbfV2PreviousDesiredSnapshotSequenceId_ = myRbfV2Input_.snapshotSequenceId;
  } else {
    WbcBase::update(stateDesired, inputDesired, rbdStateMeasured, mode, period);
  }

  // Constraints
  Task constraints = formulateConstraints();
  size_t numConstraints = constraints.b_.size() + constraints.f_.size();

  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> A(numConstraints, getNumDecisionVars());
  vector_t lbA(numConstraints), ubA(numConstraints);  // clang-format off
  A << constraints.a_,
       constraints.d_;

  lbA << constraints.b_,
         -qpOASES::INFTY * vector_t::Ones(constraints.f_.size());
  ubA << constraints.b_,
         constraints.f_;  // clang-format on

  // Cost
  Task weighedTask = formulateWeightedTasks(stateDesired, inputDesired, period);
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> H = weighedTask.a_.transpose() * weighedTask.a_;
  vector_t g = -weighedTask.a_.transpose() * weighedTask.b_;
  if (myRbfV2Input_.enabled && (!H.allFinite() || !g.allFinite() || !A.allFinite() || !lbA.allFinite() || !ubA.allFinite())) {
    myRbfV2Diagnostics_.status = MyRbfV2WbcStatus::InvalidInput;
    return {};
  }

  // Solve
  auto qpProblem = qpOASES::QProblem(getNumDecisionVars(), numConstraints);
  qpOASES::Options options;
  if (myRbfV2Input_.enabled) {
    // The hard CLF row is dense and can be poorly scaled relative to contact
    // equalities. Prefer qpOASES' robust preset in this independent mode;
    // the legacy controller retains its original MPC preset below.
    options.setToReliable();
  } else {
    options.setToMPC();
  }
  options.printLevel = qpOASES::PL_LOW;
  options.enableEqualities = qpOASES::BT_TRUE;
  qpProblem.setOptions(options);
  // The hard CLF plus force/rate limits can require substantially more active-
  // set changes during contact transients. This changes only solve effort: it
  // does not add slack, scale a request, or accept an unchecked primal result.
  int nWsr = myRbfV2Input_.enabled ? 1000 : 200;

  const auto result = qpProblem.init(H.data(), g.data(), A.data(), nullptr, nullptr, lbA.data(), ubA.data(), nWsr);
  if (!myRbfV2Input_.enabled && result != qpOASES::SUCCESSFUL_RETURN) {
    ROS_ERROR_STREAM("[WBC] Solve failed; no actuator command will be issued, code=" << static_cast<int>(result));
    return {};
  }
  if (myRbfV2Input_.enabled) {
    myRbfV2Diagnostics_.solverReturnCode = static_cast<int>(result);
    if (result != qpOASES::SUCCESSFUL_RETURN) {
      // Failure-only diagnosis: distinguish a conflict in the legacy physical
      // constraints from an infeasible hard CLF row. This never supplies a
      // command and therefore cannot weaken the fail-stop behavior below.
      const Task physical = formulateFloatingBaseEomTask() + formulateTorqueLimitsTask() + formulateFrictionConeTask() +
                            formulateNoContactMotionTask() + formulateMyRbfV2ForceLimitsTask();
      const int physicalRows = physical.b_.size() + physical.f_.size();
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> physicalA(physicalRows, getNumDecisionVars());
      vector_t physicalLb(physicalRows), physicalUb(physicalRows);
      physicalA << physical.a_, physical.d_;
      physicalLb << physical.b_, -qpOASES::INFTY * vector_t::Ones(physical.f_.size());
      physicalUb << physical.b_, physical.f_;

      auto diagnosticOptions = options;
      diagnosticOptions.printLevel = qpOASES::PL_NONE;
      qpOASES::QProblem physicalProblem(getNumDecisionVars(), physicalRows);
      physicalProblem.setOptions(diagnosticOptions);
      int physicalNwsr = 2000;
      const auto physicalResult = physicalProblem.init(H.data(), g.data(), physicalA.data(), nullptr, nullptr,
                                                       physicalLb.data(), physicalUb.data(), physicalNwsr);

      int supportCode = -1;
      double minimumClfGap = std::numeric_limits<double>::quiet_NaN();
      if (physicalResult == qpOASES::SUCCESSFUL_RETURN && myRbfV2Input_.enforceClfConstraint) {
        const Task clf = formulateMyRbfV2ClfTask();
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> supportH =
            1e-9 * Eigen::MatrixXd::Identity(getNumDecisionVars(), getNumDecisionVars());
        vector_t supportG = clf.d_.row(0).transpose();
        qpOASES::QProblem supportProblem(getNumDecisionVars(), physicalRows);
        supportProblem.setOptions(diagnosticOptions);
        int supportNwsr = 4000;
        const auto supportResult = supportProblem.init(supportH.data(), supportG.data(), physicalA.data(), nullptr, nullptr,
                                                       physicalLb.data(), physicalUb.data(), supportNwsr);
        supportCode = static_cast<int>(supportResult);
        if (supportResult == qpOASES::SUCCESSFUL_RETURN) {
          vector_t supportSolution(getNumDecisionVars());
          if (supportProblem.getPrimalSolution(supportSolution.data()) == qpOASES::SUCCESSFUL_RETURN)
            minimumClfGap = (clf.d_ * supportSolution - clf.f_)[0];
        }
      }
      ROS_ERROR_STREAM("[V2] infeasibility diagnosis: full_code=" << static_cast<int>(result)
                       << " physical_code=" << static_cast<int>(physicalResult)
                       << " clf_support_code=" << supportCode
                       << " minimum_clf_gap=" << minimumClfGap
                       << " contact_mode=" << myRbfV2Diagnostics_.contactMode
                       << " contact_rank=" << myRbfV2Diagnostics_.contactWrenchRank
                       << " chi=" << myRbfV2Diagnostics_.chi.transpose());
      myRbfV2Diagnostics_.status = MyRbfV2WbcStatus::SolverFailure;
      return {};  // No slack, scaling, stale solution, or unchecked primal output.
    }
  }
  vector_t qpSol(getNumDecisionVars());

  const auto primalResult = qpProblem.getPrimalSolution(qpSol.data());
  if (!myRbfV2Input_.enabled && (primalResult != qpOASES::SUCCESSFUL_RETURN || !qpSol.allFinite())) {
    ROS_ERROR("[WBC] Invalid primal solution; no actuator command will be issued");
    return {};
  }
  if (myRbfV2Input_.enabled) {
    if (primalResult != qpOASES::SUCCESSFUL_RETURN) {
      myRbfV2Diagnostics_.solverReturnCode = static_cast<int>(primalResult);
      myRbfV2Diagnostics_.status = MyRbfV2WbcStatus::SolverFailure;
      return {};
    }
    if (!checkMyRbfV2Solution(qpSol, constraints)) return {};
  }
  return qpSol;
}

Task WeightedWbc::formulateConstraints() {
  if (myRbfV2Input_.enabled) {
    const Task physicalConstraints = formulateFloatingBaseEomTask() + formulateTorqueLimitsTask() + formulateFrictionConeTask() +
                                     formulateNoContactMotionTask() + formulateMyRbfV2ForceLimitsTask();
    return myRbfV2Input_.enforceClfConstraint ? physicalConstraints + formulateMyRbfV2ClfTask() : physicalConstraints;
  }
  return formulateFloatingBaseEomTask() + formulateTorqueLimitsTask() + formulateFrictionConeTask() + formulateNoContactMotionTask();
}

Task WeightedWbc::formulateWeightedTasks(const vector_t& stateDesired, const vector_t& inputDesired, scalar_t period) {
  return formulateSwingLegTask() * weightSwingLeg_ + formulateBaseAccelTask(stateDesired, inputDesired, period) * weightBaseAccel_ +
         formulateContactForceTask(inputDesired) * weightContactForce_;
}

void WeightedWbc::loadTasksSetting(const std::string& taskFile, bool verbose) {
  WbcBase::loadTasksSetting(taskFile, verbose);

  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  std::string prefix = "weight.";
  if (verbose) {
    std::cerr << "\n #### WBC weight:";
    std::cerr << "\n #### =============================================================================\n";
  }
  loadData::loadPtreeValue(pt, weightSwingLeg_, prefix + "swingLeg", verbose);
  loadData::loadPtreeValue(pt, weightBaseAccel_, prefix + "baseAccel", verbose);
  loadData::loadPtreeValue(pt, weightContactForce_, prefix + "contactForce", verbose);
}

}  // namespace legged
