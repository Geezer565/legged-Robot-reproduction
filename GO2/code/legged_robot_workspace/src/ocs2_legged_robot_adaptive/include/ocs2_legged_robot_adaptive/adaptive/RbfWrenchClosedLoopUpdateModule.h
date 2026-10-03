#pragma once

#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <string>

#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>

#include "ocs2_legged_robot_adaptive/adaptive/AdaptiveModelData.h"
#include "ocs2_legged_robot_adaptive/adaptive/AdaptiveReferenceKinematics.h"
#include "ocs2_legged_robot_adaptive/adaptive/CompositeRbfWrenchEstimator.h"
#include "ocs2_legged_robot_adaptive/adaptive/RandomFourierWrenchEstimator.h"
#include "ocs2_legged_robot_adaptive/adaptive/SlidingVariable.h"
#include "ocs2_legged_robot_adaptive/adaptive/TwoFeatureMinnitiRbfWrenchEstimator.h"
#include "ocs2_legged_robot_adaptive/adaptive/WrenchResidualEstimator.h"

namespace ocs2 { namespace legged_robot_adaptive { namespace adaptive {

enum class WrenchEstimatorMethod {
  MinnitiRbf,
  CompositeRbf,
  RandomFourierOgd,
};

struct RbfWrenchClosedLoopSettings { EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  bool enabled = false;
  WrenchEstimatorMethod method = WrenchEstimatorMethod::MinnitiRbf;
  bool enableResidualFilter = true;
  scalar_t residualFilterAlpha = 0.9;
  scalar_t observationTimeTolerance = 1.0e-6;
  TwoFeatureMinnitiRbfWrenchEstimatorSettings estimator;
  CompositeRbfWrenchEstimatorSettings compositeEstimator;
  RandomFourierWrenchEstimatorSettings randomFourierEstimator;
};

RbfWrenchClosedLoopSettings loadRbfWrenchClosedLoopSettings(
    const std::string& filename,
    const std::string& prefix = "rbf_wrench_closed_loop");

struct RbfWrenchClosedLoopDiagnostics { EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  WrenchEstimatorMethod method = WrenchEstimatorMethod::MinnitiRbf;
  bool valid = false;
  bool observationMatched = false;
  bool residualValid = false;
  bool parameterUpdateEnabled = false;
  bool weightsUpdated = false;
  scalar_t observationTime = 0.0;
  scalar_t snapshotTime = 0.0;
  scalar_t residualDt = 0.0;
  std::uint64_t snapshotVersion = 0;
  vector6_t rawResidual = vector6_t::Zero();
  vector6_t filteredResidual = vector6_t::Zero();
  vector6_t sigma = vector6_t::Zero();
  vector6_t estimateBeforeUpdate = vector6_t::Zero();
  vector6_t estimateAfterUpdate = vector6_t::Zero();
  scalar_t weightNorm = 0.0;
  scalar_t weightDerivativeNorm = 0.0;
};

/**
 * Solver-synchronized closed-loop RBF update. ROS only submits executed
 * observations; this module remains the sole snapshot writer and updates once
 * before a solver run, so rollout and linearization see one immutable estimate.
 */
class RbfWrenchClosedLoopUpdateModule final : public SolverSynchronizedModule {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  RbfWrenchClosedLoopUpdateModule(
      const PinocchioInterface& pinocchioInterface,
      CentroidalModelInfo centroidalModelInfo,
      AdaptiveClfSettings adaptiveSettings,
      RbfWrenchClosedLoopSettings rbfSettings,
      std::shared_ptr<AdaptiveModelData> modelData);

  void submitObservation(scalar_t time, const vector_t& state,
                         const vector_t& input);
  void preSolverRun(scalar_t initTime, scalar_t finalTime,
                    const vector_t& initState,
                    const ReferenceManagerInterface& referenceManager) override;
  void postSolverRun(const PrimalSolution&) override {}

  AdaptiveModelSnapshotPtr snapshot() const { return modelData_->snapshot(); }
  RbfWrenchClosedLoopDiagnostics diagnostics() const;

 private:
  struct SubmittedObservation {
    scalar_t time = 0.0;
    vector_t state;
    vector_t input;
    bool valid = false;
  };

  AdaptiveClfSettings adaptiveSettings_;
  RbfWrenchClosedLoopSettings rbfSettings_;
  AdaptiveReferenceKinematicsAdapter referenceKinematics_;
  SlidingVariable slidingVariable_;
  WrenchResidualEstimator residualEstimator_;
  std::shared_ptr<AdaptiveModelData> modelData_;
  scalar_t lastProcessedTime_ = std::numeric_limits<scalar_t>::quiet_NaN();
  std::uint64_t snapshotVersion_ = 0;
  mutable std::mutex observationMutex_;
  SubmittedObservation submittedObservation_;
  mutable std::mutex diagnosticsMutex_;
  RbfWrenchClosedLoopDiagnostics diagnostics_;
  std::unique_ptr<TwoFeatureMinnitiRbfWrenchEstimator> rbfEstimator_;
  std::unique_ptr<CompositeRbfWrenchEstimator> compositeRbfEstimator_;
  std::unique_ptr<RandomFourierWrenchEstimator> randomFourierEstimator_;
};

}}}  // namespace ocs2::legged_robot_adaptive::adaptive
