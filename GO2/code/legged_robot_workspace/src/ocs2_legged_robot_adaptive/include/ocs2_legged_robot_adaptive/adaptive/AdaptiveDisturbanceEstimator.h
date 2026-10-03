#pragma once

#include <limits>

#include <ocs2_core/Types.h>
#include <ocs2_core/thread_support/Synchronized.h>
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>

#include <ocs2_legged_robot/common/Types.h>

namespace ocs2 {
namespace legged_robot_adaptive {

struct AdaptiveDisturbanceEstimate {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  legged_robot::vector3_t externalForceInWorld = legged_robot::vector3_t::Zero();
  legged_robot::vector3_t externalTorqueInWorld = legged_robot::vector3_t::Zero();
};

class AdaptiveDisturbanceEstimator final : public SolverSynchronizedModule {
 public:
  struct Config {
    scalar_t lambdaPosition = 2.0;
    scalar_t lambdaOrientation = 1.5;
    scalar_t forceAdaptationGain = 3.0;
    scalar_t torqueAdaptationGain = 1.5;
    scalar_t maxForceEstimate = 150.0;
    scalar_t maxTorqueEstimate = 80.0;
  };

  AdaptiveDisturbanceEstimator();
  explicit AdaptiveDisturbanceEstimator(Config config);

  void preSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t& initState,
                    const ReferenceManagerInterface& referenceManager) override;

  void postSolverRun(const PrimalSolution& primalSolution) override {}

  const AdaptiveDisturbanceEstimate& getActiveEstimate() const { return activeEstimate_; }
  Synchronized<AdaptiveDisturbanceEstimate>& getEstimateBuffer() { return estimateBuffer_; }
  const Synchronized<AdaptiveDisturbanceEstimate>& getEstimateBuffer() const { return estimateBuffer_; }

 private:
  static legged_robot::vector3_t clampNorm(const legged_robot::vector3_t& value, scalar_t maxNorm);

  Config config_;
  scalar_t lastUpdateTime_ = std::numeric_limits<scalar_t>::quiet_NaN();
  AdaptiveDisturbanceEstimate activeEstimate_;
  Synchronized<AdaptiveDisturbanceEstimate> estimateBuffer_;
};

}  // namespace legged_robot_adaptive
}  // namespace ocs2
