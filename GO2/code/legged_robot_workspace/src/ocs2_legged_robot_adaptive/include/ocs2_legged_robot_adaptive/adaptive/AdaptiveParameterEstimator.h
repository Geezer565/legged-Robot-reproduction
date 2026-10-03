#pragma once

#include "ocs2_legged_robot_adaptive/adaptive/AdaptiveModelSnapshot.h"

namespace ocs2 {
namespace legged_robot_adaptive {
namespace adaptive {

struct AdaptiveEstimatorUpdate {
  vector6_t sigma = vector6_t::Zero();
  regressor_t regressor = regressor_t::Identity(6, 6);
  /** Optional D5H1 nominal drive subtracted before Gamma. Empty means zero. */
  parameter_vector_t nominalDriveBaseline;
  scalar_t dt = 0.0;
  scalar_t time = 0.0;
};

class AdaptiveParameterEstimator final {
 public:
  explicit AdaptiveParameterEstimator(const AdaptiveClfSettings& settings);
  AdaptiveModelSnapshotPtr snapshot() const { return snapshot_; }
  parameter_vector_t computeParameterDerivative(
      const AdaptiveEstimatorUpdate& update) const;
  AdaptiveModelSnapshotPtr update(const AdaptiveEstimatorUpdate& update);
  AdaptiveModelSnapshotPtr reset(const parameter_vector_t& estimate, scalar_t time);

 private:
  parameter_matrix_t gamma_;
  AdaptiveParameterization parameterization_ =
      AdaptiveParameterization::ConstantWrench6D;
  scalar_t payloadMassAdaptationGain_ = 5.0;
  AdaptiveModelSnapshotPtr snapshot_;
};

}  // namespace adaptive
}  // namespace legged_robot_adaptive
}  // namespace ocs2
