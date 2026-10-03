#pragma once

#include <ocs2_core/PreComputation.h>

#include "ocs2_legged_robot_adaptive/adaptive/AdaptiveTypes.h"
#include "ocs2_legged_robot_adaptive/dynamics/NominalCoordinatePlantDynamics.h"

namespace ocs2 {
namespace legged_robot_adaptive {
namespace adaptive {

/** Result of one interval-based generalized-wrench residual observation. */
struct WrenchResidualResult {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  bool valid = false;
  scalar_t dt = 0.0;
  vector6_t observedRate = vector6_t::Zero();
  vector6_t nominalRate = vector6_t::Zero();
  vector6_t rawResidual = vector6_t::Zero();
  vector6_t filteredResidual = vector6_t::Zero();
};

/**
 * Side-channel 6D generalized-wrench residual observer.
 *
 * This object deliberately owns a fully nominal dynamics copy and has no
 * connection to adaptive estimates, MPC dynamics, or the controller.
 */
class WrenchResidualEstimator final {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  WrenchResidualEstimator(const PinocchioInterface& pinocchioInterface,
                          CentroidalModelInfo nominalInfo,
                          bool enableFilter = true,
                          scalar_t filterAlpha = 0.9);

  WrenchResidualResult update(scalar_t time, const vector_t& state,
                              const vector_t& input);
  void reset();

  static vector6_t reconstructWrenchResidual(
      scalar_t robotMass, const vector6_t& observedRate,
      const vector6_t& nominalRate);

 private:
  static void validateFilterSettings(bool enableFilter, scalar_t filterAlpha);
  bool validSample(scalar_t time, const vector_t& state,
                   const vector_t& input) const;
  void storePreviousSample(scalar_t time, const vector_t& state,
                           const vector_t& input);

  NominalCoordinatePlantDynamics nominalDynamics_;
  PreComputation preComputation_;
  scalar_t robotMass_;
  scalar_t previousTime_ = 0.0;
  vector_t previousState_;
  vector_t previousInput_;
  bool hasPreviousSample_ = false;
  vector6_t rawResidual_ = vector6_t::Zero();
  vector6_t filteredResidual_ = vector6_t::Zero();
  bool filterInitialized_ = false;
  bool enableFilter_ = true;
  scalar_t filterAlpha_ = 0.9;
};

}  // namespace adaptive
}  // namespace legged_robot_adaptive
}  // namespace ocs2
