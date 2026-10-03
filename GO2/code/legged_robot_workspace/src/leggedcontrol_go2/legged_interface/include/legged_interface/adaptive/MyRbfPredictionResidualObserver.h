#pragma once

#include "legged_interface/adaptive/AdaptiveTypes.h"

namespace legged {
namespace adaptive {

struct MyRbfPredictionResidualResult {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  bool valid = false;
  scalar_t dt = 0.0;
  vector6_t netResidual = vector6_t::Zero();
  vector6_t rawResidual = vector6_t::Zero();
  vector6_t filteredResidual = vector6_t::Zero();
};

/**
 * Reconstructs a six-dimensional generalized wrench from the innovation
 * between a previous one-step nominal prediction and the aligned measured
 * centroidal state. The first six state entries are normalized momentum.
 */
class MyRbfPredictionResidualObserver final {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  MyRbfPredictionResidualObserver(scalar_t robotMass,
                                  Eigen::Index stateDimension,
                                  bool enableFilter,
                                  scalar_t filterAlpha,
                                  scalar_t timeTolerance);

  void setPrediction(scalar_t predictedTime, scalar_t intervalDt,
                     const vector_t& predictedState,
                     const vector6_t& knownAppliedCompensation =
                         vector6_t::Zero());
  MyRbfPredictionResidualResult observe(scalar_t actualTime,
                                        const vector_t& actualState);
  void reset();

 private:
  bool validState(const vector_t& state) const;

  scalar_t robotMass_;
  Eigen::Index stateDimension_;
  bool enableFilter_;
  scalar_t filterAlpha_;
  scalar_t timeTolerance_;
  bool hasPrediction_ = false;
  scalar_t predictedTime_ = 0.0;
  scalar_t intervalDt_ = 0.0;
  vector_t predictedState_;
  vector6_t knownAppliedCompensation_ = vector6_t::Zero();
  bool filterInitialized_ = false;
  vector6_t filteredResidual_ = vector6_t::Zero();
};

}  // namespace adaptive
}  // namespace legged
