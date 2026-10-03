#pragma once

#include <array>

#include "legged_interface/adaptive/AdaptiveTypes.h"

namespace legged {
namespace adaptive {
namespace modules {

struct ResidualDynamicsSettings {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  vector6_t learningRate = vector6_t::Constant(0.1);
  vector6_t parameterLimit = vector6_t::Constant(5.0);
  vector6_t residualScale = vector6_t::Ones();
  vector6_t trackingScale = vector6_t::Ones();
  scalar_t normalizationEpsilon = 1e-9;
  unsigned minimumSamplesForConfidence = 20;
  unsigned confidenceRiseSamples = 80;
};

struct ResidualDynamicsResult {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  vector6_t predictedCurrent = vector6_t::Zero();
  vector6_t predictedAfterUpdate = vector6_t::Zero();
  vector6_t predictedNext = vector6_t::Zero();
  vector6_t innovation = vector6_t::Zero();
  vector6_t confidence = vector6_t::Zero();
  bool learningEnabled = false;
};

// A compact, allocation-free ARMAV-style residual model. Each wrench axis
// learns its own affine one-step model from [1, previous residual, chi].
// Wrench order is [Fx,Fy,Fz,Tx,Ty,Tz].
class ResidualDynamicsModule final {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit ResidualDynamicsModule(ResidualDynamicsSettings settings = ResidualDynamicsSettings{});

  ResidualDynamicsResult step(const vector6_t& residual, const vector6_t& trackingSignal,
                              scalar_t dt, bool enableLearning = true);
  void reset();

  const matrix_t& weights() const noexcept { return weights_; }
  const vector6_t& previousResidual() const noexcept { return previousResidual_; }
  const std::array<unsigned, 6>& sampleCount() const noexcept { return sampleCount_; }

 private:
  Eigen::Matrix<scalar_t, 3, 1> feature(Eigen::Index axis, const vector6_t& residualForLag,
                                        const vector6_t& trackingSignal) const;
  vector6_t evaluate(const vector6_t& residualForLag, const vector6_t& trackingSignal) const;
  void projectAxis(Eigen::Index axis);
  void validateSettings() const;

  ResidualDynamicsSettings settings_;
  matrix_t weights_ = matrix_t::Zero(3, 6);
  vector6_t previousResidual_ = vector6_t::Zero();
  std::array<unsigned, 6> sampleCount_{{0, 0, 0, 0, 0, 0}};
};

}  // namespace modules
}  // namespace adaptive
}  // namespace legged

