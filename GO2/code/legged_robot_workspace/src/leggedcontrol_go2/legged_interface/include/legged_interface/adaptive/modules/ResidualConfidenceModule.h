#pragma once

#include "legged_interface/adaptive/AdaptiveTypes.h"

#include <array>
#include <cstdint>

namespace legged {
namespace adaptive {
namespace modules {

enum ResidualConfidenceRejection : std::uint8_t {
  kNoRejection = 0,
  kInvalidData = 1u << 0,
  kContactTransition = 1u << 1,
  kLowCoverage = 1u << 2,
  kDirectionMismatch = 1u << 3,
  kEstimateRateExceeded = 1u << 4,
  kMagnitudeExceeded = 1u << 5,
};

struct ResidualConfidenceSettings {
  scalar_t riseTime = 0.4;
  scalar_t fallTime = 0.03;
  scalar_t contactTransitionHoldTime = 0.05;
  scalar_t minimumInjectionConfidence = 0.3;
  scalar_t fullInjectionConfidence = 0.8;
  scalar_t minimumCoverage = 0.15;
  scalar_t fullCoverage = 0.55;
  vector6_t residualDeadband =
      (vector6_t() << 2.0, 2.0, 3.0, 0.4, 0.4, 0.4).finished();
  vector6_t maximumWrench =
      (vector6_t() << 80.0, 80.0, 120.0, 20.0, 20.0, 20.0).finished();
  vector6_t maximumEstimateRate =
      (vector6_t() << 500.0, 500.0, 700.0, 100.0, 100.0, 100.0).finished();
  vector6_t maximumGatedWrenchRate =
      (vector6_t() << 300.0, 300.0, 400.0, 60.0, 60.0, 60.0).finished();
};

struct ResidualConfidenceInput {
  vector6_t rawEstimate = vector6_t::Zero();
  vector6_t residual = vector6_t::Zero();
  vector6_t coverage = vector6_t::Zero();
  bool contactChanged = false;
  bool dataValid = false;
  scalar_t dt = 0.0;
};

struct ResidualConfidenceOutput {
  bool valid = false;
  vector6_t confidence = vector6_t::Zero();
  vector6_t gatedWrench = vector6_t::Zero();
  std::array<std::uint8_t, 6> rejectionReasons{};
  std::array<bool, 6> magnitudeLimited{};
  std::array<bool, 6> rateLimited{};
};

/**
 * Per-axis trust filter for an estimated external wrench.
 *
 * Inputs and outputs use [Fx, Fy, Fz, Tx, Ty, Tz] in one common frame.
 * step() performs no dynamic allocation.
 */
class ResidualConfidenceModule {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit ResidualConfidenceModule(
      ResidualConfidenceSettings settings = ResidualConfidenceSettings{});

  ResidualConfidenceOutput step(const ResidualConfidenceInput& input);
  void reset();

  const ResidualConfidenceSettings& settings() const { return settings_; }
  const vector6_t& confidence() const { return confidence_; }
  const vector6_t& gatedWrench() const { return gatedWrench_; }

 private:
  ResidualConfidenceOutput rejectInvalidInput();

  ResidualConfidenceSettings settings_;
  vector6_t confidence_ = vector6_t::Zero();
  vector6_t gatedWrench_ = vector6_t::Zero();
  vector6_t previousEstimate_ = vector6_t::Zero();
  bool hasPreviousEstimate_ = false;
  scalar_t contactHoldRemaining_ = 0.0;
};

}  // namespace modules
}  // namespace adaptive
}  // namespace legged
