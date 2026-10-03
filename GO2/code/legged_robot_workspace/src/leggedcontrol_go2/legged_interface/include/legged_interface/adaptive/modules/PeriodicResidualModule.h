#pragma once

#include <array>
#include <cstddef>

#include "legged_interface/adaptive/AdaptiveTypes.h"
#include "legged_interface/adaptive/MyRbfV2FeatureInput.h"

namespace legged {
namespace adaptive {
namespace modules {

struct PeriodicResidualResult final {
  bool inputValid = false;
  bool valid = false;
  vector6_t periodicWrench = vector6_t::Zero();
  vector6_t frequencyHz = vector6_t::Zero();
  vector6_t fundamentalAmplitude = vector6_t::Zero();
  vector6_t fundamentalPhase = vector6_t::Zero();
  vector6_t secondHarmonicAmplitude = vector6_t::Zero();
  vector6_t secondHarmonicPhase = vector6_t::Zero();
  vector6_t confidence = vector6_t::Zero();
};

/**
 * Fixed-frequency, allocation-free periodic residual estimator.
 *
 * MyRbfV2FeatureInput::causalResidual must contain the residual left after
 * subtracting the slow/MyRBF estimate. Axes follow [Fx,Fy,Fz,Tx,Ty,Tz].
 */
class PeriodicResidualModule final {
 public:
  static constexpr std::size_t kAxes = 6;
  static constexpr std::size_t kMaxHarmonics = 2;
  static constexpr std::size_t kBasis = 1 + 2 * kMaxHarmonics;

  struct Settings final {
    std::size_t harmonics = 2;
    v2::Axes frequencyHz{{0.5, 0.5, 0.5, 0.5, 0.5, 0.5}};
    scalar_t forgettingFactor = 0.995;
    scalar_t initialCovariance = 100.0;
    scalar_t minimumObservedPeriods = 1.5;
    v2::Axes outputLimit{{60.0, 60.0, 80.0, 15.0, 15.0, 15.0}};
  };

  PeriodicResidualModule();
  explicit PeriodicResidualModule(const Settings& settings);

  void reset(scalar_t time = 0.0) noexcept;

  PeriodicResidualResult step(scalar_t time, scalar_t dt,
                              const MyRbfV2FeatureInput& input) noexcept;

 private:
  using Basis = std::array<scalar_t, kBasis>;
  using Covariance = std::array<Basis, kBasis>;
  using Coefficients = std::array<Basis, kAxes>;
  using Covariances = std::array<Covariance, kAxes>;

  Basis makeBasis(scalar_t time, scalar_t frequencyHz) const noexcept;
  bool updateCoefficients(const v2::Axes& residual,
                          const std::array<Basis, kAxes>& basis) noexcept;
  PeriodicResidualResult makeResult(
      scalar_t time, const std::array<Basis, kAxes>& basis) const noexcept;

  Settings settings_;
  Coefficients coefficients_{};
  Covariances covariance_{};
  scalar_t startTime_ = 0.0;
  scalar_t lastTime_ = 0.0;
  bool hasObservation_ = false;
};

}  // namespace modules
}  // namespace adaptive
}  // namespace legged

