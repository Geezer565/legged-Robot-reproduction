#include "legged_interface/adaptive/modules/PeriodicResidualModule.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace legged {
namespace adaptive {
namespace modules {
namespace {

constexpr double kPi = 3.14159265358979323846;

bool finitePositive(double value) {
  return std::isfinite(value) && value > 0.0;
}

double clampMagnitude(double value, double limit) {
  return std::max(-limit, std::min(limit, value));
}

}  // namespace

PeriodicResidualModule::PeriodicResidualModule()
    : PeriodicResidualModule(Settings{}) {}

PeriodicResidualModule::PeriodicResidualModule(const Settings& settings)
    : settings_(settings) {
  if (settings_.harmonics == 0 || settings_.harmonics > kMaxHarmonics) {
    throw std::invalid_argument("periodic residual harmonics must be 1 or 2");
  }
  if (!std::isfinite(settings_.forgettingFactor) ||
      settings_.forgettingFactor <= 0.0 || settings_.forgettingFactor > 1.0) {
    throw std::invalid_argument("periodic residual forgetting factor must be in (0, 1]");
  }
  if (!finitePositive(settings_.initialCovariance) ||
      !finitePositive(settings_.minimumObservedPeriods)) {
    throw std::invalid_argument("periodic residual covariance and observation period must be positive");
  }
  for (std::size_t axis = 0; axis < kAxes; ++axis) {
    if (!finitePositive(settings_.frequencyHz[axis]) ||
        !finitePositive(settings_.outputLimit[axis])) {
      throw std::invalid_argument("periodic residual frequencies and output limits must be positive");
    }
  }
  reset();
}

void PeriodicResidualModule::reset(scalar_t time) noexcept {
  if (!std::isfinite(time)) {
    time = 0.0;
  }
  coefficients_ = Coefficients{};
  covariance_ = Covariances{};
  for (std::size_t axis = 0; axis < kAxes; ++axis) {
    for (std::size_t index = 0; index < kBasis; ++index) {
      covariance_[axis][index][index] = settings_.initialCovariance;
    }
  }
  startTime_ = time;
  lastTime_ = time;
  hasObservation_ = false;
}

PeriodicResidualModule::Basis PeriodicResidualModule::makeBasis(
    scalar_t time, scalar_t frequencyHz) const noexcept {
  Basis basis{};
  basis[0] = 1.0;
  const double period = 1.0 / frequencyHz;
  const double localTime = std::remainder(time, period);
  const double angle = 2.0 * kPi * frequencyHz * localTime;
  for (std::size_t harmonic = 1; harmonic <= settings_.harmonics; ++harmonic) {
    const std::size_t offset = 1 + 2 * (harmonic - 1);
    basis[offset] = std::sin(static_cast<double>(harmonic) * angle);
    basis[offset + 1] = std::cos(static_cast<double>(harmonic) * angle);
  }
  return basis;
}

bool PeriodicResidualModule::updateCoefficients(
    const v2::Axes& residual, const std::array<Basis, kAxes>& basis) noexcept {
  Coefficients nextCoefficients = coefficients_;
  Covariances nextCovariance = covariance_;
  const std::size_t activeBasis = 1 + 2 * settings_.harmonics;

  for (std::size_t axis = 0; axis < kAxes; ++axis) {
    Basis covarianceTimesBasis{};
    for (std::size_t row = 0; row < activeBasis; ++row) {
      for (std::size_t column = 0; column < activeBasis; ++column) {
        covarianceTimesBasis[row] += covariance_[axis][row][column] * basis[axis][column];
      }
    }

    double denominator = settings_.forgettingFactor;
    double prediction = 0.0;
    for (std::size_t index = 0; index < activeBasis; ++index) {
      denominator += basis[axis][index] * covarianceTimesBasis[index];
      prediction += coefficients_[axis][index] * basis[axis][index];
    }
    if (!finitePositive(denominator) || !std::isfinite(prediction)) {
      return false;
    }

    Basis gain{};
    const double error = residual[axis] - prediction;
    for (std::size_t index = 0; index < activeBasis; ++index) {
      gain[index] = covarianceTimesBasis[index] / denominator;
      nextCoefficients[axis][index] += gain[index] * error;
      if (!std::isfinite(nextCoefficients[axis][index])) {
        return false;
      }
    }

    for (std::size_t row = 0; row < activeBasis; ++row) {
      for (std::size_t column = 0; column < activeBasis; ++column) {
        nextCovariance[axis][row][column] =
            (covariance_[axis][row][column] -
             gain[row] * covarianceTimesBasis[column]) /
            settings_.forgettingFactor;
        if (!std::isfinite(nextCovariance[axis][row][column])) {
          return false;
        }
      }
    }
  }

  coefficients_ = nextCoefficients;
  covariance_ = nextCovariance;
  return true;
}

PeriodicResidualResult PeriodicResidualModule::makeResult(
    scalar_t time, const std::array<Basis, kAxes>& basis) const noexcept {
  PeriodicResidualResult result;
  result.inputValid = true;
  bool allAxesReady = true;
  const double elapsed = std::max(0.0, time - startTime_);
  const std::size_t activeBasis = 1 + 2 * settings_.harmonics;

  for (std::size_t axis = 0; axis < kAxes; ++axis) {
    double estimate = 0.0;
    for (std::size_t index = 0; index < activeBasis; ++index) {
      estimate += coefficients_[axis][index] * basis[axis][index];
    }
    result.periodicWrench[axis] = clampMagnitude(estimate, settings_.outputLimit[axis]);
    result.frequencyHz[axis] = settings_.frequencyHz[axis];

    const double sinOne = coefficients_[axis][1];
    const double cosOne = coefficients_[axis][2];
    result.fundamentalAmplitude[axis] = std::hypot(sinOne, cosOne);
    result.fundamentalPhase[axis] = std::atan2(cosOne, sinOne);
    if (settings_.harmonics == 2) {
      const double sinTwo = coefficients_[axis][3];
      const double cosTwo = coefficients_[axis][4];
      result.secondHarmonicAmplitude[axis] = std::hypot(sinTwo, cosTwo);
      result.secondHarmonicPhase[axis] = std::atan2(cosTwo, sinTwo);
    }

    result.confidence[axis] = std::min(
        1.0, elapsed * settings_.frequencyHz[axis] / settings_.minimumObservedPeriods);
    allAxesReady = allAxesReady && result.confidence[axis] >= 1.0;
  }
  result.valid = allAxesReady && result.periodicWrench.allFinite() &&
                 result.frequencyHz.allFinite() && result.fundamentalAmplitude.allFinite() &&
                 result.fundamentalPhase.allFinite() &&
                 result.secondHarmonicAmplitude.allFinite() &&
                 result.secondHarmonicPhase.allFinite() && result.confidence.allFinite();
  if (!result.valid && !result.periodicWrench.allFinite()) {
    return PeriodicResidualResult{};
  }
  return result;
}

PeriodicResidualResult PeriodicResidualModule::step(
    scalar_t time, scalar_t dt, const MyRbfV2FeatureInput& input) noexcept {
  if (!std::isfinite(time) || !finitePositive(dt) ||
      (hasObservation_ && time < lastTime_)) {
    return PeriodicResidualResult{};
  }

  v2::Axes residual{};
  std::array<Basis, kAxes> basis{};
  for (std::size_t axis = 0; axis < kAxes; ++axis) {
    if (!std::isfinite(input.causalResidual[axis])) {
      return PeriodicResidualResult{};
    }
    residual[axis] = clampMagnitude(input.causalResidual[axis], settings_.outputLimit[axis]);
    basis[axis] = makeBasis(time, settings_.frequencyHz[axis]);
  }

  if (!hasObservation_) {
    startTime_ = time;
    lastTime_ = time;
    hasObservation_ = true;
  }
  if (!updateCoefficients(residual, basis)) {
    reset(time);
    return PeriodicResidualResult{};
  }
  lastTime_ = time;
  return makeResult(time, basis);
}

}  // namespace modules
}  // namespace adaptive
}  // namespace legged

