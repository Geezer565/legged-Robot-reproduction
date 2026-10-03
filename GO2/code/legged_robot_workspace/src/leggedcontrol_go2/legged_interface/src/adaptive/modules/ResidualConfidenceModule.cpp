#include "legged_interface/adaptive/modules/ResidualConfidenceModule.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace legged {
namespace adaptive {
namespace modules {
namespace {

constexpr scalar_t kSmall = 1.0e-9;

scalar_t clamp01(scalar_t value) {
  return std::max<scalar_t>(0.0, std::min<scalar_t>(1.0, value));
}

scalar_t clampSymmetric(scalar_t value, scalar_t limit) {
  const scalar_t safeLimit = std::max<scalar_t>(0.0, limit);
  return std::max(-safeLimit, std::min(safeLimit, value));
}

scalar_t normalizedScore(scalar_t value, scalar_t lower, scalar_t upper) {
  if (value <= lower) {
    return 0.0;
  }
  if (value >= upper) {
    return 1.0;
  }
  return (value - lower) / std::max(upper - lower, kSmall);
}

scalar_t smoothingGain(scalar_t dt, scalar_t timeConstant) {
  if (timeConstant <= kSmall) {
    return 1.0;
  }
  return 1.0 - std::exp(-dt / timeConstant);
}

bool allFinite(const ResidualConfidenceInput& input) {
  return std::isfinite(input.dt) && input.rawEstimate.allFinite() &&
         input.residual.allFinite() && input.coverage.allFinite();
}

void addReason(std::uint8_t& reasons, ResidualConfidenceRejection reason) {
  reasons = static_cast<std::uint8_t>(
      reasons | static_cast<std::uint8_t>(reason));
}

}  // namespace

ResidualConfidenceModule::ResidualConfidenceModule(
    ResidualConfidenceSettings settings)
    : settings_(std::move(settings)) {
  settings_.riseTime = std::max<scalar_t>(0.0, settings_.riseTime);
  settings_.fallTime = std::max<scalar_t>(0.0, settings_.fallTime);
  settings_.contactTransitionHoldTime =
      std::max<scalar_t>(0.0, settings_.contactTransitionHoldTime);
  settings_.minimumInjectionConfidence =
      clamp01(settings_.minimumInjectionConfidence);
  settings_.fullInjectionConfidence = std::max(
      settings_.minimumInjectionConfidence,
      clamp01(settings_.fullInjectionConfidence));
  settings_.minimumCoverage = clamp01(settings_.minimumCoverage);
  settings_.fullCoverage =
      std::max(settings_.minimumCoverage, clamp01(settings_.fullCoverage));
  settings_.residualDeadband = settings_.residualDeadband.cwiseMax(0.0);
  settings_.maximumWrench = settings_.maximumWrench.cwiseMax(0.0);
  settings_.maximumEstimateRate =
      settings_.maximumEstimateRate.cwiseMax(0.0);
  settings_.maximumGatedWrenchRate =
      settings_.maximumGatedWrenchRate.cwiseMax(0.0);
}

ResidualConfidenceOutput ResidualConfidenceModule::step(
    const ResidualConfidenceInput& input) {
  if (!input.dataValid || input.dt <= 0.0 || !allFinite(input)) {
    return rejectInvalidInput();
  }

  if (input.contactChanged) {
    contactHoldRemaining_ = settings_.contactTransitionHoldTime;
  }
  const bool contactUnreliable =
      input.contactChanged || contactHoldRemaining_ > 0.0;

  ResidualConfidenceOutput output;
  output.valid = true;

  for (Eigen::Index axis = 0; axis < 6; ++axis) {
    scalar_t targetConfidence = normalizedScore(
        clamp01(input.coverage(axis)), settings_.minimumCoverage,
        settings_.fullCoverage);
    if (targetConfidence <= 0.0) {
      addReason(output.rejectionReasons[axis], kLowCoverage);
    }

    if (contactUnreliable) {
      targetConfidence = 0.0;
      addReason(output.rejectionReasons[axis], kContactTransition);
    }

    const scalar_t deadband = settings_.residualDeadband(axis);
    if (std::abs(input.rawEstimate(axis)) > deadband &&
        std::abs(input.residual(axis)) > deadband &&
        input.rawEstimate(axis) * input.residual(axis) < 0.0) {
      targetConfidence = 0.0;
      addReason(output.rejectionReasons[axis], kDirectionMismatch);
    }

    const scalar_t magnitude = std::abs(input.rawEstimate(axis));
    const scalar_t magnitudeLimit = settings_.maximumWrench(axis);
    if (magnitude > magnitudeLimit) {
      output.magnitudeLimited[axis] = true;
      addReason(output.rejectionReasons[axis], kMagnitudeExceeded);
      targetConfidence *= magnitudeLimit / std::max(magnitude, kSmall);
    }

    if (hasPreviousEstimate_) {
      const scalar_t estimateRate =
          std::abs(input.rawEstimate(axis) - previousEstimate_(axis)) /
          input.dt;
      const scalar_t rateLimit = settings_.maximumEstimateRate(axis);
      if (estimateRate > rateLimit) {
        output.rateLimited[axis] = true;
        addReason(output.rejectionReasons[axis], kEstimateRateExceeded);
        targetConfidence *= rateLimit / std::max(estimateRate, kSmall);
      }
    }

    targetConfidence = clamp01(targetConfidence);
    const scalar_t timeConstant =
        targetConfidence < confidence_(axis) ? settings_.fallTime
                                             : settings_.riseTime;
    const scalar_t gain = smoothingGain(input.dt, timeConstant);
    confidence_(axis) = clamp01(
        confidence_(axis) + gain * (targetConfidence - confidence_(axis)));

    scalar_t injectionScale = 0.0;
    if (confidence_(axis) >= settings_.fullInjectionConfidence) {
      injectionScale = 1.0;
    } else if (confidence_(axis) > settings_.minimumInjectionConfidence) {
      const scalar_t normalized =
          (confidence_(axis) - settings_.minimumInjectionConfidence) /
          std::max(settings_.fullInjectionConfidence -
                       settings_.minimumInjectionConfidence,
                   kSmall);
      injectionScale = normalized * normalized * (3.0 - 2.0 * normalized);
    }

    const scalar_t limitedEstimate =
        clampSymmetric(input.rawEstimate(axis), magnitudeLimit);
    const scalar_t desiredWrench = injectionScale * limitedEstimate;
    const scalar_t maximumStep =
        settings_.maximumGatedWrenchRate(axis) * input.dt;
    gatedWrench_(axis) = clampSymmetric(
        gatedWrench_(axis) + clampSymmetric(
                                  desiredWrench - gatedWrench_(axis),
                                  maximumStep),
        magnitudeLimit);
  }

  previousEstimate_ = input.rawEstimate;
  hasPreviousEstimate_ = true;
  contactHoldRemaining_ =
      std::max<scalar_t>(0.0, contactHoldRemaining_ - input.dt);

  output.confidence = confidence_;
  output.gatedWrench = gatedWrench_;
  return output;
}

void ResidualConfidenceModule::reset() {
  confidence_.setZero();
  gatedWrench_.setZero();
  previousEstimate_.setZero();
  hasPreviousEstimate_ = false;
  contactHoldRemaining_ = 0.0;
}

ResidualConfidenceOutput ResidualConfidenceModule::rejectInvalidInput() {
  reset();
  ResidualConfidenceOutput output;
  output.rejectionReasons.fill(static_cast<std::uint8_t>(kInvalidData));
  return output;
}

}  // namespace modules
}  // namespace adaptive
}  // namespace legged
