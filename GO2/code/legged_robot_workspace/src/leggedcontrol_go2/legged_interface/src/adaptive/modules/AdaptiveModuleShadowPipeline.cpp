#include "legged_interface/adaptive/modules/AdaptiveModuleShadowPipeline.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace legged {
namespace adaptive {
namespace modules {
namespace {

bool finiteAxes(const v2::Axes& values) noexcept {
  for (const scalar_t value : values) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  return true;
}

void copyToAxes(const vector6_t& source, v2::Axes& destination) noexcept {
  for (Eigen::Index axis = 0; axis < 6; ++axis) {
    destination[static_cast<std::size_t>(axis)] = source(axis);
  }
}

scalar_t smoothStep(scalar_t value) noexcept {
  const scalar_t bounded =
      std::max<scalar_t>(0.0, std::min<scalar_t>(1.0, value));
  return bounded * bounded * (3.0 - 2.0 * bounded);
}

}  // namespace

AdaptiveModuleShadowPipeline::AdaptiveModuleShadowPipeline(
    AdaptiveModuleShadowSettings settings)
    : settings_(std::move(settings)),
      periodic_(settings_.periodic),
      payload_(settings_.payload),
      residualDynamics_(settings_.residualDynamics),
      featureAdvisor_(settings_.feature),
      confidence_(settings_.confidence) {
  validateSettings();
}

void AdaptiveModuleShadowPipeline::validateSettings() const {
  if (!std::isfinite(settings_.payloadBlendStartConfidence) ||
      !std::isfinite(settings_.payloadBlendFullConfidence) ||
      settings_.payloadBlendStartConfidence < 0.0 ||
      settings_.payloadBlendFullConfidence > 1.0 ||
      settings_.payloadBlendFullConfidence <=
          settings_.payloadBlendStartConfidence) {
    throw std::invalid_argument(
        "[AdaptiveModuleShadowPipeline] invalid payload blend confidence bounds.");
  }
  if (settings_.slowWrenchMode == SlowWrenchMode::PayloadOnly &&
      !settings_.enablePayload) {
    throw std::invalid_argument(
        "[AdaptiveModuleShadowPipeline] PayloadOnly requires the payload module.");
  }
}

bool AdaptiveModuleShadowPipeline::coreInputIsValid(
    const AdaptiveModuleShadowInput& input) const noexcept {
  if (!input.dataValid || !std::isfinite(input.time) ||
      !std::isfinite(input.dt) || input.dt <= 0.0 ||
      (hasStep_ && input.time < lastTime_) ||
      !input.physicalResidual.allFinite() ||
      !input.myRbfRawEstimate.allFinite() ||
      !input.trackingSignal.allFinite() || !input.coverage.allFinite()) {
    return false;
  }
  return finiteAxes(input.featureInput.causalResidual) &&
         finiteAxes(input.featureInput.chi) &&
         finiteAxes(input.featureInput.learningSignal) &&
         finiteAxes(input.featureInput.baseVelocity) &&
         finiteAxes(input.featureInput.referenceVelocity);
}

scalar_t AdaptiveModuleShadowPipeline::payloadBlend(
    const PayloadIdOutput& payload) const noexcept {
  if (!payload.inputValid) {
    return 0.0;
  }
  const scalar_t usefulConfidence =
      payload.parameterConfidence.template head<3>().mean();
  const scalar_t normalized =
      (usefulConfidence - settings_.payloadBlendStartConfidence) /
      (settings_.payloadBlendFullConfidence -
       settings_.payloadBlendStartConfidence);
  return smoothStep(normalized);
}

AdaptiveModuleShadowOutput AdaptiveModuleShadowPipeline::step(
    const AdaptiveModuleShadowInput& input) noexcept {
  AdaptiveModuleShadowOutput output;
  if (!coreInputIsValid(input)) {
    reset(std::isfinite(input.time) ? input.time : 0.0);
    return output;
  }

  try {
    output.myRbfComponent = input.myRbfRawEstimate;

    if (settings_.enablePayload) {
      PayloadIdInput payloadInput = input.payloadInput;
      payloadInput.externalResidual = input.physicalResidual;
      payloadInput.residualValid =
          payloadInput.residualValid && input.dataValid;
      payloadInput.updateAllowed =
          payloadInput.updateAllowed && !input.contactChanged;
      output.payloadResult = payload_.step(payloadInput, input.dt);
      if (output.payloadResult.inputValid) {
        output.payloadComponent = output.payloadResult.equivalentWrench;
      }
    }

    switch (settings_.slowWrenchMode) {
      case SlowWrenchMode::MyRbfOnly:
        output.slowWrench = output.myRbfComponent;
        break;
      case SlowWrenchMode::PayloadOnly:
        output.slowWrench = output.payloadComponent;
        output.payloadBlend = output.payloadResult.inputValid ? 1.0 : 0.0;
        break;
      case SlowWrenchMode::ConfidenceBlend:
        output.payloadBlend = payloadBlend(output.payloadResult);
        output.slowWrench =
            (1.0 - output.payloadBlend) * output.myRbfComponent +
            output.payloadBlend * output.payloadComponent;
        break;
    }

    output.residualAfterSlow = input.physicalResidual - output.slowWrench;

    if (settings_.enablePeriodic && !input.contactChanged) {
      MyRbfV2FeatureInput periodicInput = input.featureInput;
      copyToAxes(output.residualAfterSlow, periodicInput.causalResidual);
      output.periodicResult =
          periodic_.step(input.time, input.dt, periodicInput);
      if (output.periodicResult.inputValid) {
        output.periodicComponent =
            output.periodicResult.periodicWrench.cwiseProduct(
                output.periodicResult.confidence);
      }
    }

    output.residualAfterPeriodic =
        output.residualAfterSlow - output.periodicComponent;

    if (settings_.enableResidualDynamics) {
      output.dynamicsResult = residualDynamics_.step(
          output.residualAfterPeriodic, input.trackingSignal, input.dt,
          !input.contactChanged);
      output.dynamicsComponent =
          output.dynamicsResult.predictedNext.cwiseProduct(
              output.dynamicsResult.confidence);
    }

    if (settings_.enableFeatureAdvisor) {
      output.featureSuggestion =
          featureAdvisor_.observe(input.featureInput, !input.contactChanged);
    }

    output.candidateWrench = output.slowWrench + output.periodicComponent +
                             output.dynamicsComponent;
    if (!output.candidateWrench.allFinite()) {
      reset(input.time);
      return AdaptiveModuleShadowOutput{};
    }

    ResidualConfidenceInput confidenceInput;
    confidenceInput.rawEstimate = output.candidateWrench;
    confidenceInput.residual = input.physicalResidual;
    confidenceInput.coverage = input.coverage;
    confidenceInput.contactChanged = input.contactChanged;
    confidenceInput.dataValid = true;
    confidenceInput.dt = input.dt;
    output.confidenceResult = confidence_.step(confidenceInput);
    output.gatedWrench = output.confidenceResult.gatedWrench;
    output.inputValid = output.confidenceResult.valid;
    lastTime_ = input.time;
    hasStep_ = true;
    return output;
  } catch (...) {
    reset(input.time);
    return AdaptiveModuleShadowOutput{};
  }
}

void AdaptiveModuleShadowPipeline::reset(scalar_t time) {
  periodic_.reset(time);
  payload_.reset();
  residualDynamics_.reset();
  featureAdvisor_.reset();
  confidence_.reset();
  lastTime_ = std::isfinite(time) ? time : 0.0;
  hasStep_ = false;
}

}  // namespace modules
}  // namespace adaptive
}  // namespace legged

