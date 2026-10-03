#include "legged_interface/adaptive/modules/PayloadIdModule.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace legged {
namespace adaptive {
namespace modules {
namespace {

Eigen::Matrix<scalar_t, 3, 3> skew(const vector3_t& value) {
  Eigen::Matrix<scalar_t, 3, 3> result;
  result << 0.0, -value.z(), value.y(), value.z(), 0.0, -value.x(),
      -value.y(), value.x(), 0.0;
  return result;
}

scalar_t bounded(scalar_t value, scalar_t lower, scalar_t upper) {
  return std::max(lower, std::min(upper, value));
}

}  // namespace

PayloadIdModule::PayloadIdModule(PayloadIdSettings settings)
    : settings_(std::move(settings)) {
  validateSettings();
}

void PayloadIdModule::validateSettings() const {
  if (!settings_.adaptationGain.allFinite() ||
      (settings_.adaptationGain.array() <= 0.0).any() ||
      !std::isfinite(settings_.forgettingRate) ||
      settings_.forgettingRate < 0.0 ||
      !std::isfinite(settings_.informationForgettingRate) ||
      settings_.informationForgettingRate < 0.0 ||
      !std::isfinite(settings_.gravityMagnitude) ||
      settings_.gravityMagnitude <= 0.0 ||
      !std::isfinite(settings_.minDeltaMass) ||
      !std::isfinite(settings_.maxDeltaMass) ||
      settings_.minDeltaMass < 0.0 ||
      settings_.maxDeltaMass <= settings_.minDeltaMass ||
      !settings_.maxAbsDeltaCom.allFinite() ||
      (settings_.maxAbsDeltaCom.array() <= 0.0).any() ||
      !std::isfinite(settings_.normalizationEpsilon) ||
      settings_.normalizationEpsilon <= 0.0 ||
      !settings_.confidenceInformationScale.allFinite() ||
      (settings_.confidenceInformationScale.array() <= 0.0).any() ||
      !std::isfinite(settings_.maximumAngularSpeedForAutomaticRegressor) ||
      settings_.maximumAngularSpeedForAutomaticRegressor <= 0.0 ||
      !std::isfinite(settings_.massEpsilon) ||
      settings_.massEpsilon <= 0.0) {
    throw std::invalid_argument("[PayloadIdModule] invalid settings.");
  }
}

bool PayloadIdModule::inputIsValid(const PayloadIdInput& input,
                                   scalar_t dt) const {
  if (!std::isfinite(dt) || dt < 0.0 || !input.residualValid ||
      !input.externalResidual.allFinite()) {
    return false;
  }
  if (input.useCustomRegressor) {
    return input.customRegressor.allFinite();
  }
  if (!input.gravityDirection.allFinite() ||
      input.gravityDirection.norm() <= settings_.massEpsilon) {
    return false;
  }
  if (input.hasLinearAcceleration &&
      !input.linearAcceleration.allFinite()) {
    return false;
  }
  if (input.hasAngularVelocity &&
      (!input.angularVelocity.allFinite() ||
       input.angularVelocity.norm() >
           settings_.maximumAngularSpeedForAutomaticRegressor)) {
    return false;
  }
  return true;
}

payload_matrix6x4_t PayloadIdModule::makeAutomaticRegressor(
    const PayloadIdInput& input) const {
  const vector3_t gravity = settings_.gravityMagnitude *
                            input.gravityDirection.normalized();
  const vector3_t effectiveAcceleration =
      gravity - (input.hasLinearAcceleration ? input.linearAcceleration
                                             : vector3_t::Zero());
  payload_matrix6x4_t regressor = payload_matrix6x4_t::Zero();
  regressor.template block<3, 1>(0, 0) = effectiveAcceleration;
  regressor.template block<3, 3>(3, 1) = -skew(effectiveAcceleration);
  return regressor;
}

void PayloadIdModule::projectParameters() {
  parameters_(0) = bounded(parameters_(0), settings_.minDeltaMass,
                           settings_.maxDeltaMass);
  const scalar_t projectedMass = std::max(parameters_(0), 0.0);
  for (Eigen::Index axis = 0; axis < 3; ++axis) {
    const scalar_t momentLimit =
        projectedMass * settings_.maxAbsDeltaCom(axis);
    parameters_(axis + 1) =
        bounded(parameters_(axis + 1), -momentLimit, momentLimit);
  }
}

PayloadIdOutput PayloadIdModule::makeOutput(
    const payload_matrix6x4_t& regressor, const vector6_t& innovation,
    bool inputValid, bool updated, bool usedCustomRegressor) const {
  PayloadIdOutput output;
  output.inputValid = inputValid;
  output.updated = updated;
  output.usedCustomRegressor = usedCustomRegressor;
  output.deltaMass = parameters_(0);
  output.firstMoment = parameters_.tail<3>();
  if (parameters_(0) > settings_.massEpsilon) {
    output.deltaCom = output.firstMoment / parameters_(0);
  }
  output.innovation = innovation;
  output.regressor = regressor;
  if (inputValid) {
    output.equivalentWrench = regressor * parameters_;
  }
  output.parameterConfidence = information_.cwiseQuotient(
      information_ + settings_.confidenceInformationScale);
  output.confidence = output.parameterConfidence.mean();
  return output;
}

PayloadIdOutput PayloadIdModule::step(const PayloadIdInput& input,
                                      scalar_t dt) {
  if (!inputIsValid(input, dt)) {
    return makeOutput(payload_matrix6x4_t::Zero(), vector6_t::Zero(), false,
                      false, input.useCustomRegressor);
  }

  const payload_matrix6x4_t regressor =
      input.useCustomRegressor ? input.customRegressor
                               : makeAutomaticRegressor(input);
  vector6_t innovation = input.externalResidual - regressor * parameters_;
  const bool update = input.updateAllowed && dt > 0.0;
  if (update) {
    const scalar_t informationRetention =
        std::exp(-settings_.informationForgettingRate * dt);
    for (Eigen::Index parameter = 0; parameter < 4; ++parameter) {
      const scalar_t columnEnergy = regressor.col(parameter).squaredNorm();
      const scalar_t normalizedGradient =
          regressor.col(parameter).dot(innovation) /
          (settings_.normalizationEpsilon + columnEnergy);
      const scalar_t derivative =
          settings_.adaptationGain(parameter) * normalizedGradient -
          settings_.forgettingRate * parameters_(parameter);
      parameters_(parameter) += dt * derivative;
      information_(parameter) = informationRetention * information_(parameter) +
                                dt * columnEnergy;
    }
    projectParameters();
    innovation = input.externalResidual - regressor * parameters_;
  }
  return makeOutput(regressor, innovation, true, update,
                    input.useCustomRegressor);
}

void PayloadIdModule::reset() {
  parameters_.setZero();
  information_.setZero();
}

}  // namespace modules
}  // namespace adaptive
}  // namespace legged

