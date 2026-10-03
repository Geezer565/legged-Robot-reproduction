#include "legged_interface/adaptive/MyRbfV5DynamicsCore.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace legged {
namespace adaptive {
namespace v5 {
namespace {

double clamp(double value, double limit) {
  return std::max(-limit, std::min(limit, value));
}

bool finite(double value) {
  return std::isfinite(value);
}

template <typename Values, typename Centers>
void fillAxisCenters(Centers& centers, double extent) {
  for (auto& center : centers) center.fill(0.0);
  for (std::size_t dimension = 0; dimension < Values{}.size(); ++dimension) {
    centers[1 + 2 * dimension][dimension] = extent;
    centers[2 + 2 * dimension][dimension] = -extent;
  }
}

template <typename Values, typename Centers>
void appendGaussianBank(const Values& input, const Centers& centers,
                        double width, Basis& result, std::size_t& offset) {
  const double denominator = 2.0 * width * width;
  for (const auto& center : centers) {
    double squaredDistance = 0.0;
    for (std::size_t dimension = 0; dimension < input.size(); ++dimension) {
      const double delta = input[dimension] - center[dimension];
      squaredDistance += delta * delta;
    }
    result[offset++] = std::exp(-squaredDistance / denominator);
  }
}

}  // namespace

DynamicsCore::DynamicsCore(Settings settings) : settings_(settings) {
  if (!finite(settings_.centerExtent) || settings_.centerExtent <= 0.0 ||
      !finite(settings_.motionWidth) || settings_.motionWidth <= 0.0 ||
      !finite(settings_.historyWidth) || settings_.historyWidth <= 0.0 ||
      !finite(settings_.contactWidth) || settings_.contactWidth <= 0.0 ||
      !finite(settings_.inputClip) || settings_.inputClip <= 0.0 ||
      !finite(settings_.epsilon) || settings_.epsilon <= 0.0) {
    throw std::invalid_argument("V5 RBF geometry must be finite and positive");
  }
  if (!finite(settings_.gainAttackTau) || settings_.gainAttackTau <= 0.0 ||
      !finite(settings_.gainReleaseTau) || settings_.gainReleaseTau <= 0.0) {
    throw std::invalid_argument("RBF gain time constants must be positive");
  }
  for (std::size_t input = 0; input < kMotionInputs; ++input) {
    if (!finite(settings_.inputScale[input]) ||
        settings_.inputScale[input] <= 0.0 ||
        !finite(settings_.historyScale[input]) ||
        settings_.historyScale[input] <= 0.0) {
      throw std::invalid_argument("V5 input scales must be finite and positive");
    }
  }
  for (std::size_t axis = 0; axis < kAxes; ++axis) {
    if (!finite(settings_.gamma[axis]) || settings_.gamma[axis] <= 0.0 ||
        !finite(settings_.leakage[axis]) || settings_.leakage[axis] < 0.0 ||
        !finite(settings_.weightLimit[axis]) ||
        settings_.weightLimit[axis] <= 0.0 ||
        !finite(settings_.outputLimit[axis]) ||
        settings_.outputLimit[axis] <= 0.0 ||
        !finite(settings_.gainMultiplierMax[axis]) ||
        settings_.gainMultiplierMax[axis] < 1.0 ||
        !finite(settings_.uncertaintyScale[axis]) ||
        settings_.uncertaintyScale[axis] <= 0.0) {
      throw std::invalid_argument("V5 update settings are invalid");
    }
  }

  fillAxisCenters<Input>(motionCenters_, settings_.centerExtent);
  fillAxisCenters<Input>(historyCenters_, settings_.centerExtent);
  for (std::size_t mode = 0; mode < kContactNodes; ++mode) {
    for (std::size_t contact = 0; contact < kContactInputs; ++contact) {
      contactCenters_[mode][contact] =
          (mode & (std::size_t{1} << contact)) ? 1.0 : 0.0;
    }
  }
  reset();
}

void DynamicsCore::validateAxes(const Axes& values, const char* name) const {
  for (double value : values) {
    if (!finite(value)) {
      throw std::invalid_argument(std::string("V5 ") + name +
                                  " must be finite");
    }
  }
}

Input DynamicsCore::normalizedMotion(const Features& features) const {
  validateAxes(features.velocity, "velocity");
  validateAxes(features.nominalRate, "nominal rate");
  Input result{};
  for (std::size_t axis = 0; axis < kAxes; ++axis) {
    result[axis] = clamp(
        features.velocity[axis] / settings_.inputScale[axis],
        settings_.inputClip);
    result[kAxes + axis] = clamp(
        features.nominalRate[axis] / settings_.inputScale[kAxes + axis],
        settings_.inputClip);
  }
  return result;
}

Input DynamicsCore::normalizedHistory(const Features& features) const {
  validateAxes(features.residualMemory, "residual memory");
  validateAxes(features.residualTrend, "residual trend");
  Input result{};
  for (std::size_t axis = 0; axis < kAxes; ++axis) {
    result[axis] = clamp(
        features.residualMemory[axis] / settings_.historyScale[axis],
        settings_.inputClip);
    result[kAxes + axis] = clamp(
        features.residualTrend[axis] /
            settings_.historyScale[kAxes + axis],
        settings_.inputClip);
  }
  return result;
}

Basis DynamicsCore::basis(const Features& features) const {
  Basis result{};
  Contacts contacts{};
  for (std::size_t contact = 0; contact < kContactInputs; ++contact) {
    if (!finite(features.contacts[contact])) {
      throw std::invalid_argument("V5 contacts must be finite");
    }
    contacts[contact] = std::max(0.0, std::min(1.0, features.contacts[contact]));
  }
  std::size_t offset = 0;
  appendGaussianBank(normalizedMotion(features), motionCenters_,
                     settings_.motionWidth, result, offset);
  appendGaussianBank(normalizedHistory(features), historyCenters_,
                     settings_.historyWidth, result, offset);
  appendGaussianBank(contacts, contactCenters_, settings_.contactWidth,
                     result, offset);
  result[offset] = 1.0;
  return result;
}

Evaluation DynamicsCore::evaluate(const Features& features,
                                  bool enabled) const {
  Evaluation result;
  result.basis = basis(features);
  for (std::size_t axis = 0; axis < kAxes; ++axis) {
    double correction = 0.0;
    if (enabled) {
      for (std::size_t node = 0; node < kBasisSize; ++node) {
        correction += weights_[axis][node] * result.basis[node];
      }
      correction = clamp(correction, settings_.outputLimit[axis]);
    }
    result.correction[axis] = correction;
    result.totalRate[axis] = features.nominalRate[axis] + correction;
  }
  return result;
}

UpdateResult DynamicsCore::update(const Features& features,
                                  const Axes& targetResidual, double dt,
                                  bool enabled) {
  validateAxes(targetResidual, "target residual");
  if (!finite(dt) || dt <= 0.0) {
    throw std::invalid_argument("V5 update dt must be finite and positive");
  }
  UpdateResult result;
  result.before = evaluate(features, enabled);
  result.targetResidual = targetResidual;
  for (std::size_t axis = 0; axis < kAxes; ++axis) {
    result.predictionError[axis] =
        targetResidual[axis] - result.before.correction[axis];
    result.gainEvidence[axis] = std::min(
        1.0, std::abs(result.predictionError[axis]) /
                 settings_.uncertaintyScale[axis]);
    const double previous = gainEvidenceMemory_[axis];
    const double tau = result.gainEvidence[axis] > previous
                           ? settings_.gainAttackTau
                           : settings_.gainReleaseTau;
    gainEvidenceMemory_[axis] = std::max(
        0.0, std::min(1.0, previous - std::expm1(-dt / tau) *
                                    (result.gainEvidence[axis] - previous)));
    result.gainSmoothedEvidence[axis] = gainEvidenceMemory_[axis];
    result.gainMultiplier[axis] = 1.0 +
        (settings_.gainMultiplierMax[axis] - 1.0) *
            gainEvidenceMemory_[axis] * gainEvidenceMemory_[axis];
    result.effectiveGamma[axis] =
        settings_.gamma[axis] * result.gainMultiplier[axis];
  }
  if (enabled) {
    double basisNormSquared = settings_.epsilon;
    for (double value : result.before.basis) {
      basisNormSquared += value * value;
    }
    for (std::size_t axis = 0; axis < kAxes; ++axis) {
      for (std::size_t node = 0; node < kBasisSize; ++node) {
        const double derivative =
            result.effectiveGamma[axis] * result.before.basis[node] *
                result.predictionError[axis] / basisNormSquared -
            settings_.leakage[axis] * weights_[axis][node];
        weights_[axis][node] = clamp(
            weights_[axis][node] + dt * derivative,
            settings_.weightLimit[axis]);
      }
    }
  }
  result.after = evaluate(features, enabled);
  return result;
}

Axes DynamicsCore::predictNextVelocity(const Features& features, double dt,
                                       bool enabled) const {
  if (!finite(dt) || dt <= 0.0) {
    throw std::invalid_argument("V5 prediction dt must be finite and positive");
  }
  const auto evaluation = evaluate(features, enabled);
  Axes result{};
  for (std::size_t axis = 0; axis < kAxes; ++axis) {
    result[axis] =
        features.velocity[axis] + dt * evaluation.totalRate[axis];
  }
  return result;
}

void DynamicsCore::reset() {
  for (auto& axisWeights : weights_) {
    axisWeights.fill(0.0);
  }
  gainEvidenceMemory_.fill(0.0);
}

}  // namespace v5
}  // namespace adaptive
}  // namespace legged
