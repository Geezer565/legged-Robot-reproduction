#include "legged_interface/adaptive/modules/AdaptiveFeatureModule.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace legged {
namespace adaptive {
namespace modules {
namespace {

bool finiteAxes(const v2::Axes& values) {
  for (double value : values) {
    if (!std::isfinite(value)) return false;
  }
  return true;
}

}  // namespace

AdaptiveFeatureModule::AdaptiveFeatureModule(AdaptiveFeatureSettings settings)
    : settings_(std::move(settings)) {
  validateSettings();
  reset();
}

void AdaptiveFeatureModule::validateSettings() const {
  if (!finiteAxes(settings_.initialExtent) || !finiteAxes(settings_.minimumExtent) ||
      !finiteAxes(settings_.maximumExtent)) {
    throw std::invalid_argument("[AdaptiveFeatureModule] extents must be finite.");
  }
  for (std::size_t axis = 0; axis < 6; ++axis) {
    if (settings_.minimumExtent[axis] <= 0.0 ||
        settings_.minimumExtent[axis] > settings_.initialExtent[axis] ||
        settings_.initialExtent[axis] > settings_.maximumExtent[axis]) {
      throw std::invalid_argument("[AdaptiveFeatureModule] extent bounds are invalid.");
    }
  }
  if (!std::isfinite(settings_.initialWidthRatio) || settings_.initialWidthRatio <= 0.0 ||
      !std::isfinite(settings_.extentSigmaMultiplier) || settings_.extentSigmaMultiplier <= 0.0 ||
      !std::isfinite(settings_.maximumRelativeChange) || settings_.maximumRelativeChange <= 0.0 ||
      settings_.maximumRelativeChange > 1.0 || settings_.minimumSamples == 0 ||
      settings_.publishEverySamples == 0) {
    throw std::invalid_argument("[AdaptiveFeatureModule] scalar settings are invalid.");
  }
}

void AdaptiveFeatureModule::reset() {
  current_ = AdaptiveFeatureSuggestion{};
  current_.extent = settings_.initialExtent;
  current_.widthRatio = settings_.initialWidthRatio;
  current_.mean.fill(0.0);
  current_.standardDeviation.fill(0.0);
  m2_.fill(0.0);
}

void AdaptiveFeatureModule::updateStatistics(const v2::Axes& sample) {
  ++current_.sampleCount;
  const double count = static_cast<double>(current_.sampleCount);
  for (std::size_t axis = 0; axis < 6; ++axis) {
    const double delta = sample[axis] - current_.mean[axis];
    current_.mean[axis] += delta / count;
    const double deltaAfter = sample[axis] - current_.mean[axis];
    m2_[axis] += delta * deltaAfter;
    current_.standardDeviation[axis] = current_.sampleCount > 1
        ? std::sqrt(std::max(0.0, m2_[axis] / static_cast<double>(current_.sampleCount - 1)))
        : 0.0;
  }
}

void AdaptiveFeatureModule::prepareSuggestion() {
  for (std::size_t axis = 0; axis < 6; ++axis) {
    const double target = std::max(
        settings_.minimumExtent[axis],
        std::min(settings_.maximumExtent[axis],
                 settings_.extentSigmaMultiplier * current_.standardDeviation[axis]));
    const double lower = current_.extent[axis] * (1.0 - settings_.maximumRelativeChange);
    const double upper = current_.extent[axis] * (1.0 + settings_.maximumRelativeChange);
    current_.extent[axis] = std::max(lower, std::min(upper, target));
  }
  ++current_.version;
  current_.ready = true;
}

AdaptiveFeatureSuggestion AdaptiveFeatureModule::observe(const v2::FeatureInput& input,
                                                          bool sampleValid) {
  if (!finiteAxes(input.causalResidual)) {
    throw std::invalid_argument("[AdaptiveFeatureModule] causalResidual must be finite.");
  }
  if (!sampleValid) return current_;

  updateStatistics(input.causalResidual);
  if (!current_.ready && current_.sampleCount >= settings_.minimumSamples &&
      (current_.sampleCount - settings_.minimumSamples) % settings_.publishEverySamples == 0) {
    prepareSuggestion();
  }
  return current_;
}

AdaptiveFeatureSuggestion AdaptiveFeatureModule::consumePending() {
  AdaptiveFeatureSuggestion result = current_;
  current_.ready = false;
  return result;
}

}  // namespace modules
}  // namespace adaptive
}  // namespace legged

