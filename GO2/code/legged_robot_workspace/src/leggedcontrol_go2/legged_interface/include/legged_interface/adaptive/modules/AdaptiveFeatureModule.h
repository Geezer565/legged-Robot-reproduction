#pragma once

#include <array>
#include <cstdint>

#include "legged_interface/adaptive/MyRbfV2FeatureInput.h"

namespace legged {
namespace adaptive {
namespace modules {

struct AdaptiveFeatureSettings {
  v2::Axes initialExtent{{1, 1, 1, 1, 1, 1}};
  v2::Axes minimumExtent{{0.5, 0.5, 0.5, 0.5, 0.5, 0.5}};
  v2::Axes maximumExtent{{8, 8, 8, 8, 8, 8}};
  double initialWidthRatio = 0.85;
  double extentSigmaMultiplier = 2.5;
  double maximumRelativeChange = 0.05;
  unsigned minimumSamples = 100;
  unsigned publishEverySamples = 50;
};

struct AdaptiveFeatureSuggestion {
  v2::Axes extent{};
  v2::Axes mean{};
  v2::Axes standardDeviation{};
  double widthRatio = 0.85;
  unsigned sampleCount = 0;
  std::uint64_t version = 0;
  bool ready = false;
};

// Slow-path feature-map adviser. It never mutates the active MyRBF map.
// A caller may consume a ready suggestion only at the MPC synchronization point.
class AdaptiveFeatureModule final {
 public:
  explicit AdaptiveFeatureModule(AdaptiveFeatureSettings settings = AdaptiveFeatureSettings{});

  AdaptiveFeatureSuggestion observe(const v2::FeatureInput& input, bool sampleValid);
  const AdaptiveFeatureSuggestion& current() const noexcept { return current_; }
  AdaptiveFeatureSuggestion consumePending();
  void reset();

 private:
  void validateSettings() const;
  void updateStatistics(const v2::Axes& sample);
  void prepareSuggestion();

  AdaptiveFeatureSettings settings_;
  AdaptiveFeatureSuggestion current_;
  v2::Axes m2_{};
};

}  // namespace modules
}  // namespace adaptive
}  // namespace legged

