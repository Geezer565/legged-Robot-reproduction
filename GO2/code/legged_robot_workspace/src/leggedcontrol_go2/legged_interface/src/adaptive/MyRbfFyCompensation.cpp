#include "legged_interface/adaptive/MyRbfFyCompensation.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace legged {
namespace adaptive {

scalar_t computeLimitedFyCompensation(
    scalar_t previousCompensation, scalar_t estimatedFy,
    scalar_t controlGain, scalar_t forceLimit, scalar_t rateLimit,
    scalar_t dt) {
  if (!std::isfinite(previousCompensation) || !std::isfinite(estimatedFy) ||
      !std::isfinite(controlGain) || !std::isfinite(forceLimit) ||
      !std::isfinite(rateLimit) || !std::isfinite(dt) ||
      controlGain < 0.0 || controlGain > 1.0 || forceLimit <= 0.0 ||
      rateLimit <= 0.0 || dt < 0.0) {
    throw std::invalid_argument(
        "[MyRbfFyCompensation] Invalid gain, limit, rate, or sample.");
  }

  const scalar_t target = std::max(
      -forceLimit, std::min(forceLimit, controlGain * estimatedFy));
  const scalar_t maxDelta = rateLimit * dt;
  const scalar_t delta = std::max(
      -maxDelta, std::min(maxDelta, target - previousCompensation));
  return std::max(-forceLimit,
                  std::min(forceLimit, previousCompensation + delta));
}

}  // namespace adaptive
}  // namespace legged
