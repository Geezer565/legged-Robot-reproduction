#pragma once

#include "legged_interface/adaptive/AdaptiveTypes.h"

namespace legged {
namespace adaptive {

/** Returns a scaled Fy compensation after magnitude and slew-rate limits. */
scalar_t computeLimitedFyCompensation(
    scalar_t previousCompensation, scalar_t estimatedFy,
    scalar_t controlGain, scalar_t forceLimit, scalar_t rateLimit,
    scalar_t dt);

}  // namespace adaptive
}  // namespace legged
