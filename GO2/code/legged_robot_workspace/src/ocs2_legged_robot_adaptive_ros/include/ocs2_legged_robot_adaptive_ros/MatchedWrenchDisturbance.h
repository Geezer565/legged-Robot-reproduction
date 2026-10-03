#pragma once

#include <string>

#include <ocs2_core/Types.h>

#include <ocs2_legged_robot_adaptive/adaptive/AdaptiveTypes.h>

namespace ocs2 {
namespace legged_robot_adaptive_ros {

enum class MatchedWrenchDisturbanceProfile {
  Constant,
  SignReversal,
  Sine,
  PulseTrain,
  SineCosine,
};

MatchedWrenchDisturbanceProfile matchedWrenchDisturbanceProfileFromString(
    const std::string& value);
const char* toString(MatchedWrenchDisturbanceProfile profile);

struct MatchedWrenchDisturbanceConfig {
  bool enabled = false;
  scalar_t startTime = 0.0;
  scalar_t endTime = -1.0;
  legged_robot_adaptive::adaptive::vector6_t wrench =
      legged_robot_adaptive::adaptive::vector6_t::Zero();
  MatchedWrenchDisturbanceProfile profile =
      MatchedWrenchDisturbanceProfile::Constant;
  legged_robot_adaptive::adaptive::vector6_t secondaryWrench =
      legged_robot_adaptive::adaptive::vector6_t::Zero();
  scalar_t frequencyHz = 0.25;
};

/**
 * Applies a known matched world-frame wrench disturbance to animal_dummy only.
 *
 * The wrench uses world-frame [Fx, Fy, Fz, Tx, Ty, Tz] ordering and the same
 * normalized-momentum channel/sign as Eq. (10). A step ending exactly at
 * startTime is not disturbed; a finite endTime is inclusive. Constant uses
 * wrench throughout the interval. SignReversal changes to secondaryWrench at
 * the interval midpoint. Sine uses wrench as its amplitude vector. PulseTrain
 * alternates between wrench and secondaryWrench every half period. SineCosine
 * combines sine*wrench with cosine*secondaryWrench.
 */
class MatchedWrenchDisturbance final {
 public:
  explicit MatchedWrenchDisturbance(MatchedWrenchDisturbanceConfig config);

  bool isActive(scalar_t stepEndTime) const;
  legged_robot_adaptive::adaptive::vector6_t appliedWrench(scalar_t stepEndTime) const;
  void applyToState(scalar_t stepEndTime, scalar_t dt, scalar_t robotMass,
                    vector_t& state) const;

  const MatchedWrenchDisturbanceConfig& config() const { return config_; }

 private:
  MatchedWrenchDisturbanceConfig config_;
};

}  // namespace legged_robot_adaptive_ros
}  // namespace ocs2
