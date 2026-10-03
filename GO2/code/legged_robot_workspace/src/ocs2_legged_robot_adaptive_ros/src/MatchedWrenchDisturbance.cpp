#include <ocs2_legged_robot_adaptive_ros/MatchedWrenchDisturbance.h>

#include <cmath>
#include <stdexcept>
#include <utility>

#include <ocs2_legged_robot_adaptive/dynamics/AdaptiveLeggedRobotDynamics.h>

namespace ocs2 {
namespace legged_robot_adaptive_ros {

using legged_robot_adaptive::AdaptiveLeggedRobotDynamics;
using legged_robot_adaptive::adaptive::vector6_t;

MatchedWrenchDisturbanceProfile matchedWrenchDisturbanceProfileFromString(
    const std::string& value) {
  if (value == "constant") {
    return MatchedWrenchDisturbanceProfile::Constant;
  }
  if (value == "sign_reversal") {
    return MatchedWrenchDisturbanceProfile::SignReversal;
  }
  if (value == "sine") {
    return MatchedWrenchDisturbanceProfile::Sine;
  }
  if (value == "pulse_train") {
    return MatchedWrenchDisturbanceProfile::PulseTrain;
  }
  if (value == "sine_cosine") {
    return MatchedWrenchDisturbanceProfile::SineCosine;
  }
  throw std::invalid_argument("unknown matched disturbance profile: " + value);
}

const char* toString(MatchedWrenchDisturbanceProfile profile) {
  switch (profile) {
    case MatchedWrenchDisturbanceProfile::Constant:
      return "constant";
    case MatchedWrenchDisturbanceProfile::SignReversal:
      return "sign_reversal";
    case MatchedWrenchDisturbanceProfile::Sine:
      return "sine";
    case MatchedWrenchDisturbanceProfile::PulseTrain:
      return "pulse_train";
    case MatchedWrenchDisturbanceProfile::SineCosine:
      return "sine_cosine";
  }
  throw std::invalid_argument("invalid matched disturbance profile enum");
}

MatchedWrenchDisturbance::MatchedWrenchDisturbance(MatchedWrenchDisturbanceConfig config)
    : config_(std::move(config)) {
  if (!std::isfinite(config_.startTime)) {
    throw std::invalid_argument("matched disturbance startTime must be finite");
  }
  if (!(config_.endTime < 0.0) &&
      (!std::isfinite(config_.endTime) || !(config_.endTime > config_.startTime))) {
    throw std::invalid_argument("matched disturbance endTime must be negative or greater than startTime");
  }
  if (!config_.wrench.allFinite()) {
    throw std::invalid_argument("matched disturbance wrench must be finite");
  }
  if (!config_.secondaryWrench.allFinite()) {
    throw std::invalid_argument("matched disturbance secondaryWrench must be finite");
  }
  if (!(config_.frequencyHz > 0.0) || !std::isfinite(config_.frequencyHz)) {
    throw std::invalid_argument("matched disturbance frequencyHz must be positive and finite");
  }
  if (config_.profile == MatchedWrenchDisturbanceProfile::SignReversal &&
      config_.endTime < 0.0) {
    throw std::invalid_argument("sign_reversal profile requires a finite endTime");
  }
}

bool MatchedWrenchDisturbance::isActive(scalar_t stepEndTime) const {
  if (!std::isfinite(stepEndTime)) {
    throw std::invalid_argument("matched disturbance stepEndTime must be finite");
  }
  return config_.enabled && stepEndTime > config_.startTime &&
         (config_.endTime < 0.0 || stepEndTime <= config_.endTime);
}

vector6_t MatchedWrenchDisturbance::appliedWrench(scalar_t stepEndTime) const {
  if (!isActive(stepEndTime)) {
    return vector6_t::Zero();
  }
  switch (config_.profile) {
    case MatchedWrenchDisturbanceProfile::Constant:
      return config_.wrench;
    case MatchedWrenchDisturbanceProfile::SignReversal: {
      if (config_.endTime < 0.0) {
        throw std::logic_error("sign_reversal profile requires a finite endTime");
      }
      const scalar_t midpoint = 0.5 * (config_.startTime + config_.endTime);
      return stepEndTime <= midpoint ? config_.wrench : config_.secondaryWrench;
    }
    case MatchedWrenchDisturbanceProfile::Sine: {
      constexpr scalar_t twoPi = 6.283185307179586476925286766559;
      return std::sin(twoPi * config_.frequencyHz *
                      (stepEndTime - config_.startTime)) * config_.wrench;
    }
    case MatchedWrenchDisturbanceProfile::PulseTrain: {
      const scalar_t cycles =
          config_.frequencyHz * (stepEndTime - config_.startTime);
      const scalar_t phase = cycles - std::floor(cycles);
      return phase < 0.5 ? config_.wrench : config_.secondaryWrench;
    }
    case MatchedWrenchDisturbanceProfile::SineCosine: {
      constexpr scalar_t twoPi = 6.283185307179586476925286766559;
      const scalar_t phase = twoPi * config_.frequencyHz *
                             (stepEndTime - config_.startTime);
      return std::sin(phase) * config_.wrench +
             std::cos(phase) * config_.secondaryWrench;
    }
  }
  throw std::logic_error("invalid matched disturbance profile enum");
}

void MatchedWrenchDisturbance::applyToState(scalar_t stepEndTime, scalar_t dt,
                                            scalar_t robotMass, vector_t& state) const {
  if (!(dt > 0.0) || !std::isfinite(dt)) {
    throw std::invalid_argument("matched disturbance dt must be positive and finite");
  }
  if (!(robotMass > 0.0) || !std::isfinite(robotMass)) {
    throw std::invalid_argument("matched disturbance robotMass must be positive and finite");
  }
  if (state.size() < 6 || !state.allFinite()) {
    throw std::invalid_argument("matched disturbance state must be finite with at least 6 entries");
  }
  const vector6_t wrench = appliedWrench(stepEndTime);
  if (wrench.isZero(0.0)) {
    return;
  }
  state.head<6>() +=
      dt * AdaptiveLeggedRobotDynamics::normalizedMomentumRateCorrection(wrench, robotMass);
}

}  // namespace legged_robot_adaptive_ros
}  // namespace ocs2
