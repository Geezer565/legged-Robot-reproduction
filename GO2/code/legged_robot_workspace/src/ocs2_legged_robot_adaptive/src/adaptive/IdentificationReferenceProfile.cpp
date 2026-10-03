#include "ocs2_legged_robot_adaptive/adaptive/IdentificationReferenceProfile.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <sstream>
#include <utility>

#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>

namespace ocs2 {
namespace legged_robot_adaptive {
namespace adaptive {
namespace {

constexpr scalar_t kPi = 3.141592653589793238462643383279502884;

void setHarmonic(vector3_t& position, vector3_t& velocity,
                 vector3_t& acceleration, const vector3_t& amplitude,
                 const std::array<int, 3>& harmonic,
                 const vector3_t& phase, scalar_t fundamental,
                 scalar_t time) {
  for (Eigen::Index i = 0; i < 3; ++i) {
    const scalar_t omega = harmonic[static_cast<std::size_t>(i)] * fundamental;
    const scalar_t angle = omega * time + phase(i);
    position(i) = amplitude(i) * std::sin(angle);
    velocity(i) = amplitude(i) * omega * std::cos(angle);
    acceleration(i) = -amplitude(i) * omega * omega * std::sin(angle);
  }
}

void minimumJerkEnvelope(scalar_t normalizedTime, scalar_t duration,
                         scalar_t& value, scalar_t& derivative,
                         scalar_t& secondDerivative) {
  const scalar_t u = std::max<scalar_t>(0.0, std::min<scalar_t>(1.0,
                                                               normalizedTime));
  const scalar_t u2 = u * u;
  const scalar_t u3 = u2 * u;
  const scalar_t u4 = u3 * u;
  const scalar_t u5 = u4 * u;
  value = 10.0 * u3 - 15.0 * u4 + 6.0 * u5;
  derivative = (30.0 * u2 - 60.0 * u3 + 30.0 * u4) / duration;
  secondDerivative = (60.0 * u - 180.0 * u2 + 120.0 * u3) /
                     (duration * duration);
}

}  // namespace

const char* identificationExcitationCandidateName(
    IdentificationExcitationCandidate candidate) {
  switch (candidate) {
    case IdentificationExcitationCandidate::E0LegacyXStep:
      return "E0_legacy_x_step";
    case IdentificationExcitationCandidate::E1Translation:
      return "E1_translation";
    case IdentificationExcitationCandidate::E2Rotation:
      return "E2_rotation";
    case IdentificationExcitationCandidate::E3Combined:
      return "E3_combined";
  }
  throw std::invalid_argument("[IdentificationReferenceProfile] unknown candidate.");
}

const char* identificationReferenceStageName(
    IdentificationReferenceStage stage) {
  switch (stage) {
    case IdentificationReferenceStage::PreStart:
      return "pre_start";
    case IdentificationReferenceStage::RampIn:
      return "ramp_in";
    case IdentificationReferenceStage::Steady:
      return "steady";
    case IdentificationReferenceStage::RampOut:
      return "ramp_out";
    case IdentificationReferenceStage::Settle:
      return "settle";
    case IdentificationReferenceStage::Complete:
      return "complete";
  }
  throw std::invalid_argument(
      "[IdentificationReferenceSchedule] unknown stage.");
}

IdentificationReferenceProfile::IdentificationReferenceProfile(
    IdentificationReferenceProfileSettings settings)
    : settings_(std::move(settings)) {
  if (!std::isfinite(settings_.period) || !(settings_.period > 0.0) ||
      !settings_.translationAmplitudeWorld.allFinite() ||
      !settings_.eulerZyxAmplitude.allFinite() ||
      !settings_.translationPhase.allFinite() ||
      !settings_.eulerZyxPhase.allFinite() ||
      (settings_.translationAmplitudeWorld.array() < 0.0).any() ||
      (settings_.eulerZyxAmplitude.array() < 0.0).any() ||
      !std::isfinite(settings_.legacyTargetStartTime) ||
      !std::isfinite(settings_.legacyTargetDuration) ||
      !(settings_.legacyTargetDuration > 0.0) ||
      !std::isfinite(settings_.legacyXStep) ||
      std::any_of(settings_.translationHarmonics.begin(),
                  settings_.translationHarmonics.end(),
                  [](int value) { return value <= 0; }) ||
      std::any_of(settings_.eulerZyxHarmonics.begin(),
                  settings_.eulerZyxHarmonics.end(),
                  [](int value) { return value <= 0; })) {
    throw std::invalid_argument(
        "[IdentificationReferenceProfile] invalid profile settings.");
  }
}

IdentificationReferenceSample IdentificationReferenceProfile::sample(
    IdentificationExcitationCandidate candidate, scalar_t time) const {
  if (!std::isfinite(time)) {
    throw std::invalid_argument(
        "[IdentificationReferenceProfile] time must be finite.");
  }
  IdentificationReferenceSample result;
  if (candidate == IdentificationExcitationCandidate::E0LegacyXStep) {
    const scalar_t elapsed = time - settings_.legacyTargetStartTime;
    if (elapsed <= 0.0) {
      return result;
    }
    if (elapsed < settings_.legacyTargetDuration) {
      result.comPositionOffsetWorld.x() =
          settings_.legacyXStep * elapsed / settings_.legacyTargetDuration;
      result.comVelocityWorld.x() =
          settings_.legacyXStep / settings_.legacyTargetDuration;
    } else {
      result.comPositionOffsetWorld.x() = settings_.legacyXStep;
    }
    return result;
  }

  const scalar_t fundamental = 2.0 * kPi / settings_.period;
  if (candidate == IdentificationExcitationCandidate::E1Translation ||
      candidate == IdentificationExcitationCandidate::E3Combined) {
    setHarmonic(result.comPositionOffsetWorld, result.comVelocityWorld,
                result.comAccelerationWorld,
                settings_.translationAmplitudeWorld,
                settings_.translationHarmonics,
                settings_.translationPhase,
                fundamental, time);
  }
  if (candidate == IdentificationExcitationCandidate::E2Rotation ||
      candidate == IdentificationExcitationCandidate::E3Combined) {
    setHarmonic(result.eulerZyx, result.eulerZyxDerivative,
                result.eulerZyxSecondDerivative,
                settings_.eulerZyxAmplitude,
                settings_.eulerZyxHarmonics,
                settings_.eulerZyxPhase,
                fundamental, time);
    result.angularVelocityWorld =
        getGlobalAngularVelocityFromEulerAnglesZyxDerivatives<scalar_t>(
            result.eulerZyx, result.eulerZyxDerivative);
    result.angularAccelerationWorld =
        getGlobalAngularAccelerationFromEulerAnglesZyxDerivatives<scalar_t>(
            result.eulerZyx, result.eulerZyxDerivative,
            result.eulerZyxSecondDerivative);
  }
  return result;
}

std::string IdentificationReferenceProfile::profileId() const {
  std::ostringstream stream;
  stream << "d5i1_multisine_v1_T" << settings_.period
         << "_translation_h";
  for (const int value : settings_.translationHarmonics) stream << value;
  stream << "_rotation_zyx_h";
  for (const int value : settings_.eulerZyxHarmonics) stream << value;
  return stream.str();
}

IdentificationReferenceSchedule::IdentificationReferenceSchedule(
    IdentificationReferenceProfile profile,
    IdentificationReferenceScheduleSettings settings)
    : profile_(std::move(profile)), settings_(std::move(settings)) {
  if (!std::isfinite(settings_.profileStartTime) ||
      settings_.profileStartTime < 0.0 ||
      !std::isfinite(settings_.rampInDuration) ||
      !(settings_.rampInDuration > 0.0) ||
      !std::isfinite(settings_.steadyDuration) ||
      !(settings_.steadyDuration > 0.0) ||
      !std::isfinite(settings_.rampOutDuration) ||
      !(settings_.rampOutDuration > 0.0) ||
      !std::isfinite(settings_.settleDuration) ||
      settings_.settleDuration < 0.0) {
    throw std::invalid_argument(
        "[IdentificationReferenceSchedule] invalid schedule settings.");
  }
}

scalar_t IdentificationReferenceSchedule::rampInEndTime() const {
  return settings_.profileStartTime + settings_.rampInDuration;
}

scalar_t IdentificationReferenceSchedule::steadyEndTime() const {
  return rampInEndTime() + settings_.steadyDuration;
}

scalar_t IdentificationReferenceSchedule::rampOutEndTime() const {
  return steadyEndTime() + settings_.rampOutDuration;
}

scalar_t IdentificationReferenceSchedule::completeTime() const {
  return rampOutEndTime() + settings_.settleDuration;
}

ScheduledIdentificationReferenceSample IdentificationReferenceSchedule::sample(
    IdentificationExcitationCandidate candidate, scalar_t mpcTime) const {
  if (!std::isfinite(mpcTime)) {
    throw std::invalid_argument(
        "[IdentificationReferenceSchedule] time must be finite.");
  }
  if (candidate == IdentificationExcitationCandidate::E0LegacyXStep) {
    throw std::invalid_argument(
        "[IdentificationReferenceSchedule] E0 is not an online richer-reference candidate.");
  }

  ScheduledIdentificationReferenceSample result;
  if (mpcTime < settings_.profileStartTime) {
    return result;
  }

  result.profileElapsedTime = mpcTime - settings_.profileStartTime;
  result.profilePhase = std::fmod(result.profileElapsedTime,
                                  profile_.settings().period) /
                        profile_.settings().period;
  if (result.profilePhase < 0.0) result.profilePhase += 1.0;

  if (mpcTime < rampInEndTime()) {
    result.stage = IdentificationReferenceStage::RampIn;
    minimumJerkEnvelope(
        (mpcTime - settings_.profileStartTime) / settings_.rampInDuration,
        settings_.rampInDuration, result.envelope,
        result.envelopeDerivative, result.envelopeSecondDerivative);
  } else if (mpcTime < steadyEndTime()) {
    result.stage = IdentificationReferenceStage::Steady;
    result.envelope = 1.0;
  } else if (mpcTime < rampOutEndTime()) {
    result.stage = IdentificationReferenceStage::RampOut;
    scalar_t rising = 0.0, risingDerivative = 0.0,
             risingSecondDerivative = 0.0;
    minimumJerkEnvelope(
        (mpcTime - steadyEndTime()) / settings_.rampOutDuration,
        settings_.rampOutDuration, rising, risingDerivative,
        risingSecondDerivative);
    result.envelope = 1.0 - rising;
    result.envelopeDerivative = -risingDerivative;
    result.envelopeSecondDerivative = -risingSecondDerivative;
  } else if (mpcTime < completeTime()) {
    result.stage = IdentificationReferenceStage::Settle;
    return result;
  } else {
    result.stage = IdentificationReferenceStage::Complete;
    return result;
  }

  const auto raw = profile_.sample(candidate, result.profileElapsedTime);
  const scalar_t e = result.envelope;
  const scalar_t ed = result.envelopeDerivative;
  const scalar_t edd = result.envelopeSecondDerivative;
  result.reference.comPositionOffsetWorld = e * raw.comPositionOffsetWorld;
  result.reference.comVelocityWorld =
      ed * raw.comPositionOffsetWorld + e * raw.comVelocityWorld;
  result.reference.comAccelerationWorld =
      edd * raw.comPositionOffsetWorld + 2.0 * ed * raw.comVelocityWorld +
      e * raw.comAccelerationWorld;
  result.reference.eulerZyx = e * raw.eulerZyx;
  result.reference.eulerZyxDerivative =
      ed * raw.eulerZyx + e * raw.eulerZyxDerivative;
  result.reference.eulerZyxSecondDerivative =
      edd * raw.eulerZyx + 2.0 * ed * raw.eulerZyxDerivative +
      e * raw.eulerZyxSecondDerivative;
  result.reference.angularVelocityWorld =
      getGlobalAngularVelocityFromEulerAnglesZyxDerivatives<scalar_t>(
          result.reference.eulerZyx,
          result.reference.eulerZyxDerivative);
  result.reference.angularAccelerationWorld =
      getGlobalAngularAccelerationFromEulerAnglesZyxDerivatives<scalar_t>(
          result.reference.eulerZyx,
          result.reference.eulerZyxDerivative,
          result.reference.eulerZyxSecondDerivative);
  return result;
}

}  // namespace adaptive
}  // namespace legged_robot_adaptive
}  // namespace ocs2
