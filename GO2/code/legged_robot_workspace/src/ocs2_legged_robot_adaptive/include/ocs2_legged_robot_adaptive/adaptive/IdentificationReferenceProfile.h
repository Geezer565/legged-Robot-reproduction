#pragma once

#include <array>
#include <string>

#include "ocs2_legged_robot_adaptive/adaptive/AdaptiveTypes.h"

namespace ocs2 {
namespace legged_robot_adaptive {
namespace adaptive {

/** D5I richer-reference candidates.  These names are experiment identities. */
enum class IdentificationExcitationCandidate {
  E0LegacyXStep,
  E1Translation,
  E2Rotation,
  E3Combined
};

const char* identificationExcitationCandidateName(
    IdentificationExcitationCandidate candidate);

struct IdentificationReferenceProfileSettings {
  scalar_t period = 12.0;
  vector3_t translationAmplitudeWorld =
      (vector3_t() << 0.020, 0.015, 0.008).finished();
  /** ZYX order: [yaw, pitch, roll] = [0.040, 0.025, 0.025] rad. */
  vector3_t eulerZyxAmplitude =
      (vector3_t() << 0.040, 0.025, 0.025).finished();
  std::array<int, 3> translationHarmonics{{1, 2, 3}};
  /** D5I1 search-selected conservative ZYX harmonics (8,12,12). */
  std::array<int, 3> eulerZyxHarmonics{{8, 12, 12}};
  vector3_t translationPhase =
      (vector3_t() << 0.0, 2.0943951023931953,
       -0.7853981633974483).finished();
  vector3_t eulerZyxPhase =
      (vector3_t() << 0.4487989505128276, -0.7853981633974483,
       1.0471975511965976).finished();
  scalar_t legacyTargetStartTime = 1.2;
  scalar_t legacyTargetDuration = 0.2;
  scalar_t legacyXStep = 0.10;
};

/**
 * One self-consistent analytic reference sample.  Translation is the desired
 * nominal-CoM offset in world axes.  Orientation and its derivatives use ZYX
 * order; angular velocity/acceleration are expressed in world axes.
 */
struct IdentificationReferenceSample {
  vector3_t comPositionOffsetWorld = vector3_t::Zero();
  vector3_t comVelocityWorld = vector3_t::Zero();
  vector3_t comAccelerationWorld = vector3_t::Zero();
  vector3_t eulerZyx = vector3_t::Zero();
  vector3_t eulerZyxDerivative = vector3_t::Zero();
  vector3_t eulerZyxSecondDerivative = vector3_t::Zero();
  vector3_t angularVelocityWorld = vector3_t::Zero();
  vector3_t angularAccelerationWorld = vector3_t::Zero();
};

/** Explicit D5I2 online schedule.  All times use controller/MPC time. */
struct IdentificationReferenceScheduleSettings {
  scalar_t profileStartTime = 2.4;
  scalar_t rampInDuration = 2.4;
  scalar_t steadyDuration = 12.0;
  scalar_t rampOutDuration = 2.4;
  scalar_t settleDuration = 1.2;
};

enum class IdentificationReferenceStage {
  PreStart = 0,
  RampIn = 1,
  Steady = 2,
  RampOut = 3,
  Settle = 4,
  Complete = 5
};

const char* identificationReferenceStageName(
    IdentificationReferenceStage stage);

/** Scheduled sample plus the exact C2 envelope provenance. */
struct ScheduledIdentificationReferenceSample {
  IdentificationReferenceSample reference;
  IdentificationReferenceStage stage = IdentificationReferenceStage::PreStart;
  scalar_t profileElapsedTime = 0.0;
  scalar_t profilePhase = 0.0;
  scalar_t envelope = 0.0;
  scalar_t envelopeDerivative = 0.0;
  scalar_t envelopeSecondDerivative = 0.0;
};

/** Pure-math, ROS-free D5I1 profile.  It does not publish or mutate a plant. */
class IdentificationReferenceProfile final {
 public:
  explicit IdentificationReferenceProfile(
      IdentificationReferenceProfileSettings settings = {});

  IdentificationReferenceSample sample(
      IdentificationExcitationCandidate candidate, scalar_t time) const;
  const IdentificationReferenceProfileSettings& settings() const {
    return settings_;
  }
  std::string profileId() const;

 private:
  IdentificationReferenceProfileSettings settings_;
};

/**
 * Default-off D5I2 time schedule around IdentificationReferenceProfile.
 * The quintic envelope has zero value/velocity/acceleration at its inactive
 * endpoint, so ramp-in and ramp-out do not introduce derivative jumps.
 */
class IdentificationReferenceSchedule final {
 public:
  IdentificationReferenceSchedule(
      IdentificationReferenceProfile profile = IdentificationReferenceProfile(),
      IdentificationReferenceScheduleSettings settings = {});

  ScheduledIdentificationReferenceSample sample(
      IdentificationExcitationCandidate candidate, scalar_t mpcTime) const;
  const IdentificationReferenceScheduleSettings& settings() const {
    return settings_;
  }
  scalar_t rampInEndTime() const;
  scalar_t steadyEndTime() const;
  scalar_t rampOutEndTime() const;
  scalar_t completeTime() const;

 private:
  IdentificationReferenceProfile profile_;
  IdentificationReferenceScheduleSettings settings_;
};

}  // namespace adaptive
}  // namespace legged_robot_adaptive
}  // namespace ocs2
