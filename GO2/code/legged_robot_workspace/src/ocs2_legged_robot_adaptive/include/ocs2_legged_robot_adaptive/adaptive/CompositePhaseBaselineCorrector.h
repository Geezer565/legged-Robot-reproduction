#pragma once

#include <array>
#include <cstddef>
#include <map>
#include <string>
#include <tuple>

#include "ocs2_legged_robot_adaptive/adaptive/AdaptiveTypes.h"

namespace ocs2 {
namespace legged_robot_adaptive {
namespace adaptive {

/** Causal lookup result for the immutable D5I3 E3 composite-phase baseline. */
struct CompositePhaseBaselineSample {
  parameter_vector_t baseline = parameter_vector_t::Zero(10);
  bool gaitPhaseValid = false;
  bool steadyStageActive = false;
  bool keySupported = false;
  bool correctionActive = false;
  bool cycleBoundary = false;
  int profilePhaseBin = -1;
  int gaitPhaseBin = -1;
  std::size_t mode = 0;
  scalar_t profilePhase = -1.0;
  scalar_t gaitPhase = -1.0;
  scalar_t cycleStart = -1.0;
};

/**
 * Default-off D5I4 correction using the frozen nominal-only D5I3 template.
 *
 * The sparse key is (E3 profile phase bin, causally reconstructed executed
 * gait phase bin, current executed mode).  Lookup is active only in the
 * configured steady-reference interval.  Invalid gait sequences and keys not
 * present in the frozen CSV fail closed to an exactly zero baseline.
 */
class CompositePhaseBaselineCorrector final {
 public:
  CompositePhaseBaselineCorrector(const std::string& csvFile,
                                  std::size_t profilePhaseBins,
                                  std::size_t gaitPhaseBins,
                                  scalar_t profilePeriod,
                                  scalar_t profileStartTime,
                                  scalar_t rampInDuration,
                                  scalar_t steadyDuration,
                                  scalar_t gaitCycleDuration);

  CompositePhaseBaselineSample sample(scalar_t time, std::size_t mode);

  const std::string& csvFile() const { return csvFile_; }
  std::size_t occupiedKeys() const { return baseline_.size(); }
  static const std::array<std::size_t, 4>& expectedModes();

 private:
  using Key = std::tuple<std::size_t, std::size_t, std::size_t>;

  void loadCsv();
  void startCycle(scalar_t time);
  CompositePhaseBaselineSample baseSample(scalar_t time,
                                          std::size_t mode) const;
  void applyLookup(CompositePhaseBaselineSample& result) const;

  std::string csvFile_;
  std::size_t profilePhaseBins_ = 120;
  std::size_t gaitPhaseBins_ = 24;
  scalar_t profilePeriod_ = 12.0;
  scalar_t profileStartTime_ = 2.4;
  scalar_t rampInDuration_ = 2.4;
  scalar_t steadyDuration_ = 24.0;
  scalar_t gaitCycleDuration_ = 1.2;
  std::map<Key, parameter_vector_t> baseline_;

  bool hasPreviousMode_ = false;
  std::size_t previousMode_ = 0;
  int sequenceIndex_ = -1;
  bool sequenceValid_ = false;
  scalar_t cycleStart_ = -1.0;
  scalar_t lastTime_ = -1.0;
};

}  // namespace adaptive
}  // namespace legged_robot_adaptive
}  // namespace ocs2
