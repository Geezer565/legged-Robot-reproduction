#pragma once

#include <array>
#include <cstddef>
#include <string>
#include <vector>

#include "ocs2_legged_robot_adaptive/adaptive/AdaptiveTypes.h"

namespace ocs2 {
namespace legged_robot_adaptive {
namespace adaptive {

/** One causal lookup result for the frozen D5G nominal phase baseline. */
struct GaitPhaseBaselineSample {
  parameter_vector_t baseline = parameter_vector_t::Zero(10);
  bool phaseValid = false;
  bool correctionActive = false;
  bool cycleBoundary = false;
  int phaseBin = -1;
  std::size_t mode = 0;
  scalar_t normalizedPhase = -1.0;
  scalar_t cycleStart = -1.0;
};

/**
 * Causal static-walk phase tracker and immutable nominal update baseline.
 *
 * The convention intentionally matches the D5G/D5H0 analysis: mode sequence
 * 13->7->14->11, 1.2 s cycle, with the 11->13 transition sample closing the
 * previous cycle in the final phase bin.  Invalid sequences fail closed and
 * return a zero baseline until a new mode-13 cycle begins.
 */
class GaitPhaseBaselineCorrector final {
 public:
  GaitPhaseBaselineCorrector(const std::string& csvFile, std::size_t phaseBins,
                             scalar_t cycleDuration,
                             scalar_t minimumCycleStartTime);

  GaitPhaseBaselineSample sample(scalar_t time, std::size_t mode);

  const std::string& csvFile() const { return csvFile_; }
  std::size_t phaseBins() const { return phaseBins_; }
  scalar_t cycleDuration() const { return cycleDuration_; }
  scalar_t minimumCycleStartTime() const { return minimumCycleStartTime_; }
  static const std::array<std::size_t, 4>& expectedModes();

 private:
  void startCycle(scalar_t time);
  GaitPhaseBaselineSample invalidSample(scalar_t time, std::size_t mode) const;
  void loadCsv();

  std::string csvFile_;
  std::size_t phaseBins_ = 24;
  scalar_t cycleDuration_ = 1.2;
  scalar_t minimumCycleStartTime_ = 2.2;
  std::vector<parameter_vector_t> baseline_;

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
