#include "ocs2_legged_robot_adaptive/adaptive/GaitPhaseBaselineCorrector.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

namespace ocs2 {
namespace legged_robot_adaptive {
namespace adaptive {
namespace {

std::vector<std::string> splitCsvLine(const std::string& line) {
  std::vector<std::string> fields;
  std::stringstream stream(line);
  std::string field;
  while (std::getline(stream, field, ',')) {
    fields.push_back(field);
  }
  return fields;
}

const std::array<const char*, 10> kParameterNames = {
    "m", "hx", "hy", "hz", "Jxx", "Jxy", "Jxz", "Jyy", "Jyz", "Jzz"};

}  // namespace

const std::array<std::size_t, 4>& GaitPhaseBaselineCorrector::expectedModes() {
  static const std::array<std::size_t, 4> modes = {13, 7, 14, 11};
  return modes;
}

GaitPhaseBaselineCorrector::GaitPhaseBaselineCorrector(
    const std::string& csvFile, std::size_t phaseBins,
    scalar_t cycleDuration, scalar_t minimumCycleStartTime)
    : csvFile_(csvFile),
      phaseBins_(phaseBins),
      cycleDuration_(cycleDuration),
      minimumCycleStartTime_(minimumCycleStartTime) {
  if (csvFile_.empty() || phaseBins_ == 0 ||
      phaseBins_ % expectedModes().size() != 0 ||
      !std::isfinite(cycleDuration_) || cycleDuration_ <= 0.0 ||
      !std::isfinite(minimumCycleStartTime_) || minimumCycleStartTime_ < 0.0) {
    throw std::invalid_argument(
        "[GaitPhaseBaselineCorrector] invalid file/phase/timing configuration.");
  }
  loadCsv();
}

void GaitPhaseBaselineCorrector::loadCsv() {
  std::ifstream stream(csvFile_);
  if (!stream) {
    throw std::runtime_error(
        "[GaitPhaseBaselineCorrector] cannot open baseline CSV: " + csvFile_);
  }
  std::string line;
  if (!std::getline(stream, line) ||
      line != "phase_bin,phase_start,phase_end,training_duration_s,parameter,nominal_baseline_drive_rate") {
    throw std::runtime_error(
        "[GaitPhaseBaselineCorrector] baseline CSV schema mismatch.");
  }

  baseline_.assign(phaseBins_, parameter_vector_t::Constant(10,
      std::numeric_limits<scalar_t>::quiet_NaN()));
  std::vector<std::array<bool, 10>> seen(phaseBins_);
  std::unordered_map<std::string, std::size_t> parameterIndex;
  for (std::size_t index = 0; index < kParameterNames.size(); ++index) {
    parameterIndex.emplace(kParameterNames[index], index);
  }

  std::size_t lineNumber = 1;
  while (std::getline(stream, line)) {
    ++lineNumber;
    if (line.empty()) {
      continue;
    }
    const auto fields = splitCsvLine(line);
    if (fields.size() != 6) {
      throw std::runtime_error(
          "[GaitPhaseBaselineCorrector] malformed CSV row " +
          std::to_string(lineNumber));
    }
    const std::size_t phase = static_cast<std::size_t>(std::stoul(fields[0]));
    const scalar_t phaseStart = std::stod(fields[1]);
    const scalar_t phaseEnd = std::stod(fields[2]);
    const scalar_t trainingDuration = std::stod(fields[3]);
    const auto parameter = parameterIndex.find(fields[4]);
    const scalar_t value = std::stod(fields[5]);
    if (phase >= phaseBins_ || parameter == parameterIndex.end() ||
        seen[phase][parameter->second] || !std::isfinite(value) ||
        !std::isfinite(trainingDuration) || trainingDuration <= 0.0) {
      throw std::runtime_error(
          "[GaitPhaseBaselineCorrector] invalid/duplicate CSV cell.");
    }
    const scalar_t expectedStart = static_cast<scalar_t>(phase) /
                                   static_cast<scalar_t>(phaseBins_);
    const scalar_t expectedEnd = static_cast<scalar_t>(phase + 1) /
                                 static_cast<scalar_t>(phaseBins_);
    if (std::abs(phaseStart - expectedStart) > 1.0e-12 ||
        std::abs(phaseEnd - expectedEnd) > 1.0e-12) {
      throw std::runtime_error(
          "[GaitPhaseBaselineCorrector] phase bounds are not a complete partition.");
    }
    baseline_[phase](static_cast<Eigen::Index>(parameter->second)) = value;
    seen[phase][parameter->second] = true;
  }
  for (std::size_t phase = 0; phase < phaseBins_; ++phase) {
    if (!std::all_of(seen[phase].begin(), seen[phase].end(),
                     [](bool value) { return value; }) ||
        !baseline_[phase].allFinite()) {
      throw std::runtime_error(
          "[GaitPhaseBaselineCorrector] baseline CSV is incomplete/nonfinite.");
    }
  }
}

void GaitPhaseBaselineCorrector::startCycle(scalar_t time) {
  cycleStart_ = time;
  sequenceIndex_ = 0;
  sequenceValid_ = true;
}

GaitPhaseBaselineSample GaitPhaseBaselineCorrector::invalidSample(
    scalar_t, std::size_t mode) const {
  GaitPhaseBaselineSample result;
  result.mode = mode;
  return result;
}

GaitPhaseBaselineSample GaitPhaseBaselineCorrector::sample(
    scalar_t time, std::size_t mode) {
  if (!std::isfinite(time) ||
      (std::isfinite(lastTime_) && time <= lastTime_)) {
    throw std::invalid_argument(
        "[GaitPhaseBaselineCorrector] sample times must be finite and strictly increasing.");
  }
  const auto& modes = expectedModes();
  const bool modeChanged = hasPreviousMode_ && mode != previousMode_;
  const bool closePreviousCycle = modeChanged && sequenceValid_ &&
      previousMode_ == modes.back() && mode == modes.front();

  if (closePreviousCycle) {
    const scalar_t oldStart = cycleStart_;
    GaitPhaseBaselineSample result;
    result.mode = mode;
    result.phaseValid = true;
    result.correctionActive = oldStart >= minimumCycleStartTime_;
    result.cycleBoundary = true;
    result.phaseBin = static_cast<int>(phaseBins_ - 1);
    result.normalizedPhase = std::nextafter(1.0, 0.0);
    result.cycleStart = oldStart;
    if (result.correctionActive) {
      result.baseline = baseline_.back();
    }
    startCycle(time);
    previousMode_ = mode;
    hasPreviousMode_ = true;
    lastTime_ = time;
    return result;
  }

  if (!hasPreviousMode_) {
    if (mode == modes.front()) {
      startCycle(time);
    } else {
      sequenceValid_ = false;
    }
  } else if (modeChanged) {
    const int expectedNext = sequenceValid_ ? sequenceIndex_ + 1 : -1;
    if (sequenceValid_ && expectedNext < static_cast<int>(modes.size()) &&
        mode == modes[static_cast<std::size_t>(expectedNext)]) {
      sequenceIndex_ = expectedNext;
    } else if (mode == modes.front()) {
      startCycle(time);
    } else {
      sequenceValid_ = false;
      sequenceIndex_ = -1;
      cycleStart_ = -1.0;
    }
  }

  GaitPhaseBaselineSample result = invalidSample(time, mode);
  if (sequenceValid_ && cycleStart_ >= 0.0) {
    const scalar_t phase = (time - cycleStart_) / cycleDuration_;
    if (phase >= 0.0 && phase < 1.0) {
      const std::size_t phaseBin = std::min(
          static_cast<std::size_t>(std::floor(phase * phaseBins_)),
          phaseBins_ - 1);
      const std::size_t binsPerMode = phaseBins_ / modes.size();
      const int phaseModeIndex = static_cast<int>(phaseBin / binsPerMode);
      const scalar_t modeBoundary = static_cast<scalar_t>(sequenceIndex_) /
                                    static_cast<scalar_t>(modes.size());
      const bool roundingAtTransition =
          std::abs(phase - modeBoundary) <= 1.0e-9;
      if ((phaseModeIndex == sequenceIndex_ || roundingAtTransition) &&
          sequenceIndex_ >= 0 &&
          mode == modes[static_cast<std::size_t>(sequenceIndex_)]) {
        result.phaseValid = true;
        result.correctionActive = cycleStart_ >= minimumCycleStartTime_;
        result.phaseBin = static_cast<int>(phaseBin);
        result.normalizedPhase = phase;
        result.cycleStart = cycleStart_;
        if (result.correctionActive) {
          result.baseline = baseline_[phaseBin];
        }
      }
    }
  }

  previousMode_ = mode;
  hasPreviousMode_ = true;
  lastTime_ = time;
  return result;
}

}  // namespace adaptive
}  // namespace legged_robot_adaptive
}  // namespace ocs2
