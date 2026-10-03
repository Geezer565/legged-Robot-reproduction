#include "ocs2_legged_robot_adaptive/adaptive/CompositePhaseBaselineCorrector.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <vector>

namespace ocs2 {
namespace legged_robot_adaptive {
namespace adaptive {
namespace {

std::vector<std::string> splitCsvLine(const std::string& line) {
  std::vector<std::string> fields;
  std::stringstream stream(line);
  std::string field;
  while (std::getline(stream, field, ',')) fields.push_back(field);
  return fields;
}

void stripTrailingCarriageReturn(std::string& line) {
  if (!line.empty() && line.back() == '\r') line.pop_back();
}

const std::array<const char*, 10> kParameterNames = {
    "m", "hx", "hy", "hz", "Jxx", "Jxy", "Jxz", "Jyy", "Jyz", "Jzz"};

}  // namespace

const std::array<std::size_t, 4>&
CompositePhaseBaselineCorrector::expectedModes() {
  static const std::array<std::size_t, 4> modes = {13, 7, 14, 11};
  return modes;
}

CompositePhaseBaselineCorrector::CompositePhaseBaselineCorrector(
    const std::string& csvFile, std::size_t profilePhaseBins,
    std::size_t gaitPhaseBins, scalar_t profilePeriod,
    scalar_t profileStartTime, scalar_t rampInDuration,
    scalar_t steadyDuration, scalar_t gaitCycleDuration)
    : csvFile_(csvFile),
      profilePhaseBins_(profilePhaseBins),
      gaitPhaseBins_(gaitPhaseBins),
      profilePeriod_(profilePeriod),
      profileStartTime_(profileStartTime),
      rampInDuration_(rampInDuration),
      steadyDuration_(steadyDuration),
      gaitCycleDuration_(gaitCycleDuration) {
  if (csvFile_.empty() || profilePhaseBins_ == 0 || gaitPhaseBins_ == 0 ||
      gaitPhaseBins_ % expectedModes().size() != 0 ||
      !std::isfinite(profilePeriod_) || profilePeriod_ <= 0.0 ||
      !std::isfinite(profileStartTime_) || profileStartTime_ < 0.0 ||
      !std::isfinite(rampInDuration_) || rampInDuration_ <= 0.0 ||
      !std::isfinite(steadyDuration_) || steadyDuration_ <= 0.0 ||
      !std::isfinite(gaitCycleDuration_) || gaitCycleDuration_ <= 0.0) {
    throw std::invalid_argument(
        "[CompositePhaseBaselineCorrector] invalid file/bin/timing configuration.");
  }
  loadCsv();
}

void CompositePhaseBaselineCorrector::loadCsv() {
  std::ifstream stream(csvFile_);
  if (!stream) {
    throw std::runtime_error(
        "[CompositePhaseBaselineCorrector] cannot open baseline CSV: " + csvFile_);
  }
  const std::string expectedHeader =
      "profile_phase_bin,profile_phase_start,profile_phase_end,gait_phase_bin,"
      "gait_phase_start,gait_phase_end,executed_mode,training_duration_s,"
      "training_sample_count,parameter,nominal_baseline_drive_rate";
  std::string line;
  if (!std::getline(stream, line)) {
    throw std::runtime_error(
        "[CompositePhaseBaselineCorrector] baseline CSV schema mismatch.");
  }
  stripTrailingCarriageReturn(line);
  if (line != expectedHeader) {
    throw std::runtime_error(
        "[CompositePhaseBaselineCorrector] baseline CSV schema mismatch.");
  }

  std::unordered_map<std::string, std::size_t> parameterIndex;
  for (std::size_t index = 0; index < kParameterNames.size(); ++index) {
    parameterIndex.emplace(kParameterNames[index], index);
  }
  std::map<Key, std::array<bool, 10>> seen;
  std::size_t lineNumber = 1;
  while (std::getline(stream, line)) {
    ++lineNumber;
    stripTrailingCarriageReturn(line);
    if (line.empty()) continue;
    const auto fields = splitCsvLine(line);
    if (fields.size() != 11) {
      throw std::runtime_error(
          "[CompositePhaseBaselineCorrector] malformed CSV row " +
          std::to_string(lineNumber));
    }
    const std::size_t profileBin = std::stoul(fields[0]);
    const scalar_t profileStart = std::stod(fields[1]);
    const scalar_t profileEnd = std::stod(fields[2]);
    const std::size_t gaitBin = std::stoul(fields[3]);
    const scalar_t gaitStart = std::stod(fields[4]);
    const scalar_t gaitEnd = std::stod(fields[5]);
    const std::size_t mode = std::stoul(fields[6]);
    const scalar_t trainingDuration = std::stod(fields[7]);
    const long trainingSamples = std::stol(fields[8]);
    const auto parameter = parameterIndex.find(fields[9]);
    const scalar_t value = std::stod(fields[10]);
    if (profileBin >= profilePhaseBins_ || gaitBin >= gaitPhaseBins_ ||
        std::find(expectedModes().begin(), expectedModes().end(), mode) ==
            expectedModes().end() ||
        parameter == parameterIndex.end() || !std::isfinite(trainingDuration) ||
        trainingDuration <= 0.0 || trainingSamples <= 0 ||
        !std::isfinite(value)) {
      throw std::runtime_error(
          "[CompositePhaseBaselineCorrector] invalid CSV cell metadata.");
    }
    const scalar_t expectedProfileStart =
        static_cast<scalar_t>(profileBin) / profilePhaseBins_;
    const scalar_t expectedProfileEnd =
        static_cast<scalar_t>(profileBin + 1) / profilePhaseBins_;
    const scalar_t expectedGaitStart =
        static_cast<scalar_t>(gaitBin) / gaitPhaseBins_;
    const scalar_t expectedGaitEnd =
        static_cast<scalar_t>(gaitBin + 1) / gaitPhaseBins_;
    if (std::abs(profileStart - expectedProfileStart) > 1.0e-12 ||
        std::abs(profileEnd - expectedProfileEnd) > 1.0e-12 ||
        std::abs(gaitStart - expectedGaitStart) > 1.0e-12 ||
        std::abs(gaitEnd - expectedGaitEnd) > 1.0e-12) {
      throw std::runtime_error(
          "[CompositePhaseBaselineCorrector] phase bounds mismatch.");
    }
    const Key key(profileBin, gaitBin, mode);
    if (baseline_.count(key) == 0) {
      baseline_[key] = parameter_vector_t::Constant(
          10, std::numeric_limits<scalar_t>::quiet_NaN());
      seen[key].fill(false);
    }
    if (seen[key][parameter->second]) {
      throw std::runtime_error(
          "[CompositePhaseBaselineCorrector] duplicate CSV parameter cell.");
    }
    baseline_[key](static_cast<Eigen::Index>(parameter->second)) = value;
    seen[key][parameter->second] = true;
  }
  if (baseline_.empty()) {
    throw std::runtime_error(
        "[CompositePhaseBaselineCorrector] empty baseline CSV.");
  }
  for (const auto& item : baseline_) {
    const auto& flags = seen.at(item.first);
    if (!std::all_of(flags.begin(), flags.end(), [](bool value) { return value; }) ||
        !item.second.allFinite()) {
      throw std::runtime_error(
          "[CompositePhaseBaselineCorrector] incomplete/nonfinite sparse key.");
    }
  }
}

void CompositePhaseBaselineCorrector::startCycle(scalar_t time) {
  cycleStart_ = time;
  sequenceIndex_ = 0;
  sequenceValid_ = true;
}

CompositePhaseBaselineSample CompositePhaseBaselineCorrector::baseSample(
    scalar_t time, std::size_t mode) const {
  CompositePhaseBaselineSample result;
  result.mode = mode;
  const scalar_t elapsed = time - profileStartTime_;
  scalar_t profilePhase = std::fmod(elapsed, profilePeriod_) / profilePeriod_;
  if (profilePhase < 0.0) profilePhase += 1.0;
  result.profilePhase = profilePhase;
  result.profilePhaseBin = static_cast<int>(std::min(
      static_cast<std::size_t>(std::floor(profilePhase * profilePhaseBins_)),
      profilePhaseBins_ - 1));
  const scalar_t steadyStart = profileStartTime_ + rampInDuration_;
  result.steadyStageActive =
      time >= steadyStart && time < steadyStart + steadyDuration_;
  return result;
}

void CompositePhaseBaselineCorrector::applyLookup(
    CompositePhaseBaselineSample& result) const {
  if (!result.gaitPhaseValid || result.gaitPhaseBin < 0 ||
      result.profilePhaseBin < 0) return;
  const Key key(static_cast<std::size_t>(result.profilePhaseBin),
                static_cast<std::size_t>(result.gaitPhaseBin), result.mode);
  const auto value = baseline_.find(key);
  result.keySupported = value != baseline_.end();
  result.correctionActive = result.steadyStageActive && result.keySupported;
  if (result.correctionActive) result.baseline = value->second;
}

CompositePhaseBaselineSample CompositePhaseBaselineCorrector::sample(
    scalar_t time, std::size_t mode) {
  if (!std::isfinite(time) ||
      (std::isfinite(lastTime_) && time <= lastTime_)) {
    throw std::invalid_argument(
        "[CompositePhaseBaselineCorrector] times must be finite and strictly increasing.");
  }
  const auto& modes = expectedModes();
  const bool modeChanged = hasPreviousMode_ && mode != previousMode_;
  const bool closePreviousCycle = modeChanged && sequenceValid_ &&
      previousMode_ == modes.back() && mode == modes.front();

  if (closePreviousCycle) {
    CompositePhaseBaselineSample result = baseSample(time, mode);
    result.gaitPhaseValid = true;
    result.cycleBoundary = true;
    result.gaitPhaseBin = static_cast<int>(gaitPhaseBins_ - 1);
    result.gaitPhase = std::nextafter(1.0, 0.0);
    result.cycleStart = cycleStart_;
    applyLookup(result);
    startCycle(time);
    previousMode_ = mode;
    hasPreviousMode_ = true;
    lastTime_ = time;
    return result;
  }

  if (!hasPreviousMode_) {
    if (mode == modes.front()) startCycle(time);
    else sequenceValid_ = false;
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

  CompositePhaseBaselineSample result = baseSample(time, mode);
  if (sequenceValid_ && cycleStart_ >= 0.0) {
    const scalar_t phase = (time - cycleStart_) / gaitCycleDuration_;
    if (phase >= 0.0 && phase < 1.0) {
      const std::size_t phaseBin = std::min(
          static_cast<std::size_t>(std::floor(phase * gaitPhaseBins_)),
          gaitPhaseBins_ - 1);
      const std::size_t binsPerMode = gaitPhaseBins_ / modes.size();
      const int phaseModeIndex = static_cast<int>(phaseBin / binsPerMode);
      const scalar_t modeBoundary = static_cast<scalar_t>(sequenceIndex_) /
                                    static_cast<scalar_t>(modes.size());
      const bool roundingAtTransition =
          std::abs(phase - modeBoundary) <= 1.0e-9;
      if ((phaseModeIndex == sequenceIndex_ || roundingAtTransition) &&
          sequenceIndex_ >= 0 &&
          mode == modes[static_cast<std::size_t>(sequenceIndex_)]) {
        result.gaitPhaseValid = true;
        result.gaitPhaseBin = static_cast<int>(phaseBin);
        result.gaitPhase = phase;
        result.cycleStart = cycleStart_;
        applyLookup(result);
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
