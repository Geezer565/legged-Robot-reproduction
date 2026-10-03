#pragma once

#include <array>
#include <cstddef>

namespace legged {
namespace adaptive {
namespace v5 {

constexpr std::size_t kAxes = 6;
constexpr std::size_t kMotionInputs = 12;
constexpr std::size_t kHistoryInputs = 12;
constexpr std::size_t kContactInputs = 4;
constexpr std::size_t kMotionNodes = 25;
constexpr std::size_t kHistoryNodes = 25;
constexpr std::size_t kContactNodes = 16;
constexpr std::size_t kBasisSize =
    kMotionNodes + kHistoryNodes + kContactNodes + 1;

using Axes = std::array<double, kAxes>;
using Input = std::array<double, kMotionInputs>;
using Contacts = std::array<double, kContactInputs>;
using Basis = std::array<double, kBasisSize>;
using Weights = std::array<Basis, kAxes>;

struct Features {
  Axes velocity{};
  Axes nominalRate{};
  Axes residualMemory{};
  Axes residualTrend{};
  Contacts contacts{};
};

struct Settings {
  Input inputScale{{1.0, 1.0, 1.0, 3.0, 3.0, 3.0,
                    5.0, 5.0, 5.0, 10.0, 10.0, 10.0}};
  Input historyScale{{2.0, 2.0, 2.0, 4.0, 4.0, 4.0,
                      10.0, 10.0, 10.0, 20.0, 20.0, 20.0}};
  Axes gamma{{4.0, 4.0, 4.0, 2.0, 2.0, 2.0}};
  Axes leakage{{0.01, 0.01, 0.01, 0.01, 0.01, 0.01}};
  Axes weightLimit{{40.0, 40.0, 40.0, 30.0, 30.0, 30.0}};
  Axes outputLimit{{20.0, 20.0, 20.0, 30.0, 30.0, 30.0}};
  double centerExtent = 1.0;
  double motionWidth = 1.0;
  double historyWidth = 0.5;
  double contactWidth = 0.55;
  double inputClip = 2.0;
  double epsilon = 1.0e-6;
  // All-one limits preserve the frozen V5 update. V6 enables selected axes.
  Axes gainMultiplierMax{{1.0, 1.0, 1.0, 1.0, 1.0, 1.0}};
  Axes uncertaintyScale{{1.0, 1.0, 1.0, 1.0, 1.0, 1.0}};
  double gainAttackTau = 0.05;
  double gainReleaseTau = 0.50;
};

struct Evaluation {
  Basis basis{};
  Axes correction{};
  Axes totalRate{};
};

struct UpdateResult {
  Axes targetResidual{};
  Axes predictionError{};
  Evaluation before;
  Evaluation after;
  Axes gainEvidence{};
  Axes gainSmoothedEvidence{};
  Axes gainMultiplier{{1.0, 1.0, 1.0, 1.0, 1.0, 1.0}};
  Axes effectiveGamma{};
};

class DynamicsCore final {
 public:
  explicit DynamicsCore(Settings settings = Settings{});

  Evaluation evaluate(const Features& features, bool enabled = true) const;
  UpdateResult update(const Features& features, const Axes& targetResidual,
                      double dt, bool enabled = true);
  Axes predictNextVelocity(const Features& features, double dt,
                           bool enabled = true) const;
  void reset();

  const Weights& weights() const { return weights_; }
  const Settings& settings() const { return settings_; }

 private:
  Input normalizedMotion(const Features& features) const;
  Input normalizedHistory(const Features& features) const;
  Basis basis(const Features& features) const;
  void validateAxes(const Axes& values, const char* name) const;

  Settings settings_;
  std::array<Input, kMotionNodes> motionCenters_{};
  std::array<Input, kHistoryNodes> historyCenters_{};
  std::array<Contacts, kContactNodes> contactCenters_{};
  Weights weights_{};
  Axes gainEvidenceMemory_{};
};

}  // namespace v5
}  // namespace adaptive
}  // namespace legged
