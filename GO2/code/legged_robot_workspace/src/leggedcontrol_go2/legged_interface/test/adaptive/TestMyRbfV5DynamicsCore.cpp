#include "legged_interface/adaptive/MyRbfV5DynamicsCore.h"

#include <cassert>
#include <cmath>
#include <iostream>

namespace {

bool near(double a, double b, double tolerance = 1.0e-9) {
  return std::abs(a - b) <= tolerance;
}

}  // namespace

int main() {
  using legged::adaptive::v5::Axes;
  using legged::adaptive::v5::DynamicsCore;
  using legged::adaptive::v5::Features;
  using legged::adaptive::v5::Settings;

  Settings settings;
  settings.gamma.fill(4.0);
  settings.leakage.fill(0.0);
  settings.outputLimit.fill(20.0);
  DynamicsCore core(settings);

  Features features;
  features.nominalRate = {{1.0, -2.0, 0.5, 0.2, -0.4, 0.1}};
  features.contacts = {{1.0, 1.0, 1.0, 1.0}};
  const auto disabled = core.evaluate(features, false);
  for (std::size_t axis = 0; axis < 6; ++axis) {
    assert(near(disabled.correction[axis], 0.0));
    assert(near(disabled.totalRate[axis], features.nominalRate[axis]));
  }

  Axes targetResidual{};
  targetResidual[1] = 3.0;
  for (int sample = 0; sample < 400; ++sample) {
    core.update(features, targetResidual, 0.01, true);
  }
  const auto learned = core.evaluate(features, true);
  assert(learned.correction[1] > 2.0);
  assert(std::abs(learned.correction[0]) < 1.0e-9);
  assert(near(learned.totalRate[1],
              features.nominalRate[1] + learned.correction[1]));

  const auto next = core.predictNextVelocity(features, 0.1, true);
  assert(near(next[1], 0.1 * learned.totalRate[1]));

  core.reset();
  const auto reset = core.evaluate(features, true);
  for (std::size_t axis = 0; axis < 6; ++axis) {
    assert(near(reset.correction[axis], 0.0));
    assert(near(reset.totalRate[axis], features.nominalRate[axis]));
  }

  // Residual history must distinguish two time-varying disturbances even when
  // the current robot motion and nominal model are identical.
  Features positive = features;
  Features negative = features;
  positive.residualMemory[0] = 1.0;
  negative.residualMemory[0] = -1.0;
  Axes positiveTarget{};
  Axes negativeTarget{};
  positiveTarget[0] = 2.0;
  negativeTarget[0] = -2.0;
  for (int sample = 0; sample < 1200; ++sample) {
    core.update(positive, positiveTarget, 0.01, true);
    core.update(negative, negativeTarget, 0.01, true);
  }
  assert(core.evaluate(positive, true).correction[0] > 0.5);
  assert(core.evaluate(negative, true).correction[0] < -0.5);

  // Contact mode is part of the real feature vector, not just a logged label.
  Features diagonal = features;
  diagonal.contacts = {{1.0, 0.0, 0.0, 1.0}};
  const auto fullStanceBasis = core.evaluate(features, true).basis;
  const auto diagonalBasis = core.evaluate(diagonal, true).basis;
  bool contactChangedFeature = false;
  for (std::size_t node = 0; node < fullStanceBasis.size(); ++node) {
    contactChangedFeature =
        contactChangedFeature ||
        !near(fullStanceBasis[node], diagonalBasis[node], 1.0e-6);
  }
  assert(contactChangedFeature);

  // V5 remains fixed-gain; V6 raises only the configured Fy learning gain.
  DynamicsCore fixed(Settings{});
  const auto fixedStep = fixed.update(features, targetResidual, 0.01);
  for (double multiplier : fixedStep.gainMultiplier) {
    assert(near(multiplier, 1.0));
  }
  Settings scheduledSettings;
  scheduledSettings.gainMultiplierMax[1] = 3.0;
  scheduledSettings.uncertaintyScale[1] = 0.8;
  DynamicsCore scheduled(scheduledSettings);
  const auto scheduledStep = scheduled.update(features, targetResidual, 0.01);
  assert(scheduledStep.gainMultiplier[1] > 1.0);
  assert(scheduledStep.gainMultiplier[1] <= 3.0);
  assert(near(scheduledStep.effectiveGamma[1],
              scheduledSettings.gamma[1] * scheduledStep.gainMultiplier[1]));
  for (std::size_t axis = 0; axis < 6; ++axis) {
    if (axis != 1) assert(near(scheduledStep.gainMultiplier[axis], 1.0));
  }
  scheduled.reset();
  const auto afterReset = scheduled.update(features, targetResidual, 0.01);
  assert(near(afterReset.gainMultiplier[1], scheduledStep.gainMultiplier[1]));

  std::cout << "MyRBF V5 residual-dynamics core tests passed\n";
  return 0;
}
