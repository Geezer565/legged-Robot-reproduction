#pragma once

#include <array>
#include <vector>
#include "legged_interface/adaptive/MyRbfV2FixedFeatureMap.h"

namespace legged {
namespace adaptive {
namespace v2 {

enum class RobustMode { Sign, Smooth };

// Continuous-time V2 equations. This is deliberately NOT a V1 estimator adapter.
// The caller must supply chi = T(q)*(qdot-qdot_des+Lambda*(q-q_des)).
struct Settings {
  Axes residualScale{{1,1,1,1,1,1}};
  Axes outputScale{{1,1,1,1,1,1}};
  Axes extent{{2,2,2,4,4,4}};
  Axes gamma{{1,1,1,1,1,1}};  // fixed scalar diagonal Gamma per axis
  // Certified finite-window integral-history gain. In TeacherComposite this
  // is never applied to an instantaneous residual as a supervised label.
  Axes predictionGain{{0,0,0,0,0,0}};
  double compositeWindow = 0.2;
  Axes weightLimit{{2,2,2,2,2,2}};
  // V4 only.  Bias/deadzone are frozen from a no-disturbance calibration;
  // leakage is a continuous-time rate used by the normalized residual update.
  Axes residualBias{{0,0,0,0,0,0}};
  Axes residualDeadzone{{0,0,0,0,0,0}};
  Axes convergenceLeakage{{0,0,0,0,0,0}};
  Axes robustGamma{{1,1,1,1,1,1}};
  Axes nu0{{.12,.12,.12,.12,.12,.12}};
  Axes nuTime{{2,2,2,2,2,2}};
  Axes nuExponent{{.5,.5,.5,.5,.5,.5}}; // nu0/(1+t/T)^(1+delta)
  double widthRatio = .85;
  double epsilon = 1e-6;
  unsigned maxHistorySamples = 32;
  RobustMode robustMode = RobustMode::Smooth;
  FeatureMode featureMode = FeatureMode::ResidualOnly;
  UpdateLaw updateLaw = UpdateLaw::V2Tracking;
  double robotMass = 1.0;
  Axes chiScale{{1,1,1,1,1,1}};
  Axes baseVelocityScale{{1,1,1,1,1,1}};
  Axes referenceVelocityScale{{1,1,1,1,1,1}};
  double contactFeatureWidth = 1.0;
};

struct State {
  Weights weights{};
  Axes robustBound{}; // nonnegative; derivative, not an arbitrary leakage update
};

// Store completed, time-aligned physical gradients and labels together.
// certified=true means the caller established |label-g'W*|<=noiseBound.
struct Sample {
  Weights physicalGradient{};
  Axes label{};
  Axes noiseBound{};
  Axes importance{};
  bool certified = false;
};

struct Evaluation {
  // M*vDot+C*v+g = appliedWrench + externalWrenchEstimate.
  // chi and wrench must be power-conjugate at the same frame/reference point.
  Axes estimate{};
  Axes robustWrench{};
  Axes smoothingWidth{};
  Axes coverage{};
  Weights physicalGradient{};
  Weights weightDerivative{};
  Weights historyCorrection{};
  Axes boundDerivative{};
  Axes predictionDissipation{};
  unsigned projectedComponents = 0;
  Axes learningSignal{};
  bool learningEnabled = false;
  Axes externalWrenchEstimate{};       // equals legacy estimate
  Axes leftHandDisturbanceEstimate{};  // = -externalWrenchEstimate
  Axes requiredControlCorrection{};   // = -(externalWrenchEstimate + robustWrench)
};

// A final physical wrench w must obey normal'w <= rhs. Contact/torque
// feasibility must be enforced in the SAME optimizer; never clip afterward.
struct WrenchConstraint {
  Axes normal{};
  double rhs = 0;
  double violation(const Axes& achievedWrench) const;
};

class Core final {
 public:
  explicit Core(Settings settings = Settings{});
  Weights physicalGradient(const Axes& causalResidual) const;
  Weights physicalGradient(const FeatureInput& input) const;
  Evaluation evaluate(double elapsedTime, const State& state,
                      const Axes& causalResidual, const Axes& powerError,
                      const std::vector<Sample>& history = {}) const;
  Evaluation evaluate(double elapsedTime, const State& state,
                      const FeatureInput& input,
                      const std::vector<Sample>& history = {}) const;
  WrenchConstraint constraint(const Axes& powerError,
                             const Axes& referenceWrench,
                             const Axes& damping,
                             const Evaluation& evaluation) const;
  const Settings& settings() const { return settings_; }
  const FixedFeatureMap& fixedFeatureMap() const { return fixedFeatureMap_; }
 private:
  Evaluation evaluateGradient(double elapsedTime, const State& state,
                              const Weights& gradient, const Axes& powerError,
                              const std::vector<Sample>& history) const;
  Settings settings_;
  FixedFeatureMap fixedFeatureMap_;
};

} // namespace v2
} // namespace adaptive
} // namespace legged
