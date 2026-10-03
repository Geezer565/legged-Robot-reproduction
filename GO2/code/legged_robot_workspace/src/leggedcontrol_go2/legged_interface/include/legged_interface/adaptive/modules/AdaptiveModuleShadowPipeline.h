#pragma once

#include "legged_interface/adaptive/MyRbfV2FeatureInput.h"
#include "legged_interface/adaptive/modules/AdaptiveFeatureModule.h"
#include "legged_interface/adaptive/modules/PayloadIdModule.h"
#include "legged_interface/adaptive/modules/PeriodicResidualModule.h"
#include "legged_interface/adaptive/modules/ResidualConfidenceModule.h"
#include "legged_interface/adaptive/modules/ResidualDynamicsModule.h"

namespace legged {
namespace adaptive {
namespace modules {

enum class SlowWrenchMode {
  MyRbfOnly,
  PayloadOnly,
  ConfidenceBlend,
};

struct AdaptiveModuleShadowSettings {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  bool enablePeriodic = true;
  bool enablePayload = true;
  bool enableResidualDynamics = true;
  bool enableFeatureAdvisor = true;
  SlowWrenchMode slowWrenchMode = SlowWrenchMode::ConfidenceBlend;
  scalar_t payloadBlendStartConfidence = 0.25;
  scalar_t payloadBlendFullConfidence = 0.75;

  PeriodicResidualModule::Settings periodic;
  PayloadIdSettings payload;
  ResidualDynamicsSettings residualDynamics;
  AdaptiveFeatureSettings feature;
  ResidualConfidenceSettings confidence;
};

/**
 * One allocation-free sample from the current Go2 controller.
 *
 * All six-axis vectors follow [Fx,Fy,Fz,Tx,Ty,Tz] in the world frame, with
 * torque about the base origin. payloadInput supplies gravity and optional
 * kinematics/custom regressor; its residual is replaced by physicalResidual
 * so all modules consume one authoritative sample.
 */
struct AdaptiveModuleShadowInput {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  scalar_t time = 0.0;
  scalar_t dt = 0.0;
  vector6_t physicalResidual = vector6_t::Zero();
  vector6_t myRbfRawEstimate = vector6_t::Zero();
  vector6_t trackingSignal = vector6_t::Zero();
  vector6_t coverage = vector6_t::Zero();
  bool contactChanged = false;
  bool dataValid = false;
  MyRbfV2FeatureInput featureInput;
  PayloadIdInput payloadInput;
};

/**
 * Diagnostic-only output. gatedWrench is a candidate for later integration;
 * this class has no actuator or WBC command interface.
 */
struct AdaptiveModuleShadowOutput {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  bool inputValid = false;
  bool shadowOnly = true;
  scalar_t payloadBlend = 0.0;

  PeriodicResidualResult periodicResult;
  PayloadIdOutput payloadResult;
  ResidualDynamicsResult dynamicsResult;
  AdaptiveFeatureSuggestion featureSuggestion;
  ResidualConfidenceOutput confidenceResult;

  vector6_t myRbfComponent = vector6_t::Zero();
  vector6_t payloadComponent = vector6_t::Zero();
  vector6_t slowWrench = vector6_t::Zero();
  vector6_t residualAfterSlow = vector6_t::Zero();
  vector6_t periodicComponent = vector6_t::Zero();
  vector6_t residualAfterPeriodic = vector6_t::Zero();
  vector6_t dynamicsComponent = vector6_t::Zero();
  vector6_t candidateWrench = vector6_t::Zero();
  vector6_t gatedWrench = vector6_t::Zero();
};

/**
 * Runs M01--M05 behind the current controller without producing commands.
 * M04 is observed and reported but is never added to the wrench.
 */
class AdaptiveModuleShadowPipeline final {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit AdaptiveModuleShadowPipeline(
      AdaptiveModuleShadowSettings settings = AdaptiveModuleShadowSettings{});

  AdaptiveModuleShadowOutput step(
      const AdaptiveModuleShadowInput& input) noexcept;
  void reset(scalar_t time = 0.0);

 private:
  bool coreInputIsValid(const AdaptiveModuleShadowInput& input) const noexcept;
  scalar_t payloadBlend(const PayloadIdOutput& payload) const noexcept;
  void validateSettings() const;

  AdaptiveModuleShadowSettings settings_;
  PeriodicResidualModule periodic_;
  PayloadIdModule payload_;
  ResidualDynamicsModule residualDynamics_;
  AdaptiveFeatureModule featureAdvisor_;
  ResidualConfidenceModule confidence_;
  scalar_t lastTime_ = 0.0;
  bool hasStep_ = false;
};

}  // namespace modules
}  // namespace adaptive
}  // namespace legged

