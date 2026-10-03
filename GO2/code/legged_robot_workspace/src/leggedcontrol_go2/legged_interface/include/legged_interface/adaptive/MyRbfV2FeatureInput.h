#pragma once

#include <array>

namespace legged { namespace adaptive { namespace v2 {

using Axes = std::array<double, 6>;
using Basis = std::array<double, 26>;
using Weights = std::array<Basis, 6>;

enum class FeatureMode { ResidualOnly, ContactAware };
enum class UpdateLaw { V2Tracking, TeacherComposite, ResidualConvergence };

// Wrenches use [Fx,Fy,Fz,Tx,Ty,Tz]; velocities/chi use [vx,vy,vz,wx,wy,wz].
// All quantities must use a common frame AND reference point. chi is supplied
// by the physical error calculation, never reconstructed from these features.
struct FeatureInput {
  Axes causalResidual{};
  Axes chi{};
  // V3 only: the controller supplies the audited general-law signal
  // B_w^T E_c, where E_c combines previous-policy prediction and tracking
  // errors. chi remains the WBC power error.
  Axes learningSignal{};
  bool learningSignalValid = false;
  Axes baseVelocity{};
  Axes referenceVelocity{};
  std::array<bool, 4> contacts{{false, false, false, false}};
};

}}}  // namespace legged::adaptive::v2

namespace legged { namespace adaptive {
using MyRbfV2FeatureInput = v2::FeatureInput;
}}
