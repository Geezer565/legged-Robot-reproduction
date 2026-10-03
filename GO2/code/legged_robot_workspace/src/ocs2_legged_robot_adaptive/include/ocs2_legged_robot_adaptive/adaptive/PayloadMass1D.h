#pragma once

#include "ocs2_legged_robot_adaptive/adaptive/AdaptiveTypes.h"

namespace ocs2 {
namespace legged_robot_adaptive {
namespace adaptive {

/** Pure PayloadMass1D conversions shared by Eq.(8), Eq.(10), Eq.(11) and diagnostics. */
class PayloadMass1D final {
 public:
  /** Slotine-Li mass column used in Eq.(8)/(9): [a_r-g, 0,0,0]. */
  static vector6_t referenceRegressorColumn(
      const vector3_t& referenceLinearAccelerationWorld,
      const vector3_t& gravityWorld =
          (vector3_t() << 0.0, 0.0, -9.81).finished());

  static scalar_t estimatedTotalMass(scalar_t nominalMass,
                                     scalar_t payloadMassEstimate,
                                     scalar_t minimumEstimatedTotalMass);

  /** rho = delta_m_hat / (m_nominal + delta_m_hat). */
  static scalar_t estimatedMassRatio(scalar_t nominalMass,
                                     scalar_t payloadMassEstimate,
                                     scalar_t minimumEstimatedTotalMass);

  /** Eq.(10)-equivalent wrench on the current contact wrench: rho_hat*w_contact. */
  static vector6_t dynamicsEquivalentWrench(
      const vector6_t& contactWrenchWorld, scalar_t nominalMass,
      scalar_t payloadMassEstimate, scalar_t minimumEstimatedTotalMass);

  /** Exact frozen-mass correction for OCS2 normalized centroidal momentum rate. */
  static vector6_t normalizedMomentumRateCorrection(
      const vector6_t& nominalNormalizedMomentumRate, scalar_t nominalMass,
      scalar_t payloadMassEstimate, scalar_t minimumEstimatedTotalMass);
};

}  // namespace adaptive
}  // namespace legged_robot_adaptive
}  // namespace ocs2
