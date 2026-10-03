#pragma once

#include <ocs2_centroidal_model/CentroidalModelInfo.h>

#include "ocs2_legged_robot_adaptive/adaptive/AdaptiveTypes.h"

namespace ocs2 {
namespace legged_robot_adaptive {
namespace adaptive {

using vector10_t = Eigen::Matrix<scalar_t, 10, 1>;
using matrix6x10_t = Eigen::Matrix<scalar_t, 6, 10>;

/**
 * Linear spatial-inertia parameterization of a rigid payload attached to the
 * floating base.
 *
 * Parameter origin C0 is the nominal robot CoM and the axes are the base axes:
 *   pi = [m, hx, hy, hz, Jxx, Jxy, Jxz, Jyy, Jyz, Jzz].
 * h = m * c is the first moment from C0 to the payload CoM. J is the payload
 * inertia about C0. This choice keeps the parameterization linear and makes
 * Y*pi a world-frame wrench about the same nominal CoM used by the existing
 * Adaptive CLF contact-wrench diagnostics.
 */
class PayloadInertial10D final {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  struct Parameters {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    scalar_t mass = 0.0;
    vector3_t firstMomentNominalComBase = vector3_t::Zero();
    matrix3_t inertiaAboutNominalComBase = matrix3_t::Zero();
  };

  /** Modified-reference quantities used by Eq. (8) and Eq. (11). */
  struct ReferencePoint {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    matrix3_t rotationWorldFromBase = matrix3_t::Identity();
    vector3_t actualAngularVelocityWorld = vector3_t::Zero();
    vector3_t referenceLinearAccelerationNominalComWorld =
        vector3_t::Zero();
    vector3_t referenceAngularVelocityWorld = vector3_t::Zero();
    vector3_t referenceAngularAccelerationWorld = vector3_t::Zero();
    vector3_t gravityWorld =
        (vector3_t() << 0.0, 0.0, -9.81).finished();
  };

  static Parameters fromVector(const vector10_t& parameterVector);
  static vector10_t toVector(const Parameters& parameters);

  /** Build a physical payload from CoM data, all expressed in base axes. */
  static Parameters fromPhysicalPayload(
      scalar_t mass, const vector3_t& positionNominalComToPayloadComBase,
      const matrix3_t& inertiaPayloadComBase);

  /** Recover payload inertia about its own CoM. */
  static matrix3_t inertiaAboutPayloadCom(const Parameters& parameters);

  /**
   * Checks physical consistency of a ground-truth payload. Zero payload and
   * point-mass payloads are accepted; an online estimate is not required to
   * satisfy this predicate.
   */
  static bool isPhysicallyConsistent(const Parameters& parameters,
                                     scalar_t tolerance = 1.0e-10);
  static void validatePhysicalPayload(const Parameters& parameters,
                                      scalar_t tolerance = 1.0e-10);

  /** 6x10 Slotine-Li modified-reference regressor, world wrench about C0. */
  static matrix6x10_t referenceRegressor(const ReferencePoint& point);
  static vector6_t referenceWrench(const ReferencePoint& point,
                                   const vector10_t& parameterVector);

  /**
   * Compose nominal robot SRBD parameters and a physical payload into a
   * plant-only CentroidalModelInfo. The nominal object is never modified.
   */
  static CentroidalModelInfo makePlantModelInfo(
      const CentroidalModelInfo& nominalInfo,
      const Parameters& payloadParameters);

  /**
   * Compose a frozen online estimate with the nominal SRBD model. Unlike the
   * ground-truth path, the estimated increment need not be a physically valid
   * standalone payload; only the resulting total mass and total CoM inertia
   * must be physical. No clipping or projection is performed.
   */
  static CentroidalModelInfo makeEstimatedModelInfo(
      const CentroidalModelInfo& nominalInfo,
      const vector10_t& estimatedParameterVector,
      scalar_t minimumEstimatedTotalMass = 1.0);

 private:
  static matrix3_t parallelAxisMatrix(const vector3_t& displacement);
  static void validateReferencePoint(const ReferencePoint& point);
};

}  // namespace adaptive
}  // namespace legged_robot_adaptive
}  // namespace ocs2
