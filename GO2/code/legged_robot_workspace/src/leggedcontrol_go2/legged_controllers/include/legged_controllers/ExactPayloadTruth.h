#pragma once

#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <stdexcept>
#include <string>

#include <ocs2_legged_robot_adaptive/adaptive/AdaptiveReferenceKinematics.h>
#include <ocs2_legged_robot_adaptive/adaptive/PayloadInertial10D.h>

namespace legged {
namespace exact_payload_truth {

namespace eth = ocs2::legged_robot_adaptive::adaptive;

/**
 * 本实验真实负载：
 *
 * Gazebo:
 *   go2::base mass 6.921 -> 11.921 kg
 *
 * 即：
 *   Delta m = +5 kg
 *
 * 新增质量位于原 base-link inertial COM。
 *
 * 已根据：
 *
 *   nominal whole-robot CoM -> base origin
 *   [ 0.000229391904,
 *    -0.003089326974,
 *     0.027360488805 ]
 *
 * 和：
 *
 *   base origin -> base-link inertial COM
 *   [ 0.021112,
 *     0,
 *    -0.005366 ]
 *
 * 得到：
 *
 *   r =
 *   [ 0.021341391904,
 *    -0.003089326974,
 *     0.021994488805 ] m
 *
 * 并通过：
 *
 *   h = Delta_m * r
 *
 *   J = Delta_m * ( ||r||^2 I - r r^T )
 *
 * 得到下面严格对应当前 Gazebo +5kg 实验的 10D 参数增量。
 *
 * 顺序：
 *
 * [m,
 *  hx,hy,hz,
 *  Jxx,Jxy,Jxz,Jyy,Jyz,Jzz]
 */
inline eth::vector10_t truePayloadPi10() {
  eth::vector10_t pi;

  pi <<
      5.000000000000,

      0.106706959520,
     -0.015446634869,
      0.109972444025,

      0.002466507395,
      0.000329652688,
     -0.002346965027,

      0.004696062731,
      0.000339740838,

      0.002324994748;

  return pi;
}


/**
 * 计算真正的：
 *
 *       d_u^true = Y_in(q,v,v_r,dot(v_r)) * pi_in^true
 *
 * 注意：
 *
 * 1. 这里不是外力；
 * 2. 输出为 WORLD frame；
 * 3. torque reference point 为 nominal whole-robot CoM；
 * 4. 完全调用 ETH 自己的 PayloadInertial10D::referenceWrench()；
 * 5. 因此没有 Python 数值微分，也没有重新手写另一套 regressor。
 */
inline eth::vector6_t trueYpiWorldCom(
    const eth::AdaptiveReferenceKinematics& reference) {

  // ------------------------------------------------------------------
  // Exact payload truth must follow the ACTUAL experiment condition.
  //
  // The parameter vector already stored in this file corresponds to
  // the verified +5 kg mass-only perturbation:
  //
  //     go2::base 6.921 kg -> 11.921 kg
  //
  // PAYLOAD_DELTA_MASS is the same environment variable used when
  // constructing the Gazebo plant before spawn.
  //
  // Since the inertial-parameter increment and Y*pi are linear in
  // delta mass for this mass-only experiment:
  //
  //     Y*pi_true(dm)
  //       = (dm / 5 kg) * Y*pi_true(+5 kg)
  //
  // Therefore:
  //   dm = 0 kg -> exact truth is identically zero
  //   dm = 5 kg -> preserves the existing exact +5 kg truth
  //
  // Diagnostic ONLY.  This value must never enter the controller.
  // ------------------------------------------------------------------
  static const double payloadDeltaMassKg = []() {
    const char* raw = std::getenv("PAYLOAD_DELTA_MASS");

    // No payload variable means nominal plant.
    if (raw == nullptr || *raw == '\0') {
      return 0.0;
    }

    errno = 0;
    char* end = nullptr;

    const double value = std::strtod(raw, &end);

    if (errno == ERANGE ||
        end == raw ||
        end == nullptr ||
        *end != '\0' ||
        !std::isfinite(value)) {
      throw std::runtime_error(
          std::string(
              "[ExactPayloadTruth] invalid PAYLOAD_DELTA_MASS: ") +
          raw);
    }

    return value;
  }();

  // The hard-coded exact inertial vector below is the +5 kg reference.
  static const double payloadTruthScale =
      payloadDeltaMassKg / 5.0;

  eth::PayloadInertial10D::ReferencePoint point;

  point.rotationWorldFromBase =
      reference.actual.orientationWorldFromBase.toRotationMatrix();

  point.actualAngularVelocityWorld =
      reference.actual.angularVelocityWorld;

  point.referenceLinearAccelerationNominalComWorld =
      reference.referenceLinearAccelerationWorld;

  point.referenceAngularVelocityWorld =
      reference.referenceAngularVelocityWorld;

  point.referenceAngularAccelerationWorld =
      reference.referenceAngularAccelerationWorld;

  point.gravityWorld =
      eth::vector3_t(0.0, 0.0, -9.81);

  return payloadTruthScale *
      eth::PayloadInertial10D::referenceWrench(
      point,
      truePayloadPi10());
}

}  // namespace exact_payload_truth
}  // namespace legged
