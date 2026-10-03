#pragma once

#include "ocs2_legged_robot_adaptive/adaptive/AdaptiveTypes.h"

namespace ocs2 { namespace legged_robot_adaptive { namespace adaptive {

// Q_WB（base 到 world）下的 desired - actual 四元数误差，结果在 world 系。
vector3_t computeQuaternionTrackingError(const quaternion_t& actualWorldFromBase,
                                         const quaternion_t& desiredWorldFromBase);

/**
 * Q_WB 下 quaternion-vector tracking error 的 world-frame 时间导数。
 *
 * e_o = vec(Q_d * Q^{-1})，且 Qdot = 0.5 * [0, omega_W] * Q。
 * 因而 e_dot = 0.5 * (q_e.w (omega_d-omega)
 *                      + omega_d x e_o - e_o x omega)。
 * 该量用于 omega_r = omega_d + Lambda_o e_o 的 dot(omega_r)，不能把
 * e_dot 简化误写成 omega_d-omega。
 */
vector3_t computeQuaternionTrackingErrorDerivative(const quaternion_t& actualWorldFromBase,
                                                   const quaternion_t& desiredWorldFromBase,
                                                   const vector3_t& actualAngularVelocityWorld,
                                                   const vector3_t& desiredAngularVelocityWorld);
} } }
