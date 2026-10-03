#pragma once

#include <Eigen/Core>

#include "legged_interface/adaptive/AdaptiveTypes.h"

namespace legged {
namespace adaptive {
namespace modules {

using payload_vector4_t = Eigen::Matrix<scalar_t, 4, 1>;
using payload_matrix6x4_t = Eigen::Matrix<scalar_t, 6, 4>;

/**
 * Settings for the first-stage payload identifier.
 *
 * The estimated linear parameters are
 *   [deltaMass, deltaMass * deltaCom.x(), deltaMass * deltaCom.y(),
 *    deltaMass * deltaCom.z()].
 * This keeps the wrench model linear while exposing deltaCom in the result.
 */
struct PayloadIdSettings {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  payload_vector4_t adaptationGain = payload_vector4_t::Constant(2.0);
  scalar_t forgettingRate = 0.002;
  scalar_t informationForgettingRate = 0.02;
  scalar_t gravityMagnitude = 9.81;
  scalar_t minDeltaMass = 0.0;
  scalar_t maxDeltaMass = 8.0;
  vector3_t maxAbsDeltaCom = (vector3_t() << 0.15, 0.15, 0.20).finished();
  scalar_t normalizationEpsilon = 1e-6;
  payload_vector4_t confidenceInformationScale =
      payload_vector4_t::Constant(20.0);
  scalar_t maximumAngularSpeedForAutomaticRegressor = 1.0;
  scalar_t massEpsilon = 1e-6;
};

/**
 * All vectors use the same world frame as the current Go2 six-axis residual.
 * externalResidual and the output wrench use [Fx,Fy,Fz,Tx,Ty,Tz], with torque
 * about the base origin.
 *
 * The automatic regressor uses gravity minus base linear acceleration.  It is
 * intended for standing and moderate walking.  A caller with a complete
 * rigid-body regressor can set useCustomRegressor and supply that matrix.
 */
struct PayloadIdInput {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  vector6_t externalResidual = vector6_t::Zero();
  vector3_t gravityDirection = (vector3_t() << 0.0, 0.0, -1.0).finished();
  bool residualValid = true;
  bool updateAllowed = true;

  bool hasLinearAcceleration = false;
  vector3_t linearAcceleration = vector3_t::Zero();
  bool hasAngularVelocity = false;
  vector3_t angularVelocity = vector3_t::Zero();

  bool useCustomRegressor = false;
  payload_matrix6x4_t customRegressor = payload_matrix6x4_t::Zero();
};

struct PayloadIdOutput {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  bool inputValid = false;
  bool updated = false;
  bool usedCustomRegressor = false;
  scalar_t deltaMass = 0.0;
  vector3_t deltaCom = vector3_t::Zero();
  vector3_t firstMoment = vector3_t::Zero();
  vector6_t equivalentWrench = vector6_t::Zero();
  vector6_t innovation = vector6_t::Zero();
  payload_vector4_t parameterConfidence = payload_vector4_t::Zero();
  scalar_t confidence = 0.0;
  payload_matrix6x4_t regressor = payload_matrix6x4_t::Zero();
};

/**
 * A fixed-size projected normalized-gradient payload identifier.
 *
 * step() performs no dynamic allocation, file access, or locking.  The first
 * version only emits an equivalent wrench; publishing deltaMass/deltaCom into
 * the MPC model must happen later at a solver synchronization boundary.
 */
class PayloadIdModule final {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit PayloadIdModule(PayloadIdSettings settings = PayloadIdSettings());

  PayloadIdOutput step(const PayloadIdInput& input, scalar_t dt);
  void reset();

  const payload_vector4_t& parameters() const { return parameters_; }
  const payload_vector4_t& information() const { return information_; }

 private:
  bool inputIsValid(const PayloadIdInput& input, scalar_t dt) const;
  payload_matrix6x4_t makeAutomaticRegressor(
      const PayloadIdInput& input) const;
  void projectParameters();
  PayloadIdOutput makeOutput(const payload_matrix6x4_t& regressor,
                             const vector6_t& innovation,
                             bool inputValid, bool updated,
                             bool usedCustomRegressor) const;
  void validateSettings() const;

  PayloadIdSettings settings_;
  payload_vector4_t parameters_ = payload_vector4_t::Zero();
  payload_vector4_t information_ = payload_vector4_t::Zero();
};

}  // namespace modules
}  // namespace adaptive
}  // namespace legged

