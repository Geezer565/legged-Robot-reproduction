#pragma once

#include <cstddef>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ocs2_core/Types.h>

namespace ocs2 {
namespace legged_robot_adaptive {
namespace adaptive {

using vector3_t = Eigen::Matrix<scalar_t, 3, 1>;
using matrix3_t = Eigen::Matrix<scalar_t, 3, 3>;
using vector6_t = Eigen::Matrix<scalar_t, 6, 1>;
using vector16_t = Eigen::Matrix<scalar_t, 16, 1>;
using matrix6_t = Eigen::Matrix<scalar_t, 6, 6>;
/** The physical uncertainty wrench and sigma remain six-dimensional. */
using parameter_vector_t = vector_t;
using parameter_matrix_t = matrix_t;
using regressor_t = Eigen::Matrix<scalar_t, 6, Eigen::Dynamic>;
using matrix6x16_t = Eigen::Matrix<scalar_t, 6, 16>;
using quaternion_t = Eigen::Quaternion<scalar_t>;

/**
 * Adaptive parameter semantics.
 *
 * ConstantWrench6D stores world [Fx,Fy,Fz,Tx,Ty,Tz]. PayloadMass1D retains
 * the legacy six-element storage and stores delta-mass [kg] in estimate(0).
 * PayloadInertial10D stores [m,hx,hy,hz,Jxx,Jxy,Jxz,Jyy,Jyz,Jzz].
 */
enum class AdaptiveParameterization {
  ConstantWrench6D,
  PayloadMass1D,
  PayloadInertial10D,
  /** [payload inertial increment(10); extra wrench(6)]. In paper mode the final six parameters are base-frame [F;T]; legacy mode keeps the historical world-frame convention. */
  PayloadInertial10DPlusWrench6D
};

/**
 * Prediction model used for Eq. (10).
 *
 * LegacyEstimatedModel preserves historical experiments that rebuilt the
 * complete SRBD from pi_hat. PaperRegressorCompensation implements the
 * published equation literally: nominal flow minus Y_u(q,v,v_r,dot(v_r))
 * pi_hat through the nominal inertia.
 */
enum class AdaptiveDynamicsForm {
  LegacyEstimatedModel,
  PaperRegressorCompensation
};

/** Storage dimension p used by Eq. (8), Eq. (11), and frozen snapshots. */
std::size_t parameterDimension(AdaptiveParameterization parameterization);

/**
 * 公共 ACLF 参数。所有矩阵均由 INFO 配置读取；enable 标志由 interface
 * 分别控制 Eq.(8)、Eq.(10)、Eq.(11) 的装配/运行模式。
 */
struct AdaptiveClfSettings {
  matrix3_t lambdaLinear = matrix3_t::Identity();
  matrix3_t lambdaAngular = matrix3_t::Identity();
  parameter_matrix_t gamma = parameter_matrix_t::Identity(6, 6);
  matrix6_t kd = matrix6_t::Identity();
  parameter_vector_t initialParameterEstimate = parameter_vector_t::Zero(6);
  AdaptiveParameterization parameterization = AdaptiveParameterization::ConstantWrench6D;
  AdaptiveDynamicsForm adaptiveDynamicsForm = AdaptiveDynamicsForm::LegacyEstimatedModel;
  /** Paper Sec. V-A Gamma_m for the scalar payload-mass parameter [kg]. */
  scalar_t payloadMassAdaptationGain = 5.0;
  /** Numerical validity floor used only by PayloadMass1D prediction dynamics. */
  scalar_t minimumEstimatedTotalMass = 1.0;
  /**
   * D5H1 robustness candidate.  When enabled for PayloadInertial10D only,
   * the frozen nominal gait-phase mean is subtracted from Y^T sigma before
   * Gamma is applied.  It is default-off so all historical A--D modes retain
   * the exact paper Eq. (8) implementation.
   */
  bool enableGaitPhaseBaselineCorrection = false;
  std::string gaitPhaseBaselineFile;
  std::size_t gaitPhaseBins = 24;
  scalar_t gaitCycleDuration = 1.2;
  scalar_t gaitBaselineMinimumCycleStartTime = 2.2;
  /**
   * D5I4 E3 composite-phase robustness candidate.  This branch is mutually
   * exclusive with the historical D5H1 gait-only correction and default OFF.
   */
  bool enableCompositePhaseBaselineCorrection = false;
  std::string compositePhaseBaselineFile;
  std::size_t compositeProfilePhaseBins = 120;
  std::size_t compositeGaitPhaseBins = 24;
  scalar_t compositeProfilePeriod = 12.0;
  scalar_t compositeProfileStartTime = 2.4;
  scalar_t compositeRampInDuration = 2.4;
  scalar_t compositeSteadyDuration = 24.0;
  scalar_t compositeGaitCycleDuration = 1.2;
  /**
   * D5I1 reference transport candidate.  When true, desired linear/angular
   * velocity is decoded from the centroidal momentum stored in
   * TargetTrajectories, and desired acceleration is the segment derivative of
   * that encoded velocity.  Default false preserves the historical x-only
   * pose-secant contract byte for byte.
   */
  bool enableEncodedReferenceDerivatives = false;
  bool enableParameterUpdate = true;
  bool enableAdaptiveDynamics = true;
  bool enableClfConstraint = false;
  bool enableInputBiasCost = false;
  bool useSoftConstraint = true;
  scalar_t softConstraintMu = 0.1;
  scalar_t softConstraintDelta = 5.0;
  scalar_t timestampTolerance = 1.0e-9;
};

/** 从 adaptive_settings.info 的 adaptive_clf_mpc 节读取并验证 6D 公共数学参数。 */
AdaptiveClfSettings loadAdaptiveClfSettings(const std::string& filename,
                                            const std::string& prefix = "adaptive_clf_mpc");
void validateAdaptiveClfSettings(const AdaptiveClfSettings& settings);
const char* adaptiveParameterizationName(AdaptiveParameterization parameterization);
const char* adaptiveDynamicsFormName(AdaptiveDynamicsForm form);

}  // namespace adaptive
}  // namespace legged_robot_adaptive
}  // namespace ocs2
