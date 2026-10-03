#include "ocs2_legged_robot_adaptive/adaptive/AdaptiveTypes.h"

#include <stdexcept>
#include <utility>
#include <Eigen/Cholesky>
#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <ocs2_core/misc/LoadData.h>

namespace ocs2 {
namespace legged_robot_adaptive {
namespace adaptive {
namespace {
template <typename MatrixType>
void loadMatrix(const std::string& file, const std::string& key, MatrixType& value) {
  // OCS2 的 (row,column) INFO 矩阵格式保留 K_D 的完整矩阵语义，绝不退化为 clfRate。
  ocs2::loadData::loadEigenMatrix(file, key, value);
}
template <typename MatrixType>
void requireSpd(const MatrixType& value, const char* name) {
  if (!value.allFinite() || (value - value.transpose()).norm() > 1e-10 ||
      Eigen::LLT<matrix_t>(matrix_t(value)).info() != Eigen::Success) {
    throw std::invalid_argument(std::string("[AdaptiveClfSettings] ") + name + " 必须为有限对称正定矩阵。");
  }
}
}  // namespace

void validateAdaptiveClfSettings(const AdaptiveClfSettings& settings) {
  requireSpd(settings.lambdaLinear, "Lambda_linear");
  requireSpd(settings.lambdaAngular, "Lambda_angular");
  requireSpd(settings.kd, "K_D");
  const Eigen::Index p = static_cast<Eigen::Index>(
      parameterDimension(settings.parameterization));
  if (settings.gamma.rows() != p || settings.gamma.cols() != p ||
      settings.initialParameterEstimate.size() != p) {
    throw std::invalid_argument(
        "[AdaptiveClfSettings] Gamma/initial estimate dimensions do not match parameterization.");
  }
  requireSpd(settings.gamma, "Gamma");
  if (!settings.initialParameterEstimate.allFinite() || !std::isfinite(settings.timestampTolerance) ||
      settings.timestampTolerance < 0.0) {
    throw std::invalid_argument("[AdaptiveClfSettings] initial estimate 或 timestampTolerance 非法。");
  }
  if (!std::isfinite(settings.payloadMassAdaptationGain) ||
      !(settings.payloadMassAdaptationGain > 0.0) ||
      !std::isfinite(settings.minimumEstimatedTotalMass) ||
      !(settings.minimumEstimatedTotalMass > 0.0)) {
    throw std::invalid_argument(
        "[AdaptiveClfSettings] PayloadMass1D gain/minimum total mass must be finite and positive.");
  }
  if (settings.parameterization == AdaptiveParameterization::PayloadMass1D &&
      !settings.initialParameterEstimate.tail(5).isZero(0.0)) {
    throw std::invalid_argument(
        "[AdaptiveClfSettings] PayloadMass1D only stores delta-mass in initialParameterEstimate(0).");
  }
  if (settings.enableGaitPhaseBaselineCorrection) {
    if (settings.parameterization != AdaptiveParameterization::PayloadInertial10D ||
        settings.gaitPhaseBaselineFile.empty() || settings.gaitPhaseBins == 0 ||
        settings.gaitPhaseBins % 4 != 0 ||
        !std::isfinite(settings.gaitCycleDuration) ||
        settings.gaitCycleDuration <= 0.0 ||
        !std::isfinite(settings.gaitBaselineMinimumCycleStartTime) ||
        settings.gaitBaselineMinimumCycleStartTime < 0.0) {
      throw std::invalid_argument(
          "[AdaptiveClfSettings] gait-phase baseline correction requires "
          "PayloadInertial10D, a file, valid 4-mode bins, and finite timing.");
    }
  }
  if (settings.enableCompositePhaseBaselineCorrection) {
    if (settings.enableGaitPhaseBaselineCorrection ||
        settings.parameterization != AdaptiveParameterization::PayloadInertial10D ||
        settings.compositePhaseBaselineFile.empty() ||
        settings.compositeProfilePhaseBins == 0 ||
        settings.compositeGaitPhaseBins == 0 ||
        settings.compositeGaitPhaseBins % 4 != 0 ||
        !std::isfinite(settings.compositeProfilePeriod) ||
        settings.compositeProfilePeriod <= 0.0 ||
        !std::isfinite(settings.compositeProfileStartTime) ||
        settings.compositeProfileStartTime < 0.0 ||
        !std::isfinite(settings.compositeRampInDuration) ||
        settings.compositeRampInDuration <= 0.0 ||
        !std::isfinite(settings.compositeSteadyDuration) ||
        settings.compositeSteadyDuration <= 0.0 ||
        !std::isfinite(settings.compositeGaitCycleDuration) ||
        settings.compositeGaitCycleDuration <= 0.0) {
      throw std::invalid_argument(
          "[AdaptiveClfSettings] composite-phase correction requires a "
          "PayloadInertial10D sparse baseline, valid timing, and no gait-only correction.");
    }
  }
}

std::size_t parameterDimension(AdaptiveParameterization parameterization) {
  switch (parameterization) {
    case AdaptiveParameterization::ConstantWrench6D:
    case AdaptiveParameterization::PayloadMass1D:
      return 6;
    case AdaptiveParameterization::PayloadInertial10D:
      return 10;
    case AdaptiveParameterization::PayloadInertial10DPlusWrench6D:
      return 16;
  }
  throw std::invalid_argument("[AdaptiveClfSettings] Unknown adaptive parameterization.");
}

const char* adaptiveParameterizationName(AdaptiveParameterization parameterization) {
  switch (parameterization) {
    case AdaptiveParameterization::ConstantWrench6D:
      return "constant_wrench_6d";
    case AdaptiveParameterization::PayloadMass1D:
      return "payload_mass_1d";
    case AdaptiveParameterization::PayloadInertial10D:
      return "payload_inertial_10d";
    case AdaptiveParameterization::PayloadInertial10DPlusWrench6D:
      return "payload_inertial_10d_plus_wrench_6d";
  }
  throw std::invalid_argument("[AdaptiveClfSettings] Unknown adaptive parameterization.");
}

const char* adaptiveDynamicsFormName(AdaptiveDynamicsForm form) {
  switch (form) {
    case AdaptiveDynamicsForm::LegacyEstimatedModel:
      return "legacy_estimated_model";
    case AdaptiveDynamicsForm::PaperRegressorCompensation:
      return "paper_regressor_compensation";
  }
  throw std::invalid_argument("[AdaptiveClfSettings] Unknown adaptive dynamics form.");
}

AdaptiveClfSettings loadAdaptiveClfSettings(const std::string& file, const std::string& prefix) {
  AdaptiveClfSettings settings;
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(file, pt);
  const std::string parameterization = pt.get<std::string>(prefix + ".parameterization");
  if (parameterization == "constant_wrench_6d") {
    settings.parameterization = AdaptiveParameterization::ConstantWrench6D;
  } else if (parameterization == "payload_mass_1d") {
    settings.parameterization = AdaptiveParameterization::PayloadMass1D;
  } else if (parameterization == "payload_inertial_10d") {
    settings.parameterization = AdaptiveParameterization::PayloadInertial10D;
  } else if (parameterization == "payload_inertial_10d_plus_wrench_6d") {
    settings.parameterization = AdaptiveParameterization::PayloadInertial10DPlusWrench6D;
  } else {
    throw std::invalid_argument(
        "[AdaptiveClfSettings] parameterization must be constant_wrench_6d, payload_mass_1d, payload_inertial_10d, or payload_inertial_10d_plus_wrench_6d.");
  }
  const std::string dynamicsForm = pt.get<std::string>(
      prefix + ".adaptiveDynamicsForm", "legacy_estimated_model");
  if (dynamicsForm == "legacy_estimated_model") {
    settings.adaptiveDynamicsForm = AdaptiveDynamicsForm::LegacyEstimatedModel;
  } else if (dynamicsForm == "paper_regressor_compensation") {
    settings.adaptiveDynamicsForm =
        AdaptiveDynamicsForm::PaperRegressorCompensation;
  } else {
    throw std::invalid_argument(
        "[AdaptiveClfSettings] adaptiveDynamicsForm must be "
        "legacy_estimated_model or paper_regressor_compensation.");
  }
  const Eigen::Index p = static_cast<Eigen::Index>(
      parameterDimension(settings.parameterization));
  settings.gamma = parameter_matrix_t::Identity(p, p);
  settings.initialParameterEstimate = parameter_vector_t::Zero(p);
  settings.enableParameterUpdate = pt.get<bool>(prefix + ".enableParameterUpdate", true);
  settings.enableAdaptiveDynamics = pt.get<bool>(prefix + ".enableAdaptiveDynamics", true);
  settings.enableClfConstraint = pt.get<bool>(prefix + ".enableClfConstraint", false);
  settings.enableInputBiasCost = pt.get<bool>(prefix + ".enableInputBiasCost", false);
  settings.useSoftConstraint = pt.get<bool>(prefix + ".useSoftConstraint", true);
  settings.softConstraintMu = pt.get<scalar_t>(prefix + ".softConstraintMu", 0.1);
  settings.softConstraintDelta = pt.get<scalar_t>(prefix + ".softConstraintDelta", 5.0);
  settings.timestampTolerance = pt.get<scalar_t>(prefix + ".timestampTolerance", 1.0e-9);
  settings.payloadMassAdaptationGain =
      pt.get<scalar_t>(prefix + ".payloadMassAdaptationGain", 5.0);
  settings.minimumEstimatedTotalMass =
      pt.get<scalar_t>(prefix + ".minimumEstimatedTotalMass", 1.0);
  settings.enableGaitPhaseBaselineCorrection =
      pt.get<bool>(prefix + ".enableGaitPhaseBaselineCorrection", false);
  settings.gaitPhaseBaselineFile =
      pt.get<std::string>(prefix + ".gaitPhaseBaselineFile", "");
  settings.gaitPhaseBins =
      pt.get<std::size_t>(prefix + ".gaitPhaseBins", 24);
  settings.gaitCycleDuration =
      pt.get<scalar_t>(prefix + ".gaitCycleDuration", 1.2);
  settings.gaitBaselineMinimumCycleStartTime =
      pt.get<scalar_t>(prefix + ".gaitBaselineMinimumCycleStartTime", 2.2);
  settings.enableCompositePhaseBaselineCorrection =
      pt.get<bool>(prefix + ".enableCompositePhaseBaselineCorrection", false);
  settings.compositePhaseBaselineFile =
      pt.get<std::string>(prefix + ".compositePhaseBaselineFile", "");
  settings.compositeProfilePhaseBins =
      pt.get<std::size_t>(prefix + ".compositeProfilePhaseBins", 120);
  settings.compositeGaitPhaseBins =
      pt.get<std::size_t>(prefix + ".compositeGaitPhaseBins", 24);
  settings.compositeProfilePeriod =
      pt.get<scalar_t>(prefix + ".compositeProfilePeriod", 12.0);
  settings.compositeProfileStartTime =
      pt.get<scalar_t>(prefix + ".compositeProfileStartTime", 2.4);
  settings.compositeRampInDuration =
      pt.get<scalar_t>(prefix + ".compositeRampInDuration", 2.4);
  settings.compositeSteadyDuration =
      pt.get<scalar_t>(prefix + ".compositeSteadyDuration", 24.0);
  settings.compositeGaitCycleDuration =
      pt.get<scalar_t>(prefix + ".compositeGaitCycleDuration", 1.2);
  settings.enableEncodedReferenceDerivatives =
      pt.get<bool>(prefix + ".enableEncodedReferenceDerivatives", false);
  loadMatrix(file, prefix + ".lambdaLinear", settings.lambdaLinear);
  loadMatrix(file, prefix + ".lambdaAngular", settings.lambdaAngular);
  loadMatrix(file, prefix + ".gamma", settings.gamma);
  loadMatrix(file, prefix + ".kd", settings.kd);
  loadMatrix(file, prefix + ".initialParameterEstimate", settings.initialParameterEstimate);
  validateAdaptiveClfSettings(settings);
  return settings;
}

}  // namespace adaptive
}  // namespace legged_robot_adaptive
}  // namespace ocs2
