#include "ocs2_legged_robot_adaptive/dynamics/AdaptiveLeggedRobotDynamics.h"

#include "ocs2_legged_robot_adaptive/adaptive/PayloadInertial10D.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace ocs2 {
namespace legged_robot_adaptive {

AdaptiveLeggedRobotDynamics::AdaptiveLeggedRobotDynamics(
    const PinocchioInterface& pinocchioInterface,
    CentroidalModelInfo centroidalModelInfo, const std::string& modelName,
    const legged_robot::ModelSettings& modelSettings,
    adaptive::AdaptiveClfSettings settings,
    std::shared_ptr<const adaptive::AdaptiveModelData> modelData,
    const legged_robot::SwitchedModelReferenceManager* referenceManager)
    : nominalDynamics_(pinocchioInterface, centroidalModelInfo,
                       modelName + "_adaptive", modelSettings),
      pinocchioInterface_(pinocchioInterface),
      info_(std::move(centroidalModelInfo)),
      modelName_(modelName),
      modelSettings_(modelSettings),
      settings_(std::move(settings)),
      parameterization_(settings_.parameterization),
      adaptiveDynamicsForm_(settings_.adaptiveDynamicsForm),
      minimumEstimatedTotalMass_(settings_.minimumEstimatedTotalMass),
      modelData_(std::move(modelData)),
      referenceManagerPtr_(referenceManager),
      regressor_({info_.robotMass, info_.centroidalInertiaNominal,
                  adaptive::vector3_t(0.0, 0.0, -9.81)}) {
  if (info_.centroidalModelType !=
          CentroidalModelType::SingleRigidBodyDynamics ||
      !(info_.robotMass > 0.0) || !modelData_) {
    throw std::invalid_argument(
        "[AdaptiveLeggedRobotDynamics] valid SRBD, mass, and snapshot channel "
        "are required.");
  }
  if (adaptiveDynamicsForm_ ==
      adaptive::AdaptiveDynamicsForm::PaperRegressorCompensation) {
    if (referenceManagerPtr_ == nullptr) {
      throw std::invalid_argument(
          "[AdaptiveLeggedRobotDynamics] paper Eq. (10) requires the active "
          "reference manager.");
    }
    referenceKinematicsPtr_.reset(
        new adaptive::AdaptiveReferenceKinematicsAdapter(
            pinocchioInterface_, info_, settings_));
  }
}

AdaptiveLeggedRobotDynamics* AdaptiveLeggedRobotDynamics::clone() const {
  return new AdaptiveLeggedRobotDynamics(
      pinocchioInterface_, info_, modelName_, modelSettings_, settings_,
      modelData_, referenceManagerPtr_);
}

adaptive::vector6_t
AdaptiveLeggedRobotDynamics::normalizedMomentumRateCorrection(
    const adaptive::vector6_t& parameterEstimate, scalar_t robotMass) {
  if (!parameterEstimate.allFinite() || !(robotMass > 0.0)) {
    throw std::invalid_argument(
        "[AdaptiveLeggedRobotDynamics] invalid ConstantWrench6D or mass.");
  }
  return -parameterEstimate / robotMass;
}

adaptive::vector6_t
AdaptiveLeggedRobotDynamics::normalizedMomentumRateCorrection(
    const adaptive::regressor_t& regressor,
    const adaptive::parameter_vector_t& parameterEstimate,
    scalar_t robotMass) {
  if (regressor.rows() != 6 ||
      regressor.cols() != parameterEstimate.size() ||
      !regressor.allFinite() || !parameterEstimate.allFinite() ||
      !(robotMass > 0.0)) {
    throw std::invalid_argument(
        "[AdaptiveLeggedRobotDynamics] invalid paper Eq. (10) regressor, "
        "estimate, or nominal mass.");
  }
  return -(regressor * parameterEstimate) / robotMass;
}

adaptive::vector6_t
AdaptiveLeggedRobotDynamics::normalizedMomentumRateCorrection(
    adaptive::AdaptiveParameterization parameterization,
    const adaptive::vector6_t& parameterEstimate,
    const adaptive::vector6_t& nominalNormalizedMomentumRate,
    scalar_t robotMass, scalar_t minimumEstimatedTotalMass) {
  if (parameterization ==
      adaptive::AdaptiveParameterization::ConstantWrench6D) {
    return normalizedMomentumRateCorrection(parameterEstimate, robotMass);
  }
  if (parameterization !=
      adaptive::AdaptiveParameterization::PayloadMass1D) {
    throw std::invalid_argument(
        "[AdaptiveLeggedRobotDynamics] use the regressor overload for "
        "PayloadInertial10D paper dynamics.");
  }
  if (!parameterEstimate.tail<5>().isZero(0.0)) {
    throw std::invalid_argument(
        "[AdaptiveLeggedRobotDynamics] PayloadMass1D inactive estimate "
        "elements must be zero.");
  }
  return adaptive::PayloadMass1D::normalizedMomentumRateCorrection(
      nominalNormalizedMomentumRate, robotMass, parameterEstimate(0),
      minimumEstimatedTotalMass);
}

adaptive::vector6_t AdaptiveLeggedRobotDynamics::paperRegressorCorrection(
    scalar_t time, const vector_t& state,
    const adaptive::AdaptiveModelSnapshotPtr& snapshot) const {
  if (!snapshot || !snapshot->valid() || !referenceKinematicsPtr_ ||
      !referenceManagerPtr_) {
    throw std::runtime_error(
        "[AdaptiveLeggedRobotDynamics] paper Eq. (10) dependencies are not "
        "available.");
  }
  const auto reference = referenceKinematicsPtr_->compute(
      time, state, referenceManagerPtr_->getTargetTrajectories());
  adaptive::AdaptiveRigidBodyRegressor::EvaluationPoint point;
  point.rotationWorldFromBase =
      reference.actual.orientationWorldFromBase.toRotationMatrix();
  point.actualAngularVelocityWorld =
      reference.actual.angularVelocityWorld;
  point.referenceLinearAccelerationWorld =
      reference.referenceLinearAccelerationWorld;
  point.referenceAngularVelocityWorld =
      reference.referenceAngularVelocityWorld;
  point.referenceAngularAccelerationWorld =
      reference.referenceAngularAccelerationWorld;
  const adaptive::regressor_t regressor =
      regressor_.computeUnknownDynamicsRegressor(
          point, parameterization_, adaptiveDynamicsForm_);
  return normalizedMomentumRateCorrection(
      regressor, snapshot->estimate(), info_.robotMass);
}

vector_t AdaptiveLeggedRobotDynamics::computeFlowMap(
    scalar_t time, const vector_t& state, const vector_t& input,
    const PreComputation& preComputation) {
  const auto snapshot = modelData_->snapshot();
  if (!snapshot || !snapshot->valid()) {
    throw std::runtime_error(
        "[AdaptiveLeggedRobotDynamics] a valid frozen snapshot is required.");
  }

  if (adaptiveDynamicsForm_ ==
      adaptive::AdaptiveDynamicsForm::PaperRegressorCompensation) {
    vector_t flow =
        nominalDynamics_.computeFlowMap(time, state, input, preComputation);
    flow.head<6>() += paperRegressorCorrection(time, state, snapshot);
    return flow;
  }

  if (parameterization_ ==
      adaptive::AdaptiveParameterization::PayloadInertial10D) {
    return payloadInertialDynamics(snapshot).computeFlowMap(
        time, state, input, preComputation);
  }
  if (parameterization_ ==
      adaptive::AdaptiveParameterization::PayloadInertial10DPlusWrench6D) {
    if (snapshot->estimate().size() != 16) {
      throw std::runtime_error(
          "[AdaptiveLeggedRobotDynamics] joint mode requires a 16D frozen "
          "snapshot.");
    }
    vector_t flow =
        payloadInertialDynamics(snapshot->estimate().head<10>(),
                                snapshot->version())
            .computeFlowMap(time, state, input, preComputation);
    flow.head<6>() += normalizedMomentumRateCorrection(
        snapshot->estimate().tail<6>(), info_.robotMass);
    return flow;
  }
  if (snapshot->estimate().size() != 6) {
    throw std::runtime_error(
        "[AdaptiveLeggedRobotDynamics] legacy snapshot must have dimension "
        "six.");
  }
  vector_t flowMap =
      nominalDynamics_.computeFlowMap(time, state, input, preComputation);
  const adaptive::vector6_t nominalMomentumRate = flowMap.head<6>();
  flowMap.head<6>() += normalizedMomentumRateCorrection(
      parameterization_, snapshot->estimate().head<6>(),
      nominalMomentumRate, info_.robotMass, minimumEstimatedTotalMass_);
  return flowMap;
}

VectorFunctionLinearApproximation
AdaptiveLeggedRobotDynamics::linearApproximation(
    scalar_t time, const vector_t& state, const vector_t& input,
    const PreComputation& preComputation) {
  const auto snapshot = modelData_->snapshot();
  if (!snapshot || !snapshot->valid()) {
    throw std::runtime_error(
        "[AdaptiveLeggedRobotDynamics] a valid frozen snapshot is required "
        "before linearization.");
  }

  if (adaptiveDynamicsForm_ ==
      adaptive::AdaptiveDynamicsForm::PaperRegressorCompensation) {
    auto approximation = nominalDynamics_.linearApproximation(
        time, state, input, preComputation);
    approximation.f.head<6>() +=
        paperRegressorCorrection(time, state, snapshot);

    // Y_u depends on q, v, v_r, and dot(v_r). The estimate is frozen during
    // one solver run, but the regressor is not. Differentiate that exact
    // state dependence instead of treating Y_u*pi_hat as a constant wrench.
    for (Eigen::Index column = 0; column < state.size(); ++column) {
      const scalar_t step =
          1.0e-6 * std::max<scalar_t>(1.0, std::abs(state(column)));
      vector_t plusState = state;
      vector_t minusState = state;
      plusState(column) += step;
      minusState(column) -= step;
      approximation.dfdx.block(0, column, 6, 1) +=
          (paperRegressorCorrection(time, plusState, snapshot) -
           paperRegressorCorrection(time, minusState, snapshot)) /
          (2.0 * step);
    }
    return approximation;
  }

  if (parameterization_ ==
      adaptive::AdaptiveParameterization::PayloadInertial10D) {
    return payloadInertialDynamics(snapshot).linearApproximation(
        time, state, input, preComputation);
  }
  if (parameterization_ ==
      adaptive::AdaptiveParameterization::PayloadInertial10DPlusWrench6D) {
    if (snapshot->estimate().size() != 16) {
      throw std::runtime_error(
          "[AdaptiveLeggedRobotDynamics] joint mode requires a 16D frozen "
          "snapshot.");
    }
    auto approximation =
        payloadInertialDynamics(snapshot->estimate().head<10>(),
                                snapshot->version())
            .linearApproximation(time, state, input, preComputation);
    approximation.f.head<6>() += normalizedMomentumRateCorrection(
        snapshot->estimate().tail<6>(), info_.robotMass);
    return approximation;
  }
  if (snapshot->estimate().size() != 6) {
    throw std::runtime_error(
        "[AdaptiveLeggedRobotDynamics] legacy snapshot must have dimension "
        "six.");
  }
  auto approximation = nominalDynamics_.linearApproximation(
      time, state, input, preComputation);
  const adaptive::vector6_t nominalMomentumRate =
      approximation.f.head<6>();
  approximation.f.head<6>() += normalizedMomentumRateCorrection(
      parameterization_, snapshot->estimate().head<6>(),
      nominalMomentumRate, info_.robotMass, minimumEstimatedTotalMass_);
  if (parameterization_ ==
      adaptive::AdaptiveParameterization::PayloadMass1D) {
    const scalar_t estimatedMass =
        adaptive::PayloadMass1D::estimatedTotalMass(
            info_.robotMass, snapshot->estimate()(0),
            minimumEstimatedTotalMass_);
    const scalar_t scale = info_.robotMass / estimatedMass;
    approximation.dfdx.topRows<6>() *= scale;
    approximation.dfdu.topRows<6>() *= scale;
  }
  return approximation;
}

NominalCoordinatePlantDynamics&
AdaptiveLeggedRobotDynamics::payloadInertialDynamics(
    const adaptive::AdaptiveModelSnapshotPtr& snapshot) {
  if (!snapshot || !snapshot->valid() ||
      snapshot->estimate().size() != 10) {
    throw std::invalid_argument(
        "[AdaptiveLeggedRobotDynamics] PayloadInertial10D requires a valid "
        "10D frozen snapshot.");
  }
  return payloadInertialDynamics(snapshot->estimate(), snapshot->version());
}

NominalCoordinatePlantDynamics&
AdaptiveLeggedRobotDynamics::payloadInertialDynamics(
    const vector_t& estimate, std::uint64_t snapshotVersion) {
  if (estimate.size() != 10 || !estimate.allFinite() ||
      snapshotVersion == 0) {
    throw std::invalid_argument(
        "[AdaptiveLeggedRobotDynamics] invalid frozen inertial estimate.");
  }
  if (!payloadInertialDynamicsPtr_ ||
      payloadInertialSnapshotVersion_ != snapshotVersion) {
    const adaptive::vector10_t inertialEstimate = estimate;
    const CentroidalModelInfo estimatedInfo =
        adaptive::PayloadInertial10D::makeEstimatedModelInfo(
            info_, inertialEstimate, minimumEstimatedTotalMass_);
    payloadInertialDynamicsPtr_.reset(
        new NominalCoordinatePlantDynamics(
            pinocchioInterface_, info_, estimatedInfo));
    payloadInertialSnapshotVersion_ = snapshotVersion;
  }
  return *payloadInertialDynamicsPtr_;
}

}  // namespace legged_robot_adaptive
}  // namespace ocs2