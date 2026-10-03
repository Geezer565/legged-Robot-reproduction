//
// Created by qiayuan on 2022/7/1.
//
#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include "legged_wbc/WbcBase.h"
#include "legged_wbc/MyRbfV2WbcMath.h"
#include "legged_wbc/MyRbfV2WbcDynamics.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <utility>
#include <algorithm>
#include <cmath>
#include <Eigen/LU>

namespace legged {
WbcBase::WbcBase(const PinocchioInterface& pinocchioInterface, CentroidalModelInfo info, const PinocchioEndEffectorKinematics& eeKinematics)
    : pinocchioInterfaceMeasured_(pinocchioInterface),
      pinocchioInterfaceDesired_(pinocchioInterface),
      info_(std::move(info)),
      mapping_(info_),
      inputLast_(vector_t::Zero(info_.inputDim)),
      eeKinematics_(eeKinematics.clone()) {
  numDecisionVars_ = info_.generalizedCoordinatesNum + 3 * info_.numThreeDofContacts + info_.actuatedDofNum;
  qMeasured_ = vector_t(info_.generalizedCoordinatesNum);
  vMeasured_ = vector_t(info_.generalizedCoordinatesNum);
}

vector_t WbcBase::update(const vector_t& stateDesired, const vector_t& inputDesired, const vector_t& rbdStateMeasured, size_t mode,
                         scalar_t /*period*/) {
  contactFlag_ = modeNumber2StanceLeg(mode);
  numContacts_ = 0;
  for (bool flag : contactFlag_) {
    if (flag) {
      numContacts_++;
    }
  }

  updateMeasured(rbdStateMeasured);
  updateDesired(stateDesired, inputDesired);

  return {};
}

void WbcBase::updateMeasured(const vector_t& rbdStateMeasured) {
  qMeasured_.head<3>() = rbdStateMeasured.segment<3>(3);
  qMeasured_.segment<3>(3) = rbdStateMeasured.head<3>();
  qMeasured_.tail(info_.actuatedDofNum) = rbdStateMeasured.segment(6, info_.actuatedDofNum);
  vMeasured_.head<3>() = rbdStateMeasured.segment<3>(info_.generalizedCoordinatesNum + 3);
  vMeasured_.segment<3>(3) = getEulerAnglesZyxDerivativesFromGlobalAngularVelocity<scalar_t>(
      qMeasured_.segment<3>(3), rbdStateMeasured.segment<3>(info_.generalizedCoordinatesNum));
  vMeasured_.tail(info_.actuatedDofNum) = rbdStateMeasured.segment(info_.generalizedCoordinatesNum + 6, info_.actuatedDofNum);

  const auto& model = pinocchioInterfaceMeasured_.getModel();
  auto& data = pinocchioInterfaceMeasured_.getData();

  // For floating base EoM task
  pinocchio::forwardKinematics(model, data, qMeasured_, vMeasured_);
  pinocchio::computeJointJacobians(model, data);
  pinocchio::updateFramePlacements(model, data);
  pinocchio::crba(model, data, qMeasured_);
  data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();
  pinocchio::nonLinearEffects(model, data, qMeasured_, vMeasured_);
  j_ = matrix_t(3 * info_.numThreeDofContacts, info_.generalizedCoordinatesNum);
  for (size_t i = 0; i < info_.numThreeDofContacts; ++i) {
    Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> jac;
    jac.setZero(6, info_.generalizedCoordinatesNum);
    pinocchio::getFrameJacobian(model, data, info_.endEffectorFrameIndices[i], pinocchio::LOCAL_WORLD_ALIGNED, jac);
    j_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) = jac.template topRows<3>();
  }

  // For not contact motion task
  pinocchio::computeJointJacobiansTimeVariation(model, data, qMeasured_, vMeasured_);
  dj_ = matrix_t(3 * info_.numThreeDofContacts, info_.generalizedCoordinatesNum);
  for (size_t i = 0; i < info_.numThreeDofContacts; ++i) {
    Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> jac;
    jac.setZero(6, info_.generalizedCoordinatesNum);
    pinocchio::getFrameJacobianTimeVariation(model, data, info_.endEffectorFrameIndices[i], pinocchio::LOCAL_WORLD_ALIGNED, jac);
    dj_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) = jac.template topRows<3>();
  }
}

void WbcBase::updateDesired(const vector_t& stateDesired, const vector_t& inputDesired) {
  const auto& model = pinocchioInterfaceDesired_.getModel();
  auto& data = pinocchioInterfaceDesired_.getData();

  mapping_.setPinocchioInterface(pinocchioInterfaceDesired_);
  const auto qDesired = mapping_.getPinocchioJointPosition(stateDesired);
  pinocchio::forwardKinematics(model, data, qDesired);
  pinocchio::computeJointJacobians(model, data, qDesired);
  pinocchio::updateFramePlacements(model, data);
  updateCentroidalDynamics(pinocchioInterfaceDesired_, info_, qDesired);
  const vector_t vDesired = mapping_.getPinocchioJointVelocity(stateDesired, inputDesired);
  pinocchio::forwardKinematics(model, data, qDesired, vDesired);
}

Task WbcBase::formulateFloatingBaseEomTask() {
  auto& data = pinocchioInterfaceMeasured_.getData();

  matrix_t s(info_.actuatedDofNum, info_.generalizedCoordinatesNum);
  s.block(0, 0, info_.actuatedDofNum, 6).setZero();
  s.block(0, 6, info_.actuatedDofNum, info_.actuatedDofNum).setIdentity();

  matrix_t a = (matrix_t(info_.generalizedCoordinatesNum, numDecisionVars_) << data.M, -j_.transpose(), -s.transpose()).finished();
  vector_t b = -data.nle;
  if (myRbfV2Input_.enabled && myRbfV2Input_.injectExternalWrenchInDynamics) {
    // M*qdd + nle = J^T*f + S^T*tau + [T^T*w_ext; 0].
    b.head<6>() += myRbfV2Transform_.transpose() * myRbfV2Input_.externalWrenchEstimate;
  }

  return {a, b, matrix_t(), vector_t()};
}

Task WbcBase::formulateTorqueLimitsTask() {
  matrix_t d(2 * info_.actuatedDofNum, numDecisionVars_);
  d.setZero();
  matrix_t i = matrix_t::Identity(info_.actuatedDofNum, info_.actuatedDofNum);
  d.block(0, info_.generalizedCoordinatesNum + 3 * info_.numThreeDofContacts, info_.actuatedDofNum, info_.actuatedDofNum) = i;
  d.block(info_.actuatedDofNum, info_.generalizedCoordinatesNum + 3 * info_.numThreeDofContacts, info_.actuatedDofNum,
          info_.actuatedDofNum) = -i;
  vector_t f(2 * info_.actuatedDofNum);
  for (size_t l = 0; l < 2 * info_.actuatedDofNum / 3; ++l) {
    f.segment<3>(3 * l) = torqueLimits_;
  }

  return {matrix_t(), vector_t(), d, f};
}

Task WbcBase::formulateNoContactMotionTask() {
  matrix_t a(3 * numContacts_, numDecisionVars_);
  vector_t b(a.rows());
  a.setZero();
  b.setZero();
  size_t j = 0;
  for (size_t i = 0; i < info_.numThreeDofContacts; i++) {
    if (contactFlag_[i]) {
      a.block(3 * j, 0, 3, info_.generalizedCoordinatesNum) = j_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum);
      b.segment(3 * j, 3) = -dj_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) * vMeasured_;
      j++;
    }
  }

  return {a, b, matrix_t(), vector_t()};
}

Task WbcBase::formulateFrictionConeTask() {
  matrix_t a(3 * (info_.numThreeDofContacts - numContacts_), numDecisionVars_);
  a.setZero();
  size_t j = 0;
  for (size_t i = 0; i < info_.numThreeDofContacts; ++i) {
    if (!contactFlag_[i]) {
      a.block(3 * j++, info_.generalizedCoordinatesNum + 3 * i, 3, 3) = matrix_t::Identity(3, 3);
    }
  }
  vector_t b(a.rows());
  b.setZero();

  matrix_t frictionPyramic(5, 3);  // clang-format off
  frictionPyramic << 0, 0, -1,
                     1, 0, -frictionCoeff_,
                    -1, 0, -frictionCoeff_,
                     0, 1, -frictionCoeff_,
                     0,-1, -frictionCoeff_;  // clang-format on
  if (myRbfV2Input_.enabled) {
    // Inscribed Coulomb pyramid on the frozen common plane. Legacy rows above
    // are retained byte-for-byte when the conditional sampled mode is disabled.
    frictionPyramic = myrbf_wbc::contactPyramid(myRbfV2ContactFrame_, frictionCoeff_);
  }

  matrix_t d(5 * numContacts_ + 3 * (info_.numThreeDofContacts - numContacts_), numDecisionVars_);
  d.setZero();
  j = 0;
  for (size_t i = 0; i < info_.numThreeDofContacts; ++i) {
    if (contactFlag_[i]) {
      d.block(5 * j++, info_.generalizedCoordinatesNum + 3 * i, 5, 3) = frictionPyramic;
    }
  }
  vector_t f = Eigen::VectorXd::Zero(d.rows());

  return {a, b, d, f};
}

Task WbcBase::formulateBaseAccelTask(const vector_t& stateDesired, const vector_t& inputDesired, scalar_t period) {
  matrix_t a(6, numDecisionVars_);
  a.setZero();
  a.block(0, 0, 6, 6) = matrix_t::Identity(6, 6);

  vector_t jointAccel = centroidal_model::getJointVelocities(inputDesired - inputLast_, info_) / period;
  inputLast_ = inputDesired;
  mapping_.setPinocchioInterface(pinocchioInterfaceDesired_);

  const auto& model = pinocchioInterfaceDesired_.getModel();
  auto& data = pinocchioInterfaceDesired_.getData();
  const auto qDesired = mapping_.getPinocchioJointPosition(stateDesired);
  const vector_t vDesired = mapping_.getPinocchioJointVelocity(stateDesired, inputDesired);

  const auto& A = getCentroidalMomentumMatrix(pinocchioInterfaceDesired_);
  const Matrix6 Ab = A.template leftCols<6>();
  const auto AbInv = computeFloatingBaseCentroidalMomentumMatrixInverse(Ab);
  const auto Aj = A.rightCols(info_.actuatedDofNum);
  const auto ADot = pinocchio::dccrba(model, data, qDesired, vDesired);
  Vector6 centroidalMomentumRate = info_.robotMass * getNormalizedCentroidalMomentumRate(pinocchioInterfaceDesired_, info_, inputDesired);
  if(myRbfV2Input_.enabled && myRbfV2Input_.v11MatchedAcceleration){
    // Same BODY/CoM estimate as the MPC, evaluated at the desired attitude.
    // Do not infer acceleration from nominal contact forces alone after MPC
    // has already planned those forces using the uncertain model.
    const Eigen::Matrix3d R=(Eigen::AngleAxisd(qDesired(3),Eigen::Vector3d::UnitZ())*
        Eigen::AngleAxisd(qDesired(4),Eigen::Vector3d::UnitY())*
        Eigen::AngleAxisd(qDesired(5),Eigen::Vector3d::UnitX())).toRotationMatrix();
    centroidalMomentumRate.head<3>()-=R*myRbfV2Input_.v11LeftHandBodyComWrench.head<3>();
    centroidalMomentumRate.tail<3>()-=R*myRbfV2Input_.v11LeftHandBodyComWrench.tail<3>();
  }
  centroidalMomentumRate.noalias() -= ADot * vDesired;
  centroidalMomentumRate.noalias() -= Aj * jointAccel;

  Vector6 b = AbInv * centroidalMomentumRate;

  return {a, b, matrix_t(), vector_t()};
}

Task WbcBase::formulateSwingLegTask() {
  eeKinematics_->setPinocchioInterface(pinocchioInterfaceMeasured_);
  std::vector<vector3_t> posMeasured = eeKinematics_->getPosition(vector_t());
  std::vector<vector3_t> velMeasured = eeKinematics_->getVelocity(vector_t(), vector_t());
  eeKinematics_->setPinocchioInterface(pinocchioInterfaceDesired_);
  std::vector<vector3_t> posDesired = eeKinematics_->getPosition(vector_t());
  std::vector<vector3_t> velDesired = eeKinematics_->getVelocity(vector_t(), vector_t());

  matrix_t a(3 * (info_.numThreeDofContacts - numContacts_), numDecisionVars_);
  vector_t b(a.rows());
  a.setZero();
  b.setZero();
  size_t j = 0;
  for (size_t i = 0; i < info_.numThreeDofContacts; ++i) {
    if (!contactFlag_[i]) {
      vector3_t accel = swingKp_ * (posDesired[i] - posMeasured[i]) + swingKd_ * (velDesired[i] - velMeasured[i]);
      a.block(3 * j, 0, 3, info_.generalizedCoordinatesNum) = j_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum);
      b.segment(3 * j, 3) = accel - dj_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) * vMeasured_;
      j++;
    }
  }

  return {a, b, matrix_t(), vector_t()};
}

Task WbcBase::formulateContactForceTask(const vector_t& inputDesired) const {
  matrix_t a(3 * info_.numThreeDofContacts, numDecisionVars_);
  vector_t b(a.rows());
  a.setZero();

  for (size_t i = 0; i < info_.numThreeDofContacts; ++i) {
    a.block(3 * i, info_.generalizedCoordinatesNum + 3 * i, 3, 3) = matrix_t::Identity(3, 3);
  }
  b = inputDesired.head(a.rows());

  return {a, b, matrix_t(), vector_t()};
}

void WbcBase::configureMyRbfV2(const MyRbfV2WbcInput& input) {
  if (!input.enabled || !myRbfV2Input_.enabled) {
    myRbfV2PreviousDesiredVelocity_.resize(0);
    myRbfV2PreviousDesiredSnapshotSequenceId_ = 0;
  }
  myRbfV2Input_ = input;
  if (!input.enabled) myRbfV2Diagnostics_ = MyRbfV2WbcDiagnostics{};
}

bool WbcBase::validateMyRbfV2Cycle(const vector_t& stateDesired, const vector_t& inputDesired,
                                 const vector_t& measured, scalar_t period) {
  myRbfV2Diagnostics_ = MyRbfV2WbcDiagnostics{};
  auto& diag = myRbfV2Diagnostics_;
  const auto& in = myRbfV2Input_;
  diag.enabled = in.enabled;
  diag.snapshotSequenceId = in.snapshotSequenceId;
  diag.externalWrenchEstimate = in.externalWrenchEstimate;
  diag.status = MyRbfV2WbcStatus::InvalidInput;
  const auto& model = pinocchioInterfaceMeasured_.getModel();
  const bool dimensions = info_.generalizedCoordinatesNum == info_.actuatedDofNum + 6 &&
      info_.numThreeDofContacts == 4 && info_.numSixDofContacts == 0 && info_.actuatedDofNum == 12 &&
      model.nq == info_.generalizedCoordinatesNum && model.nv == info_.generalizedCoordinatesNum &&
      measured.size() == 2 * info_.generalizedCoordinatesNum && stateDesired.size() == info_.stateDim &&
      inputDesired.size() == info_.inputDim && torqueLimits_.size() == 3 && info_.endEffectorFrameIndices.size() == 4;
  if (!dimensions || !std::isfinite(period) || period <= 0.0 || !measured.allFinite() ||
      !stateDesired.allFinite() || !inputDesired.allFinite() || !in.externalWrenchEstimate.allFinite() ||
      !in.robustBound.allFinite() || (in.robustBound.array() < 0.0).any() || !in.damping.allFinite() ||
      (in.damping.array() <= 0.0).any() || !in.lambda.allFinite() || (in.lambda.array() <= 0.0).any() ||
      !std::isfinite(in.feasibilityTolerance) || in.feasibilityTolerance <= 0.0 ||
      !std::isfinite(in.minEulerCosPitch) || in.minEulerCosPitch <= 0.0 || in.minEulerCosPitch >= 1.0 ||
      !std::isfinite(in.smoothWidth) || in.smoothWidth <= 0.0 ||
      (in.robustMode != MyRbfV2RobustMode::Sign && in.robustMode != MyRbfV2RobustMode::Smooth) ||
      !std::isfinite(in.minNormalForce) || !std::isfinite(in.maxNormalForce) || in.minNormalForce < 0.0 ||
      !in.surfaceNormal.allFinite() || !std::isfinite(in.surfaceNormal.stableNorm()) || in.surfaceNormal.stableNorm() <= 0.0 ||
      in.maxNormalForce < in.minNormalForce || !in.maxAbsContactForce.allFinite() ||
      (in.maxAbsContactForce.array() <= 0.0).any() || !in.maxContactForceRate.allFinite() ||
      (in.maxContactForceRate.array() < 0.0).any() || !torqueLimits_.allFinite() ||
      (torqueLimits_.array() <= 0.0).any() || !std::isfinite(frictionCoeff_) || frictionCoeff_ < 0.0 ||
      !in.previousContactForces.allFinite() ||
      (in.previousContactForces.size() != 0 && in.previousContactForces.size() != 3 * info_.numThreeDofContacts)) return false;
  for (const auto frame : info_.endEffectorFrameIndices) if (frame >= model.nframes) return false;
  // Check measured pitch before angular-velocity inversion in updateMeasured().
  if (std::abs(std::cos(measured[1])) < in.minEulerCosPitch) return false;
  mapping_.setPinocchioInterface(pinocchioInterfaceDesired_);
  const vector_t qd = mapping_.getPinocchioJointPosition(stateDesired);
  if (!qd.allFinite() || std::abs(std::cos(qd[4])) < in.minEulerCosPitch) return false;
  myRbfV2Period_ = period;
  myRbfV2ContactFrame_ = myrbf_wbc::surfaceFrame(in.surfaceNormal);
  diag.status = MyRbfV2WbcStatus::NotSolved;
  return true;
}

bool WbcBase::prepareMyRbfV2Tracking(const vector_t& stateDesired, const vector_t& inputDesired,
                                   const vector_t& measured, size_t mode, scalar_t period) {
  if (!myRbfV2Input_.enabled) return false;
  if (!validateMyRbfV2Cycle(stateDesired, inputDesired, measured, period) || mode > 15) {
    myRbfV2Diagnostics_.status = MyRbfV2WbcStatus::InvalidInput;
    myRbfV2PreviousDesiredVelocity_.resize(0);
    myRbfV2PreviousDesiredSnapshotSequenceId_ = 0;
    return false;
  }
  myRbfV2Diagnostics_.contactMode = mode;
  if (myRbfV2Input_.v11MatchedAcceleration && modeNumber2StanceLeg(mode) != contactFlag_) {
    // A hybrid contact switch permits a force jump: liftoff requires zero
    // force and the remaining/new stance feet must immediately carry load.
    // V1.1 refreshes this input from its configured values on EVERY cycle,
    // so the original rate bound resumes when the contact mode is unchanged.
    // Keep all amplitude, unilateral, friction and torque bounds intact.
    myRbfV2Input_.maxContactForceRate = myRbfV2Input_.maxContactForceRate.cwiseMax(
        (2.0 / period) * myRbfV2Input_.maxAbsContactForce);
  }
  WbcBase::update(stateDesired, inputDesired, measured, mode, period);
  if (!prepareMyRbfV2(stateDesired, inputDesired, period)) {
    myRbfV2Diagnostics_.status = MyRbfV2WbcStatus::InvalidInput;
    myRbfV2PreviousDesiredVelocity_.resize(0);
    myRbfV2PreviousDesiredSnapshotSequenceId_ = 0;
    return false;
  }
  return true;
}

bool WbcBase::prepareMyRbfV2(const vector_t& stateDesired, const vector_t& inputDesired, scalar_t period) {
  const auto& model = pinocchioInterfaceMeasured_.getModel();
  const auto& data = pinocchioInterfaceMeasured_.getData();
  mapping_.setPinocchioInterface(pinocchioInterfaceDesired_);
  const vector_t qd = mapping_.getPinocchioJointPosition(stateDesired);
  const vector_t vd = mapping_.getPinocchioJointVelocity(stateDesired, inputDesired);
  if (!qMeasured_.allFinite() || !vMeasured_.allFinite() || !vd.allFinite()) return false;
  const Vector6 error = myrbf_wbc::localError(qMeasured_.head<6>(), qd.head<6>());
  // Reject the local chart's wrapped-error discontinuity rather than claim a derivative there.
  if ((error.tail<3>().array().abs() > std::acos(-1.0) - 1e-6).any()) return false;
  bool accelerationInitialized = false;
  const Vector6 ad = myrbf_wbc::desiredAcceleration(vd.head<6>(), myRbfV2PreviousDesiredVelocity_, period,
      myRbfV2Input_.snapshotSequenceId, myRbfV2PreviousDesiredSnapshotSequenceId_, accelerationInitialized);
  myRbfV2Diagnostics_.desiredAccelerationInitialized = accelerationInitialized;
  myRbfV2CurrentDesiredVelocity_ = vd.head<6>();
  // Separate data prevents C/g calculations from altering EoM and contact kinematics caches.
  auto referenceData = pinocchioInterfaceMeasured_.getData();
  vector_t vr = vMeasured_;
  vr.head<6>() = vd.head<6>() - myRbfV2Input_.lambda.cwiseProduct(error);
  const vector_t referenceBias = myrbf_wbc::eulerReferenceBias(model, referenceData, qMeasured_, vMeasured_, vr);
  const auto ref = myrbf_wbc::referenceFromBias(data.M, referenceBias, qMeasured_, vMeasured_, qd.head<6>(),
                                      vd.head<6>(), ad, myRbfV2Input_.lambda);
  myRbfV2Transform_ = myrbf_wbc::baseTransform(qMeasured_.segment<3>(3));
  myRbfV2ReferenceConstant_ = ref.constantWrench;
  myRbfV2ReferenceJointAccel_ = ref.jointAccelerationWrench;
  myRbfV2Diagnostics_.chi = ref.chi;
  myRbfV2Diagnostics_.referenceWrench = ref.constantWrench;  // affine constant until solved
  myRbfV2Dissipation_ = myrbf_wbc::dissipation(ref.chi, myRbfV2Input_);
  myRbfV2WrenchMap_ = matrix_t::Zero(6, 3 * info_.numThreeDofContacts);
  matrix_t activeMap(6, 3 * numContacts_);
  size_t active = 0;
  for (size_t i = 0; i < info_.numThreeDofContacts; ++i) {
    const vector3_t r = data.oMf[info_.endEffectorFrameIndices[i]].translation() - qMeasured_.head<3>();
    myRbfV2WrenchMap_.block<3, 3>(0, 3 * i).setIdentity();
    myRbfV2WrenchMap_.block<3, 3>(3, 3 * i) = myrbf_wbc::skew(r);
    if (contactFlag_[i]) activeMap.middleCols(3 * active++, 3) = myRbfV2WrenchMap_.middleCols(3 * i, 3);
  }
  myRbfV2Diagnostics_.contactWrenchRank = numContacts_ ? activeMap.fullPivLu().rank() : 0;
  // Check the assumed Euler base ordering against the actual full-model foot Jacobians.
  const matrix_t mapped = myRbfV2Transform_.transpose() * myRbfV2WrenchMap_;
  if (!data.M.allFinite() || !data.nle.allFinite() || !j_.allFinite() || !dj_.allFinite() ||
      !ref.chi.allFinite() || !ref.constantWrench.allFinite() || !ref.jointAccelerationWrench.allFinite() ||
      !std::isfinite(myRbfV2Dissipation_) || !mapped.isApprox(j_.leftCols(6).transpose(), 1e-7)) return false;
  return true;
}

Task WbcBase::formulateMyRbfV2ClfTask() const {
  const auto& chi = myRbfV2Diagnostics_.chi;
  // chi^T (w_contact - w_reference + w_external) + dissipation <= 0.
  // w_reference = w0 + T^{-T} Mbj*qdd_j: retain acceleration columns.
  matrix_t d = myrbf_wbc::clfRow(chi, myRbfV2ReferenceJointAccel_, myRbfV2WrenchMap_,
                               info_.generalizedCoordinatesNum, info_.actuatedDofNum);
  vector_t f(1);
  f[0] = myrbf_wbc::clfBound(chi, myRbfV2ReferenceConstant_, myRbfV2Input_);
  return {matrix_t(), vector_t(), d, f};
}

Task WbcBase::formulateMyRbfV2ForceLimitsTask() const {
  const int nf = 3 * info_.numThreeDofContacts;
  matrix_t d = matrix_t::Zero(4 * nf + 2 * numContacts_, numDecisionVars_);
  vector_t f = vector_t::Zero(d.rows());
  int row = 0;
  for (size_t i = 0; i < info_.numThreeDofContacts; ++i) {
    for (int k = 0; k < 3; ++k) {
      const int col = info_.generalizedCoordinatesNum + 3 * i + k;
      const double previous = myRbfV2Input_.previousContactForces.size() == 0 ? 0.0 : myRbfV2Input_.previousContactForces[3 * i + k];
      const double step = myRbfV2Input_.maxContactForceRate[k] * myRbfV2Period_;
      d(row, col) = 1.; f[row++] = myRbfV2Input_.maxAbsContactForce[k];
      d(row, col) = -1.; f[row++] = myRbfV2Input_.maxAbsContactForce[k];
      d(row, col) = 1.; f[row++] = previous + step;
      d(row, col) = -1.; f[row++] = -previous + step;
    }
    if (contactFlag_[i]) {
      const int col = info_.generalizedCoordinatesNum + 3 * i;
      d.block<1, 3>(row, col) = myRbfV2ContactFrame_.col(2).transpose(); f[row++] = myRbfV2Input_.maxNormalForce;
      d.block<1, 3>(row, col) = -myRbfV2ContactFrame_.col(2).transpose(); f[row++] = -myRbfV2Input_.minNormalForce;
    }
  }
  return {matrix_t(), vector_t(), d, f};
}

bool WbcBase::checkMyRbfV2Solution(const vector_t& solution, const Task& constraints) {
  auto& diag = myRbfV2Diagnostics_;
  if (solution.size() != numDecisionVars_ || !solution.allFinite()) {
    diag.status = MyRbfV2WbcStatus::ConstraintViolation;
    return false;
  }
  const vector_t forces = solution.segment(info_.generalizedCoordinatesNum, 3 * info_.numThreeDofContacts);
  diag.finalContactForces = forces;
  diag.finalContactWrench = myRbfV2WrenchMap_ * forces;
  diag.referenceWrench = myRbfV2ReferenceConstant_ + myRbfV2ReferenceJointAccel_ * solution.segment(6, info_.actuatedDofNum);
  diag.referenceWrenchIsFinal = true;
  diag.clfLhs = diag.chi.dot(diag.finalContactWrench);
  diag.clfRhs = diag.chi.dot(diag.referenceWrench - myRbfV2Input_.externalWrenchEstimate) - myRbfV2Dissipation_;
  diag.clfViolation = std::max(0.0, diag.clfLhs - diag.clfRhs);
  diag.maxEqualityViolation = constraints.b_.size() ? (constraints.a_ * solution - constraints.b_).lpNorm<Eigen::Infinity>() : 0.;
  diag.maxInequalityViolation = constraints.f_.size() ? std::max(0.0, (constraints.d_ * solution - constraints.f_).maxCoeff()) : 0.;
  diag.torqueLimitMargin = std::numeric_limits<double>::infinity();
  for (size_t j = 0; j < info_.actuatedDofNum; ++j)
    diag.torqueLimitMargin = std::min(diag.torqueLimitMargin, torqueLimits_[j % 3] - std::abs(solution[numDecisionVars_ - info_.actuatedDofNum + j]));
  diag.frictionMargin = diag.normalForceMargin = std::numeric_limits<double>::infinity();
  diag.contactForceMargin = diag.contactForceRateMargin = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < info_.numThreeDofContacts; ++i) {
    const vector3_t force = forces.segment<3>(3 * i);
    if (contactFlag_[i]) {
      const double normalForce = myRbfV2ContactFrame_.col(2).dot(force);
      diag.frictionMargin = std::min(diag.frictionMargin, myrbf_wbc::frictionMargin(myRbfV2ContactFrame_, force, frictionCoeff_));
      diag.normalForceMargin = std::min(diag.normalForceMargin, std::min(normalForce - myRbfV2Input_.minNormalForce, myRbfV2Input_.maxNormalForce - normalForce));
    }
    const vector3_t previous = myRbfV2Input_.previousContactForces.size() == 0 ? vector3_t::Zero().eval() : myRbfV2Input_.previousContactForces.segment<3>(3 * i).eval();
    diag.contactForceMargin = std::min(diag.contactForceMargin, (myRbfV2Input_.maxAbsContactForce - force.cwiseAbs()).minCoeff());
    diag.contactForceRateMargin = std::min(diag.contactForceRateMargin,
        (myRbfV2Input_.maxContactForceRate * myRbfV2Period_ - (force - previous).cwiseAbs()).minCoeff());
  }
  const double tol = myRbfV2Input_.feasibilityTolerance;
  diag.feasible = std::isfinite(diag.clfLhs) && std::isfinite(diag.clfRhs) &&
      std::isfinite(diag.maxEqualityViolation) && std::isfinite(diag.maxInequalityViolation) &&
      diag.maxEqualityViolation <= tol && diag.maxInequalityViolation <= tol &&
      (!myRbfV2Input_.enforceClfConstraint || diag.clfViolation <= tol);
  diag.status = diag.feasible ? MyRbfV2WbcStatus::Solved : MyRbfV2WbcStatus::ConstraintViolation;
  return diag.feasible;
}

void WbcBase::loadTasksSetting(const std::string& taskFile, bool verbose) {
  // Load task file
  torqueLimits_ = vector_t(info_.actuatedDofNum / 4);
  loadData::loadEigenMatrix(taskFile, "torqueLimitsTask", torqueLimits_);
  if (verbose) {
    std::cerr << "\n #### Torque Limits Task:";
    std::cerr << "\n #### =============================================================================\n";
    std::cerr << "\n #### HAA HFE KFE: " << torqueLimits_.transpose() << "\n";
    std::cerr << " #### =============================================================================\n";
  }
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  std::string prefix = "frictionConeTask.";
  if (verbose) {
    std::cerr << "\n #### Friction Cone Task:";
    std::cerr << "\n #### =============================================================================\n";
  }
  loadData::loadPtreeValue(pt, frictionCoeff_, prefix + "frictionCoefficient", verbose);
  if (verbose) {
    std::cerr << " #### =============================================================================\n";
  }
  prefix = "swingLegTask.";
  if (verbose) {
    std::cerr << "\n #### Swing Leg Task:";
    std::cerr << "\n #### =============================================================================\n";
  }
  loadData::loadPtreeValue(pt, swingKp_, prefix + "kp", verbose);
  loadData::loadPtreeValue(pt, swingKd_, prefix + "kd", verbose);
}

}  // namespace legged
