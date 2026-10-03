#pragma once

#include "legged_wbc/MyRbfV2Wbc.h"
#include <Eigen/LU>
#include <Eigen/Geometry>
#include <cmath>

namespace legged {
namespace myrbf_wbc {

// q=[p_world,yaw,pitch,roll,joints], v=qdot; omega_world=E(q)*EulerDot.
inline Eigen::Matrix<double, 6, 6> baseTransform(const Eigen::Vector3d& zyx) {
  const double cz = std::cos(zyx[0]), sz = std::sin(zyx[0]);
  const double cy = std::cos(zyx[1]), sy = std::sin(zyx[1]);
  Eigen::Matrix<double, 6, 6> t = Eigen::Matrix<double, 6, 6>::Identity();
  t.bottomRightCorner<3, 3>() << 0., -sz, cz * cy, 0., cz, sz * cy, 1., 0., -sy;
  return t;
}

inline Eigen::Matrix3d skew(const Eigen::Vector3d& r) {
  Eigen::Matrix3d s;
  s << 0., -r.z(), r.y(), r.z(), 0., -r.x(), -r.y(), r.x(), 0.;
  return s;
}

// Deterministic right-handed [t1,t2,n]. UnitZ reproduces the existing flat axes.
// Caller validates that the normal has a finite, nonzero norm.
inline Eigen::Matrix3d surfaceFrame(Eigen::Vector3d normal) {
  normal /= normal.stableNorm();
  const Eigen::Vector3d seed = std::abs(normal.z()) > 0.9 ? Eigen::Vector3d::UnitX().eval() : Eigen::Vector3d::UnitZ().eval();
  const Eigen::Vector3d tangent = (seed - normal.dot(seed) * normal).normalized();
  Eigen::Matrix3d frame;
  frame.col(0) = tangent; frame.col(1) = normal.cross(tangent); frame.col(2) = normal;
  return frame;
}

inline Eigen::Matrix<double, 5, 3> contactPyramid(const Eigen::Matrix3d& frame, double friction) {
  Eigen::Matrix<double, 5, 3> pyramid;
  const Eigen::RowVector3d n = frame.col(2).transpose(), t1 = frame.col(0).transpose(), t2 = frame.col(1).transpose();
  const double mu = friction / std::sqrt(2.);
  pyramid.row(0) = -n;
  pyramid.row(1) = t1 - mu * n; pyramid.row(2) = -t1 - mu * n;
  pyramid.row(3) = t2 - mu * n; pyramid.row(4) = -t2 - mu * n;
  return pyramid;
}

inline double frictionMargin(const Eigen::Matrix3d& frame, const Eigen::Vector3d& force, double friction) {
  const Eigen::Vector3d local = frame.transpose() * force;
  return friction * local.z() - std::hypot(local.x(), local.y());
}

// Local wrapped Euler subtraction; not a global SO(3) tracking claim.
inline WbcVector6 localError(const WbcVector6& q, const WbcVector6& qd) {
  WbcVector6 e = q - qd;
  for (int i = 3; i < 6; ++i) e[i] = std::atan2(std::sin(e[i]), std::cos(e[i]));
  return e;
}

// The desired trajectory is supplied as immutable MPC policy snapshots. A
// finite difference is meaningful only within one snapshot; differencing the
// last sample of the old policy and the first sample of a new policy creates a
// fictitious acceleration impulse at every policy swap.
inline WbcVector6 desiredAcceleration(const WbcVector6& currentVelocity,
                                      const Eigen::VectorXd& previousVelocity,
                                      double period, std::uint64_t snapshotSequenceId,
                                      std::uint64_t previousSnapshotSequenceId,
                                      bool& initialized) {
  initialized = previousVelocity.size() == 6 && period > 0. && std::isfinite(period) &&
                snapshotSequenceId == previousSnapshotSequenceId;
  if (!initialized) return WbcVector6::Zero();
  return (currentVelocity - previousVelocity).eval() / period;
}

inline double dissipation(const WbcVector6& chi, const MyRbfV2WbcInput& input) {
  double result = chi.dot(input.damping.cwiseProduct(chi));
  for (int i = 0; i < 6; ++i) {
    result += input.robustBound[i] *
              (input.robustMode == MyRbfV2RobustMode::Sign
                   ? std::abs(chi[i])
                   : chi[i] * (chi[i] / std::hypot(chi[i], input.smoothWidth)));
  }
  return result;
}

struct Reference {
  WbcVector6 chi;
  WbcVector6 constantWrench;
  Eigen::MatrixXd jointAccelerationWrench;
};

inline Eigen::RowVectorXd clfRow(const WbcVector6& chi, const Eigen::MatrixXd& referenceJointAcceleration,
                                 const Eigen::MatrixXd& wrenchMap, int generalizedDof, int actuatedDof) {
  Eigen::RowVectorXd row = Eigen::RowVectorXd::Zero(generalizedDof + wrenchMap.cols() + actuatedDof);
  row.segment(6, actuatedDof) = -chi.transpose() * referenceJointAcceleration;
  row.segment(generalizedDof, wrenchMap.cols()) = chi.transpose() * wrenchMap;
  return row;
}

inline double clfBound(const WbcVector6& chi, const WbcVector6& referenceConstant, const MyRbfV2WbcInput& input) {
  return chi.dot(referenceConstant - input.externalWrenchEstimate) - dissipation(chi, input);
}

// Full Euler-coordinate dynamics at ACTUAL (q,v): Y=M*ar+C(q,v)*vr+g.
// vr_joint=v_joint and ar_joint=QP joint acceleration give s_joint=0.
// Retain Mbj*qdd_j, Cbj*v_j and the configuration dependence of Mbb.
// rnea(q,vr,ar) uses C(q,vr)*vr and must NOT replace this reference.
inline Reference referenceFromBias(const Eigen::MatrixXd& mass, const Eigen::VectorXd& referenceBias,
                           const Eigen::VectorXd& q,
                           const Eigen::VectorXd& v, const WbcVector6& qd,
                           const WbcVector6& vd, const WbcVector6& ad,
                           const WbcVector6& lambda) {
  const auto t = baseTransform(q.segment<3>(3));
  Eigen::VectorXd vr = v;
  vr.head<6>() = vd - lambda.cwiseProduct(localError(q.head<6>(), qd));
  const WbcVector6 ar = ad - lambda.cwiseProduct(v.head<6>() - vd);
  Reference result;
  result.chi = t * (v.head<6>() - vr.head<6>());
  const WbcVector6 y0 = mass.topLeftCorner<6, 6>() * ar + referenceBias.head<6>();
  result.constantWrench = t.transpose().fullPivLu().solve(y0);
  result.jointAccelerationWrench = t.transpose().fullPivLu().solve(mass.topRows<6>().rightCols(v.size() - 6));
  return result;
}

inline Reference reference(const Eigen::MatrixXd& mass, const Eigen::MatrixXd& coriolis,
                           const Eigen::VectorXd& gravity, const Eigen::VectorXd& q,
                           const Eigen::VectorXd& v, const WbcVector6& qd,
                           const WbcVector6& vd, const WbcVector6& ad, const WbcVector6& lambda) {
  Eigen::VectorXd vr = v;
  vr.head<6>() = vd - lambda.cwiseProduct(localError(q.head<6>(), qd));
  return referenceFromBias(mass, coriolis * vr + gravity, q, v, qd, vd, ad, lambda);
}

}  // namespace myrbf_wbc
}  // namespace legged
