#pragma once

#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/rnea.hpp>

namespace legged {
namespace myrbf_wbc {

// Exact canonical Christoffel C_Gamma(q,v)*vr + g for rigid-body dynamics in
// the local Euler chart v=qdot. nle(q,u)=g+B_q(u,u), B_q symmetric bilinear:
// C_Gamma(q,v)*vr = (nle(q,v+vr)-nle(q,v-vr))/4.
// This is NOT rnea(q,vr,ar). Pinocchio 2.5.1 computeCoriolisMatrix fails the
// Cv=nle-g and Mdot=C+C^T identities for the Translation+SphericalZYX
// composite used here; retain this tested coordinate-consistent definition.
inline Eigen::VectorXd eulerReferenceBias(const pinocchio::Model& model, pinocchio::Data& data,
                                        const Eigen::VectorXd& q, const Eigen::VectorXd& v,
                                        const Eigen::VectorXd& vr) {
  const Eigen::VectorXd plus = pinocchio::nonLinearEffects(model, data, q, (v + vr).eval());
  const Eigen::VectorXd minus = pinocchio::nonLinearEffects(model, data, q, (v - vr).eval());
  const Eigen::VectorXd gravity = pinocchio::computeGeneralizedGravity(model, data, q);
  return 0.25 * (plus - minus) + gravity;
}

}  // namespace myrbf_wbc
}  // namespace legged
