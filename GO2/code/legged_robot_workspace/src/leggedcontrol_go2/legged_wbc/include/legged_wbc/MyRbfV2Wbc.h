#pragma once

#include <Eigen/Core>
#include <cstdint>
#include <cstddef>
#include <limits>

namespace legged {

using WbcVector6 = Eigen::Matrix<double, 6, 1>;
enum class MyRbfV2RobustMode { Sign, Smooth };
enum class MyRbfV2WbcStatus { Disabled, NotSolved, Solved, InvalidInput, Unsupported, SolverFailure, ConstraintViolation };

// Frozen per-cycle input. All wrenches are [F_world, torque_world ABOUT BASE ORIGIN].
// No dependency on legged_interface: the controller copies its frozen snapshot here.
struct MyRbfV2WbcInput {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  bool enabled = false;
  // V2 enforces the sampled CLF row as a hard QP constraint.  The Word-style
  // V3 update keeps the same diagnostic quantities but does not impose that
  // legacy row on the whole-body solution.
  bool enforceClfConstraint = true;
  // V2 may compensate the estimated wrench directly in the whole-body
  // dynamics.  The Word-style V3 uses the estimate only in the MPC model.
  bool injectExternalWrenchInDynamics = true;
  bool v11MatchedAcceleration = false;
  WbcVector6 v11LeftHandBodyComWrench = WbcVector6::Zero();
  std::uint64_t snapshotSequenceId = 0;
  WbcVector6 externalWrenchEstimate = WbcVector6::Zero();
  WbcVector6 robustBound = WbcVector6::Zero();
  MyRbfV2RobustMode robustMode = MyRbfV2RobustMode::Sign;
  // Damping and robust bounds operate on world power-conjugate chi.
  WbcVector6 damping = WbcVector6::Ones();
  // Lambda operates on local generalized [position, ZYX Euler] coordinate error.
  WbcVector6 lambda = WbcVector6::Ones();
  double smoothWidth = 0.01;
  double feasibilityTolerance = 1e-6;
  double minEulerCosPitch = 1e-3;
  double minNormalForce = 0.0;
  double maxNormalForce = 350.0;
  // One frozen common plane for all stance contacts; no mixed/stepping terrain claim.
  Eigen::Vector3d surfaceNormal = Eigen::Vector3d::UnitZ();
  // Component amplitude and rate limits remain in WORLD axes, including on slopes.
  Eigen::Vector3d maxAbsContactForce = Eigen::Vector3d::Constant(350.0);
  Eigen::Vector3d maxContactForceRate = Eigen::Vector3d::Constant(10000.0);  // N/s
  // Required final forces from the preceding applied cycle (3 per foot). Empty means
  // a zero-force starting boundary. Rate bounds include touchdown AND liftoff.
  Eigen::VectorXd previousContactForces;
};

struct MyRbfV2WbcDiagnostics {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  bool enabled = false;
  bool feasible = false;
  bool desiredAccelerationInitialized = false;
  bool referenceWrenchIsFinal = false;
  std::size_t contactMode = 0;
  std::uint64_t snapshotSequenceId = 0;
  MyRbfV2WbcStatus status = MyRbfV2WbcStatus::Disabled;
  int solverReturnCode = 0;
  int contactWrenchRank = 0;
  WbcVector6 chi = WbcVector6::Zero();
  WbcVector6 externalWrenchEstimate = WbcVector6::Zero();
  WbcVector6 referenceWrench = WbcVector6::Zero();
  WbcVector6 finalContactWrench = WbcVector6::Zero();
  Eigen::VectorXd finalContactForces;
  double clfLhs = std::numeric_limits<double>::quiet_NaN();
  double clfRhs = std::numeric_limits<double>::quiet_NaN();
  double clfViolation = std::numeric_limits<double>::infinity();
  double maxEqualityViolation = std::numeric_limits<double>::infinity();
  double maxInequalityViolation = std::numeric_limits<double>::infinity();
  double torqueLimitMargin = -std::numeric_limits<double>::infinity();
  double frictionMargin = -std::numeric_limits<double>::infinity();
  double normalForceMargin = -std::numeric_limits<double>::infinity();
  double contactForceMargin = -std::numeric_limits<double>::infinity();
  double contactForceRateMargin = -std::numeric_limits<double>::infinity();
};

}  // namespace legged
