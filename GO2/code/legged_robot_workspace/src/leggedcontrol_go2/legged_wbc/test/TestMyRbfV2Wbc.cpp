#include "legged_wbc/MyRbfV2WbcMath.h"
#include <gtest/gtest.h>
#include <qpOASES.hpp>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <array>
#include <random>

namespace legged {
namespace {
using Mat = Eigen::MatrixXd;
using Vec = Eigen::VectorXd;
using RowMat = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

TEST(MyRbfV2WbcMath, EulerPowerAndAngularVelocityAtNonzeroAttitude) {
  std::mt19937 gen(941);
  std::uniform_real_distribution<double> dist(-1., 1.);
  for (int trial = 0; trial < 1000; ++trial) {
    Eigen::Vector3d angles(dist(gen), 0.9 * dist(gen), dist(gen));
    const auto t = myrbf_wbc::baseTransform(angles);
    WbcVector6 s, w;
    for (int i = 0; i < 6; ++i) { s[i] = dist(gen); w[i] = 100. * dist(gen); }
    EXPECT_NEAR((t * s).dot(w), s.dot(t.transpose() * w), 2e-12);
    const auto rotation = [](const Eigen::Vector3d& a) -> Eigen::Matrix3d {
      return (Eigen::AngleAxisd(a[0], Eigen::Vector3d::UnitZ()) *
              Eigen::AngleAxisd(a[1], Eigen::Vector3d::UnitY()) *
              Eigen::AngleAxisd(a[2], Eigen::Vector3d::UnitX())).toRotationMatrix();
    };
    const double h = 1e-6;
    const Eigen::Matrix3d rdot = (rotation(angles + h * s.tail<3>()) - rotation(angles - h * s.tail<3>())) / (2. * h);
    const Eigen::Matrix3d omegaSkew = rdot * rotation(angles).transpose();
    const Eigen::Vector3d omega(omegaSkew(2, 1), omegaSkew(0, 2), omegaSkew(1, 0));
    EXPECT_LT((omega - t.bottomRightCorner<3, 3>() * s.tail<3>()).norm(), 5e-10);
  }
}

TEST(MyRbfV2WbcMath, WrappedLocalErrorAndRobustModes) {
  WbcVector6 q = WbcVector6::Zero(), qd = q;
  q[3] = std::acos(-1.) - 0.01;
  qd[3] = -std::acos(-1.) + 0.01;
  EXPECT_NEAR(myrbf_wbc::localError(q, qd)[3], -0.02, 1e-14);
  MyRbfV2WbcInput input;
  input.robustBound.setConstant(2.);
  WbcVector6 chi; chi << -1., 0., 0.2, -0.3, 0.5, -0.9;
  EXPECT_DOUBLE_EQ(myrbf_wbc::dissipation(WbcVector6::Zero(), input), 0.);
  const double strict = myrbf_wbc::dissipation(chi, input);
  EXPECT_NEAR(strict, chi.squaredNorm() + 2. * chi.lpNorm<1>(), 1e-14);
  input.robustMode = MyRbfV2RobustMode::Smooth;
  const double smooth = myrbf_wbc::dissipation(chi, input);
  EXPECT_LT(smooth, strict);
  EXPECT_GE(smooth, chi.squaredNorm());
}

TEST(MyRbfV2WbcMath, DesiredAccelerationResetsAtMpcPolicySwap) {
  WbcVector6 previous = WbcVector6::Zero();
  WbcVector6 current = WbcVector6::Constant(0.01);
  bool initialized = false;
  const auto withinPolicy = myrbf_wbc::desiredAcceleration(current, previous, 0.001, 41, 41, initialized);
  EXPECT_TRUE(initialized);
  EXPECT_TRUE(withinPolicy.isApprox(WbcVector6::Constant(10.), 1e-14));

  current.setConstant(2.0);
  const auto afterSwap = myrbf_wbc::desiredAcceleration(current, previous, 0.001, 42, 41, initialized);
  EXPECT_FALSE(initialized);
  EXPECT_TRUE(afterSwap.isZero(0.));

  Eigen::VectorXd empty;
  const auto firstSample = myrbf_wbc::desiredAcceleration(current, empty, 0.001, 42, 42, initialized);
  EXPECT_FALSE(initialized);
  EXPECT_TRUE(firstSample.isZero(0.));
}

TEST(MyRbfV2WbcMath, FullReferenceRetainsMovingLegsAndActualCoriolis) {
  std::srand(138);
  for (int trial = 0; trial < 100; ++trial) {
    const Mat factor = Mat::Random(18, 18);
    const Mat mass = factor.transpose() * factor + Mat::Identity(18, 18);
    const Mat coriolis = Mat::Random(18, 18);
    const Vec gravity = Vec::Random(18), q = Vec::Random(18), v = Vec::Random(18);
    const WbcVector6 qd = WbcVector6::Random(), vd = WbcVector6::Random(), ad = WbcVector6::Random();
    const WbcVector6 lambda = WbcVector6::Constant(1.7);
    const Vec aj = Vec::Random(12);
    const auto ref = myrbf_wbc::reference(mass, coriolis, gravity, q, v, qd, vd, ad, lambda);
    Vec vr = v, ar = Vec::Zero(18);
    vr.head<6>() = vd - lambda.cwiseProduct(myrbf_wbc::localError(q.head<6>(), qd));
    ar.head<6>() = ad - lambda.cwiseProduct(v.head<6>() - vd);
    ar.tail(12) = aj;
    const Vec y = mass * ar + coriolis * vr + gravity;
    const auto t = myrbf_wbc::baseTransform(q.segment<3>(3));
    const WbcVector6 reconstructed = ref.constantWrench + ref.jointAccelerationWrench * aj;
    EXPECT_LT((t.transpose() * reconstructed - y.head<6>()).norm(), 2e-12);
    EXPECT_NEAR(ref.chi.dot(reconstructed), (v.head<6>() - vr.head<6>()).dot(y.head<6>()), 2e-11);
    EXPECT_GT((ref.jointAccelerationWrench * aj).norm(), 1e-3);
    Mat map = Mat::Random(6, 12);
    Vec solution = Vec::Random(42);
    solution.segment(6, 12) = aj;
    MyRbfV2WbcInput input;
    input.externalWrenchEstimate = WbcVector6::Random();
    input.robustBound.setConstant(0.2);
    const auto row = myrbf_wbc::clfRow(ref.chi, ref.jointAccelerationWrench, map, 18, 12);
    const double bound = myrbf_wbc::clfBound(ref.chi, ref.constantWrench, input);
    const double direct = ref.chi.dot(map * solution.segment(18, 12) - reconstructed + input.externalWrenchEstimate)
                        + myrbf_wbc::dissipation(ref.chi, input);
    EXPECT_NEAR(row.dot(solution) - bound, direct, 2e-11);
  }
}

TEST(MyRbfV2WbcMath, CommonPlaneFrameAndConservativeFrictionRotateTogether) {
  EXPECT_TRUE(myrbf_wbc::surfaceFrame(Eigen::Vector3d::UnitZ()).isApprox(Eigen::Matrix3d::Identity(), 0.));
  const double angle = 5. * std::acos(-1.) / 180.;
  const Eigen::Vector3d n(-std::sin(angle), 0., std::cos(angle));
  const auto frame = myrbf_wbc::surfaceFrame(7. * n);
  EXPECT_TRUE((frame.transpose() * frame).isApprox(Eigen::Matrix3d::Identity(), 1e-14));
  EXPECT_GT(frame.determinant(), 0.999999999);
  const auto pyramid = myrbf_wbc::contactPyramid(frame, 0.6);
  const Eigen::Vector3d force = frame * Eigen::Vector3d(10., -10., 50.);
  EXPECT_LE((pyramid * force).maxCoeff(), 0.);
  EXPECT_NEAR(frame.col(2).dot(force), 50., 1e-12);
  EXPECT_NEAR(myrbf_wbc::frictionMargin(frame, force, 0.6), 30. - std::sqrt(200.), 1e-12);
  const Eigen::Vector3d harmful = frame * Eigen::Vector3d(29., 29., 50.);
  EXPECT_GT((pyramid * harmful).maxCoeff(), 0.);
  EXPECT_LT(myrbf_wbc::frictionMargin(frame, harmful, 0.6), 0.);
}

// Representative 18-DoF articulated plant. All 42 decision variables are solved
// together: full dynamics, stance kinematics, swing zero force, torque limits,
// friction pyramid, normal/axis/rate limits, and the production CLF row.
// This numerical fixture is not a substitute for the separate Pinocchio-model test.
struct PlantQp {
  static constexpr int n = 42;
  std::array<bool, 4> stance;
  Mat mass = Mat::Identity(18, 18), jac = Mat::Zero(12, 18), map = Mat::Zero(6, 12);
  Vec gravity = Vec::Zero(18);
  MyRbfV2WbcInput input;
  myrbf_wbc::Reference ref;
  RowMat a, h = RowMat::Identity(n, n);
  Vec lo, hi, gradient = Vec::Zero(n), solution;
  int equalityRows = 0;
  qpOASES::returnValue status = qpOASES::RET_QP_NOT_SOLVED;
  double friction = 0.6, torqueLimit = 500.;
  const double period = 0.01;

  explicit PlantQp(std::array<bool, 4> contacts) : stance(contacts) {
    input.enabled = true;
    input.previousContactForces = Vec::Zero(12);
    input.maxContactForceRate.setConstant(100000.);
    mass.topLeftCorner<3, 3>() *= 10.;
    gravity[2] = 98.1;
    const std::array<Eigen::Vector3d, 4> feet{{{0.3, 0.2, -0.4}, {0.3, -0.2, -0.4},
                                            {-0.3, 0.2, -0.4}, {-0.3, -0.2, -0.4}}};
    for (int i = 0; i < 4; ++i) {
      map.block<3, 3>(0, 3 * i).setIdentity();
      map.block<3, 3>(3, 3 * i) = myrbf_wbc::skew(feet[i]);
    }
    jac.leftCols(6) = map.transpose() * myrbf_wbc::baseTransform(Eigen::Vector3d::Zero());
    jac.rightCols(12).setIdentity();
  }

  Mat activeMap() const {
    int count = 0;
    for (bool c : stance) if (c) ++count;
    Mat result(6, 3 * count);
    count = 0;
    for (int i = 0; i < 4; ++i) if (stance[i]) result.middleCols(3 * count++, 3) = map.middleCols(3 * i, 3);
    return result;
  }

  void assemble(const WbcVector6& chi) {
    Vec q = Vec::Zero(18), v = Vec::Zero(18);
    const auto t = myrbf_wbc::baseTransform(Eigen::Vector3d::Zero());
    const WbcVector6 qd = -t.fullPivLu().solve(chi);
    ref = myrbf_wbc::reference(mass, Mat::Zero(18, 18), gravity, q, v, qd,
                              WbcVector6::Zero(), WbcVector6::Zero(), input.lambda);
    std::vector<Eigen::RowVectorXd> rows;
    std::vector<double> lower, upper;
    const auto add = [&](const Eigen::RowVectorXd& row, double lb, double ub) {
      rows.push_back(row); lower.push_back(lb); upper.push_back(ub);
    };
    Mat eom = Mat::Zero(18, n);
    eom.leftCols(18) = mass;
    eom.middleCols(18, 12) = -jac.transpose();
    eom.block(6, 30, 12, 12) = -Mat::Identity(12, 12);
    Vec rhs = -gravity;
    rhs.head<6>() += t.transpose() * input.externalWrenchEstimate;
    for (int i = 0; i < 18; ++i) add(eom.row(i), rhs[i], rhs[i]);
    for (int i = 0; i < 4; ++i) for (int k = 0; k < 3; ++k) {
      Eigen::RowVectorXd row = Eigen::RowVectorXd::Zero(n);
      if (stance[i]) row.head(18) = jac.row(3 * i + k);
      else row[18 + 3 * i + k] = 1.;
      add(row, 0., 0.);
    }
    equalityRows = rows.size();
    for (int j = 0; j < 12; ++j) {
      Eigen::RowVectorXd row = Eigen::RowVectorXd::Zero(n); row[30 + j] = 1.;
      add(row, -torqueLimit, torqueLimit);
    }
    for (int i = 0; i < 4; ++i) {
      for (int k = 0; k < 3; ++k) {
        Eigen::RowVectorXd row = Eigen::RowVectorXd::Zero(n); row[18 + 3 * i + k] = 1.;
        add(row, -input.maxAbsContactForce[k], input.maxAbsContactForce[k]);
        const double previous = input.previousContactForces[3 * i + k], step = input.maxContactForceRate[k] * period;
        add(row, previous - step, previous + step);
      }
      if (stance[i]) {
        const auto frame = myrbf_wbc::surfaceFrame(input.surfaceNormal);
        Eigen::RowVectorXd row = Eigen::RowVectorXd::Zero(n);
        row.segment<3>(18 + 3 * i) = frame.col(2).transpose();
        add(row, input.minNormalForce, input.maxNormalForce);
        const auto pyramid = myrbf_wbc::contactPyramid(frame, friction);
        for (int k = 0; k < 5; ++k) {
          row.setZero(); row.segment<3>(18 + 3 * i) = pyramid.row(k);
          add(row, -qpOASES::INFTY, 0.);
        }
      }
    }
    add(myrbf_wbc::clfRow(ref.chi, ref.jointAccelerationWrench, map, 18, 12),
        -qpOASES::INFTY, myrbf_wbc::clfBound(ref.chi, ref.constantWrench, input));
    a.resize(rows.size(), n); lo.resize(rows.size()); hi.resize(rows.size());
    for (size_t i = 0; i < rows.size(); ++i) { a.row(i) = rows[i]; lo[i] = lower[i]; hi[i] = upper[i]; }
  }

  bool solve() {
    qpOASES::QProblem qp(n, a.rows());
    qpOASES::Options options; options.setToMPC(); options.printLevel = qpOASES::PL_NONE;
    options.enableEqualities = qpOASES::BT_TRUE; qp.setOptions(options);
    int nwsr = 200;
    status = qp.init(h.data(), gradient.data(), a.data(), nullptr, nullptr, lo.data(), hi.data(), nwsr);
    solution.resize(0);
    if (status != qpOASES::SUCCESSFUL_RETURN) return false;
    Vec candidate(n);
    if (qp.getPrimalSolution(candidate.data()) != qpOASES::SUCCESSFUL_RETURN) return false;
    if (!candidate.allFinite() || (a * candidate - hi).maxCoeff() > 1e-6 || (lo - a * candidate).maxCoeff() > 1e-6) return false;
    solution = candidate;
    return true;
  }

  void checkFinal() const {
    ASSERT_EQ(solution.size(), n);
    EXPECT_LE((a * solution - hi).maxCoeff(), 1e-6);
    EXPECT_LE((lo - a * solution).maxCoeff(), 1e-6);
    const WbcVector6 finalWrench = map * solution.segment(18, 12);
    for (int i = 0; i < 4; ++i) if (stance[i]) {
      const Eigen::Vector3d force = solution.segment<3>(18 + 3 * i);
      EXPECT_GE(myrbf_wbc::frictionMargin(myrbf_wbc::surfaceFrame(input.surfaceNormal), force, friction), -1e-6);
    }
    const WbcVector6 reference = ref.constantWrench + ref.jointAccelerationWrench * solution.segment(6, 12);
    EXPECT_LE(ref.chi.dot(finalWrench - reference + input.externalWrenchEstimate) + myrbf_wbc::dissipation(ref.chi, input), 1e-6);
  }
};

constexpr int PlantQp::n;

TEST(MyRbfV2WbcQp, FourFeetFeasibleAllBoundsAndHarmfulForceDetected) {
  PlantQp p(std::array<bool, 4>{{true, true, true, true}});
  WbcVector6 chi = WbcVector6::Zero(); chi[0] = 0.1;
  p.input.robustBound.setConstant(0.3);
  p.input.externalWrenchEstimate[0] = 3.;
  p.assemble(chi);
  ASSERT_TRUE(p.solve()) << p.status;
  EXPECT_EQ(p.activeMap().fullPivLu().rank(), 6);
  p.checkFinal();
  Vec harmful = p.solution;
  harmful[18] += 100.;
  EXPECT_GT(p.a.row(p.a.rows() - 1).dot(harmful) - p.hi.tail(1)[0], 1.);
}

TEST(MyRbfV2WbcQp, FourFeetInfeasibleHardClfHasNoSlack) {
  PlantQp p(std::array<bool, 4>{{true, true, true, true}});
  WbcVector6 chi = WbcVector6::Zero(); chi[0] = 0.1;
  p.input.robustBound[0] = 1e6;
  p.assemble(chi);
  EXPECT_FALSE(p.solve());
  EXPECT_EQ(p.solution.size(), 0);
}

TEST(MyRbfV2WbcQp, DiagonalTwoFeetFeasibleDespiteRankFive) {
  PlantQp p(std::array<bool, 4>{{true, false, false, true}});
  WbcVector6 chi = WbcVector6::Zero(); chi[0] = 0.1;
  p.assemble(chi);
  ASSERT_TRUE(p.solve()) << p.status;
  EXPECT_EQ(p.activeMap().fullPivLu().rank(), 5);
  p.checkFinal();
  EXPECT_LT(p.solution.segment<3>(21).norm(), 1e-6);
  EXPECT_LT(p.solution.segment<3>(24).norm(), 1e-6);
}

TEST(MyRbfV2WbcQp, DiagonalTwoFeetNullWrenchDirectionIsInfeasible) {
  PlantQp p(std::array<bool, 4>{{true, false, false, true}});
  Eigen::JacobiSVD<Mat> svd(p.activeMap(), Eigen::ComputeFullU);
  const WbcVector6 chi = 0.1 * svd.matrixU().col(5);
  EXPECT_LT((chi.transpose() * p.activeMap()).norm(), 1e-14);
  p.assemble(chi);
  EXPECT_FALSE(p.solve());
  EXPECT_EQ(p.solution.size(), 0);
}

TEST(MyRbfV2WbcQp, RateLimitIncludesLiftoffAndRejectsImpossibleZeroForce) {
  PlantQp p(std::array<bool, 4>{{true, false, false, true}});
  p.input.previousContactForces[5] = 50.;
  p.input.maxContactForceRate.setConstant(1.);
  p.assemble(WbcVector6::Zero());
  EXPECT_FALSE(p.solve());
  EXPECT_EQ(p.solution.size(), 0);
}

TEST(MyRbfV2WbcQp, JointCouplingInFinalReference) {
  PlantQp p(std::array<bool, 4>{{true, true, true, true}});
  p.mass(0, 6) = p.mass(6, 0) = 0.25;
  WbcVector6 chi = WbcVector6::Zero(); chi[0] = 0.2;
  p.input.externalWrenchEstimate[0] = 5.;
  p.assemble(chi);
  ASSERT_TRUE(p.solve()) << p.status;
  p.checkFinal();
  EXPECT_GT((p.ref.jointAccelerationWrench * p.solution.segment(6, 12)).norm(), 1e-5);
}

TEST(MyRbfV2WbcQp, FiveDegreeCommonPlaneHasFeasibleRotatedForces) {
  PlantQp p(std::array<bool, 4>{{true, true, true, true}});
  const double angle = -5. * std::acos(-1.) / 180.;
  const Eigen::Matrix3d rotation = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitY()).toRotationMatrix();
  p.input.surfaceNormal = rotation * Eigen::Vector3d::UnitZ();
  p.input.minNormalForce = 5.;
  for (int i = 0; i < 4; ++i) {
    const Eigen::Vector3d foot(p.map(5, 3 * i + 1), p.map(3, 3 * i + 2), p.map(4, 3 * i));
    p.map.block<3, 3>(3, 3 * i) = myrbf_wbc::skew(rotation * foot);
  }
  p.jac.leftCols(6) = p.map.transpose() * myrbf_wbc::baseTransform(Eigen::Vector3d::Zero());
  WbcVector6 chi = WbcVector6::Zero(); chi[0] = 0.1;
  p.assemble(chi);
  ASSERT_TRUE(p.solve()) << p.status;
  p.checkFinal();
}

}  // namespace
}  // namespace legged
