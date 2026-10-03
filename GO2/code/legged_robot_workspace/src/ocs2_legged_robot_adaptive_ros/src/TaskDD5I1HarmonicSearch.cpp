#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>

#include <Eigen/Eigenvalues>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include "ocs2_legged_robot_adaptive/adaptive/IdentificationReferenceProfile.h"
#include "ocs2_legged_robot_adaptive/adaptive/PayloadInertial10D.h"

namespace adaptive = ocs2::legged_robot_adaptive::adaptive;

namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kMass = 5.213485;
constexpr double kLength = 0.18601075237738277;

struct Metrics {
  double smallestRatio = 0.0;
  double maxAngularSpeed = 0.0;
  double observableFraction = 0.0;
  double jzzColumnRms = 0.0;
  int rank1e3 = 0;
};

Metrics evaluate(const adaptive::IdentificationReferenceProfile& profile) {
  constexpr double dt = 0.04;
  constexpr double duration = 12.0;
  Eigen::Matrix<double, 10, 10> gram =
      Eigen::Matrix<double, 10, 10>::Zero();
  Metrics metrics;
  Eigen::Matrix<double, 10, 10> dPi =
      Eigen::Matrix<double, 10, 10>::Zero();
  dPi.diagonal() << kMass, kMass * kLength, kMass * kLength,
      kMass * kLength, kMass * kLength * kLength,
      kMass * kLength * kLength, kMass * kLength * kLength,
      kMass * kLength * kLength, kMass * kLength * kLength,
      kMass * kLength * kLength;
  Eigen::Matrix<double, 6, 6> dWInverse =
      Eigen::Matrix<double, 6, 6>::Identity();
  dWInverse.diagonal().tail<3>().setConstant(1.0 / kLength);
  const std::size_t count = static_cast<std::size_t>(duration / dt);
  for (std::size_t i = 0; i < count; ++i) {
    const double time = dt * static_cast<double>(i);
    const auto sample = profile.sample(
        adaptive::IdentificationExcitationCandidate::E3Combined, time);
    metrics.maxAngularSpeed =
        std::max(metrics.maxAngularSpeed,
                 sample.angularVelocityWorld.norm());
    adaptive::PayloadInertial10D::ReferencePoint point;
    point.rotationWorldFromBase =
        ocs2::getRotationMatrixFromZyxEulerAngles<double>(sample.eulerZyx);
    point.actualAngularVelocityWorld = sample.angularVelocityWorld;
    point.referenceLinearAccelerationNominalComWorld =
        sample.comAccelerationWorld;
    point.referenceAngularVelocityWorld = sample.angularVelocityWorld;
    point.referenceAngularAccelerationWorld =
        sample.angularAccelerationWorld;
    const Eigen::Matrix<double, 6, 10> block =
        dWInverse * adaptive::PayloadInertial10D::referenceRegressor(point) *
        dPi;
    gram.noalias() += dt * block.transpose() * block;
  }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 10, 10>> solver(gram);
  if (solver.info() != Eigen::Success) {
    throw std::runtime_error("D5I1 harmonic search eigensolver failed.");
  }
  const auto eigenvalues = solver.eigenvalues();
  const double largest = std::sqrt(std::max(0.0, eigenvalues(9)));
  const double smallest = std::sqrt(std::max(0.0, eigenvalues(0)));
  metrics.smallestRatio = largest > 0.0 ? smallest / largest : 0.0;
  for (int i = 0; i < 10; ++i) {
    const double ratio =
        largest > 0.0 ? std::sqrt(std::max(0.0, eigenvalues(i))) / largest
                      : 0.0;
    if (ratio >= 1.0e-3) ++metrics.rank1e3;
  }
  metrics.jzzColumnRms = std::sqrt(gram(9, 9) / (6.0 * duration));

  adaptive::vector10_t piStar;
  piStar << 5.213485, 0.8341576, -0.15640455, 0.46921365,
      0.126921365, 0.025024728, -0.075074184, 0.2756944445,
      0.0140764095, 0.2581573525;
  const Eigen::Matrix<double, 10, 1> theta =
      dPi.diagonal().cwiseInverse().asDiagonal() * piStar;
  Eigen::Matrix<double, 10, 1> projection =
      Eigen::Matrix<double, 10, 1>::Zero();
  for (int i = 0; i < 10; ++i) {
    const double ratio =
        largest > 0.0 ? std::sqrt(std::max(0.0, eigenvalues(i))) / largest
                      : 0.0;
    if (ratio >= 1.0e-3) {
      const auto direction = solver.eigenvectors().col(i);
      projection.noalias() += direction * direction.dot(theta);
    }
  }
  metrics.observableFraction = projection.norm() / theta.norm();
  return metrics;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "usage: task_d_d5i1_harmonic_search OUTPUT.csv\n";
    return 2;
  }
  std::ofstream output(argv[1]);
  if (!output) return 2;
  output << std::setprecision(17)
         << "phase_set,yaw_h,pitch_h,roll_h,max_angular_speed,rank_1e3,"
            "smallest_ratio,pi_star_observable_fraction,jzz_column_rms\n";

  const std::array<adaptive::vector3_t, 3> phaseSets{{
      (adaptive::vector3_t() << 0.0, kPi / 5.0, -kPi / 6.0).finished(),
      (adaptive::vector3_t() << 0.0, 2.0 * kPi / 3.0,
       -2.0 * kPi / 3.0).finished(),
      (adaptive::vector3_t() << kPi / 7.0, -kPi / 4.0,
       kPi / 3.0).finished(),
  }};
  double bestRatio = -std::numeric_limits<double>::infinity();
  std::array<int, 4> best{{-1, -1, -1, -1}};
  Metrics bestMetrics;
  std::size_t evaluated = 0;
  std::size_t speedRejected = 0;
  for (std::size_t phaseIndex = 0; phaseIndex < phaseSets.size();
       ++phaseIndex) {
    for (int yaw = 4; yaw <= 20; ++yaw) {
      for (int pitch = 4; pitch <= 20; ++pitch) {
        for (int roll = 4; roll <= 20; ++roll) {
          adaptive::IdentificationReferenceProfileSettings settings;
          settings.eulerZyxHarmonics = {{yaw, pitch, roll}};
          settings.eulerZyxPhase = phaseSets[phaseIndex];
          const adaptive::IdentificationReferenceProfile profile(settings);
          const Metrics metrics = evaluate(profile);
          if (metrics.maxAngularSpeed > 0.3 + 1.0e-12) {
            ++speedRejected;
            continue;
          }
          ++evaluated;
          output << phaseIndex << ',' << yaw << ',' << pitch << ',' << roll
                 << ',' << metrics.maxAngularSpeed << ',' << metrics.rank1e3
                 << ',' << metrics.smallestRatio << ','
                 << metrics.observableFraction << ','
                 << metrics.jzzColumnRms << '\n';
          if (metrics.smallestRatio > bestRatio) {
            bestRatio = metrics.smallestRatio;
            best = {{static_cast<int>(phaseIndex), yaw, pitch, roll}};
            bestMetrics = metrics;
          }
        }
      }
    }
  }
  std::cout << "evaluated=" << evaluated
            << " speed_rejected=" << speedRejected
            << " best_phase_set=" << best[0]
            << " best_zyx_harmonics=" << best[1] << ',' << best[2] << ','
            << best[3] << " max_omega=" << bestMetrics.maxAngularSpeed
            << " rank_1e3=" << bestMetrics.rank1e3
            << " smallest_ratio=" << bestMetrics.smallestRatio
            << " observable_fraction=" << bestMetrics.observableFraction
            << " jzz_rms=" << bestMetrics.jzzColumnRms << '\n';
  return output ? 0 : 2;
}
