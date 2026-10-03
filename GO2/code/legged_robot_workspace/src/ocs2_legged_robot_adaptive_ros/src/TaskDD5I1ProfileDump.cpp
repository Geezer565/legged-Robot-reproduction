#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include "ocs2_legged_robot_adaptive/adaptive/IdentificationReferenceProfile.h"
#include "ocs2_legged_robot_adaptive/adaptive/PayloadInertial10D.h"

namespace adaptive = ocs2::legged_robot_adaptive::adaptive;

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "usage: task_d_d5i1_profile_dump OUTPUT.csv\n";
    return 2;
  }
  std::ofstream output(argv[1]);
  if (!output) {
    std::cerr << "cannot open output: " << argv[1] << "\n";
    return 2;
  }

  const adaptive::IdentificationReferenceProfile profile;
  output << std::setprecision(17);
  output << "candidate,time,px,py,pz,vx,vy,vz,ax,ay,az,yaw,pitch,roll,"
            "omega_x,omega_y,omega_z,alpha_x,alpha_y,alpha_z";
  for (int row = 0; row < 6; ++row) {
    for (int column = 0; column < 10; ++column) {
      output << ",Y_" << row << "_" << column;
    }
  }
  output << '\n';

  constexpr ocs2::scalar_t dt = 0.01;
  constexpr ocs2::scalar_t duration = 24.0;
  const auto writeCandidate = [&](adaptive::IdentificationExcitationCandidate candidate) {
    const std::size_t count = static_cast<std::size_t>(duration / dt);
    for (std::size_t i = 0; i <= count; ++i) {
      const ocs2::scalar_t time = dt * static_cast<ocs2::scalar_t>(i);
      const auto sample = profile.sample(candidate, time);
      adaptive::PayloadInertial10D::ReferencePoint point;
      point.rotationWorldFromBase =
          ocs2::getRotationMatrixFromZyxEulerAngles<ocs2::scalar_t>(
              sample.eulerZyx);
      point.actualAngularVelocityWorld = sample.angularVelocityWorld;
      point.referenceLinearAccelerationNominalComWorld =
          sample.comAccelerationWorld;
      point.referenceAngularVelocityWorld = sample.angularVelocityWorld;
      point.referenceAngularAccelerationWorld =
          sample.angularAccelerationWorld;
      const auto regressor =
          adaptive::PayloadInertial10D::referenceRegressor(point);
      output << adaptive::identificationExcitationCandidateName(candidate)
             << ',' << time;
      for (const auto& vector : {
               sample.comPositionOffsetWorld, sample.comVelocityWorld,
               sample.comAccelerationWorld, sample.eulerZyx,
               sample.angularVelocityWorld,
               sample.angularAccelerationWorld}) {
        for (Eigen::Index j = 0; j < 3; ++j) output << ',' << vector(j);
      }
      for (int row = 0; row < 6; ++row) {
        for (int column = 0; column < 10; ++column) {
          output << ',' << regressor(row, column);
        }
      }
      output << '\n';
    }
  };

  writeCandidate(adaptive::IdentificationExcitationCandidate::E0LegacyXStep);
  writeCandidate(adaptive::IdentificationExcitationCandidate::E1Translation);
  writeCandidate(adaptive::IdentificationExcitationCandidate::E2Rotation);
  writeCandidate(adaptive::IdentificationExcitationCandidate::E3Combined);
  if (!output) {
    std::cerr << "failed while writing: " << argv[1] << "\n";
    return 2;
  }
  std::cout << "profile_id=" << profile.profileId() << "\n"
            << "samples_per_candidate=2401\n"
            << "dt=0.01\n";
  return 0;
}
