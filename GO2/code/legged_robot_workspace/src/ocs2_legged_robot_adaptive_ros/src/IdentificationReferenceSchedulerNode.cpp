#include <cmath>
#include <cstdint>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/rate.h>
#include <ros/spinner.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>

#include <ocs2_legged_robot/LeggedRobotInterface.h>
#include <ocs2_msgs/mode_schedule.h>
#include <ocs2_msgs/mpc_observation.h>
#include <ocs2_msgs/mpc_target_trajectories.h>
#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>

#include "ocs2_legged_robot_adaptive/adaptive/AdaptiveReferenceStateEncoder.h"
#include "ocs2_legged_robot_adaptive/adaptive/AdaptiveStateAdapter.h"
#include "ocs2_legged_robot_adaptive/adaptive/IdentificationReferenceProfile.h"

namespace {

using ocs2::scalar_t;
using ocs2::vector_t;
using ocs2::legged_robot_adaptive::adaptive::AdaptiveReferenceStateEncoder;
using ocs2::legged_robot_adaptive::adaptive::AdaptiveStateAdapter;
using ocs2::legged_robot_adaptive::adaptive::IdentificationExcitationCandidate;
using ocs2::legged_robot_adaptive::adaptive::IdentificationReferenceProfile;
using ocs2::legged_robot_adaptive::adaptive::IdentificationReferenceSchedule;
using ocs2::legged_robot_adaptive::adaptive::IdentificationReferenceScheduleSettings;
using ocs2::legged_robot_adaptive::adaptive::identificationExcitationCandidateName;

constexpr int kStance = 15;
const std::vector<double> kStaticWalkSwitchingTimes{0.0, 0.3, 0.6, 0.9,
                                                    1.2};
const std::vector<int8_t> kStaticWalkModes{13, 7, 14, 11};

class ObservationBuffer {
 public:
  void callback(const ocs2_msgs::mpc_observation::ConstPtr& message) {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_ = *message;
    available_ = true;
  }

  bool latest(ocs2_msgs::mpc_observation& output) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!available_) return false;
    output = latest_;
    return true;
  }

 private:
  mutable std::mutex mutex_;
  bool available_ = false;
  ocs2_msgs::mpc_observation latest_;
};

bool waitForSubscribers(const ros::Publisher& publisher, uint32_t count,
                        double timeoutSeconds, const std::string& topic) {
  const auto deadline = ros::WallTime::now() + ros::WallDuration(timeoutSeconds);
  ros::WallRate rate(100.0);
  while (ros::ok() && publisher.getNumSubscribers() < count &&
         ros::WallTime::now() < deadline) {
    rate.sleep();
  }
  if (publisher.getNumSubscribers() < count) {
    ROS_ERROR_STREAM("Timed out waiting for " << count << " subscribers on "
                                               << topic << "; got "
                                               << publisher.getNumSubscribers());
    return false;
  }
  return true;
}

IdentificationExcitationCandidate parseCandidate(const std::string& name) {
  if (name == "E2" || name == "E2_rotation") {
    return IdentificationExcitationCandidate::E2Rotation;
  }
  if (name == "E3" || name == "E3_combined") {
    return IdentificationExcitationCandidate::E3Combined;
  }
  throw std::invalid_argument(
      "D5I2 online smoke only admits E2_rotation or E3_combined.");
}

template <typename Derived>
void append(std::vector<double>& output,
            const Eigen::MatrixBase<Derived>& values) {
  for (Eigen::Index index = 0; index < values.size(); ++index) {
    output.push_back(values(index));
  }
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "identification_reference_scheduler");
  ros::NodeHandle nodeHandle;
  ros::NodeHandle privateNodeHandle("~");

  std::string taskFile, urdfFile, referenceFile;
  if (!nodeHandle.getParam("/taskFile", taskFile) ||
      !nodeHandle.getParam("/urdfFile", urdfFile) ||
      !nodeHandle.getParam("/referenceFile", referenceFile)) {
    ROS_ERROR("Missing /taskFile, /urdfFile, or /referenceFile");
    return 1;
  }

  std::string candidateName = "E3_combined";
  double knotDt = 0.01;
  IdentificationReferenceScheduleSettings scheduleSettings;
  privateNodeHandle.param("candidate", candidateName, candidateName);
  privateNodeHandle.param("knot_dt", knotDt, knotDt);
  privateNodeHandle.param("profile_start_time",
                          scheduleSettings.profileStartTime,
                          scheduleSettings.profileStartTime);
  privateNodeHandle.param("ramp_in_duration",
                          scheduleSettings.rampInDuration,
                          scheduleSettings.rampInDuration);
  privateNodeHandle.param("steady_duration",
                          scheduleSettings.steadyDuration,
                          scheduleSettings.steadyDuration);
  privateNodeHandle.param("ramp_out_duration",
                          scheduleSettings.rampOutDuration,
                          scheduleSettings.rampOutDuration);
  privateNodeHandle.param("settle_duration",
                          scheduleSettings.settleDuration,
                          scheduleSettings.settleDuration);
  if (!std::isfinite(knotDt) || knotDt <= 0.0 || knotDt > 0.05) {
    ROS_ERROR("D5I2 knot_dt must be in (0, 0.05] s");
    return 1;
  }

  IdentificationExcitationCandidate candidate;
  try {
    candidate = parseCandidate(candidateName);
  } catch (const std::exception& error) {
    ROS_ERROR_STREAM(error.what());
    return 1;
  }
  const IdentificationReferenceProfile profile;
  const IdentificationReferenceSchedule schedule(profile, scheduleSettings);

  // This second, read-only nominal model instance exists only to encode target
  // states in the exact controller-side centroidal coordinates.  It never
  // receives a plant override and it does not own an MPC solver.
  ocs2::legged_robot::LeggedRobotInterface nominalInterface(
      taskFile, urdfFile, referenceFile);
  AdaptiveStateAdapter stateAdapter(nominalInterface.getPinocchioInterface(),
                                    nominalInterface.getCentroidalModelInfo());
  AdaptiveReferenceStateEncoder encoder(
      nominalInterface.getPinocchioInterface(),
      nominalInterface.getCentroidalModelInfo());

  ObservationBuffer observations;
  const auto observationSubscriber =
      nodeHandle.subscribe<ocs2_msgs::mpc_observation>(
          "/legged_robot_mpc_observation", 1, &ObservationBuffer::callback,
          &observations);
  const auto gaitPublisher = nodeHandle.advertise<ocs2_msgs::mode_schedule>(
      "/legged_robot_mpc_mode_schedule", 1, true);
  const auto targetPublisher =
      nodeHandle.advertise<ocs2_msgs::mpc_target_trajectories>(
          "/legged_robot_mpc_target", 1, false);
  const auto legacyStatusPublisher = nodeHandle.advertise<std_msgs::String>(
      "/task_c/locomotion_scheduler_status", 1, true);
  const auto identityPublisher = nodeHandle.advertise<std_msgs::String>(
      "/task_d/d5i2_scheduler_identity", 1, true);
  const auto samplePublisher =
      nodeHandle.advertise<std_msgs::Float64MultiArray>(
          "/task_d/d5i2_scheduler_sample", 10, false);

  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::WallRate rate(100.0);
  ocs2_msgs::mpc_observation observation;
  const auto observationDeadline =
      ros::WallTime::now() + ros::WallDuration(180.0);
  while (ros::ok() && !observations.latest(observation) &&
         ros::WallTime::now() < observationDeadline) {
    rate.sleep();
  }
  if (!observations.latest(observation)) {
    ROS_ERROR("Timed out waiting for first MPC observation");
    return 1;
  }
  const auto initialObservation = observation;
  if (initialObservation.state.value.size() != 24) {
    ROS_ERROR_STREAM("Expected 24 state elements, got "
                     << initialObservation.state.value.size());
    return 1;
  }

  if (!waitForSubscribers(gaitPublisher, 2, 180.0,
                          "/legged_robot_mpc_mode_schedule")) {
    return 1;
  }
  ocs2_msgs::mode_schedule gait;
  gait.eventTimes = kStaticWalkSwitchingTimes;
  gait.modeSequence = kStaticWalkModes;
  gaitPublisher.publish(gait);
  bool walking = false;
  int firstExecutedMode = observation.mode;
  const auto gaitDeadline = ros::WallTime::now() + ros::WallDuration(180.0);
  while (ros::ok() && ros::WallTime::now() < gaitDeadline) {
    observations.latest(observation);
    if (observation.mode != kStance) {
      walking = true;
      firstExecutedMode = observation.mode;
      break;
    }
    rate.sleep();
  }
  if (!walking) {
    ROS_ERROR("Timed out waiting for first executed non-STANCE observation");
    return 1;
  }
  if (observation.time > scheduleSettings.profileStartTime - 0.05) {
    ROS_ERROR_STREAM(
        "Executed gait was acknowledged too late for the fixed D5I2 schedule: "
        << observation.time << " > " << scheduleSettings.profileStartTime - 0.05);
    return 1;
  }
  if (!waitForSubscribers(targetPublisher, 2, 30.0,
                          "/legged_robot_mpc_target")) {
    return 1;
  }

  vector_t initialState(24);
  for (Eigen::Index index = 0; index < initialState.size(); ++index) {
    initialState(index) = initialObservation.state.value[
        static_cast<std::size_t>(index)];
  }
  const auto initialPhysical = stateAdapter.adaptActual(initialState);
  const auto anchorCom = initialPhysical.comPositionWorld;
  const auto anchorEuler = initialState.segment<3>(9);
  const vector_t desiredJoints = initialState.tail(12);

  const std::size_t intervals = static_cast<std::size_t>(
      std::ceil(schedule.completeTime() / knotDt));
  ocs2_msgs::mpc_target_trajectories target;
  target.timeTrajectory.reserve(intervals + 1);
  target.stateTrajectory.reserve(intervals + 1);
  target.inputTrajectory.reserve(intervals + 1);
  for (std::size_t index = 0; index <= intervals; ++index) {
    const scalar_t time =
        index == intervals ? schedule.completeTime() : index * knotDt;
    const auto scheduled = schedule.sample(candidate, time);
    const auto desiredEuler = anchorEuler + scheduled.reference.eulerZyx;
    const auto desiredAngularVelocity =
        ocs2::getGlobalAngularVelocityFromEulerAnglesZyxDerivatives<scalar_t>(
            desiredEuler, scheduled.reference.eulerZyxDerivative);
    const vector_t state = encoder.encode(
        anchorCom + scheduled.reference.comPositionOffsetWorld,
        desiredEuler, scheduled.reference.comVelocityWorld,
        desiredAngularVelocity, desiredJoints);
    target.timeTrajectory.push_back(time);
    ocs2_msgs::mpc_state stateMessage;
    stateMessage.value.assign(state.data(), state.data() + state.size());
    target.stateTrajectory.push_back(std::move(stateMessage));
    ocs2_msgs::mpc_input inputMessage;
    inputMessage.value.assign(initialObservation.input.value.size(), 0.0);
    target.inputTrajectory.push_back(std::move(inputMessage));
  }
  targetPublisher.publish(target);

  std_msgs::String identity;
  std::ostringstream identityStream;
  identityStream.precision(17);
  identityStream
      << "{\"schema_version\":1,\"stage\":\"D5I2_online_safety_information_smoke\"," 
      << "\"profile_id\":\"" << profile.profileId() << "\"," 
      << "\"candidate\":\"" << identificationExcitationCandidateName(candidate)
      << "\",\"c2_envelope\":\"10u3-15u4+6u5\"," 
      << "\"profile_start_time\":" << scheduleSettings.profileStartTime
      << ",\"ramp_in_end_time\":" << schedule.rampInEndTime()
      << ",\"steady_end_time\":" << schedule.steadyEndTime()
      << ",\"ramp_out_end_time\":" << schedule.rampOutEndTime()
      << ",\"complete_time\":" << schedule.completeTime()
      << ",\"knot_dt\":" << knotDt
      << ",\"knot_count\":" << target.timeTrajectory.size()
      << ",\"gait\":\"static_walk\",\"mode_sequence\":[13,7,14,11],"
      << "\"controller_model_only_encoder\":true,\"plant_override\":false,"
      << "\"anchor_com_world\":[" << anchorCom.x() << ',' << anchorCom.y()
      << ',' << anchorCom.z() << "],\"anchor_euler_zyx\":["
      << anchorEuler.x() << ',' << anchorEuler.y() << ',' << anchorEuler.z()
      << "]}";
  identity.data = identityStream.str();
  identityPublisher.publish(identity);

  std_msgs::String legacyStatus;
  std::ostringstream legacyStream;
  legacyStream.precision(17);
  legacyStream
      << "{\"schema_version\":3,\"stage\":\"target_published\"," 
      << "\"experiment_condition\":\"d5i2_richer_reference\"," 
      << "\"gait\":\"static_walk\",\"first_non_stance_time\":"
      << observation.time << ",\"first_non_stance_mode\":"
      << firstExecutedMode << ",\"target_start_time\":"
      << scheduleSettings.profileStartTime << ",\"target_end_time\":"
      << schedule.rampOutEndTime() << ",\"target_published\":true,"
      << "\"relative_x_step\":0.0,\"profile_id\":\""
      << profile.profileId() << "\",\"candidate\":\""
      << identificationExcitationCandidateName(candidate) << "\"}";
  legacyStatus.data = legacyStream.str();
  legacyStatusPublisher.publish(legacyStatus);

  ROS_INFO_STREAM("D5I2 " << identificationExcitationCandidateName(candidate)
                           << " target published with "
                           << target.timeTrajectory.size()
                           << " knots; fixed MPC horizon ends at "
                           << schedule.completeTime());

  double lastPublishedObservationTime = -1.0;
  while (ros::ok()) {
    observations.latest(observation);
    if (observation.time > lastPublishedObservationTime) {
      const auto scheduled = schedule.sample(candidate, observation.time);
      const auto desiredCom =
          anchorCom + scheduled.reference.comPositionOffsetWorld;
      const auto desiredEuler = anchorEuler + scheduled.reference.eulerZyx;
      const auto desiredOmega =
          ocs2::getGlobalAngularVelocityFromEulerAnglesZyxDerivatives<scalar_t>(
              desiredEuler, scheduled.reference.eulerZyxDerivative);
      const auto desiredAlpha =
          ocs2::getGlobalAngularAccelerationFromEulerAnglesZyxDerivatives<scalar_t>(
              desiredEuler, scheduled.reference.eulerZyxDerivative,
              scheduled.reference.eulerZyxSecondDerivative);
      std_msgs::Float64MultiArray sample;
      sample.layout.dim.resize(1);
      sample.layout.dim[0].label = "d5i2_scheduler_v1_28_fields";
      sample.layout.dim[0].size = 28;
      sample.layout.dim[0].stride = 28;
      sample.data.reserve(28);
      sample.data.push_back(observation.time);                         // 0
      sample.data.push_back(scheduled.profileElapsedTime);             // 1
      sample.data.push_back(scheduled.profilePhase);                   // 2
      sample.data.push_back(scheduled.envelope);                       // 3
      sample.data.push_back(scheduled.envelopeDerivative);             // 4
      sample.data.push_back(scheduled.envelopeSecondDerivative);       // 5
      sample.data.push_back(static_cast<double>(scheduled.stage));      // 6
      sample.data.push_back(static_cast<double>(candidate));            // 7
      sample.data.push_back(1.0);                                       // 8
      append(sample.data, desiredCom);                                  // 9:12
      append(sample.data, scheduled.reference.comVelocityWorld);        // 12:15
      append(sample.data, scheduled.reference.comAccelerationWorld);    // 15:18
      append(sample.data, desiredEuler);                                // 18:21
      append(sample.data, desiredOmega);                                // 21:24
      append(sample.data, desiredAlpha);                                // 24:27
      sample.data.push_back(static_cast<double>(observation.mode));      // 27
      samplePublisher.publish(sample);
      lastPublishedObservationTime = observation.time;
    }
    if (observation.time >= schedule.completeTime()) break;
    rate.sleep();
  }
  ros::WallDuration(0.5).sleep();
  return 0;
}
