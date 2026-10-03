#include <cmath>
#include <cstdint>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/rate.h>
#include <ros/spinner.h>
#include <std_msgs/String.h>

#include <ocs2_core/Types.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_msgs/mode_schedule.h>
#include <ocs2_msgs/mpc_observation.h>
#include <ocs2_msgs/mpc_target_trajectories.h>

namespace {

constexpr int kStance = 15;
const std::vector<double> kStaticWalkSwitchingTimes{0.0, 0.3, 0.6, 0.9, 1.2};
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
         ros::WallTime::now() < deadline) rate.sleep();
  if (publisher.getNumSubscribers() < count) {
    ROS_ERROR_STREAM("Timed out waiting for " << count << " subscribers on " << topic
                                               << "; got " << publisher.getNumSubscribers());
    return false;
  }
  return true;
}

std::string gaitStatus(double time, size_t count, uint32_t subscribers,
                       const std::string& condition) {
  std::ostringstream stream;
  stream.precision(17);
  stream << "{\"schema_version\":2,\"stage\":\"gait_published\","
         << "\"experiment_condition\":\"" << condition << "\","
         << "\"gait\":\"static_walk\",\"gait_publish_observation_time\":" << time
         << ",\"gait_publish_count\":" << count
         << ",\"gait_subscriber_count\":" << subscribers
         << ",\"switching_times\":[0.0,0.3,0.6,0.9,1.2],"
         << "\"mode_sequence\":[13,7,14,11]}";
  return stream.str();
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "deterministic_locomotion_scheduler");
  ros::NodeHandle nodeHandle;
  ros::NodeHandle privateNodeHandle("~");
  double step = 0.10, targetStartTime = 1.20, targetDuration = -1.0;
  std::string experimentCondition = "walk_target";
  privateNodeHandle.param("target_step", step, step);
  privateNodeHandle.param("target_start_time", targetStartTime, targetStartTime);
  privateNodeHandle.param("target_duration", targetDuration, targetDuration);
  privateNodeHandle.param("experiment_condition", experimentCondition,
                          experimentCondition);
  const bool publishGait = experimentCondition == "walk_target";
  const bool publishTarget = experimentCondition != "stance_hold";
  if (experimentCondition != "stance_hold" &&
      experimentCondition != "stance_target" &&
      experimentCondition != "walk_target") {
    ROS_ERROR_STREAM("Unsupported deterministic scheduler condition: "
                     << experimentCondition);
    return 1;
  }
  if (!(step > 0.0) || !std::isfinite(step) || targetStartTime < 0.0 ||
      !std::isfinite(targetStartTime) ||
      (!(targetDuration < 0.0) &&
       (!(targetDuration > 0.0) || !std::isfinite(targetDuration)))) {
    ROS_ERROR("Locomotion target parameters are invalid");
    return 1;
  }

  std::string referenceFile;
  if (!nodeHandle.getParam("/referenceFile", referenceFile)) {
    ROS_ERROR("Missing /referenceFile");
    return 1;
  }
  double comHeight = 0.0, displacementVelocity = 0.0;
  ocs2::vector_t defaultJointState(12);
  ocs2::loadData::loadCppDataType(referenceFile, "comHeight", comHeight);
  ocs2::loadData::loadCppDataType(referenceFile, "targetDisplacementVelocity", displacementVelocity);
  ocs2::loadData::loadEigenMatrix(referenceFile, "defaultJointState", defaultJointState);
  if (!(displacementVelocity > 0.0)) {
    ROS_ERROR("targetDisplacementVelocity must be positive");
    return 1;
  }

  ObservationBuffer observations;
  const auto observationSubscriber = nodeHandle.subscribe<ocs2_msgs::mpc_observation>(
      "/legged_robot_mpc_observation", 1, &ObservationBuffer::callback, &observations);
  const auto gaitPublisher = nodeHandle.advertise<ocs2_msgs::mode_schedule>(
      "/legged_robot_mpc_mode_schedule", 1, true);
  const auto targetPublisher = nodeHandle.advertise<ocs2_msgs::mpc_target_trajectories>(
      // Target 仍只按既有协议发布一次；改为 latched 仅保证随后完成连接的
      // rosbag recorder 能收到这条实际交给 MPC 的同一消息，不改变 target 数值、
      // 时刻或控制链路。
      "/legged_robot_mpc_target", 1, true);
  const auto statusPublisher = nodeHandle.advertise<std_msgs::String>(
      "/task_c/locomotion_scheduler_status", 1, true);
  ros::AsyncSpinner spinner(2);
  spinner.start();

  ocs2_msgs::mpc_observation observation;
  ros::WallRate rate(100.0);
  const auto observationDeadline = ros::WallTime::now() + ros::WallDuration(180.0);
  while (ros::ok() && !observations.latest(observation) &&
         ros::WallTime::now() < observationDeadline) rate.sleep();
  if (!observations.latest(observation)) {
    ROS_ERROR("Timed out waiting for first MPC observation");
    return 1;
  }
  const auto initialObservation = observation;
  double firstPublishTime = observation.time;
  size_t publishCount = 0;
  int firstExecutedMode = observation.mode;
  if (publishGait) {
    // rosbag plus C++ GaitReceiver. roscpp is required because GaitReceiver
    // explicitly requests UDPROS.
    if (!waitForSubscribers(gaitPublisher, 2, 180.0,
                            "/legged_robot_mpc_mode_schedule")) return 1;
    ocs2_msgs::mode_schedule gait;
    gait.eventTimes = kStaticWalkSwitchingTimes;
    gait.modeSequence = kStaticWalkModes;
    firstPublishTime = observation.time;
    publishCount = 1;
    gaitPublisher.publish(gait);
    std_msgs::String gaitStatusMessage;
    gaitStatusMessage.data = gaitStatus(
        observation.time, publishCount, gaitPublisher.getNumSubscribers(),
        experimentCondition);
    statusPublisher.publish(gaitStatusMessage);
    bool walking = false;
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
  } else if (observation.mode != kStance) {
    ROS_ERROR_STREAM("STANCE condition started in unexpected mode "
                     << static_cast<int>(observation.mode));
    return 1;
  }

  if (!publishTarget) {
    const auto& initialState = initialObservation.state.value;
    if (initialState.size() != 24) {
      ROS_ERROR_STREAM("Expected 24 state elements, got " << initialState.size());
      return 1;
    }
    std_msgs::String status;
    std::ostringstream stream;
    stream.precision(17);
    stream << "{\"schema_version\":2,\"stage\":\"condition_ready\","
           << "\"experiment_condition\":\"" << experimentCondition << "\","
           << "\"gait\":\"stance\",\"gait_publish_count\":0,"
           << "\"target_published\":false,\"target_start_time\":"
           << targetStartTime << ",\"relative_x_step\":0.0,"
           << "\"initial_observation_time\":" << initialObservation.time
           << ",\"initial_x\":" << initialState[6] << "}";
    status.data = stream.str();
    statusPublisher.publish(status);
    ROS_INFO_STREAM("Deterministic stance_hold configured at MPC time "
                    << observation.time << "; no gait or target published");
    ros::WallDuration(0.5).sleep();
    return 0;
  }

  if (!waitForSubscribers(targetPublisher, 2, 30.0,
                          "/legged_robot_mpc_target")) return 1;

  if (observation.time > targetStartTime - 0.05) {
    ROS_ERROR_STREAM("Executed gait was acknowledged too late for the fixed target schedule: "
                     << observation.time << " > " << targetStartTime - 0.05);
    return 1;
  }
  const double targetEndTime =
      targetDuration > 0.0 ? targetStartTime + targetDuration
                           : targetStartTime + step / displacementVelocity;
  const double commandedVelocity = step / (targetEndTime - targetStartTime);
  const auto& initialState = initialObservation.state.value;
  if (initialState.size() != 24) {
    ROS_ERROR_STREAM("Expected 24 state elements, got " << initialState.size());
    return 1;
  }
  ocs2_msgs::mpc_target_trajectories target;
  target.timeTrajectory = {targetStartTime, targetEndTime};
  target.stateTrajectory.resize(2);
  target.inputTrajectory.resize(2);
  target.stateTrajectory[0].value.assign(24, 0.0);
  target.stateTrajectory[1].value.assign(24, 0.0);
  for (size_t i = 0; i < 6; ++i) {
    target.stateTrajectory[0].value[6 + i] = initialState[6 + i];
    target.stateTrajectory[1].value[6 + i] = initialState[6 + i];
  }
  target.stateTrajectory[1].value[6] += step;
  target.stateTrajectory[1].value[8] = comHeight;
  for (size_t i = 0; i < 12; ++i) {
    target.stateTrajectory[0].value[12 + i] = defaultJointState(i);
    target.stateTrajectory[1].value[12 + i] = defaultJointState(i);
  }
  target.inputTrajectory[0].value.assign(observation.input.value.size(), 0.0);
  target.inputTrajectory[1].value.assign(observation.input.value.size(), 0.0);
  targetPublisher.publish(target);

  std_msgs::String status;
  std::ostringstream stream;
  stream.precision(17);
  stream << "{\"schema_version\":2,\"stage\":\"target_published\","
         << "\"experiment_condition\":\"" << experimentCondition << "\","
         << "\"gait\":\"" << (publishGait ? "static_walk" : "stance")
         << "\",\"gait_publish_observation_time\":" << firstPublishTime
         << ",\"gait_publish_count\":" << publishCount
         << ",\"first_non_stance_time\":" << observation.time
         << ",\"first_non_stance_mode\":" << firstExecutedMode
         << ",\"target_start_time\":" << targetStartTime
         << ",\"target_end_time\":" << targetEndTime
         << ",\"target_start_x\":" << initialState[6]
         << ",\"target_final_x\":" << initialState[6] + step
         << ",\"relative_x_step\":" << step
         << ",\"target_displacement_velocity\":" << commandedVelocity
         << ",\"switching_times\":["
         << (publishGait ? "0.0,0.3,0.6,0.9,1.2" : "") << "],"
         << "\"mode_sequence\":["
         << (publishGait ? "13,7,14,11" : "15") << "]}";
  status.data = stream.str();
  statusPublisher.publish(status);
  ROS_INFO_STREAM(experimentCondition << " configured at MPC time "
                                      << observation.time << "; +" << step
                                      << " m target "
                                                        << initialState[6] << " -> "
                                                        << initialState[6] + step
                                                        << " at fixed MPC time " << targetStartTime);
  ros::WallDuration(0.5).sleep();
  return 0;
}
