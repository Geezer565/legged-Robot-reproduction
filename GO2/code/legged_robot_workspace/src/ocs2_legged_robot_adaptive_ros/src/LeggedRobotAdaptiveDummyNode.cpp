#include <array>
#include <cmath>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/service.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>

#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_legged_robot/LeggedRobotInterface.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Dummy_Loop.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>

#include <ocs2_legged_robot_adaptive/dynamics/MassMismatchGroundTruth.h>
#include <ocs2_legged_robot_adaptive/dynamics/NominalCoordinatePlantDynamics.h>
#include <ocs2_legged_robot_adaptive/adaptive/PayloadInertial10D.h>
#include <ocs2_legged_robot_adaptive_ros/MatchedWrenchDisturbance.h>
#include <ocs2_legged_robot_ros/visualization/LeggedRobotVisualizer.h>

namespace adaptive = ocs2::legged_robot_adaptive::adaptive;

namespace ocs2 {
namespace legged_robot_adaptive_ros {

using legged_robot_adaptive::MassMismatchGroundTruth;
using legged_robot_adaptive::NominalCoordinatePlantDynamics;
using legged_robot_adaptive::adaptive::PayloadInertial10D;

class AdaptiveExperimentDummyLoop final : public MRT_ROS_Dummy_Loop {
 public:
  AdaptiveExperimentDummyLoop(MRT_ROS_Interface& mrt, scalar_t mrtFrequency,
                              scalar_t mpcFrequency, scalar_t robotMass,
                              scalar_t truePlantMassScale,
                              bool taskCDiagnosticsEnabled,
                              MassMismatchGroundTruth& massGroundTruth,
                              NominalCoordinatePlantDynamics*
                                  nominalCoordinateDynamics,
                              NominalCoordinatePlantDynamics*
                                  payloadInertialPlantDynamics,
                              PayloadInertial10D::Parameters payloadParameters,
                              MatchedWrenchDisturbance disturbance,
                              const std::string& urdfFile,
                              ros::NodeHandle& nodeHandle)
      : MRT_ROS_Dummy_Loop(mrt, mrtFrequency, mpcFrequency),
        dt_(1.0 / mrtFrequency),
        robotMass_(robotMass),
        truePlantMassScale_(truePlantMassScale),
        taskCDiagnosticsEnabled_(taskCDiagnosticsEnabled),
        massGroundTruth_(massGroundTruth),
        nominalCoordinateDynamics_(nominalCoordinateDynamics),
        payloadInertialPlantDynamics_(payloadInertialPlantDynamics),
        payloadParameters_(std::move(payloadParameters)),
        payloadTruth_(payloadInertialPlantDynamics_
                          ? PayloadInertial10D::toVector(payloadParameters_)
                          : adaptive::vector10_t::Zero()),
        disturbance_(std::move(disturbance)),
        samplePublisher_(nodeHandle.advertise<std_msgs::Float64MultiArray>(
            "/adaptive_update/disturbance_sample", 10)),
        modelMismatchPublisher_(
            nodeHandle.advertise<std_msgs::Float64MultiArray>(
                "/task_c/model_mismatch_sample", 10)),
        modelIdentityPublisher_(nodeHandle.advertise<std_msgs::String>(
            "/task_c/model_identity", 1, true)),
        payloadInertialPlantPublisher_(
            nodeHandle.advertise<std_msgs::Float64MultiArray>(
                "/task_d/payload_inertial_plant_sample", 10)),
        payloadInertialIdentityPublisher_(nodeHandle.advertise<std_msgs::String>(
            "/task_d/model_identity", 1, true)) {
    std_msgs::String identity;
    std::ostringstream stream;
    const char* plantModelId = payloadInertialOverrideActive()
                                   ? "srbd_payload_inertial_10d_nominal_coordinates_v2"
                               : massOverrideActive()
                                   ? "srbd_mass_scalar_nominal_coordinates_v2"
                                   : "nominal_ocs2_rollout_v1";
    stream << std::setprecision(17)
           << "{\"schema_version\":1,"
           << "\"controller_model_id\":\"nominal_urdf_srbd_v1\","
           << "\"plant_model_id\":\"" << plantModelId << "\","
           << "\"nominal_mass\":" << massGroundTruth_.nominalMass() << ","
           << "\"true_mass\":" << massGroundTruth_.trueMass() << ","
           << "\"payload_delta\":"
           << massGroundTruth_.trueMass() - massGroundTruth_.nominalMass()
           << ",\"true_plant_mass_scale\":" << truePlantMassScale_ << ","
           << "\"override_active\":"
           << (massOverrideActive() ? "true" : "false") << ","
           << "\"task_c_diagnostics_enabled\":"
           << (taskCDiagnosticsEnabled_ ? "true" : "false") << ","
           << "\"controller_urdf\":\"" << urdfFile << "\"}";
    identity.data = stream.str();
    modelIdentityPublisher_.publish(identity);

    std_msgs::String taskDIdentity;
    std::ostringstream taskDStream;
    const scalar_t payloadMass = payloadInertialOverrideActive()
                                     ? payloadParameters_.mass
                                     : 0.0;
    adaptive::vector3_t payloadCom = adaptive::vector3_t::Zero();
    adaptive::matrix3_t payloadInertiaCom = adaptive::matrix3_t::Zero();
    if (payloadInertialOverrideActive()) {
      payloadCom = payloadParameters_.firstMomentNominalComBase /
                   payloadParameters_.mass;
      payloadInertiaCom =
          PayloadInertial10D::inertiaAboutPayloadCom(payloadParameters_);
    }
    taskDStream << std::setprecision(17)
                << "{\"schema_version\":2,"
                << "\"controller_model_id\":\"nominal_urdf_srbd_v1\","
                << "\"plant_model_id\":\"" << plantModelId << "\","
                << "\"controller_nominal_mass\":"
                << massGroundTruth_.nominalMass() << ","
                << "\"plant_true_total_mass\":"
                << (massGroundTruth_.nominalMass() + payloadMass) << ","
                << "\"payload_inertial_override_active\":"
                << (payloadInertialOverrideActive() ? "true" : "false") << ","
                << "\"payload_mass\":" << payloadMass << ","
                << "\"payload_com_nominal_c0_base\":[" << payloadCom(0) << ","
                << payloadCom(1) << "," << payloadCom(2) << "],"
                << "\"payload_inertia_com_base\":[" << payloadInertiaCom(0, 0)
                << "," << payloadInertiaCom(0, 1) << ","
                << payloadInertiaCom(0, 2) << "," << payloadInertiaCom(1, 1)
                << "," << payloadInertiaCom(1, 2) << ","
                << payloadInertiaCom(2, 2) << "],"
                << "\"controller_urdf\":\"" << urdfFile << "\"}";
    taskDIdentity.data = taskDStream.str();
    payloadInertialIdentityPublisher_.publish(taskDIdentity);
  }

 protected:
  void modifyObservation(SystemObservation& observation) override {
    disturbance_.applyToState(observation.time, dt_, robotMass_, observation.state);
    const auto applied = disturbance_.appliedWrench(observation.time);

    std_msgs::Float64MultiArray sample;
    sample.data.reserve(8);
    sample.data.push_back(observation.time);
    sample.data.push_back(disturbance_.isActive(observation.time) ? 1.0 : 0.0);
    sample.data.insert(sample.data.end(), applied.data(), applied.data() + 6);
    samplePublisher_.publish(sample);

    if (payloadInertialOverrideActive()) {
      const vector_t plantState =
          payloadInertialPlantDynamics_->plantStateFromNominalState(
              observation.state);
      const vector_t roundTrip =
          payloadInertialPlantDynamics_->nominalStateFromPlantState(
              plantState);
      PreComputation preComputation;
      const vector_t nominalFlow = nominalCoordinateDynamics_->computeFlowMap(
          observation.time, observation.state, observation.input,
          preComputation);
      const vector_t plantFlow = payloadInertialPlantDynamics_->computeFlowMap(
          observation.time, observation.state, observation.input,
          preComputation);
      const vector_t residual = plantFlow - nominalFlow;

      // Fixed 74-field plant-truth schema: identity [14], pi_star [10],
      // scalar consistency [2], nominal flow [24], true flow [24]. All flows
      // use the controller's nominal centroidal coordinates.
      std_msgs::Float64MultiArray plantSample;
      plantSample.layout.dim.resize(1);
      plantSample.layout.dim[0].label =
          "payload_inertial_plant_v2_74_fields";
      plantSample.layout.dim[0].size = 74;
      plantSample.layout.dim[0].stride = 74;
      plantSample.data.reserve(74);
      plantSample.data.push_back(observation.time);                       // 0
      plantSample.data.push_back(1.0);                                    // 1
      plantSample.data.push_back(massGroundTruth_.nominalMass());         // 2
      plantSample.data.push_back(
          payloadInertialPlantDynamics_->plantInfo().robotMass);          // 3
      plantSample.data.push_back(payloadParameters_.mass);                // 4
      const adaptive::vector3_t payloadCom =
          payloadParameters_.firstMomentNominalComBase /
          payloadParameters_.mass;
      plantSample.data.insert(plantSample.data.end(), payloadCom.data(),
                              payloadCom.data() + 3);                      // 5:8
      const adaptive::matrix3_t inertiaCom =
          PayloadInertial10D::inertiaAboutPayloadCom(payloadParameters_);
      plantSample.data.push_back(inertiaCom(0, 0));                        // 8
      plantSample.data.push_back(inertiaCom(0, 1));                        // 9
      plantSample.data.push_back(inertiaCom(0, 2));                        // 10
      plantSample.data.push_back(inertiaCom(1, 1));                        // 11
      plantSample.data.push_back(inertiaCom(1, 2));                        // 12
      plantSample.data.push_back(inertiaCom(2, 2));                        // 13
      plantSample.data.insert(plantSample.data.end(), payloadTruth_.data(),
                              payloadTruth_.data() + 10);                  // 14:24
      plantSample.data.push_back((roundTrip - observation.state).norm()); // 24
      plantSample.data.push_back(residual.norm());                         // 25
      plantSample.data.insert(plantSample.data.end(), nominalFlow.data(),
                              nominalFlow.data() + nominalFlow.size());    // 26:50
      plantSample.data.insert(plantSample.data.end(), plantFlow.data(),
                              plantFlow.data() + plantFlow.size());        // 50:74
      payloadInertialPlantPublisher_.publish(plantSample);
    }

    if (!taskCDiagnosticsEnabled_) {
      return;
    }

    const auto groundTruth = massGroundTruth_.evaluate(
        observation.time, observation.state, observation.input);
    std_msgs::Float64MultiArray modelSample;
    modelSample.layout.dim.resize(1);
    modelSample.layout.dim[0].label = "task_c_model_mismatch_v1_41_fields";
    modelSample.layout.dim[0].size = 41;
    modelSample.layout.dim[0].stride = 41;
    modelSample.data.reserve(41);
    modelSample.data.push_back(observation.time);                         // 0
    modelSample.data.push_back(massGroundTruth_.nominalMass());          // 1
    modelSample.data.push_back(massGroundTruth_.trueMass());             // 2
    modelSample.data.push_back(massGroundTruth_.trueMass() -
                               massGroundTruth_.nominalMass());          // 3
    modelSample.data.push_back(truePlantMassScale_);                     // 4
    modelSample.data.push_back(massOverrideActive() ? 1.0 : 0.0);        // 5
    modelSample.data.insert(modelSample.data.end(),
                            groundTruth.nominalFlow.data(),
                            groundTruth.nominalFlow.data() + 6);          // 6:12
    modelSample.data.insert(modelSample.data.end(), groundTruth.trueFlow.data(),
                            groundTruth.trueFlow.data() + 6);             // 12:18
    modelSample.data.insert(
        modelSample.data.end(), groundTruth.normalizedDynamicsResidual.data(),
        groundTruth.normalizedDynamicsResidual.data() + 6);               // 18:24
    modelSample.data.insert(modelSample.data.end(), groundTruth.piStar.data(),
                            groundTruth.piStar.data() + 6);                // 24:30
    modelSample.data.insert(
        modelSample.data.end(), groundTruth.reconstructionError.data(),
        groundTruth.reconstructionError.data() + 6);                       // 30:36
    modelSample.data.push_back(groundTruth.fullFlowResidualNorm);          // 36
    modelSample.data.push_back(groundTruth.unmatchedFlowResidualNorm);     // 37
    modelSample.data.push_back(groundTruth.piStar.norm());                 // 38
    modelSample.data.push_back(
        groundTruth.reconstructionError.cwiseAbs().maxCoeff());            // 39
    const bool valid = groundTruth.nominalFlow.allFinite() &&
                       groundTruth.trueFlow.allFinite() &&
                       groundTruth.normalizedDynamicsResidual.allFinite() &&
                       groundTruth.piStar.allFinite() &&
                       groundTruth.reconstructionError.allFinite() &&
                       std::isfinite(groundTruth.fullFlowResidualNorm) &&
                       std::isfinite(groundTruth.unmatchedFlowResidualNorm);
    modelSample.data.push_back(valid ? 1.0 : 0.0);                         // 40
    modelMismatchPublisher_.publish(modelSample);
  }

 private:
  bool massOverrideActive() const {
    return std::abs(truePlantMassScale_ - 1.0) > 1.0e-12;
  }

  bool payloadInertialOverrideActive() const {
    return nominalCoordinateDynamics_ != nullptr &&
           payloadInertialPlantDynamics_ != nullptr;
  }

  scalar_t dt_;
  scalar_t robotMass_;
  scalar_t truePlantMassScale_;
  bool taskCDiagnosticsEnabled_;
  MassMismatchGroundTruth& massGroundTruth_;
  NominalCoordinatePlantDynamics* nominalCoordinateDynamics_;
  NominalCoordinatePlantDynamics* payloadInertialPlantDynamics_;
  PayloadInertial10D::Parameters payloadParameters_;
  adaptive::vector10_t payloadTruth_;
  MatchedWrenchDisturbance disturbance_;
  ros::Publisher samplePublisher_;
  ros::Publisher modelMismatchPublisher_;
  ros::Publisher modelIdentityPublisher_;
  ros::Publisher payloadInertialPlantPublisher_;
  ros::Publisher payloadInertialIdentityPublisher_;
};

MatchedWrenchDisturbanceConfig loadDisturbanceConfig(
    ros::NodeHandle& privateNodeHandle) {
  MatchedWrenchDisturbanceConfig config;
  privateNodeHandle.param("disturbance_enabled", config.enabled, false);
  privateNodeHandle.param("disturbance_start_time", config.startTime, 1.0);
  privateNodeHandle.param("disturbance_end_time", config.endTime, 4.0);
  std::string profile = "constant";
  privateNodeHandle.param("disturbance_profile", profile, profile);
  config.profile = matchedWrenchDisturbanceProfileFromString(profile);
  privateNodeHandle.param("disturbance_frequency_hz", config.frequencyHz, 0.25);
  constexpr std::array<const char*, 6> names{{
      "disturbance_fx", "disturbance_fy", "disturbance_fz",
      "disturbance_tx", "disturbance_ty", "disturbance_tz"}};
  constexpr std::array<const char*, 6> secondaryNames{{
      "disturbance_secondary_fx", "disturbance_secondary_fy",
      "disturbance_secondary_fz", "disturbance_secondary_tx",
      "disturbance_secondary_ty", "disturbance_secondary_tz"}};
  for (size_t i = 0; i < names.size(); ++i) {
    privateNodeHandle.param(names[i], config.wrench(i), 0.0);
    privateNodeHandle.param(secondaryNames[i], config.secondaryWrench(i), 0.0);
  }
  return config;
}

}  // namespace legged_robot_adaptive_ros
}  // namespace ocs2

int main(int argc, char** argv) {
  using namespace ocs2;
  using namespace ocs2::legged_robot;
  using namespace ocs2::legged_robot_adaptive_ros;

  const std::string robotName = "legged_robot";
  ros::init(argc, argv, robotName + "_adaptive_mrt");
  ros::NodeHandle nodeHandle;
  ros::NodeHandle privateNodeHandle("~");

  std::string taskFile, urdfFile, referenceFile;
  nodeHandle.getParam("/taskFile", taskFile);
  nodeHandle.getParam("/urdfFile", urdfFile);
  nodeHandle.getParam("/referenceFile", referenceFile);

  LeggedRobotInterface interface(taskFile, urdfFile, referenceFile);

  scalar_t truePlantMassScale = 1.0;
  privateNodeHandle.param("true_plant_mass_scale", truePlantMassScale, 1.0);
  bool payloadInertialOverrideEnabled = false;
  privateNodeHandle.param("true_plant_payload_inertial_enabled",
                          payloadInertialOverrideEnabled, false);
  bool taskCDiagnosticsEnabled = false;
  privateNodeHandle.param("task_c_diagnostics_enabled",
                          taskCDiagnosticsEnabled, false);
  if (!(truePlantMassScale > 0.0) || !std::isfinite(truePlantMassScale)) {
    ROS_ERROR_STREAM("~true_plant_mass_scale must be positive and finite, got "
                     << truePlantMassScale);
    return 1;
  }
  const scalar_t nominalMass = interface.getCentroidalModelInfo().robotMass;
  const scalar_t truePlantMass = nominalMass * truePlantMassScale;
  if (payloadInertialOverrideEnabled &&
      std::abs(truePlantMassScale - 1.0) > 1.0e-12) {
    ROS_ERROR("Mass-scale and payload-inertial plant overrides are mutually exclusive.");
    return 1;
  }

  PayloadInertial10D::Parameters payloadParameters;
  if (payloadInertialOverrideEnabled) {
    scalar_t payloadMass = 0.0;
    adaptive::vector3_t payloadCom = adaptive::vector3_t::Zero();
    adaptive::matrix3_t payloadInertiaCom = adaptive::matrix3_t::Zero();
    privateNodeHandle.param("true_plant_payload_mass", payloadMass, 0.0);
    privateNodeHandle.param("true_plant_payload_com_x", payloadCom(0), 0.0);
    privateNodeHandle.param("true_plant_payload_com_y", payloadCom(1), 0.0);
    privateNodeHandle.param("true_plant_payload_com_z", payloadCom(2), 0.0);
    privateNodeHandle.param("true_plant_payload_inertia_xx",
                            payloadInertiaCom(0, 0), 0.0);
    privateNodeHandle.param("true_plant_payload_inertia_xy",
                            payloadInertiaCom(0, 1), 0.0);
    privateNodeHandle.param("true_plant_payload_inertia_xz",
                            payloadInertiaCom(0, 2), 0.0);
    privateNodeHandle.param("true_plant_payload_inertia_yy",
                            payloadInertiaCom(1, 1), 0.0);
    privateNodeHandle.param("true_plant_payload_inertia_yz",
                            payloadInertiaCom(1, 2), 0.0);
    privateNodeHandle.param("true_plant_payload_inertia_zz",
                            payloadInertiaCom(2, 2), 0.0);
    payloadInertiaCom(1, 0) = payloadInertiaCom(0, 1);
    payloadInertiaCom(2, 0) = payloadInertiaCom(0, 2);
    payloadInertiaCom(2, 1) = payloadInertiaCom(1, 2);
    try {
      payloadParameters = PayloadInertial10D::fromPhysicalPayload(
          payloadMass, payloadCom, payloadInertiaCom);
    } catch (const std::exception& error) {
      ROS_ERROR_STREAM("Invalid true plant payload-inertial override: "
                       << error.what());
      return 1;
    }
  }
  if (std::abs(truePlantMassScale - 1.0) > 1.0e-12 &&
      !taskCDiagnosticsEnabled) {
    ROS_WARN("Plant mass override is active while Task C diagnostics are disabled.");
  }
  MassMismatchGroundTruth massGroundTruth(
      interface.getPinocchioInterface(), interface.getCentroidalModelInfo(),
      truePlantMass);

  MRT_ROS_Interface mrt(robotName);
  std::unique_ptr<TimeTriggeredRollout> truePlantRollout;
  std::unique_ptr<NominalCoordinatePlantDynamics>
      nominalCoordinateDynamics;
  std::unique_ptr<NominalCoordinatePlantDynamics>
      truePlantCoordinateDynamics;
  if (payloadInertialOverrideEnabled) {
    const auto payloadInfo = PayloadInertial10D::makePlantModelInfo(
        interface.getCentroidalModelInfo(), payloadParameters);
    nominalCoordinateDynamics.reset(new NominalCoordinatePlantDynamics(
        interface.getPinocchioInterface(), interface.getCentroidalModelInfo(),
        interface.getCentroidalModelInfo()));
    truePlantCoordinateDynamics.reset(new NominalCoordinatePlantDynamics(
        interface.getPinocchioInterface(), interface.getCentroidalModelInfo(),
        payloadInfo));
    truePlantRollout.reset(new TimeTriggeredRollout(
        *truePlantCoordinateDynamics, interface.rolloutSettings()));
    mrt.initRollout(truePlantRollout.get());
  } else if (std::abs(truePlantMassScale - 1.0) > 1.0e-12) {
    // A plant with altered inertia/mass has a different normalized momentum
    // coordinate even at the same physical pose/twist. The rollout therefore
    // integrates a push-forward flow in the unchanged controller coordinate.
    truePlantCoordinateDynamics.reset(new NominalCoordinatePlantDynamics(
        interface.getPinocchioInterface(), interface.getCentroidalModelInfo(),
        massGroundTruth.trueInfo()));
    truePlantRollout.reset(new TimeTriggeredRollout(
        *truePlantCoordinateDynamics, interface.rolloutSettings()));
    mrt.initRollout(truePlantRollout.get());
  } else {
    // Preserve the exact Task A/B baseline when the override is disabled.
    mrt.initRollout(&interface.getRollout());
  }
  mrt.launchNodes(nodeHandle);

  ROS_INFO_STREAM(std::setprecision(17)
                  << "Task C plant identity: nominal_mass=" << nominalMass
                  << ", true_mass=" << truePlantMass
                  << ", true_plant_mass_scale=" << truePlantMassScale
                  << ", override_active="
                  << (std::abs(truePlantMassScale - 1.0) > 1.0e-12)
                  << ", payload_inertial_override_active="
                  << payloadInertialOverrideEnabled
                  << ", diagnostics_enabled=" << taskCDiagnosticsEnabled);

  CentroidalModelPinocchioMapping pinocchioMapping(
      interface.getCentroidalModelInfo());
  PinocchioEndEffectorKinematics endEffectorKinematics(
      interface.getPinocchioInterface(), pinocchioMapping,
      interface.modelSettings().contactNames3DoF);
  auto visualizer = std::make_shared<LeggedRobotVisualizer>(
      interface.getPinocchioInterface(), interface.getCentroidalModelInfo(),
      endEffectorKinematics, nodeHandle);

  AdaptiveExperimentDummyLoop simulator(
      mrt, interface.mpcSettings().mrtDesiredFrequency_,
      interface.mpcSettings().mpcDesiredFrequency_,
      nominalMass, truePlantMassScale, taskCDiagnosticsEnabled,
      massGroundTruth, nominalCoordinateDynamics.get(),
      payloadInertialOverrideEnabled ? truePlantCoordinateDynamics.get()
                                     : nullptr,
      payloadParameters,
      MatchedWrenchDisturbance(loadDisturbanceConfig(privateNodeHandle)),
      urdfFile, nodeHandle);
  simulator.subscribeObservers({visualizer});

  SystemObservation initialObservation;
  initialObservation.state = interface.getInitialState();
  initialObservation.input =
      vector_t::Zero(interface.getCentroidalModelInfo().inputDim);
  initialObservation.mode = ModeNumber::STANCE;
  TargetTrajectories initialTarget({0.0}, {initialObservation.state},
                                   {initialObservation.input});
const std::string resetService = "/" + robotName + "_mpc_reset";
  if (!ros::service::waitForService(resetService, ros::Duration(120.0))) {
    ROS_ERROR_STREAM("Timed out waiting for MPC reset service: " << resetService);
    return 1;
  }
  simulator.run(initialObservation, initialTarget);
  return 0;
}
