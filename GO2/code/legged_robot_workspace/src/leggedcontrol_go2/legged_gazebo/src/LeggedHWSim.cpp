/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2021, Qiayuan Liao
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

//
// Created by qiayuan on 2/10/21.
//

#include "legged_gazebo/LeggedHWSim.h"
#include <gazebo_ros_control/gazebo_ros_control_plugin.h>
#include <xmlrpcpp/XmlRpcException.h>
namespace legged {
// Gazebo otherwise catches Load exceptions and discards their explanation.
class LoggedGazeboRosControlPlugin : public gazebo_ros_control::GazeboRosControlPlugin {
 public:
  void Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf) override {
    try {
      GazeboRosControlPlugin::Load(parent, sdf);
    } catch (const std::exception& error) {
      gzerr << "Go2 control plugin Load failed: " << error.what() << std::endl;
      throw;
    } catch (const XmlRpc::XmlRpcException& error) {
      gzerr << "Go2 control plugin parameter error: " << error.getMessage() << std::endl;
      throw;
    }
  }
};

bool LeggedHWSim::initSim(const std::string& robot_namespace, ros::NodeHandle model_nh, gazebo::physics::ModelPtr parent_model,
                          const urdf::Model* urdf_model, std::vector<transmission_interface::TransmissionInfo> transmissions) {
  bool ret = DefaultRobotHWSim::initSim(robot_namespace, model_nh, parent_model, urdf_model, transmissions);
  // Joint interface
  registerInterface(&hybridJointInterface_);
  std::vector<std::string> names = ej_interface_.getNames();
  for (const auto& name : names) {
    hybridJointDatas_.push_back(HybridJointData{.joint_ = ej_interface_.getHandle(name)});
    HybridJointData& back = hybridJointDatas_.back();
    hybridJointInterface_.registerHandle(HybridJointHandle(back.joint_, &back.posDes_, &back.velDes_, &back.kp_, &back.kd_, &back.ff_));
    cmdBuffer_.insert(std::make_pair(name.c_str(), std::deque<HybridJointCommand>()));
  }



  

  // IMU interface
  registerInterface(&imuSensorInterface_);
  XmlRpc::XmlRpcValue xmlRpcValue;
  if (!model_nh.getParam("gazebo/imus", xmlRpcValue)) {
    ROS_WARN("No imu specified");
  } else {
    parseImu(xmlRpcValue, parent_model);
  }
  if (!model_nh.getParam("gazebo/delay", delay_)) {
    delay_ = 0.;
  }
  if (!model_nh.getParam("gazebo/contacts", xmlRpcValue)) {
    ROS_WARN("No contacts specified");
  } else {
    parseContacts(xmlRpcValue);
  }

  contactManager_ = parent_model->GetWorld()->Physics()->GetContactManager();
  contactManager_->SetNeverDropContacts(true);  // NOTE: If false, we need to select view->contacts in gazebo GUI to
                                                // avoid returning nothing when calling ContactManager::GetContacts()
  return ret;
}

void LeggedHWSim::readSim(ros::Time time, ros::Duration period) {
  //  gazebo_ros_control::DefaultRobotHWSim::readSim(time, period); The DefaultRobotHWSim Provide a bias joint velocity
  const bool initializeHistory = !jointPositionHistoryValid_ || time == ros::Time(period.toSec());
  for (unsigned int j = 0; j < n_dof_; j++) {
    double position = sim_joints_[j]->Position(0);
    // The base class seeds position with 1.0, not a previous measurement.
    // Never differentiate that placeholder on the first physics update/reset.
    if (initializeHistory) joint_position_[j] = position;
    joint_velocity_[j] = (position - joint_position_[j]) / period.toSec();
    if (joint_types_[j] == urdf::Joint::PRISMATIC) {
      joint_position_[j] = position;
    } else {
      joint_position_[j] += angles::shortest_angular_distance(joint_position_[j], position);
    }
    joint_effort_[j] = sim_joints_[j]->GetForce((unsigned int)(0));
  }
  jointPositionHistoryValid_ = true;

  // Imu Sensor
  for (auto& imu : imuDatas_) {
    // TODO(qiayuan) Add noise
    ignition::math::Pose3d pose = imu.linkPtr_->WorldPose();
    imu.ori_[0] = pose.Rot().X();
    imu.ori_[1] = pose.Rot().Y();
    imu.ori_[2] = pose.Rot().Z();
    imu.ori_[3] = pose.Rot().W();
    ignition::math::Vector3d rate = imu.linkPtr_->RelativeAngularVel();
    imu.angularVel_[0] = rate.X();
    imu.angularVel_[1] = rate.Y();
    imu.angularVel_[2] = rate.Z();

    ignition::math::Vector3d gravity = {0., 0., -9.81};
    ignition::math::Vector3d accel = imu.linkPtr_->RelativeLinearAccel() - pose.Rot().RotateVectorReverse(gravity);
    imu.linearAcc_[0] = accel.X();
    imu.linearAcc_[1] = accel.Y();
    imu.linearAcc_[2] = accel.Z();
  }

  // ---------------------------------------------------------------------------
  // Contact Sensor + diagnostic-only physical Gazebo contact force
  //
  // IMPORTANT:
  //   1. The original boolean contact logic is preserved.
  //   2. The wrench is read directly from Gazebo ContactManager.
  //   3. It is NEVER fed into MPC / WBC / RBF.
  //   4. This is privileged simulation ground truth for observability diagnosis.
  //
  // Gazebo ODE stores Contact::wrench body forces in each collision
  // LINK frame.  We explicitly rotate the selected robot-foot force back
  // into WORLD frame before accumulating the physical GRF.
  // ---------------------------------------------------------------------------

  for (auto& state : name2contact_) {
    state.second = false;
  }

  double directTotalFootFzWorld = 0.0;
  int directContactPointCount = 0;

  for (const auto& contact : contactManager_->GetContacts()) {
    if (static_cast<uint32_t>(contact->time.sec) != (time - period).sec ||
        static_cast<uint32_t>(contact->time.nsec) != (time - period).nsec) {
      continue;
    }

    const std::string linkName1 =
        contact->collision1->GetLink()->GetName();

    const std::string linkName2 =
        contact->collision2->GetLink()->GetName();

    const bool robotFootIsBody1 =
        name2contact_.find(linkName1) != name2contact_.end();

    const bool robotFootIsBody2 =
        name2contact_.find(linkName2) != name2contact_.end();

    // Preserve the production contact-boolean behavior.
    if (robotFootIsBody1) {
      name2contact_[linkName1] = true;
    }

    if (robotFootIsBody2) {
      name2contact_[linkName2] = true;
    }

    // Sum every contact point belonging to a robot foot.
    //
    // body1Force = force acting on collision1's body
    // body2Force = force acting on collision2's body
    //
    // Therefore this always selects the force ACTING ON THE ROBOT FOOT,
    // regardless of whether Gazebo happened to store the robot as body1
    // or body2.
    for (int k = 0; k < contact->count; ++k) {
      // IMPORTANT:
      // Gazebo ODE stores contact->wrench body1/body2 forces
      // in the corresponding LINK frame, not in world frame.
      //
      // Convert the force acting on the robot foot back to WORLD
      // before taking its vertical component.
      if (robotFootIsBody1) {
        const auto footForceWorld =
            contact->collision1->GetLink()
                ->WorldPose().Rot()
                .RotateVector(
                    contact->wrench[k].body1Force);

        directTotalFootFzWorld +=
            footForceWorld.Z();

        ++directContactPointCount;
      }

      if (robotFootIsBody2) {
        const auto footForceWorld =
            contact->collision2->GetLink()
                ->WorldPose().Rot()
                .RotateVector(
                    contact->wrench[k].body2Force);

        directTotalFootFzWorld +=
            footForceWorld.Z();

        ++directContactPointCount;
      }
    }
  }

  // ---------------------------------------------------------------------------
  // 1-second simulation-time mean.
  //
  // Contact force during trot is highly oscillatory, so instantaneous Fz is
  // not useful for comparing 0 kg vs +5 kg.  We accumulate it over ~1 s.
  // ---------------------------------------------------------------------------

  static ros::Time directDiagStart;
  static double directDiagFzSum = 0.0;
  static double directDiagContactPointSum = 0.0;
  static unsigned long long directDiagSamples = 0;

  // Handle startup / simulation reset.
  if (directDiagStart.isZero() || time < directDiagStart) {
    directDiagStart = time;
    directDiagFzSum = 0.0;
    directDiagContactPointSum = 0.0;
    directDiagSamples = 0;
  }

  directDiagFzSum += directTotalFootFzWorld;
  directDiagContactPointSum +=
      static_cast<double>(directContactPointCount);
  ++directDiagSamples;

  if ((time - directDiagStart).toSec() >= 1.0 &&
      directDiagSamples > 0) {

    const double meanFz =
        directDiagFzSum /
        static_cast<double>(directDiagSamples);

    const double meanContactPoints =
        directDiagContactPointSum /
        static_cast<double>(directDiagSamples);

    ROS_WARN_STREAM(
        "[GAZEBO_CONTACT_DIRECT]"
        << " t=" << time.toSec()
        << " mean_total_foot_fz_world=" << meanFz
        << " mean_contact_points=" << meanContactPoints
        << " samples=" << directDiagSamples);

    directDiagStart = time;
    directDiagFzSum = 0.0;
    directDiagContactPointSum = 0.0;
    directDiagSamples = 0;
  }

  // Set cmd to zero to avoid crazy soft limit oscillation when not controller loaded
  for (auto& cmd : joint_effort_command_) {
    cmd = 0;
  }
  for (auto& cmd : joint_velocity_command_) {
    cmd = 0;
  }
}

void LeggedHWSim::writeSim(ros::Time time, ros::Duration period) {
  int i=0;
  for (auto joint : hybridJointDatas_) {
    auto& buffer = cmdBuffer_.find(joint.joint_.getName())->second;
    if (time == ros::Time(period.toSec())) {  // Simulation reset
      buffer.clear();
    }

    while (!buffer.empty() && buffer.back().stamp_ + ros::Duration(delay_) < time) {
      buffer.pop_back();
    }
    buffer.push_front(HybridJointCommand{
        .stamp_ = time, .posDes_ = joint.posDes_, .velDes_ = joint.velDes_, .kp_ = joint.kp_, .kd_ = joint.kd_, .ff_ = joint.ff_});

    const auto& cmd = buffer.back();
    joint.joint_.setCommand(cmd.kp_ * (cmd.posDes_ - joint.joint_.getPosition()) + cmd.kd_ * (cmd.velDes_ - joint.joint_.getVelocity()) +
                            cmd.ff_);
    
    
    // if (i==2){
    //   std_msgs::Float64MultiArray kp_msg;
    //   kp_msg.data = {cmd.kp_,cmd.kd_,cmd.posDes_,cmd.velDes_,cmd.ff_};
    //   kp_pub.publish(kp_msg);
    // }
    // i++;
    // if (i==11){
    //   i=0;
    // }
  }
  DefaultRobotHWSim::writeSim(time, period);
}

void LeggedHWSim::parseImu(XmlRpc::XmlRpcValue& imuDatas, const gazebo::physics::ModelPtr& parentModel) {
  ROS_ASSERT(imuDatas.getType() == XmlRpc::XmlRpcValue::TypeStruct);
  for (auto it = imuDatas.begin(); it != imuDatas.end(); ++it) {
    if (!it->second.hasMember("frame_id")) {
      ROS_ERROR_STREAM("Imu " << it->first << " has no associated frame id.");
      continue;
    } else if (!it->second.hasMember("orientation_covariance_diagonal")) {
      ROS_ERROR_STREAM("Imu " << it->first << " has no associated orientation covariance diagonal.");
      continue;
    } else if (!it->second.hasMember("angular_velocity_covariance")) {
      ROS_ERROR_STREAM("Imu " << it->first << " has no associated angular velocity covariance.");
      continue;
    } else if (!it->second.hasMember("linear_acceleration_covariance")) {
      ROS_ERROR_STREAM("Imu " << it->first << " has no associated linear acceleration covariance.");
      continue;
    }
    XmlRpc::XmlRpcValue oriCov = imuDatas[it->first]["orientation_covariance_diagonal"];
    ROS_ASSERT(oriCov.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(oriCov.size() == 3);
    for (int i = 0; i < oriCov.size(); ++i) {
      ROS_ASSERT(oriCov[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    }
    XmlRpc::XmlRpcValue angularCov = imuDatas[it->first]["angular_velocity_covariance"];
    ROS_ASSERT(angularCov.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(angularCov.size() == 3);
    for (int i = 0; i < angularCov.size(); ++i) {
      ROS_ASSERT(angularCov[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    }
    XmlRpc::XmlRpcValue linearCov = imuDatas[it->first]["linear_acceleration_covariance"];
    ROS_ASSERT(linearCov.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(linearCov.size() == 3);
    for (int i = 0; i < linearCov.size(); ++i) {
      ROS_ASSERT(linearCov[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    }

    std::string frameId = imuDatas[it->first]["frame_id"];
    gazebo::physics::LinkPtr linkPtr = parentModel->GetLink(frameId);
    ROS_ASSERT(linkPtr != nullptr);
    imuDatas_.push_back((ImuData{
        .linkPtr_ = linkPtr,
        .ori_ = {0., 0., 0., 0.},
        .oriCov_ = {static_cast<double>(oriCov[0]), 0., 0., 0., static_cast<double>(oriCov[1]), 0., 0., 0., static_cast<double>(oriCov[2])},
        .angularVel_ = {0., 0., 0.},
        .angularVelCov_ = {static_cast<double>(angularCov[0]), 0., 0., 0., static_cast<double>(angularCov[1]), 0., 0., 0.,
                           static_cast<double>(angularCov[2])},
        .linearAcc_ = {0., 0., 0.},
        .linearAccCov_ = {static_cast<double>(linearCov[0]), 0., 0., 0., static_cast<double>(linearCov[1]), 0., 0., 0.,
                          static_cast<double>(linearCov[2])}}));
    ImuData& imuData = imuDatas_.back();
    imuSensorInterface_.registerHandle(hardware_interface::ImuSensorHandle(it->first, frameId, imuData.ori_, imuData.oriCov_,
                                                                           imuData.angularVel_, imuData.angularVelCov_, imuData.linearAcc_,
                                                                           imuData.linearAccCov_));
  }
}

void LeggedHWSim::parseContacts(XmlRpc::XmlRpcValue& contactNames) {
  ROS_ASSERT(contactNames.getType() == XmlRpc::XmlRpcValue::TypeArray);
  for (int i = 0; i < contactNames.size(); ++i) {  // NOLINT(modernize-loop-convert)
    std::string name = contactNames[i];
    name2contact_.insert(std::make_pair(name, false));
    contactSensorInterface_.registerHandle(ContactSensorHandle(name, &name2contact_[name]));
  }
  registerInterface(&contactSensorInterface_);
}

}  // namespace legged

PLUGINLIB_EXPORT_CLASS(legged::LeggedHWSim, gazebo_ros_control::RobotHWSim)
GZ_REGISTER_MODEL_PLUGIN(legged::LoggedGazeboRosControlPlugin)
