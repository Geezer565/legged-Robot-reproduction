#include <pinocchio/fwd.hpp>
#include <Eigen/LU>
#include "legged_controllers/LeggedController.h"
#include "legged_controllers/ExactPayloadTruth.h"
#include <ocs2_legged_robot_adaptive/adaptive/AdaptiveReferenceKinematics.h>
#include <ocs2_legged_robot_adaptive/adaptive/AdaptiveTypes.h>
#include <ros/package.h>
#include <legged_interface/adaptive/MyRbfV11Model.h>
#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>

namespace legged {
namespace {
template<class V> void parameter(ros::NodeHandle& nh,const std::string& key,V& destination){
  std::vector<double> values;
  if(!nh.getParam("myrbf_shadow/"+key,values) || values.size()!=destination.size())
    throw std::invalid_argument("V1.1 missing/malformed parameter: "+key);
  for(int i=0;i<destination.size();++i)destination(i)=values[i];
}
std::vector<std::string> columns(){
  std::vector<std::string> c={"time","diagnostic_sequence","diagnostic_publish_drops","v11_enabled",
      "actual_update_dt","prediction_valid","learning_enabled","projected","snapshot_sequence_id",
      "mpc_snapshot_id","wbc_snapshot_id","solve_id","contact_mode","clf_ec_value","clf_ec_violation",
      "clf_ec_slack","wbc_status","snapshot_published","nominal_mass","gamma_ratio"};
  for(const std::string group:{"velocity","reference_velocity","predicted_velocity","ee","filtered_ee","er","ec","xi","xi_self","torque_cross_xi",
      "learning_gradient","filtered_learning_gradient","effective_learning_gradient","equivalent_wrench_body","active_equivalent_wrench_body",
      "reconstructed_reference_body",
      "nominal_dc_residual_body",
      "corrected_dynamics_reference_body",
      "filtered_dynamics_reference_body",
      "dynamics_error_body",
      "external_equivalent_world_base",
      "nominal_rate","prediction_rate","reference_rate"})
    for(const std::string axis:{"x","y","z","rx","ry","rz"})c.push_back(group+"_"+axis);
  for(const std::string group:{"mpc_contact_force","wbc_contact_force"})
    for(int i=0;i<12;++i)c.push_back(group+"_"+std::to_string(i));

  // ----------------------------------------------------------
  // Joint-side diagnostic only.
  // These quantities DO NOT enter the RBF / MPC / WBC update.
  //
  // joint_effort_measured:
  //   Effort read back through HybridJointHandle / Gazebo.
  //
  // joint_torque_ff:
  //   WBC feed-forward torque.
  //
  // joint_torque_pd:
  //   Low-level KD contribution:
  //       kd * (vel_des - vel_measured)
  //   Current V1.1 uses kp=0, kd=3.
  //
  // joint_torque_command:
  //   Total torque sent to the simulated actuator:
  //       torque_ff + torque_pd
  // ----------------------------------------------------------
  for(const std::string group:{
        "joint_effort_measured",
        "joint_torque_ff",
        "joint_torque_pd",
        "joint_torque_command"})
    for(int i=0;i<12;++i)
      c.push_back(group+"_"+std::to_string(i));

  for(const std::string group:{"weight","feature"})
    for(int i=0;i<6;++i)for(int j=0;j<26;++j)c.push_back(group+"_"+std::to_string(i)+"_"+std::to_string(j));
  for(const std::string axis:
      {"fx","fy","fz","tx","ty","tz"})
    c.push_back("true_yupi_world_com_"+axis);
  return c;
}
}
void LeggedController::configureMyRbfV11(ros::NodeHandle& nh){
  adaptive::v11::Settings s;

  nh.param(
      "myrbf_shadow/v11_enable_torque_coupled_feature",
      s.torqueCoupledFeature,
      false);

  nh.param(
      "myrbf_shadow/v11_torque_feature_angular_ratio",
      s.torqueFeatureAngularRatio,
      0.30);

  // Use the same nominal geometry as the rest of the Go2 model.
  // Do not duplicate the lever arm in the adaptive YAML.
  std::vector<double> v11ComToBaseBody;

  if(!nh.getParam(
         "/go2_nominal_model/com_to_base_body",
         v11ComToBaseBody) ||
     v11ComToBaseBody.size()!=3)
    throw std::invalid_argument(
        "missing/malformed /go2_nominal_model/com_to_base_body");

  s.comToBaseBody <<
      v11ComToBaseBody[0],
      v11ComToBaseBody[1],
      v11ComToBaseBody[2];
  if(!nh.getParam("myrbf_shadow/v11_composite_ratio",s.ratio))throw std::invalid_argument("missing V1.1 ratio");
  parameter(nh,"v11_gradient_filter_tau",s.gradientFilterTau);
  parameter(nh,"v11_input_scale",s.scale);parameter(nh,"v11_gamma",s.gamma);
  parameter(nh,"v11_gradient_deadzone",s.gradientDeadzone);parameter(nh,"v11_leakage",s.leakage);
  parameter(nh,"v11_weight_limit",s.weightLimit);parameter(nh,"v11_clf_decay",s.decay);
  parameter(nh,"v11_centers",s.centers);parameter(nh,"v11_widths",s.widths);

  nh.param(
      "myrbf_shadow/v12_enable_dynamics_residual",
      s.dynamicsResidualEnabled,
      false);

  parameter(
      nh,
      "v12_dynamics_residual_filter_tau",
      s.dynamicsResidualFilterTau);

  parameter(
      nh,
      "v12_dynamics_bias_gain",
      s.dynamicsBiasGain);

  parameter(
      nh,
      "v12_bias_weight_limit",
      s.biasWeightLimit);
  for(const std::string key:{"v5_gamma","v6_gain_multiplier_max","prediction_gain","leakage","output_scale",
      "residual_bias","residual_deadzone","weight_gamma"})
    if(nh.hasParam("myrbf_shadow/"+key))throw std::invalid_argument("legacy option forbidden in V1.1: "+key);
  // ==========================================================
  // V1.2-B nominal DC residual cancellation.
  // ==========================================================
  nh.param(
      "myrbf_shadow/v12_enable_nominal_dc_cancellation",
      myRbfV12NominalDcCancellation_,
      false);

  parameter(
      nh,
      "v12_nominal_dc_residual",
      myRbfV12NominalDcResidual_);

  myRbfV11Core_=std::make_unique<adaptive::v11::Core>(s);
  nh.param("myrbf_shadow/activation_warmup",myRbfV2ActivationWarmup_,1.);
  if(!std::isfinite(myRbfV2ActivationWarmup_) || myRbfV2ActivationWarmup_<0)throw std::invalid_argument("invalid V1.1 warmup");
  myRbfV2ResidualModel_=std::make_unique<PinocchioInterface>(leggedInterface_->getPinocchioInterface());
  myRbfV2WbcInput_.enabled=true;
  myRbfV2WbcInput_.enforceClfConstraint=false; // soft Ec-CLF is in MPC, NOT the legacy sigma row
  myRbfV2WbcInput_.injectExternalWrenchInDynamics=true;
  myRbfV2WbcInput_.v11MatchedAcceleration=true;
  // One nominal standing cycle supplies the actual initial contact-force boundary.
  myRbfV2WbcInput_.enabled=false;
  myRbfV2WbcInput_.robustBound.setZero();
  ros::NodeHandle node;node.setParam("/myrbf_v11_columns",columns());
  myRbfV11Publisher_=std::make_unique<realtime_tools::RealtimePublisher<std_msgs::Float64MultiArray>>(
      node,"/myrbf_v11_diagnostics",100);
  myRbfV11Publisher_->msg_.data.resize(columns().size());
}
void LeggedController::updateMyRbfV11(const ros::Time& time,double dt,const vector_t& stateDesired,
                                   const vector_t& inputDesired,std::size_t mode){
  using namespace adaptive::v11;
  const auto active=myRbfV2PolicyBinding_->active();
  if(!active || !active->valid || !active->v11Enabled)throw std::runtime_error("V1.1 policy/snapshot mismatch");
  const auto& info=leggedInterface_->getCentroidalModelInfo();
  const double t=currentObservation_.time;
  const auto& x=currentObservation_.state;
  const Vector6 v=velocity(x,info);
  const auto R=rotation(x.segment<3>(9));
  const Matrix6 G=mapping(info.robotMass,info.centroidalInertiaNominal,R);
  const auto& target=mpcMrtInterface_->getCommand().mpcTargetTrajectories_;

  // ==========================================================
  // Exact Y_u*pi_u truth evaluator for the +5 kg experiment.
  //
  // This DOES NOT enter the RBF control law.
  // It only evaluates the paper ETH regressor on the RBF
  // closed-loop trajectory, so the ground truth is:
  //
  //   d_u^true = Y_in(q,v,v_r,dot(v_r)) * pi_in^true
  //
  // using the same ETH reference convention.
  // ==========================================================
  namespace eth_truth =
      ocs2::legged_robot_adaptive::adaptive;

  static const eth_truth::AdaptiveClfSettings truthSettings = []() {
    const std::string packagePath =
        ros::package::getPath("ocs2_legged_robot_adaptive");

    if (packagePath.empty()) {
      throw std::runtime_error(
          "cannot resolve ocs2_legged_robot_adaptive "
          "for exact Y*pi truth");
    }

    return eth_truth::loadAdaptiveClfSettings(
        packagePath +
        "/config/adaptive_settings_eth_paper_full_16d.info");
  }();

  static std::unique_ptr<
      eth_truth::AdaptiveReferenceKinematicsAdapter>
      exactTruthReferenceAdapter;

  if (!exactTruthReferenceAdapter) {
    exactTruthReferenceAdapter =
        std::make_unique<
            eth_truth::AdaptiveReferenceKinematicsAdapter>(
                leggedInterface_->getPinocchioInterface(),
                info,
                truthSettings);
  }

  const auto exactTruthReference =
      exactTruthReferenceAdapter->compute(
          t,
          x,
          target);

  const eth_truth::vector6_t trueYpiWorldCom =
      exact_payload_truth::trueYpiWorldCom(
          exactTruthReference);

  Vector6 ref,refRate;reference(target,t,info,ref,refRate);
  const double elapsed=myRbfV11PredictionValid_?t-myRbfV11PreviousTime_:0.;
  if(myRbfV11PredictionValid_ && (!(elapsed>0) || elapsed>.1))throw std::runtime_error("V1.1 prediction timestamp gap");
  Vector6 predicted=v,ee=Vector6::Zero(),reconstructed=Vector6::Zero();
  if(myRbfV11PredictionValid_){
    predicted=myRbfV11PreviousVelocity_+elapsed*myRbfV11PreviousRate_;
    ee=v-predicted;
    // V1.2-A online dynamics reconstruction.
    // This uses nominal model + measured motion only.
    // It is NOT Gazebo truth and NOT true Y*pi.
    reconstructed=myRbfV11PreviousG_.fullPivLu().solve(
        myRbfV11PreviousNominalRate_-(v-myRbfV11PreviousVelocity_)/elapsed);
  }
  // ==========================================================
  // V1.2-B corrected dynamics teacher.
  //
  // raw reconstruction:
  //   d_rec = G^{-1}(vdot_nominal - vdot_measured)
  //
  // corrected teacher:
  //   d_teacher = d_rec - d_nominal_dc
  //
  // All signals use:
  // BODY / nominal CoM / left-hand wrench convention.
  // ==========================================================
  Vector6 correctedDynamicsReference=reconstructed;

  if(myRbfV12NominalDcCancellation_ &&
     myRbfV11PredictionValid_){

    correctedDynamicsReference=
        reconstructed-
        myRbfV12NominalDcResidual_;
  }

  const bool learning=myRbfV11PredictionValid_ && t-myRbfV2ActivationStartTime_>=myRbfV2ActivationWarmup_;
  const auto estimate=
      myRbfV11Core_->update(
          ee,
          v-ref,
          G,
          correctedDynamicsReference,
          myRbfV11PreviousActiveDisturbance_,
          myRbfV11PredictionValid_,
          elapsed,
          learning);
  adaptive::MyRbfV2Snapshot pending;
  pending.valid=true;pending.v11Enabled=true;pending.timestamp=t;pending.contactMode=mode;
  pending.sequenceId=++myRbfV2Sequence_;
  pending.v11Disturbance=estimate.d;pending.v11Settings=myRbfV11Core_->settings();
  pending.v11PredictionValid=learning;pending.v11PredictionTime=myRbfV11PreviousTime_;
  pending.v11PredictionOrigin=myRbfV11PreviousVelocity_;pending.v11PredictionRate=myRbfV11PreviousRate_;
  pending.v11Target=target;
  const bool published=myRbfV2Snapshots_->tryPublish(pending);

  // Use the estimate bound to the ACTIVE solved policy, never a pending update.
  Vector6 external;
  external.head<3>()=-R*active->v11Disturbance.head<3>();
  const Eigen::Vector3d baseToCom=-R*info.comToBasePositionNominal;
  external.tail<3>()=-R*active->v11Disturbance.tail<3>()+baseToCom.cross(external.head<3>());
  myRbfV2WbcInput_.snapshotSequenceId=active->sequenceId;
  myRbfV2WbcInput_.enabled=myRbfV11PredictionValid_;
  myRbfV2WbcInput_.externalWrenchEstimate=external;
  myRbfV2WbcInput_.v11LeftHandBodyComWrench=active->v11Disturbance;
  wbc_->configureMyRbfV2(myRbfV2WbcInput_);
  const auto solution=wbc_->update(stateDesired,inputDesired,measuredRbdState_,mode,dt);
  if(solution.size()!=info.generalizedCoordinatesNum+3*info.numThreeDofContacts+info.actuatedDofNum || !solution.allFinite())
    throw std::runtime_error("V1.1 WBC did not produce a valid solution");
  const auto forces=solution.segment(info.generalizedCoordinatesNum,12).eval();
  currentObservation_.input=inputDesired;
  if(!safetyChecker_->check(currentObservation_,stateDesired,inputDesired))throw std::runtime_error("V1.1 state safety check failed");
  const auto pos=centroidal_model::getJointAngles(stateDesired,info);
  const auto vel=centroidal_model::getJointVelocities(inputDesired,info);
  const auto torque=solution.tail(info.actuatedDofNum).eval();

  // ==========================================================
  // Joint-side observability diagnostics.
  //
  // IMPORTANT:
  // These variables are logging-only.  They do not feed back
  // into the adaptive law or controller.
  //
  // getEffort() is the effort state provided by the underlying
  // JointStateHandle.  In Gazebo this originates from
  // Joint::GetForce(0) in LeggedHWSim::readSim().
  //
  // Because readSim() occurs before this controller cycle,
  // jointEffortMeasured is expected to correspond approximately
  // to the actuator effort from the previous physics/control
  // step.  We will check this one-step relationship offline.
  // ==========================================================
  vector_t jointEffortMeasured(info.actuatedDofNum);
  vector_t jointPdTorque(info.actuatedDofNum);
  vector_t jointTorqueCommand(info.actuatedDofNum);

  for(std::size_t i=0;i<info.actuatedDofNum;++i){
    const double measuredJointVelocity =
        hybridJointHandles_[i].getVelocity();

    jointEffortMeasured(i) =
        hybridJointHandles_[i].getEffort();

    // Current V1.1 command:
    // kp = 0, kd = 3, ff = WBC torque.
    jointPdTorque(i) =
        3.0 * (vel(i) - measuredJointVelocity);

    jointTorqueCommand(i) =
        torque(i) + jointPdTorque(i);

    hybridJointHandles_[i].setCommand(
        pos(i),
        vel(i),
        0,
        3,
        torque(i));
  }
  myRbfV2WbcInput_.previousContactForces=forces;
  const Vector6 nominal=nominalRate(*myRbfV2ResidualModel_,info,x,inputDesired);
  const Vector6 rate=nominal-G*active->v11Disturbance;
  double h=0;
  if(active->v11PredictionValid){
    Vector6 av,ar;reference(active->v11Target,t,info,av,ar);Evaluation ae;
    Vector6 ap,apRate;
    oneStepPrediction(active->v11PredictionOrigin,active->v11PredictionRate,active->v11PredictionTime,
                      active->timestamp,t,ap,apRate);
    ae.ee=v-ap;
    ae.er=v-av;ae.ec=active->v11Settings.ratio*ae.ee+(1-active->v11Settings.ratio)*ae.er;
    h=clf(ae,rate,apRate,ar,active->v11Settings);
  }
  ++myRbfV2DiagnosticSequence_;
  if(myRbfV11Publisher_->trylock()){
    auto& data=myRbfV11Publisher_->msg_.data;std::size_t n=0;
    const auto add=[&](double z){data.at(n++)=z;};
    for(double z:{t,double(myRbfV2DiagnosticSequence_),double(myRbfV2DiagnosticDrops_),1.,elapsed,
        double(myRbfV11PredictionValid_),double(learning),double(estimate.projected),double(pending.sequenceId),
        double(active->sequenceId),double(wbc_->getMyRbfV2Diagnostics().snapshotSequenceId),double(myRbfV2PolicyBinding_->activeSolveId()),double(mode),
        h,std::max(0.,-h),std::max(0.,-h),double(wbc_->getMyRbfV2Diagnostics().status),double(published),info.robotMass,
        myRbfV11Core_->settings().ratio})add(z);
    for(const Vector6& group:{v,ref,predicted,estimate.ee,estimate.eeFiltered,estimate.er,estimate.ec,estimate.xi,estimate.xiSelf,estimate.torqueCrossXi,estimate.gradient,
        estimate.filteredGradient,
        estimate.effectiveGradient,
        estimate.d,
        active->v11Disturbance,
        reconstructed,
        myRbfV12NominalDcResidual_,
        correctedDynamicsReference,
        estimate.filteredDynamicsReference,
        estimate.dynamicsError,
        external,
        nominal,
        rate,
        refRate})for(int i=0;i<6;++i)add(group(i));
    for(int i=0;i<12;++i)add(inputDesired(i));
    for(int i=0;i<12;++i)add(forces(i));

    // Joint-side diagnostics: must exactly match columns() order.
    for(int i=0;i<12;++i)add(jointEffortMeasured(i));
    for(int i=0;i<12;++i)add(torque(i));
    for(int i=0;i<12;++i)add(jointPdTorque(i));
    for(int i=0;i<12;++i)add(jointTorqueCommand(i));

    for(const auto& group:{myRbfV11Core_->weights(),estimate.features})for(int i=0;i<6;++i)for(int j=0;j<26;++j)add(group(i,j));
    for(int i=0;i<6;++i)add(trueYpiWorldCom(i));
    myRbfV11Publisher_->unlockAndPublish();
  }else ++myRbfV2DiagnosticDrops_;
  myRbfV11PreviousVelocity_=v;
  myRbfV11PreviousRate_=rate;
  myRbfV11PreviousNominalRate_=nominal;

  // This estimate was actually active during the interval ending now.
  // It is used on the next update to construct time-aligned e_d.
  myRbfV11PreviousActiveDisturbance_=
      active->v11Disturbance;

  myRbfV11PreviousG_=G;
  myRbfV11PreviousTime_=t;
  myRbfV11PredictionValid_=true;
  observationPublisher_.publish(ros_msg_conversions::createObservationMsg(currentObservation_));
  robotVisualizer_->update(currentObservation_,mpcMrtInterface_->getPolicy(),mpcMrtInterface_->getCommand());
}
}
