#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/data.hpp>
#include "legged_controllers/LeggedController.h"
#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <stdexcept>
#include <cmath>
#include <sstream>

namespace legged {
namespace {
adaptive::v2::Axes axes(const WbcVector6& x) {
  adaptive::v2::Axes a{}; for(int i=0;i<6;++i)a[i]=x(i);return a;
}
WbcVector6 vector6(const adaptive::v2::Axes& a) {
  WbcVector6 x;for(int i=0;i<6;++i)x(i)=a[i];return x;
}
WbcVector6 parameter6(ros::NodeHandle& nh,const std::string& name,const WbcVector6& fallback) {
  std::vector<double> a;
  if(!nh.hasParam("myrbf_shadow/"+name))return fallback;
  if(!nh.getParam("myrbf_shadow/"+name,a))throw std::invalid_argument(name+" requires a numeric vector");
  if(a.size()!=6)throw std::invalid_argument(name+" requires six entries");
  WbcVector6 x;for(int i=0;i<6;++i)x(i)=a[i];
  if(!x.allFinite())throw std::invalid_argument(name+" must be finite");
  return x;
}
std::vector<std::string> columns() {
  std::vector<std::string> names={"time","snapshot_sequence_id","mpc_snapshot_id","wbc_snapshot_id",
    "solve_id","contact_mode","feature_mode","robust_mode","clf_lhs","clf_rhs","clf_violation",
    "clf_feasible","wbc_status","solver_return_code","contact_wrench_rank","snapshot_age_s",
    "hold_mismatch_power","torque_limit_margin","friction_margin","normal_force_margin",
    "contact_force_margin","contact_force_rate_margin","max_equality_violation","max_inequality_violation",
    "diagnostic_sequence","diagnostic_publish_drops","desired_acceleration_initialized","reference_wrench_is_final",
    "v2_armed","activation_elapsed_s","v3_teacher_enabled","v4_convergence_enabled","v3_previous_policy_valid",
    "v3_composite_ratio","v3_robot_mass","v3_actual_update_dt"};
  const std::vector<std::string> suffix={"fx","fy","fz","tx","ty","tz"};
  for(const auto& group:std::vector<std::string>{"chi","external_wrench_estimate","left_hand_disturbance_estimate",
      "reference_wrench","robust_bound","robust_wrench","smoothing_width","final_contact_wrench",
      "external_residual","known_applied_compensation","v3_prediction_error",
      "v3_tracking_error","v3_composite_error"})
    for(const auto& s:suffix)names.push_back(group+"_"+s);
  for(const auto& group:std::vector<std::string>{"v3_prediction_state_error",
      "v3_tracking_state_error","v3_composite_state_error"})
    for(int i=0;i<24;++i)names.push_back(group+"_"+std::to_string(i));
  for(int i=0;i<12;++i)names.push_back("final_contact_force_"+std::to_string(i));
  for(int i=0;i<6;++i)for(int j=0;j<26;++j)names.push_back("weight_"+std::to_string(i)+"_"+std::to_string(j));
  for(int i=0;i<6;++i)for(int j=0;j<26;++j)names.push_back("feature_"+std::to_string(i)+"_"+std::to_string(j));
  return names;
}
}

void LeggedController::configureMyRbfV2Lyapunov(ros::NodeHandle& nh) {
  // A present but malformed parameter must never silently select a fallback.
  for(const std::string key:{"residual_scale","output_scale","prediction_gain","leakage","weight_limit",
      "residual_bias","residual_deadzone",
      "weight_gamma","robust_gamma","v2_gamma","v2_robust_gamma","v2_nu0","v2_nu_time","v2_nu_exponent",
      "control_gain_vector","clf_damping"})parameter6(nh,key,WbcVector6::Zero());
  for(const std::string key:{"clf_enabled","clf_hard_constraint","mpc_model_injection",
      "v3_shared_allocator","normalize_update","tracking_bias_only"}) {
    bool value;
    if(nh.hasParam("myrbf_shadow/"+key) && !nh.getParam("myrbf_shadow/"+key,value))
      throw std::invalid_argument(key+" requires true or false");
  }
  for(const std::string key:{"minimum_coverage","lambda_linear","lambda_angular","update_period","time_tolerance",
      "filter_alpha","linear_extent","angular_extent","width_ratio","update_epsilon","smooth_width","clf_tolerance",
      "minimum_normal_force","maximum_normal_force","maximum_contact_force","maximum_contact_force_rate",
      "activation_warmup","v3_composite_ratio","composite_window"}) {
    double value;
    if(nh.hasParam("myrbf_shadow/"+key) && (!nh.getParam("myrbf_shadow/"+key,value) || !std::isfinite(value)))
      throw std::invalid_argument(key+" requires a finite number");
  }
  for(const std::string key:{"feature_mode","robust_mode"}) {
    std::string value;
    if(nh.hasParam("myrbf_shadow/"+key) && !nh.getParam("myrbf_shadow/"+key,value))
      throw std::invalid_argument(key+" requires a mode name");
  }
  bool clf=false,hard=false,injection=false;
  nh.param("myrbf_shadow/clf_enabled",clf,false);
  nh.param("myrbf_shadow/clf_hard_constraint",hard,false);
  nh.param("myrbf_shadow/mpc_model_injection",injection,false);
  nh.param("myrbf_shadow/v3_shared_allocator",myRbfV3SharedAllocator_,false);
  const bool sharedAdaptive=myRbfV3Teacher_ || myRbfV4Convergence_;
  if(sharedAdaptive) {
    if(clf || hard)
      throw std::invalid_argument("V3/V4 must not enable the legacy V2 hard CLF constraint");
    if(myRbfV3SharedAllocator_ == injection)
      throw std::invalid_argument("V3/V4 must select exactly one correction delivery path");
    if(myRbfV3SharedAllocator_ && myRbfControlInterface_ != "wrench_allocator")
      throw std::invalid_argument("V3/V4 shared comparison requires the wrench allocator");
  } else if(!injection || !clf || !hard) {
    throw std::invalid_argument("V2 Lyapunov requires model injection and hard CLF");
  }
  if((!sharedAdaptive || !myRbfV3SharedAllocator_) &&
      !myRbfControlGainVector_.isApprox(WbcVector6::Ones(),1e-12)) {
    std::ostringstream message;
    message << "model-injected V2/V3 requires unit output gain; resolved value="
            << myRbfControlGainVector_.transpose();
    throw std::invalid_argument(message.str());
  }
  const auto predictionGain=parameter6(nh,"prediction_gain",WbcVector6::Zero());
  if(!myRbfV3Teacher_ && !predictionGain.isZero(0))
    throw std::invalid_argument("V2 Lyapunov prediction labels have not been certified");
  if((predictionGain.array()<0).any())
    throw std::invalid_argument("V3 integral-history gains must be nonnegative");
  const auto leakage=parameter6(nh,"leakage",WbcVector6::Zero());
  if(!myRbfV4Convergence_ && !leakage.isZero(0))
    throw std::invalid_argument("V2 Lyapunov forbids leakage");
  if((leakage.array()<0).any())
    throw std::invalid_argument("V4 convergence leakage must be nonnegative");
  bool normalized=false,biasOnly=false;
  double coverage=0;
  nh.param("myrbf_shadow/normalize_update",normalized,false);
  nh.param("myrbf_shadow/tracking_bias_only",biasOnly,false);
  nh.param("myrbf_shadow/minimum_coverage",coverage,0.0);
  if(!myRbfV4Convergence_ && (normalized || biasOnly || coverage!=0))
    throw std::invalid_argument("V2 Lyapunov requires all-node updates without normalization or coverage gating");
  if(myRbfV4Convergence_ && (!normalized || biasOnly || coverage!=0))
    throw std::invalid_argument("V4 requires normalized all-node updates without coverage gating");
  nh.param("myrbf_shadow/feature_mode",myRbfV2FeatureMode_,std::string("contact_aware"));
  nh.param("myrbf_shadow/robust_mode",myRbfV2RobustMode_,std::string("sign"));
  if(myRbfV2FeatureMode_!="contact_aware" && myRbfV2FeatureMode_!="residual_only")throw std::invalid_argument("Unknown V2 feature mode");
  if(myRbfV2RobustMode_!="sign" && myRbfV2RobustMode_!="smooth")throw std::invalid_argument("Unknown V2 robust mode");
  nh.param("myrbf_shadow/v3_composite_ratio",myRbfV3CompositeRatio_,0.75);
  if(myRbfV3Teacher_ && (!(myRbfV3CompositeRatio_>0.) || !(myRbfV3CompositeRatio_<1.)))
    throw std::invalid_argument("V3 composite ratio must be strictly between zero and one");
  if((myRbfV3Teacher_ || myRbfV4Convergence_) && myRbfV2FeatureMode_!="residual_only")
    throw std::invalid_argument("V3/V4 require one-dimensional residual-only RBF inputs");
  myRbfV2WbcInput_.enabled=true;
  myRbfV2WbcInput_.enforceClfConstraint=clf && hard;
  myRbfV2WbcInput_.injectExternalWrenchInDynamics=!(myRbfV3Teacher_ || myRbfV4Convergence_);
  myRbfV2WbcInput_.robustMode=myRbfV2RobustMode_=="sign" ? MyRbfV2RobustMode::Sign : MyRbfV2RobustMode::Smooth;
  myRbfV2WbcInput_.damping=parameter6(nh,"clf_damping",WbcVector6::Constant(3));
  if((myRbfV2WbcInput_.damping.array()<=0).any())
    throw std::invalid_argument("V2 damping must be strictly positive on all six axes");
  myRbfV2WbcInput_.lambda << myRbfShadowLambdaLinear_,myRbfShadowLambdaLinear_,myRbfShadowLambdaLinear_,
      myRbfShadowLambdaAngular_,myRbfShadowLambdaAngular_,myRbfShadowLambdaAngular_;
  nh.param("myrbf_shadow/smooth_width",myRbfV2WbcInput_.smoothWidth,0.08);
  nh.param("myrbf_shadow/clf_tolerance",myRbfV2WbcInput_.feasibilityTolerance,1e-5);
  nh.param("myrbf_shadow/activation_warmup",myRbfV2ActivationWarmup_,1.0);
  if(myRbfV2ActivationWarmup_<0 || !std::isfinite(myRbfV2ActivationWarmup_))
    throw std::invalid_argument("V2 activation_warmup must be finite and nonnegative");
  nh.param("myrbf_shadow/minimum_normal_force",myRbfV2WbcInput_.minNormalForce,0.0);
  nh.param("myrbf_shadow/maximum_normal_force",myRbfV2WbcInput_.maxNormalForce,350.0);
  double force=350,rate=100000;
  nh.param("myrbf_shadow/maximum_contact_force",force,350.0);
  nh.param("myrbf_shadow/maximum_contact_force_rate",rate,100000.0);
  myRbfV2WbcInput_.maxAbsContactForce.setConstant(force);
  myRbfV2WbcInput_.maxContactForceRate.setConstant(rate);
  std::vector<double> surfaceNormal;
  if(nh.hasParam("myrbf_shadow/surface_normal_world")) {
    if(!nh.getParam("myrbf_shadow/surface_normal_world",surfaceNormal))
      throw std::invalid_argument("surface_normal_world requires a numeric vector");
    if(surfaceNormal.size()!=3)throw std::invalid_argument("surface_normal_world requires three entries");
    for(int i=0;i<3;++i)myRbfV2WbcInput_.surfaceNormal(i)=surfaceNormal[i];
    if(!myRbfV2WbcInput_.surfaceNormal.allFinite() || myRbfV2WbcInput_.surfaceNormal.norm()<1e-9)
      throw std::invalid_argument("surface normal must be finite and nonzero");
    myRbfV2WbcInput_.surfaceNormal.normalize();
  }
  myRbfV2ResidualModel_=std::make_unique<PinocchioInterface>(leggedInterface_->getPinocchioInterface());
  const auto names=columns();
  ros::NodeHandle node;
  node.setParam("/myrbf_v2_lyapunov_columns",names);
  myRbfV2Publisher_=std::make_unique<realtime_tools::RealtimePublisher<std_msgs::Float64MultiArray>>(
      node,"/myrbf_v2_lyapunov_diagnostics",100);
  myRbfV2Publisher_->msg_.data.resize(names.size(),0);
}

bool LeggedController::updateMyRbfV2Warmup(const ros::Time& time,double dt,
    const vector_t& stateDesired,const vector_t& inputDesired,std::size_t mode) {
  const double elapsed=currentObservation_.time-myRbfV2ActivationStartTime_;
  if(elapsed>=myRbfV2ActivationWarmup_)return false;
  const auto active=myRbfV2PolicyBinding_->active();
  if(!active || !active->valid)throw std::runtime_error("V2 warmup has no active policy snapshot");
  auto disabled=myRbfV2WbcInput_;
  disabled.enabled=false;
  wbc_->configureMyRbfV2(disabled);
  wbcTimer_.startTimer();
  const vector_t solution=wbc_->update(stateDesired,inputDesired,measuredRbdState_,mode,dt);
  wbcTimer_.endTimer();
  const auto& info=leggedInterface_->getCentroidalModelInfo();
  const int expected=info.generalizedCoordinatesNum+3*info.numThreeDofContacts+info.actuatedDofNum;
  if(solution.size()!=expected || !solution.allFinite())throw std::runtime_error("nominal WBC failed during V2 warmup");
  vector_t finalInput=inputDesired;
  const vector_t forces=solution.segment(info.generalizedCoordinatesNum,3*info.numThreeDofContacts);
  finalInput.head(3*info.numThreeDofContacts)=forces;
  currentObservation_.input=finalInput;
  if(!safetyChecker_->check(currentObservation_,stateDesired,finalInput))
    throw std::runtime_error("state safety check failed during V2 warmup");
  myRbfV2WbcInput_.previousContactForces=forces;
  myRbfV2MomentumValid_=false;
  const vector_t pos=centroidal_model::getJointAngles(stateDesired,info);
  const vector_t vel=centroidal_model::getJointVelocities(finalInput,info);
  const vector_t torque=solution.tail(info.actuatedDofNum);
  for(std::size_t j=0;j<info.actuatedDofNum;++j)
    hybridJointHandles_[j].setCommand(pos(j),vel(j),0,3,torque(j));
  MyRbfV2WbcDiagnostics warmup;
  warmup.contactMode=mode;
  warmup.snapshotSequenceId=active->sequenceId;
  warmup.status=MyRbfV2WbcStatus::NotSolved;
  warmup.finalContactForces=forces;
  publishMyRbfV2Lyapunov(warmup,*active,0);
  robotVisualizer_->update(currentObservation_,mpcMrtInterface_->getPolicy(),mpcMrtInterface_->getCommand());
  selfCollisionVisualization_->update(currentObservation_);
  observationPublisher_.publish(ros_msg_conversions::createObservationMsg(currentObservation_));
  return true;
}

void LeggedController::updateMyRbfV2Control(const ros::Time& time,double dt,
    const vector_t& stateDesired,const vector_t& inputDesired,std::size_t mode) {
  // Terminal failure is explicitly outside the valid controller. Clear the last
  // command so stopping never silently leaves an old feasible torque applied.
  const auto failStop=[&]{
    for(auto& joint:hybridJointHandles_)joint.setCommand(0,0,0,0,0);
    stopRequest(time);
  };
  const auto active=myRbfV2PolicyBinding_->active();
  if(!active || !active->valid) {
    ROS_ERROR("[V2] No snapshot attached to active MPC policy");failStop();return;
  }
  const auto& info=leggedInterface_->getCentroidalModelInfo();
  vector_t deliveredInput=inputDesired;
  if((myRbfV3Teacher_ || myRbfV4Convergence_) && myRbfV3SharedAllocator_) {
    const auto delivery=adaptive::resolveV3ControlDelivery(*active,true);
    myRbfAdaptiveWrench_=vector6(delivery.allocatorCorrection);
    applyMyRbfWrenchCompensation(deliveredInput,mode,dt);
  }
  myRbfV2WbcInput_.snapshotSequenceId=active->sequenceId;
  myRbfV2WbcInput_.externalWrenchEstimate=vector6(active->externalWrenchEstimate);
  myRbfV2WbcInput_.robustBound=vector6(active->robustBound);
  if(!(myRbfV3Teacher_ || myRbfV4Convergence_) && myRbfV2RobustMode_=="smooth")
    myRbfV2WbcInput_.smoothWidth=active->smoothingWidth[0];
  wbc_->configureMyRbfV2(myRbfV2WbcInput_);
  if(!wbc_->prepareMyRbfV2Tracking(stateDesired,deliveredInput,measuredRbdState_,mode,dt)) {
    publishMyRbfV2Lyapunov(wbc_->getMyRbfV2Diagnostics(),*active,0);
    ROS_ERROR("[MyRBF] Invalid physical reference or coordinate transformation");failStop();return;
  }
  const auto tracking=wbc_->getMyRbfV2Diagnostics();
  const WbcVector6 h=info.robotMass*currentObservation_.state.head<6>();
  const vector_t q=currentObservation_.state.tail(info.generalizedCoordinatesNum);
  updateCentroidalDynamics(*myRbfV2ResidualModel_,info,q);
  const Eigen::Vector3d currentComOffset=
      myRbfV2ResidualModel_->getData().com[0]-q.head<3>();
  double mismatch=0;
  if(myRbfV2MomentumValid_) {
    const double elapsed=currentObservation_.time-myRbfV2PreviousTime_;
    if(!(elapsed>0) || !std::isfinite(elapsed)) {failStop();return;}
    myRbfV3UpdateDt_=elapsed;
    // Momentum balance uses the FINAL preceding WBC contact forces. No
    // discarded pre-WBC compensation request is treated as an applied force.
    myRbfV2ExternalResidual_=(h-myRbfV2PreviousMomentum_)/elapsed-myRbfV2PreviousNominalRate_;
    myRbfV2ExternalResidual_.tail<3>()+=myRbfV2PreviousComOffset_.cross(myRbfV2ExternalResidual_.head<3>());
    adaptive::v2::FeatureInput feature;
    feature.chi=axes(tracking.chi);
    myRbfV3TeacherErrorValid_=false;
    myRbfV3PredictionError_.setZero();
    myRbfV3TrackingError_.setZero();
    myRbfV3CompositeError_.setZero();
    myRbfV3PredictionStateError_.setZero(info.stateDim);
    myRbfV3TrackingStateError_.setZero(info.stateDim);
    myRbfV3CompositeStateError_.setZero(info.stateDim);
    if(myRbfV3Teacher_) {
      myRbfV3TeacherErrorValid_=myRbfV2PolicyBinding_->sampleTeacherErrors(
          currentObservation_.time,currentObservation_.state,
          myRbfV3PredictionStateError_,myRbfV3TrackingStateError_);
      if(myRbfV3TeacherErrorValid_) {
        myRbfV3CompositeStateError_=myRbfV3CompositeRatio_*myRbfV3PredictionStateError_+
            (1.-myRbfV3CompositeRatio_)*myRbfV3TrackingStateError_;
        myRbfV3PredictionError_=myRbfV3PredictionStateError_.head<6>();
        myRbfV3TrackingError_=myRbfV3TrackingStateError_.head<6>();
        myRbfV3CompositeError_=myRbfV3CompositeStateError_.head<6>();
        // Audited base-origin world-wrench map:
        // B_w=(1/m)[I 0; -[r]_x I], therefore
        // B_w^T E_c=(1/m)[E_linear+r x E_angular; E_angular].
        WbcVector6 generalLearningSignal=myRbfV3CompositeError_;
        generalLearningSignal.head<3>()+=
            currentComOffset.cross(myRbfV3CompositeError_.tail<3>());
        generalLearningSignal/=info.robotMass;
        feature.learningSignal=axes(generalLearningSignal);
        feature.learningSignalValid=true;
      }
    }
    WbcVector6 velocity;
    velocity.head<3>()=measuredRbdState_.segment<3>(info.generalizedCoordinatesNum+3);
    velocity.tail<3>()=measuredRbdState_.segment<3>(info.generalizedCoordinatesNum);
    feature.baseVelocity=axes(velocity);
    feature.referenceVelocity=axes(velocity-tracking.chi);
    feature.contacts=modeNumber2StanceLeg(mode);
    try {
      const auto learning=myRbfV2Pipeline_->advanceExternalResidual(currentObservation_.time,elapsed,myRbfV2ExternalResidual_,feature);
      adaptive::MyRbfV2Snapshot next;
      next.valid=learning.valid;next.timestamp=currentObservation_.time;
      next.sequenceId=++myRbfV2Sequence_;next.contactMode=mode;
      next.modelUsesSubtractedCorrection=
          myRbfV3Teacher_ && !myRbfV3SharedAllocator_;
      next.feature=learning.evaluationAfterUpdate.physicalGradient;
      next.weights=myRbfV2Pipeline_->state().weights;
      next.modelWrenchEstimate=axes(learning.externalWrenchEstimate);
      const WbcVector6 physicalExternalEstimate=myRbfV3Teacher_
          ? -learning.externalWrenchEstimate : learning.externalWrenchEstimate;
      next.externalWrenchEstimate=axes(physicalExternalEstimate);
      next.leftHandDisturbanceEstimate=axes(-physicalExternalEstimate);
      next.requiredControlCorrection=axes(
          -(physicalExternalEstimate+learning.robustWrench));
      const auto delivery=adaptive::resolveV3ControlDelivery(
          next,(myRbfV3Teacher_ || myRbfV4Convergence_) && myRbfV3SharedAllocator_);
      next.modelWrenchEstimate=delivery.modelWrench;
      next.robustBound=axes(learning.robustBound);next.robustWrench=axes(learning.robustWrench);
      next.smoothingWidth=axes(learning.smoothingWidth);next.chi=axes(tracking.chi);
      next.referenceWrench=axes(tracking.referenceWrench);
      auto physicalEvaluation=learning.evaluationAfterUpdate;
      physicalEvaluation.estimate=axes(physicalExternalEstimate);
      const auto constraint=myRbfV2Pipeline_->core().constraint(next.chi,next.referenceWrench,
          axes(myRbfV2WbcInput_.damping),physicalEvaluation);
      next.clfNormal=constraint.normal;next.clfRhs=constraint.rhs;
      myRbfV2Snapshots_->tryPublish(next);
      // This is a logged sample/hold remainder, NOT assumed zero merely
      // because MPC and WBC snapshot IDs match.
      mismatch=tracking.chi.dot(
          physicalExternalEstimate-vector6(active->externalWrenchEstimate));
    } catch(const std::exception& e) {
      ROS_ERROR_STREAM("[V2] Learner failure: "<<e.what());failStop();return;
    }
  }
  const vector_t solution=wbc_->update(stateDesired,deliveredInput,measuredRbdState_,mode,dt);
  const auto& diagnostic=wbc_->getMyRbfV2Diagnostics();
  if(solution.size()!=info.generalizedCoordinatesNum+3*info.numThreeDofContacts+info.actuatedDofNum || !diagnostic.feasible) {
    publishMyRbfV2Lyapunov(diagnostic,*active,mismatch);
    ROS_ERROR_STREAM("[MyRBF] WBC failed; status="<<int(diagnostic.status));
    failStop();return;
  }
  vector_t finalInput=deliveredInput;
  finalInput.head(3*info.numThreeDofContacts)=diagnostic.finalContactForces;
  currentObservation_.input=finalInput;
  if(!safetyChecker_->check(currentObservation_,stateDesired,finalInput)) {
    publishMyRbfV2Lyapunov(diagnostic,*active,mismatch);
    ROS_ERROR("[V2] State safety check failed");failStop();return;
  }
  myRbfV2Armed_=true;
  myRbfV2PreviousComOffset_=currentComOffset;
  const WbcVector6 finalRate=info.robotMass*getNormalizedCentroidalMomentumRate(*myRbfV2ResidualModel_,info,finalInput);
  const WbcVector6 nominalRate=info.robotMass*getNormalizedCentroidalMomentumRate(*myRbfV2ResidualModel_,info,inputDesired);
  myRbfV2AppliedCorrection_=finalRate-nominalRate;
  myRbfV2AppliedCorrection_.tail<3>()+=myRbfV2PreviousComOffset_.cross(myRbfV2AppliedCorrection_.head<3>());
  myRbfV2PreviousNominalRate_=finalRate;
  myRbfV2PreviousMomentum_=h;myRbfV2PreviousTime_=currentObservation_.time;myRbfV2MomentumValid_=true;
  myRbfV2WbcInput_.previousContactForces=diagnostic.finalContactForces;
  const vector_t pos=centroidal_model::getJointAngles(stateDesired,info);
  const vector_t vel=centroidal_model::getJointVelocities(finalInput,info);
  const vector_t torque=solution.tail(info.actuatedDofNum);
  if(myRbfV2SolverFault_) {failStop();return;}
  for(std::size_t j=0;j<info.actuatedDofNum;++j)
    hybridJointHandles_[j].setCommand(pos(j),vel(j),0,0,torque(j));
  publishMyRbfV2Lyapunov(diagnostic,*active,mismatch);
  robotVisualizer_->update(currentObservation_,mpcMrtInterface_->getPolicy(),mpcMrtInterface_->getCommand());
  selfCollisionVisualization_->update(currentObservation_);
  observationPublisher_.publish(ros_msg_conversions::createObservationMsg(currentObservation_));
}

void LeggedController::publishMyRbfV2Lyapunov(const MyRbfV2WbcDiagnostics& d,
    const adaptive::MyRbfV2Snapshot& s,double mismatch) {
  ++myRbfV2DiagnosticSequence_;
  if(!myRbfV2Publisher_ || !myRbfV2Publisher_->trylock()) {++myRbfV2DiagnosticDrops_;return;}
  auto& out=myRbfV2Publisher_->msg_.data;
  std::size_t k=0;
  const auto add=[&](double value){out.at(k++)=value;};
  add(currentObservation_.time);add(s.sequenceId);add(s.sequenceId);add(d.snapshotSequenceId);
  add(myRbfV2PolicyBinding_->activeSolveId());add(d.contactMode);
  add(myRbfV2FeatureMode_=="contact_aware"?1:0);add(myRbfV2RobustMode_=="sign"?1:0);
  add(d.clfLhs);add(d.clfRhs);add(d.clfViolation);add(d.feasible?1:0);
  add(int(d.status));add(d.solverReturnCode);add(d.contactWrenchRank);
  add(currentObservation_.time-s.timestamp);add(mismatch);
  add(d.torqueLimitMargin);add(d.frictionMargin);add(d.normalForceMargin);
  add(d.contactForceMargin);add(d.contactForceRateMargin);add(d.maxEqualityViolation);add(d.maxInequalityViolation);
  add(myRbfV2DiagnosticSequence_);add(myRbfV2DiagnosticDrops_);
  add(d.desiredAccelerationInitialized?1:0);add(d.referenceWrenchIsFinal?1:0);
  add(myRbfV2Armed_?1:0);add(currentObservation_.time-myRbfV2ActivationStartTime_);
  add(myRbfV3Teacher_?1:0);add(myRbfV4Convergence_?1:0);
  add(myRbfV3TeacherErrorValid_?1:0);
  add(myRbfV3CompositeRatio_);add(leggedInterface_->getCentroidalModelInfo().robotMass);
  add(myRbfV3UpdateDt_);
  const WbcVector6 robust=[&]{WbcVector6 r;for(int i=0;i<6;++i) {
    const double c=d.chi(i);
    r(i)=s.robustBound[i]*(myRbfV2RobustMode_=="sign" ? double((c>0)-(c<0)) : c/std::hypot(c,s.smoothingWidth[i]));
  }return r;}();
  for(const auto& value:std::vector<WbcVector6>{d.chi,vector6(s.externalWrenchEstimate),
      vector6(s.leftHandDisturbanceEstimate),d.referenceWrench,vector6(s.robustBound),robust,
      vector6(s.smoothingWidth),d.finalContactWrench,myRbfV2ExternalResidual_,myRbfV2AppliedCorrection_,
      myRbfV3PredictionError_,myRbfV3TrackingError_,myRbfV3CompositeError_})
    for(int i=0;i<6;++i)add(value(i));
  for(const auto* value:std::vector<const vector_t*>{&myRbfV3PredictionStateError_,
      &myRbfV3TrackingStateError_,&myRbfV3CompositeStateError_}) {
    if(value->size()!=24 || !value->allFinite())throw std::runtime_error("V3 teacher state error must be finite and 24-dimensional");
    for(int i=0;i<24;++i)add((*value)(i));
  }
  for(int i=0;i<12;++i)add(i<d.finalContactForces.size()?d.finalContactForces(i):std::numeric_limits<double>::quiet_NaN());
  for(const auto& a:s.weights)for(double x:a)add(x);
  for(const auto& a:s.feature)for(double x:a)add(x);
  myRbfV2Publisher_->unlockAndPublish();
}
}
