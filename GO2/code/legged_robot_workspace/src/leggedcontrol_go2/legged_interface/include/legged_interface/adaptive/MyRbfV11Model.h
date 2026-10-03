#pragma once
#include <pinocchio/fwd.hpp>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include <ocs2_core/reference/TargetTrajectories.h>
#include "legged_interface/adaptive/MyRbfV11Core.h"
namespace legged { namespace adaptive { namespace v11 {
inline void oneStepPrediction(const Vector6& origin,const Vector6& stepRate,double start,double end,
                              double t,Vector6& value,Vector6& rate){
  // A control-cycle Euler prediction is only valid until its recorded end.
  // Hold that endpoint in the MPC horizon; never extend a 1-ms acceleration
  // sample for a whole second. The derivative of the held signal is zero.
  const double stop=std::max(start,end);
  value=origin+(std::max(start,std::min(t,stop))-start)*stepRate;
  rate=(t>=start && t<stop)?stepRate:Vector6::Zero();
}
inline Vector6 velocity(const ocs2::vector_t& x,const ocs2::CentroidalModelInfo& info){
  if(x.size()!=info.stateDim || !x.allFinite())throw std::invalid_argument("invalid V1.1 state");
  Vector6 v;v.head<3>()=x.head<3>();
  const auto R=rotation(x.segment<3>(9));
  v.tail<3>()=info.centroidalInertiaNominal.ldlt().solve(R.transpose()*info.robotMass*x.segment<3>(3));
  return v; // [CoM velocity WORLD, angular velocity BODY], never Euler rates
}
inline Vector6 nominalRate(ocs2::PinocchioInterface& pin,const ocs2::CentroidalModelInfo& info,
                          const ocs2::vector_t& x,const ocs2::vector_t& u){
  const ocs2::vector_t q=x.tail(info.generalizedCoordinatesNum);
  ocs2::updateCentroidalDynamics(pin,info,q);
  const auto hdot=ocs2::getNormalizedCentroidalMomentumRate(pin,info,u);
  const Eigen::Vector3d omega=velocity(x,info).tail<3>();
  Vector6 rate;rate.head<3>()=hdot.head<3>();
  rate.tail<3>()=info.centroidalInertiaNominal.ldlt().solve(
      rotation(x.segment<3>(9)).transpose()*info.robotMass*hdot.tail<3>()-
      omega.cross(info.centroidalInertiaNominal*omega));
  return rate;
}
inline void reference(const ocs2::TargetTrajectories& target,double t,const ocs2::CentroidalModelInfo& info,
                      Vector6& v,Vector6& rate){
  if(target.empty())throw std::invalid_argument("V1.1 requires a paired velocity reference");
  v=velocity(target.getDesiredState(t),info);rate.setZero();
  if(target.timeTrajectory.size()<2)return;
  const double lo=std::max(target.timeTrajectory.front(),t-1e-4);
  const double hi=std::min(target.timeTrajectory.back(),t+1e-4);
  if(hi>lo)rate=(velocity(target.getDesiredState(hi),info)-velocity(target.getDesiredState(lo),info))/(hi-lo);
}
}}}
