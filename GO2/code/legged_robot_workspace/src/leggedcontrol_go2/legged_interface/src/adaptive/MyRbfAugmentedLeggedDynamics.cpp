#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/data.hpp>
#include "legged_interface/adaptive/MyRbfAugmentedLeggedDynamics.h"
#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include <stdexcept>
namespace legged { namespace adaptive {
MyRbfAugmentedLeggedDynamics::MyRbfAugmentedLeggedDynamics(
    const ocs2::SystemDynamicsBase& nominal, const ocs2::PinocchioInterface& p,
    const ocs2::CentroidalModelInfo& info, std::shared_ptr<MyRbfV2SnapshotStore> snapshots)
    : nominal_(nominal.clone()), pinocchio_(p), info_(info), snapshots_(std::move(snapshots)) {
  if(!snapshots_ || !(info_.robotMass>0)) throw std::invalid_argument("V2 requires snapshots and positive mass");
}
MyRbfAugmentedLeggedDynamics::MyRbfAugmentedLeggedDynamics(const MyRbfAugmentedLeggedDynamics& other)
    : SystemDynamicsBase(other),nominal_(other.nominal_->clone()),pinocchio_(other.pinocchio_),
      info_(other.info_),snapshots_(other.snapshots_) {}
MyRbfAugmentedLeggedDynamics* MyRbfAugmentedLeggedDynamics::clone() const {
  return new MyRbfAugmentedLeggedDynamics(*this);
}
ocs2::vector_t MyRbfAugmentedLeggedDynamics::addition(const ocs2::vector_t& x,ocs2::matrix_t* derivative) {
  ocs2::vector_t value=ocs2::vector_t::Zero(x.size());
  if(derivative) derivative->setZero(x.size(),x.size());
  const auto& snapshot=*snapshots_->frozenForSolver;
  if(!snapshot.valid) return value;
  if(snapshot.v11Enabled){
    const auto evaluate=[&](const ocs2::vector_t& state){
      ocs2::vector_t correction=ocs2::vector_t::Zero(state.size());
      const auto R=v11::rotation(state.segment<3>(9));
      correction.head<3>()=-R*snapshot.v11Disturbance.head<3>()/info_.robotMass;
      correction.segment<3>(3)=-R*snapshot.v11Disturbance.tail<3>()/info_.robotMass;
      return correction; // BODY wrench about CoM, left-hand sign
    };
    value=evaluate(x);
    if(derivative)for(int j=9;j<12;++j){
      auto xp=x,xm=x;const double h=1e-6;xp(j)+=h;xm(j)-=h;
      derivative->col(j)=(evaluate(xp)-evaluate(xm))/(2*h);
    }
    return value;
  }
  const ocs2::vector_t q=x.tail(info_.generalizedCoordinatesNum);
  ocs2::updateCentroidalDynamics(pinocchio_,info_,q);
  const auto& data=pinocchio_.getData();
  const Eigen::Vector3d offset=data.com[0]-q.head<3>();
  const ocs2::matrix_t Jcom=data.Ag.topRows(3)/info_.robotMass;
  const double modelSign=snapshot.modelUsesSubtractedCorrection ? -1. : 1.;
  Eigen::Matrix<double,6,1> wrench;
  for(int i=0;i<6;++i) wrench(i)=snapshot.modelWrenchEstimate[i];
  value.head<3>()=modelSign*wrench.head<3>()/info_.robotMass;
  value.segment<3>(3)=modelSign*(wrench.tail<3>()-offset.cross(wrench.head<3>()))/info_.robotMass;
  if(derivative) for(int j=0;j<info_.generalizedCoordinatesNum;++j) {
    Eigen::Vector3d dOffset=Jcom.col(j);
    if(j<3) dOffset(j)-=1.0;
    derivative->block<3,1>(3,6+j)=-modelSign*dOffset.cross(wrench.head<3>())/info_.robotMass;
  }
  return value;
}
ocs2::vector_t MyRbfAugmentedLeggedDynamics::computeFlowMap(ocs2::scalar_t t,
    const ocs2::vector_t& x,const ocs2::vector_t& u,const ocs2::PreComputation& pre) {
  return nominal_->computeFlowMap(t,x,u,pre)+addition(x,nullptr);
}
ocs2::VectorFunctionLinearApproximation MyRbfAugmentedLeggedDynamics::linearApproximation(
    ocs2::scalar_t t,const ocs2::vector_t& x,const ocs2::vector_t& u,const ocs2::PreComputation& pre) {
  auto result=nominal_->linearApproximation(t,x,u,pre);
  ocs2::matrix_t derivative;
  result.f+=addition(x,&derivative);
  result.dfdx+=derivative;
  return result;
}
}}
