#pragma once
#include <pinocchio/fwd.hpp>
#include <ocs2_core/constraint/StateInputConstraint.h>
#include <ocs2_core/dynamics/SystemDynamicsBase.h>
#include "legged_interface/adaptive/MyRbfV11Model.h"
#include "legged_interface/adaptive/MyRbfV2Snapshot.h"
namespace legged { namespace adaptive {
class MyRbfV11Clf final: public ocs2::StateInputConstraint {
 public:
  MyRbfV11Clf(const ocs2::SystemDynamicsBase& dynamics,const ocs2::CentroidalModelInfo& info,
             std::shared_ptr<MyRbfV2SnapshotStore> store)
      :StateInputConstraint(ocs2::ConstraintOrder::Linear),dynamics_(dynamics.clone()),info_(info),store_(std::move(store)){}
  MyRbfV11Clf(const MyRbfV11Clf& other):StateInputConstraint(other),
      dynamics_(other.dynamics_->clone()),info_(other.info_),store_(other.store_){}
  MyRbfV11Clf* clone()const override{return new MyRbfV11Clf(*this);}
  bool isActive(double)const override{return store_->frozenForSolver->v11PredictionValid;}
  std::size_t getNumConstraints(double)const override{return 1;}
  ocs2::vector_t getValue(double t,const ocs2::vector_t& x,const ocs2::vector_t& u,
                         const ocs2::PreComputation& pre)const override{
    const auto& s=*store_->frozenForSolver;
    v11::Vector6 ref,refRate;v11::reference(s.v11Target,t,info_,ref,refRate);
    v11::Evaluation e;
    v11::Vector6 predicted,predictedRate;
    v11::oneStepPrediction(s.v11PredictionOrigin,s.v11PredictionRate,s.v11PredictionTime,s.timestamp,t,predicted,predictedRate);
    e.ee=v11::velocity(x,info_)-predicted;
    e.er=v11::velocity(x,info_)-ref;
    e.ec=s.v11Settings.ratio*e.ee+(1-s.v11Settings.ratio)*e.er;
    const v11::Vector6 rate=velocityRate(x,dynamics_->computeFlowMap(t,x,u,pre));
    return ocs2::vector_t::Constant(1,v11::clf(e,rate,predictedRate,refRate,s.v11Settings));
  }
  ocs2::VectorFunctionLinearApproximation getLinearApproximation(double t,const ocs2::vector_t& x,
      const ocs2::vector_t& u,const ocs2::PreComputation& pre)const override{
    auto a=ocs2::VectorFunctionLinearApproximation::Zero(1,x.size(),u.size());
    const auto flow=dynamics_->linearApproximation(t,x,u,pre);
    const auto& s=*store_->frozenForSolver;const auto v=v11::velocity(x,info_);
    const auto R=v11::rotation(x.segment<3>(9));
    const Eigen::Matrix3d invI=info_.centroidalInertiaNominal.inverse();
    const Eigen::Vector3d angles=x.segment<3>(9);
    const auto dR=ocs2::getRotationMatrixZyxGradient(angles);
    ocs2::matrix_t J=ocs2::matrix_t::Zero(6,x.size());J.topLeftCorner<3,3>().setIdentity();
    J.block<3,3>(3,3)=invI*info_.robotMass*R.transpose();
    for(int j=0;j<3;++j)J.block<3,1>(3,9+j)=invI*info_.robotMass*dR[j].transpose()*x.segment<3>(3);
    const auto skew=[](const Eigen::Vector3d& p){Eigen::Matrix3d m;m<<0,-p.z(),p.y(),p.z(),0,-p.x(),-p.y(),p.x(),0;return m;};
    ocs2::matrix_t rateX(6,x.size()),rateU(6,u.size());
    rateX.topRows<3>()=flow.dfdx.topRows<3>();rateU.topRows<3>()=flow.dfdu.topRows<3>();
    rateX.bottomRows<3>()=invI*(info_.robotMass*R.transpose()*flow.dfdx.middleRows<3>(3)+
        (skew(info_.centroidalInertiaNominal*v.tail<3>())-skew(v.tail<3>())*info_.centroidalInertiaNominal)*J.bottomRows<3>());
    for(int j=0;j<3;++j)rateX.block<3,1>(3,9+j)+=invI*info_.robotMass*dR[j].transpose()*flow.f.segment<3>(3);
    rateU.bottomRows<3>()=invI*info_.robotMass*R.transpose()*flow.dfdu.middleRows<3>(3);
    v11::Vector6 ref,refRate;v11::reference(s.v11Target,t,info_,ref,refRate);
    v11::Vector6 predicted,predictedRate;
    v11::oneStepPrediction(s.v11PredictionOrigin,s.v11PredictionRate,s.v11PredictionTime,s.timestamp,t,predicted,predictedRate);
    v11::Evaluation e;e.ee=v-predicted;
    e.er=v-ref;e.ec=s.v11Settings.ratio*e.ee+(1-s.v11Settings.ratio)*e.er;
    const auto rate=velocityRate(x,flow.f);
    a.f(0)=v11::clf(e,rate,predictedRate,refRate,s.v11Settings);
    a.dfdx=(-rate+s.v11Settings.ratio*predictedRate+(1-s.v11Settings.ratio)*refRate-
        s.v11Settings.decay.cwiseProduct(e.ec)).transpose()*J-e.ec.transpose()*rateX;
    a.dfdu=-e.ec.transpose()*rateU;
    return a;
  }
 private:
  v11::Vector6 velocityRate(const ocs2::vector_t& x,const ocs2::vector_t& flow)const{
    const Eigen::Vector3d omega=v11::velocity(x,info_).tail<3>();
    v11::Vector6 rate;rate.head<3>()=flow.head<3>();
    rate.tail<3>()=info_.centroidalInertiaNominal.ldlt().solve(
        info_.robotMass*v11::rotation(x.segment<3>(9)).transpose()*flow.segment<3>(3)-
        omega.cross(info_.centroidalInertiaNominal*omega));return rate;
  }
  std::unique_ptr<ocs2::SystemDynamicsBase> dynamics_;ocs2::CentroidalModelInfo info_;
  std::shared_ptr<MyRbfV2SnapshotStore> store_;
};
}}
