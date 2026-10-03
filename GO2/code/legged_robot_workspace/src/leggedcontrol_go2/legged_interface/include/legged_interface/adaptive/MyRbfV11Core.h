#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Cholesky>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace legged { namespace adaptive { namespace v11 {
using Vector6 = Eigen::Matrix<double, 6, 1, Eigen::DontAlign>;
using Matrix6 = Eigen::Matrix<double, 6, 6, Eigen::DontAlign>;
using Weights = Eigen::Matrix<double, 6, 26, Eigen::DontAlign>;
struct Settings {
  double ratio = 0.9;
  Vector6 scale = (Vector6() << .02,.02,.02,.05,.05,.05).finished();
  Vector6 gamma = (Vector6() << 10,10,10,1,1,1).finished();
  Vector6 weightLimit = (Vector6() << 20,20,20,3,3,3).finished();
  Vector6 decay = Vector6::Constant(.2);
  Eigen::Matrix<double,25,1> centers = Eigen::Matrix<double,25,1>::LinSpaced(-1.,1.);
  Eigen::Matrix<double,25,1> widths = Eigen::Matrix<double,25,1>::Constant(.12);
};
struct Evaluation {
  Vector6 ee=Vector6::Zero(), er=Vector6::Zero(), ec=Vector6::Zero();
  Vector6 xi=Vector6::Zero(), gradient=Vector6::Zero(), d=Vector6::Zero();
  Weights features=Weights::Zero(), derivative=Weights::Zero();
  bool projected=false;
};
class Core {
 public:
  explicit Core(Settings settings=Settings()):settings_(std::move(settings)) {
    if(!(settings_.ratio>.5 && settings_.ratio<1.) || !settings_.scale.allFinite() ||
       !settings_.gamma.allFinite() || !settings_.weightLimit.allFinite() ||
       !settings_.decay.allFinite() || !settings_.centers.allFinite() || !settings_.widths.allFinite() ||
       (settings_.scale.array()<=0).any() || (settings_.gamma.array()<=0).any() ||
       (settings_.weightLimit.array()<=0).any() || (settings_.decay.array()<=0).any() ||
       (settings_.widths.array()<=0).any()) throw std::invalid_argument("invalid V1.1 settings");
  }
  void reset(){weights_.setZero();}
  const Weights& weights() const{return weights_;}
  const Settings& settings() const{return settings_;}
  Evaluation update(const Vector6& ee,const Vector6& er,const Matrix6& G,double dt,bool enabled) {
    if(!ee.allFinite() || !er.allFinite() || !G.allFinite() || !std::isfinite(dt) || dt<0)
      throw std::invalid_argument("invalid V1.1 observation");
    Evaluation out; out.ee=ee;out.er=er;out.ec=settings_.ratio*ee+(1-settings_.ratio)*er;
    out.gradient=G.transpose()*out.ec;
    for(int i=0;i<6;++i){
      out.xi(i)=std::max(-1.,std::min(1.,ee(i)/settings_.scale(i)));
      for(int j=0;j<25;++j){
        const double z=(out.xi(i)-settings_.centers(j))/settings_.widths(j);
        out.features(i,j)=std::exp(-.5*z*z);
      }
      out.features(i,25)=1.;
      // V1.1 section 3.6: no leakage, residual supervision, normalization,
      // history term, output scaling, or variable learning gain.
      out.derivative.row(i)=-settings_.gamma(i)*out.gradient(i)*out.features.row(i);
      if(enabled) for(int j=0;j<26;++j){
        const double proposal=weights_(i,j)+dt*out.derivative(i,j);
        const double bounded=std::max(-settings_.weightLimit(i),std::min(settings_.weightLimit(i),proposal));
        out.projected=out.projected || bounded!=proposal;weights_(i,j)=bounded;
      }
      out.d(i)=weights_.row(i).dot(out.features.row(i));
    }
    if(!weights_.allFinite() || !out.d.allFinite())throw std::runtime_error("nonfinite V1.1 update");
    return out;
  }
 private:
  Settings settings_; Weights weights_=Weights::Zero();
};
inline Eigen::Matrix3d rotation(const Eigen::Vector3d& zyx){
  return (Eigen::AngleAxisd(zyx(0),Eigen::Vector3d::UnitZ())*
          Eigen::AngleAxisd(zyx(1),Eigen::Vector3d::UnitY())*
          Eigen::AngleAxisd(zyx(2),Eigen::Vector3d::UnitX())).toRotationMatrix();
}
inline Matrix6 mapping(double mass,const Eigen::Matrix3d& inertia,const Eigen::Matrix3d& R){
  if(!(mass>0) || !std::isfinite(mass) || !inertia.allFinite() || !R.allFinite())
    throw std::invalid_argument("invalid V1.1 nominal model");
  Eigen::LLT<Eigen::Matrix3d> factor(inertia);
  if(factor.info()!=Eigen::Success)throw std::invalid_argument("V1.1 inertia must be positive definite");
  Matrix6 G=Matrix6::Zero();G.topLeftCorner<3,3>()=R/mass;
  G.bottomRightCorner<3,3>()=factor.solve(Eigen::Matrix3d::Identity());return G;
}
inline double clf(const Evaluation& e,const Vector6& rate,const Vector6& previousPredictionRate,
                  const Vector6& referenceRate,const Settings& s){
  return -e.ec.dot(rate)+s.ratio*e.ee.dot(previousPredictionRate)+
      (1-s.ratio)*e.er.dot(referenceRate)-.5*e.ec.dot(s.decay.cwiseProduct(e.ec));
}
}}}
