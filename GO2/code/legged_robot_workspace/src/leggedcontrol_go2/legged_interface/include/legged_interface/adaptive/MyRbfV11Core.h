#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Cholesky>
#include <algorithm>
#include <cstddef>
#include <cmath>
#include <stdexcept>

namespace legged { namespace adaptive { namespace v11 {
using Vector6 = Eigen::Matrix<double, 6, 1, Eigen::DontAlign>;
using Matrix6 = Eigen::Matrix<double, 6, 6, Eigen::DontAlign>;
using Weights = Eigen::Matrix<double, 6, 26, Eigen::DontAlign>;
struct Settings {
  double ratio = 0.9;
  // Cross-coupled torque basis feature.
  //
  // false: original six independent scalar RBF inputs.
  // true : torque RBFs additionally use the translational
  //        prediction-residual feature mapped through the
  //        nominal CoM->base lever arm.
  bool torqueCoupledFeature = false;

  // xi_tau =
  //   alpha * xi_tau_self
  // + (1-alpha) * xi_tau_cross
  double torqueFeatureAngularRatio = 0.30;

  // Nominal overall-CoM -> base-link-CoM vector in BODY.
  // Loaded from /go2_nominal_model/com_to_base_body.
  Eigen::Vector3d comToBaseBody = Eigen::Vector3d::Zero();

 // Time constant of the first-order low-pass filter applied to q = G^T E_c.
  Vector6 gradientFilterTau = Vector6::Constant(0.05);
  Vector6 scale = (Vector6() << .02,.02,.02,.05,.05,.05).finished();
  Vector6 gamma = (Vector6() << 10,10,10,1,1,1).finished();
  // Soft deadzone applied to the filtered adaptive gradient q_f = LPF(G^T E_c).
  // Units are identical to the corresponding component of G^T E_c.
  Vector6 gradientDeadzone = Vector6::Zero();
  // Sigma-modification leakage [1/s].
  // Adaptive law: Wdot = -Gamma*h*q_filtered - leakage*W.
  Vector6 leakage = Vector6::Zero();
  Vector6 weightLimit = (Vector6() << 20,20,20,3,3,3).finished();
  Vector6 decay = Vector6::Constant(.2);

  // ==========================================================
  // V1.2-A: online dynamics-residual supervision
  //
  // d_rec = G^{-1}(vdot_nominal - vdot_measured)
  //
  // e_d = d_hat_active(previous interval)
  //       - LPF(d_rec)
  //
  // Only the constant bias feature receives this additional
  // supervision. No Gazebo truth / true Y*pi enters learning.
  // ==========================================================
  bool dynamicsResidualEnabled = false;

  Vector6 dynamicsResidualFilterTau =
      Vector6::Constant(0.50);

  Vector6 dynamicsBiasGain =
      Vector6::Zero();

  // Original Gaussian weights keep v11_weight_limit.
  // The constant bias requires a larger Fz range because
  // +5 kg corresponds to about 49 N.
  Vector6 biasWeightLimit =
      (Vector6() <<
       40.,40.,70.,10.,10.,10.).finished();

  Eigen::Matrix<double,25,1> centers = Eigen::Matrix<double,25,1>::LinSpaced(-1.,1.);
  Eigen::Matrix<double,25,1> widths = Eigen::Matrix<double,25,1>::Constant(.12);
};
struct Evaluation {
  // ee: 原始的一步预测误差，用于构造复合误差 Ec。
  // eeFiltered: ee 的 20 点因果滑动平均，只用于 RBF 基函数输入。
  Vector6 ee=Vector6::Zero(), eeFiltered=Vector6::Zero();
  Vector6 er=Vector6::Zero(), ec=Vector6::Zero();
  Vector6 xi=Vector6::Zero(), xiSelf=Vector6::Zero(), torqueCrossXi=Vector6::Zero(), gradient=Vector6::Zero(),filteredGradient=Vector6::Zero(),
        effectiveGradient=Vector6::Zero(),
        filteredDynamicsReference=Vector6::Zero(),
        dynamicsError=Vector6::Zero(),
        d=Vector6::Zero();
  Weights features=Weights::Zero(), derivative=Weights::Zero();
  bool projected=false;
};
class Core {
 public:
  explicit Core(Settings settings=Settings()):settings_(std::move(settings)) {

    if(!std::isfinite(settings_.torqueFeatureAngularRatio) ||
       settings_.torqueFeatureAngularRatio < 0. ||
       settings_.torqueFeatureAngularRatio > 1. ||
       !settings_.comToBaseBody.allFinite() ||
       (settings_.torqueCoupledFeature &&
        settings_.comToBaseBody.norm() <= 1e-9))
      throw std::invalid_argument(
          "invalid V1.1 torque-coupled feature settings");

    if(!(settings_.ratio>.5 && settings_.ratio<1.) || !settings_.gradientFilterTau.allFinite() || (settings_.gradientFilterTau.array()<=0).any() || !settings_.scale.allFinite() ||
       !settings_.gamma.allFinite() || !settings_.gradientDeadzone.allFinite() || !settings_.leakage.allFinite() || !settings_.weightLimit.allFinite() ||
       !settings_.decay.allFinite() || !settings_.centers.allFinite() || !settings_.widths.allFinite() ||
       (settings_.scale.array()<=0).any() || (settings_.gamma.array()<=0).any() || (settings_.gradientDeadzone.array()<0).any() || (settings_.leakage.array()<0).any() ||
       (settings_.weightLimit.array()<=0).any() || (settings_.decay.array()<=0).any() ||
       (settings_.widths.array()<=0).any()) throw std::invalid_argument("invalid V1.1 settings");
  }
  void reset(){
    weights_.setZero();

    // 同时清空 20 点滑动平均的历史状态。
    eeHistory_.setZero();
    eeSum_.setZero();
    eeHistoryIndex_=0;
    eeHistoryInitialized_=false;
    filteredGradient_.setZero();

    filteredDynamicsReference_.setZero();
    dynamicsReferenceInitialized_=false;
  }
  const Weights& weights() const{return weights_;}
  const Settings& settings() const{return settings_;}
  Evaluation update(
      const Vector6& ee,
      const Vector6& er,
      const Matrix6& G,
      const Vector6& dynamicsReference,
      const Vector6& previousAppliedDisturbance,
      bool dynamicsValid,
      double dt,
      bool enabled) {

    if(!ee.allFinite() ||
       !er.allFinite() ||
       !G.allFinite() ||
       !std::isfinite(dt) ||
       dt<0 ||
       (dynamicsValid &&
        (!dynamicsReference.allFinite() ||
         !previousAppliedDisturbance.allFinite())))
      throw std::invalid_argument("invalid V1.2-A observation");
    Evaluation out;

    // ------------------------------------------------------------
    // 复合误差仍然严格使用原始预测误差 ee：
    //
    // Ec = gamma * Ee + (1-gamma) * Er
    //
    // 滑动平均绝不能进入这里，否则会改变原来的 Lyapunov/CLF 结构。
    // ------------------------------------------------------------
    out.ee=ee;
    out.er=er;
    out.ec=settings_.ratio*ee+(1-settings_.ratio)*er;

    // ------------------------------------------------------------
    // 20 点因果滑动平均：
    //
    // Ee_bar(k) = [Ee(k)+Ee(k-1)+...+Ee(k-19)] / 20
    //
    // 该量只用于 RBF 的 Gaussian basis 输入。
    //
    // 第一个有效预测误差出现时，用该样本填满 20 个历史槽位，
    // 而不是用 19 个零进行补齐，这样不会人为制造启动瞬态。
    // dt>0 表示当前具有有效的一步预测时间间隔。
    // ------------------------------------------------------------
    Vector6 basisEe=ee;

    if(dt>0.){
      if(!eeHistoryInitialized_){
        for(int j=0;j<20;++j){
          eeHistory_.col(j)=ee;
        }

        eeSum_=20.*ee;
        eeHistoryIndex_=0;
        eeHistoryInitialized_=true;
      }else{
        // O(1) 更新滑动窗口：
        // 减去最旧样本，再加入当前样本。
        eeSum_-=eeHistory_.col(eeHistoryIndex_);
        eeHistory_.col(eeHistoryIndex_)=ee;
        eeSum_+=ee;

        eeHistoryIndex_=(eeHistoryIndex_+1)%20;
      }

      basisEe=eeSum_/20.;
    }else if(eeHistoryInitialized_){
      // 没有新的有效预测时不更新历史，只沿用当前滤波值。
      basisEe=eeSum_/20.;
    }

    out.eeFiltered=basisEe;

    // ==========================================================
    // V1.2-A dynamics reference.
    //
    // reconstructed uncertainty and d_hat use the same convention:
    //
    // BODY frame / nominal CoM / left-hand equivalent wrench.
    //
    // Low-pass filtering removes the large gait-frequency component
    // caused by numerical differentiation while preserving payload DC.
    // ==========================================================
    if(dynamicsValid && dt>0.){
      if(!dynamicsReferenceInitialized_){
        filteredDynamicsReference_=dynamicsReference;
        dynamicsReferenceInitialized_=true;
      }else{
        for(int i=0;i<6;++i){
          const double alpha =
              dt/(settings_.dynamicsResidualFilterTau(i)+dt);

          filteredDynamicsReference_(i) +=
              alpha*
              (dynamicsReference(i)-
               filteredDynamicsReference_(i));
        }
      }
    }

    out.filteredDynamicsReference=
        filteredDynamicsReference_;

    if(dynamicsValid &&
       dynamicsReferenceInitialized_){

      // Time-aligned error:
      //
      // e_d(k) =
      // d_hat that was active over [k-1,k]
      // -
      // reconstructed disturbance over [k-1,k]
      out.dynamicsError=
          previousAppliedDisturbance -
          filteredDynamicsReference_;
    }else{
      out.dynamicsError.setZero();
    }

    // 权重更新中的 G^T Ec 保持原有路径。
    out.gradient=G.transpose()*out.ec;// ------------------------------------------------------------
// Raw adaptive gradient:
//
// q = G^T E_c
// ------------------------------------------------------------
out.gradient=G.transpose()*out.ec;

// ------------------------------------------------------------
// First-order low-pass filter:
//
// tau * q_f_dot = q - q_f
//
// Discrete implementation:
// q_f(k) = q_f(k-1) + alpha * [q(k)-q_f(k-1)]
// alpha  = dt / (tau + dt)
//
// The filter is updated whenever a valid dt exists, even during
// adaptation warmup. Therefore q_f is already settled when
// weight learning is enabled.
// ------------------------------------------------------------
if(dt>0.){
  // Each of the six uncertainty channels uses its own
  // first-order low-pass time constant:
  //
  // q_f,i(k) = q_f,i(k-1)
  //          + dt/(tau_i+dt) * (q_i-q_f,i)
  //
  // Force channels remain responsive while torque channels
  // can reject more gait-periodic contamination.
  for(int i=0;i<6;++i){
    const double alpha =
        dt/(settings_.gradientFilterTau(i)+dt);

    filteredGradient_(i) +=
        alpha*(out.gradient(i)-filteredGradient_(i));
  }
}

out.filteredGradient=filteredGradient_;

  // ------------------------------------------------------------
  // Soft deadzone on the filtered adaptive gradient.
  //
  // phi_delta(q) = 0                         , |q| <= delta
  //                q - delta                 , q >  delta
  //                q + delta                 , q < -delta
  //
  // This preserves the sign and removes the normal small residual
  // instead of introducing the discontinuity of a hard threshold.
  // ------------------------------------------------------------
  for(int i=0;i<6;++i){
    const double q = out.filteredGradient(i);
    const double delta = settings_.gradientDeadzone(i);

    if(q > delta){
      out.effectiveGradient(i)=q-delta;
    }else if(q < -delta){
      out.effectiveGradient(i)=q+delta;
    }else{
      out.effectiveGradient(i)=0.;
    }
  }


    for(int i=0;i<6;++i){
      // 修改仅发生在这里：
      // raw Ee -> 20点平均 -> scale -> clip -> Gaussian basis
      // --------------------------------------------------------
      // Basis input.
      //
      // Force channels keep the original filtered scalar
      // prediction-residual feature.
      //
      // Torque channels can additionally use:
      //
      //   xi_tau_cross = c_hat x xi_force
      //
      // where c_hat is the normalized nominal CoM->base
      // BODY lever-arm direction.
      // --------------------------------------------------------
      const double selfXi =
          std::max(-1.,std::min(1., basisEe(i)/settings_.scale(i)));

      out.xiSelf(i)=selfXi;
      out.xi(i)=selfXi;

      if(i>=3 && settings_.comToBaseBody.norm()>1e-9){
        const Eigen::Vector3d leverDirection =
            settings_.comToBaseBody /
            settings_.comToBaseBody.norm();

        // Force feature is already normalized and clipped.
        const Eigen::Vector3d forceXi =
            out.xiSelf.head<3>();

        const Eigen::Vector3d crossXi =
            leverDirection.cross(forceXi);

        out.torqueCrossXi.tail<3>()=crossXi;

        if(settings_.torqueCoupledFeature){
          const double alpha =
              settings_.torqueFeatureAngularRatio;

          const double mixedXi =
              alpha*selfXi +
              (1.-alpha)*crossXi(i-3);

          out.xi(i)=
              std::max(-1.,std::min(1.,mixedXi));
        }
      }
      for(int j=0;j<25;++j){
        const double z=(out.xi(i)-settings_.centers(j))/settings_.widths(j);
        out.features(i,j)=std::exp(-.5*z*z);
      }
      out.features(i,25)=1.;
      // V1.1 section 3.6: no leakage, residual supervision, normalization,
      // history term, output scaling, or variable learning gain.
      // Robust sigma-modification:
      //
      // Wdot_i = -Gamma_i * h_i * q_f,i - sigma_i * W_i
      //
      // The leakage term suppresses long-term parameter drift caused by
      // persistent gait/model residuals.  It is applied only while the
      // adaptive update itself is enabled.
      // ======================================================
      // Original V1.1 adaptive path remains unchanged:
      //
      // Wdot = -Gamma h q_f - sigma W
      // ======================================================
      out.derivative.row(i)=
          -settings_.gamma(i)*
           out.effectiveGradient(i)*
           out.features.row(i)
          -settings_.leakage(i)*
           weights_.row(i);

      // ======================================================
      // V1.2-A additional dynamics supervision.
      //
      // Only constant feature j=25:
      //
      // wdot_bias += -k_d * e_d
      //
      // If payload is under-estimated:
      //
      //   d_hat < d_rec  -> e_d < 0
      //
      // therefore the new term increases the bias.
      // ======================================================
      if(settings_.dynamicsResidualEnabled &&
         dynamicsValid &&
         dynamicsReferenceInitialized_){

        out.derivative(i,25) +=
            -settings_.dynamicsBiasGain(i)*
             out.dynamicsError(i);
      }

      if(enabled) for(int j=0;j<26;++j){
        const double proposal=
            weights_(i,j)+dt*out.derivative(i,j);

        const double limit=
            (j==25)
            ? settings_.biasWeightLimit(i)
            : settings_.weightLimit(i);

        const double bounded=
            std::max(
                -limit,
                std::min(limit,proposal));

        out.projected=
            out.projected ||
            bounded!=proposal;

        weights_(i,j)=bounded;
      }
      out.d(i)=weights_.row(i).dot(out.features.row(i));
    }
    if(!weights_.allFinite() || !out.d.allFinite())throw std::runtime_error("nonfinite V1.1 update");
    return out;
  }
 private:
  using ErrorHistory=Eigen::Matrix<double,6,20,Eigen::DontAlign>;

  Settings settings_;
  Weights weights_=Weights::Zero();

  // 六维预测误差的最近 20 个有效样本。
  // 每一列对应一个时刻的 Ee。
  ErrorHistory eeHistory_=ErrorHistory::Zero();

  // 保存窗口和，使每次更新不必重新累加 20 个样本。
  Vector6 eeSum_=Vector6::Zero();

  // 指向下一次要被最新样本覆盖的最旧历史列。
  std::size_t eeHistoryIndex_=0;
// Filtered q = G^T E_c used by the adaptive update law.
  Vector6 filteredGradient_=Vector6::Zero();

  // V1.2-A LPF state for reconstructed dynamics.
  Vector6 filteredDynamicsReference_=
      Vector6::Zero();

  bool dynamicsReferenceInitialized_=false;

  bool eeHistoryInitialized_=false;
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
