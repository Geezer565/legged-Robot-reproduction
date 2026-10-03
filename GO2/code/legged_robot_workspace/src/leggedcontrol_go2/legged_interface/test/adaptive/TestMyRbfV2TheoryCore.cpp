#include "legged_interface/adaptive/MyRbfV2Core.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <random>
#include <stdexcept>

using namespace legged::adaptive::v2;
namespace {
void require(bool ok, const char* message) { if (!ok) throw std::runtime_error(message); }
void near(double a, double b, double tolerance, const char* message) {
  require(std::isfinite(a) && std::isfinite(b) && std::abs(a-b)<=tolerance, message);
}
template<class F> void rejects(F f) {
  bool caught=false; try { f(); } catch (const std::exception&) { caught=true; }
  require(caught,"invalid input accepted");
}

void featuresAndAllWeights() {
  Settings cfg; cfg.featureMode=FeatureMode::ContactAware; cfg.gamma.fill(.7);
  cfg.outputScale={{1,2,3,4,5,6}};
  Core core(cfg); FeatureInput input;
  input.causalResidual.fill(.2); input.chi.fill(.3);
  input.baseVelocity.fill(-.1); input.referenceVelocity.fill(.4);
  const auto centers=core.fixedFeatureMap().centers();
  const double width=core.fixedFeatureMap().width();
  const auto baseline=core.physicalGradient(input);
  unsigned updates=0;
  for (unsigned mask=0; mask<16; ++mask) {
    for (unsigned k=0; k<4; ++k) input.contacts[k]=(mask>>k)&1;
    const auto eval=core.evaluate(0,State{},input);
    for (unsigned i=0; i<6; ++i) for (unsigned j=0; j<26; ++j) {
      require(eval.physicalGradient[i][j]>0,"participating Gaussian disappeared");
      near(eval.weightDerivative[i][j],.7*eval.physicalGradient[i][j]*.3,1e-15,
           "tracking update contains gate, normalization, or missing node");
      if (mask==0 && eval.weightDerivative[i][j]>0) ++updates;
    }
  }
  require(updates==156,"not all 156 weights updated");
  input.contacts.fill(false);
  for (unsigned k=0; k<8; ++k) {
    auto changed=input;
    if (k==0) changed.causalResidual[0]+=.5;
    else if (k==1) changed.chi[0]+=.5;
    else if (k==2) changed.baseVelocity[0]+=.5;
    else if (k==3) changed.referenceVelocity[0]+=.5;
    else changed.contacts[k-4]=true;
    require(core.physicalGradient(changed)[0]!=baseline[0],"an input coordinate is ignored");
  }
  State state; for (auto& axis:state.weights) axis.fill(.4);
  input.chi.fill(0);
  const auto noTracking=core.evaluate(10,state,input);
  for (const auto& axis:noTracking.weightDerivative) for(double v:axis)
    require(v==0,"hidden leakage or nontracking update");
  require(centers==core.fixedFeatureMap().centers() && width==core.fixedFeatureMap().width(),
          "feature centers/width changed online");
  input.baseVelocity[1]=std::numeric_limits<double>::quiet_NaN();
  rejects([&]{core.evaluate(0,state,input);});
}

void robustAndProjection() {
  Settings cfg; cfg.robustMode=RobustMode::Sign; cfg.robustGamma.fill(.8);
  Core core(cfg); FeatureInput input; input.chi={{.4,-.3,0,-0.0,.1,-.2}};
  State state; state.robustBound.fill(.6);
  const auto eval=core.evaluate(std::numeric_limits<double>::max(),state,input);
  for (unsigned i=0; i<6; ++i) {
    const double sign=(input.chi[i]>0)-(input.chi[i]<0);
    near(eval.robustWrench[i],.6*sign,0,"sign output wrong");
    near(eval.boundDerivative[i],.8*std::abs(input.chi[i]),0,"sign rho derivative wrong");
    near(eval.smoothingWidth[i],0,0,"sign mode has a boundary layer");
  }
  for (unsigned i=0;i<6;++i) state.weights[i].fill(cfg.weightLimit[i]);
  input.chi.fill(.2);
  const auto outward=core.evaluate(0,state,input);
  require(outward.projectedComponents==156,"projection did not cover every node");
  input.chi.fill(-.2);
  const auto inward=core.evaluate(0,state,input);
  for (const auto& axis:inward.weightDerivative) for(double v:axis)
    require(v<0,"projection removed inward update");
  cfg.robustMode=static_cast<RobustMode>(90); rejects([&]{Core bad(cfg);});
}

void legacyRegression() {
  Settings cfg; Core core(cfg); State state; FeatureInput input;
  for(unsigned i=0;i<6;++i) {
    input.causalResidual[i]=.12*i-.25; input.chi[i]=.1*i-.23;
    state.robustBound[i]=.1+.02*i;
    for(unsigned j=0;j<26;++j) state.weights[i][j]=.03*std::cos(i+j);
  }
  require(cfg.featureMode==FeatureMode::ResidualOnly && cfg.robustMode==RobustMode::Smooth,
          "legacy defaults changed");
  const double t=.71;
  const auto e=core.evaluate(t,state,input.causalResidual,input.chi);
  const auto f=core.evaluate(t,state,input);
  require(e.physicalGradient==f.physicalGradient && e.weightDerivative==f.weightDerivative,
          "old and new residual-only overloads disagree");
  for(unsigned i=0;i<6;++i) {
    double estimate=state.weights[i][0]*cfg.outputScale[i];
    for(unsigned j=0;j<25;++j) {
      const double spacing=2*cfg.extent[i]/24;
      const double n=(input.causalResidual[i]/cfg.residualScale[i]+cfg.extent[i]-spacing*j)/
                     (spacing*cfg.widthRatio);
      const double phi=cfg.outputScale[i]*std::exp(-.5*n*n);
      near(e.physicalGradient[i][j+1],phi,1e-14,"legacy Gaussian changed");
      estimate+=state.weights[i][j+1]*phi;
    }
    near(e.estimate[i],estimate,1e-14,"legacy estimate changed");
    const double nu=cfg.nu0[i]/std::pow(1+t/cfg.nuTime[i],1+cfg.nuExponent[i]);
    const double ratio=input.chi[i]/std::hypot(input.chi[i],nu);
    near(e.robustWrench[i],state.robustBound[i]*ratio,1e-14,"legacy smooth output changed");
    near(e.boundDerivative[i],cfg.robustGamma[i]*input.chi[i]*ratio,1e-14,"legacy adaptation changed");
    near(e.externalWrenchEstimate[i],e.estimate[i],0,"external alias differs");
    near(e.leftHandDisturbanceEstimate[i],-e.estimate[i],0,"left-hand alias sign wrong");
    near(e.requiredControlCorrection[i],-(e.estimate[i]+e.robustWrench[i]),0,"correction sign wrong");
  }
}

void teacherCompositeLaw() {
  Settings cfg;
  cfg.updateLaw=UpdateLaw::TeacherComposite;
  cfg.robotMass=10.;
  cfg.gamma.fill(2.);
  cfg.outputScale.fill(5.);
  cfg.featureMode=FeatureMode::ResidualOnly;
  Core core(cfg);
  FeatureInput input;
  input.causalResidual.fill(0.);
  input.chi.fill(-.4);
  input.learningSignal.fill(0.);
  input.learningSignal[0]=.125;
  input.learningSignalValid=true;
  const auto enabled=core.evaluate(0.,State{},input);
  near(enabled.weightDerivative[0][0],-1.25,1e-14,
       "V3 general B_w^T E_c signal or correction sign is wrong");
  near(enabled.robustWrench[0],0.,0.,
       "zero robust-bound state should remain zero");
  require(enabled.learningEnabled,"valid previous-policy error did not enable V3 learning");
  for(unsigned j=0;j<26;++j)
    require(enabled.weightDerivative[0][j]<0,
            "positive B_w^T E_c must decrease the V3 correction estimate");

  Settings composite=cfg;
  composite.predictionGain.fill(.5);
  Core compositeCore(composite);
  auto noHistoryInput=input;
  noHistoryInput.learningSignal.fill(0.);
  noHistoryInput.causalResidual.fill(0.);
  noHistoryInput.causalResidual[0]=-3.;
  const auto noHistory=compositeCore.evaluate(0.,State{},noHistoryInput);
  for(double value:noHistory.weightDerivative[0])
    require(value==0.,
            "V3 must not fit the instantaneous residual as a supervised label");

  Sample integralSample;
  integralSample.certified=true;
  integralSample.importance[0]=1.;
  integralSample.physicalGradient[0][0]=1.;
  integralSample.label[0]=-2.;
  const auto historyDriven=compositeCore.evaluate(
      0.,State{},noHistoryInput,{integralSample});
  const double expectedDerivative=-composite.gamma[0]*
      composite.predictionGain[0]*2./(1.+composite.epsilon);
  near(historyDriven.weightDerivative[0][0],expectedDerivative,1e-14,
       "V3 integral-history correction has the wrong sign or scale");
  require(historyDriven.learningEnabled,
          "a certified history sample did not enable composite learning");

  State converging;
  auto historyOnly=noHistoryInput;
  historyOnly.learningSignalValid=false;
  constexpr double dt=.01;
  for(unsigned step=0;step<500;++step) {
    const auto update=compositeCore.evaluate(
        step*dt,converging,historyOnly,{integralSample});
    converging.weights[0][0]+=dt*update.weightDerivative[0][0];
  }
  near(converging.weights[0][0],-2.,.02,
       "V3 certified history did not converge at an identifiable operating point");

  input.learningSignalValid=false;
  const auto skipped=core.evaluate(0.,State{},input);
  require(!skipped.learningEnabled,"invalid previous-policy sample enabled V3 learning");
  for(const auto& axis:skipped.weightDerivative) for(double value:axis)
    require(value==0.,"V3 changed a weight without a valid previous-policy sample");
  near(enabled.boundDerivative[0],0.,0.,
       "Word V3 must not inherit the V2 robust-bound update");
  near(skipped.boundDerivative[0],0.,0.,
       "Word V3 must keep the V2 robust-bound update disabled");
}

void cancellationAndEulerEnergy() {
  std::mt19937 random(20260908); std::uniform_real_distribution<double> u(-1,1);
  double maxCross=0, maxBound=-1e100;
  for (auto mode:{RobustMode::Sign,RobustMode::Smooth}) {
    Settings cfg; cfg.robustMode=mode; cfg.featureMode=FeatureMode::ContactAware;
    cfg.gamma.fill(.7); cfg.robustGamma.fill(.9); cfg.weightLimit.fill(2);
    Core core(cfg);
    for(unsigned trial=0;trial<1000;++trial) {
      State state; Weights ideal{}; FeatureInput input; Axes wr{},damping{},wrench{},xdot{},rhoStar{};
      for(unsigned i=0;i<6;++i) {
        input.chi[i]=u(random); input.causalResidual[i]=u(random);
        input.baseVelocity[i]=u(random); input.referenceVelocity[i]=u(random);
        state.robustBound[i]=.4+.2*u(random); rhoStar[i]=.3; damping[i]=1.7;
        for(unsigned j=0;j<26;++j) {
          ideal[i][j]=.3*u(random); state.weights[i][j]=.3*u(random);
        }
      }
      for(unsigned k=0;k<4;++k) input.contacts[k]=(trial>>k)&1;
      const auto e=core.evaluate(.01*trial,state,input);
      double V=0,Vdot=0,bound=0,quadratic=0,cross=0;
      for(unsigned i=0;i<6;++i) {
        double external=.3*u(random);
        for(unsigned j=0;j<26;++j) {
          external+=e.physicalGradient[i][j]*ideal[i][j];
          const double error=state.weights[i][j]-ideal[i][j];
          cross+=-input.chi[i]*e.physicalGradient[i][j]*error+
                 error/cfg.gamma[i]*e.weightDerivative[i][j];
          V+=.5*error*error/cfg.gamma[i];
          Vdot+=error*e.weightDerivative[i][j]/cfg.gamma[i];
          quadratic+=e.weightDerivative[i][j]*e.weightDerivative[i][j]/cfg.gamma[i];
        }
        wrench[i]=wr[i]-damping[i]*input.chi[i]+e.requiredControlCorrection[i];
        xdot[i]=wrench[i]-wr[i]+external; // actual external-force plant
        const double rhoError=state.robustBound[i]-rhoStar[i];
        V+=.5*input.chi[i]*input.chi[i]+.5*rhoError*rhoError/cfg.robustGamma[i];
        Vdot+=input.chi[i]*xdot[i]+rhoError*e.boundDerivative[i]/cfg.robustGamma[i];
        quadratic+=xdot[i]*xdot[i]+e.boundDerivative[i]*e.boundDerivative[i]/cfg.robustGamma[i];
        bound+=-damping[i]*input.chi[i]*input.chi[i]+rhoStar[i]*e.smoothingWidth[i];
      }
      maxCross=std::max(maxCross,std::abs(cross));
      maxBound=std::max(maxBound,Vdot-bound);
      require(std::abs(cross)<1e-12,"actual weight cross terms did not cancel");
      require(Vdot<=bound+1e-12,"robust continuous bound failed");
      near(core.constraint(input.chi,wr,damping,e).violation(wrench),0,1e-12,"CLF sign mismatch");
      for(unsigned i=0;i<6;++i) wrench[i]+=input.chi[i];
      require(core.constraint(input.chi,wr,damping,e).violation(wrench)>0,"harmful wrench accepted");
      for(double dt:{.02,.01,.005}) {
        double nextV=0;
        for(unsigned i=0;i<6;++i) {
          const double x=input.chi[i]+dt*xdot[i];
          const double rho=state.robustBound[i]+dt*e.boundDerivative[i]-rhoStar[i];
          nextV+=.5*x*x+.5*rho*rho/cfg.robustGamma[i];
          for(unsigned j=0;j<26;++j) {
            const double w=state.weights[i][j]+dt*e.weightDerivative[i][j]-ideal[i][j];
            nextV+=.5*w*w/cfg.gamma[i];
          }
        }
        near(nextV-V,dt*Vdot+.5*dt*dt*quadratic,1e-12,"Euler one-step energy identity failed");
        require(nextV-V<=dt*bound+.5*dt*dt*quadratic+1e-12,"one-step energy upper bound failed");
      }
    }
  }
  std::cout << "cancellation_cases=2000 max_cross=" << maxCross
            << " max_continuous_bound_violation=" << maxBound << '\n';
}
}
int main() {
  try {
    featuresAndAllWeights(); robustAndProjection(); legacyRegression(); teacherCompositeLaw(); cancellationAndEulerEnergy();
    std::cout << "PASS: fixed contact features, all156, sign/smooth, physical cancellation, Euler energy, legacy regression\n";
    return 0;
  } catch(const std::exception& e) { std::cerr << "FAIL: " << e.what() << '\n'; return 1; }
}
