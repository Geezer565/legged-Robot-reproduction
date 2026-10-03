#include "legged_interface/adaptive/MyRbfV2Core.h"
#include <cmath>
#include <iostream>
#include <limits>
#include <random>
#include <stdexcept>
#include <algorithm>
using namespace legged::adaptive::v2;
namespace {
void require(bool pass,const char* message) { if(!pass) throw std::runtime_error(message); }
template<class F> void rejects(F f) { bool caught=false; try {f();}catch(const std::exception&){caught=true;}require(caught,"expected invalid input rejection"); }
double norm(const Axes& a) { double n=0;for(double x:a)n+=x*x;return std::sqrt(n); }
struct Model { Axes e{},s{};State network; };
Model plus(const Model& a,const Model& b,double h) {
 Model out=a;
 for(unsigned i=0;i<6;++i) {
  out.e[i]+=h*b.e[i];out.s[i]+=h*b.s[i];out.network.robustBound[i]+=h*b.network.robustBound[i];
  for(unsigned j=0;j<26;++j)out.network.weights[i][j]+=h*b.network.weights[i][j];
 }
 return out;
}
Model derivative(const Core& core,double t,const Model& y) {
 Axes residual{},wr{},damping{{3,3,3,3,3,3}},achieved{};
 for(unsigned i=0;i<6;++i){residual[i]=.65*std::sin(.23*t+.43*i);wr[i]=-y.s[i];}
 const auto eval=core.evaluate(t,y.network,residual,y.s);
 Model d;d.network.weights=eval.weightDerivative;d.network.robustBound=eval.boundDerivative;
 for(unsigned i=0;i<6;++i) {
  double unknown=.04+.12*std::sin(1.3*t+.43*i)+.05*std::cos(.53*t);
  for(unsigned j=0;j<26;++j)unknown+=eval.physicalGradient[i][j]*.035*std::cos(.31*j+.43*i);
  achieved[i]=wr[i]-eval.estimate[i]-3*y.s[i]-eval.robustWrench[i];
  d.e[i]=y.s[i]-y.e[i];
  d.s[i]=achieved[i]+unknown+d.e[i];
 }
 require(core.constraint(y.s,wr,damping,eval).violation(achieved)<1e-10,"model violates force constraint");
 return d;
}
Model simulate(double h) {
 Settings cfg;cfg.weightLimit.fill(5);Core core(cfg);Model y;
 y.e.fill(.3);y.s.fill(.1);y.network.robustBound.fill(.05);
 const int count=static_cast<int>(40/h+.5);
 for(int k=0;k<count;++k) {
  double t=k*h;
  const auto a=derivative(core,t,y);const auto b=derivative(core,t+h/2,plus(y,a,h/2));
  const auto c=derivative(core,t+h/2,plus(y,b,h/2));const auto d=derivative(core,t+h,plus(y,c,h));
  y=plus(plus(plus(plus(y,a,h/6),b,h/3),c,h/3),d,h/6);
 }
 require(norm(y.e)<.01,"representative tracking did not become small");
 require(norm(y.s)<.02,"representative velocity surface did not become small");
 return y;
}
}
int main() { try {
 Core core;Axes zero{},chi{};chi.fill(.4);State state;
 auto eval=core.evaluate(0,state,zero,chi);
 require(eval.weightDerivative[0][13]>0,"tracking must update Gaussian nodes");
 Axes outside{};outside.fill(1e5);
 auto uncovered=core.evaluate(0,state,outside,chi);
 require(uncovered.coverage[0]==0 && uncovered.weightDerivative[0][0]>.0,"coverage must not freeze tracking");
 state.weights[0][1]=.5;
 require(core.evaluate(0,state,zero,zero).weightDerivative[0][1]==0,"V2 must not leak weights");
 state.weights[0][0]=2;eval=core.evaluate(0,state,zero,chi);
 require(eval.weightDerivative[0][0]==0 && eval.projectedComponents>0,"outward projection failed");
 chi[0]=-.4;require(core.evaluate(0,state,zero,chi).weightDerivative[0][0]<0,"inward projection frozen");
 rejects([&]{core.evaluate(-1,state,zero,chi);});
 Axes bad=zero;bad[0]=std::numeric_limits<double>::quiet_NaN();rejects([&]{core.evaluate(0,state,bad,chi);});
 State invalid=state;invalid.weights[0][0]=2.01;rejects([&]{core.evaluate(0,invalid,zero,chi);});
 invalid=state;invalid.robustBound[0]=-1;rejects([&]{core.evaluate(0,invalid,zero,chi);});
 Settings wrong;wrong.nuExponent[0]=0;rejects([&]{Core c(wrong);});
 Settings pred;pred.predictionGain.fill(1);Core cp(pred);Sample sample;sample.physicalGradient=cp.physicalGradient(zero);sample.importance.fill(1);
 rejects([&]{cp.evaluate(0,State{},zero,zero,{sample});});
 sample.certified=true;sample.label.fill(.05);sample.noiseBound.fill(.1);
 require(cp.evaluate(0,State{},zero,zero,{sample}).weightDerivative[0][0]==0,"deadzone must suppress uncertain labels");
 sample.noiseBound[0]=-1;rejects([&]{cp.evaluate(0,State{},zero,zero,{sample});});
 require(core.evaluate(100,state,zero,chi).smoothingWidth[0]<core.evaluate(0,state,zero,chi).smoothingWidth[0],"smoothing width did not decrease");
 // Finite-precision underflow must be explicit, not a constant-width fallback.
 rejects([&]{core.evaluate(std::numeric_limits<double>::max(),state,zero,chi);});

 std::mt19937_64 random(20260905);std::uniform_real_distribution<double> uniform(-1,1);
 double maxViolation=-1e100;unsigned projected=0;
 for(unsigned k=0;k<10000;++k) {
  Settings cfg;cfg.predictionGain.fill(.4);cfg.gamma.fill(.7);Core c(cfg);
  State st;Axes r{},x{},bstar{},residual{},wr{},damping{{1,1,1,1,1,1}};Weights ideal{};
  Sample h;h.certified=true;h.importance.fill(1);h.noiseBound.fill(.05);
  for(unsigned i=0;i<6;++i) {
   r[i]=uniform(random);x[i]=uniform(random);bstar[i]=.3;residual[i]=.3*uniform(random);st.robustBound[i]=.5+.4*uniform(random);
   for(unsigned j=0;j<26;++j){st.weights[i][j]=uniform(random);ideal[i][j]=.5*uniform(random);}
   st.weights[i][k%26]=(k%2 ? 2 : -2);
  }
  h.physicalGradient=c.physicalGradient(r);
  for(unsigned i=0;i<6;++i){h.label[i]=.05*uniform(random);for(unsigned j=0;j<26;++j)h.label[i]+=h.physicalGradient[i][j]*ideal[i][j];}
  const auto a=c.evaluate(.01*k,st,r,x,{h});projected+=a.projectedComponents;
  double actual=0,bound=0;Axes wrench{};
  for(unsigned i=0;i<6;++i) {
   actual-=x[i]*x[i];actual+=x[i]*residual[i]-x[i]*a.robustWrench[i];
   for(unsigned j=0;j<26;++j){const double tw=st.weights[i][j]-ideal[i][j];actual-=x[i]*a.physicalGradient[i][j]*tw;actual+=tw/cfg.gamma[i]*a.weightDerivative[i][j];}
   actual+=(st.robustBound[i]-bstar[i])/cfg.robustGamma[i]*a.boundDerivative[i];
   bound+=-x[i]*x[i]-a.predictionDissipation[i]+bstar[i]*a.smoothingWidth[i];
   wrench[i]=wr[i]-a.estimate[i]-x[i]-a.robustWrench[i];
  }
  maxViolation=std::max(maxViolation,actual-bound);require(actual<=bound+1e-9,"C++ V2 Lyapunov cancellation failed");
  const auto constraint=c.constraint(x,wr,damping,a);require(std::abs(constraint.violation(wrench))<1e-9,"force constraint sign error");
  for(unsigned i=0;i<6;++i)wrench[i]+=x[i];
  require(constraint.violation(wrench)>0,"constraint missed harmful force modification");
 }
 const auto coarse=simulate(.002);const auto fine=simulate(.001);
 double discrepancy=0;
 for(unsigned i=0;i<6;++i){discrepancy=std::max(discrepancy,std::abs(coarse.e[i]-fine.e[i]));discrepancy=std::max(discrepancy,std::abs(coarse.s[i]-fine.s[i]));}
 require(discrepancy<1e-5,"integration refinement disagrees");
 std::cout<<"PASS: V2 core invariants, invalid inputs, projection, deadzone and control constraint\n"
          <<"cancellation_cases=10000 projected_components="<<projected<<" max_Vdot_violation="<<maxViolation<<"\n"
          <<"representative_40s_error="<<norm(fine.e)<<" s="<<norm(fine.s)<<" refinement_difference="<<discrepancy<<"\n"
          <<"Scope: continuous model core; NOT a fixed-sample or Go2 controller certificate\n";
 return 0;
 }catch(const std::exception& error){std::cerr<<"FAIL: "<<error.what()<<'\n';return 1;}}
