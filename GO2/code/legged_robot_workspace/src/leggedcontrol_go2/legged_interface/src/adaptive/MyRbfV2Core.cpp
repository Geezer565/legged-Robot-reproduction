#include "legged_interface/adaptive/MyRbfV2Core.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace legged { namespace adaptive { namespace v2 {
namespace {
void finite(double x) {
  if (!std::isfinite(x)) throw std::invalid_argument("MyRBF V2 requires finite values");
}
void positive(double x) { finite(x); if (x<=0) throw std::invalid_argument("MyRBF V2 requires positive values"); }
void nonnegative(double x) { finite(x); if (x<0) throw std::invalid_argument("MyRBF V2 requires nonnegative values"); }
void checkedResult(double x) {
  if (!std::isfinite(x)) throw std::overflow_error("MyRBF V2 arithmetic overflow");
}
double deadzone(double e, double bound) {
  return std::copysign(std::max(std::abs(e)-bound,0.0),e);
}
}

Core::Core(Settings settings) : settings_(std::move(settings)),
    fixedFeatureMap_(settings_.residualScale, settings_.chiScale,
                     settings_.baseVelocityScale, settings_.referenceVelocityScale,
                     settings_.outputScale, settings_.contactFeatureWidth) {
  if (settings_.robustMode!=RobustMode::Sign && settings_.robustMode!=RobustMode::Smooth)
    throw std::invalid_argument("MyRBF V2 unknown robust mode");
  if (settings_.featureMode!=FeatureMode::ResidualOnly && settings_.featureMode!=FeatureMode::ContactAware)
    throw std::invalid_argument("MyRBF V2 unknown feature mode");
  if (settings_.updateLaw!=UpdateLaw::V2Tracking &&
      settings_.updateLaw!=UpdateLaw::TeacherComposite &&
      settings_.updateLaw!=UpdateLaw::ResidualConvergence)
    throw std::invalid_argument("MyRBF unknown update law");
  positive(settings_.robotMass);
  positive(settings_.widthRatio); positive(settings_.epsilon);
  positive(settings_.compositeWindow);
  if (!settings_.maxHistorySamples) throw std::invalid_argument("MyRBF V2 history capacity must be positive");
  for (unsigned i=0;i<6;++i) {
    positive(settings_.residualScale[i]); positive(settings_.outputScale[i]);
    positive(settings_.extent[i]); positive(settings_.gamma[i]);
    finite(settings_.residualBias[i]);
    nonnegative(settings_.residualDeadzone[i]);
    nonnegative(settings_.convergenceLeakage[i]);
    nonnegative(settings_.predictionGain[i]); positive(settings_.weightLimit[i]);
    positive(settings_.robustGamma[i]); positive(settings_.nu0[i]);
    positive(settings_.nuTime[i]); positive(settings_.nuExponent[i]);
  }
}

Weights Core::physicalGradient(const Axes& residual) const {
  Weights g{};
  for (unsigned i=0;i<6;++i) {
    finite(residual[i]);
    const double z=residual[i]/settings_.residualScale[i]; checkedResult(z);
    const double spacing=2*settings_.extent[i]/24;
    const double width=spacing*settings_.widthRatio; positive(width);
    g[i][0]=settings_.outputScale[i];
    for (unsigned j=0;j<25;++j) {
      const double center=-settings_.extent[i]+spacing*j;
      const double normalized=(z-center)/width;
      // For large finite inputs the Gaussian may safely underflow to zero.
      g[i][j+1]=settings_.outputScale[i]*std::exp(-.5*normalized*normalized);
      checkedResult(g[i][j+1]);
    }
  }
  return g;
}

Evaluation Core::evaluate(double t,const State& state,const Axes& residual,
                          const Axes& chi,const std::vector<Sample>& history) const {
  FeatureInput input; input.causalResidual=residual; input.chi=chi;
  return evaluate(t,state,input,history);
}

Weights Core::physicalGradient(const FeatureInput& input) const {
  return settings_.featureMode==FeatureMode::ContactAware
      ? fixedFeatureMap_.physicalGradient(input) : physicalGradient(input.causalResidual);
}

Evaluation Core::evaluate(double t,const State& state,const FeatureInput& input,
                          const std::vector<Sample>& history) const {
  auto out=evaluateGradient(t,state,physicalGradient(input),input.chi,history);
  if(settings_.updateLaw==UpdateLaw::TeacherComposite) {
    bool historyEnabled=false;
    for(const auto& sample:history) for(unsigned i=0;i<6;++i)
      historyEnabled=historyEnabled ||
          (sample.importance[i]>0. && settings_.predictionGain[i]>0.);
    out.learningEnabled=input.learningSignalValid || historyEnabled;
    out.projectedComponents=0;
    for(unsigned i=0;i<6;++i) {
      finite(input.learningSignal[i]);
      out.learningSignal[i]=input.learningSignalValid
          ? input.learningSignal[i] : 0.;
      for(unsigned j=0;j<26;++j) {
        // Word's B_w^T E_c cancellation term plus a derivative-free,
        // completed finite-window concurrent-learning term.
        double v=-settings_.gamma[i]*(
            out.physicalGradient[i][j]*out.learningSignal[i]
            +out.historyCorrection[i][j]);
        checkedResult(v);
        if((state.weights[i][j]>=settings_.weightLimit[i] && v>0) ||
           (state.weights[i][j]<=-settings_.weightLimit[i] && v<0)) {
          v=0; ++out.projectedComponents;
        }
        out.weightDerivative[i][j]=v;
      }
    }
  } else if(settings_.updateLaw==UpdateLaw::ResidualConvergence) {
    // Empirical V4: normalized residual prediction with forgetting.  This is
    // deliberately separate from the Word/V3 composite-error law and carries
    // no Lyapunov/stability claim.
    out.learningEnabled=true;
    out.learningSignal=input.causalResidual;
    out.projectedComponents=0;
    for(unsigned i=0;i<6;++i) {
      const double error=input.causalResidual[i]-out.estimate[i];
      double normalization=settings_.epsilon;
      for(double value:out.physicalGradient[i])
        normalization+=value*value;
      checkedResult(error); checkedResult(normalization);
      for(unsigned j=0;j<26;++j) {
        double v=settings_.gamma[i]*out.physicalGradient[i][j]*error/
            normalization-settings_.convergenceLeakage[i]*state.weights[i][j];
        checkedResult(v);
        if((state.weights[i][j]>=settings_.weightLimit[i] && v>0) ||
           (state.weights[i][j]<=-settings_.weightLimit[i] && v<0)) {
          v=0; ++out.projectedComponents;
        }
        out.weightDerivative[i][j]=v;
      }
    }
  } else {
    out.learningEnabled=true;
    out.learningSignal=input.chi;
  }
  return out;
}

Evaluation Core::evaluateGradient(double t,const State& state,const Weights& gradient,
                                  const Axes& chi,const std::vector<Sample>& history) const {
  nonnegative(t);
  if(history.size()>settings_.maxHistorySamples) throw std::invalid_argument("MyRBF V2 history capacity exceeded");
  Evaluation out; out.physicalGradient=gradient;
  for(unsigned i=0;i<6;++i) {
    finite(chi[i]); nonnegative(state.robustBound[i]);
    for(unsigned j=0;j<26;++j) {
      finite(state.weights[i][j]);
      if(std::abs(state.weights[i][j])>settings_.weightLimit[i])
        throw std::invalid_argument("MyRBF V2 state is outside its weight box");
      out.estimate[i]+=state.weights[i][j]*out.physicalGradient[i][j];
      if(j) out.coverage[i]=std::max(out.coverage[i],out.physicalGradient[i][j]/settings_.outputScale[i]);
    }
    checkedResult(out.estimate[i]);
  }
  for(const auto& sample:history) for(unsigned i=0;i<6;++i) {
    nonnegative(sample.importance[i]);
    if(sample.importance[i]==0 || settings_.predictionGain[i]==0) continue;
    if(!sample.certified) throw std::invalid_argument("MyRBF V2 refuses uncertified prediction labels");
    finite(sample.label[i]); nonnegative(sample.noiseBound[i]);
    double e=-sample.label[i],N=settings_.epsilon;
    for(unsigned j=0;j<26;++j) {
      finite(sample.physicalGradient[i][j]);
      e+=sample.physicalGradient[i][j]*state.weights[i][j];
      N+=sample.physicalGradient[i][j]*sample.physicalGradient[i][j];
    }
    checkedResult(e); checkedResult(N);
    const double D=deadzone(e,sample.noiseBound[i]);
    const double rate=settings_.predictionGain[i]*sample.importance[i]/N;
    for(unsigned j=0;j<26;++j)
      out.historyCorrection[i][j]+=rate*sample.physicalGradient[i][j]*D;
    out.predictionDissipation[i]+=rate*D*D;
    checkedResult(out.predictionDissipation[i]);
  }
  for(unsigned i=0;i<6;++i) {
    if (settings_.updateLaw==UpdateLaw::TeacherComposite ||
        settings_.updateLaw==UpdateLaw::ResidualConvergence) {
      // The Word-style V3 defines only the composite-error RBF update.  Keep
      // it, and V4's empirical residual update, out of the V2 robust-bound path.
      out.robustWrench[i]=0;
      out.boundDerivative[i]=0;
      out.smoothingWidth[i]=0;
    } else if (settings_.robustMode==RobustMode::Sign) {
      const double sign=(chi[i]>0) ? 1.0 : ((chi[i]<0) ? -1.0 : 0.0);
      out.robustWrench[i]=state.robustBound[i]*sign;
      out.boundDerivative[i]=settings_.robustGamma[i]*std::abs(chi[i]);
      out.smoothingWidth[i]=0;
    } else {
      // Smooth mode admits a boundary-layer remainder, not the Sign theorem.
      // Underflow is an explicit error, never a hidden fixed-width clamp.
      out.smoothingWidth[i]=settings_.nu0[i]*std::exp(-(1+settings_.nuExponent[i])*std::log1p(t/settings_.nuTime[i]));
      positive(out.smoothingWidth[i]);
      const double denominator=std::hypot(chi[i],out.smoothingWidth[i]);
      checkedResult(denominator);
      const double ratio=chi[i]/denominator;
      out.robustWrench[i]=state.robustBound[i]*ratio;
      out.boundDerivative[i]=settings_.robustGamma[i]*chi[i]*ratio;
    }
    checkedResult(out.robustWrench[i]); checkedResult(out.boundDerivative[i]);
    out.externalWrenchEstimate[i]=out.estimate[i];
    out.leftHandDisturbanceEstimate[i]=-out.estimate[i];
    out.requiredControlCorrection[i]=-(out.estimate[i]+out.robustWrench[i]);
    checkedResult(out.requiredControlCorrection[i]);
    for(unsigned j=0;j<26;++j) {
      // External-force convention: +Gamma*phi*chi cancels the plant term
      // -chi*phi*(W_hat-W_star). All 156 output weights participate.
      double v=settings_.gamma[i]*(out.physicalGradient[i][j]*chi[i]
          -out.historyCorrection[i][j]);
      checkedResult(v);
      if((state.weights[i][j]>=settings_.weightLimit[i] && v>0) ||
         (state.weights[i][j]<=-settings_.weightLimit[i] && v<0)) {
        v=0; ++out.projectedComponents;
      }
      out.weightDerivative[i][j]=v;
    }
  }
  return out;
}

WrenchConstraint Core::constraint(const Axes& chi,const Axes& wr,
                                   const Axes& damping,const Evaluation& e) const {
  WrenchConstraint c; c.normal=chi;
  for(unsigned i=0;i<6;++i) {
    finite(chi[i]); finite(wr[i]); positive(damping[i]);
    finite(e.estimate[i]); finite(e.robustWrench[i]);
    c.rhs+=chi[i]*(wr[i]-e.estimate[i]-damping[i]*chi[i]-e.robustWrench[i]);
  }
  checkedResult(c.rhs); return c;
}
double WrenchConstraint::violation(const Axes& achieved) const {
  finite(rhs); double value=-rhs;
  for(unsigned i=0;i<6;++i) { finite(normal[i]); finite(achieved[i]); value+=normal[i]*achieved[i]; }
  checkedResult(value); return value;
}

}}} // namespace legged::adaptive::v2
