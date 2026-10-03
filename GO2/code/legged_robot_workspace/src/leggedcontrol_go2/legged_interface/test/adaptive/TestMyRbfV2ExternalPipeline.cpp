#include <gtest/gtest.h>
#include "legged_interface/adaptive/MyRbfV2Pipeline.h"
#include "legged_interface/adaptive/MyRbfV2Snapshot.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace legged { namespace adaptive { namespace {

TEST(MyRbfV2ExternalPipeline, ExternalResidualBypassesObserverAndPreservesPhysicalChi) {
  v2::Settings cfg; cfg.robustMode=v2::RobustMode::Sign;
  cfg.featureMode=v2::FeatureMode::ContactAware;
  MyRbfV2Pipeline pipeline(10,12,false,0,1e-6,cfg);
  v2::FeatureInput input; input.chi={{.3,-.2,.1,-.4,.25,-.15}};
  input.baseVelocity.fill(.2); input.referenceVelocity.fill(-.1);
  input.contacts={{true,false,false,true}}; input.causalResidual.fill(-99);
  const vector6_t external=vector6_t::Constant(.7);
  const auto result=pipeline.advanceExternalResidual(5,.01,external,input);
  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.residual.valid);
  EXPECT_TRUE(pipeline.hasEvaluation());
  for(unsigned i=0;i<6;++i) {
    EXPECT_DOUBLE_EQ(result.featureInput.causalResidual[i],.7);
    EXPECT_DOUBLE_EQ(result.residual.rawResidual(i),.7);
    EXPECT_DOUBLE_EQ(result.residual.filteredResidual(i),.7);
    EXPECT_DOUBLE_EQ(result.featureInput.chi[i],input.chi[i]);
    EXPECT_DOUBLE_EQ(result.chi(i),input.chi[i]);
    EXPECT_LT(result.requiredControlCorrection(i)*input.chi[i],0);
    EXPECT_DOUBLE_EQ(result.externalWrenchEstimate(i),result.estimateAfterUpdate(i));
    EXPECT_DOUBLE_EQ(result.leftHandDisturbanceEstimate(i),-result.externalWrenchEstimate(i));
    EXPECT_DOUBLE_EQ(result.requiredControlCorrection(i),
                     -result.externalWrenchEstimate(i)-result.robustWrench(i));
    for(unsigned j=0;j<26;++j) {
      EXPECT_NE(pipeline.state().weights[i][j],0);
      EXPECT_DOUBLE_EQ(pipeline.state().weights[i][j],
                       .01*result.evaluationBeforeUpdate.weightDerivative[i][j]);
    }
  }
  EXPECT_EQ(pipeline.evaluation().physicalGradient,result.evaluationAfterUpdate.physicalGradient);
  EXPECT_EQ(pipeline.evaluation().externalWrenchEstimate,result.evaluationAfterUpdate.externalWrenchEstimate);
  EXPECT_EQ(pipeline.featureInput().contacts,input.contacts);
  const auto reconstructed=pipeline.core().evaluate(0,pipeline.state(),result.featureInput);
  EXPECT_EQ(reconstructed.externalWrenchEstimate,pipeline.evaluation().externalWrenchEstimate);
  EXPECT_EQ(reconstructed.robustWrench,pipeline.evaluation().robustWrench);
}

TEST(MyRbfV2ExternalPipeline,
     V4CalibratesThePhysicalResidualAndConvergesWithoutUnforcedAxisDrift) {
  v2::Settings cfg;
  cfg.updateLaw = v2::UpdateLaw::ResidualConvergence;
  cfg.featureMode = v2::FeatureMode::ResidualOnly;
  cfg.gamma.fill(10.0);
  cfg.convergenceLeakage.fill(0.02);
  cfg.residualBias.fill(1.0);
  cfg.residualDeadzone.fill(0.5);
  cfg.weightLimit.fill(10.0);
  MyRbfV2Pipeline pipeline(10, 12, false, 0.0, 1e-6, cfg);
  v2::FeatureInput input;
  input.chi.fill(0.0);

  vector6_t external = vector6_t::Constant(1.25);
  external(1) = 5.5;
  MyRbfV2PipelineResult result;
  constexpr double dt = 0.001;
  for (unsigned step = 0; step < 1500; ++step) {
    result = pipeline.advanceExternalResidual(step * dt, dt, external, input);
  }

  ASSERT_TRUE(result.valid);
  EXPECT_DOUBLE_EQ(result.residual.rawResidual(0), 0.0);
  EXPECT_DOUBLE_EQ(result.residual.rawResidual(1), 4.0);
  EXPECT_DOUBLE_EQ(result.featureInput.causalResidual[0], 0.0);
  EXPECT_DOUBLE_EQ(result.featureInput.causalResidual[1], 4.0);
  EXPECT_NEAR(result.externalWrenchEstimate(1), 4.0, 0.08);
  EXPECT_NEAR(result.requiredControlCorrection(1), -4.0, 0.08);
  for (unsigned axis : {0u, 2u, 3u, 4u, 5u}) {
    EXPECT_NEAR(result.externalWrenchEstimate(axis), 0.0, 1e-12);
    EXPECT_NEAR(result.requiredControlCorrection(axis), 0.0, 1e-12);
    for (double weight : pipeline.state().weights[axis]) {
      EXPECT_NEAR(weight, 0.0, 1e-12);
    }
  }
}

TEST(MyRbfV2ExternalPipeline, V4DeadzoneIsContinuousAtItsNoiseBoundary) {
  v2::Settings cfg;
  cfg.updateLaw = v2::UpdateLaw::ResidualConvergence;
  cfg.featureMode = v2::FeatureMode::ResidualOnly;
  cfg.residualBias.fill(1.0);
  cfg.residualDeadzone.fill(0.5);
  MyRbfV2Pipeline pipeline(10, 12, false, 0.0, 1e-6, cfg);
  v2::FeatureInput input;
  input.chi.fill(0.0);

  vector6_t external = vector6_t::Constant(1.5);
  const auto atBoundary =
      pipeline.advanceExternalResidual(0.0, 0.001, external, input);
  ASSERT_TRUE(atBoundary.valid);
  EXPECT_DOUBLE_EQ(atBoundary.residual.rawResidual(0), 0.0);

  external.fill(1.6);
  const auto justOutside =
      pipeline.advanceExternalResidual(0.001, 0.001, external, input);
  ASSERT_TRUE(justOutside.valid);
  EXPECT_NEAR(justOutside.residual.rawResidual(0), 0.1, 1e-12);
  EXPECT_LT(std::abs(justOutside.residual.rawResidual(0)), 0.11);
}

TEST(MyRbfV2ExternalPipeline, TeacherCompositeUsesWordResidualDirectionAndConfiguredFilter) {
  v2::Settings cfg;
  cfg.updateLaw = v2::UpdateLaw::TeacherComposite;
  MyRbfV2Pipeline pipeline(10, 12, true, .75, 1e-6, cfg);
  v2::FeatureInput input;
  input.chi.fill(.1);

  const auto first = pipeline.advanceExternalResidual(
      0.0, .01, vector6_t::Constant(2.0), input);
  ASSERT_TRUE(first.valid);
  ASSERT_TRUE(first.residual.valid);
  for (unsigned axis = 0; axis < 6; ++axis) {
    EXPECT_DOUBLE_EQ(first.residual.rawResidual(axis), -2.0);
    EXPECT_DOUBLE_EQ(first.residual.filteredResidual(axis), -2.0);
    EXPECT_DOUBLE_EQ(first.featureInput.causalResidual[axis], -2.0);
  }

  auto invalid = input;
  invalid.chi[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(pipeline.advanceExternalResidual(
                   .005, .01, vector6_t::Constant(100.0), invalid),
               std::invalid_argument);

  const auto second = pipeline.advanceExternalResidual(
      .01, .01, vector6_t::Constant(6.0), input);
  ASSERT_TRUE(second.valid);
  ASSERT_TRUE(second.residual.valid);
  for (unsigned axis = 0; axis < 6; ++axis) {
    EXPECT_DOUBLE_EQ(second.residual.rawResidual(axis), -6.0);
    EXPECT_DOUBLE_EQ(second.residual.filteredResidual(axis), -3.0);
    EXPECT_DOUBLE_EQ(second.featureInput.causalResidual[axis], -3.0);
  }

  pipeline.reset();
  const auto afterReset = pipeline.advanceExternalResidual(
      0.0, .01, vector6_t::Constant(10.0), input);
  ASSERT_TRUE(afterReset.valid);
  for (unsigned axis = 0; axis < 6; ++axis) {
    EXPECT_DOUBLE_EQ(afterReset.featureInput.causalResidual[axis], -10.0);
  }
}

TEST(MyRbfV2ExternalPipeline, TeacherCompositeBuildsCausalIntegralHistoryAndConverges) {
  v2::Settings cfg;
  cfg.updateLaw = v2::UpdateLaw::TeacherComposite;
  cfg.gamma.fill(1.0);
  cfg.predictionGain.fill(0.5);
  cfg.compositeWindow = 0.1;
  cfg.maxHistorySamples = 16;
  cfg.weightLimit.fill(5.0);
  MyRbfV2Pipeline pipeline(10, 12, false, 0.0, 1e-6, cfg);
  v2::FeatureInput input;
  input.learningSignalValid = false;

  MyRbfV2PipelineResult result;
  constexpr double dt = 0.01;
  for (unsigned step = 0; step < 500; ++step) {
    result = pipeline.advanceExternalResidual(
        step * dt, dt, vector6_t::Constant(2.0), input);
  }

  ASSERT_TRUE(result.valid);
  EXPECT_GT(pipeline.historySize(), 0u);
  for (unsigned axis = 0; axis < 6; ++axis) {
    EXPECT_NEAR(result.estimateAfterUpdate(axis), -2.0, 0.08);
    EXPECT_LT(result.weightDerivativeNorm(axis), 0.08);
  }
}

TEST(MyRbfV2ExternalPipeline,
     TeacherSharedAllocatorDeliversCorrectionThroughExactlyOnePath) {
  MyRbfV2Snapshot snapshot;
  snapshot.modelWrenchEstimate = {{1.0, -2.0, 3.0, -0.4, 0.5, -0.6}};
  snapshot.requiredControlCorrection = {{-1.0, 2.0, -3.0, 0.4, -0.5, 0.6}};

  const auto shared = resolveV3ControlDelivery(snapshot, true);
  EXPECT_EQ(shared.modelWrench, v2::Axes{});
  EXPECT_EQ(shared.allocatorCorrection,
            snapshot.requiredControlCorrection);
  for (unsigned axis = 0; axis < 6; ++axis) {
    EXPECT_DOUBLE_EQ(shared.modelWrench[axis] *
                         shared.allocatorCorrection[axis],
                     0.0);
  }

  const auto planned = resolveV3ControlDelivery(snapshot, false);
  EXPECT_EQ(planned.modelWrench, snapshot.modelWrenchEstimate);
  EXPECT_EQ(planned.allocatorCorrection, v2::Axes{});
  for (unsigned axis = 0; axis < 6; ++axis) {
    EXPECT_DOUBLE_EQ(planned.modelWrench[axis] *
                         planned.allocatorCorrection[axis],
                     0.0);
  }
}

TEST(MyRbfV2ExternalPipeline, LegacyObserverSignAndDefaultNumericsRemainUnchanged) {
  v2::Settings cfg; MyRbfV2Pipeline pipeline(10,12,false,0,1e-6,cfg);
  const vector_t predicted=vector_t::Zero(12);
  vector_t actual=predicted; actual.head<6>().setConstant(.002);
  const vector6_t known=vector6_t::Constant(.1);
  pipeline.setPrediction(.01,.01,predicted,known);
  const vector6_t chi=vector6_t::Constant(.2);
  const auto result=pipeline.observeActual(.01,actual,chi);
  ASSERT_TRUE(result.valid);
  v2::Axes residual{},x{}; residual.fill(-1.9); x.fill(.2);
  v2::Core core(cfg); v2::State expected;
  const auto before=core.evaluate(0,expected,residual,x);
  for(unsigned i=0;i<6;++i) {
    EXPECT_NEAR(result.residual.filteredResidual(i),-1.9,1e-14);
    for(unsigned j=0;j<26;++j) expected.weights[i][j]=.01*before.weightDerivative[i][j];
    expected.robustBound[i]=.01*before.boundDerivative[i];
  }
  const auto after=core.evaluate(0,expected,residual,x);
  for(unsigned i=0;i<6;++i) {
    EXPECT_NEAR(result.estimateAfterUpdate(i),after.estimate[i],1e-14);
    EXPECT_NEAR(pipeline.compensationWrench()(i),-(after.estimate[i]+after.robustWrench[i]),1e-14);
  }
}

TEST(MyRbfV2ExternalPipeline, FailedUpdateIsAtomicAndResetClearsSnapshotAccess) {
  v2::Settings cfg; cfg.featureMode=v2::FeatureMode::ContactAware;
  MyRbfV2Pipeline pipeline(10,12,false,0,1e-6,cfg);
  v2::FeatureInput input; input.chi.fill(.1);
  const vector6_t residual=vector6_t::Constant(.2);
  pipeline.advanceExternalResidual(1,.01,residual,input);
  const auto state=pipeline.state(); const auto evaluation=pipeline.evaluation();
  EXPECT_THROW(pipeline.advanceExternalResidual(1,0,residual,input),std::invalid_argument);
  EXPECT_THROW(pipeline.advanceExternalResidual(.9,.01,residual,input),std::invalid_argument);
  auto invalid=input; invalid.baseVelocity[0]=std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(pipeline.advanceExternalResidual(1.01,.01,residual,invalid),std::invalid_argument);
  auto overflowing=input; overflowing.chi.fill(2);
  EXPECT_THROW(pipeline.advanceExternalResidual(1.01,std::numeric_limits<double>::max(),residual,overflowing),std::overflow_error);
  EXPECT_EQ(pipeline.state().weights,state.weights);
  EXPECT_EQ(pipeline.state().robustBound,state.robustBound);
  EXPECT_EQ(pipeline.evaluation().externalWrenchEstimate,evaluation.externalWrenchEstimate);
  EXPECT_TRUE(pipeline.advanceExternalResidual(1.01,.01,residual,input).valid);
  pipeline.reset();
  EXPECT_FALSE(pipeline.hasEvaluation());
  EXPECT_TRUE(pipeline.requiredControlCorrection().isZero());
  EXPECT_EQ(pipeline.evaluation().physicalGradient,v2::Weights{});
  EXPECT_TRUE(pipeline.advanceExternalResidual(0,.01,residual,input).valid);
}

TEST(MyRbfV2ExternalPipeline, ProjectionClipsAndThenAllowsInwardUpdate) {
  v2::Settings cfg; cfg.featureMode=v2::FeatureMode::ContactAware; cfg.weightLimit.fill(.001);
  MyRbfV2Pipeline pipeline(10,12,false,0,1e-6,cfg);
  v2::FeatureInput input; input.chi.fill(.8);
  const auto first=pipeline.advanceExternalResidual(0,.1,vector6_t::Zero(),input);
  ASSERT_TRUE(first.valid);
  for(unsigned i=0;i<6;++i) {
    EXPECT_TRUE(first.projectionActive[i]);
    EXPECT_DOUBLE_EQ(pipeline.state().weights[i][0],.001);
    for(double w:pipeline.state().weights[i]) EXPECT_LE(std::abs(w),.001);
  }
  input.chi.fill(-.1);
  pipeline.advanceExternalResidual(.001,.001,vector6_t::Zero(),input);
  for(unsigned i=0;i<6;++i) EXPECT_LT(pipeline.state().weights[i][0],.001);
}

struct Simulation { vector6_t x=vector6_t::Zero(); v2::State state; };
double energy(const vector6_t& x,const v2::State& state,const v2::Settings& cfg,
              const v2::Weights& ideal,double remainderBound) {
  double value=.5*x.squaredNorm();
  for(unsigned i=0;i<6;++i) {
    const double r=state.robustBound[i]-remainderBound;
    value+=.5*r*r/cfg.robustGamma[i];
    for(unsigned j=0;j<26;++j) {
      const double w=state.weights[i][j]-ideal[i][j];
      value+=.5*w*w/cfg.gamma[i];
    }
  }
  return value;
}

Simulation simulate(double dt,v2::RobustMode mode) {
  v2::Settings cfg; cfg.robustMode=mode; cfg.featureMode=v2::FeatureMode::ContactAware;
  cfg.gamma.fill(.4); cfg.robustGamma.fill(.3); cfg.weightLimit.fill(4);
  MyRbfV2Pipeline pipeline(10,12,false,0,1e-6,cfg);
  v2::Weights ideal{};
  // A representable constant external load plus a bounded unmatched remainder.
  for(unsigned i=0;i<6;++i) ideal[i][0]=.15+.025*i;
  Simulation model; model.x.setConstant(.25);
  constexpr double remainderBound=.04, damping=2;
  const int steps=static_cast<int>(2/dt+.5);
  for(int k=0;k<steps;++k) {
    const double time=k*dt;
    v2::FeatureInput input;
    vector6_t external;
    for(unsigned i=0;i<6;++i) {
      input.chi[i]=model.x(i); input.baseVelocity[i]=model.x(i);
      input.referenceVelocity[i]=.1*std::sin(time);
      external(i)=ideal[i][0]+remainderBound*std::sin(1.3*time+.2*i);
      input.causalResidual[i]=external(i);
    }
    input.contacts=time<1 ? std::array<bool,4>{{true,true,true,true}}
                          : std::array<bool,4>{{true,false,false,true}};
    const double beforeEnergy=energy(model.x,pipeline.state(),cfg,ideal,remainderBound);
    const auto result=pipeline.advanceExternalResidual(time,dt,external,input);
    const vector6_t xdot=-damping*model.x+external+result.requiredControlCorrection;
    double bound=-damping*model.x.squaredNorm(), quadratic=xdot.squaredNorm();
    for(unsigned i=0;i<6;++i) {
      bound+=remainderBound*result.evaluationBeforeUpdate.smoothingWidth[i];
      const double rdot=result.evaluationBeforeUpdate.boundDerivative[i];
      quadratic+=rdot*rdot/cfg.robustGamma[i];
      for(double wdot:result.evaluationBeforeUpdate.weightDerivative[i]) quadratic+=wdot*wdot/cfg.gamma[i];
    }
    model.x+=dt*xdot;
    const double afterEnergy=energy(model.x,pipeline.state(),cfg,ideal,remainderBound);
    // Applying the post-update correction adds nonpositive adaptation terms;
    // the explicit dt^2 remainder is retained, not called continuous stability.
    EXPECT_LE(afterEnergy-beforeEnergy,dt*bound+.5*dt*dt*quadratic+2e-12);
  }
  model.state=pipeline.state();
  return model;
}
double distance(const Simulation& a,const Simulation& b) {
  double result=(a.x-b.x).squaredNorm();
  for(unsigned i=0;i<6;++i) {
    result+=std::pow(a.state.robustBound[i]-b.state.robustBound[i],2);
    for(unsigned j=0;j<26;++j) result+=std::pow(a.state.weights[i][j]-b.state.weights[i][j],2);
  }
  return std::sqrt(result);
}
TEST(MyRbfV2ExternalPipeline, ActualEulerClosedLoopEnergyAndStepRefinement) {
  for(auto mode:{v2::RobustMode::Sign,v2::RobustMode::Smooth}) {
    const auto coarse=simulate(.01,mode), medium=simulate(.005,mode), fine=simulate(.0025,mode);
    const auto reference=simulate(.000625,mode);
    EXPECT_LT(distance(medium,reference),distance(coarse,reference));
    EXPECT_LT(distance(fine,reference),distance(medium,reference));
    EXPECT_LT(fine.x.norm(),.08);
  }
}

}}}  // namespace legged::adaptive
