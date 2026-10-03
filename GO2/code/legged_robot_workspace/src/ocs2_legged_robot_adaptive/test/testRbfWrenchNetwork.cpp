#include <gtest/gtest.h>
#include <limits>
#include "ocs2_legged_robot_adaptive/adaptive/RbfWrenchNetwork.h"
namespace ocs2 { namespace legged_robot_adaptive { namespace adaptive { namespace {
RbfWrenchNetwork make(scalar_t beta=.7){RbfWrenchSettings s;s.residualScale<<20,20,50,5,5,5;s.sigmaScale.setOnes();s.learningGain.setOnes();s.beta=beta;matrix_t c(2,6);c.setZero();c(1,0)=1;vector_t b(2);b.setOnes();return RbfWrenchNetwork(s,c,b);}
TEST(Rbf,Basis){auto n=make();vector6_t z=vector6_t::Zero();auto p=n.computeBasis(z);ASSERT_EQ(p.size(),3);EXPECT_DOUBLE_EQ(p(0),1);EXPECT_DOUBLE_EQ(p(1),1);EXPECT_NEAR(p(2),std::exp(-.5),1e-14);}
TEST(Rbf,IndependentOutputsAndScale){auto n=make();matrix_t w=matrix_t::Zero(3,6);w(0,0)=1; n.setWeights(w);vector6_t r=vector6_t::Zero();auto e=n.evaluate(r);EXPECT_DOUBLE_EQ(e.physicalOutput(0),20);EXPECT_TRUE(e.physicalOutput.tail<5>().isZero());w.setZero();w(0,4)=.4;n.setWeights(w);e=n.evaluate(r);EXPECT_DOUBLE_EQ(e.physicalOutput(4),2);EXPECT_TRUE(e.physicalOutput.head<4>().isZero());EXPECT_DOUBLE_EQ(e.physicalOutput(5),0);}
TEST(Rbf,PhysicalScale){auto n=make();matrix_t w=matrix_t::Zero(3,6);w.row(0)<<1,.5,-1,.2,-.4,0;n.setWeights(w);auto o=n.evaluate(vector6_t::Zero()).physicalOutput;vector6_t x;x<<20,10,-50,1,-2,0;EXPECT_TRUE(o.isApprox(x,1e-14));}
TEST(Rbf,PureResidualDirection){auto n=make(1);vector6_t r=vector6_t::Zero(),s=vector6_t::Zero();r(0)=20;auto u=n.updateWeights(r,s,.1);EXPECT_GT(u.weightDerivative(0,0),0);EXPECT_GT(n.evaluate(r).physicalOutput(0),0);EXPECT_TRUE(n.evaluate(r).physicalOutput.tail<5>().isZero());}
TEST(Rbf,TrackingAndComposite){auto n=make(0);vector6_t r=vector6_t::Zero(),s=vector6_t::Zero();s(0)=1;auto u=n.updateWeights(r,s,.1);EXPECT_GT(u.weightDerivative(0,0),0);EXPECT_TRUE(u.weightDerivative.rightCols(5).isZero());auto c=make(.7);vector6_t w=vector6_t::Zero();r(0)=20;s(0)=-.5;EXPECT_NEAR(c.computeCompositeError(r,w,s)(0),.55,1e-14);s(0)=.5;EXPECT_NEAR(c.computeCompositeError(r,w,s)(0),.85,1e-14);}
TEST(Rbf,DtAndInvalid){
 auto a=make(1),b=make(1); vector6_t r=vector6_t::Zero(),s=vector6_t::Zero(); r(0)=20;
 a.updateWeights(r,s,.1); b.updateWeights(r,s,.2); EXPECT_TRUE(b.weights().isApprox(2.*a.weights(),1e-14));
 EXPECT_THROW(a.updateWeights(r,s,0),std::invalid_argument);
 RbfWrenchSettings x; x.residualScale.setOnes(); x.residualScale(0)=0; x.sigmaScale.setOnes(); x.learningGain.setOnes();
 matrix_t c(1,6); c.setZero(); vector_t q(1); q.setOnes();
 { EXPECT_THROW(RbfWrenchNetwork(x,c,q),std::invalid_argument); }
 x.residualScale.setOnes(); q(0)=0.;
 { EXPECT_THROW(RbfWrenchNetwork(x,c,q),std::invalid_argument); }
 q(0)=std::numeric_limits<double>::quiet_NaN(); x.residualScale.setOnes();
 { EXPECT_THROW(RbfWrenchNetwork(x,c,q),std::invalid_argument); }
}
}}}}
