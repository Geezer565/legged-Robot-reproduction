#include <gtest/gtest.h>
#include <limits>
#include "ocs2_legged_robot_adaptive/adaptive/TwoFeatureRbfWrenchInput.h"
namespace ocs2 { namespace legged_robot_adaptive { namespace adaptive { namespace {
vector6_t scale(){ vector6_t s; s<<18.40147689318011,18.757520233566048,17.534482646627026,2.3370077074651903,5.466041588930613,1.7261893548458138; return s; }
TEST(TwoFeatureInput, Zero){const auto z=TwoFeatureRbfWrenchInput(scale(),scale()).compute(vector6_t::Zero(),vector6_t::Zero()).feature; EXPECT_TRUE(z.isZero());}
TEST(TwoFeatureInput, FiveWithoutEstimate){vector6_t r=vector6_t::Zero();r(0)=5.;const auto z=TwoFeatureRbfWrenchInput(scale(),scale()).compute(r,vector6_t::Zero()).feature;EXPECT_NEAR(z(0,0),5./scale()(0),1e-14);EXPECT_NEAR(z(1,0),5./scale()(0),1e-14);}
TEST(TwoFeatureInput, TwentyWithFifteenEstimate){vector6_t r=vector6_t::Zero(),p=vector6_t::Zero();r(0)=20.;p(0)=15.;const auto z=TwoFeatureRbfWrenchInput(scale(),scale()).compute(r,p).feature;EXPECT_NEAR(z(0,0),20./scale()(0),1e-14);EXPECT_NEAR(z(1,0),5./scale()(0),1e-14);}
TEST(TwoFeatureInput, SameResidualDifferentProgress){vector6_t r=vector6_t::Zero(),p=vector6_t::Zero();r(0)=20.;const auto a=TwoFeatureRbfWrenchInput(scale(),scale()).compute(r,p).feature;p(0)=20.;const auto b=TwoFeatureRbfWrenchInput(scale(),scale()).compute(r,p).feature;EXPECT_DOUBLE_EQ(a(0,0),b(0,0));EXPECT_GT(a(1,0),0.);EXPECT_DOUBLE_EQ(b(1,0),0.);}
TEST(TwoFeatureInput, SixAxisIndependent){vector6_t r,p;r<<1,2,3,4,5,6;p<<.1,.2,.3,.4,.5,.6;const auto z=TwoFeatureRbfWrenchInput(scale(),scale()).compute(r,p).feature;for(Eigen::Index i=0;i<6;++i){EXPECT_NEAR(z(0,i),r(i)/scale()(i),1e-14);EXPECT_NEAR(z(1,i),(r(i)-p(i))/scale()(i),1e-14);}}
TEST(TwoFeatureInput, RejectsZeroScale){vector6_t s=scale();s(0)=0.;EXPECT_THROW(static_cast<void>(TwoFeatureRbfWrenchInput(s,scale())),std::invalid_argument);}
TEST(TwoFeatureInput, RejectsNanScale){vector6_t s=scale();s(0)=std::numeric_limits<scalar_t>::quiet_NaN();EXPECT_THROW(static_cast<void>(TwoFeatureRbfWrenchInput(s,scale())),std::invalid_argument);}
TEST(TwoFeatureInput, RejectsInfiniteInput){vector6_t v=vector6_t::Zero();v(0)=std::numeric_limits<scalar_t>::infinity();EXPECT_THROW(TwoFeatureRbfWrenchInput(scale(),scale()).compute(v,vector6_t::Zero()),std::invalid_argument);}
}}}}
