#include "ocs2_legged_robot_adaptive/adaptive/TwoFeatureSixSubnetworkRbfWeightAdapter.h"
#include <cmath>
#include <stdexcept>
#include <utility>
namespace ocs2 { namespace legged_robot_adaptive { namespace adaptive {
TwoFeatureSixSubnetworkRbfWeightAdapter::TwoFeatureSixSubnetworkRbfWeightAdapter(TwoFeatureRbfWeightAdapterSettings settings):settings_(std::move(settings)){
  if(!settings_.gamma.allFinite() || !settings_.outputScale.allFinite() || (settings_.gamma.array()<0.).any() || (settings_.outputScale.array()<=0.).any()) throw std::invalid_argument("[TwoFeatureSixSubnetworkRbfWeightAdapter] gamma must be finite >=0 and outputScale finite >0.");
}
void TwoFeatureSixSubnetworkRbfWeightAdapter::validateMatrix(const matrix_t& v,const char* name) const {if(v.rows()!=kBasisCount||v.cols()!=kAxisCount||!v.allFinite())throw std::invalid_argument(std::string("[TwoFeatureSixSubnetworkRbfWeightAdapter] ")+name+" must be finite 26x6.");}
matrix_t TwoFeatureSixSubnetworkRbfWeightAdapter::weightDerivative(const matrix_t& basis,const vector6_t& sigma) const {validateMatrix(basis,"basis");if(!sigma.allFinite())throw std::invalid_argument("[TwoFeatureSixSubnetworkRbfWeightAdapter] sigma must be finite.");matrix_t d(kBasisCount,kAxisCount);for(Eigen::Index i=0;i<kAxisCount;++i)d.col(i)=settings_.gamma(i)*settings_.outputScale(i)*basis.col(i)*sigma(i);return d;}
matrix_t TwoFeatureSixSubnetworkRbfWeightAdapter::eulerStep(const matrix_t& weights,const matrix_t& derivative,scalar_t dt) const {validateMatrix(weights,"weights");validateMatrix(derivative,"weightDerivative");if(!std::isfinite(dt)||dt<0.)throw std::invalid_argument("[TwoFeatureSixSubnetworkRbfWeightAdapter] dt must be finite and >=0.");const matrix_t next=weights+dt*derivative;if(!next.allFinite())throw std::overflow_error("[TwoFeatureSixSubnetworkRbfWeightAdapter] Euler result is not finite.");return next;}
}}}
