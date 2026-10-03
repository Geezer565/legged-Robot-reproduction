#include "ocs2_legged_robot_adaptive/adaptive/TwoFeatureSixSubnetworkRbfWrenchNetwork.h"
#include <array>
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <utility>
#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <ocs2_core/misc/LoadData.h>
namespace ocs2 { namespace legged_robot_adaptive { namespace adaptive {
TwoFeatureSixSubnetworkRbfWrenchSettings loadTwoFeatureSixSubnetworkRbfWrenchSettings(const std::string& filename,const std::string& prefix){
  boost::property_tree::ptree pt;boost::property_tree::read_info(filename,pt);
  if(pt.get<Eigen::Index>(prefix+".subnetworkCount")!=6||pt.get<Eigen::Index>(prefix+".inputDimensionPerSubnetwork")!=2||pt.get<Eigen::Index>(prefix+".gaussianNodeCount")!=25)throw std::invalid_argument("[TwoFeatureSixSubnetworkRbfWrenchNetwork] frozen basis metadata must be 6x2x25.");
  TwoFeatureSixSubnetworkRbfWrenchSettings s;s.centers=matrix_t::Zero(6,50);s.widths.resize(6,25);
  loadData::loadEigenMatrix(filename,prefix+".residualScale",s.residualScale);
  loadData::loadEigenMatrix(filename,prefix+".discrepancyScale",s.discrepancyScale);
  loadData::loadEigenMatrix(filename,prefix+".outputScale",s.outputScale);
  const std::string layout=pt.get<std::string>(prefix+".centerLayout","frozen");
  if(layout=="symmetric_grid_5x5"){
    const scalar_t linearExtent=pt.get<scalar_t>(prefix+".linearExtent");
    const scalar_t angularExtent=pt.get<scalar_t>(prefix+".angularExtent");
    const scalar_t widthRatio=pt.get<scalar_t>(prefix+".gridWidthRatio");
    if(!std::isfinite(linearExtent)||!std::isfinite(angularExtent)||!std::isfinite(widthRatio)||linearExtent<=0.||angularExtent<=0.||widthRatio<=0.)throw std::invalid_argument("[TwoFeatureSixSubnetworkRbfWrenchNetwork] symmetric grid extents and width ratio must be finite and positive.");
    for(Eigen::Index axis=0;axis<6;++axis){
      const scalar_t extent=axis<3?linearExtent:angularExtent;
      const scalar_t spacing=extent/2.;
      for(Eigen::Index row=0;row<5;++row)for(Eigen::Index column=0;column<5;++column){
        const Eigen::Index node=5*row+column;
        s.centers(axis,2*node)=-extent+spacing*column;
        s.centers(axis,2*node+1)=-extent+spacing*row;
        s.widths(axis,node)=widthRatio*spacing;
      }
    }
  }else if(layout=="frozen"){
    loadData::loadEigenMatrix(filename,prefix+".widths",s.widths);
    std::array<bool,300> seen{};std::size_t count=0;
    for(const auto& entry:pt.get_child(prefix+".centers")){int axis=-1,node=-1,feature=-1;char extra='\0';if(std::sscanf(entry.first.c_str(),"(%d,%d,%d)%c",&axis,&node,&feature,&extra)!=3||axis<0||axis>=6||node<0||node>=25||feature<0||feature>=2)throw std::invalid_argument("[TwoFeatureSixSubnetworkRbfWrenchNetwork] invalid center index.");const std::size_t flat=static_cast<std::size_t>((axis*25+node)*2+feature);if(seen[flat])throw std::invalid_argument("[TwoFeatureSixSubnetworkRbfWrenchNetwork] duplicate center index.");seen[flat]=true;++count;s.centers(axis,2*node+feature)=entry.second.get_value<scalar_t>();}
    if(count!=300)throw std::invalid_argument("[TwoFeatureSixSubnetworkRbfWrenchNetwork] frozen basis must contain exactly 300 centers.");
  }else{
    throw std::invalid_argument("[TwoFeatureSixSubnetworkRbfWrenchNetwork] unsupported center layout: "+layout);
  }
  static_cast<void>(TwoFeatureSixSubnetworkRbfWrenchNetwork(s));return s;
}
TwoFeatureSixSubnetworkRbfWrenchNetwork::TwoFeatureSixSubnetworkRbfWrenchNetwork(TwoFeatureSixSubnetworkRbfWrenchSettings settings) : settings_(std::move(settings)), weights_(matrix_t::Zero(kBasisCount,kAxisCount)) { validateSettings(); }
void TwoFeatureSixSubnetworkRbfWrenchNetwork::validateSettings() const {
  if (!settings_.residualScale.allFinite() || !settings_.discrepancyScale.allFinite() || !settings_.outputScale.allFinite() || (settings_.residualScale.array()<=0.).any() || (settings_.discrepancyScale.array()<=0.).any() || (settings_.outputScale.array()<=0.).any() || settings_.centers.rows()!=6 || settings_.centers.cols()!=50 || settings_.widths.rows()!=6 || settings_.widths.cols()!=25 || !settings_.centers.allFinite() || !settings_.widths.allFinite() || (settings_.widths.array()<=0.).any()) throw std::invalid_argument("[TwoFeatureSixSubnetworkRbfWrenchNetwork] invalid finite scales, 6x50 centers, or positive 6x25 widths.");
}
matrix_t TwoFeatureSixSubnetworkRbfWrenchNetwork::computeBasis(const matrix_t& feature) const {
  if (feature.rows()!=2 || feature.cols()!=6 || !feature.allFinite()) throw std::invalid_argument("[TwoFeatureSixSubnetworkRbfWrenchNetwork] feature must be finite 2x6 and already normalized.");
  matrix_t basis(kBasisCount,kAxisCount); basis.row(0).setOnes();
  for(Eigen::Index axis=0;axis<6;++axis) for(Eigen::Index node=0;node<25;++node) { const scalar_t dx=feature(0,axis)-settings_.centers(axis,2*node); const scalar_t dy=feature(1,axis)-settings_.centers(axis,2*node+1); const scalar_t b=settings_.widths(axis,node); basis(node+1,axis)=std::exp(-(dx*dx+dy*dy)/(2.*b*b)); }
  return basis;
}
vector6_t TwoFeatureSixSubnetworkRbfWrenchNetwork::evaluateNormalized(const matrix_t& feature) const { const matrix_t basis=computeBasis(feature); vector6_t output; for(Eigen::Index i=0;i<6;++i) output(i)=weights_.col(i).dot(basis.col(i)); return output; }
TwoFeatureSixSubnetworkRbfWrenchEvaluation TwoFeatureSixSubnetworkRbfWrenchNetwork::evaluate(const matrix_t& feature) const { TwoFeatureSixSubnetworkRbfWrenchEvaluation r; r.basis=computeBasis(feature); for(Eigen::Index i=0;i<6;++i) r.normalizedOutput(i)=weights_.col(i).dot(r.basis.col(i)); r.physicalOutput=settings_.outputScale.cwiseProduct(r.normalizedOutput); return r; }
void TwoFeatureSixSubnetworkRbfWrenchNetwork::setWeights(const matrix_t& weights) { if(weights.rows()!=26||weights.cols()!=6||!weights.allFinite()) throw std::invalid_argument("[TwoFeatureSixSubnetworkRbfWrenchNetwork] weights must be finite 26x6."); weights_=weights; }
void TwoFeatureSixSubnetworkRbfWrenchNetwork::resetWeights(){weights_.setZero();}
}}}
