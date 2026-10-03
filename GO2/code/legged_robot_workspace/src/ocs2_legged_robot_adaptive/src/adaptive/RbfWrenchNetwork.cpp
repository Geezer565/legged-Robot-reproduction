#include "ocs2_legged_robot_adaptive/adaptive/RbfWrenchNetwork.h"
#include <cmath>
#include <stdexcept>
#include <utility>
namespace ocs2 { namespace legged_robot_adaptive { namespace adaptive {
RbfWrenchNetwork::RbfWrenchNetwork(RbfWrenchSettings settings, matrix_t centers, vector_t widths)
 : settings_(std::move(settings)), centers_(std::move(centers)), widths_(std::move(widths)) {
 validateSettings();
 if (centers_.rows() <= 0 || centers_.cols() != 6 || widths_.size() != centers_.rows() || !centers_.allFinite() || !widths_.allFinite() || (widths_.array() <= 0.).any()) throw std::invalid_argument("[RbfWrenchNetwork] centers must be Mx6 and widths positive finite M-vector.");
 weights_ = matrix_t::Zero(centers_.rows()+1, 6);
}
void RbfWrenchNetwork::validateSettings() const { if (!settings_.residualScale.allFinite() || !settings_.sigmaScale.allFinite() || !settings_.learningGain.allFinite() || (settings_.residualScale.array()<=0.).any() || (settings_.sigmaScale.array()<=0.).any() || (settings_.learningGain.array()<=0.).any() || !std::isfinite(settings_.beta) || settings_.beta<0. || settings_.beta>1.) throw std::invalid_argument("[RbfWrenchNetwork] scales/gains positive finite and beta in [0,1] required."); }
void RbfWrenchNetwork::validateVector(const vector6_t& v,const char* n) const { if(!v.allFinite()) throw std::invalid_argument(std::string("[RbfWrenchNetwork] ")+n+" must be finite."); }
vector6_t RbfWrenchNetwork::normalizeResidual(const vector6_t& r) const { validateVector(r,"residual"); return r.cwiseQuotient(settings_.residualScale); }
vector_t RbfWrenchNetwork::computeBasis(const vector6_t& chi) const { validateVector(chi,"normalized input"); vector_t p(centers_.rows()+1); p(0)=1.; for(Eigen::Index j=0;j<centers_.rows();++j) p(j+1)=std::exp(-(chi-centers_.row(j).transpose()).squaredNorm()/(2.*widths_(j)*widths_(j))); return p; }
RbfWrenchEvaluation RbfWrenchNetwork::evaluate(const vector6_t& r) const { RbfWrenchEvaluation e; e.normalizedInput=normalizeResidual(r); e.basis=computeBasis(e.normalizedInput); e.normalizedOutput=weights_.transpose()*e.basis; e.physicalOutput=settings_.residualScale.cwiseProduct(e.normalizedOutput); return e; }
vector6_t RbfWrenchNetwork::computeCompositeError(const vector6_t& r,const vector6_t& w,const vector6_t& s) const { validateVector(r,"residual");validateVector(w,"wrench");validateVector(s,"sigma"); return settings_.beta*normalizeResidual(r-w)+(1.-settings_.beta)*s.cwiseQuotient(settings_.sigmaScale); }
matrix_t RbfWrenchNetwork::computeWeightDerivative(const vector_t& p,const vector6_t& e) const { if(p.size()!=weights_.rows()||!p.allFinite()) throw std::invalid_argument("[RbfWrenchNetwork] invalid basis.");validateVector(e,"composite error"); return p*(settings_.learningGain.array()*e.array()).matrix().transpose(); }
RbfWrenchUpdateResult RbfWrenchNetwork::updateWeights(const vector6_t&r,const vector6_t&s,scalar_t dt){if(!std::isfinite(dt)||dt<=0.)throw std::invalid_argument("[RbfWrenchNetwork] dt must be positive finite.");RbfWrenchUpdateResult q;q.evaluation=evaluate(r);q.normalizedResidualError=normalizeResidual(r-q.evaluation.physicalOutput);q.normalizedSigma=s.cwiseQuotient(settings_.sigmaScale);q.compositeError=computeCompositeError(r,q.evaluation.physicalOutput,s);q.weightDerivative=computeWeightDerivative(q.evaluation.basis,q.compositeError);weights_+=dt*q.weightDerivative;return q;}
void RbfWrenchNetwork::setWeights(const matrix_t&w){if(w.rows()!=weights_.rows()||w.cols()!=6||!w.allFinite())throw std::invalid_argument("[RbfWrenchNetwork] weights must be finite (M+1)x6.");weights_=w;} void RbfWrenchNetwork::resetWeights(){weights_.setZero();}
}}}
