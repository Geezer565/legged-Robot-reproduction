#include "legged_interface/adaptive/MyRbfPredictionResidualObserver.h"

#include <cmath>
#include <stdexcept>

namespace legged {
namespace adaptive {

MyRbfPredictionResidualObserver::MyRbfPredictionResidualObserver(
    scalar_t robotMass, Eigen::Index stateDimension, bool enableFilter,
    scalar_t filterAlpha, scalar_t timeTolerance)
    : robotMass_(robotMass),
      stateDimension_(stateDimension),
      enableFilter_(enableFilter),
      filterAlpha_(filterAlpha),
      timeTolerance_(timeTolerance) {
  if (!std::isfinite(robotMass_) || robotMass_ <= 0.0) {
    throw std::invalid_argument(
        "[MyRbfPredictionResidualObserver] robotMass must be positive and finite.");
  }
  if (stateDimension_ < 6) {
    throw std::invalid_argument(
        "[MyRbfPredictionResidualObserver] stateDimension must be at least six.");
  }
  if (!std::isfinite(filterAlpha_) || filterAlpha_ < 0.0 ||
      filterAlpha_ >= 1.0) {
    throw std::invalid_argument(
        "[MyRbfPredictionResidualObserver] filterAlpha must satisfy 0 <= alpha < 1.");
  }
  if (!std::isfinite(timeTolerance_) || timeTolerance_ < 0.0) {
    throw std::invalid_argument(
        "[MyRbfPredictionResidualObserver] timeTolerance must be finite and nonnegative.");
  }
}

bool MyRbfPredictionResidualObserver::validState(
    const vector_t& state) const {
  return state.size() == stateDimension_ && state.allFinite();
}

void MyRbfPredictionResidualObserver::setPrediction(
    scalar_t predictedTime, scalar_t intervalDt,
    const vector_t& predictedState,
    const vector6_t& knownAppliedCompensation) {
  if (!std::isfinite(predictedTime) || !std::isfinite(intervalDt) ||
      intervalDt <= 0.0 || !validState(predictedState) ||
      !knownAppliedCompensation.allFinite()) {
    throw std::invalid_argument(
        "[MyRbfPredictionResidualObserver] prediction must have finite time, positive dt, and a valid state.");
  }
  predictedTime_ = predictedTime;
  intervalDt_ = intervalDt;
  predictedState_ = predictedState;
  knownAppliedCompensation_ = knownAppliedCompensation;
  hasPrediction_ = true;
}

MyRbfPredictionResidualResult
MyRbfPredictionResidualObserver::observe(
    scalar_t actualTime, const vector_t& actualState) {
  MyRbfPredictionResidualResult result;
  if (!hasPrediction_ || !std::isfinite(actualTime) ||
      !validState(actualState)) {
    return result;
  }

  hasPrediction_ = false;
  if (std::abs(actualTime - predictedTime_) > timeTolerance_) {
    return result;
  }

  result.dt = intervalDt_;
  result.netResidual =
      -robotMass_ *
      (actualState.head<6>() - predictedState_.head<6>()) /
      intervalDt_;
  result.rawResidual = result.netResidual + knownAppliedCompensation_;
  if (!result.rawResidual.allFinite()) {
    return result;
  }

  if (!enableFilter_ || !filterInitialized_) {
    filteredResidual_ = result.rawResidual;
    filterInitialized_ = true;
  } else {
    filteredResidual_ = filterAlpha_ * filteredResidual_ +
                        (1.0 - filterAlpha_) * result.rawResidual;
  }
  result.filteredResidual = filteredResidual_;
  result.valid = true;
  return result;
}

void MyRbfPredictionResidualObserver::reset() {
  hasPrediction_ = false;
  predictedTime_ = 0.0;
  intervalDt_ = 0.0;
  predictedState_.resize(0);
  knownAppliedCompensation_.setZero();
  filterInitialized_ = false;
  filteredResidual_.setZero();
}

}  // namespace adaptive
}  // namespace legged
