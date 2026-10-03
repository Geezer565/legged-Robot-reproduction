#pragma once

#include "ocs2_legged_robot_adaptive/adaptive/PayloadInertial10D.h"
#include "ocs2_legged_robot_adaptive/dynamics/MassMismatchGroundTruth.h"

namespace ocs2 {
namespace legged_robot_adaptive {

struct PayloadInertialGroundTruthSample {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  vector_t nominalFlow;
  vector_t trueFlow;
  vector_t fullFlowResidual;
  adaptive::vector6_t normalizedMomentumRateResidual =
      adaptive::vector6_t::Zero();
  vector_t coordinateFlowResidual;
  scalar_t normalizedMomentumRateResidualNorm = 0.0;
  scalar_t coordinateFlowResidualNorm = 0.0;
  scalar_t fullFlowResidualNorm = 0.0;
};

/**
 * D1A ground truth for a physical 10D payload. Both flows are evaluated at
 * the same OCS2 state/input, while all head/tail/full residuals are retained.
 * It intentionally does not collapse the result into a 6D equivalent wrench.
 */
class PayloadInertialGroundTruth final {
 public:
  PayloadInertialGroundTruth(
      const PinocchioInterface& pinocchioInterface,
      CentroidalModelInfo nominalInfo,
      adaptive::PayloadInertial10D::Parameters payloadParameters);

  PayloadInertialGroundTruthSample evaluate(scalar_t time,
                                            const vector_t& state,
                                            const vector_t& input);

  const CentroidalModelInfo& nominalInfo() const { return nominalInfo_; }
  const CentroidalModelInfo& trueInfo() const { return trueInfo_; }
  const adaptive::PayloadInertial10D::Parameters& payloadParameters() const {
    return payloadParameters_;
  }
  adaptive::vector10_t piStar() const {
    return adaptive::PayloadInertial10D::toVector(payloadParameters_);
  }
  const MassMismatchPlantDynamics& truePlantDynamics() const {
    return trueDynamics_;
  }

 private:
  CentroidalModelInfo nominalInfo_;
  CentroidalModelInfo trueInfo_;
  adaptive::PayloadInertial10D::Parameters payloadParameters_;
  MassMismatchPlantDynamics nominalDynamics_;
  MassMismatchPlantDynamics trueDynamics_;
};

}  // namespace legged_robot_adaptive
}  // namespace ocs2

