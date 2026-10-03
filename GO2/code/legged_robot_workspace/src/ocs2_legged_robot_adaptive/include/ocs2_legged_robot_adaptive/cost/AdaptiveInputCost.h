#pragma once

#include <ocs2_core/cost/QuadraticStateInputCost.h>

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_legged_robot/common/utils.h>
#include <ocs2_legged_robot/reference_manager/SwitchedModelReferenceManager.h>

#include "ocs2_legged_robot_adaptive/adaptive/AdaptiveDisturbanceEstimator.h"

namespace ocs2 {
namespace legged_robot_adaptive {

class AdaptiveInputBiasCost final : public QuadraticStateInputCost {
 public:
  AdaptiveInputBiasCost(matrix_t R, CentroidalModelInfo info, const legged_robot::SwitchedModelReferenceManager& referenceManager,
                        const AdaptiveDisturbanceEstimator& estimator)
      : QuadraticStateInputCost(matrix_t::Zero(info.stateDim, info.stateDim), std::move(R)),
        info_(std::move(info)),
        referenceManagerPtr_(&referenceManager),
        estimatorPtr_(&estimator) {}

  ~AdaptiveInputBiasCost() override = default;
  AdaptiveInputBiasCost* clone() const override { return new AdaptiveInputBiasCost(*this); }

 private:
  AdaptiveInputBiasCost(const AdaptiveInputBiasCost& rhs) = default;

  std::pair<vector_t, vector_t> getStateInputDeviation(scalar_t time, const vector_t& state, const vector_t& input,
                                                       const TargetTrajectories& targetTrajectories) const override {
    const auto contactFlags = referenceManagerPtr_->getContactFlags(time);
    vector_t nominalInput = legged_robot::weightCompensatingInput(info_, contactFlags);

    const size_t numStanceLegs = legged_robot::numberOfClosedContacts(contactFlags);
    if (numStanceLegs > 0) {
      const auto estimate = estimatorPtr_->getActiveEstimate();
      const legged_robot::vector3_t compensationPerFoot = -estimate.externalForceInWorld / static_cast<scalar_t>(numStanceLegs);
      for (size_t i = 0; i < contactFlags.size(); i++) {
        if (contactFlags[i]) {
          centroidal_model::getContactForces(nominalInput, i, info_) += compensationPerFoot;
        }
      }
    }

    return {vector_t::Zero(state.size()), input - nominalInput};
  }

  const CentroidalModelInfo info_;
  const legged_robot::SwitchedModelReferenceManager* referenceManagerPtr_;
  const AdaptiveDisturbanceEstimator* estimatorPtr_;
};

}  // namespace legged_robot_adaptive
}  // namespace ocs2
