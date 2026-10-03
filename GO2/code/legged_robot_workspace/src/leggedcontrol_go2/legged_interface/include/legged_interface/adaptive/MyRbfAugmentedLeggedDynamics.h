#pragma once
#include <ocs2_core/dynamics/SystemDynamicsBase.h>
#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include "legged_interface/adaptive/MyRbfV2Snapshot.h"
namespace legged { namespace adaptive {
class MyRbfAugmentedLeggedDynamics final : public ocs2::SystemDynamicsBase {
 public:
  MyRbfAugmentedLeggedDynamics(const ocs2::SystemDynamicsBase& nominal,
      const ocs2::PinocchioInterface& pinocchio, const ocs2::CentroidalModelInfo& info,
      std::shared_ptr<MyRbfV2SnapshotStore> snapshots);
  MyRbfAugmentedLeggedDynamics(const MyRbfAugmentedLeggedDynamics& other);
  MyRbfAugmentedLeggedDynamics* clone() const override;
  ocs2::vector_t computeFlowMap(ocs2::scalar_t t, const ocs2::vector_t& x,
      const ocs2::vector_t& u, const ocs2::PreComputation& pre) override;
  ocs2::VectorFunctionLinearApproximation linearApproximation(ocs2::scalar_t t,
      const ocs2::vector_t& x, const ocs2::vector_t& u, const ocs2::PreComputation& pre) override;
 private:
  ocs2::vector_t addition(const ocs2::vector_t& x, ocs2::matrix_t* derivative);
  std::unique_ptr<ocs2::SystemDynamicsBase> nominal_;
  ocs2::PinocchioInterface pinocchio_;
  ocs2::CentroidalModelInfo info_;
  std::shared_ptr<MyRbfV2SnapshotStore> snapshots_;
};
}}
