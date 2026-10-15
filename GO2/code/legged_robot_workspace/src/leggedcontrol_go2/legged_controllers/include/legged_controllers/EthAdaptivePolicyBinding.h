#pragma once

#include <cmath>
#include <memory>
#include <stdexcept>

#include <ocs2_mpc/MrtObserver.h>
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>

#include <ocs2_legged_robot_adaptive/adaptive/AdaptiveModelData.h>
#include <ocs2_legged_robot_adaptive/adaptive/AdaptiveParameterUpdateModule.h>

namespace legged {

/**
 * Binds an unmodified ETH Eq.(8) snapshot to the policy generated with it.
 * The update module remains the only writer; this class only carries its
 * immutable snapshot through the MRT buffered/active-policy handoff.
 */
class EthAdaptivePolicyBinding final : public ocs2::SolverSynchronizedModule,
                                       public ocs2::MrtObserver {
 public:
  using Snapshot = ocs2::legged_robot_adaptive::adaptive::AdaptiveModelSnapshotPtr;
  using Update = ocs2::legged_robot_adaptive::adaptive::AdaptiveParameterUpdateModule;

  explicit EthAdaptivePolicyBinding(std::shared_ptr<Update> update,
                                    std::shared_ptr<const ocs2::legged_robot_adaptive::adaptive::AdaptiveModelData> data)
      : update_(std::move(update)), data_(std::move(data)) {
    if (!update_ || !data_) throw std::invalid_argument("ETH policy binding needs update module and model data");
  }

  void preSolverRun(ocs2::scalar_t initTime, ocs2::scalar_t finalTime,
                    const ocs2::vector_t& initState,
                    const ocs2::ReferenceManagerInterface& reference) override {
    update_->preSolverRun(initTime, finalTime, initState, reference);
    frozen_ = data_->snapshot();
    if (!frozen_ || !frozen_->valid() || !std::isfinite(frozen_->time()) ||
        std::abs(frozen_->time() - initTime) > 1e-9) {
      throw std::runtime_error("ETH Eq.(8) snapshot is not synchronized with solver observation");
    }
    solveStart_ = initTime;
  }
  void postSolverRun(const ocs2::PrimalSolution&) override {}
  void modifyBufferedSolution(const ocs2::CommandData& command,
                              ocs2::PrimalSolution&) override {
    if (!frozen_ || std::abs(command.mpcInitObservation_.time - solveStart_) > 1e-9)
      throw std::runtime_error("ETH frozen snapshot does not match buffered policy");
    buffered_ = frozen_;
  }
  void modifyActiveSolution(const ocs2::CommandData&, ocs2::PrimalSolution&) override {
    active_ = buffered_;
  }
  Snapshot active() const { return active_; }

 private:
  std::shared_ptr<Update> update_;
  std::shared_ptr<const ocs2::legged_robot_adaptive::adaptive::AdaptiveModelData> data_;
  Snapshot frozen_, buffered_, active_;
  ocs2::scalar_t solveStart_ = 0.0;
};

}  // namespace legged
