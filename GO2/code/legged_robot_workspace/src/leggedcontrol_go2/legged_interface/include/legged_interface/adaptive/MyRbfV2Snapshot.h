#pragma once
#include "legged_interface/adaptive/MyRbfV2Core.h"
#include "legged_interface/adaptive/MyRbfV11Core.h"
#include <ocs2_core/reference/TargetTrajectories.h>
#include <cstdint>
#include <memory>
#include <mutex>
namespace legged { namespace adaptive {
// World [force, torque ABOUT BASE]. External is RIGHT-hand force;
// paper left-hand disturbance = -externalWrenchEstimate.
struct MyRbfV2Snapshot {
  double timestamp=0, clfRhs=0;
  std::uint64_t sequenceId=0;
  std::size_t contactMode=15;
  bool valid=false;
  bool v11Enabled=false, v11PredictionValid=false;
  v11::Vector6 v11Disturbance=v11::Vector6::Zero(); // left-hand, BODY, about CoM
  v11::Vector6 v11PredictionOrigin=v11::Vector6::Zero(), v11PredictionRate=v11::Vector6::Zero();
  double v11PredictionTime=0;
  v11::Settings v11Settings;
  ocs2::TargetTrajectories v11Target;
  // Legacy V2 estimates are added to the nominal model. V3 represents the
  // paper's correction w_u in xdot=f_n-B_w*w_u and is therefore subtracted.
  bool modelUsesSubtractedCorrection=false;
  v2::Weights feature{}, weights{};
  // Quantity entering the augmented model. For V2 this equals the physical
  // external-wrench estimate; for V3 it is w_u and the model subtracts it.
  v2::Axes modelWrenchEstimate{};
  v2::Axes externalWrenchEstimate{}, leftHandDisturbanceEstimate{}, requiredControlCorrection{};
  v2::Axes robustBound{}, robustWrench{}, smoothingWidth{}, chi{}, referenceWrench{}, clfNormal{};
};

struct V3ControlDelivery {
  v2::Axes modelWrench{};
  v2::Axes allocatorCorrection{};
};

// A learned correction is delivered exactly once. The legacy V3 path places
// it in the MPC model; the fair-comparison path sends the physical correction
// through the same bounded contact-wrench allocator as comparison methods.
inline V3ControlDelivery resolveV3ControlDelivery(
    const MyRbfV2Snapshot& snapshot, bool sharedAllocator) {
  V3ControlDelivery result;
  if (sharedAllocator)
    result.allocatorCorrection = snapshot.requiredControlCorrection;
  else
    result.modelWrench = snapshot.modelWrenchEstimate;
  return result;
}
// Control thread never waits: skip publication if solver is copying.
// Only solver thread allocates or changes frozenForSolver, BETWEEN solves.
class MyRbfV2SnapshotStore {
 public:
  bool v11Mode=false; // set before model construction, immutable while running
  MyRbfV2SnapshotStore():frozenForSolver(std::make_shared<const MyRbfV2Snapshot>()){}
  bool tryPublish(const MyRbfV2Snapshot& value) {
    std::unique_lock<std::mutex> lock(mutex_,std::try_to_lock);
    if(!lock.owns_lock()) return false;
    pending_=value; return true;
  }
  void freezeForSolver() {
    MyRbfV2Snapshot value;
    {std::lock_guard<std::mutex> lock(mutex_);value=pending_;}
    frozenForSolver=std::make_shared<const MyRbfV2Snapshot>(value);
  }
  std::shared_ptr<const MyRbfV2Snapshot> frozenForSolver;
 private:
  std::mutex mutex_;
  MyRbfV2Snapshot pending_;
};
}}
