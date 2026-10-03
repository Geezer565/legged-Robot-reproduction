//
// Created by qiayuan on 22-12-23.
//

#include "legged_wbc/HierarchicalWbc.h"

#include "legged_wbc/HoQp.h"

namespace legged {
vector_t HierarchicalWbc::update(const vector_t& stateDesired, const vector_t& inputDesired, const vector_t& rbdStateMeasured, size_t mode,
                                 scalar_t period) {
  if (myRbfV2Input_.enabled) {
    myRbfV2Diagnostics_ = MyRbfV2WbcDiagnostics{};
    myRbfV2Diagnostics_.enabled = true;
    myRbfV2Diagnostics_.snapshotSequenceId = myRbfV2Input_.snapshotSequenceId;
    myRbfV2Diagnostics_.status = MyRbfV2WbcStatus::Unsupported;
    return {};  // This hierarchy does not preserve the new mode's hard CLF.
  }
  WbcBase::update(stateDesired, inputDesired, rbdStateMeasured, mode, period);

  Task task0 = formulateFloatingBaseEomTask() + formulateTorqueLimitsTask() + formulateFrictionConeTask() + formulateNoContactMotionTask();
  Task task1 = formulateBaseAccelTask(stateDesired, inputDesired, period) + formulateSwingLegTask();
  Task task2 = formulateContactForceTask(inputDesired);
  HoQp hoQp(task2, std::make_shared<HoQp>(task1, std::make_shared<HoQp>(task0)));

  return hoQp.getSolutions();
}

}  // namespace legged
