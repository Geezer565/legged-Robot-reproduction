#pragma once
#include <ocs2_mpc/MrtObserver.h>
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>
#include <legged_interface/adaptive/MyRbfV2Snapshot.h>
#include <legged_interface/adaptive/AdaptiveTypes.h>
#include <Eigen/Geometry>
#include <stdexcept>
#include <cmath>
#include <algorithm>

namespace legged {
// The solver thread freezes the learner before its worker threads start.
// MRT invokes both sidecar callbacks under its OWN policy-buffer mutex;
// hence a policy cannot be adopted with a snapshot from another solve.
class MyRbfV2PolicyBinding final : public ocs2::SolverSynchronizedModule, public ocs2::MrtObserver {
 public:
  explicit MyRbfV2PolicyBinding(std::shared_ptr<adaptive::MyRbfV2SnapshotStore> store):store_(std::move(store)){}
  // Lifecycle thread only, after the controller has quiesced the solver.
  void resetQuiescent() {
    buffered_.reset();active_.reset();bufferedTeacher_.reset();activeTeacher_.reset();
    solveStart_=0;solveId_=bufferedSolveId_=activeSolveId_=0;
  }
  void preSolverRun(ocs2::scalar_t initTime,ocs2::scalar_t finalTime,const ocs2::vector_t&,
      const ocs2::ReferenceManagerInterface& reference) override {
    if(!std::isfinite(initTime) || !std::isfinite(finalTime))
      throw std::invalid_argument("V2 solver times must be finite");
    store_->freezeForSolver();
    if (store_->frozenForSolver->v11Enabled) {
      // SolverBase has already accepted this solve's reference. Do not retain
      // a target copied from the previously active policy by the control thread.
      auto paired = std::make_shared<adaptive::MyRbfV2Snapshot>(*store_->frozenForSolver);
      paired->v11Target = reference.getTargetTrajectories();
      store_->frozenForSolver = std::move(paired);
    }
    solveStart_=initTime;
    ++solveId_;
  }
  void postSolverRun(const ocs2::PrimalSolution&) override {}
  void modifyBufferedSolution(const ocs2::CommandData& command,ocs2::PrimalSolution& policy) override {
    if(!std::isfinite(command.mpcInitObservation_.time) || !std::isfinite(solveStart_) ||
       std::abs(command.mpcInitObservation_.time-solveStart_)>1e-9)
      throw std::runtime_error("V2 snapshot does not match solved policy time");
    buffered_=store_->frozenForSolver;
    bufferedSolveId_=solveId_;
    auto teacher=std::make_shared<TeacherPolicy>();
    teacher->time=policy.timeTrajectory_;
    teacher->state=policy.stateTrajectory_;
    teacher->target=command.mpcTargetTrajectories_;
    bufferedTeacher_=std::move(teacher);
  }
  void modifyActiveSolution(const ocs2::CommandData&,ocs2::PrimalSolution&) override {
    active_=buffered_;
    activeTeacher_=bufferedTeacher_;
    activeSolveId_=bufferedSolveId_;
  }
  // Called only on the control thread, after updatePolicy().
  const std::shared_ptr<const adaptive::MyRbfV2Snapshot>& active() const {return active_;}
  std::uint64_t activeSolveId() const {return activeSolveId_;}
  bool sampleTeacherErrors(ocs2::scalar_t time,const ocs2::vector_t& actual,
      ocs2::vector_t& predictionError,ocs2::vector_t& trackingError) const {
    if(!activeTeacher_ || actual.size()!=kTeacherStateDim || !actual.allFinite() || !std::isfinite(time))return false;
    ocs2::vector_t predicted;
    if(!interpolatePolicy(*activeTeacher_,time,predicted) || predicted.size()!=actual.size() ||
       activeTeacher_->target.empty() || activeTeacher_->target.timeTrajectory.empty() ||
       time<activeTeacher_->target.timeTrajectory.front() ||
       time>activeTeacher_->target.timeTrajectory.back())return false;
    const auto target=activeTeacher_->target.getDesiredState(time);
    if(target.size()!=actual.size() || !target.allFinite())return false;
    return stateBoxMinus(actual,predicted,predictionError) &&
           stateBoxMinus(actual,target,trackingError);
  }
 private:
  static constexpr Eigen::Index kTeacherStateDim=24;
  struct TeacherPolicy {
    ocs2::scalar_array_t time;
    ocs2::vector_array_t state;
    ocs2::TargetTrajectories target;
  };
  static bool interpolatePolicy(const TeacherPolicy& policy,ocs2::scalar_t query,
                                ocs2::vector_t& value) {
    if(policy.time.empty() || policy.time.size()!=policy.state.size() ||
       query<policy.time.front() || query>policy.time.back())return false;
    auto upper=std::upper_bound(policy.time.begin(),policy.time.end(),query);
    if(upper==policy.time.begin())return false;
    const std::size_t lo=static_cast<std::size_t>(upper-policy.time.begin()-1);
    if((lo>0 && policy.time[lo]==policy.time[lo-1]) ||
       (lo+1<policy.time.size() && policy.time[lo]==policy.time[lo+1]))return false;
    if(upper==policy.time.end()) {
      if(query!=policy.time.back())return false;
      value=policy.state.back();
      return value.allFinite();
    }
    const std::size_t hi=lo+1;
    const double dt=policy.time[hi]-policy.time[lo];
    if(!(dt>0) || policy.state[lo].size()!=policy.state[hi].size() ||
       !policy.state[lo].allFinite() || !policy.state[hi].allFinite())return false;
    const double alpha=(query-policy.time[lo])/dt;
    value=(1.-alpha)*policy.state[lo]+alpha*policy.state[hi];
    return value.allFinite();
  }
  static Eigen::Matrix3d rotationFromZyx(const Eigen::Vector3d& zyx) {
    const Eigen::AngleAxisd yaw(zyx(0),Eigen::Vector3d::UnitZ());
    const Eigen::AngleAxisd pitch(zyx(1),Eigen::Vector3d::UnitY());
    const Eigen::AngleAxisd roll(zyx(2),Eigen::Vector3d::UnitX());
    return (yaw*pitch*roll).toRotationMatrix();
  }
  static Eigen::Vector3d rotationErrorInWorld(const Eigen::Vector3d& actualZyx,
                                               const Eigen::Vector3d& referenceZyx) {
    Eigen::Quaterniond delta(rotationFromZyx(actualZyx)*rotationFromZyx(referenceZyx).transpose());
    delta.normalize();
    if(delta.w()<0.)delta.coeffs()*=-1.;
    const double norm=delta.vec().norm();
    if(norm<1e-12)return 2.*delta.vec();
    return (2.*std::atan2(norm,delta.w())/norm)*delta.vec();
  }
  static bool stateBoxMinus(const ocs2::vector_t& actual,const ocs2::vector_t& reference,
                            ocs2::vector_t& error) {
    if(actual.size()!=kTeacherStateDim || reference.size()!=actual.size() ||
       !actual.allFinite() || !reference.allFinite())return false;
    error=actual-reference;
    // OCS2 centroidal state: [normalized momentum(6), base xyz(3),
    // base Euler ZYX(3), joint positions(12)]. The orientation error itself
    // is a world-frame rotation vector, never a wrapped Euler subtraction.
    error.segment<3>(9)=rotationErrorInWorld(actual.segment<3>(9),reference.segment<3>(9));
    return error.size()==kTeacherStateDim && error.allFinite();
  }
  std::shared_ptr<adaptive::MyRbfV2SnapshotStore> store_;
  std::shared_ptr<const adaptive::MyRbfV2Snapshot> buffered_,active_;
  std::shared_ptr<const TeacherPolicy> bufferedTeacher_,activeTeacher_;
  double solveStart_=0;
  std::uint64_t solveId_=0,bufferedSolveId_=0,activeSolveId_=0;
};
}
