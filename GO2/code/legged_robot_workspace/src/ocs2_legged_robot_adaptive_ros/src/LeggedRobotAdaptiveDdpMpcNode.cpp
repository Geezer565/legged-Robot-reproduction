#include <ros/init.h>
#include <ros/package.h>
#include <algorithm>
#include <cmath>
#include <mutex>
#include <limits>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/WrenchStamped.h>
#include <vector>

#include <ocs2_msgs/mpc_observation.h>

#include <ocs2_ddp/GaussNewtonDDP_MPC.h>
#include <ocs2_ros_interfaces/mpc/MPC_ROS_Interface.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>

#include <ocs2_legged_robot_ros/gait/GaitReceiver.h>

#include <ocs2_legged_robot_adaptive/AdaptiveLeggedRobotInterface.h>
#include <ocs2_legged_robot_adaptive/adaptive/PayloadInertial10D.h>
#include <ocs2_legged_robot_adaptive/adaptive/WrenchResidualEstimator.h>
#include <ocs2_legged_robot_adaptive/dynamics/PayloadInertialRuntimeDiagnostics.h>

using namespace ocs2;
using namespace legged_robot;
using namespace legged_robot_adaptive;

int main(int argc, char** argv) {
  const std::string robotName = "legged_robot";

  ::ros::init(argc, argv, robotName + "_adaptive_mpc");
  ::ros::NodeHandle nodeHandle;

  std::string taskFile, urdfFile, referenceFile, adaptiveSettingsFile;
  nodeHandle.getParam("/taskFile", taskFile);
  nodeHandle.getParam("/referenceFile", referenceFile);
  nodeHandle.getParam("/urdfFile", urdfFile);
  nodeHandle.param<std::string>("/adaptiveSettingsFile", adaptiveSettingsFile,
                                std::string());

  AdaptiveLeggedRobotInterface interface(taskFile, urdfFile, referenceFile,
                                         false, adaptiveSettingsFile);
  const auto adaptiveSettings = interface.getAdaptiveClfSettings();
  const scalar_t robotMass = interface.getCentroidalModelInfo().robotMass;

  auto gaitReceiverPtr =
      std::make_shared<GaitReceiver>(nodeHandle, interface.getSwitchedModelReferenceManagerPtr()->getGaitSchedule(), robotName);

  auto rosReferenceManagerPtr = std::make_shared<RosReferenceManager>(robotName, interface.getReferenceManagerPtr());
  rosReferenceManagerPtr->subscribe(nodeHandle);

  GaussNewtonDDP_MPC mpc(interface.mpcSettings(), interface.ddpSettings(), interface.getRollout(), interface.getOptimalControlProblem(),
                         interface.getInitializer());
  mpc.getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);
  mpc.getSolverPtr()->addSynchronizedModule(gaitReceiverPtr);
  const auto adaptiveUpdateModule =
      interface.getAdaptiveParameterUpdateModulePtr();
  const auto rbfWrenchClosedLoopUpdateModule =
      interface.getRbfWrenchClosedLoopUpdateModulePtr();
  if (rbfWrenchClosedLoopUpdateModule) {
    // RBF mode is an explicit replacement for the historical Eq.(8) writer.
    // Register exactly one snapshot writer so the solver never observes two
    // competing estimates in the same cycle.
    mpc.getSolverPtr()->addSynchronizedModule(
        rbfWrenchClosedLoopUpdateModule);
  } else if (adaptiveUpdateModule) {
    mpc.getSolverPtr()->addSynchronizedModule(adaptiveUpdateModule);
  }

  ros::Timer diagnosticTimer;
  // executed monitor 的 subscriber/timer 必须存活至 spin 结束；不能作为 if block
  // 内局部对象析构，否则 topic callback 根本不会持续工作。
  std::vector<ros::Subscriber> retainedSubscribers;
  std::vector<ros::Timer> retainedTimers;
  bool enableExperimentDiagnostics = true;
  nodeHandle.param("enableExperimentDiagnostics",
                   enableExperimentDiagnostics, true);
  if (auto clfConstraint = interface.getAdaptiveClfConstraintPtr()) {
    // Eq.(11) diagnostics 只在 ROS 外层 10 Hz 发布；不能从 constraint evaluate() 发布，
    // 因为 DDP 和中心差分会在单个求解周期内重复调用该函数。
    const auto sigmaLinearPublisher = nodeHandle.advertise<std_msgs::Float64MultiArray>("/aclf/sigma_linear", 1);
    const auto sigmaAngularPublisher = nodeHandle.advertise<std_msgs::Float64MultiArray>("/aclf/sigma_angular", 1);
    const auto sigmaNormPublisher = nodeHandle.advertise<std_msgs::Float64>("/aclf/sigma_norm", 1);
    const auto piHatPublisher = nodeHandle.advertise<std_msgs::Float64MultiArray>("/aclf/pi_hat", 1);
    const auto hClfPublisher = nodeHandle.advertise<std_msgs::Float64>("/aclf/h_clf", 1);
    const auto violationPublisher = nodeHandle.advertise<std_msgs::Float64>("/aclf/violation", 1);
    const auto dampingPublisher = nodeHandle.advertise<std_msgs::Float64>("/aclf/damping_term", 1);
    const auto contactPublisher = nodeHandle.advertise<std_msgs::Float64MultiArray>("/aclf/contact_wrench", 1);
    const auto nominalPublisher = nodeHandle.advertise<std_msgs::Float64MultiArray>("/aclf/nominal_wrench", 1);
    const auto unknownPublisher = nodeHandle.advertise<std_msgs::Float64MultiArray>("/aclf/estimated_unknown_wrench", 1);
    // Eq.(11) 括号内的 residual 与每一轴 sigma_i*residual_i。两者皆为 world 系：
    // 前三维为 force，后三维为关于系统 COM 的 torque。
    const auto residualPublisher = nodeHandle.advertise<std_msgs::Float64MultiArray>("/aclf/residual_wrench", 1);
    const auto contributionPublisher = nodeHandle.advertise<std_msgs::Float64MultiArray>("/aclf/contribution", 1);
    const auto linearTermPublisher = nodeHandle.advertise<std_msgs::Float64>("/aclf/linear_term", 1);
    const auto reconstructedHPublisher = nodeHandle.advertise<std_msgs::Float64>("/aclf/reconstructed_h", 1);
    const auto publishVector = [](const auto& publisher, const auto& value) { std_msgs::Float64MultiArray message; message.data.assign(value.data(), value.data()+value.size()); publisher.publish(message); };
    diagnosticTimer = nodeHandle.createTimer(ros::Duration(0.1), [clfConstraint, sigmaLinearPublisher, sigmaAngularPublisher, sigmaNormPublisher, piHatPublisher, hClfPublisher, violationPublisher, dampingPublisher, contactPublisher, nominalPublisher, unknownPublisher, residualPublisher, contributionPublisher, linearTermPublisher, reconstructedHPublisher, publishVector](const ros::TimerEvent&) {
      // 仅 timer 在 ROS 外层低频读取 shared diagnostics。不能在 evaluate() 直接发布，
      // 因为有限差分及 DDP 内迭会重复调用 evaluate()，造成刷屏和 solver 开销。
      const auto d=clfConstraint->diagnostics(); std_msgs::Float64MultiArray linear, angular; linear.data.assign(d.sigma.data(),d.sigma.data()+3); angular.data.assign(d.sigma.data()+3,d.sigma.data()+6);
      sigmaLinearPublisher.publish(linear); sigmaAngularPublisher.publish(angular); std_msgs::Float64 scalar; scalar.data=d.sigma.norm(); sigmaNormPublisher.publish(scalar); publishVector(piHatPublisher,d.piHat); scalar.data=d.hClf; hClfPublisher.publish(scalar); scalar.data=std::max<scalar_t>(0.,-d.hClf); violationPublisher.publish(scalar); scalar.data=d.dampingTerm; dampingPublisher.publish(scalar); publishVector(contactPublisher,d.contactWrench); publishVector(nominalPublisher,d.nominalReferenceWrench); publishVector(unknownPublisher,d.estimatedUnknownWrench); publishVector(residualPublisher,d.residualWrench); publishVector(contributionPublisher,d.clfContribution); scalar.data=d.clfLinearTerm; linearTermPublisher.publish(scalar); scalar.data=d.reconstructedH; reconstructedHPublisher.publish(scalar);
    });
  }

  // ============================================================
  // executed Eq.(11) diagnostics
  //
  // 此 monitor 读取 animal_dummy 发布的真实 SystemObservation：state/input/mode。
  // 它不进入 OptimalControlProblem，不写 constraint shared diagnostics，也绝不影响
  // MPC policy。这样 CLF OFF 时仍可评价同一 hypothetical Eq.(11)。
  //
  // snapshot 由 Eq.(8) synchronized module 在相同 observation timestamp 发布。
  // ROS 回调次序不保证，故暂存 observation，直到 snapshot.time 与 observation.time
  // 在 tolerance 内完全匹配才计算；不匹配时宁可不发布，也不误用前/后帧 pi_hat。
  // ============================================================
  const auto executedEvaluator = interface.getAdaptiveClfEvaluatorPtr();
  if (enableExperimentDiagnostics && executedEvaluator) {
    const auto modelData = interface.getAdaptiveModelDataPtr();
    bool enableResidualFilter = true;
    scalar_t residualFilterAlpha = 0.9;
    nodeHandle.param("enableResidualFilter", enableResidualFilter, true);
    nodeHandle.param("residualFilterAlpha", residualFilterAlpha, 0.9);
    // 该 observer 是纯 diagnostic 旁路：只用 executed observation 和完全 nominal
    // dynamics，绝不进入 MPC/OCP、adaptive estimator、CLF 或控制输入。
    const auto wrenchResidualEstimator =
        std::make_shared<adaptive::WrenchResidualEstimator>(
            interface.getPinocchioInterface(), interface.getCentroidalModelInfo(),
            enableResidualFilter, residualFilterAlpha);
    std::shared_ptr<PayloadInertialRuntimeDiagnostics>
        payloadInertialRuntimeDiagnostics;
    if (adaptiveSettings.parameterization ==
        adaptive::AdaptiveParameterization::PayloadInertial10D) {
      payloadInertialRuntimeDiagnostics =
          std::make_shared<PayloadInertialRuntimeDiagnostics>(
              interface.getPinocchioInterface(),
              interface.getCentroidalModelInfo(),
              adaptiveSettings.minimumEstimatedTotalMass);
    }
    struct PendingObservation {
      scalar_t time = 0.0;
      vector_t state;
      vector_t input;
      int mode = 0;
      bool valid = false;
    };
    auto pendingObservation = std::make_shared<PendingObservation>();
    auto pendingMutex = std::make_shared<std::mutex>();
    auto lastPublished = std::make_shared<std::pair<scalar_t, std::uint64_t>>(
        std::numeric_limits<scalar_t>::quiet_NaN(), 0);

    const auto executedH = nodeHandle.advertise<std_msgs::Float64>("/aclf_experiment/executed_h_clf", 1);
    const auto executedViolation = nodeHandle.advertise<std_msgs::Float64>("/aclf_experiment/executed_violation", 1);
    const auto executedSigmaNorm = nodeHandle.advertise<std_msgs::Float64>("/aclf_experiment/executed_sigma_norm", 1);
    const auto executedContribution = nodeHandle.advertise<std_msgs::Float64MultiArray>("/aclf_experiment/executed_contribution", 1);
    const auto executedResidual = nodeHandle.advertise<std_msgs::Float64MultiArray>("/aclf_experiment/executed_residual_wrench", 1);
    const auto executedDamping = nodeHandle.advertise<std_msgs::Float64>("/aclf_experiment/executed_damping_term", 1);
    const auto executedLinear = nodeHandle.advertise<std_msgs::Float64>("/aclf_experiment/executed_linear_term", 1);
    const auto executedObservationTime = nodeHandle.advertise<std_msgs::Float64>("/aclf_experiment/executed_observation_time", 1);
    const auto executedSnapshotTime = nodeHandle.advertise<std_msgs::Float64>("/aclf_experiment/executed_snapshot_time", 1);
    const auto executedTimeError = nodeHandle.advertise<std_msgs::Float64>("/aclf_experiment/executed_time_error", 1);
    const auto adaptiveUpdateSample = nodeHandle.advertise<std_msgs::Float64MultiArray>("/adaptive_update/sample", 10);
    const auto payloadMassSample = nodeHandle.advertise<std_msgs::Float64MultiArray>("/adaptive_update/payload_mass_sample", 10);
    const auto equation10Sample = nodeHandle.advertise<std_msgs::Float64MultiArray>("/task_b/equation10_sample", 10);
    const auto payloadInertialSample =
        nodeHandle.advertise<std_msgs::Float64MultiArray>(
            "/task_d/payload_inertial_10d_sample", 10);
    const auto payloadInertialFullFlowSample =
        nodeHandle.advertise<std_msgs::Float64MultiArray>(
            "/task_d/equation10_full_flow_sample", 10);
    const auto payloadInertialV3Sample =
        nodeHandle.advertise<std_msgs::Float64MultiArray>(
            "/task_d/payload_inertial_10d_v3_sample", 10);
    // 联合 16D 专用：一次消息同时给出 pi_in、pi_f、两个 Eq.(11) 加项和
    // 合成 wrench，便于 rqt_plot 直接核对而不依赖 D5H/D5I 旁路流程。
    const auto jointUncertaintySample =
        nodeHandle.advertise<std_msgs::Float64MultiArray>(
            "/adaptive_update/payload_inertial_plus_wrench_6d_sample", 10);
    // Diagnostics-only exact joint-uncertainty audit.  This publisher reads
    // the same immutable snapshot and Y_in already used by Eq.(8)/(11); its
    // truth values never enter the estimator, dynamics, constraint, or input.
    const auto exactJointTruthSample = nodeHandle.advertise<std_msgs::Float64MultiArray>(
        "/adaptive_diagnostics/exact_joint_uncertainty_wrench", 10);
    // rqt_plot 友好标量；全部从下方同一 synchronized 16D snapshot 的
    // AdaptiveClfDiagnostics 直接拆分，绝不另行估计或重算参数。
    const auto plotMassHat = nodeHandle.advertise<std_msgs::Float64>("/adaptive_plot/mass_hat", 10);
    const auto plotHxHat = nodeHandle.advertise<std_msgs::Float64>("/adaptive_plot/hx_hat", 10);
    const auto plotHyHat = nodeHandle.advertise<std_msgs::Float64>("/adaptive_plot/hy_hat", 10);
    const auto plotHzHat = nodeHandle.advertise<std_msgs::Float64>("/adaptive_plot/hz_hat", 10);
    const auto plotFxHat = nodeHandle.advertise<std_msgs::Float64>("/adaptive_plot/Fx_hat", 10);
    const auto plotFzHat = nodeHandle.advertise<std_msgs::Float64>("/adaptive_plot/Fz_hat", 10);
    const auto plotSigmaNorm = nodeHandle.advertise<std_msgs::Float64>("/adaptive_plot/sigma_norm", 10);
    const auto payloadInertialSigmaDecompositionSample =
        nodeHandle.advertise<std_msgs::Float64MultiArray>(
            "/task_d/payload_inertial_sigma_decomposition_sample", 10);
    const auto d5h1AppliedUpdateSample =
        nodeHandle.advertise<std_msgs::Float64MultiArray>(
            "/task_d/d5h1_applied_update_sample", 10);
    const auto d5h1ClfSample =
        nodeHandle.advertise<std_msgs::Float64MultiArray>(
            "/task_d/d5h1_eq11_sample", 10);
    const auto d5i4CompositeUpdateSample =
        nodeHandle.advertise<std_msgs::Float64MultiArray>(
            "/task_d/d5i4_composite_update_sample", 10);
    const auto d5i1ReferenceSample =
        nodeHandle.advertise<std_msgs::Float64MultiArray>(
            "/task_d/d5i1_reference_sample", 10);
    const auto wrenchResidualRawPublisher =
        nodeHandle.advertise<geometry_msgs::WrenchStamped>(
            "/rbf_diagnostics/wrench_residual_raw", 10);
    const auto wrenchResidualFilteredPublisher =
        nodeHandle.advertise<geometry_msgs::WrenchStamped>(
            "/rbf_diagnostics/wrench_residual_filtered", 10);
    const auto wrenchResidualDtPublisher =
        nodeHandle.advertise<std_msgs::Float64>("/rbf_diagnostics/residual_dt", 10);
    const auto rbfClosedLoopSample =
        nodeHandle.advertise<std_msgs::Float64MultiArray>(
            "/rbf_closed_loop/sample", 10);

    ros::Subscriber executedObservationSubscriber = nodeHandle.subscribe<ocs2_msgs::mpc_observation>(
        "/legged_robot_mpc_observation", 1,
        [pendingObservation, pendingMutex, wrenchResidualEstimator,
         rbfWrenchClosedLoopUpdateModule,
         wrenchResidualRawPublisher, wrenchResidualFilteredPublisher,
         wrenchResidualDtPublisher](const ocs2_msgs::mpc_observation::ConstPtr& message) {
          std::lock_guard<std::mutex> lock(*pendingMutex);
          // 消息中的 input 是 dummy policy rollout 在该 observation 时刻的执行 input。
          // 不在 callback 求 Eq.(11)，因为对应 snapshot 可能尚未由 preSolverRun 发布。
          pendingObservation->time = message->time;
          // ocs2_msgs 使用 float32[]，而 OCS2 scalar_t 为 double；显式逐元素转换，
          // 避免把 float buffer 错映射为 Eigen<double> 而破坏真实 observation。
          pendingObservation->state.resize(message->state.value.size());
          pendingObservation->input.resize(message->input.value.size());
          for (size_t i = 0; i < message->state.value.size(); ++i) {
            pendingObservation->state(i) = message->state.value[i];
          }
          for (size_t i = 0; i < message->input.value.size(); ++i) {
            pendingObservation->input(i) = message->input.value[i];
          }
          pendingObservation->mode = message->mode;
          pendingObservation->valid = true;

          if (rbfWrenchClosedLoopUpdateModule) {
            rbfWrenchClosedLoopUpdateModule->submitObservation(
                pendingObservation->time, pendingObservation->state,
                pendingObservation->input);
          }

          const auto residual = wrenchResidualEstimator->update(
              pendingObservation->time, pendingObservation->state,
              pendingObservation->input);
          if (!residual.valid) {
            return;
          }
          // observation.time 是 dummy 的 OCS2 simulation time；以它直接构造 stamp，
          // 保证 raw/filtered 与 observation 严格同时间基。frame_id 留空，避免在此
          // diagnostic 层未经验证地宣称坐标系名称。
          geometry_msgs::WrenchStamped rawMessage;
          geometry_msgs::WrenchStamped filteredMessage;
          rawMessage.header.stamp.fromSec(pendingObservation->time);
          filteredMessage.header.stamp = rawMessage.header.stamp;
          const auto assignWrench = [](geometry_msgs::Wrench& wrench,
                                       const adaptive::vector6_t& value) {
            wrench.force.x = value(0); wrench.force.y = value(1);
            wrench.force.z = value(2); wrench.torque.x = value(3);
            wrench.torque.y = value(4); wrench.torque.z = value(5);
          };
          assignWrench(rawMessage.wrench, residual.rawResidual);
          assignWrench(filteredMessage.wrench, residual.filteredResidual);
          wrenchResidualRawPublisher.publish(rawMessage);
          wrenchResidualFilteredPublisher.publish(filteredMessage);
          std_msgs::Float64 dtMessage;
          dtMessage.data = residual.dt;
          wrenchResidualDtPublisher.publish(dtMessage);
        });

    const scalar_t snapshotTimeTolerance = 1.0e-6;
    ros::Timer executedDiagnosticTimer = nodeHandle.createTimer(
        ros::Duration(0.02),
        [executedEvaluator, modelData, adaptiveUpdateModule,
         rbfWrenchClosedLoopUpdateModule, rbfClosedLoopSample,
         pendingObservation, pendingMutex, lastPublished, snapshotTimeTolerance,
         executedH, executedViolation, executedSigmaNorm, executedContribution, executedResidual, executedDamping,
         executedLinear, executedObservationTime, executedSnapshotTime, executedTimeError,
         adaptiveUpdateSample, payloadMassSample, equation10Sample,
         payloadInertialSample, payloadInertialFullFlowSample,
         payloadInertialV3Sample, jointUncertaintySample, exactJointTruthSample,
         payloadInertialSigmaDecompositionSample,
         plotMassHat, plotHxHat, plotHyHat, plotHzHat, plotFxHat, plotFzHat,
         plotSigmaNorm,
         d5h1AppliedUpdateSample, d5h1ClfSample,
         d5i4CompositeUpdateSample,
         d5i1ReferenceSample,
         payloadInertialRuntimeDiagnostics, adaptiveSettings,
         robotMass](const ros::TimerEvent&) {
          PendingObservation observation;
          {
            std::lock_guard<std::mutex> lock(*pendingMutex);
            if (!pendingObservation->valid) {
              return;
            }
            observation = *pendingObservation;
          }

          const auto snapshot = modelData->snapshot();
          if (!adaptive::AdaptiveClfEvaluator::isSnapshotTimeSynchronized(
                  observation.time, snapshot->time(), snapshotTimeTolerance)) {
            return;  // 保留 pending observation，等待对应 Eq.(8) snapshot；绝不使用未来 snapshot。
          }
          if (lastPublished->first == observation.time && lastPublished->second == snapshot->version()) {
            return;
          }

          // evaluator 与 StateInput constraint 共用唯一 Eq.(11) 实现。其 Pinocchio
          // 副本只在该低频 timer 使用，不会写 solver 的 latest diagnostics。
          auto d = executedEvaluator->evaluate(
              observation.time, observation.state, observation.input, snapshot);
          const auto updateDiagnostics = adaptiveUpdateModule->diagnostics();
          const bool updateDiagnosticsSynchronized = updateDiagnostics.valid &&
              std::abs(updateDiagnostics.observationTime - observation.time) <=
                  snapshotTimeTolerance &&
              std::abs(updateDiagnostics.snapshotTime - snapshot->time()) <=
                  snapshotTimeTolerance &&
              updateDiagnostics.snapshotVersion == snapshot->version();
          if (adaptiveSettings.parameterization ==
                  adaptive::AdaptiveParameterization::PayloadInertial10D &&
              (!updateDiagnosticsSynchronized ||
               updateDiagnostics.appliedParameterDerivative.size() != 10)) {
            return;  // D5H1/v3 must never publish a hypothetical derivative.
          }
          if (updateDiagnosticsSynchronized) {
            // D5I1 read-only reference transport diagnostics. Quaternion is
            // Eigen coeff order [x,y,z,w].  The topic is observational only;
            // profile identity/phase will be supplied by the future default-
            // OFF scheduler and is deliberately not invented here.
            std_msgs::Float64MultiArray referenceSample;
            referenceSample.layout.dim.resize(1);
            referenceSample.layout.dim[0].label =
                "d5i1_reference_v1_39_fields";
            referenceSample.layout.dim[0].size = 39;
            referenceSample.layout.dim[0].stride = 39;
            referenceSample.data.reserve(39);
            referenceSample.data.push_back(updateDiagnostics.observationTime);
            referenceSample.data.push_back(updateDiagnostics.snapshotTime);
            referenceSample.data.push_back(
                updateDiagnostics.snapshotTime -
                updateDiagnostics.observationTime);
            referenceSample.data.push_back(static_cast<double>(
                updateDiagnostics.snapshotVersion));
            referenceSample.data.push_back(
                updateDiagnostics.encodedReferenceDerivativesEnabled ? 1.0 : 0.0);
            const auto append3 = [&referenceSample](const auto& value) {
              referenceSample.data.insert(referenceSample.data.end(),
                                          value.data(), value.data() + 3);
            };
            append3(updateDiagnostics.desiredComPositionWorld);
            referenceSample.data.insert(
                referenceSample.data.end(),
                updateDiagnostics.desiredOrientationWorldFromBase.coeffs().data(),
                updateDiagnostics.desiredOrientationWorldFromBase.coeffs().data() + 4);
            append3(updateDiagnostics.desiredComVelocityWorld);
            append3(updateDiagnostics.desiredComAccelerationWorld);
            append3(updateDiagnostics.desiredAngularVelocityWorld);
            append3(updateDiagnostics.desiredAngularAccelerationWorld);
            append3(updateDiagnostics.referenceLinearVelocityWorld);
            append3(updateDiagnostics.referenceLinearAccelerationWorld);
            append3(updateDiagnostics.referenceAngularVelocityWorld);
            append3(updateDiagnostics.referenceAngularAccelerationWorld);
            referenceSample.data.push_back(
                static_cast<double>(updateDiagnostics.mode));
            referenceSample.data.push_back(
                updateDiagnostics.normalizedPhase);
            referenceSample.data.push_back(updateDiagnostics.cycleStart);
            d5i1ReferenceSample.publish(referenceSample);
          }
          if (adaptiveSettings.parameterization ==
              adaptive::AdaptiveParameterization::PayloadInertial10D) {
            d.parameterDerivative =
                updateDiagnostics.appliedParameterDerivative;
          }
          std_msgs::Float64 scalar;
          scalar.data = d.hClf; executedH.publish(scalar);
          scalar.data = d.violation; executedViolation.publish(scalar);
          scalar.data = d.sigma.norm(); executedSigmaNorm.publish(scalar);
          scalar.data = d.dampingTerm; executedDamping.publish(scalar);
          scalar.data = d.clfLinearTerm; executedLinear.publish(scalar);
          scalar.data = d.observationTime; executedObservationTime.publish(scalar);
          scalar.data = d.snapshotTime; executedSnapshotTime.publish(scalar);
          scalar.data = d.timeError; executedTimeError.publish(scalar);
          std_msgs::Float64MultiArray vectorMessage;
          vectorMessage.data.assign(d.clfContribution.data(), d.clfContribution.data() + 6);
          executedContribution.publish(vectorMessage);
          vectorMessage.data.assign(d.residualWrench.data(), d.residualWrench.data() + 6);
          executedResidual.publish(vectorMessage);
          // The Task A/B fixed 6D schema must never silently truncate or
          // mislabel a 10D estimate. It remains byte-for-byte unchanged for
          // the two legacy parameterizations only.
          if (adaptiveSettings.parameterization !=
              adaptive::AdaptiveParameterization::PayloadInertial10D) {
            std_msgs::Float64MultiArray updateSample;
            updateSample.data.reserve(16);
            updateSample.data.push_back(d.observationTime);
            updateSample.data.push_back(d.snapshotTime);
            updateSample.data.push_back(d.timeError);
            updateSample.data.push_back(static_cast<double>(snapshot->version()));
            updateSample.data.insert(updateSample.data.end(), d.piHat.data(), d.piHat.data() + 6);
            updateSample.data.insert(updateSample.data.end(), d.sigma.data(), d.sigma.data() + 6);
            adaptiveUpdateSample.publish(updateSample);
          }

          if (rbfWrenchClosedLoopUpdateModule) {
            const auto rbf = rbfWrenchClosedLoopUpdateModule->diagnostics();
            const bool rbfSynchronized = rbf.valid &&
                std::abs(rbf.observationTime - observation.time) <=
                    snapshotTimeTolerance &&
                std::abs(rbf.snapshotTime - snapshot->time()) <=
                    snapshotTimeTolerance &&
                rbf.snapshotVersion == snapshot->version();
            if (rbfSynchronized) {
              // 50 fields: time/version and flags [12], raw/filtered residual,
              // sigma, estimate before/after, published estimate [36], then
              // two norms [2]. All values belong to the same solver cycle.
              std_msgs::Float64MultiArray sample;
              sample.layout.dim.resize(1);
              sample.layout.dim[0].label =
                  "rbf_closed_loop_v1_50_fields";
              sample.layout.dim[0].size = 50;
              sample.layout.dim[0].stride = 50;
              sample.data.reserve(50);
              sample.data.push_back(rbf.observationTime);
              sample.data.push_back(rbf.snapshotTime);
              sample.data.push_back(rbf.snapshotTime - rbf.observationTime);
              sample.data.push_back(static_cast<double>(rbf.snapshotVersion));
              sample.data.push_back(rbf.observationMatched ? 1.0 : 0.0);
              sample.data.push_back(rbf.residualValid ? 1.0 : 0.0);
              sample.data.push_back(rbf.parameterUpdateEnabled ? 1.0 : 0.0);
              sample.data.push_back(rbf.weightsUpdated ? 1.0 : 0.0);
              sample.data.push_back(adaptiveSettings.enableAdaptiveDynamics ? 1.0 : 0.0);
              sample.data.push_back(adaptiveSettings.enableClfConstraint ? 1.0 : 0.0);
              sample.data.push_back(rbf.residualDt);
              sample.data.push_back(rbf.weightNorm);
              sample.data.insert(sample.data.end(), rbf.rawResidual.data(),
                                 rbf.rawResidual.data() + 6);
              sample.data.insert(sample.data.end(), rbf.filteredResidual.data(),
                                 rbf.filteredResidual.data() + 6);
              sample.data.insert(sample.data.end(), rbf.sigma.data(),
                                 rbf.sigma.data() + 6);
              sample.data.insert(sample.data.end(),
                                 rbf.estimateBeforeUpdate.data(),
                                 rbf.estimateBeforeUpdate.data() + 6);
              sample.data.insert(sample.data.end(),
                                 rbf.estimateAfterUpdate.data(),
                                 rbf.estimateAfterUpdate.data() + 6);
              sample.data.insert(sample.data.end(), snapshot->estimate().data(),
                                 snapshot->estimate().data() + 6);
              sample.data.push_back(rbf.weightDerivativeNorm);
              sample.data.push_back(
                  (snapshot->estimate() - rbf.estimateAfterUpdate).norm());
              rbfClosedLoopSample.publish(sample);
            }
          }

          if (adaptiveSettings.parameterization ==
              adaptive::AdaptiveParameterization::PayloadMass1D) {
            // 21 fields: time/snapshot metadata [4], nominal/delta/total/rho
            // and delta_m_dot [5], reference mass column [6], then the
            // Eq.(10)-equivalent current wrench [6]. No true plant mass is
            // exposed to the controller or estimator.
            std_msgs::Float64MultiArray massSample;
            massSample.layout.dim.resize(1);
            massSample.layout.dim[0].label =
                "payload_mass_1d_v1_21_fields";
            massSample.layout.dim[0].size = 21;
            massSample.layout.dim[0].stride = 21;
            massSample.data.reserve(21);
            massSample.data.push_back(d.observationTime);
            massSample.data.push_back(d.snapshotTime);
            massSample.data.push_back(d.timeError);
            massSample.data.push_back(static_cast<double>(snapshot->version()));
            massSample.data.push_back(robotMass);
            massSample.data.push_back(d.payloadMassEstimate);
            massSample.data.push_back(d.estimatedTotalMass);
            massSample.data.push_back(d.estimatedMassRatio);
            massSample.data.push_back(d.parameterDerivative(0));
            massSample.data.insert(
                massSample.data.end(),
                d.unknownReferenceRegressor.col(0).data(),
                d.unknownReferenceRegressor.col(0).data() + 6);
            massSample.data.insert(
                massSample.data.end(),
                d.adaptiveDynamicsEquivalentWrench.data(),
                d.adaptiveDynamicsEquivalentWrench.data() + 6);
            payloadMassSample.publish(massSample);
          }

          if (adaptiveSettings.parameterization ==
              adaptive::AdaptiveParameterization::PayloadInertial10DPlusWrench6D) {
            // 44 fields: synchronized time/version [2], pi_in [10], pi_f [6],
            // Y_in*pi_in [6], pi_f [6], combined wrench [6], sigma [6], |sigma| [1],
            // parameter dimension [1]. The snapshot is the same 16D immutable
            // object used by Eq.(8), Eq.(10), and Eq.(11) in this solver cycle.
            std_msgs::Float64MultiArray jointSample;
            jointSample.layout.dim.resize(1);
            jointSample.layout.dim[0].label =
                "payload_inertial_10d_plus_wrench_6d_v1_44_fields";
            jointSample.layout.dim[0].size = 44;
            jointSample.layout.dim[0].stride = 44;
            jointSample.data.reserve(44);
            jointSample.data.push_back(d.snapshotTime);
            jointSample.data.push_back(static_cast<double>(snapshot->version()));
            jointSample.data.insert(jointSample.data.end(),
                d.inertialParameterEstimate.data(), d.inertialParameterEstimate.data() + 10);
            jointSample.data.insert(jointSample.data.end(),
                d.piHat.data() + 10, d.piHat.data() + 16);
            jointSample.data.insert(jointSample.data.end(),
                d.inertialEstimatedWrench.data(), d.inertialEstimatedWrench.data() + 6);
            jointSample.data.insert(jointSample.data.end(),
                d.piHat.data() + 10, d.piHat.data() + 16);
            jointSample.data.insert(jointSample.data.end(),
                d.estimatedUnknownWrench.data(), d.estimatedUnknownWrench.data() + 6);
            jointSample.data.insert(jointSample.data.end(), d.sigma.data(), d.sigma.data() + 6);
            jointSample.data.push_back(d.sigma.norm());
            jointSample.data.push_back(static_cast<double>(d.piHat.size()));
            jointUncertaintySample.publish(jointSample);
            const adaptive::vector10_t piInTrue = adaptive::PayloadInertial10D::toVector(
                adaptive::PayloadInertial10D::fromPhysicalPayload(
                    5.213485, (adaptive::vector3_t() << 0.05, 0.0, 0.0).finished(),
                    adaptive::matrix3_t::Zero()));
            const adaptive::vector6_t wInTrue =
                d.unknownReferenceRegressor.leftCols(10) * piInTrue;
            adaptive::vector6_t wFTrue = adaptive::vector6_t::Zero();
            if (d.snapshotTime >= 4.0 && d.snapshotTime <= 18.0) wFTrue(0) = 30.0;
            const adaptive::vector6_t wHat = d.estimatedUnknownWrench;
            const adaptive::vector6_t wTrue = wInTrue + wFTrue;
            std_msgs::Float64MultiArray exact; exact.data.reserve(44);
            exact.data.push_back(d.snapshotTime);
            exact.data.insert(exact.data.end(), d.inertialEstimatedWrench.data(), d.inertialEstimatedWrench.data()+6);
            exact.data.insert(exact.data.end(), wInTrue.data(), wInTrue.data()+6);
            exact.data.insert(exact.data.end(), d.constantWrenchEstimate.data(), d.constantWrenchEstimate.data()+6);
            exact.data.insert(exact.data.end(), wFTrue.data(), wFTrue.data()+6);
            exact.data.insert(exact.data.end(), wHat.data(), wHat.data()+6);
            exact.data.insert(exact.data.end(), wTrue.data(), wTrue.data()+6);
            const adaptive::vector6_t epsilon = wHat-wTrue;
            exact.data.insert(exact.data.end(), epsilon.data(), epsilon.data()+6);
            exact.data.push_back(epsilon.norm());
            exactJointTruthSample.publish(exact);
            std_msgs::Float64 scalar;
            scalar.data = d.inertialParameterEstimate(0); plotMassHat.publish(scalar);
            scalar.data = d.inertialParameterEstimate(1); plotHxHat.publish(scalar);
            scalar.data = d.inertialParameterEstimate(2); plotHyHat.publish(scalar);
            scalar.data = d.inertialParameterEstimate(3); plotHzHat.publish(scalar);
            scalar.data = d.constantWrenchEstimate(0); plotFxHat.publish(scalar);
            scalar.data = d.constantWrenchEstimate(2); plotFzHat.publish(scalar);
            scalar.data = d.sigma.norm(); plotSigmaNorm.publish(scalar);
          } else if (adaptiveSettings.parameterization ==
                     adaptive::AdaptiveParameterization::ConstantWrench6D) {
            // 与联合模式相同，均取本 solver 周期的同步 diagnostics；6D 模式没有
            // inertial 参数，故只发布可定义的 Fx 与 |sigma|。
            std_msgs::Float64 scalar;
            scalar.data = d.piHat(0); plotFxHat.publish(scalar);
            scalar.data = d.sigma.norm(); plotSigmaNorm.publish(scalar);
          } else if (adaptiveSettings.parameterization ==
                     adaptive::AdaptiveParameterization::PayloadInertial10D) {
            // 10D 模式同理，只发布物理上存在的 payload mass 与 |sigma|。
            std_msgs::Float64 scalar;
            scalar.data = d.piHat(0); plotMassHat.publish(scalar);
            scalar.data = d.sigma.norm(); plotSigmaNorm.publish(scalar);
          }

          // Task B Eq.(10) 直接诊断，固定 24 列：
          // [observation_time, snapshot_time, time_error, snapshot_version,
          //  enableParameterUpdate, enableAdaptiveDynamics,
          //  Yu*pi_hat[6], candidate(-Yu*pi_hat/m)[6], applied_correction[6]].
          // candidate 在 estimate_only 中仍可非零，用于证明 Eq.(8) 正常运行；
          // applied_correction 只有 full_adaptive 非零，与实际 dynamics 装配完全一致。
          if (adaptiveSettings.parameterization !=
              adaptive::AdaptiveParameterization::PayloadInertial10D) {
            const adaptive::vector6_t candidateCorrection =
                d.candidateDynamicsCorrection;
            adaptive::vector6_t appliedCorrection = adaptive::vector6_t::Zero();
            if (adaptiveSettings.enableAdaptiveDynamics) {
              appliedCorrection = candidateCorrection;
            }
            std_msgs::Float64MultiArray dynamicsSample;
            dynamicsSample.data.reserve(24);
            dynamicsSample.data.push_back(d.observationTime);
            dynamicsSample.data.push_back(d.snapshotTime);
            dynamicsSample.data.push_back(d.timeError);
            dynamicsSample.data.push_back(static_cast<double>(snapshot->version()));
            dynamicsSample.data.push_back(adaptiveSettings.enableParameterUpdate ? 1.0 : 0.0);
            dynamicsSample.data.push_back(adaptiveSettings.enableAdaptiveDynamics ? 1.0 : 0.0);
            dynamicsSample.data.insert(
                dynamicsSample.data.end(),
                d.adaptiveDynamicsEquivalentWrench.data(),
                d.adaptiveDynamicsEquivalentWrench.data() + 6);
            dynamicsSample.data.insert(dynamicsSample.data.end(), candidateCorrection.data(),
                                       candidateCorrection.data() + 6);
            dynamicsSample.data.insert(dynamicsSample.data.end(), appliedCorrection.data(),
                                       appliedCorrection.data() + 6);
            equation10Sample.publish(dynamicsSample);
          } else {
            const auto flow = payloadInertialRuntimeDiagnostics->evaluate(
                observation.time, observation.state, observation.input,
                snapshot, adaptiveSettings.enableAdaptiveDynamics);

            // Versioned, fixed 112-field schema:
            // metadata/flags [12], composed model [10], pi_hat [10],
            // pi_dot [10], Y [60 row-major], Y*pi_hat [6], norms [4].
            std_msgs::Float64MultiArray inertialSample;
            inertialSample.layout.dim.resize(1);
            inertialSample.layout.dim[0].label =
                "payload_inertial_10d_v2_112_fields";
            inertialSample.layout.dim[0].size = 112;
            inertialSample.layout.dim[0].stride = 112;
            inertialSample.data.reserve(112);
            inertialSample.data.push_back(d.observationTime);             // 0
            inertialSample.data.push_back(d.snapshotTime);                // 1
            inertialSample.data.push_back(d.timeError);                   // 2
            inertialSample.data.push_back(static_cast<double>(snapshot->version())); // 3
            inertialSample.data.push_back(adaptiveSettings.enableParameterUpdate ? 1.0 : 0.0); // 4
            inertialSample.data.push_back(adaptiveSettings.enableAdaptiveDynamics ? 1.0 : 0.0); // 5
            inertialSample.data.push_back(adaptiveSettings.enableClfConstraint ? 1.0 : 0.0);     // 6
            inertialSample.data.push_back(10.0);                           // 7
            inertialSample.data.push_back(flow.estimatedModelValid ? 1.0 : 0.0); // 8
            inertialSample.data.push_back(flow.adaptiveDynamicsApplied ? 1.0 : 0.0); // 9
            inertialSample.data.push_back(0.0);                            // 10: no projection/fallback
            inertialSample.data.push_back(robotMass);                      // 11
            inertialSample.data.push_back(flow.estimatedTotalMass);        // 12
            inertialSample.data.insert(inertialSample.data.end(),
                flow.estimatedTotalComShiftNominalComBase.data(),
                flow.estimatedTotalComShiftNominalComBase.data() + 3);     // 13:16
            const auto& inertia = flow.estimatedTotalInertiaComBase;
            inertialSample.data.push_back(inertia(0, 0));                  // 16
            inertialSample.data.push_back(inertia(0, 1));                  // 17
            inertialSample.data.push_back(inertia(0, 2));                  // 18
            inertialSample.data.push_back(inertia(1, 1));                  // 19
            inertialSample.data.push_back(inertia(1, 2));                  // 20
            inertialSample.data.push_back(inertia(2, 2));                  // 21
            inertialSample.data.insert(inertialSample.data.end(),
                d.piHat.data(), d.piHat.data() + 10);                      // 22:32
            inertialSample.data.insert(inertialSample.data.end(),
                d.parameterDerivative.data(),
                d.parameterDerivative.data() + 10);                       // 32:42
            for (Eigen::Index row = 0; row < 6; ++row) {
              for (Eigen::Index column = 0; column < 10; ++column) {
                inertialSample.data.push_back(
                    d.unknownReferenceRegressor(row, column));            // 42:102
              }
            }
            inertialSample.data.insert(inertialSample.data.end(),
                d.estimatedUnknownWrench.data(),
                d.estimatedUnknownWrench.data() + 6);                     // 102:108
            inertialSample.data.push_back(d.piHat.norm());                // 108
            inertialSample.data.push_back(d.parameterDerivative.norm());  // 109
            inertialSample.data.push_back(flow.candidateFullFlowCorrection.norm()); // 110
            inertialSample.data.push_back(flow.appliedFullFlowCorrection.norm());   // 111
            payloadInertialSample.publish(inertialSample);

            // D3 adds the synchronized executed sigma[6] without changing
            // the historical D2 v2 topic/schema. The remaining fields retain
            // their v2 order after the six inserted sigma components.
            std_msgs::Float64MultiArray inertialV3Sample;
            inertialV3Sample.layout.dim.resize(1);
            inertialV3Sample.layout.dim[0].label =
                "payload_inertial_10d_v3_118_fields";
            inertialV3Sample.layout.dim[0].size = 118;
            inertialV3Sample.layout.dim[0].stride = 118;
            inertialV3Sample.data.reserve(118);
            inertialV3Sample.data.insert(inertialV3Sample.data.end(),
                inertialSample.data.begin(), inertialSample.data.begin() + 42);
            inertialV3Sample.data.insert(inertialV3Sample.data.end(),
                d.sigma.data(), d.sigma.data() + 6);
            inertialV3Sample.data.insert(inertialV3Sample.data.end(),
                inertialSample.data.begin() + 42, inertialSample.data.end());
            payloadInertialV3Sample.publish(inertialV3Sample);

            // D5H1 exact applied-update evidence, fixed 71-field schema:
            // metadata/phase/flags [15], raw drive [10], nominal baseline
            // [10], applied drive [10], actual pi_dot [10], pi_hat [10],
            // and the exact synchronized sigma [6].  These values come from
            // the SolverSynchronizedModule that performed Eq.(8), not a ROS
            // replay or a second phase tracker.
            std_msgs::Float64MultiArray appliedUpdateSample;
            appliedUpdateSample.layout.dim.resize(1);
            appliedUpdateSample.layout.dim[0].label =
                "payload_inertial_d5h1_applied_v1_71_fields";
            appliedUpdateSample.layout.dim[0].size = 71;
            appliedUpdateSample.layout.dim[0].stride = 71;
            appliedUpdateSample.data.reserve(71);
            appliedUpdateSample.data.push_back(updateDiagnostics.observationTime); // 0
            appliedUpdateSample.data.push_back(updateDiagnostics.snapshotTime);    // 1
            appliedUpdateSample.data.push_back(
                updateDiagnostics.snapshotTime - updateDiagnostics.observationTime); // 2
            appliedUpdateSample.data.push_back(
                static_cast<double>(updateDiagnostics.snapshotVersion));           // 3
            appliedUpdateSample.data.push_back(
                updateDiagnostics.parameterUpdateEnabled ? 1.0 : 0.0);             // 4
            appliedUpdateSample.data.push_back(
                updateDiagnostics.gaitPhaseBaselineConfigured ? 1.0 : 0.0);         // 5
            appliedUpdateSample.data.push_back(
                updateDiagnostics.phaseValid ? 1.0 : 0.0);                         // 6
            appliedUpdateSample.data.push_back(
                updateDiagnostics.correctionApplied ? 1.0 : 0.0);                  // 7
            appliedUpdateSample.data.push_back(
                updateDiagnostics.cycleBoundary ? 1.0 : 0.0);                      // 8
            appliedUpdateSample.data.push_back(
                static_cast<double>(updateDiagnostics.phaseBin));                  // 9
            appliedUpdateSample.data.push_back(
                static_cast<double>(updateDiagnostics.mode));                      // 10
            appliedUpdateSample.data.push_back(updateDiagnostics.normalizedPhase); // 11
            appliedUpdateSample.data.push_back(updateDiagnostics.cycleStart);       // 12
            appliedUpdateSample.data.push_back(
                updateDiagnostics.rawParameterDrive.norm());                       // 13
            appliedUpdateSample.data.push_back(
                updateDiagnostics.appliedParameterDrive.norm());                   // 14
            appliedUpdateSample.data.insert(appliedUpdateSample.data.end(),
                updateDiagnostics.rawParameterDrive.data(),
                updateDiagnostics.rawParameterDrive.data() + 10);                  // 15:25
            appliedUpdateSample.data.insert(appliedUpdateSample.data.end(),
                updateDiagnostics.nominalDriveBaseline.data(),
                updateDiagnostics.nominalDriveBaseline.data() + 10);               // 25:35
            appliedUpdateSample.data.insert(appliedUpdateSample.data.end(),
                updateDiagnostics.appliedParameterDrive.data(),
                updateDiagnostics.appliedParameterDrive.data() + 10);              // 35:45
            appliedUpdateSample.data.insert(appliedUpdateSample.data.end(),
                updateDiagnostics.appliedParameterDerivative.data(),
                updateDiagnostics.appliedParameterDerivative.data() + 10);         // 45:55
            appliedUpdateSample.data.insert(appliedUpdateSample.data.end(),
                d.piHat.data(), d.piHat.data() + 10);                              // 55:65
            appliedUpdateSample.data.insert(appliedUpdateSample.data.end(),
                updateDiagnostics.sigma.data(),
                updateDiagnostics.sigma.data() + 6);                               // 65:71
            d5h1AppliedUpdateSample.publish(appliedUpdateSample);

            // D5I4 exact online composite-phase correction evidence.  Fixed
            // 76-field schema: metadata [4], flags/key/phase/norms [16], raw
            // drive [10], frozen baseline [10], applied drive [10], actual
            // pi_dot [10], frozen pi_hat [10], synchronized sigma [6].  The
            // baseline is the exact value supplied to Eq.(8), never a ROS
            // replay. Original modes publish the same schema with configured=0
            // and an exactly zero baseline.
            std_msgs::Float64MultiArray compositeUpdateSample;
            compositeUpdateSample.layout.dim.resize(1);
            compositeUpdateSample.layout.dim[0].label =
                "payload_inertial_d5i4_composite_update_v1_76_fields";
            compositeUpdateSample.layout.dim[0].size = 76;
            compositeUpdateSample.layout.dim[0].stride = 76;
            compositeUpdateSample.data.reserve(76);
            compositeUpdateSample.data.push_back(updateDiagnostics.observationTime); // 0
            compositeUpdateSample.data.push_back(updateDiagnostics.snapshotTime);    // 1
            compositeUpdateSample.data.push_back(
                updateDiagnostics.snapshotTime - updateDiagnostics.observationTime); // 2
            compositeUpdateSample.data.push_back(
                static_cast<double>(updateDiagnostics.snapshotVersion));              // 3
            compositeUpdateSample.data.push_back(
                updateDiagnostics.parameterUpdateEnabled ? 1.0 : 0.0);                // 4
            compositeUpdateSample.data.push_back(
                updateDiagnostics.compositePhaseBaselineConfigured ? 1.0 : 0.0);      // 5
            compositeUpdateSample.data.push_back(
                updateDiagnostics.compositeSteadyStageActive ? 1.0 : 0.0);             // 6
            compositeUpdateSample.data.push_back(
                updateDiagnostics.compositeGaitPhaseValid ? 1.0 : 0.0);                // 7
            compositeUpdateSample.data.push_back(
                updateDiagnostics.compositeKeySupported ? 1.0 : 0.0);                 // 8
            compositeUpdateSample.data.push_back(
                updateDiagnostics.correctionApplied ? 1.0 : 0.0);                     // 9
            compositeUpdateSample.data.push_back(
                updateDiagnostics.cycleBoundary ? 1.0 : 0.0);                         // 10
            compositeUpdateSample.data.push_back(static_cast<double>(
                updateDiagnostics.compositeProfilePhaseBin));                         // 11
            compositeUpdateSample.data.push_back(static_cast<double>(
                updateDiagnostics.compositeGaitPhaseBin));                            // 12
            compositeUpdateSample.data.push_back(
                static_cast<double>(updateDiagnostics.mode));                         // 13
            compositeUpdateSample.data.push_back(
                updateDiagnostics.compositeProfilePhase);                             // 14
            compositeUpdateSample.data.push_back(
                updateDiagnostics.compositeGaitPhase);                                // 15
            compositeUpdateSample.data.push_back(
                updateDiagnostics.compositeCycleStart);                               // 16
            compositeUpdateSample.data.push_back(
                updateDiagnostics.rawParameterDrive.norm());                          // 17
            compositeUpdateSample.data.push_back(
                updateDiagnostics.nominalDriveBaseline.norm());                       // 18
            compositeUpdateSample.data.push_back(
                updateDiagnostics.appliedParameterDrive.norm());                      // 19
            compositeUpdateSample.data.insert(compositeUpdateSample.data.end(),
                updateDiagnostics.rawParameterDrive.data(),
                updateDiagnostics.rawParameterDrive.data() + 10);                     // 20:30
            compositeUpdateSample.data.insert(compositeUpdateSample.data.end(),
                updateDiagnostics.nominalDriveBaseline.data(),
                updateDiagnostics.nominalDriveBaseline.data() + 10);                  // 30:40
            compositeUpdateSample.data.insert(compositeUpdateSample.data.end(),
                updateDiagnostics.appliedParameterDrive.data(),
                updateDiagnostics.appliedParameterDrive.data() + 10);                 // 40:50
            compositeUpdateSample.data.insert(compositeUpdateSample.data.end(),
                updateDiagnostics.appliedParameterDerivative.data(),
                updateDiagnostics.appliedParameterDerivative.data() + 10);            // 50:60
            compositeUpdateSample.data.insert(compositeUpdateSample.data.end(),
                d.piHat.data(), d.piHat.data() + 10);                                 // 60:70
            compositeUpdateSample.data.insert(compositeUpdateSample.data.end(),
                updateDiagnostics.sigma.data(),
                updateDiagnostics.sigma.data() + 6);                                  // 70:76
            d5i4CompositeUpdateSample.publish(compositeUpdateSample);

            // D5H1 Eq.(11) audit evidence, fixed 119-field schema.  This is
            // emitted from the same executed evaluator result and immutable
            // snapshot used by the other D5H1 diagnostics; it does not add a
            // second CLF implementation or modify the solver constraint.
            std_msgs::Float64MultiArray clfSample;
            clfSample.layout.dim.resize(1);
            clfSample.layout.dim[0].label =
                "payload_inertial_d5h1_eq11_v1_119_fields";
            clfSample.layout.dim[0].size = 119;
            clfSample.layout.dim[0].stride = 119;
            clfSample.data.reserve(119);
            clfSample.data.push_back(d.observationTime);                         // 0
            clfSample.data.push_back(d.snapshotTime);                            // 1
            clfSample.data.push_back(d.timeError);                               // 2
            clfSample.data.push_back(static_cast<double>(snapshot->version()));  // 3
            clfSample.data.push_back(adaptiveSettings.enableClfConstraint ? 1.0 : 0.0); // 4
            clfSample.data.push_back(adaptiveSettings.useSoftConstraint ? 1.0 : 0.0);   // 5
            clfSample.data.push_back(d.usesFullFlowAdaptiveDynamics ? 1.0 : 0.0);       // 6
            clfSample.data.push_back(10.0);                                      // 7
            clfSample.data.push_back(d.hClf);                                    // 8
            clfSample.data.push_back(d.reconstructedH);                          // 9
            clfSample.data.push_back(d.violation);                               // 10
            clfSample.data.push_back(d.clfLinearTerm);                           // 11
            clfSample.data.push_back(d.dampingTerm);                             // 12
            clfSample.data.insert(clfSample.data.end(),
                d.sigma.data(), d.sigma.data() + 6);                             // 13:19
            clfSample.data.insert(clfSample.data.end(),
                d.piHat.data(), d.piHat.data() + 10);                            // 19:29
            clfSample.data.insert(clfSample.data.end(),
                d.contactWrench.data(), d.contactWrench.data() + 6);             // 29:35
            clfSample.data.insert(clfSample.data.end(),
                d.nominalReferenceWrench.data(),
                d.nominalReferenceWrench.data() + 6);                            // 35:41
            clfSample.data.insert(clfSample.data.end(),
                d.estimatedUnknownWrench.data(),
                d.estimatedUnknownWrench.data() + 6);                            // 41:47
            clfSample.data.insert(clfSample.data.end(),
                d.residualWrench.data(), d.residualWrench.data() + 6);           // 47:53
            clfSample.data.insert(clfSample.data.end(),
                d.clfContribution.data(), d.clfContribution.data() + 6);         // 53:59
            for (Eigen::Index row = 0; row < 6; ++row) {
              for (Eigen::Index column = 0; column < 10; ++column) {
                clfSample.data.push_back(
                    d.unknownReferenceRegressor(row, column));                   // 59:119
              }
            }
            d5h1ClfSample.publish(clfSample);

            // D5D diagnostic-only schema. It decomposes the exact sliding
            // variable used by Eq.(8), without changing the estimator:
            // sigma = [v_d-v; omega_d-omega]
            //       + [Lambda_l(p_d-p); Lambda_o*e_o].
            // The final 20 values are the corresponding two additive
            // contributions to the raw pre-robustness candidate
            // Gamma*Y^T*sigma. The exact applied D5H1 derivative is published
            // by d5h1AppliedUpdateSample above.
            const adaptive::parameter_vector_t velocityParameterDerivative =
                adaptiveSettings.gamma *
                d.unknownReferenceRegressor.transpose() *
                d.velocityErrorWorld;
            const adaptive::parameter_vector_t poseParameterDerivative =
                adaptiveSettings.gamma *
                d.unknownReferenceRegressor.transpose() *
                d.lambdaPoseErrorWorld;
            std_msgs::Float64MultiArray sigmaDecompositionSample;
            sigmaDecompositionSample.layout.dim.resize(1);
            sigmaDecompositionSample.layout.dim[0].label =
                "payload_inertial_sigma_decomposition_v1_52_fields";
            sigmaDecompositionSample.layout.dim[0].size = 52;
            sigmaDecompositionSample.layout.dim[0].stride = 52;
            sigmaDecompositionSample.data.reserve(52);
            sigmaDecompositionSample.data.push_back(d.observationTime);       // 0
            sigmaDecompositionSample.data.push_back(d.snapshotTime);          // 1
            sigmaDecompositionSample.data.push_back(d.timeError);             // 2
            sigmaDecompositionSample.data.push_back(
                static_cast<double>(snapshot->version()));                    // 3
            sigmaDecompositionSample.data.insert(
                sigmaDecompositionSample.data.end(),
                d.velocityErrorWorld.data(),
                d.velocityErrorWorld.data() + 6);                              // 4:10
            sigmaDecompositionSample.data.insert(
                sigmaDecompositionSample.data.end(),
                d.lambdaPoseErrorWorld.data(),
                d.lambdaPoseErrorWorld.data() + 6);                            // 10:16
            sigmaDecompositionSample.data.insert(
                sigmaDecompositionSample.data.end(),
                d.sigma.data(), d.sigma.data() + 6);                           // 16:22
            sigmaDecompositionSample.data.insert(
                sigmaDecompositionSample.data.end(),
                d.piHat.data(), d.piHat.data() + 10);                          // 22:32
            sigmaDecompositionSample.data.insert(
                sigmaDecompositionSample.data.end(),
                velocityParameterDerivative.data(),
                velocityParameterDerivative.data() + 10);                     // 32:42
            sigmaDecompositionSample.data.insert(
                sigmaDecompositionSample.data.end(),
                poseParameterDerivative.data(),
                poseParameterDerivative.data() + 10);                         // 42:52
            payloadInertialSigmaDecompositionSample.publish(
                sigmaDecompositionSample);

            // Full state-dimensional Eq. (10) evidence in nominal controller
            // coordinates: nominal flow, candidate delta, actually applied delta.
            std_msgs::Float64MultiArray fullFlowSample;
            fullFlowSample.layout.dim.resize(1);
            fullFlowSample.layout.dim[0].label =
                "payload_inertial_eq10_full_flow_v2_80_fields";
            fullFlowSample.layout.dim[0].size = 80;
            fullFlowSample.layout.dim[0].stride = 80;
            fullFlowSample.data.reserve(80);
            fullFlowSample.data.push_back(d.observationTime);
            fullFlowSample.data.push_back(d.snapshotTime);
            fullFlowSample.data.push_back(d.timeError);
            fullFlowSample.data.push_back(static_cast<double>(snapshot->version()));
            fullFlowSample.data.push_back(adaptiveSettings.enableParameterUpdate ? 1.0 : 0.0);
            fullFlowSample.data.push_back(adaptiveSettings.enableAdaptiveDynamics ? 1.0 : 0.0);
            fullFlowSample.data.push_back(flow.estimatedModelValid ? 1.0 : 0.0);
            fullFlowSample.data.push_back(flow.adaptiveDynamicsApplied ? 1.0 : 0.0);
            fullFlowSample.data.insert(fullFlowSample.data.end(),
                flow.nominalFlow.data(), flow.nominalFlow.data() + flow.nominalFlow.size());
            fullFlowSample.data.insert(fullFlowSample.data.end(),
                flow.candidateFullFlowCorrection.data(),
                flow.candidateFullFlowCorrection.data() + flow.candidateFullFlowCorrection.size());
            fullFlowSample.data.insert(fullFlowSample.data.end(),
                flow.appliedFullFlowCorrection.data(),
                flow.appliedFullFlowCorrection.data() + flow.appliedFullFlowCorrection.size());
            payloadInertialFullFlowSample.publish(fullFlowSample);
          }
          lastPublished->first = observation.time;
          lastPublished->second = snapshot->version();
        });

    retainedSubscribers.emplace_back(std::move(executedObservationSubscriber));
    retainedTimers.emplace_back(std::move(executedDiagnosticTimer));
  }

  MPC_ROS_Interface mpcNode(mpc, robotName);
  mpcNode.launchNodes(nodeHandle);

  return 0;
}
