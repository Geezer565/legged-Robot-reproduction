# Go2 MyRBF 六维闭环接入 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [x]`) syntax for tracking.

**Goal:** 在老师已有的 Go2/OCS2 工作区中接入六维 MyRBF，使它能根据 MPC 一步预测与实际运动之间的差异在线估计六维补偿，并在明确开关、幅值限制和足端约束保护下进入闭环控制。

**Architecture:** 保持原 OCS2、Pinocchio、hpp-fcl、NMPC 和 WBC 不变，把 RBF 算法放在 `legged_interface`，把 Go2 数据适配和控制接线放在 `legged_controllers`。控制顺序固定为“状态估计 → MPC 当前策略 → RBF 观察和更新 → 六维补偿分配到支撑脚 → WBC → 关节命令”。

**Tech Stack:** ROS Noetic、catkin、OCS2、Eigen、Pinocchio、C++、Python、Gazebo、Docker。

**Spec:** [`docs/MYRBF_GO2_V4_CONTROL_INTERFACE_20260829.md`](../../MYRBF_GO2_V4_CONTROL_INTERFACE_20260829.md)

## Global Constraints

- 只在独立工作树 `D:\code\go2_myrbf_baseline_port_20260829` 修改，不覆盖老师原始工作区。
- 两个自适应源码包必须依附老师已有 OCS2 工作区使用，不复制 OCS2、Pinocchio、hpp-fcl 等底层依赖。
- 默认关闭控制补偿；未显式打开时，原控制器输出不得被改变。
- 必须保留 Baseline、ETHRBF、MyRBF v3 和 MyRBF v4 四种运行方式。
- 六维顺序统一为 `[Fx, Fy, Fz, Tx, Ty, Tz]`，前三项单位为 N，后三项单位为 N·m。
- 网络输出表示控制器需要施加的反向补偿，不当作真实六维力传感器读数。
- RBF 不直接写 12 个关节力矩，只修改 MPC 输出中的四脚接触力，再交给原 WBC。
- 真机默认不打开补偿；必须先通过 Gazebo 无扰动、恒定扰动、周期扰动和行走门禁。

---

## 1. 完成标准

只有下面各项同时满足，才算“RBF 已接入 Go2”。

- [x] 当前工作区是独立 Git 工作树，分支和基线提交可追溯。
- [x] Docker 内编译的源码与当前隔离工作区一致。
- [x] 六个独立 RBF 子网络可编译、可更新、可复位。
- [x] 每个子网络确实使用本方向的两个输入特征。
- [x] 上周期 MPC 一步预测与本周期实际状态按时间对齐。
- [x] 已施加的补偿不会被观察器误当成外力消失。
- [x] ETH 跟踪误差和 MyRBF 预测误差能通过配置分别打开或组合。
- [x] 六维输出经过增益、幅值和变化速度限制。
- [x] 六维补偿按当前支撑脚位置分配，并检查摩擦与支撑力。
- [x] 控制补偿插在 MPC 当前输出之后、WBC 之前。
- [x] Baseline 关闭补偿后不改变原控制链。
- [x] 诊断记录包含残差、估计、两个输入、目标/实际状态和实际执行补偿。
- [x] C++ 检查、Python 检查、无扰动闭环和受扰动闭环全部通过。

## 2. 当前代码结构

```text
go2_myrbf_baseline_port_20260829/
├─ legged_interface/
│  ├─ include/legged_interface/adaptive/   RBF 对外接口和数据类型
│  ├─ src/adaptive/                        RBF、观察器、更新律和足端分配
│  └─ test/adaptive/                       六组 C++ 检查
├─ legged_controllers/
│  ├─ include/legged_controllers/          控制器成员和接入函数声明
│  ├─ src/LeggedController.cpp             控制周期中的实际接线
│  ├─ config/go2/                          四种方法的配置
│  ├─ launch/                              Gazebo 一键启动入口
│  ├─ scripts/                             扰动、记录和结果分析
│  └─ test/                                两组 Python 检查
├─ legged_estimation/                      Go2 状态估计
├─ legged_wbc/                             全身控制器
├─ legged_hw/                              真机接口
├─ legged_gazebo/                          仿真接口
└─ results/                                已验证结果
```

### 2.1 数据流

```text
Go2 实际状态 currentObservation
          │
          ├───────────────┐
          │               │
          ▼               │
MPC evaluatePolicy        │ 上周期 rolloutPolicy 一步预测
          │               │
          ▼               ▼
optimizedState/Input   时间对齐并计算六维运动差异
          │               │
          │               ▼
          │       六个 RBF 子网络在线更新
          │               │
          │               ▼
          └─────── 六维补偿 [F,T]
                          │
                          ▼
               分配为四只支撑脚的受力改变量
                          │
                          ▼
                   原 WBC 计算关节力矩
```

### 2.2 每个 RBF 子网络的输入与输出

对第 `i` 个方向：

```text
输入 1 = filteredResidual[i] / residualScale[i]
输入 2 = (filteredResidual[i] - previousEstimate[i]) / discrepancyScale[i]
输出   = outputScale[i] × Σ(weight[j,i] × basis[j,i])
```

其中：

- `filteredResidual` 是实际运动和上一周期 MPC 一步预测之间的六维差异经过滤波后的结果；
- `previousEstimate` 是网络上一轮输出；
- 第二个输入表示还有多少差异没有被上一轮输出解释；
- 六个网络分别产生 `Fx、Fy、Fz、Tx、Ty、Tz`；
- 每个网络包含 25 个固定高斯节点和 1 个常数节点；
- 中心和宽度固定，运行时只更新 26 个输出权值。

更新式在当前实现中为：

```text
dW/dt = trackingGain × sigma × gradient
      + predictionGain × (residual - estimate) × gradient
      - leakage × W
```

- `trackingGain > 0, predictionGain = 0`：接近 ETHRBF。
- `trackingGain = 0, predictionGain > 0`：当前 MyRBF v4。
- 两项都大于零：混合更新，但必须重新做稳定性和参数验证。

## 3. 文件迁移清单

从当前已验证版本迁到另一份干净 Go2 基线时，按下面清单操作。不要只复制 `MyRbfShadowEstimator.cpp`，否则缺少观察、控制和记录接口。

### 3.1 原样加入的算法文件

```text
legged_interface/include/legged_interface/adaptive/AdaptiveTypes.h
legged_interface/include/legged_interface/adaptive/MyRbfPredictionResidualObserver.h
legged_interface/include/legged_interface/adaptive/MyRbfShadowEstimator.h
legged_interface/include/legged_interface/adaptive/MyRbfShadowPipeline.h
legged_interface/include/legged_interface/adaptive/MyRbfTrackingError.h
legged_interface/include/legged_interface/adaptive/MyRbfFyCompensation.h
legged_interface/include/legged_interface/adaptive/MyRbfWrenchCompensation.h

legged_interface/src/adaptive/MyRbfPredictionResidualObserver.cpp
legged_interface/src/adaptive/MyRbfShadowEstimator.cpp
legged_interface/src/adaptive/MyRbfShadowPipeline.cpp
legged_interface/src/adaptive/MyRbfTrackingError.cpp
legged_interface/src/adaptive/MyRbfFyCompensation.cpp
legged_interface/src/adaptive/MyRbfWrenchCompensation.cpp
```

### 3.2 原样加入的测试文件

```text
legged_interface/test/adaptive/TestMyRbfPredictionResidualObserver.cpp
legged_interface/test/adaptive/TestMyRbfShadowEstimator.cpp
legged_interface/test/adaptive/TestMyRbfShadowPipeline.cpp
legged_interface/test/adaptive/TestMyRbfTrackingError.cpp
legged_interface/test/adaptive/TestMyRbfFyCompensation.cpp
legged_interface/test/adaptive/TestMyRbfWrenchCompensation.cpp

legged_controllers/test/test_myrbf_shadow_recording.py
legged_controllers/test/test_myrbf_wrench_profile.py
```

### 3.3 原样加入的配置和运行文件

```text
legged_controllers/config/go2/myrbf_shadow_baseline.yaml
legged_controllers/config/go2/myrbf_shadow_eth.yaml
legged_controllers/config/go2/myrbf_shadow_v3.yaml
legged_controllers/config/go2/myrbf_shadow_v3_go2.yaml
legged_controllers/config/go2/myrbf_shadow_v4_go2.yaml

legged_controllers/launch/myrbf_shadow_gazebo.launch
legged_controllers/scripts/apply_myrbf_wrench.py
legged_controllers/scripts/record_myrbf_shadow.py
legged_controllers/scripts/record_stage8c.py
legged_controllers/scripts/unpause_when_controller_ready.py
```

### 3.4 需要合并而不是整文件覆盖的文件

```text
legged_interface/CMakeLists.txt
legged_controllers/CMakeLists.txt
legged_controllers/package.xml
legged_controllers/include/legged_controllers/LeggedController.h
legged_controllers/src/LeggedController.cpp
legged_controllers/launch/load_controller.launch
legged_examples/legged_unitree/legged_unitree_description/launch/empty_world.launch
```

如果目标仓库在上述文件中已经有其他人的修改，必须逐段合并，不能整文件覆盖。

---

### Task 1: 确认隔离工作区和干净基线

**Files:**
- Inspect: repository root and `.git`
- Test: existing RBF test targets

**Interfaces:**
- Consumes: 老师已有 Go2/OCS2 工作区和已安装依赖。
- Produces: 一个不污染老师原目录、可以单独回退的接入分支。

- [x] **Step 1: 核对当前分支和改动**

```powershell
Set-Location 'D:\code\go2_myrbf_baseline_port_20260829'
git status --short --branch
git rev-parse --git-dir
git rev-parse --git-common-dir
git worktree list
```

预期：当前分支为 `codex/myrbf-go2-baseline-port`；工作树目录与主仓库目录不同；开始修改前没有未说明的改动。

- [x] **Step 2: 核对 Docker 程序和容器**

Docker Desktop 启动程序：

```powershell
& 'C:\Users\lenovo\AppData\Local\Programs\DockerDesktop\Docker Desktop.exe'
```

Docker 命令程序：

```powershell
& 'C:\Users\lenovo\AppData\Local\Programs\DockerDesktop\resources\bin\docker.exe' ps
```

预期：能看到容器 `legged-robot-noetic`，状态为 `Up`。

- [x] **Step 3: 核对容器内源码与当前工作树一致**

```powershell
Get-FileHash 'D:\code\go2_myrbf_baseline_port_20260829\legged_controllers\src\LeggedController.cpp' -Algorithm SHA256
& 'C:\Users\lenovo\AppData\Local\Programs\DockerDesktop\resources\bin\docker.exe' exec legged-robot-noetic bash -lc "sha256sum /tmp/myrbf_go2_catkin/src/leggedcontrol_go2/legged_controllers/src/LeggedController.cpp"
```

预期：两个 SHA256 完全相同。不同则先同步源码，不能继续用旧容器结果冒充当前版本。

### Task 2: 接入六维 RBF 算法层

**Files:**
- Create: `legged_interface/include/legged_interface/adaptive/AdaptiveTypes.h`
- Create: `legged_interface/include/legged_interface/adaptive/MyRbfShadowEstimator.h`
- Create: `legged_interface/src/adaptive/MyRbfShadowEstimator.cpp`
- Modify: `legged_interface/CMakeLists.txt`
- Test: `legged_interface/test/adaptive/TestMyRbfShadowEstimator.cpp`

**Interfaces:**
- Consumes: 六维滤波残差 `vector6_t`、六维跟踪误差 `vector6_t`、更新时间 `scalar_t`。
- Produces: `MyRbfShadowStepResult` 和最新六维估计 `estimate()`。

- [x] **Step 1: 先加入网络行为检查**

检查至少覆盖：

```cpp
EXPECT_EQ(MyRbfShadowEstimator::kAxisCount, 6);
EXPECT_EQ(MyRbfShadowEstimator::kInputDimension, 2);
EXPECT_EQ(MyRbfShadowEstimator::kGaussianNodeCount, 25);
EXPECT_EQ(MyRbfShadowEstimator::kBasisCount, 26);
```

还必须覆盖零输入、恒定残差、不同方向相互独立、非法中心/宽度/尺度拒绝、复位后权值清零。

- [x] **Step 2: 建立固定结构的六个子网络**

核心接口保持为：

```cpp
MyRbfShadowStepResult step(const vector6_t& residualFiltered,
                           const vector6_t& sigma, scalar_t dt);
const vector6_t& estimate() const;
void reset();
```

两个输入必须按下面形式形成，不能把完整 12 维数据交叉送进每个子网络：

```cpp
feature.row(0) =
    residualFiltered.cwiseQuotient(settings_.residualScale).transpose();
feature.row(1) =
    (residualFiltered - previousEstimate_)
        .cwiseQuotient(settings_.discrepancyScale)
        .transpose();
```

- [x] **Step 3: 只在线更新最后一层权值**

```cpp
const vector6_t predictionError =
    residualFiltered - result.estimateBeforeUpdate;
result.weightDerivative.col(axis) =
    settings_.trackingGain(axis) * sigma(axis) * physicalGradient +
    settings_.predictionGain(axis) * predictionError(axis) * physicalGradient -
    settings_.leakage(axis) * weights_.col(axis);
weights_ += dt * result.weightDerivative;
```

高斯中心和宽度运行时保持不变。任何尺度、宽度、时间或权值出现非有限数值时必须拒绝。

- [x] **Step 4: 把源文件和测试加入构建**

在 `legged_interface/CMakeLists.txt` 的库源文件中加入六个 `src/adaptive/*.cpp`，在 `CATKIN_ENABLE_TESTING` 下加入六个 `catkin_add_gtest` 目标。

- [x] **Step 5: 编译并只运行网络检查**

```bash
source /opt/ros/noetic/setup.bash
cd /tmp/myrbf_go2_catkin
catkin build legged_interface -j2 -l2
cd build/legged_interface
ctest -R myrbf_shadow_estimator --output-on-failure
```

预期：编译成功，网络检查通过。

### Task 3: 接入一步预测残差和补偿感知

**Files:**
- Create: `legged_interface/include/legged_interface/adaptive/MyRbfPredictionResidualObserver.h`
- Create: `legged_interface/src/adaptive/MyRbfPredictionResidualObserver.cpp`
- Create: `legged_interface/include/legged_interface/adaptive/MyRbfShadowPipeline.h`
- Create: `legged_interface/src/adaptive/MyRbfShadowPipeline.cpp`
- Test: `legged_interface/test/adaptive/TestMyRbfPredictionResidualObserver.cpp`
- Test: `legged_interface/test/adaptive/TestMyRbfShadowPipeline.cpp`

**Interfaces:**
- Consumes: 上周期 MPC 一步预测状态、预测时刻、间隔、当前实际状态、上一周期已施加补偿。
- Produces: `netResidual`、`rawResidual`、`filteredResidual` 和更新后的 RBF 估计。

- [x] **Step 1: 先检查时间对齐和符号**

测试必须覆盖：没有预测时拒绝、实际时间不在容差内时拒绝、使用一次后预测失效、残差符号正确、滤波正确、复位正确。

- [x] **Step 2: 保存上周期一步预测**

```cpp
void setPrediction(scalar_t predictedTime, scalar_t intervalDt,
                   const vector_t& predictedState,
                   const vector6_t& knownAppliedCompensation);
```

预测只允许被下一次有效观察使用一次，避免旧预测反复更新网络。

- [x] **Step 3: 由前六维动量差形成六维补偿残差**

```cpp
netResidual = -robotMass *
    (actualState.head<6>() - predictedState.head<6>()) / intervalDt;
rawResidual = netResidual + knownAppliedCompensation;
```

这里加回 `knownAppliedCompensation` 的原因是：机器人实际运动已经受到了 RBF 补偿影响。若不加回，网络会把“自己补偿后的剩余差异”误认为完整外部扰动，输出容易提前变小。

- [x] **Step 4: 滤波后再送入网络**

```cpp
filteredResidual = alpha * previousFilteredResidual
                 + (1.0 - alpha) * rawResidual;
```

首个有效样本直接初始化，不与零值混合。

- [x] **Step 5: 组合为一条管线**

```cpp
result.residual = residualObserver_.observe(actualTime, actualState);
if (!result.residual.valid) return result;
result.estimator = estimator_.step(
    result.residual.filteredResidual, trackingError, result.residual.dt);
result.valid = true;
```

- [x] **Step 6: 运行观察器和管线检查**

```bash
source /opt/ros/noetic/setup.bash
source /tmp/myrbf_go2_catkin/devel/setup.bash
cd /tmp/myrbf_go2_catkin/build/legged_interface
ctest -R "myrbf_(prediction_residual_observer|shadow_pipeline)" --output-on-failure
```

预期：时间错位样本被拒绝，补偿感知计算正确，两组检查通过。

### Task 4: 保留 ETH 更新并允许与 MyRBF 分开配置

**Files:**
- Create: `legged_interface/include/legged_interface/adaptive/MyRbfTrackingError.h`
- Create: `legged_interface/src/adaptive/MyRbfTrackingError.cpp`
- Test: `legged_interface/test/adaptive/TestMyRbfTrackingError.cpp`
- Create: `legged_controllers/config/go2/myrbf_shadow_eth.yaml`
- Create: `legged_controllers/config/go2/myrbf_shadow_v4_go2.yaml`

**Interfaces:**
- Consumes: 实际刚体状态、MPC 目标刚体状态、线性和角度误差系数。
- Produces: 六维 `sigma=[sigma_v, sigma_w]`，作为权值更新的可选项。

- [x] **Step 1: 检查目标等于实际时误差为零**

```cpp
const vector6_t sigma = computeMyRbfTrackingError(
    actualRbdState, desiredRbdState, generalizedCoordinatesNum,
    lambdaLinear, lambdaAngular);
EXPECT_TRUE(sigma.isZero(1e-12));
```

- [x] **Step 2: 计算线速度、位置、角速度和姿态组合误差**

```cpp
sigma.head<3>() = desiredLinearVelocity - actualLinearVelocity +
                  lambdaLinear * (desiredPosition - actualPosition);
sigma.tail<3>() = desiredAngularVelocity - actualAngularVelocity +
                  lambdaAngular * quaternionTrackingError(actual, desired);
```

- [x] **Step 3: 用配置区分三种更新**

```yaml
# ETHRBF
tracking_gain: [非零六维值]
prediction_gain: [0, 0, 0, 0, 0, 0]

# MyRBF v4
tracking_gain: [0, 0, 0, 0, 0, 0]
prediction_gain: [按 Go2 标定的六维值]
```

混合更新只有在单独实验目录和单独配置名下才能启用，不能覆盖 ETHRBF 或 v4 配置。

- [x] **Step 4: 运行跟踪误差检查**

```bash
source /opt/ros/noetic/setup.bash
source /tmp/myrbf_go2_catkin/devel/setup.bash
cd /tmp/myrbf_go2_catkin/build/legged_interface
ctest -R myrbf_tracking_error --output-on-failure
```

预期：位置、速度、姿态和角速度方向均正确。

### Task 5: 在 Go2 控制器初始化阶段建立 RBF

**Files:**
- Modify: `legged_controllers/include/legged_controllers/LeggedController.h`
- Modify: `legged_controllers/src/LeggedController.cpp`
- Modify: `legged_controllers/package.xml`

**Interfaces:**
- Consumes: ROS 参数、Go2 质量、状态维数、Pinocchio 模型和四个足端名称。
- Produces: `myRbfShadowPipeline_`、六维足端分配所需模型对象和诊断发布器。

- [x] **Step 1: 在控制器头文件声明五个接入函数**

```cpp
void updateMyRbfShadowObservation(const vector_t&, const vector_t&);
void queueMyRbfShadowPrediction();
void applyMyRbfControlCompensation(vector_t&, std::size_t, scalar_t);
void applyMyRbfWrenchCompensation(vector_t&, std::size_t, scalar_t);
void publishMyRbfShadowDiagnostics(
    const adaptive::MyRbfShadowPipelineResult&, const vector_t&);
```

- [x] **Step 2: 所有功能使用显式开关**

```cpp
controller_nh.param("enable_myrbf_shadow", enableMyRbfShadow_, false);
controller_nh.param("enable_myrbf_control_injection",
                    enableMyRbfControlInjection_, false);
```

当 `enable_myrbf_control_injection=true` 但 `enable_myrbf_shadow=false` 时，初始化必须失败，防止控制器使用不存在的估计。

- [x] **Step 3: 加载并检查全部参数**

必须检查：更新时间为正、时间容差非负、六维尺度为正、学习系数非负、补偿幅值和变化速度限制为正、摩擦系数为正、单脚最大改变量为正。

- [x] **Step 4: 仅在六维接口启用时建立足端模型**

```cpp
if (myRbfControlInterface_ == "wrench_allocator") {
  // 建立 Pinocchio 副本、状态映射和四足末端位置计算对象
}
```

这样 Baseline 和只观察模式不会承担不必要的计算与风险。

- [x] **Step 5: 启动和停止时清空历史**

控制器每次启动必须清空上次预测、RBF 权值、已施加补偿和计数器，不能把上一场实验的学习结果带入下一场。

### Task 6: 在 MPC 输出之后、WBC 之前接入闭环

**Files:**
- Modify: `legged_controllers/src/LeggedController.cpp`

**Interfaces:**
- Consumes: `optimizedState`、`optimizedInput`、当前接触模式和控制周期。
- Produces: 加入安全补偿后的 `optimizedInput`，继续交给原 WBC。

- [x] **Step 1: 保持控制周期顺序固定**

```cpp
mpcMrtInterface_->evaluatePolicy(
    currentObservation_.time, currentObservation_.state,
    optimizedState, optimizedInput, plannedMode);

updateMyRbfShadowObservation(optimizedState, optimizedInput);
applyMyRbfControlCompensation(optimizedInput, plannedMode, period.toSec());
queueMyRbfShadowPrediction();

currentObservation_.input = optimizedInput;
vector_t x = wbc_->update(
    optimizedState, optimizedInput, measuredRbdState_,
    plannedMode, period.toSec());
```

不能把 `applyMyRbfControlCompensation` 放到 WBC 之后，否则只能绕过原控制约束直接改关节命令。

- [x] **Step 2: 用上一周期预测更新网络**

`updateMyRbfShadowObservation` 要完成：

1. 检查是否存在待比较的一步预测；
2. 将当前 MPC 目标转换为刚体目标状态；
3. 计算 ETH 可选跟踪误差；
4. 用当前实际状态更新观察器和 RBF；
5. 发布本周期诊断。

- [x] **Step 3: 在本周期 MPC 接触力上叠加六维补偿**

四脚接触力位于 `optimizedInput.segment<3>(3 * contact)`。只允许修改当前处于支撑状态的脚。

- [x] **Step 4: 生成下一周期的一步预测**

```cpp
mpcMrtInterface_->rolloutPolicy(
    currentObservation_.time, currentObservation_.state,
    updatePeriod, predictedState, predictedInput, predictedMode);

pipeline.setPrediction(
    currentObservation_.time + updatePeriod,
    updatePeriod, predictedState, appliedWrench);
```

这里保存的是实际分配后成功施加的 `appliedWrench`，不能保存未经足端约束裁剪的请求值。

### Task 7: 加入六维限制和四脚分配

**Files:**
- Create: `legged_interface/include/legged_interface/adaptive/MyRbfWrenchCompensation.h`
- Create: `legged_interface/src/adaptive/MyRbfWrenchCompensation.cpp`
- Test: `legged_interface/test/adaptive/TestMyRbfWrenchCompensation.cpp`

**Interfaces:**
- Consumes: 六维网络估计、上一轮补偿、每轴增益/幅值/变化速度限制、支撑脚位置和原计划受力。
- Produces: 每只脚三维受力改变量、实际完成的六维补偿和安全缩放比例。

- [x] **Step 1: 先检查补偿限制**

测试必须覆盖：每轴增益、幅值裁剪、变化速度裁剪、非法参数拒绝、零估计不产生补偿。

- [x] **Step 2: 对六维请求逐轴限制**

```cpp
target[axis] = clamp(
    controlGain[axis] * estimatedWrench[axis],
    -magnitudeLimit[axis], magnitudeLimit[axis]);
applied[axis] = rateLimitFromPrevious(
    previous[axis], target[axis], rateLimit[axis], dt);
```

- [x] **Step 3: 建立六维量到足端力的关系**

每只支撑脚的三维改变量 `Δf_i` 同时产生力和相对质心的力矩：

```text
Σ Δf_i              = requestedForce
Σ (r_i × Δf_i)      = requestedTorque
```

其中 `r_i` 是第 `i` 只脚相对当前质心的位置。

- [x] **Step 4: 检查每只脚的安全范围**

分配后必须同时满足：

```text
||Δf_i|| ≤ maximumForceDelta
finalFz_i ≥ minimumNormalForce
sqrt(finalFx_i² + finalFy_i²) ≤ frictionCoefficient × finalFz_i
```

若完整请求不满足，则整体缩小六维补偿；若仍无法满足，则本周期返回无效，不写入足端力。

- [x] **Step 5: 运行六维分配检查**

```bash
source /opt/ros/noetic/setup.bash
source /tmp/myrbf_go2_catkin/devel/setup.bash
cd /tmp/myrbf_go2_catkin/build/legged_interface
ctest -R myrbf_wrench_compensation --output-on-failure
```

预期：纯力、纯力矩、混合六维请求、部分支撑、摩擦限制和失败回退全部通过。

### Task 8: 接入配置、启动、扰动和记录

**Files:**
- Create: `legged_controllers/config/go2/myrbf_shadow_baseline.yaml`
- Create: `legged_controllers/config/go2/myrbf_shadow_eth.yaml`
- Create: `legged_controllers/config/go2/myrbf_shadow_v3_go2.yaml`
- Create: `legged_controllers/config/go2/myrbf_shadow_v4_go2.yaml`
- Create: `legged_controllers/launch/myrbf_shadow_gazebo.launch`
- Create: `legged_controllers/scripts/apply_myrbf_wrench.py`
- Create: `legged_controllers/scripts/record_myrbf_shadow.py`
- Modify: `legged_controllers/CMakeLists.txt`
- Modify: `legged_controllers/package.xml`

**Interfaces:**
- Consumes: ROS 启动参数和 `/myrbf_shadow_diagnostics`。
- Produces: 可切换方法的启动入口、外部扰动、状态 CSV、RBF CSV 和扰动 CSV。

- [x] **Step 1: 保持安全默认值**

```xml
<arg name="enable_shadow" default="true"/>
<arg name="enable_control_injection" default="false"/>
<arg name="myrbf_config" default="myrbf_shadow_v3.yaml"/>
```

普通 Go2 启动文件不得默认打开 MyRBF v4。

- [x] **Step 2: v4 使用六维接口和补偿感知**

```yaml
control_interface: wrench_allocator
compensation_aware_observer: true
tracking_gain: [0, 0, 0, 0, 0, 0]
control_gain_vector: [0.25, 0.25, 0.15, 0.08, 0.08, 0.08]
```

第一轮迁移不要提高这些比例。先确认方向、站立和记录正确，再按单独实验调节。

- [x] **Step 3: 诊断信息固定为 80 项**

记录必须包括：

```text
时间和周期                         2
原始/滤波/更新前估计/更新后估计    24
两组输入特征                      12
六维跟踪误差                       6
目标和实际机身姿态位置             12
旧 Fy 接口和状态计数                4
净残差                             6
请求补偿和实际补偿                 12
分配缩放和接口编号                  2
合计                              80
```

- [x] **Step 4: 检查记录和扰动脚本**

```bash
source /opt/ros/noetic/setup.bash
source /tmp/myrbf_go2_catkin/devel/setup.bash
cd /tmp/myrbf_go2_catkin/src/leggedcontrol_go2/legged_controllers
python3 -m unittest discover -s test -p "test_myrbf_*.py"
```

预期：6 项 Python 检查通过。

### Task 9: 完整编译和全部自动检查

**Files:**
- Verify: `legged_interface`
- Verify: `legged_controllers`

**Interfaces:**
- Consumes: 老师已有 `/opt/ros/noetic` 和 `/tmp/myrbf_go2_catkin`。
- Produces: 可被 Gazebo 和 Go2 控制器加载的构建结果。

- [x] **Step 1: 编译两个直接修改的包及依赖**

```bash
source /opt/ros/noetic/setup.bash
cd /tmp/myrbf_go2_catkin
catkin build legged_interface legged_controllers -j2 -l2
```

预期：所有涉及的包成功，无失败包。

- [x] **Step 2: 运行全部 C++ 检查**

```bash
source /opt/ros/noetic/setup.bash
source /tmp/myrbf_go2_catkin/devel/setup.bash
cd /tmp/myrbf_go2_catkin/build/legged_interface
ctest --output-on-failure
```

预期：6/6 通过。

- [x] **Step 3: 运行全部 Python 检查**

```bash
source /opt/ros/noetic/setup.bash
source /tmp/myrbf_go2_catkin/devel/setup.bash
cd /tmp/myrbf_go2_catkin/src/leggedcontrol_go2/legged_controllers
python3 -m unittest discover -s test -p "test_myrbf_*.py"
```

预期：6/6 通过。

### Task 10: 闭环冒烟验收

**Files:**
- Verify: `legged_controllers/launch/myrbf_shadow_gazebo.launch`
- Output: `/tmp/myrbf_acceptance_*`

**Interfaces:**
- Consumes: 编译后的 Go2 Gazebo、MyRBF v4 配置和扰动脚本。
- Produces: 一组无扰动数据和一组恒定扰动数据，用于判断闭环接线是否正常。

- [x] **Step 1: 无扰动运行 v4**

```bash
source /opt/ros/noetic/setup.bash
source /tmp/myrbf_go2_catkin/devel/setup.bash
roslaunch legged_controllers myrbf_shadow_gazebo.launch \
  gui:=false duration:=8.0 \
  state_output:=/tmp/myrbf_acceptance_nodist_state.csv \
  shadow_output:=/tmp/myrbf_acceptance_nodist_shadow.csv \
  wrench_output:=/tmp/myrbf_acceptance_nodist_wrench.csv \
  myrbf_config:=myrbf_shadow_v4_go2.yaml \
  enable_shadow:=true enable_control_injection:=true \
  enable_wrench:=false
```

通过条件：机器人没有安全退出；状态和 RBF 记录非空；控制接口编号为六维模式；实际补偿有限；机身没有持续漂移或倾倒。

- [x] **Step 2: 施加 10 N 恒定侧向扰动**

```bash
roslaunch legged_controllers myrbf_shadow_gazebo.launch \
  gui:=false duration:=10.0 \
  state_output:=/tmp/myrbf_acceptance_10n_state.csv \
  shadow_output:=/tmp/myrbf_acceptance_10n_shadow.csv \
  wrench_output:=/tmp/myrbf_acceptance_10n_wrench.csv \
  myrbf_config:=myrbf_shadow_v4_go2.yaml \
  enable_shadow:=true enable_control_injection:=true \
  enable_wrench:=true wrench_profile:=constant wrench_axis:=fy \
  wrench_value:=10.0 wrench_delay:=4.0 wrench_duration:=3.0
```

通过条件：扰动调用成功；RBF 的 Fy 输出方向正确；实际补偿不超过配置限制；机器人保持站立；撤力后没有持续放大的补偿。

- [x] **Step 3: 核对 CSV 内容**

```bash
python3 - <<'PY'
import csv
from pathlib import Path

for name in ("nodist", "10n"):
    state = Path(f"/tmp/myrbf_acceptance_{name}_state.csv")
    shadow = Path(f"/tmp/myrbf_acceptance_{name}_shadow.csv")
    assert state.stat().st_size > 100
    assert shadow.stat().st_size > 100
    with shadow.open() as handle:
        rows = list(csv.DictReader(handle))
    assert rows
    assert all(float(row["control_interface_mode"]) == 2.0 for row in rows)
print("MyRBF Go2 acceptance CSV check passed")
PY
```

预期：输出 `MyRBF Go2 acceptance CSV check passed`。

### Task 11: 回退验证和交付检查

**Files:**
- Verify: all modified files
- Update: this document execution record

**Interfaces:**
- Consumes: 完成的代码、配置和验收记录。
- Produces: 可以交给下一位同学继续使用的接入版本。

- [x] **Step 1: 验证 Baseline 回退**

使用同一启动文件，但设置：

```text
enable_shadow:=true
enable_control_injection:=false
```

预期：仍可记录 RBF，但 `optimizedInput` 不被 RBF 修改，实际执行补偿为零。

- [x] **Step 2: 验证方法切换关系**

```text
Baseline : enable_control_injection=false
ETHRBF   : myrbf_config=myrbf_shadow_eth.yaml
MyRBF v3 : myrbf_config=myrbf_shadow_v3_go2.yaml
MyRBF v4 : myrbf_config=myrbf_shadow_v4_go2.yaml
```

四种方式使用相同 Go2 模型、MPC、WBC、扰动脚本和记录字段。

- [x] **Step 3: 检查最终改动范围**

```powershell
git status --short
git diff --check
git diff --stat
```

预期：没有空白错误，没有意外修改老师原始工作区，没有生成文件混入源码目录。

## 4. 当前接入边界

当前实现已经是闭环控制，但补偿位置在 MPC 当前输出和 WBC 之间：

```text
MPC 当前策略输出 → MyRBF 六维接触力补偿 → WBC
```

这意味着：

- 当前周期的 RBF 输出确实会影响机器人；
- 下一周期的状态和 MPC 会看到这次控制造成的结果；
- 观察器记录了已施加补偿，避免自我干扰；
- MPC 的整个未来预测区间内部尚未显式加入 RBF 扰动模型。

因此当前成果应称为“Go2 上的 MyRBF-MPC/WBC 闭环接口”，不应写成“已经修改了 MPC 预测模型”。如果论文下一阶段确实要求把估计量放进预测时域，应另建分支，把扰动作为 MPC 动力学的外部参数传入，并重新检查求解时间、梯度一致性和稳定性；不能在本版本中悄悄改变这个边界。

## 5. 故障定位顺序

### 5.1 没有 RBF 数据

按顺序检查：

1. `enable_myrbf_shadow` 是否为 `true`；
2. `/myrbf_shadow_diagnostics` 是否存在；
3. `valid_sample_count` 是否增长；
4. `rejected_sample_count` 是否快速增长；
5. `update_period` 与控制器实际周期是否相容；
6. MPC `rolloutPolicy` 是否成功。

### 5.2 有估计，但完全没有控制效果

按顺序检查：

1. `enable_myrbf_control_injection` 是否为 `true`；
2. `control_interface_mode` 是否为 `2`；
3. `requested_control_*` 是否非零；
4. `achieved_control_*` 是否非零；
5. `allocator_scale` 是否长期为零；
6. 当前是否有支撑脚；
7. 摩擦、最小法向力或单脚改变量是否过严。

### 5.3 估计一开闭环就变小

检查 `compensation_aware_observer` 是否为 `true`，并确认保存的是 `achievedWrench` 而不是未经限制的 `requestedWrench`。

### 5.4 侧移变小但侧倾变大

不要继续放大单一 `Fy`。确认使用 `wrench_allocator`，并联合使用六维力和力矩分配；同时观察 `Ty/Tx`、质心位置和各脚相对质心位置是否正确。

### 5.5 仿真偶尔启动失败

确认 Gazebo 以暂停状态启动，控制器加载完成后由 `unpause_when_controller_ready.py` 解除暂停；扰动脚本必须等模型真正出现后再开始计时。

## 6. 执行记录

执行日期：2026-08-30。

| 门禁 | 结果 | 证据 |
|---|---|---|
| 隔离工作树 | 通过 | `codex/myrbf-go2-baseline-port`，基于 `7d87e51`，老师原目录未修改 |
| Docker 源码一致性 | 通过 | 主控制器 SHA256 两侧均为 `10fe80de...b2524a5`；本轮修改的头文件已同步后重编译 |
| 编译 | 通过 | 26 个涉及包全部成功，无失败、无警告 |
| C++ 检查 | 通过 | 6/6 通过 |
| Python 检查 | 通过 | 6/6 通过 |
| 无扰动闭环 | 通过 | 557 个 RBF 样本；最大侧移 0.220 cm；最大侧倾 0.265°；最低高度 0.3438 m |
| 10 N 恒定扰动闭环 | 通过 | 扰动调用 150/150；平均估计 -8.331 N；结束前 -9.064 N；机器人保持站立 |
| Baseline 回退 | 通过 | 已保存的 1554 个样本中实际侧向补偿最大值为 0 |
| 最终改动检查 | 通过 | `git diff --check` 无输出；改动仅包含本接入手册、验收数据、README 入口和三处过时说明修正 |

本轮原始验收数据已经在 M01 开始前从活动工作区清理，需要时可从 Git 历史恢复。这两场只用于确认接入正确，不替代后续论文实验。

## 7. 最终交付内容

接入完成后，交付至少包含：

```text
1. legged_interface/adaptive 全部 RBF 算法文件
2. LeggedController 的初始化、更新、补偿和记录接线
3. Baseline、ETHRBF、MyRBF v3、MyRBF v4 配置
4. Gazebo 启动、扰动和记录脚本
5. C++ 与 Python 检查
6. 本接入手册
7. 一组无扰动和一组受扰动的验收记录
```

交付时必须同时说明：当前验证属于 Go2 Gazebo，不是真实 Go2；真机补偿默认关闭；真正进入真机前还需要低速行走、负重、周期扰动、接触切换、通信延迟和紧急关闭检查。
