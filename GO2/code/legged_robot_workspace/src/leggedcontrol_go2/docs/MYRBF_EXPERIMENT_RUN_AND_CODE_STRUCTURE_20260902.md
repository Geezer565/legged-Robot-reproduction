# MyRBF 实验运行与代码结构

## 1. 这份说明回答什么

这份文件说明当前 Go2 + OCS2 仿真版本的代码怎样分工，以及一场实验从启动到出结果的完整过程。

当前实验是 Gazebo 仿真，不是真实 Go2。代码中已经预留了地形法向、摩擦和六维受力的接口，但当前启动文件使用的是平地，尚未完成多地形感知和实机验证。

## 2. 代码总结构

```text
legged_interface/       机器人模型接口、MPC 约束、MyRBF 观测和补偿
legged_controllers/     ROS 控制器、启动文件、参数和实验脚本
legged_examples/        Go2/A1 等机器人模型、URDF/Xacro 和 Gazebo 配置
legged_gazebo/          Gazebo 世界和仿真支持
legged_wbc/             全身控制器
results/                已保存的实验数据、图和报告
scripts/                批量实验和报告一致性检查
docs/                   设计、运行、版本和实验记录
```

### 2.1 MyRBF 与控制器相关文件

| 功能 | 主要文件 | 作用 |
|---|---|---|
| 一步预测残差 | `legged_interface/src/adaptive/MyRbfPredictionResidualObserver.cpp` | 把真实状态和上一周期 MPC 的一步预测对齐，计算六维残差 |
| 六个 RBF 子网络 | `legged_interface/src/adaptive/MyRbfShadowEstimator.cpp` | 分别估计 `Fx`、`Fy`、`Fz`、`Tx`、`Ty`、`Tz` |
| 网络流程 | `legged_interface/src/adaptive/MyRbfShadowPipeline.cpp` | 串联残差观测、跟踪信号和网络更新 |
| ETH 跟踪误差 | `legged_interface/src/adaptive/MyRbfTrackingError.cpp` | 计算位置、速度、姿态和角速度误差组合 |
| 六维补偿 | `legged_interface/src/adaptive/MyRbfWrenchCompensation.cpp` | 限制补偿大小和变化速度，并把机身力/力矩分配到支撑脚 |
| Go2 控制器接入 | `legged_controllers/src/LeggedController.cpp` | 在 MPC 输出和 WBC 输入之间调用 MyRBF |
| 参数 | `legged_controllers/config/go2/myrbf_shadow_*.yaml` | 选择方法、更新参数、补偿限幅和分配参数 |

### 2.2 实验相关文件

| 功能 | 文件 | 作用 |
|---|---|---|
| 一键启动 | `legged_controllers/launch/myrbf_shadow_gazebo.launch` | 启动 Go2 Gazebo、控制器、记录器和可选的扰动/行走节点 |
| 发布行走命令 | `legged_controllers/scripts/publish_myrbf_motion.py` | 在指定时间发布速度和步态命令，并记录命令时间 |
| 施加外力 | `legged_controllers/scripts/apply_myrbf_wrench.py` | 通过 Gazebo 的 `apply_body_wrench` 服务施加力或力矩，并记录真实施加结果 |
| 记录机器人状态 | `legged_controllers/scripts/record_stage8c.py` | 保存状态时间序列 |
| 记录 MyRBF | `legged_controllers/scripts/record_myrbf_shadow.py` | 保存残差、估计、补偿、更新和安全诊断 |
| 单场分析 | `legged_controllers/scripts/analyze_myrbf_v5a_paired.py` | 检查时间覆盖、外力窗口并生成批次指标 |
| 批量实验 | `legged_controllers/scripts/run_myrbf_v5a_paired_campaign.py` | 生成或执行 V4/V5-A 成对实验 |
| 报告检查 | `scripts/check_myrbf_v5a_repeat_report.py` | 检查报告、日志、源码锁和证据文件是否对应 |

## 3. 地形接口现在在哪里

### 3.1 当前已经有的部分

地形相关的底层接口主要在：

- `legged_interface/include/legged_interface/constraint/FrictionConeConstraint.h`
- `legged_interface/src/constraint/FrictionConeConstraint.cpp`
- `legged_interface/src/SwitchedModelReferenceManager.cpp`
- `legged_controllers/config/go2/task.info`
- `legged_examples/legged_unitree/legged_unitree_description/launch/empty_world.launch`
- `legged_gazebo/worlds/empty_world.world`

当前代码能够表达：

1. 接触脚的摩擦系数；
2. 接触力的摩擦锥约束；
3. 地形法向 `setSurfaceNormalInWorld(...)`；
4. 摆动脚参考高度；
5. Gazebo 模型中的接触摩擦参数。

### 3.2 当前尚未完成的部分

当前 `SwitchedModelReferenceManager.cpp` 中的地面高度仍固定为 `0.0`，启动文件加载的是平地世界。因此，现在不能说已经完成“多地形接口”。目前更准确的说法是：已有地形约束接口和未来扩展位置，但实验仍是平地仿真。

### 3.3 以后增加多地形时的接入方向

建议把地形信息统一成一个输入：

```text
Gazebo 地形 / 地形估计器
          ↓
高度、法向、摩擦系数
          ↓
参考轨迹 + 摆动脚高度 + 摩擦锥约束
          ↓
MPC / WBC / MyRBF 六维补偿分配
```

至少需要同步更新四个地方：

- 地形高度和摆动脚参考；
- 每个接触脚的地形法向；
- MPC/WBC 使用的摩擦系数；
- MyRBF 接触力分配时使用的摩擦系数和接触位置。

## 4. 受力接口现在在哪里

### 4.1 外部实验受力接口

`apply_myrbf_wrench.py` 是实验用的外力入口。它调用 Gazebo 服务：

```text
/gazebo/apply_body_wrench
```

通过启动参数指定：

| 参数 | 含义 |
|---|---|
| `wrench_axis` | `fx`、`fy`、`fz`、`tx`、`ty`、`tz` |
| `wrench_profile` | `constant`、`sine` 或 `multisine` |
| `wrench_value` | 恒定力/力矩，单位分别是 N 或 N·m |
| `wrench_amplitude` | 正弦或多频信号幅值 |
| `wrench_delay` | 开始施加的时间 |
| `wrench_duration` | 持续时间 |
| `wrench_output` | 保存实际调用和施加结果的 CSV |

这个接口提供的是“仿真中的外部扰动真值”，用于判断网络有没有估计到正确的方向和大小。

### 4.2 MyRBF 内部补偿接口

MyRBF 的输出是机身六维补偿：

```text
[Fx, Fy, Fz, Tx, Ty, Tz]
```

控制器不会直接把这个六维量当成关节力矩，而是经过：

1. 每个方向的补偿增益；
2. 每个方向的最大补偿量；
3. 每个方向的变化速度限制；
4. 当前支撑脚判断；
5. 接触位置和名义接触力计算；
6. 摩擦系数、最小法向力和单脚力增量检查；
7. 六维力/力矩到各支撑脚三维接触力的分配。

最后得到每只支撑脚的接触力增量，加到 MPC 原来的接触力上，再交给 WBC。

当前有两条补偿路径：

- `control_interface: wrench_allocator`：当前 V4、Robust MyRBF 和 V5-A 使用的六维路径；
- 旧的 Fy-only 路径：用于复现早期侧向力结果，不具备完整六维分配能力。

## 5. 一场实验的完整流程

```text
启动 myrbf_shadow_gazebo.launch
          ↓
Gazebo 加载 Go2 和控制器
          ↓
MPC 产生当前状态、接触力和下一步预测
          ↓
MyRBF 对比真实状态与一步预测
          ↓
六个子网络更新并给出六维估计
          ↓
补偿限幅、限速、接触和摩擦检查
          ↓
修改 MPC 接触力输入
          ↓
WBC 根据修改后的输入计算关节力矩
          ↓
Gazebo 执行并记录四类 CSV
          ↓
分析脚本检查时间、外力和恢复指标
```

需要注意：MyRBF 是在线更新的，不是先离线训练再回放。每个控制周期利用新到达的真实状态更新网络权值。

## 6. 在 Docker 中运行

以下命令在现有 `legged-robot-noetic` 容器内执行：

```bash
source /opt/ros/noetic/setup.bash
cd /tmp/myrbf_go2_catkin
catkin build legged_interface legged_controllers legged_gazebo legged_unitree_description
source devel/setup.bash
export ROBOT_TYPE=go2
```

先做无扰动低速行走：

```bash
roslaunch legged_controllers myrbf_shadow_gazebo.launch gui:=true duration:=15 enable_shadow:=true enable_control_injection:=true myrbf_config:=myrbf_shadow_v4_go2_robust.yaml enable_motion:=true motion_vx:=0.08 motion_duration:=9 enable_wrench:=false
```

运行 Baseline 时使用：

```bash
enable_control_injection:=false myrbf_config:=myrbf_shadow_baseline.yaml
```

运行时建议显式指定输出路径，避免覆盖上一场实验：

```bash
state_output:=/tmp/myrbf_case/state.csv shadow_output:=/tmp/myrbf_case/shadow.csv wrench_output:=/tmp/myrbf_case/wrench.csv motion_output:=/tmp/myrbf_case/motion.csv
```

## 7. 三类典型实验

### 7.1 无扰动行走

只打开行走，不打开外力：

```bash
enable_motion:=true motion_vx:=0.08 motion_delay:=4 motion_duration:=9 enable_wrench:=false
```

用于确认机器人、控制器和网络在正常行走下不会自己发散。

### 7.2 等效负重

先用向下的 `Fz` 模拟额外重量：

```bash
enable_motion:=true motion_vx:=0.08 motion_duration:=9 enable_wrench:=true wrench_axis:=fz wrench_profile:=constant wrench_value:=-39.2 wrench_delay:=6 wrench_duration:=6
```

`-39.2 N` 约等效于额外 4 kg 的重力，但没有改变机器人质量分布和转动惯量。因此它是第一阶段的负重近似，不等于真实挂载重物。

### 7.3 侧向或六维扰动

侧向恒力示例：

```bash
enable_motion:=true motion_vx:=0.08 motion_duration:=9 enable_wrench:=true wrench_axis:=fy wrench_profile:=constant wrench_value:=20 wrench_delay:=6 wrench_duration:=6
```

多频侧向力示例：

```bash
enable_motion:=true motion_vx:=0.08 motion_duration:=9 enable_wrench:=true wrench_axis:=fy wrench_profile:=multisine wrench_amplitude:=10 wrench_frequency:=0.22 wrench_secondary_amplitude:=6 wrench_secondary_frequency:=0.77 wrench_delay:=6 wrench_duration:=6
```

同一轮对比时，除了方法配置外，速度、持续时间、随机种子和外力参数都应保持不变。

## 8. V5-A 批量实验

批量脚本把三类场景、五个随机种子和 V4/V5-A 两种方法组合起来，共生成 30 场：

```bash
python3 legged_controllers/scripts/run_myrbf_v5a_paired_campaign.py --dry-run
```

`--dry-run` 只生成命令，不启动仿真。正式执行必须提供冻结的实验协议、源码提交号和源码锁文件，避免实验中途改变参数或代码。完成后可运行：

```bash
python3 legged_controllers/scripts/analyze_myrbf_v5a_paired.py /tmp/myrbf_v5a_repeat_20260901
python3 scripts/check_myrbf_v5a_repeat_report.py --root /tmp/myrbf_v5a_repeat_20260901 --worktree /tmp/myrbf_final_verify_src_20260901
```

## 9. 结果检查顺序

1. 看 `state.csv`：是否覆盖完整时间，机器人是否提前停止；
2. 看 `motion.csv`：行走命令是否在计划时间开始和结束；
3. 看 `wrench.csv`：外力是否真的成功施加；
4. 看 `shadow.csv`：估计、残差和补偿是否有非有限值、突跳或饱和；
5. 最后查看分析脚本生成的汇总和曲线。

如果安全检查触发，先保留这场记录并查明原因，不要直接删除后重跑。

## 10. 当前能力边界

- 当前已验证的是 Gazebo 控制链路；
- 当前地形是平地，地形法向和高度还没有接入在线地形估计；
- 当前负重实验主要是向下外力近似，不包含真实负载惯量变化；
- 真实 Go2、复杂多地形和实物负重实验需要单独的安全检查和实验方案。
