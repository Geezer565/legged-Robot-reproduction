# Go2 MyRBF 实验、迁移与代码交付报告

日期：2026-08-30

代码分支：`codex/myrbf-go2-baseline-port`

迁移起点：`7d87e51`，老师原有的 Go2 NMPC-WBC 可运行版本

当前提交：`c225677`

## 1. 先给结论

MyRBF 已经成功迁移到 Go2 的 Gazebo 闭环控制中。当前代码不再使用 ANYmal 的实验接口，也不是仅用离线数据模拟网络输出；它直接读取 Go2 控制器中的实际状态和 MPC 一步预测，在线更新六个 RBF 子网络，并把六维补偿经过限制后分配到支撑脚，参与全身控制。

目前已经完成的是 **Go2 Gazebo 仿真迁移**，还不是实际 Go2 机器验证。当前补偿位于 MPC 输出和全身控制之间，MPC 会在下一周期看到补偿产生的运动结果；但补偿还没有直接写入 MPC 预测时域的动力学方程。因此准确名称是“Go2 上的 MyRBF-MPC/WBC 闭环接口”，不应写成“已经完成 MPC 内部自适应模型改造”。

## 2. 当前可以做哪些实验

### 2.1 扰动形式

现有扰动工具可以对机身六个方向分别施加：

- 三个力方向：`Fx`、`Fy`、`Fz`；
- 三个力矩方向：`Tx`、`Ty`、`Tz`；
- 恒定扰动；
- 单一正弦扰动；
- 两个频率叠加的多频扰动；
- 固定时序的不规则正负切换扰动。

这些扰动可以在站立或低速行走时施加，并且每次使用同一时间表，适合做方法间的公平对比。

### 2.2 当前可以运行的实验矩阵

| 类别 | 机器人状态 | 扰动 | 主要用途 | 当前验证情况 |
| --- | --- | --- | --- | --- |
| 安全基线 | 站立或低速行走 | 无 | 检查接入后是否自行漂移或失稳 | 已验证 |
| 恒定侧向力 | 站立 | `Fy` 恒力 | 检查持续外力估计和稳态偏移 | 已验证 |
| 周期侧向力 | 站立 | `Fy` 正弦 | 检查周期跟随 | 已验证 |
| 复杂侧向力 | 站立或行走 | `Fy` 多频或不规则 | 检查复杂扰动和行走稳定性 | 已正式对比 |
| 等效负重 | 低速行走 | `Fz` 持续向下 | 检查额外重量下的高度和姿态 | 已正式对比 |
| 侧倾力矩 | 站立 | `Tx` 正弦或多频 | 检查力矩估计和抗侧倾能力 | 已正式对比 |
| 其余方向 | 站立或行走 | `Fx`、`Ty`、`Tz` | 检查六维完整性 | 接口可用，尚未正式验证 |
| 多地形 | 行走 | 坡面、碎石、台阶等 | 检查接触变化和迁移能力 | 尚未接入地形场景 |
| 真机负重 | 真实 Go2 | 实际重物 | 检查真实传感、时延和惯量变化 | 尚未进行 |

### 2.3 四种可直接对比的方法

| 方法 | 配置文件 | 含义 |
| --- | --- | --- |
| Baseline | [`myrbf_shadow_baseline.yaml`](../legged_controllers/config/go2/myrbf_shadow_baseline.yaml) | 原 Go2 控制器，不使用自适应补偿 |
| ETHRBF | [`myrbf_shadow_eth_go2_closed_loop.yaml`](../legged_controllers/config/go2/myrbf_shadow_eth_go2_closed_loop.yaml) | 根据机器人跟踪误差更新权值 |
| OriginalV4 | [`myrbf_shadow_v4_go2.yaml`](../legged_controllers/config/go2/myrbf_shadow_v4_go2.yaml) | 根据预测残差更新，使用原保守参数 |
| RobustMyRBF | [`myrbf_shadow_v4_go2_robust.yaml`](../legged_controllers/config/go2/myrbf_shadow_v4_go2_robust.yaml) | 根据预测残差更新，使用本轮锁定参数 |

四种方法共用同一个 Go2 模型、MPC、全身控制器、步态、运动命令、扰动脚本和记录方式。ETHRBF 与 RobustMyRBF 的网络表示、控制入口和安全限制相同，主要区别是用什么误差信号更新网络。

## 3. 已完成的正式对比

### 3.1 行走时多频侧向扰动

在低速行走时施加两个新频率叠加的侧向力。RobustMyRBF 相对 Baseline：

- 机身侧移均方根降低 35.8%；
- 机身侧倾均方根降低 29.5%；
- 最大侧移从 4.528 cm 降到 2.808 cm。

相对 OriginalV4，机身侧移和侧倾分别降低 23.1% 和 21.5%。这一组中 OriginalV4 的力估计数字略好，但 RobustMyRBF 的机器人运动更稳定，说明最终参数是按照闭环效果选择的。

### 3.2 行走时等效 4 kg 负重

通过持续向下施加 39.2 N 模拟额外 4 kg 重量。RobustMyRBF 相对 Baseline：

- 最低机身高度提高约 7.9 mm；
- 高度波动降低 29.0%；
- 最大侧倾降低 50.5%。

相对 OriginalV4，负重估计误差降低 45.8%。这一场只模拟额外重力，没有模拟真实重物引起的质量分布和转动惯量变化，因此真机前还需要真实负载模型或实物负重试验。

### 3.3 站立时多频侧倾力矩

RobustMyRBF 相对 Baseline：

- 侧倾均方根降低 12.8%；
- 最大侧倾降低 14.0%；
- 力矩波形相关性达到 0.804。

力矩方向已经获得稳定改善，但提升小于侧向力和负重方向，应作为后续继续优化的方向。

完整数字、调参过程和三张正式图见：

- [`results/go2_method_compare_20260830/EXPERIMENT_REPORT.md`](../results/go2_method_compare_20260830/EXPERIMENT_REPORT.md)
- [`MYRBF_GO2_TUNING_AND_COMPARISON_20260830.md`](MYRBF_GO2_TUNING_AND_COMPARISON_20260830.md)
- [`results/go2_method_compare_20260830/RESULT_SUMMARY.md`](../results/go2_method_compare_20260830/RESULT_SUMMARY.md)

## 4. MyRBF 是怎样迁移到 Go2 的

### 4.1 原有 Go2 控制链

老师原来的代码使用以下控制顺序：

```text
状态估计 → MPC 计算期望运动和接触力 → 全身控制器 → 关节力矩 → Gazebo/机器人
```

迁移没有替换 OCS2、Pinocchio、接触模型或原来的 MPC。MyRBF 作为单独模块接到老师已有控制器中。

### 4.2 迁移后的控制链

```text
当前实际状态 ───────────────┐
                            ├─ 与上一周期 MPC 一步预测比较
上一周期 MPC 一步预测 ──────┘
                                      ↓
                           六维运动残差与未解释残差
                                      ↓
                      六个独立 RBF 子网络在线更新
                                      ↓
                          Fx Fy Fz Tx Ty Tz 补偿估计
                                      ↓
                         限幅、限速、接触与摩擦检查
                                      ↓
                    分配到当前支撑脚的三维接触力增量
                                      ↓
                  MPC 原接触力 + MyRBF 接触力增量
                                      ↓
                               全身控制器
```

### 4.3 一步预测和残差

控制器每个更新周期保存 MPC 对下一个短时刻的状态预测。下一周期真实状态到达后，将两者对齐并计算六维运动差异。观测器还会减去上一个周期已经施加的补偿，避免网络把自己的控制作用再次当成未知外力。

相关位置：

- 控制器主循环：[`LeggedController.cpp`](../legged_controllers/src/LeggedController.cpp#L307)
- 读取实际状态并更新网络：[`LeggedController.cpp`](../legged_controllers/src/LeggedController.cpp#L358)
- 保存下一步 MPC 预测：[`LeggedController.cpp`](../legged_controllers/src/LeggedController.cpp#L398)
- 预测与实际状态对齐：[`MyRbfPredictionResidualObserver.cpp`](../legged_interface/src/adaptive/MyRbfPredictionResidualObserver.cpp#L41)
- 计算预测残差：[`MyRbfPredictionResidualObserver.cpp`](../legged_interface/src/adaptive/MyRbfPredictionResidualObserver.cpp#L59)

### 4.4 六个 RBF 子网络

网络分别输出 `Fx`、`Fy`、`Fz`、`Tx`、`Ty`、`Tz`。每个子网络使用本方向的两个输入：

1. 实际运动与上一周期 MPC 一步预测之间的差异；
2. 减去当前网络估计后仍未被解释的部分。

每个方向使用 25 个高斯节点和 1 个常数节点。中心和宽度固定，运行时只更新输出权值。

相关位置：

- 创建六个网络和节点网格：[`MyRbfShadowEstimator.cpp`](../legged_interface/src/adaptive/MyRbfShadowEstimator.cpp#L34)
- 计算输出和更新权值：[`MyRbfShadowEstimator.cpp`](../legged_interface/src/adaptive/MyRbfShadowEstimator.cpp#L137)
- 串联观测器与网络：[`MyRbfShadowPipeline.cpp`](../legged_interface/src/adaptive/MyRbfShadowPipeline.cpp#L18)
- ETH 跟踪误差：[`MyRbfTrackingError.cpp`](../legged_interface/src/adaptive/MyRbfTrackingError.cpp#L30)

### 4.5 怎样进入 Go2 控制

网络输出不会直接无条件加到关节命令。控制器先对六个方向分别限制作用比例、最大值和变化速度，再根据当前支撑脚位置，把机身所需的力和力矩联合分配为每只支撑脚的接触力增量。分配过程检查：

- 当前脚是否接触地面；
- 摩擦限制；
- 最小法向支撑力；
- 单脚允许增加的最大力；
- 六维目标是否能够同时实现。

如果请求过大，会整体缩小；如果计算异常，会停止本周期补偿。

相关位置：

- 选择旧侧向接口或六维接口：[`LeggedController.cpp`](../legged_controllers/src/LeggedController.cpp#L428)
- 六维补偿进入 Go2 接触力：[`LeggedController.cpp`](../legged_controllers/src/LeggedController.cpp#L479)
- 六维限幅和限速：[`MyRbfWrenchCompensation.cpp`](../legged_interface/src/adaptive/MyRbfWrenchCompensation.cpp#L44)
- 将机身六维目标分配到支撑脚：[`MyRbfWrenchCompensation.cpp`](../legged_interface/src/adaptive/MyRbfWrenchCompensation.cpp#L73)

## 5. 代码文件地图

| 功能 | 主要文件 |
| --- | --- |
| Go2 控制器接入 | [`LeggedController.h`](../legged_controllers/include/legged_controllers/LeggedController.h)、[`LeggedController.cpp`](../legged_controllers/src/LeggedController.cpp) |
| 六维数据类型 | [`AdaptiveTypes.h`](../legged_interface/include/legged_interface/adaptive/AdaptiveTypes.h) |
| 一步预测残差 | [`MyRbfPredictionResidualObserver.h`](../legged_interface/include/legged_interface/adaptive/MyRbfPredictionResidualObserver.h)、[`MyRbfPredictionResidualObserver.cpp`](../legged_interface/src/adaptive/MyRbfPredictionResidualObserver.cpp) |
| 六个 RBF 网络 | [`MyRbfShadowEstimator.h`](../legged_interface/include/legged_interface/adaptive/MyRbfShadowEstimator.h)、[`MyRbfShadowEstimator.cpp`](../legged_interface/src/adaptive/MyRbfShadowEstimator.cpp) |
| 网络与观测器流程 | [`MyRbfShadowPipeline.h`](../legged_interface/include/legged_interface/adaptive/MyRbfShadowPipeline.h)、[`MyRbfShadowPipeline.cpp`](../legged_interface/src/adaptive/MyRbfShadowPipeline.cpp) |
| ETH 跟踪误差 | [`MyRbfTrackingError.h`](../legged_interface/include/legged_interface/adaptive/MyRbfTrackingError.h)、[`MyRbfTrackingError.cpp`](../legged_interface/src/adaptive/MyRbfTrackingError.cpp) |
| 六维补偿安全分配 | [`MyRbfWrenchCompensation.h`](../legged_interface/include/legged_interface/adaptive/MyRbfWrenchCompensation.h)、[`MyRbfWrenchCompensation.cpp`](../legged_interface/src/adaptive/MyRbfWrenchCompensation.cpp) |
| 一键 Gazebo 入口 | [`myrbf_shadow_gazebo.launch`](../legged_controllers/launch/myrbf_shadow_gazebo.launch) |
| 六维扰动脚本 | [`apply_myrbf_wrench.py`](../legged_controllers/scripts/apply_myrbf_wrench.py) |
| 行走命令脚本 | [`publish_myrbf_motion.py`](../legged_controllers/scripts/publish_myrbf_motion.py) |
| 状态和网络记录 | [`record_stage8c.py`](../legged_controllers/scripts/record_stage8c.py)、[`record_myrbf_shadow.py`](../legged_controllers/scripts/record_myrbf_shadow.py) |
| 正式统计与画图 | [`analyze_myrbf_campaign.py`](../legged_controllers/scripts/analyze_myrbf_campaign.py) |
| 自动检查 | [`legged_interface/test/adaptive`](../legged_interface/test/adaptive)、[`legged_controllers/test`](../legged_controllers/test) |

## 6. 怎样把交付代码放进老师原工作区

交付压缩包不是一个新的 ROS 环境，也不包含 OCS2、Pinocchio、hpp-fcl、Gazebo 等底层依赖。它必须覆盖到老师原有、已经能运行 Go2 的仓库中。

老师原仓库根目录记为：

```text
<teacher_ocs2_ws>/src/leggedcontrol_go2
```

有两种安装方式。

### 方式 A：源码覆盖

将压缩包中的 `source_overlay` 内容复制到上述仓库根目录，保持目录结构不变。覆盖前先建立新分支或备份当前改动。

### 方式 B：补丁安装

在老师原仓库根目录执行：

```bash
git apply --check patches/go2_myrbf_source.patch
git apply patches/go2_myrbf_source.patch
```

如果 `git apply --check` 不通过，不要强行覆盖，说明老师当前版本已经与迁移起点不同，需要逐文件合并。

完成覆盖后，在原 Docker 中执行：

```bash
cd <teacher_ocs2_ws>
source /opt/ros/noetic/setup.bash
catkin build legged_interface legged_controllers legged_gazebo legged_unitree_description
source devel/setup.bash
export ROBOT_TYPE=go2
```

## 7. 最小运行示例

以下命令均在老师原 Docker 中运行。

### 7.1 无扰动低速行走安全检查

```bash
roslaunch legged_controllers myrbf_shadow_gazebo.launch \
  gui:=true duration:=15 \
  enable_shadow:=true enable_control_injection:=true \
  myrbf_config:=myrbf_shadow_v4_go2_robust.yaml \
  enable_motion:=true motion_vx:=0.08 motion_duration:=9 \
  enable_wrench:=false
```

### 7.2 行走时等效 4 kg 负重

```bash
roslaunch legged_controllers myrbf_shadow_gazebo.launch \
  gui:=true duration:=15 \
  enable_shadow:=true enable_control_injection:=true \
  myrbf_config:=myrbf_shadow_v4_go2_robust.yaml \
  enable_motion:=true motion_vx:=0.08 motion_duration:=9 \
  enable_wrench:=true wrench_axis:=fz wrench_profile:=constant \
  wrench_value:=-39.2 wrench_delay:=6 wrench_duration:=6
```

### 7.3 行走时多频侧向扰动

```bash
roslaunch legged_controllers myrbf_shadow_gazebo.launch \
  gui:=true duration:=15 \
  enable_shadow:=true enable_control_injection:=true \
  myrbf_config:=myrbf_shadow_v4_go2_robust.yaml \
  enable_motion:=true motion_vx:=0.08 motion_duration:=9 \
  enable_wrench:=true wrench_axis:=fy wrench_profile:=multisine \
  wrench_value:=0 wrench_amplitude:=10 wrench_frequency:=0.22 \
  wrench_secondary_amplitude:=6 wrench_secondary_frequency:=0.77 \
  wrench_delay:=6 wrench_duration:=6
```

### 7.4 切换对比方法

保持其他参数完全不变，只切换以下两项：

| 方法 | `enable_control_injection` | `myrbf_config` |
| --- | --- | --- |
| Baseline | `false` | `myrbf_shadow_baseline.yaml` |
| ETHRBF | `true` | `myrbf_shadow_eth_go2_closed_loop.yaml` |
| OriginalV4 | `true` | `myrbf_shadow_v4_go2.yaml` |
| RobustMyRBF | `true` | `myrbf_shadow_v4_go2_robust.yaml` |

## 8. 队友接手后建议的实验顺序

1. 先运行无扰动站立和低速行走，确认环境、模型和控制器一致；
2. 复跑 V1、V2、V3，各方法先只跑一次；
3. 固定参数后，对关键场景做多次重复；
4. 增加坡面、不同摩擦、台阶和松散地面；
5. 用真实质量与惯量修改模型，替代单纯向下施力；
6. 最后才进入真机：先只记录，再小比例补偿，再低速负重和周期扰动。

真机默认必须关闭补偿，确认急停、通信延迟、状态估计和接触检测正常后才能逐步打开。

## 9. 交付包说明

交付压缩包包含：

- `source_overlay`：需要覆盖到老师原 Go2 仓库的源码；
- `patches/go2_myrbf_source.patch`：相对于迁移起点 `7d87e51` 的源码补丁；
- `docs`：本报告、调参报告和控制接口说明；
- `results`：正式结果摘要、三张 PNG/PDF 图和自动统计；
- `MANIFEST.sha256`：所有交付文件的完整性校验值。

交付包不包含底层依赖，但已经包含 V1–V3 十二场正式对比的原始记录、图和报告。活动工作区只保留正式报告与图；早期接口检查、调参扫描和中间实验记录已经清理。
