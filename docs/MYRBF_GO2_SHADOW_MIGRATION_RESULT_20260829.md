# MyRBF 迁移到 Go2：影子模式验证记录

日期：2026-08-29  
状态：影子估计、三方法对比、Go2 横向参数校准和有限闭环检查均已完成。MyRBF 的估计优势明显；闭环改善目前约 3%，只能说明方向有效，还不能宣称控制性能有显著提升。

## 1. 本轮完成了什么

本轮把现有 MyRBF v3 从 ANYmal 实验代码迁入 Go2 的 Gazebo、NMPC 和 WBC 数据链。迁移使用独立 Git 工作树，基于已验证能稳定站立的 Go2 基线提交 `7d87e51`，分支为 `codex/myrbf-go2-baseline-port`，没有修改老师的原目录。

现在每个控制周期都会取得：

1. Go2 当前实际状态；
2. 上一周期 MPC 给出的一步预测状态；
3. 两者时间对齐后的六维运动残差；
4. MyRBF 六个子网络给出的六维补偿估计；
5. 机身位置、姿态、六维估计、有效样本数和拒绝样本数。

当前估计结果只发布和记录，不送入 MPC、WBC 或关节命令。这样可以先验证输入、时间对齐、噪声和估计方向，而不会让尚未按 Go2 调好的参数影响机器人。

## 2. 数据链与符号

```text
上一周期 MPC 一步预测 ─┐
                       ├─ 时间对齐 ─ 六维运动残差 ─ 滤波 ─ MyRBF ─ 六维补偿估计 ─ 只记录
当前 Go2 实际状态 ─────┘
```

MyRBF 仍是六个互相独立的子网络，分别输出 Fx、Fy、Fz、Tx、Ty、Tz。每个子网络使用两个输入：本方向的运动残差，以及该残差中尚未被当前网络解释的部分。每个子网络有 25 个高斯节点和 1 个常数节点，运行时只更新最后一层权值。

必须注意符号：这里输出的是“为了抵消外界扰动而需要施加的补偿”，不是力传感器读到的外力。外界施加 `+20 N Fy` 时，理想补偿是 `-20 N Fy`。实验图同时画出了外力、理想反向补偿和网络输出，避免误读。

## 3. 代码位置

| 功能 | 文件 |
|---|---|
| 六维残差和时间对齐 | `legged_interface/include/legged_interface/adaptive/MyRbfPredictionResidualObserver.h`、`legged_interface/src/adaptive/MyRbfPredictionResidualObserver.cpp` |
| 六子网络与在线更新 | `legged_interface/include/legged_interface/adaptive/MyRbfShadowEstimator.h`、`legged_interface/src/adaptive/MyRbfShadowEstimator.cpp` |
| 完整影子通道 | `legged_interface/include/legged_interface/adaptive/MyRbfShadowPipeline.h`、`legged_interface/src/adaptive/MyRbfShadowPipeline.cpp` |
| ETHRBF 跟踪误差 | `legged_interface/include/legged_interface/adaptive/MyRbfTrackingError.h`、`legged_interface/src/adaptive/MyRbfTrackingError.cpp` |
| Fy 有限闭环保护 | `legged_interface/include/legged_interface/adaptive/MyRbfFyCompensation.h`、`legged_interface/src/adaptive/MyRbfFyCompensation.cpp` |
| 接入 Go2 控制周期 | `legged_controllers/include/legged_controllers/LeggedController.h`、`legged_controllers/src/LeggedController.cpp` |
| Baseline、ETHRBF、原 MyRBF、Go2 校准参数 | `legged_controllers/config/go2/myrbf_shadow_baseline.yaml`、`myrbf_shadow_eth.yaml`、`myrbf_shadow_v3.yaml`、`myrbf_shadow_v3_go2.yaml` |
| 一键实验入口 | `legged_controllers/launch/myrbf_shadow_gazebo.launch` |
| 稳定启动 Go2 | `legged_controllers/scripts/unpause_when_controller_ready.py` |
| 恒定与正弦扰动 | `legged_controllers/scripts/apply_myrbf_wrench.py` |
| 状态和估计记录 | `legged_controllers/scripts/record_stage8c.py`、`legged_controllers/scripts/record_myrbf_shadow.py` |
| 三方法指标和作图 | `legged_controllers/scripts/analyze_go2_three_method_shadow.py` |
| 横向参数标定 | `legged_controllers/scripts/calibrate_myrbf_fy_gain.py` |
| 闭环指标和作图 | `legged_controllers/scripts/analyze_go2_closedloop.py` |
| 单元检查 | `legged_interface/test/adaptive/`、`legged_controllers/test/` |

## 4. Docker 和运行方法

Docker 启动程序：

```powershell
& 'C:\Users\lenovo\AppData\Local\Programs\DockerDesktop\Docker Desktop.exe'
```

使用的现有容器：`legged-robot-noetic`。临时编译工作区：`/tmp/myrbf_go2_catkin`。

容器中进入环境：

```bash
source /opt/ros/noetic/setup.bash
source /tmp/myrbf_go2_catkin/devel/setup.bash
```

无扰动影子检查：

```bash
roslaunch legged_controllers myrbf_shadow_gazebo.launch \
  gui:=false duration:=10 enable_shadow:=true enable_wrench:=false
```

20 N 恒定横向扰动：

```bash
roslaunch legged_controllers myrbf_shadow_gazebo.launch \
  gui:=false duration:=14 enable_shadow:=true enable_wrench:=true \
  wrench_profile:=constant wrench_axis:=fy wrench_value:=20 \
  wrench_delay:=5 wrench_duration:=4
```

15 N、0.5 Hz 正弦横向扰动：

```bash
roslaunch legged_controllers myrbf_shadow_gazebo.launch \
  gui:=false duration:=15 enable_shadow:=true enable_wrench:=true \
  wrench_profile:=sine wrench_axis:=fy wrench_value:=0 \
  wrench_amplitude:=15 wrench_frequency:=0.5 \
  wrench_delay:=5 wrench_duration:=6
```

扰动程序会等到 Go2 模型真正生成后再开始计时。控制器也会在关节初始姿态设置完成后再解除仿真暂停，避免启动竞态导致机器人落地。

## 5. 实际验证结果

以下均为 Gazebo 中真实跑出的单次受限验证，不是离线伪造数据。

| 场景 | 是否站稳 | 主要结果 |
|---|---:|---|
| G0 原 Go2 基线，无 MyRBF | 是 | 最终机身高度 0.34384 m，最大横滚 0.325° |
| G1 MyRBF 影子、无扰动 | 是 | 最终高度 0.34385 m，与 G0 基本一致，说明影子通道没有改变控制 |
| G2 `+20 N Fy` 恒定 4 s | 是 | 理想补偿 -20 N；平均估计 -14.20 N，峰值 -18.17 N；最大横向偏移 3.80 cm；撤力后恢复 |
| G3 `±15 N Fy`、0.5 Hz 正弦 6 s | 是 | 补偿波形相关性 0.821；对齐约 0.16 s 延迟后相关性 0.961；最大横向偏移 2.10 cm |

扰动调用成功率两组均为 100%。四个场景均未触发安全检查，也没有倒地。

无扰动时 Fy 输出的均方根约为 0.80 N，峰值约 0.97 N；Fz 的均方根约为 3.67 N。这说明横向通道已经具备可用信噪比，但竖直方向仍含较明显的模型与接触偏差，需要按 Go2 重新标定。

正式数据与图位于：

- `results/G0_baseline/`
- `results/G1_shadow_no_disturbance/`
- `results/G2_constant_20N_fy_final/`
- `results/G3_sine_15N_fy_final/`
- `results/analysis_final/`

## 6. 现在能说明什么，不能说明什么

已经能说明：

- MyRBF 已经接到 Go2 Gazebo 的真实控制数据，而不是继续使用 ANYmal 离线接口；
- 当前状态与上一周期 MPC 一步预测可以稳定形成六维输入；
- 恒定外力下补偿方向正确，并能逐步接近目标；
- 正弦外力下能跟随主要波形，机器人在整个实验中保持站立；
- 影子模式与原基线的无扰动状态几乎一致，迁移没有偷偷改变控制输出。

尚不能说明：

- MyRBF 已经让 Go2 比 Baseline 或 ETHRBF 更稳定；当前它没有参与控制；
- 当前 ANYmal 参数已经是 Go2 最优参数；
- 行走、多地形、负重行走和真实 Go2 已经验证；
- 六个方向都达到同样精度；目前只正式施加了 Fy 扰动，Fz 还需要重点校准。

## 7. 第二阶段：三方法同场景对比

三种方法使用相同的 Go2、控制器、扰动和记录接口，每个条件只运行一次：

| 方法 | 恒定 +20 N 估计误差 (N) | 正弦 ±15 N 估计误差 (N) | 正弦波形相关性 |
|---|---:|---:|---:|
| Baseline | 20.00 | 10.62 | — |
| ETHRBF | 11.51 | 9.41 | 0.723 |
| MyRBF（Go2 校准） | 2.65 | 1.92 | 0.992 |

三种方法在这组对比中都只观察，不参与控制。因此这张表能说明 MyRBF 更接近实际扰动，但不能用三条几乎重合的机身曲线声称 MyRBF 已改善控制。

ETHRBF 使用目标与实际运动的差来更新；MyRBF 使用动力学残差与当前估计之间尚未解释的差来更新。Go2 校准只改变了 Fy 的学习速度，由原来的约 0.001 调整为 0.05；选择的是离线重放中接近最优、同时更保守的值，随后又用两次完整 Gazebo 运行确认。

正式结果位于：

- `experiments/go2_three_method_shadow_20260829/analysis/RESULT_SUMMARY.md`
- `experiments/go2_three_method_shadow_20260829/analysis/01_constant_estimation_comparison.png`
- `experiments/go2_three_method_shadow_20260829/analysis/02_sine_estimation_comparison.png`
- `experiments/go2_three_method_shadow_20260829/analysis/03_constant_target_vs_actual.png`
- `experiments/go2_three_method_shadow_20260829/analysis/04_sine_target_vs_actual.png`

## 8. 第三阶段：有限闭环检查

闭环默认关闭。打开时只允许 Fy 一个方向进入足端接触力，并同时限制补偿比例、最大幅值和每秒变化量。先做无扰动，再做 +20 N 恒定扰动，最后才做 ±15 N 正弦扰动。

| 条件 | 最大横向偏移 (cm) | 横向均方根偏移 (cm) | 最大机身横滚 (deg) | 最低高度 (m) |
|---|---:|---:|---:|---:|
| MyRBF 只观察 | 3.514 | 3.238 | 1.595 | 0.343 |
| 闭环，较小限制 | 3.452 | 3.190 | 1.656 | 0.343 |
| 闭环，中等限制 | 3.396 | 3.130 | 1.716 | 0.343 |

中等限制下，恒定扰动的最大横向偏移改善 3.4%，正弦扰动改善 3.5%。机器人全程保持站立，高度没有恶化；正弦波形也继续被正确估计。但最大机身横滚分别增加约 7.6% 和 5.8%。因此结果只证明补偿已经真正进入闭环且方向正确，不能证明整体稳定性更好，也不应继续简单放大补偿。

正式闭环结果位于：

- `experiments/go2_myrbf_closedloop_20260829/analysis/RESULT_SUMMARY.md`
- `experiments/go2_myrbf_closedloop_20260829/analysis/01_closedloop_constant_comparison.png`
- `experiments/go2_myrbf_closedloop_20260829/analysis/02_closedloop_metric_summary.png`
- `experiments/go2_myrbf_closedloop_20260829/analysis/03_closedloop_sine_comparison.png`

受限闭环启动示例：

```bash
roslaunch legged_controllers myrbf_shadow_gazebo.launch \
  gui:=false duration:=15 enable_shadow:=true \
  myrbf_config:=myrbf_shadow_v3_go2.yaml \
  enable_control_injection:=true control_gain:=0.25 \
  control_force_limit:=6.0 control_rate_limit:=10.0 \
  enable_wrench:=true wrench_profile:=sine wrench_axis:=fy \
  wrench_amplitude:=15 wrench_frequency:=0.5 \
  wrench_delay:=5 wrench_duration:=6
```

## 9. 当前结论边界与下一步

现在可以确认：MyRBF 已经从 ANYmal 接到 Go2 Gazebo；Baseline、ETHRBF 和 MyRBF 能在相同条件下运行；MyRBF 的横向估计明显优于另外两种；补偿已经通过严格限制接入真实控制链并保持稳定。

现在仍不能确认：六维方向都已经标定；行走、负重、多地形和真实 Go2 已通过；当前约 3% 的闭环改善具有重复统计意义。

下一步不建议继续简单放大补偿。应先把高层固定参考与 MPC 内部短期目标分开记录，再进行平地行走负重、行走周期扰动和不同地形的单次门禁。通过后再做重复实验和统计，最后才进入真机低速验证。
