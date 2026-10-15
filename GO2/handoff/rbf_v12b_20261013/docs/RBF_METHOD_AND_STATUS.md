# RBF V1.1 → V1.2-A/B：源码核对笔记

## 一、变量定义与模型结构

`legged_interface/include/legged_interface/adaptive/MyRbfV11Core.h`：`Weights = Matrix<double,6,26>`。
每个通道 `i∈{Fx,Fy,Fz,Tx,Ty,Tz}` 有 25 个高斯基函数和一个固定常数基函数，输出 `d_hat_i = sum_{j=0}^{25} W_{i,j} phi_{i,j}`，其中 `phi_{i,25}=1`；无额外输出缩放系数。

- `Ee=v_measured-v_predicted`，`Er=v_measured-v_reference`。
- `Ec=ratio*Ee+(1-ratio)*Er`，主配置 `ratio=0.8`。
- `G=mapping(mass,inertia,R)`，`q=G^T Ec`。
- `q_f`：6 通道一阶低通滤波 `q_f[k]=q_f[k-1]+dt/(tau_q+dt)*(q[k]-q_f[k-1])`；再经过 `v11_gradient_deadzone` 得到 `q_eff`。主配置 `tau_q=0.05s`、死区全零。
- RBF 基函数自变量：预测误差 `Ee` 的**20 点因果滑动平均**，经各通道 `v11_input_scale` 缩放并裁剪到 `[-1,1]`，不是 `Ec`。默认六独立输入，torque-coupled feature 已配置关闭。
- 25 Gaussian centers 在 `[-1,1]` 均匀分布；宽度主配置 `0.16`。

## 二、权重微分方程——以运行代码为准

对 `j=0..25`，代码实际表达为：

```text
Wdot[i,j] = - Gamma[i] * q_eff[i] * phi[i,j]
            - leakage[i] * W[i,j]
            - I(j==25) * enable_dynamics * k_d[i] * e_d[i]
```

该微分方程还受 `enabled` (warmup / prediction valid)、`dynamicsValid`、动力学滤波初始化等条件约束；积分后应用对应 projection：Gaussian 系数沿用 `v11_weight_limit`，常数 bias 使用 `v12_bias_weight_limit`。

代码位置：
- `MyRbfV11Core.h`：`Core::update()` 中 `out.gradient`、`filteredGradient_`、`out.derivative.row(i)` 与 `out.derivative(i,25)`。
- `MyRbfV11Control.cpp`：`configureMyRbfV11()` 参数读取、`updateMyRbfV11()` 误差和在线 dynamics reference 重构。

### V1.2 新增动力学误差的明确公式

由相邻两个控制时刻的观测值近似 `v_dot_measured = (v[k]-v[k-1])/dt`。

```text
d_rec = G_previous^{-1} * [v_dot_nominal_previous - v_dot_measured]
d_corrected = d_rec - d_nominal_dc            # 仅启用 V1.2-B DC cancellation 时
filtered_dynamics_ref = LPF(d_corrected)
e_d = d_hat_active_previous - filtered_dynamics_ref
```

然后仅对常数基函数索引 `25`：

```text
Wdot[i,25] += - k_d[i] * e_d[i]
```

关键细节：在 `MyRbfV11Control.cpp` 中传给 `Core::update()` 的是前一控制周期**实际生效的** `active->v11Disturbance`，不是当前候选估计。重构使用名义模型和测量运动；`ExactPayloadTruth.h`/`trueYpiWorldCom` 只用于独立诊断，未作为在线 teacher 输入。

**注意：这是常数偏置更新通道，不要笼统写成** `Wdot = -Gamma h q - k_d E_d` **对所有 RBF 权重加相同动力学更新。**

## 三、原始与现在参数对照（Git diff 源于 7884111）

| 参数 | 队友 GitHub 原始版本 | 当前主 YAML | 改动 |
|---|---|---|---|
| `v11_gamma` Fx,Fy,Fz | 22.8221449 | 684.664347 | 全部 30× |
| `v11_gamma` Tx | 0.00321391182 | 0.0964173546 | 30× |
| `v11_gamma` Ty | 0.0245852383 | 0.737557149 | 30× |
| `v11_gamma` Tz | 0.0257958114 | 0.773874342 | 30× |
| `v11_composite_ratio` | 0.9 | 0.8 | 改变复合误差权重 |
| `activation_warmup` | 1 s | 5 s | 延迟开始自适应学习 |
| `v11_widths` | 每维 0.12 | 每维 0.16 | 改变高斯核宽度 |
| `v11_gradient_filter_tau` | 无 | 每维 0.05 s | 新增梯度 LPF |
| `v11_leakage` | 无 | 全部为 0 | 保留实现，实际关断 |
| `v12_enable_dynamics_residual` | 无 | 主 YAML=false，050/080=true | 条件启用新增监督 |

因此性能变化不仅来自 Gamma，也可能受 ratio、宽度、warmup、滤波等共同影响；论文效果归因需消融对比。

## 四、V1.2-B 试验配置

`rbf_experiment_configs/myrbf_v12B_fz_bias_gain_050.yaml`：
- `v12_enable_dynamics_residual: true`
- `v12_dynamics_bias_gain=[0.05,0.05,0.50,0.05,0.10,0.05]`
- `v12_enable_nominal_dc_cancellation: true`

`rbf_experiment_configs/myrbf_v12B_fz_bias_gain_080.yaml`：仅 Fz 的动力学 bias 增益从 0.50 变成 0.80，其余相同。

共同的 dynamics reference LPF 时间常数 `[0.50,0.50,0.50,0.30,0.30,0.30]` s，bias 投影限幅 `[40,40,70,10,10,10]`；标定得到的 `v12_nominal_dc_residual=[-1.05418263700267,0.80738610392791,7.23600116141266,-0.0103322119562094,-0.0371705890384023,-0.0126735631891554]` （BODY / nominal CoM / left-hand）。

## 五、leakage 状态

主 RBF YAML 及 050/080 两份 V1.2-B YAML 的 `v11_leakage` 都是 `[0,0,0,0,0,0]`；不会通过 sigma-modification 抑制权重。历史 phase2 candidate 的 Fz leakage 是 0.02，未进入正式 overlay。`optional_not_tested/` 中提供全零的实验副本，但**不是已经验证的运行配置**。

## 六、实验进展与解释限制

根据先前实验聊天记录：`vx_ref=0.45 m/s`；-20 N RBF vx RMSE ≈0.0413 m/s；-25 N RBF vx RMSE ≈0.0730 m/s。-25 N teacher 原有 Fx RMSE ≈39.38 N，离线接触力修正后 ≈2.12 N，**但该修正未完成在线控制验证**。

以上指标来自先前交流记录，并非本交接包重新跑仿真获得；当前包也**未包含对应实验的 effective_rosparams.yaml**，不能将这些数字无条件归因于本包某一个 YAML。需要以各次实验保存的实际参数快照核对后才能宣称特定配置与指标一一对应。

## 七、仍待进行的验证

1. 在同一组建环境对交接 overlay 进行 ROS/OCS2 编译；从已有原仓库依赖补全源码。此包只提供增量，不是独立项目。
2. 明确分别使用主 YAML（dynamics OFF）与 050/080（dynamics ON）运行，并核对 `effective_rosparams.yaml`，确认学习确实开启。
3. 对齐时刻/坐标/单位后评价六维未知量 `Y_uπ_u` 与估计，并区分外力真值和以 nominal model 得到的重构 teacher。
4. 同条件开展 Pure MPC / ETH / RBF 对比；每轮 Gazebo 启动前检查并清理上一轮 ROS/Gazebo 残留进程。
5. 如要评估力矩通道的 torque-coupled feature，需单独改对应配置并验证，不能把该候选方案当当前基线。
