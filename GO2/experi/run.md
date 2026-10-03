# V1.1 恒速直行：运行与绘图

最新完整结果（2026-10-01）：[16 项记录与 8 张对比图](results/v1_1/20261001_comparison_delivery/README.md)。原整轮 13 项有效，另 3 项经启动排障补齐，不是一次性全部成功。各项条件相同，启动修复版本差别已在结果说明中列出。2026-10-03 交接清理保留这 16 份有效原始记录，旧失败原始大文件清理后仅留摘要；详见交接目录的 cleanup_provenance.json 和 cleanup_result.json。

更新：2026-09-29。当前只有一个运动设置：准备 3 s，第 2.4 s 切换 trot 步态，第 3–5 s 用三次平滑曲线加速到机身前向 0.45 m/s，随后保持至第 33 s 记录结束后停车。横向速度、转向角速度为 0，没有目标点、路径、距离限制或终点驻留；高度参考 0.33 m。

施力修正：Gazebo 世界坐标的净力/力矩使用服务参考点 `[0,0,0]`，净力矩关于 `go2::base` 连杆质心。不能将机身世界位置填进该参数，否则会产生额外的 `位置×力` 转矩。旧的非零参考点结果不进入新方法比较，清理后仅留故障来源摘要。

## 一次运行全部实验

Windows PowerShell：

```powershell
wsl.exe -d Ubuntu-20.04 -u root -- bash /mnt/d/GO2/experi/scripts/run_all.sh
```

已进入 WSL 时：

```bash
bash /mnt/d/GO2/experi/scripts/run_all.sh
```

总入口自动加载现有 ROS 环境。四种扰动 × V1.1 / ETH / Liu-VAN / Zhou-RFF，共 16 次。各条件新运行一次，不复用历史结果替代，不自动重试；失败保存后继续下一项，结束后统一汇总。主动中断或环境无法启动时停止。

只查看清单、不运行：

```bash
bash /mnt/d/GO2/experi/scripts/run_all.sh --plan
```

当前为用户批准的 V1.1 可行性修复候选：估计对象、RBF 输入和权值更新不变；原 MPC 硬下降要求改为代价 0.5·μ·max(0,−H_clf)²，μ 从 task.info 的 v11Feasibility.clfPenaltyWeight 读取并写入每项记录，当前 0.00001。不能称为 Word 严格原版，不保证每时刻满足下降条件。修复是否通过以新批次记录为准。

2026-09-30 固定增益按 Go2 模型校准为 `[22.8221449,22.8221449,22.8221449,0.00321391182,0.0245852383,0.0257958114]`，统一 `Gamma_i·||G[:,i]||²=0.1`；无在线变增益。旧恒力排障项属于中间结果；当前完整对比请使用页首的 16 项结果入口。

直行时四种方法统一保持第一次速度指令时的朝向；零转向不再把当前已经漂移的朝向当成新目标。只在本实验入口启用该选项，不更改其他手动控制入口的默认行为，也不增加位置路径规划。

总入口现在在运行结束后自动绘图，保存到本批次 figures_final。只有 16 项记录有效、四组各两张图的 PNG/SVG 全部导出，campaign_result.json 的 delivery_complete 才为 true；运行进程结束本身不代表交付完成。

## 四个小实验入口

每个入口默认运行四种方法，各一次：

| 条件 | 外力（世界系 Fy，正式记录全程持续） | 入口 |
|---|---|---|
| E00 无人工外力 | 0 | run_e00.sh |
| E01 恒力 | 5 N | run_e01.sh |
| E02 正弦 | 5 sin(2π·0.15t+π/2) N | run_e02.sh |
| E03 双正弦 | 3 sin(2π·0.15t+π/2)+2 sin(2π·0.30t+π/2) N | run_e03.sh |

例如只运行正弦组：

```bash
bash /mnt/d/GO2/experi/scripts/run_e02.sh
```

明确只运行 V1.1 时：

```bash
bash /mnt/d/GO2/experi/scripts/run_e02.sh --method v11_equivalent_wrench
```

修改全组共用速度或时长：

```bash
bash /mnt/d/GO2/experi/scripts/run_all.sh --vx 0.45 --duration 33 --motion-delay 3
```

--duration 是总记录时长，--motion-delay 是起步等待；运动持续到记录结束。速度是沿机身前方，不是世界坐标中固定方向的路径约束。当前不提供 T01/T02、指定距离或到点停止选项。

## 绘图

只画六维速度跟踪与六维等效不确定力/力矩。指令来自 /cmd_vel，控制器使用其现有的短时参考转换；速度图读取实际收到的参考，不用积分速度人为制造全程路径。世界系速度分量可能随机身姿态变化，不能直接把每个世界系分量都画成恒定值。

WSL 中，将批次目录名换成实际输出：

```bash
source /opt/ros/noetic/setup.bash
source /workspace/devel/setup.bash
PLOT=/workspace/src/leggedcontrol_go2/legged_controllers/scripts/plot_go2_method_comparison.py
BATCH=/mnt/d/GO2/experi/results/v1_1/批次目录名
python3 "$PLOT" --case-index "$BATCH/campaign_case_index.json" \
  --output "$BATCH/figures_final" --dpi 300
```

输出目录必须尚不存在。缺少有效记录或目标数据时不补零作图；人工施加外力不作为总等效不确定量的真值。失败项必须连同结果汇总展示，不能用部分成功图冒充整套成功。

## 保存位置和旧设置

- 唯一代码环境：D:/GO2/code/legged_robot_workspace。
- 数据：D:/GO2/experi/results/v1_1/，每次总入口对应一个新批次。
- 每个条件保存状态、motion.csv 速度指令、mpc_target.csv 实收目标、施力、估计、原始记录、配置和源码信息。
- campaign_result.json 汇总成功和失败；campaign_case_index.json 用于按本批次绘图。
- 原 T01/T02、坡面/台阶站立入口及旧说明归档至 D:/GO2/archive 归档/before_constant_velocity_20260929。多地形场景文件保留，但不纳入当前恒速平地总入口。
- 旧 Word 手册的路径实验命令已经过期，当前运行方法以本文件为准。旧实验数据不删除、不改名为恒速结果。

详细绘图定义见 [实验结构与绘图规范](实验结构与绘图规范.md)。
