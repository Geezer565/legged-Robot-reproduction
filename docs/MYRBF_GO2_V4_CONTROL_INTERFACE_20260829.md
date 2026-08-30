# MyRBF v4 在 Go2 上的完整控制接口

这次修改已经把 MyRBF 从观察状态接入 Go2 反馈控制。原来的网络结构和权值更新没有推倒重做，变化集中在估计结果怎样进入控制器。

## 为什么之前接入后效果弱

旧接口只取六维估计中的侧向力，再把一小部分平均分给四只支撑脚。它会遇到三个问题。

1. 网络看到的残差里混有控制器自己施加的补偿。补偿越大，网络观察到的扰动越小，估计会提前停住。
2. 四只脚平均增加侧向力会附带机身滚转力矩。侧移可能略有改善，侧倾却可能变大。
3. 其余五个方向的估计没有进入控制，六维网络在控制端仍然是一维接口。

## v4 做了什么

v4 会记录上一段预测期间实际施加的补偿。下一次计算外部扰动时，这部分已知量会从机器人受到的合力中分离出来。网络因此继续估计外界扰动，不会把自身控制效果误认为外力消失。

六维估计经过增益、幅值和变化速度限制后，形成机身所需的三维力与三维力矩。控制器根据每只支撑脚相对机身质心的位置，把这六个量联合分配到各脚。分配结果同时检查摩擦、最小支撑力和单脚最大改变量。超出范围时会整体缩小，不会硬塞进控制器。

旧方法仍然保留。关闭补偿就是 Baseline，选择旧配置可以运行 ETHRBF 和 MyRBF v3，选择旧接口可以复现只使用侧向力的结果。新功能由 `myrbf_shadow_v4_go2.yaml` 单独启用。

## 已完成的仿真验证

下面两组都在 Go2 Gazebo 中实际运行。每组使用同一控制器、同一模型和同一扰动，区别只在是否启用 v4 补偿。统计范围按照扰动记录中的真实起止时间选取。

| 场景 | 方法 | 最大侧移 cm | 最大侧倾 deg | 估计误差 N | 波形相关性 | 平均分配后补偿 N |
| --- | --- | --- | --- | --- | --- | --- |
| 恒定侧向力 20 N | Baseline | 3.289 | 1.598 | 2.537 | 0.000 | 0.000 |
| 恒定侧向力 20 N | MyRBF v4 | 2.529 | 1.368 | 1.841 | 0.000 | 4.605 |
| 正弦侧向力 15 N，0.5 Hz | Baseline | 2.260 | 1.560 | 2.046 | 0.992 | 0.00 |
| 正弦侧向力 15 N，0.5 Hz | MyRBF v4 | 1.780 | 1.350 | 1.802 | 0.993 | 2.19 |

恒力实验里，最大侧移下降约 23%，最大侧倾下降约 14%。正弦实验里，最大侧移下降约 21%，最大侧倾下降约 13%。四脚分配比例全程为 1，说明这两组实验没有触发安全缩放。机器人保持站立，控制器没有安全退出。

无扰动时也启用了 v4。十秒稳定段内最大侧移约 0.22 cm，最大侧倾约 0.21°，没有触发安全退出。分配后的三个方向补偿平均幅值都低于 0.7 N，说明静止偏差没有被网络持续放大。

这里的补偿没有追求等于完整的 20 N。当前配置只使用估计值的四分之一，目标是先验证反馈方向和姿态收益。补偿增益可以继续提高，但需要补做更大扰动、不同接触状态和行走实验后再决定。

## 怎样运行

先启动 Docker Desktop。当前电脑使用的程序位于下面的位置。

```powershell
& 'C:\Users\lenovo\AppData\Local\Programs\DockerDesktop\Docker Desktop.exe'
```

进入容器并加载隔离工作区。

```bash
docker exec -it legged-robot-noetic bash
source /opt/ros/noetic/setup.bash
source /tmp/myrbf_go2_catkin/devel/setup.bash
```

运行 20 N 恒定扰动。

```bash
roslaunch legged_controllers myrbf_shadow_gazebo.launch \
  gui:=false duration:=15.0 \
  myrbf_config:=myrbf_shadow_v4_go2.yaml \
  enable_shadow:=true enable_control_injection:=true \
  enable_wrench:=true wrench_profile:=constant wrench_axis:=fy \
  wrench_value:=20.0 wrench_delay:=5.0 wrench_duration:=5.0
```

运行 15 N 正弦扰动。

```bash
roslaunch legged_controllers myrbf_shadow_gazebo.launch \
  gui:=false duration:=15.0 \
  myrbf_config:=myrbf_shadow_v4_go2.yaml \
  enable_shadow:=true enable_control_injection:=true \
  enable_wrench:=true wrench_profile:=sine wrench_axis:=fy \
  wrench_value:=0.0 wrench_amplitude:=15.0 wrench_frequency:=0.5 \
  wrench_delay:=5.0 wrench_duration:=5.0
```

把 `enable_control_injection` 改为 `false` 就能得到同条件 Baseline。

## 代码位置

| 内容 | 位置 |
| --- | --- |
| 补偿感知的扰动观察 | [`legged_interface/include/legged_interface/adaptive/MyRbfPredictionResidualObserver.h`](../legged_interface/include/legged_interface/adaptive/MyRbfPredictionResidualObserver.h) 和 [`legged_interface/src/adaptive/MyRbfPredictionResidualObserver.cpp`](../legged_interface/src/adaptive/MyRbfPredictionResidualObserver.cpp) |
| 六维限制与四脚分配 | [`legged_interface/include/legged_interface/adaptive/MyRbfWrenchCompensation.h`](../legged_interface/include/legged_interface/adaptive/MyRbfWrenchCompensation.h) 和 [`legged_interface/src/adaptive/MyRbfWrenchCompensation.cpp`](../legged_interface/src/adaptive/MyRbfWrenchCompensation.cpp) |
| Go2 控制循环接入 | [`legged_controllers/src/LeggedController.cpp`](../legged_controllers/src/LeggedController.cpp) |
| v4 参数 | [`legged_controllers/config/go2/myrbf_shadow_v4_go2.yaml`](../legged_controllers/config/go2/myrbf_shadow_v4_go2.yaml) |
| 六维诊断记录 | [`legged_controllers/scripts/record_myrbf_shadow.py`](../legged_controllers/scripts/record_myrbf_shadow.py) |
| 结果统计 | [`legged_controllers/scripts/analyze_myrbf_v4_interface.py`](../legged_controllers/scripts/analyze_myrbf_v4_interface.py) |
| 原始验证数据 | 已从活动工作区清理，可从 Git 历史恢复 |

## 目前的边界

这次完成的是 Go2 仿真里的完整反馈控制接口。补偿进入了全身控制器的接触力输入，MPC 下一周期能够看到已经执行的输入。它还没有写进 MPC 预测时域内的动力学模型，因此当前应称为 MyRBF 加 MPC 与全身控制回路，不应称为预测时域内部的自适应动力学模型。

下一阶段应先做行走负重、行走正弦扰动和接触切换，再考虑提高补偿比例。迁移到真机以前还要增加传感器异常、通信延迟和紧急关闭检查。
