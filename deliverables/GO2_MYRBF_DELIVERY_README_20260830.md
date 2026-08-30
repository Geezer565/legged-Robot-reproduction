# Go2 MyRBF 代码交付说明

本交付包用于老师原有、已经能够运行 Go2 的 OCS2/ROS Noetic 工作区。它不是独立工程，不包含 OCS2、Pinocchio、hpp-fcl、Gazebo 等底层依赖。

## 内容

- `source_overlay`：保持目录结构覆盖到老师的 Go2 仓库根目录；
- `source_overlay/docs/GO2_MYRBF_EXPERIMENT_AND_CODE_DELIVERY_20260830.md`：完整实验、迁移和运行报告；
- `source_overlay/results/go2_method_compare_20260830/EXPERIMENT_REPORT.md`：三组正式实验的条件、曲线、表格和分析；
- `source_overlay/results/go2_method_compare_20260830/V1_*、V2_*、V3_*`：12 场正式实验的原始记录；
- `patches/go2_myrbf_source.patch`：相对于提交 `7d87e51` 的源码补丁；
- `MANIFEST.sha256`：文件校验值；
- `BASE_AND_HEAD.txt`：迁移起点和当前交付提交。

## 安装位置

将 `source_overlay` 中的内容覆盖到：

```text
<teacher_ocs2_ws>/src/leggedcontrol_go2
```

不要把两个源码包放到新的空工作区单独编译。

## 编译

```bash
cd <teacher_ocs2_ws>
source /opt/ros/noetic/setup.bash
catkin build legged_interface legged_controllers legged_gazebo legged_unitree_description
source devel/setup.bash
export ROBOT_TYPE=go2
```

首次运行前请先阅读完整交付报告。当前成果已经通过 Go2 Gazebo 闭环验证，但尚未完成真实 Go2 验证；真机默认必须关闭补偿。
