# 源文件作用分类（重要：不要把所有修改都说成 RBF 创新）

| 类别 | 主要文件（均相对于 Git 仓库根目录） | 说明 |
|---|---|---|
| RBF 数学核心 | `GO2/code/legged_robot_workspace/src/leggedcontrol_go2/legged_interface/include/legged_interface/adaptive/MyRbfV11Core.h` | 6×26 RBF，q 低通、leakage、V1.2 bias 动力学项、权重投影 |
| RBF 控制接入 | `.../legged_controllers/src/MyRbfV11Control.cpp` | 传感速度/参考、重构 dynamics residual、前一生效扰动、读取参数、诊断 |
| RBF 配置 | `.../legged_controllers/config/go2/myrbf_v11_equivalent_wrench_go2.yaml`、`GO2/code/legged_robot_workspace/rbf_experiment_configs/myrbf_v12B_fz_bias_gain_{050,080}.yaml` | 当前默认 OFF，试验 050/080 ON；leakage 全零 |
| MPC 与 WBC 数据桥 | `.../legged_interface/{include/legged_interface/LeggedInterface.h,src/LeggedInterface.cpp}`、`.../legged_controllers/src/LeggedController.cpp` 等 | MPC/WBC 调用、控制器和接口；有部分 ETH 共用改动 |
| 实验脚本/启动 | `.../legged_controllers/{launch/go2_method_comparison.launch,scripts/run_go2_method_comparison.py,scripts/record_stage8c.py,scripts/plot_go2_method_comparison.py}` | Pure MPC / ETH / RBF 方法比较、记录、绘图 |
| ETH 对比实现 | `.../legged_controllers/src/EthAdaptiveControl.cpp`、`.../include/legged_controllers/EthAdaptivePolicyBinding.h`、`.../ocs2_legged_robot_adaptive/**` | 为 ETH 复现及 CLF-MPC 比较引入；不是 RBF 学习律本体 |
| 诊断/仿真接入 | `.../include/legged_controllers/ExactPayloadTruth.h`、`.../legged_gazebo/src/LeggedHWSim.cpp`、`.../scripts/record_gazebo_contacts.py` | 真实 Yπ、关节 effort、接触力记录；其中真值仅诊断，不用于在线学习 |
| 实验辅助 | `.../scripts/analyze_phase15_xforce.py`、`.../scripts/make_final_fx_prototype.py` 等 | 分析、负载与验证辅助；不属于数学核心 |

**完整的具体相对路径及 SHA256 在 `manifest_release.json` 中；此文档的 `...` 只是为阅读简洁。**

## 跳过文件的说明

审查包记录了一个跳过路径：`.../legged_controllers/include/legged_controllers/MyRbfV11Control.h`，原因是工作区中不存在文件或为软链接。已审查的 `MyRbfV11Control.cpp` 中并未包含这个同名头文件；目前没有证据表明它是构建必需文件，不需要人为凭空补造。但这不能代替真实编译检查。
