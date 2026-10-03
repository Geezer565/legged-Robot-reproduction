# 旧实验脚本清理记录（2026-09-14）

## 清理边界

本次仅删除旧批量实验、旧结果分析、旧独立启动入口及其专用测试。
原始 Gazebo 和实机实验数据没有删除，控制算法代码没有删除。

## 当前保留的实验链

- `run_go2_method_comparison.py`：当前统一批量入口，支持恒力、正弦、叠加正弦和多地形；
- `plot_go2_method_comparison.py`：当前统一统计与绘图；
- `go2_method_comparison.launch`：当前单场 Gazebo 启动入口；
- `apply_myrbf_wrench.py`：统一外力入口；
- `publish_tp01_goal.py`、`record_tp01_target.py`：当前目标点发布与记录基础组件；
- `publish_myrbf_motion.py`：速度命令基础组件；
- `record_stage8c.py`、`record_myrbf_shadow.py`、`record_adaptive_wrench.py`：状态和方法诊断记录；
- `unpause_when_controller_ready.py`：统一启动同步。

上述文件由当前五方法统一实验实际调用，因此保留。
后续新实验确定最终名称后再整体改名，避免在本次清理中同时改变运行行为。

## 已删除的旧实验系列

- M01 无扰动旧批量与分析；
- D01 旧连续力、V1/V2 对照和平滑度分析；
- TP01 旧门限、旧扰动对比和独立启动入口；
- 早期 shadow、三方法、闭环、V4 接口和横向增益标定分析；
- V3 Word 旧结果拼装脚本；
- 上述入口对应的专用测试和旧地形截图辅助脚本。

## 后续规则

新实验只从当前统一入口扩展，不再为某次临时测试复制新的批量脚本。
低层施扰、目标发布和记录组件保持复用；实验条件由统一配置和清单管理。
