# GitHub 版本索引（2026-09-02）

## 这个仓库现在包含什么

这里整理的是 Go2 + OCS2 控制框架中的 MyRBF 接入、Gazebo 行走实验、重复验证脚本和安全诊断记录。

现有正式结果来自 Gazebo 仿真，不是实机结果。真实 Go2 的通信、急停、限幅和硬件保护链路尚未通过独立验证，因此不能把本仓库直接当成实机可运行版本。

## 推荐保留的版本

| 名称 | 提交 | 用途 |
|---|---|---|
| `main` | `67e3e95` | 原始论文复现主线 |
| `codex/fix-timestamp-clf-audit` | `7d87e51` | 已确认能够启动的 Go2 NMPC-WBC 基线 |
| `codex/myrbf-go2-baseline-port` | `dd84fb3` | MyRBF V4 接入和 M01 无扰动实验 |
| `codex/myrbf-v5a-safe-update` | `51b4041` | V5-A 安全更新方式 |
| `codex/myrbf-v5a-repeat-gate` | `ef73688` | 当前开发基线，增加重复实验、证据校验和安全终止门禁 |
| 正式实验源码锁 | `f5dab9d` | 2026-09-01 正式批次所用源码，不应改写 |

后续开发应从 `codex/myrbf-v5a-repeat-gate` 开始；历史实验需要复核时，应按对应提交或标签重现，不要覆盖原始结果。

## 建议阅读顺序

1. [`README.md`](../README.md)：项目环境、现有实验和运行入口。
2. [`RUNNABLE_BASELINE_POLICY_20260902.md`](RUNNABLE_BASELINE_POLICY_20260902.md)：以后在哪个版本上继续修改。
3. [`MYRBF_V5A_REPEAT_VALIDATION_RESULT_20260901.md`](MYRBF_V5A_REPEAT_VALIDATION_RESULT_20260901.md)：V5-A 重复验证结果与结论边界。
4. [`superpowers/plans/2026-09-01-myrbf-v5a-safety-and-solver-diagnosis.md`](superpowers/plans/2026-09-01-myrbf-v5a-safety-and-solver-diagnosis.md)：下一步安全和求解诊断计划。
5. [`GO2_MYRBF_EXPERIMENT_AND_CODE_DELIVERY_20260830.md`](GO2_MYRBF_EXPERIMENT_AND_CODE_DELIVERY_20260830.md)：Go2 Gazebo 交付内容和复现实验入口。

## 主要内容在哪里

- `legged_controllers/`：MyRBF 接入控制器的位置。
- `legged_controllers/test/`：更新规律、安全门禁和实验工具的自动检查。
- `scripts/`：批量实验、结果核对和报告生成入口。
- `results/`：正式实验记录、汇总表、日志和证据文件。
- `docs/`：设计依据、版本说明、验证结论和后续计划。
- `deliverables/`：当时锁定的可交付快照。

## 目前能够支持的结论

- M01 已完成 Baseline、ETHRBF 和 MyRBF 的无扰动行走对比。
- V5-A 已完成无扰动重复实验和运行链路验证。
- 正式扰动批次在 V4 的 20 N 条件触发安全终止，因此后续场次没有继续执行。
- 由于 V4 和 V5-A 没有在同一套完整扰动条件下都得到有效数据，目前不能据此宣称 V5-A 的综合性能优于 V4。
- 多地形、负重和真实机器人实验仍是后续工作。

## 上传原则

- 只上传已经提交并验证过的版本。
- 不上传个人临时目录、编译产物、缓存和未整理改动。
- 正式实验结果保留原始记录，不用新运行覆盖旧证据。
- README 和报告必须明确区分仿真与实机。
