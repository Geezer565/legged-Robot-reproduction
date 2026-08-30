# Go2 四方法正式对比结果

本目录保存 2026-08-30 的 Go2 Gazebo 正式对比。最终 MyRBF 参数在 V1–V3 运行前已经锁定；每个条件均为单次确定性运行。

## 结论

| 正式场景 | 相对 Baseline 的主要改善 | 相对原 MyRBF v4 的主要改善 |
| --- | --- | --- |
| 行走多频侧向扰动 | 侧移均方根降低 35.8%，侧倾均方根降低 29.5% | 侧移降低 23.1%，侧倾降低 21.5% |
| 行走等效 4 kg 负重 | 最低高度提高 7.9 mm，高度波动降低 29.0%，最大侧倾降低 50.5% | 估计误差降低 45.8%，高度波动降低 23.7% |
| 站立多频侧倾力矩 | 侧倾均方根降低 12.8%，最大侧倾降低 14.0% | 侧倾降低 8.4%，估计误差降低 16.6% |

四种方法为 Baseline、ETHRBF、OriginalV4 和 RobustMyRBF。所有正式运行正常结束；V1、V2 完成低速行走并回到四脚接触，V3 保持站立。详细设计、调参过程、完整表格和限制见 [`docs/MYRBF_GO2_TUNING_AND_COMPARISON_20260830.md`](../../docs/MYRBF_GO2_TUNING_AND_COMPARISON_20260830.md)。

包含完整实验条件、扰动公式、三张曲线和逐组分析的报告见 [`EXPERIMENT_REPORT.md`](EXPERIMENT_REPORT.md)。

## 图和自动统计

- [`analysis/01_walk_multisine_fy.png`](analysis/01_walk_multisine_fy.png)：行走多频侧向扰动；
- [`analysis/02_walk_payload_fz.png`](analysis/02_walk_payload_fz.png)：行走等效 4 kg 负重；
- [`analysis/03_stand_multisine_tx.png`](analysis/03_stand_multisine_tx.png)：站立多频侧倾力矩；
- 同目录保留 PDF、Markdown 汇总和完整 JSON 指标。

## 原始数据目录

- `V1_walk_multisine_*`：复杂侧向力；
- `V2_walk_4kg_*`：等效负重；
- `V3_multisine_tx_*`：复杂侧倾力矩；
- `C1_*`、`C2_*`、`D1_*`：参数筛选与行走保护检查；
- `G0_*`、`G1_*`：无扰动安全检查。

注意：这些是 Gazebo 闭环仿真，不是真实 Go2 实验。4 kg 场景通过 39.2 N 向下力模拟重量，不包含真实负载惯量变化。
