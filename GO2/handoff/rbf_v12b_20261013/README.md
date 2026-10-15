# Go2 RBF V1.2-B 科研进度交接包（2026-10-13）

本交接包面向 `Geezer565/legged-Robot-reproduction` 的协作者。**源自用户 2026-10-13 13:01 的本地审查包；本次只做静态检查，未编译、未在 Gazebo 中验证。**

## 1. 基线与内容

- GitHub 基础提交（完整 SHA）：`788411184823d7723034321492b3d417f9efffea`；原本地分支：`rbf-filter-feasibility`。
- `overlay/`：直接覆盖到此 Git 基线之上的**集成工作区快照**，包含 26 个已跟踪修改文件及 17 个必要新文件（其中 2 个被 Git 忽略的实验 YAML），共 43 个文件（**RBF + ETH + 诊断 / 控制器集成**，不宣称全部是 RBF 本体创新）。不含实验结果目录和本地构建产物。
- `git_diff/tracked_changes_from_7884111.patch`：26 个已跟踪文件对基线的原始 Git 差异；**不含 17 个新增文件**，不可以仅应用此 patch 代替 overlay。
- `historical_only/`：原始历史候选参数，保留非零泄漏记录，但**不部署**。
- `optional_not_tested/`：历史候选的 leakage=0 编辑副本，**未仿真验证，不部署**。
- `docs/RBF_METHOD_AND_STATUS.md`：更新率、输入输出、参数与实验边界。
- `docs/FILE_MAP.md`：哪些文件是算法本体，哪些是 ETH 及运行依赖。
- `tools/install_overlay.py`：先检查基线、冲突与 SHA，默认只读；显式 `--apply` 才拷贝文件。不会执行 git commit / push。
- `tools/verify_bundle.py`：仅验证压缩包文件完整性、leakage 配置和语法；不启动 Gazebo。

## 2. 为什么不直接上传所有实验文件

工作区内含大量 Gazebo 记录、图像、日志及 ETH 临时诊断备份。为了不污染 GitHub，**本包只有可核对的代码差异和必要新增文件**。历史数据指标记录在方法文档中，非新仿真结果。

## 3. 关键实验配置与防止误用

**默认主 YAML** `overlay/GO2/code/legged_robot_workspace/src/leggedcontrol_go2/legged_controllers/config/go2/myrbf_v11_equivalent_wrench_go2.yaml` 中 `v12_enable_dynamics_residual: false`，**不会启用新增动力学更新项**。这是源工作区真实状态，不做暗改。

**启用 V1.2 动力学项**需要显式使用：

- `overlay/GO2/code/legged_robot_workspace/rbf_experiment_configs/myrbf_v12B_fz_bias_gain_050.yaml`：Fz bias gain = 0.50。
- `overlay/GO2/code/legged_robot_workspace/rbf_experiment_configs/myrbf_v12B_fz_bias_gain_080.yaml`：Fz bias gain = 0.80。

三者的六维 `v11_leakage` 均为 **[0,0,0,0,0,0]**。C++ 中保留 `-leakage * W`，但这些配置使该项严格为零。

这两份 050/080 配置均为工作区外置试验文件，实际运行时必须通过 runner 的 `--v11-config`（参见 `--help`）显式指定，不能因为文件存在就假设本次实验使用过它。

历史 `phase2_candidate_01_gamma_fy_2x.yaml` 原始 leakage 第三维为 0.02，已经从 `overlay/` 排除；切勿将它误称为本次正式全零泄漏配置。

## 4. 队友如何安全接入

```bash
# 解压后进入交接包目录
python3 tools/verify_bundle.py

# 在包含项目的另一份、干净的 Git 克隆目录上，先检查：
python3 tools/install_overlay.py --repo /path/to/legged-Robot-reproduction --check

# 检查通过后，在专用实验分支上显式应用：
python3 tools/install_overlay.py --repo /path/to/legged-Robot-reproduction --apply

# 人工审查：
git -C /path/to/legged-Robot-reproduction diff --stat
git -C /path/to/legged-Robot-reproduction status --short
```

应用脚本要求 HEAD 与基础提交**完全一致**，任何待覆盖文件已有更改或与 Git 原版不一致时会拒绝修改；应用前会先做全部文件的冲突检查。请勿在已经跑实验、已包含本次修改的作者工作区上重复执行。

**运行环境**：原仓库的 ROS Noetic / OCS2 / Go2 Gazebo 依赖仍需按照仓库原说明配置，本包没有内置所有第三方依赖。新代码编译、运行和每组 Gazebo 清理尚需队友在自己的环境验证。

## 5. 论文表述注意

1. 动力学项 **仅更新每个 RBF 子网络的第 26 个常数 bias 权重**，不是整个 RBF 核权重统一接收动力学梯度。
2. 在线重构动态误差使用名义模型和测得运动，不使用 `trueYpiWorldCom` 真值参加学习；真值仅用于诊断。
3. V1.2-A/B 目前属于工程研究实现，不能仅据静态代码声称已经建立全系统稳定性证明。
4. 之前 −25 N 的 teacher 离线修正效果不代表本包代码已在线验证相同改进。

## 6. 如果队友需要上传自己的 GitHub 分支

在 **已经应用 overlay 的干净基线克隆**中，完成代码复核和必要的编译检查后，再执行（不要在作者当前的实验目录执行）：

```bash
# 请先在应用 overlay 前创建分支：git switch -c feature/rbf-v12b-sync
# 应用后，按具体路径分组，避免把缓存和备份一同提交：
git add -A -- GO2/code/legged_robot_workspace/src/leggedcontrol_go2 GO2/code/legged_robot_workspace/src/ocs2_legged_robot_adaptive

# 注意！两份 rbf_experiment_configs/*.yaml 在作者 Git 工作区被忽略，
# 普通 git add -A 不会自动上传它们；必须明确 -f：
git add -f -- \
  GO2/code/legged_robot_workspace/rbf_experiment_configs/myrbf_v12B_fz_bias_gain_050.yaml \
  GO2/code/legged_robot_workspace/rbf_experiment_configs/myrbf_v12B_fz_bias_gain_080.yaml

git diff --cached --stat
git status --short
# 确认没有 .rbf_sync_backups、仿真结果与私人文件后再 commit/push。
git commit -m 'Sync Go2 RBF V1.2-B and ETH comparison integration'
git push -u origin feature/rbf-v12b-sync
```

**上面的 push 要求本人拥有目标仓库写权限；如果没有，可以推送至自己的 fork 后创建 Pull Request。** 如原仓库已有其他提交，务必先处理版本差异，不能直接强制覆盖。

