# 本次静态审查记录

- 审查源：`Go2_RBF_source_review_20261013_130135.tar.gz`（用户上传）
- 源文件：46 个（26 tracked modified，16 untracked selected，4 base unchanged references）
- 发行 overlay：43 个（26 tracked modified + 15 untracked + 2 git-ignored experiment configs，排除历史 candidate）
- 原始 tracked patch 可执行 `git apply --reverse --check` 验证（审查时通过）；解包阶段存在 4 个脚本执行权限丢失，发行 overlay 已恢复对应 `.py` 的可执行标志。
- 初始审查包跳过一个不存在或软链接的 `MyRbfV11Control.h` 路径，不凭空补造。
- `MyRbfV11Core.h` 中 `out.gradient=G.transpose()*out.ec` 连续计算两次，计算等价但属于可读性重复；本次不修改受审源码。
- 当前主配置 `v12_enable_dynamics_residual: false`，必须使用专门的 050/080 测试配置才能打开 bias dynamics term。
- `phase2_candidate_01_gamma_fy_2x.yaml` 非零 leakage 已隔离到历史目录，不当作全零版实验成果；零泄漏副本未运行。
- 当前 C++ 还包括 ETH、Yπ 真值诊断和 Gazebo 关节诊断等工程改动。全量 overlay 是"当前集成源码快照"，不是单一 RBF 创新补丁；如需纯 RBF PR 应进一步按子系统拆分提交并编译测试。
- 不能因静态比对就推断某次仿真的 online teacher 开关或 `effective_rosparams.yaml`，需追溯具体实验 run manifest。
