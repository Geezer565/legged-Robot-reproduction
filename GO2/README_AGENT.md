# Go2 V1.1 交接与开发入口

交接日期：2026-10-03。本文是本交接包的现行入口；原始论文、旧源码注释只作来源参考，不覆盖本文的实验约定。

补充设计说明：[V1.1 公式与六维等效不确定量](V1.1公式与六维等效不确定量.md)。相同内容已加入交接 Word 第 7 章，含可编辑公式。按当前代码核对了质量、惯量、六个输出、误差、固定增益更新、快照与软下降处理；不要把总等效量称为已分别识别的负载、摩擦或外力。

交接 Word 第 8 章新增后续实验候选：扰动强度、负载、单轴力矩，以及可选速度/频率、坡道/台阶、接触条件与学习消融；第 9 章为发送清单。这些只是计划，不改变当前 run_all 矩阵，不表示接口都已实现或实验已通过；未获运行指令不得执行。默认仍然每个条件只运行一次，不自动重试。

## 1. 当前状态与证据

自研主线只有 V1.1，与 ETH、Liu-VAN、Zhou-RFF 共四种方法比较。正式任务是平地恒速前进，不是目标点或路径跟踪。上一轮保留 16 份有效原始记录、8 张 PNG 和对应 SVG，入口：`experi/results/v1_1/20261001_comparison_delivery/README.md`。

原整轮 13 项成功、3 项启动失败；排障后补齐三项，共经历五次补录尝试。因此这不是同一二进制一次全部成功的冻结批次。各有效记录的配置、源码快照、日志和 raw.bag 保留。失败尝试的摘要保存在 `handoff/cleanup_provenance.json`，旧失败原始大文件不纳入交接。

V1.1 保留文档的六维等效不确定量对象及权值更新原则；下降条件采用经用户允许的软约束，允许违反并记录。不能声称与原文严格约束完全一致，也不能凭运行成功宣称稳定性证明成立或性能优于其他方法。ETH 不使用 RBF；其余两个对比方法是 Go2 接口移植版，不冒称完全复现论文全部实现。

## 2. 包结构

压缩包顶层为 `GO2/`，推荐解压到 D 盘，使根目录为 `D:/GO2`。不要覆盖另一份仍在使用的工程。

- `code/legged_robot_workspace/src/`：控制、估计、OCS2、机器人模型和仿真场景源码。
- `code/legged_robot_workspace/deps/`：HPP-FCL、Pinocchio 及已展开的依赖源码。
- `code/legged_robot_workspace/scripts/`：首次安装、编译、WSL 配置入口。
- `experi/scripts/`：统一运行入口和 E00–E03 单组入口。
- `experi/results/v1_1/`：上一轮选定的 16 项原始记录及完整绘图。
- `docs/V1.1/`：原始 Word 设计资料。
- `handoff/`：交接说明 Word、文件校验清单、清理来源摘要。

包内不含 Git 历史、build/devel 编译目录、开发缓存和旧实验批次。保留供应商 SDK 的预编译库，它们不是可随意删除的缓存。当前工程编译仍引用 MyRbfV2Control.cpp、MyRbfV5Control.cpp 等共享实现，文件名旧不等于可以删除；公开实验选择中不再提供 V2/V5。

## 3. 环境与首次部署

已使用的运行环境：Ubuntu 20.04 / ROS Noetic / Gazebo 11，HPP-FCL 1.6.0，Pinocchio 2.5.1；Windows 下通过 WSL 使用。`/workspace` 对应 `D:/GO2/code/legged_robot_workspace`。源码包不是 WSL 系统镜像，也不是离线安装包，部分第三方构建需要联网。

已有匹配的 Ubuntu/ROS 环境时，先核对 `/workspace` 未指向其他工程，再执行：

```bash
bash /mnt/d/GO2/code/legged_robot_workspace/scripts/setup_wsl_workspace.sh
bash /workspace/scripts/bootstrap.sh
sudo apt-get update
sudo apt-get install -y python3-numpy python3-matplotlib python3-yaml python3-pil xvfb
```

首次安装应使用 bootstrap.sh；只执行 build.sh 不会完成全部首次依赖安装。上面涉及系统安装，需要管理员权限。已有运行环境只修改源码后，使用 `bash /workspace/scripts/build.sh`。本交接只核对了构建入口，未在空白系统重建验证。

另有 Docker 路径：在工程根目录执行 `docker compose up -d --build dev`，随后 `docker compose exec dev bash /workspace/scripts/bootstrap.sh`。仍需安装上述绘图依赖。先检查 compose.yaml 的固定路径、GPU 要求；换机器需调整。容器中实验路径需采用挂载后的 `/experi`，不要照搬 WSL 的 `/mnt/d` 命令。以 WSL 路径为当前已使用方式。

`src/CMakeLists.txt` 在源码工程中原为 ROS 符号链接；交接包存放其实际内容，避免 Windows 打包产生空文件。若目标环境需要重建，可在目标 Linux 的 src 目录执行 `catkin_init_workspace`，不要覆盖已有有效文件。

## 4. 正式实验入口

以下命令在 WSL Bash 中运行。Windows PowerShell 可以在命令前加 `wsl.exe -d Ubuntu-20.04 -u root --`。

```bash
# 仅查看计划，不启动仿真
bash /mnt/d/GO2/experi/scripts/run_all.sh --plan
# 四种条件 × 四种方法，各运行一次；失败保留后继续，不自动重试
bash /mnt/d/GO2/experi/scripts/run_all.sh
# 单组运行，默认仍比较四种方法
bash /mnt/d/GO2/experi/scripts/run_e00.sh
bash /mnt/d/GO2/experi/scripts/run_e01.sh
bash /mnt/d/GO2/experi/scripts/run_e02.sh
bash /mnt/d/GO2/experi/scripts/run_e03.sh
```

E00 无人工外力；E01 世界系 Fy=5 N；E02 Fy=5 sin(2π×0.15t+π/2) N；E03 Fy=3 sin(2π×0.15t+π/2)+2 sin(2π×0.30t+π/2) N。扰动贯穿正式记录，不中途撤去。施力参考点为 go2::base 的 CoM，不应混称全机器人 CoM。

每项记录 33 s；2.4 s 请求 trot，3–5 s 平滑加速至机身前向 0.45 m/s，随后保持；无终点、无规划路径。不要将速度积分伪装成规划目标。种子为 4102；每项只跑一遍；当前默认单项墙钟超时 900 s。

## 5. 控制器与参数选择

```bash
bash /mnt/d/GO2/experi/scripts/run_e01.sh --method v11_equivalent_wrench
bash /mnt/d/GO2/experi/scripts/run_e01.sh --method eth
bash /mnt/d/GO2/experi/scripts/run_e01.sh --method liu_van_mpc_port
bash /mnt/d/GO2/experi/scripts/run_e01.sh --method zhou_ssi
# 修改速度、时长、外力幅值比例，仅查看计划
bash /mnt/d/GO2/experi/scripts/run_e02.sh --vx 0.45 --duration 33 --motion-delay 3 --force-scale 1 --plan
```

run_all 固定整轮，不接受缩减方法/地形来代替整轮。筛选请用 run_e00–run_e03。支持 `--gui`、`--output-root`；控制器自定义配置分别用 `--v11-config`、`--eth-config`、`--liu-config`、`--zhou-config`。频率目前在 run_go2_method_comparison.py 的 profile_settings() 中定义，没有通用 `--frequency` 参数；修改应统一作用于所有方法并保存配置。

配置目录：`src/leggedcontrol_go2/legged_controllers/config/go2/`。

- V1.1：myrbf_v11_equivalent_wrench_go2.yaml。
- ETH：myrbf_shadow_eth_go2_closed_loop.yaml。
- Liu：myrbf_shadow_liu_hybrid_go2.yaml。
- Zhou：myrbf_shadow_zhou_ssi_rff_go2.yaml。

## 6. 地形接口及边界

支持 `--terrain flat`、`slope`、`steps`、`all`。默认正式 16 项只有 flat。slope/steps 当前强制原地站立检查，不是地形行走实验，不属于上一轮已验证结果。

```bash
# 先看计划；明确要启动时去掉 --plan
bash /mnt/d/GO2/experi/scripts/run_e00.sh --terrain slope --method v11_equivalent_wrench --plan
bash /mnt/d/GO2/experi/scripts/run_e00.sh --terrain steps --method eth --plan
```

flat 使用 myrbf_flat_control.world；slope、steps 使用 myrbf_multi_terrain.world，均在 legged_gazebo/worlds/ 下。斜坡 5°，初始位置约 (3,0,0.49) m、俯仰 −5°；台阶初始位置约 (6.8,0,0.39) m、首级 0.04 m。当前约束法向仍按世界系向上，不把场景存在等同于多地形控制已完成。没有独立 run_terrain.sh，复用单组入口的 --terrain。

## 7. 绘图与完成判断

每个条件仅两组图：六维速度跟踪、六维等效不确定力/力矩；共 8 张，不画位置图和柱状图。实际收到的参考才可作为速度目标。前三维是世界系质心线速度，后三维是机身角速度，不是 roll/pitch/yaw 的导数。保留原始波动，不补零、不为美观改变数据。

不确定量画网络乘积输出，不是权值；重建参考不是独立真实外力。不同方法的估计对象原本不完全相同，不能仅按与人工 Fy 的差值给六维估计排名。

```bash
source /opt/ros/noetic/setup.bash
source /workspace/devel/setup.bash
BATCH=/mnt/d/GO2/experi/results/v1_1/20261001_comparison_delivery
python3 /workspace/src/leggedcontrol_go2/legged_controllers/scripts/plot_go2_method_comparison.py --case-index "$BATCH/campaign_case_index.json" --output "$BATCH/figures_replot" --dpi 300
```

重画时使用新的输出目录，避免覆盖冻结结果。索引支持相对路径，可随包搬移。旧日志中绝对路径保留为来源记录，不是新机器的运行路径。

整轮完成应同时检查：16 项状态、原始记录完整性、最终图片确实导出、失败项明确列出。仅脚本退出或启动 Gazebo 不算全部完成。失败不自动重复；不得把缺少的方法静默漏掉后宣称完整。

## 8. Agent 开发地图与限制

入口链：experi/scripts → legged_controllers/scripts/run_go2_method_comparison.py → launch/go2_method_comparison.launch → 控制器/记录节点 → plot_go2_method_comparison.py。

重点文件：legged_controllers/src/LeggedController.cpp、MyRbfV11Control.cpp，config/go2/myrbf_v11_equivalent_wrench_go2.yaml；模型及约束在 legged_interface 和 OCS2 adaptive 包；全身控制在 legged_wbc。修改前定位具体符号，不仅根据历史文件名判断调用关系。

保持估计对象、wrench 顺序、坐标系、符号、质量来源一致；控制器不能读取仿真施加的真实外力作为学习输入。V1.1 的六个独立一维 RBF 各 25 个中心加偏置，归一化输入与更新周期以当前配置为准。原始公式见 docs/V1.1/ 的 Word；软下降处理必须继续如实记录。

用户只要求设置或绘图时，不启动新实验。修改范围最小化；不新建平行主线；保留其他方法；不擅自恢复 V4/V5 为公开控制器。不为追求曲线好看筛样本或隐瞒失败。仿真、实机和理论证明的结论分开。

本次交接仅为绘图脚本加入相对索引路径支持，不改控制器、学习参数、实验条件；没有重新运行实验。已保留当前可运行的本机 build/devel，它们仅不进入交接包。
