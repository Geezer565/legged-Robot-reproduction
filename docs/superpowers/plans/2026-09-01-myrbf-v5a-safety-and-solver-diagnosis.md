# MyRBF V5-A 安全与求解诊断计划

## 目标和边界

目标是先回答“20 N 平地横向力下为什么会失去安全”，再决定是否允许继续做 V4/V5-A 成对验证。本计划不调网络，不改变安全线，不接入 V5-B，不做多地形或载荷实验，也不作创新性或新颖性结论。

本计划开始前冻结现有证据根目录：

`C:/Users/lenovo/Documents/GO2/results/myrbf_v5a_repeat_20260901`

冻结基准为正式清单 SHA-256 `464cb053433891e84a6488d7869035e9bc36a994dc0e93c8c1dded6628a4eb85`、57 文件映射 SHA-256 `f7300c9958cf5d6dbc782fa057ea97275eaf575c1f3e873f654fb350e65f8867` 和源码提交 `f5dab9d71e20c786fc8e5866af47b9ed8829f116`。新诊断写入新的结果根目录，绝不覆盖这批数据。

## 要分开的三类原因

1. 基础控制器或求解器在横向载荷下不稳定。
2. V4/V5-A 补偿请求与力分配、接触约束之间相互影响。
3. 记录或调度有缺口，但没有造成真实的物理失稳。

三类原因在诊断完成前都只是假设。

## 固定场景和三种方法

所有运行固定为平地、无载荷、Go2、`static_walk`、`vx=0.08 m/s`、总时长 18 s。横向力在第 6 s 开始，持续 6 s。`fy` 以外五个轴固定为 0。地形和载荷参数不得出现在运行矩阵中。

三种方法使用同一源码、同一场景、同一种子：

- `BASE`：使用 V4 配置，但关闭补偿注入；保留只读诊断，用来检查基础控制器和求解器。
- `V4`：使用冻结的 V4 配置并开启补偿注入。
- `V5-A`：使用冻结的 V5-A 配置并开启补偿注入。

禁止在一轮中改变控制增益、补偿限幅、学习参数、安全线、步态、速度、时长或力窗口。

## 实施文件

未来实施只允许新建诊断专用文件，冻结分析器和本轮正式证据不动：

- `legged_controllers/scripts/run_myrbf_safety_solver_diagnosis.py`
- `legged_controllers/scripts/analyze_myrbf_safety_solver_diagnosis.py`
- `legged_controllers/test/test_myrbf_safety_solver_diagnosis_runner.py`
- `legged_controllers/test/test_myrbf_safety_solver_diagnosis_analysis.py`
- `legged_wbc/include/legged_wbc/SolverDiagnostics.h`
- `legged_wbc/test/SolverDiagnosticsTest.cpp`
- 新证据根目录：`C:/Users/lenovo/Documents/GO2/results/myrbf_v5a_safety_solver_diagnosis_20260901`

若现有接口不能给出求解状态，只能增加只读发布和记录；不得把诊断值反馈到控制。行为不变测试必须先证明同一输入下的力分配和控制输出逐项相同，容差 `1e-12`。

## 必须记录的诊断

每项都使用统一开始时刻，并同时保留 ROS 时间和墙钟时间：

- 机身高度、姿态、速度、足端接触和安全停止信号。
- 求解器返回状态、工作集重算次数、迭代次数、求解耗时、约束残差和不可行标志。
- 期望接触力、实际接触力、请求补偿、限幅后补偿、实际分配补偿、分配比例和连续饱和时长。
- V4/V5-A 的估计值、冻结、投影和更新分量；`BASE` 也记录同一诊断，但不注入控制。
- 每个记录器的目标周期、实际间隔、丢行数、最大间隔、调度延迟和进程退出状态。
- 每次运行的源码锁、配置哈希、完整命令、种子、方法顺序和所有失败尝试。

## 分级力和种子

### 阶段 0：离线重放

先只读重算现有三次恒力尝试：状态间隔、最低高度、20 N 窗口、求解错误时间分布、请求与实际分配差值。该阶段不启动仿真。若重算结果与 `REPORT_CHECK.json` 不同，停止并检查工具；不得开始新实验。

### 阶段 1：0 N 基线

使用种子 `7101、7102、7103`，每个种子都运行 `BASE、V4、V5-A`。顺序轮换：

- 7101：`BASE → V4 → V5-A`
- 7102：`V4 → V5-A → BASE`
- 7103：`V5-A → BASE → V4`

只有 9 次都通过记录完整性和安全门，才进入施力阶段。

### 阶段 2：安全力梯

按 `5 N → 10 N → 15 N → 20 N` 逐级运行。每一级仍用 `7101、7102、7103` 和上述三种方法、轮换顺序。上一级所有预定运行通过后才能升一级。

任何安全失败都会停止当前级别和更高力级，不补跑更有利的种子。

### 阶段 3：20 N 独立确认

只有阶段 2 的 20 N 全部通过，才用新种子 `7201、7202、7203、7204、7205` 确认。每个种子都运行三种方法，顺序循环轮换，形成 15 次预先固定的运行。不得删掉失败结果，也不得因为某个方法失败而只重跑另一个方法。

20 N 安全成立的定义是：这 15 次运行全部完成、全部安全、没有选择性重试。通过这个门只允许继续平地诊断；多地形、载荷和 V5-B 在本计划内仍保持关闭。

## 提前停止和重试规则

任一条件出现即停止本次施力并安全结束当前运行：

- 单个样本机身高度低于 `0.25 m`，或高度低于 `0.28 m` 持续 `0.10 s`。
- `|roll|` 或 `|pitch|` 超过 `0.45 rad` 持续 `0.10 s`。
- 现有安全检查器发出停止信号。
- 求解器连续 `0.25 s` 返回失败，或 1 s 窗口内工作集错误达到 100 次。
- 分配连续饱和超过冻结门槛 `0.25 s`。
- 状态、扳手或求解诊断超过冻结的最大采样间隔，无法实时确认安全。

安全失败不允许重试。纯记录完整性失败只有在状态和安全通道明确没有触发上述条件时才允许处理，而且必须把同一种子的 `BASE、V4、V5-A` 整组重放一次；原尝试全部保留。每个种子组最多重放一次，规则必须在开跑前写入清单。

## 每级检查门

每次运行必须同时满足：

- 退出正常，所需文件完整、有限、时钟严格递增，覆盖至少 95% 时长。
- 最低高度不低于 `0.25 m`，无安全停止。
- 求解器没有连续 0.25 s 失败，工作集错误没有达到提前停止条件。
- 连续分配饱和不超过 `0.25 s`。
- 扳手在 6–12 s 内为指定 `fy`，窗口外为 0，其余五轴始终为 0。
- 同一种子的三种方法具有相同初始状态、力、时长、步态和速度。

任何一项不满足，该运行就不能进入比较。只要存在安全失败，整个更高力级关闭。

## 判断分支

### 分支 A：`BASE` 先失败

如果 `BASE` 在某一级安全失败，先进入基础控制器/求解器诊断。暂停 V4/V5-A 性能比较，检查接触约束、求解残差、迭代上限和期望接触力。此时不能把失败归因于网络。

### 分支 B：`BASE` 安全，V4 和/或 V5-A 失败

检查请求补偿、限幅后补偿、实际分配补偿、接触余量和饱和时长。只有完整匹配的同种子数据才能区分方法；单个失败不能推出 V5-A 比 V4 好或差。后续若需改变一个补偿参数，必须另开计划和新证据根目录。

### 分支 C：物理通道安全，只有记录失败

如果独立安全通道、机身状态和求解器状态都连续且安全，而普通记录器出现缺口，则进入记录/调度修复。修复只能改变记录方式，不得改变控制。修复后按预先规定整组重放，不能只保留成功文件。

### 分支 D：20 N 全部通过

如果阶段 2 和阶段 3 全部通过，记录“20 N 平地安全门通过”。这仍不是 V5-A 相对 V4 的性能结论，也不是多地形、载荷或新颖性结论。是否恢复完整成对验证必须另行审核。

## 测试和执行命令

实现后先运行：

```bash
python3 -m unittest \
  legged_controllers.test.test_myrbf_safety_solver_diagnosis_runner \
  legged_controllers.test.test_myrbf_safety_solver_diagnosis_analysis -v
catkin build --no-status legged_interface legged_controllers -j2 -l2
ctest --test-dir build/legged_interface --output-on-failure
python3 -m unittest discover -s legged_controllers/test -t . -p "test_myrbf_*.py" -v
```

先检查矩阵，不启动仿真：

```bash
python3 legged_controllers/scripts/run_myrbf_safety_solver_diagnosis.py \
  --output-root C:/Users/lenovo/Documents/GO2/results/myrbf_v5a_safety_solver_diagnosis_20260901 \
  --forces 0 5 10 15 20 --seeds 7101 7102 7103 --dry-run
```

每一级单独授权执行，例如：

```bash
python3 legged_controllers/scripts/run_myrbf_safety_solver_diagnosis.py \
  --output-root C:/Users/lenovo/Documents/GO2/results/myrbf_v5a_safety_solver_diagnosis_20260901 \
  --force 5 --seeds 7101 7102 7103 --execute
python3 legged_controllers/scripts/analyze_myrbf_safety_solver_diagnosis.py \
  --root C:/Users/lenovo/Documents/GO2/results/myrbf_v5a_safety_solver_diagnosis_20260901 \
  --check-force 5
```

分析命令只有在该级所有预定运行结束后才给出分支结果；它不得生成缺失方法的成对指标。

## 交付条件

- 原 Task 5 证据哈希保持不变。
- 运行矩阵、顺序、种子、重试规则和提前停止条件在新数据产生前冻结。
- 三类原因得到可区分的诊断信号和明确分支。
- 20 N 未满足全部预定安全运行前，不恢复架构扩展。
- 本计划结束时仍不创建 V5-B 计划，多地形和载荷仍关闭。
