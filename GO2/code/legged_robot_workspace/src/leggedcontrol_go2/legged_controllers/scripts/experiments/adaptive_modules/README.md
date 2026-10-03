# M01–M05 与统一管线本地验证入口

这个目录提供一条独立的 Linux 验证命令。它不依赖项目现有的 CMake 测试注册，也不会改动工作空间中的编译结果。

## 一条命令运行

在 Ubuntu、WSL 或项目 Docker 容器中执行：

```bash
cd /mnt/d/GO2/code/legged_robot_workspace/src/leggedcontrol_go2
bash legged_controllers/scripts/experiments/adaptive_modules/run_adaptive_module_tests.sh
```

如果已经位于仓库任意目录，也可以直接用脚本的绝对路径运行；脚本会根据自身位置找到仓库和 workspace。

Windows PowerShell 可直接调用现有 WSL：

```powershell
wsl.exe -d Ubuntu-20.04 -- bash -lc 'cd /mnt/d/GO2/code/legged_robot_workspace/src/leggedcontrol_go2 && bash legged_controllers/scripts/experiments/adaptive_modules/run_adaptive_module_tests.sh'
```

## 它会做什么

脚本把五个模块、统一影子管线及其测试编译成一个临时测试程序，然后一次运行 6 套、共 33 项测试：

| 模块 | 主要检查内容 |
|---|---|
| M01 周期扰动 | 谐波学习、方向隔离、重置和异常输入 |
| M02 负载辨识 | 质量与质心辨识、安全范围、冻结和重置 |
| M03 残差动力学 | 在线学习、方向隔离、冻结和异常输入 |
| M04 特征自适应 | 有效样本门槛、单方向调整、发布步长和重置 |
| M05 残差可信度 | 快降慢升、接触变化、六维门控、大小与变化速度限制 |
| 统一影子管线 | 模块组合、开关隔离、异常归零、门控输出和整体重置 |

编译使用 C++14，并打开严格警告；警告、编译错误或任意测试失败都会使脚本立即退出并返回非零状态。临时文件在运行结束后自动清理。

## 自动寻找依赖

脚本会自动寻找：

- 当前 workspace 内的 `ocs2_core/include`；
- 系统 `pkg-config` 或常见位置中的 Eigen3；
- `pkg-config`、系统静态库或系统源码形式的 gtest。

在非标准环境中可显式指定：

```bash
OCS2_CORE_INCLUDE=/path/to/ocs2_core/include \
EIGEN3_INCLUDE_DIR=/path/to/eigen3 \
GTEST_ROOT=/path/to/googletest \
bash legged_controllers/scripts/experiments/adaptive_modules/run_adaptive_module_tests.sh
```

可用 `CXX=clang++` 切换编译器；用 `KEEP_BUILD=1` 保留临时测试程序。脚本后面的参数会继续传给 gtest，例如只运行统一管线：

```bash
bash legged_controllers/scripts/experiments/adaptive_modules/run_adaptive_module_tests.sh \
  --gtest_filter='AdaptiveModuleShadowPipeline.*'
```
