# Go2 工作区 Docker 启动与构建

更新日期：2026-09-13

## 固定位置

- 工作区：`D:\GO2\code\legged_robot_workspace`
- Docker Desktop：`C:\Users\lenovo\AppData\Local\Programs\DockerDesktop\Docker Desktop.exe`
- Docker 命令：`C:\Users\lenovo\AppData\Local\Programs\DockerDesktop\resources\bin\docker.exe`
- 容器内工作区：`/workspace`
- 容器内实验数据：`/experi`
- 现有镜像：`legged-robot-noetic:ubuntu20.04`

不要再使用旧的 `D:\code\legged_robot_workspace` 路径。

## 日常启动

在 PowerShell 中执行：

```powershell
powershell -ExecutionPolicy Bypass -File 'D:\GO2\code\legged_robot_workspace\scripts\start.ps1'
```

脚本会检查 Docker；未启动时自动打开 Docker Desktop，等待服务就绪，再进入开发容器。

## 一条命令构建完整工作区

```powershell
& 'C:\Users\lenovo\AppData\Local\Programs\DockerDesktop\resources\bin\docker.exe' run --rm --shm-size 2g -e ROBOT_TYPE=go2 -e CCACHE_DIR=/ccache -e GO2_EXPERIMENT_ROOT=/experi -v 'D:/GO2/code/legged_robot_workspace:/workspace' -v 'D:/GO2/experi:/experi' -v 'go2-ros-home:/root/.ros' -v 'go2-ccache:/ccache' -w /workspace legged-robot-noetic:ubuntu20.04 bash /workspace/scripts/build.sh
```

这条命令直接复用已有镜像，把当前唯一工作区挂载到 `/workspace`，把实验数据目录挂载到 `/experi`，不会另建一份源码。构建脚本默认只开 1 个编译任务，适合当前电脑的内存条件；机器资源充足时可额外加入 `-e BUILD_JOBS=2`。

## Docker 不稳定时的本机备用方式

当前电脑的 `Ubuntu-20.04` 已复用同一个老师工作区，`/workspace` 指向 `D:\GO2\code\legged_robot_workspace`。

如果重装过 WSL，或 `/workspace` 不存在，先执行一次恢复脚本：

```powershell
wsl.exe -d Ubuntu-20.04 -u root -- bash /mnt/d/GO2/code/legged_robot_workspace/scripts/setup_wsl_workspace.sh
```

随后执行完整构建：

```powershell
wsl.exe -d Ubuntu-20.04 -u root -- env BUILD_JOBS=1 bash /workspace/scripts/build.sh
```

2026-09-12 已用这条命令把完整工程编译到 100%。Docker 和 WSL 不要同时编译同一个工作区，也不要在一方仍在运行时强制关闭另一方；中断后应先确认没有残留的空编译文件再续编。

## 使用 Compose

```powershell
& 'C:\Users\lenovo\AppData\Local\Programs\DockerDesktop\resources\bin\docker.exe' compose -f 'D:\GO2\code\legged_robot_workspace\compose.yaml' up -d --build
& 'C:\Users\lenovo\AppData\Local\Programs\DockerDesktop\resources\bin\docker.exe' compose -f 'D:\GO2\code\legged_robot_workspace\compose.yaml' exec dev bash
```

容器内构建：

```bash
bash /workspace/scripts/build.sh
```

停止长期容器：

```powershell
& 'C:\Users\lenovo\AppData\Local\Programs\DockerDesktop\resources\bin\docker.exe' compose -f 'D:\GO2\code\legged_robot_workspace\compose.yaml' down
```

## 快速验证 M01–M05

不启动 Gazebo，只检查五个新模块和统一旁路组合器：

```powershell
wsl.exe -d Ubuntu-20.04 -- bash /mnt/d/GO2/code/legged_robot_workspace/src/leggedcontrol_go2/legged_controllers/scripts/experiments/adaptive_modules/run_adaptive_module_tests.sh
```

2026-09-12 的正式结果为 6 组、33 项全部通过，其中第 6 组是五个模块的统一旁路组合测试。

## 本机 Docker 修复记录

2026-09-12，Docker 的运行目录曾被改为跨盘目录链接，导致服务无法启动。原链接已保留为：

`C:\Users\lenovo\AppData\Local\Docker\run.bad-junction-20260912-1435`

当前已恢复默认普通目录，镜像、容器和源码没有删除。若以后再次启动失败，先运行：

```powershell
& 'C:\Users\lenovo\AppData\Local\Programs\DockerDesktop\resources\bin\docker.exe' info
```

只有该命令成功后再开始 Docker 构建；若仍不稳定，使用上面的 WSL 备用命令。
