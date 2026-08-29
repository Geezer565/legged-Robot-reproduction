# MyRBF Go2 v4 交付说明

这份交付包用于稳定版 Go2 仓库，基准提交为 `7d87e51`。它是源码覆盖包，不包含 OCS2、Pinocchio、Gazebo 和其他底层依赖。

`source_overlay` 里的目录结构与 Go2 仓库根目录一致。把里面的内容复制到仓库根目录，保留同名文件替换，即可得到本轮版本。建议先在新分支或独立工作树操作。

包内保留四条方法路径。

| 方法 | 使用方式 |
| --- | --- |
| Baseline | 关闭 `enable_control_injection` |
| ETHRBF | 使用 `myrbf_shadow_eth.yaml` |
| MyRBF v3 | 使用 `myrbf_shadow_v3_go2.yaml`，关闭控制注入时为观察模式 |
| MyRBF v4 | 使用 `myrbf_shadow_v4_go2.yaml`，打开控制注入 |

详细原理、运行命令、结果与限制见 `docs/MYRBF_GO2_V4_CONTROL_INTERFACE_20260829.md`。`results/v4_interface_validation` 保存本轮无扰动、20 N 恒力和 15 N 正弦扰动的原始记录与自动统计。

在已经配置好的 Docker 环境里执行下面的检查。

```bash
source /opt/ros/noetic/setup.bash
cd /tmp/myrbf_go2_catkin
catkin build legged_interface legged_controllers -j2 -l2
cd build/legged_interface
ctest --output-on-failure
cd ../../src/leggedcontrol_go2/legged_controllers
python3 -m unittest discover -s test -p "test_myrbf_*.py"
```

真实 Go2 上默认不要打开 v4。先完成行走负重、行走周期扰动、接触切换、通信延迟和紧急关闭检查，再逐步开放补偿。
