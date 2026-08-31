# MyRBF V5-A 修改前基线

日期：2026-09-01  
分支：`codex/myrbf-v5a-safe-update`  
起点提交：`dd84fb3d038ca2bfbb3af1395e7cfb2b18af824b`

## 工作树状态

该 V5-A 工作树从现有 Go2 MyRBF 分支的提交点单独创建。创建时状态干净，不包含原实验工作树中尚未提交的 2026-08-31 调参脚本、配置和结果，因此不会覆盖或混入那些修改。

## 源码一致性

宿主机与 ROS 容器中的以下三个核心文件 SHA-256 完全一致：

- `MyRbfShadowEstimator.h`：`498b3f78fa2b9d9d6e58ad6ce89fe984905705181ef636562d5aee18627e8c35`
- `MyRbfShadowEstimator.cpp`：`ff00549fa9ea25ed0d4fc10bfbbd5c35fceddc32c44a32ab97e6983f0473256d`
- `TestMyRbfShadowEstimator.cpp`：`e7b74957e7a62e872cb19527930294f3826850e4c11503e739154c345be6cfde`

## 修改前检查

在 ROS Noetic 容器的 `/tmp/myrbf_go2_catkin` 中执行：

```bash
catkin build legged_interface -j2 -l2
cd build/legged_interface
ctest --output-on-failure
```

结果：17 个依赖包和目标包全部构建成功，无警告；`legged_interface` 共 6 项测试全部通过，0 项失败。

## 已冻结的 v4 单次调参参考

以下数字来自 2026-08-30 的 Fy075 单次仿真，只用于回归参考，不作为统计性论文结论：

| 指标 | 数值 |
| --- | ---: |
| 最大横向位移 | 1.283 cm |
| 横向位移均方根 | 0.715 cm |
| 最大横滚角 | 1.044 deg |
| 估计误差 | 1.421 N |
| 波形相关性 | 0.996 |
| 平均施加补偿 | 5.997 N |
