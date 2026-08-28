# Robot HAL 依赖基线

本仓库在本地开发阶段通过 Cargo `path` 依赖使用工作区外的 Robot HAL：

```text
../links/seeed-robotic/robot-hal
```

当前已验证的 HAL 基线：

- workspace version: `0.5.0-rc.1`
- commit: `10d24ea` (`fix(release): include root protobuf build inputs`)
- 适用组件：SO-ARM serial、B601-DM serial、B601-RS SocketCAN/MIT

各节点的 `Cargo.toml` 会按其自身目录使用相应的相对路径。所有 HAL 依赖必须保持
上述本地 path 方式，确保 SO-ARM、DM、RS 节点共享同一份最新 HAL 源码、协议实现和生命周期语义。发布构建再将 HAL 以经过审计的版本
源闭包/包形式固定；不得在工作流中直接访问串口、CAN 或供应商设备路径。

## 更新检查

```bash
git -C ../links/seeed-robotic/robot-hal log -1 --oneline
cargo tree -p dora-lerobot-hal | rg 'robot-hal-(core|can|serial|runtime)'
cargo check --workspace
```

更新 HAL 后必须重新运行整仓 Rust/Python 测试以及 Dora workflow build；真机
测试前仍需显式配置设备资源并由操作者触发 enable/calibration，系统不会自动
使能或执行机械零位动作。
