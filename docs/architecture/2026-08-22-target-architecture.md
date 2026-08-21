# Dora-LeRobot 目标架构

## 目标

本项目重构为一个可扩展的本地机器人运行时与具身学习平台：以 Dora
编排运行时数据流，以 LeRobot 作为数据集、训练、评测和策略的标准中心。
平台必须支持 SO-ARM、Seeed RS、Seeed DM、双臂、未来机械臂以及本地或
云端策略，而不让硬件实现与模型实现相互耦合。

## 逻辑分层

```text
                         Dora Workflow
  采集 / 遥操 / 校准编排 / 回放 / 本地推理 / 云端推理 / 监控
               |                         |
               |                         +--> LeRobot bridge
               |                                dataset v3 / policy
               v
                         Robot Adapter
       observation / action / lifecycle / capability / safety boundary
                               |
                               v
                         Device Driver
    RS MIT / DM CAN-or-MIT / SO-ARM / camera / gripper / future SDK
```

LeRobot 不位于硬件控制层。它通过 recorder 和 policy bridge 与 workflow
连接，并且是 episode、训练输入、评测结果和模型契约的唯一标准来源。

## 层级职责

### Dora Workflow

- 负责节点编排、时间同步、队列策略、遥测与数据流可观测性。
- 将采集、遥操、校准、回放、本地推理和云端推理表达成可组合的子图。
- 只持有组合和环境配置；不持有串口/CAN 协议、机器人运动学或隐式模型逻辑。

### Robot Adapter

- 为上层提供稳定的 robot-level observation、action、lifecycle 与 capability。
- 组合电机、夹爪、相机与传感器为一个具备机器人语义的对象。
- 负责连接、校准、使能、禁用、故障报告、安全停止和断开连接的语义。
- 双臂是 left/right adapter 的组合，加上共享时间同步、标定和安全策略。

### Device Driver

- 仅负责传输、设备协议、寄存器或帧编码、低层读写和设备故障码。
- 不依赖 Dora graph、LeRobot dataset、policy 或任务语义。
- RS、DM、SO-ARM 或新硬件的变化应局限在 driver 与 adapter 层。

### LeRobot Bridge

- Recorder bridge 将 workflow 的已同步 observation/action 写入 LeRobot dataset v3。
- Policy bridge 将 observation 转成模型输入，并把模型 action 转成统一 action。
- 训练、评测、回放和部署只依赖 LeRobot 兼容数据集与策略契约。

## 跨层约束

- observation 与 action 必须版本化、带单位和时间戳，并明确关节顺序、坐标系、
  图像元数据与夹爪约定。
- adapter 必须声明能力，不能用最低公分母抹平设备差异；能力包括控制模式、
  自由度、夹爪、相机、力传感器与支持频率。
- 所有动作在本地经过 enable 状态、限位、速度/加速度和故障检查；云端 policy
  无权绕过此边界。
- 云端推理必须有 deadline、模型版本身份和确定性的本地降级策略。
- 校准由 workflow 引导和可视化，权威状态由 adapter/driver 持久化。

## 目标技术基线

- Dora：`v1.0.0-rc.4`，其 CLI/daemon、Python Node API 与 Rust Node API 必须来自同一
  release revision，禁止与旧 `0.3.x` 节点混用。
- LeRobot：`0.6.1`，使用 dataset v3。
- Python：`3.12`，作为 LeRobot 兼容性和三方硬件 wheel 的首个受支持基线。
- Rust：`1.98.0`，通过 `rust-toolchain.toml` 锁定。

## 非目标

- 不迁移旧 `dora-record` 的 `dataset.parquet` 格式作为新的数据标准。
- 不让 Dora 替换 LeRobot 的 dataset、训练、评测或 policy 抽象。
- 不让云服务进入安全关键控制回路。
- 不以“让全部旧示例继续运行”为升级完成标准；SO-ARM 的端到端闭环是首个
  验收目标。

## 首个验收闭环

SO-ARM 单臂必须能够在目标工具链上完成：连接与校准、遥操、同步采集、写入
LeRobot dataset v3、官方 LeRobot 读取/训练、回放，以及经本地安全过滤后的 policy
推理。该闭环通过后，RS、DM 和双臂只应沿既定边界扩展。
