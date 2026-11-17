# 参考资料

## 上游项目

| 项目 | 用途 | 固定参考版本 |
|---|---|---|
| [RMUC-OfflineRL](https://github.com/Harkerbest/RMUC-OfflineRL) | 离线学习方法与数据格式参考 | `064d8336edd18053ce320202187862dce010b5c7` |

版本清单保存在 [`integrations/rmuc_offlinerl/`](../integrations/rmuc_offlinerl/)。
许可证与数据使用条件见 [第三方声明](../THIRD_PARTY_NOTICES.md)。

## 官方数据

[RMUC 2026 区域赛部分赛事数据](https://bbs.robomaster.com/article/1936220)：

- 613 局，4,015,383 条状态，1,474,178 条事件。
- 同一场对阵包含多局，数据划分应保留 `(赛区, 场次号)` 分组。
- 发弹是累计值；受击事件的 robot_id 是受害实体，数值是负扣血。
- 射击/受击事件的目标 ID 缺失，不能直接建立逐机器人伤害归因。
- 全局位置不代表机器人实际可见性或规则允许的通信能力。

下载脚本核对公开附件 MD5 和 SQLite SHA-256，具体值见
[`source.json`](../integrations/rmuc_offlinerl/source.json) 及脚本中的固定数据库校验值。

## ROS / Nav2

- [ROS 2 Jazzy 安装](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
- [ROS Action 设计](https://design.ros2.org/articles/actions.html)
- [Nav2 Humble → Iron](https://docs.nav2.org/migration/Humble.html)
- [Nav2 Iron → Jazzy](https://docs.nav2.org/jazzy/configuration_and_development/migration_guides/iron/Iron/)
- [Jazzy BT Navigator](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/configuring_bt_navigator/)
- [Jazzy KeepoutFilter](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/costmap_2d/costmap_filters/keepout_filter/)

Jazzy 使用 BehaviorTree.CPP 4 系列，内置 Nav2 BT 库由系统注册。
本工程显式使用 Twist；不要直接套用后续发行版的 TwistStamped 默认值。
取消 Action 的服务响应不等于终态 Result。KeepoutFilter 后单独加入膨胀层，以一致处理机器人外形。

## 开源 RoboMaster 决策、导航与强化学习参考项目

以下公开仓库覆盖了 RoboMaster 哨兵导航、规则行为树、多智能体强化学习与离线强化学习等代表性工程方向（许可证与适用边界以各上游仓库声明为准）：

| 项目 | 许可与技术栈 | 定位与互补关系 |
|---|---|---|
| [`Harkerbest/RMUC-OfflineRL`](https://github.com/Harkerbest/RMUC-OfflineRL) | MIT（仅代码）· Python + PyTorch | BNBU Navigator 战队基于 RMUC 2026 区域赛公开日志的全兵种离线强化学习（IQL / BC / DT）框架，为本仓库离线数据管线与方法设计的主要上游参考 |
| [`SMBU-PolarBear-Robotics-Team/pb2025_sentry_behavior`](https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_behavior) + [`pb2025_sentry_nav`](https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav) | Apache-2.0 · C++17 + ROS 2 Humble | 北极熊战队哨兵 `BehaviorTree.CPP` 4.6 规则行为树及 Livox Mid-360 / Point-LIO / `small_gicp` 全向导航栈；底层定位与局部控制可作为外部导航栈与 SentryWeave 组合 |
| [`PolarisXQ/SCURM_SentryNavigation`](https://github.com/PolarisXQ/SCURM_SentryNavigation)（[`rm_decision_cpp`](https://github.com/PolarisXQ/SCURM_SentryNavigation/tree/master/rm_decision_cpp)） | 源码公开（子包各自许可）· C++ + ROS 2 Humble | 四川大学火锅战队整车导航与修改版 `BehaviorTree.CPP` 决策栈（含重定位版 `FAST_LIO` 与地形分析），可与本仓库上层决策互补 |
| [`baiyeweiguang/CSU-RM-Sentry`](https://github.com/baiyeweiguang/CSU-RM-Sentry) | 源码公开 · ROS 哨兵导航 | 中南大学哨兵点云重定位与局部路径规划参考，可作为底层导航模块通过目标点桥接接入 |
| [`RoboMaster/RoboRTS`](https://github.com/RoboMaster/RoboRTS) + [`TARS-Go-Nav/tars_sentry_decision`](https://github.com/TARS-Go-Nav/tars_sentry_decision) | GPL-3.0 · C++（ROS 1 Kinetic / ROS 2 Humble） | DJI 经典自研头文件行为树与 `Blackboard` 决策框架，以及吉林大学 TARS_Go 战队的 ROS 2 Humble 哨兵移植实现 |
| [`DRL-CASIA/RMAI2020-Decision`](https://github.com/DRL-CASIA/RMAI2020-Decision) | MIT · C++ + ROS 1 + `libtorch` | 中科院自动化所 Team Neurons 面向 ICRA 2v2 的行为树与 FC-LSTM 离散动作 A3C 强化学习决策实现 |
| [`RM-Arena/Arena-1`](https://github.com/RM-Arena/Arena-1) + [`Arena-RL`](https://github.com/RM-Arena/Arena-RL) | Arena-1 非商业学习许可 1.0（非 OSI 开源许可，禁商用）· Rust + Python | RMUC 2027/2026 物理仿真对战平台与分层 PPO / LLM 战术训练环境；本仓库仅在作者本机独立环境中用其评测缺失鲁棒性，不随仓库分发其任何代码、资产或原始回放 |

## 论文

- Kostrikov et al., [Offline Reinforcement Learning with Implicit Q-Learning](https://arxiv.org/abs/2110.06169).
- Chen et al., [Decision Transformer: Reinforcement Learning via Sequence Modeling](https://arxiv.org/abs/2106.01345).

轨迹拟合、离线价值估计和闭环战术收益是不同证据，不能相互替代。

## 仓库内相关指南

- [入门教程：从首次克隆到训练与运行](getting-started.md)
- [开放输入输出接口与缺失降级规范](interfaces.md)
- [接入自有导航栈指南](navigation-integration.md)
- [实车上车教程](real-robot.md)
- [系统架构](architecture.md)
- [ROS 接口与部署](deployment.md)
- [离线强化学习](offline-rl.md)
- [规则、HMM 与效用权重基线](baselines.md)
- [实验基线与验证范围](benchmarks.md)
