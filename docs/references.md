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

## 论文

- Kostrikov et al., [Offline Reinforcement Learning with Implicit Q-Learning](https://arxiv.org/abs/2110.06169).
- Chen et al., [Decision Transformer: Reinforcement Learning via Sequence Modeling](https://arxiv.org/abs/2106.01345).

轨迹拟合、离线价值估计和闭环战术收益是不同证据，不能相互替代。
