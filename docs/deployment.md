# ROS 接口与部署

## 运行前的坐标和时间约定

默认 namespace 为 `/sentry`，战场坐标为 `map`，车体为 `base_link`，里程计为 `odom`。
TF 主题为 `/sentry/tf` 与 `/sentry/tf_static`，帧名不重复添加 namespace。
设备驱动应与这个约定一致，或者在启动中显式重映射。`/clock` 始终全局。

位姿以米、角度以弧度、资源以 `[0,1]` 比例表示。源采样时刻必须在同一 ROS 时钟域；
不允许用每次转发的接收时刻刷新旧雷达数据。TF 转换使用观测时刻的变换，失败则丢弃，
不悄悄回退到“最新 TF”。移动坐标系中的速度不被当成世界速度，转换后从位置历史重估。

## 输入 Interface

以下路径都在所选 namespace（默认 `/sentry`）内，完整逐字段契约、超时阈值、缺失降级表与桥接节点示例见 [开放输入输出接口](interfaces.md)：

| Topic / TF | 消息 | `world_model` 订阅 QoS | 约定 |
|---|---|---|---|
| `game/status` | `sentry_interfaces/GameStatus` | **`RELIABLE`**, Volatile, KeepLast(1) | 赛季、己方红蓝、比赛阶段、规则剩余时间、补给区可用性（推荐主接口） |
| `robots/status` | `sentry_interfaces/RobotStatusArray` | **`RELIABLE`**, Volatile, KeepLast(10) | 敌我机器人血量、上限、存活、等级、允许发弹比例（支持裁判串口、雷达站、队内链路多桥接节点并发增量发布） |
| `structures/status` | `sentry_interfaces/StructureStatus` | **`RELIABLE`**, Volatile, KeepLast(10) | 双方基地与前哨站血量、存活、护盾状态（推荐主接口） |
| `perception/tracks` | `sentry_interfaces/ObservationArray` | `SensorDataQoS`（`BEST_EFFORT`） | 带稳定 `robot_id`、相对阵营、源 `stamp`、`frame`、位姿与质量的敌我位置观测；速度可选 |
| `self_pose` | `geometry_msgs/PoseWithCovarianceStamped` | `SensorDataQoS`（`BEST_EFFORT`） | `self_from_tf=false` 时作为本机权威位姿 |
| `map -> odom -> base_link` | TF | TF 标准 QoS | `self_from_tf=true`（默认）时使用最新有时间戳的定位链（车体帧可通过 `base_frame` 修改） |
| `localization_ok` | `std_msgs/Bool` | **`RELIABLE`**, Volatile, KeepLast(1) | `require_localization_health=true`（`robot.launch.py` 默认开启）时必须持续发布；`false` 或超时立即撤销自身有效性 |
| `referee/state` | `sentry_interfaces/RefereeState` | **`RELIABLE`**, Volatile, KeepLast(1) | **旧版兼容接口（将废弃）**；同一语义字段若上述强类型主接口有效且未超时，始终以新接口为准 |
| `scan` | `sensor_msgs/LaserScan` | `SensorDataQoS` | 默认 Nav2 障碍与碰撞监控输入 |
| `odom` | `nav_msgs/Odometry` | Nav2 约定 | Nav2 的底盘里程计 |

> **QoS 兼容要求**：ROS 2 DDS 中 `BEST_EFFORT` 发布端无法匹配 `RELIABLE` 订阅端！编写 `game/status`、`robots/status`、`structures/status`、`localization_ok`（及旧版 `referee/state`）桥接节点时，必须显式使用 `RELIABLE` QoS，否则 `world_model` 将静默收不到任何状态消息。

阵营与编号约定：
- `RobotObservation.team` 与 `RobotStatus.team` 始终编码**相对己方的阵营**（`ALLY=1 / ENEMY=2`），绝不是红=1/蓝=2；`GameStatus.own_team` 则编码本机红蓝方（`TEAM_UNKNOWN=0 / RED=1 / BLUE=2`，若非 0 且与节点配置的 `own_team` 不一致将触发 `TEAM_MISMATCH` 安全停机）。
- 默认战术配置文件（`tactics_rmuc2027.yaml`、`tactics_rmuc2026.yaml`、`tactics.yaml`）省略 `self_id`，随 `own_team` 自动推导 `RED -> 7`、`BLUE -> 107`；如使用自定义编号，可在战术 YAML 中显式填写 `self_id`。`perception/tracks` 中与 `(ALLY, self_id)` 相同的条目会被自动忽略，防止外部雷达覆盖本机定位。
- 单包数组上限分别为 `ObservationArray <= 32`、`RobotStatusArray <= 64`、`StructureStatus <= 16`；单条轨迹最多保留 128 个历史观测，避免异常输入导致内存无限增长。

强类型状态适配器按已知程度填写有效位，缺失字段将对应 `*_valid` 置为 `false`（内部统一记为 `None` 未知，绝不当作 `0` 或满值）：

```text
game/status:       header.stamp, season, own_team,
                   phase + phase_valid, remaining_time + remaining_time_valid,
                   supply_available + supply_valid, source
robots/status:     robots[] -> stamp, robot_id, team (ALLY=1/ENEMY=2),
                   hp + hp_valid, max_hp + max_hp_valid (两者相互独立！),
                   alive + alive_valid, level + level_valid,
                   ammo_ratio + ammo_valid, source
structures/status: structures[] -> stamp, id (own_base/own_outpost/enemy_base/enemy_outpost),
                   hp + max_hp + hp_valid (单个 hp_valid 同时覆盖 hp 与 max_hp！),
                   alive + alive_valid, shield + shield_valid, source
```

注意 `RobotStatus` 的 `hp_valid` 与 `max_hp_valid` 相互独立（仅知当前 `hp` 不知上限时仍保留 `hp`，`hp=0` 可判定阵亡）；而 `StructureState` 的单个 `hp_valid` 同时校验 `hp` 与 `max_hp`（若仅知建筑存活状态，应填写 `alive` + `alive_valid=true` 或一并填入规则手册 `max_hp`）。

这些是本工程的规范化开放 Interface，**不是 RoboMaster 官方串口字节包**。
裁判串口与雷达站驱动应将设备数据映射到上述话题；官方协议解析与硬件驱动由部署方接入，可用字段、通信链路和更新频率以当季规则为准。

## 输出与控制

| Topic / Action / Service | 用途 |
|---|---|
| `world/state` | 带质量、年龄、强类型比赛/机器人/建筑状态和原始轨迹历史的 `WorldState` |
| `team/intents` | 概率意图，包含 `UNKNOWN` 和 `valid` |
| `threat/state` | 分防区威胁（`score` 恒为防区内单敌最大威胁分） |
| `tactical/task` | 最终经仲裁的有期限任务 `TacticalTask` |
| `tactical/feedback` | 接受、运行、取消、到位、失败（`TaskFeedback`） |
| `tactical/trace` | 完整单行 JSON 解释（含 `missing_info` 与 `policy_info`）与回放数据 |
| `compute_path_to_pose` | `Executor` 的最新可达性预检（可通过 `check_path_before_navigate:=false` 关闭） |
| `navigate_to_pose` | `Executor` 独占的 Nav2 目标 |
| `motion/permit` | 带会话和序号的运动许可心跳 |
| `executor/reset_fault` | `std_srvs/Trigger`，解除已可安全清除的执行故障 |
| `cmd_vel` | 最终经 `velocity_guard` 门控输出给底盘的 `Twist` |

输出状态与任务使用 `RELIABLE / VOLATILE / KeepLast(1)`（`tactical/feedback` 深度为 20），原始传感器与轨迹用 `SensorDataQoS`（`BEST_EFFORT`）。
任务不使用 transient-local 重放。执行器用会话、递增 ID、有效期和 immutable 目标定义
处理丢包、重启及迟到消息。同一 ID 不能改变目标，仅能续租。

同时保留旧决策节点、RViz `2D Goal Pose` 控制或另一套 Action Client 会破坏独占假设；
运行内置 `Executor` 时应让 Nav2 只接收本执行器的任务。遥控接管应在底盘控制权仲裁处实现，
并先撤销自主运动许可。
启动执行器意味着接管所选 namespace 的导航：它会取消该 Action Server 上遗留的 Goal，
等待全部终态后再开放新任务。启动清理超时或被拒绝时保持停止，并要求恢复 Nav2 后重启执行器。

## 实车启动

`decision.launch.py` 与各 ROS 2 节点现默认加载 RMUC 2027 战术配置 `tactics_rmuc2027.yaml`（`schema_version: 2`，引用 `field_rmuc2027.yaml` 凸多边形场地语义并启用 `mirror: auto`）。传入 `own_team:=RED` 或 `own_team:=BLUE` 即可在加载阶段自动完成蓝方 180° 场地/禁区镜像与 `7` / `107` 号哨兵 ID 推导。

由设备驱动提供定位、里程计、障碍扫描、定位健康和规范化裁判/雷达数据后，若需同时启动仓库参考 Nav2 与决策链，在仓库根目录执行：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch sentry_bringup robot.launch.py \
  namespace:=sentry \
  own_team:=RED \
  map:=/absolute/path/season_map.yaml \
  config_file:=/absolute/path/tactics_rmuc2027.yaml \
  trace_path:=/absolute/path/logs/match.jsonl
```

`robot.launch.py` 默认叠加 Jazzy MPPI Omni；`nav2.yaml` 本身是差速 RPP 参考配置。
控制器、机器人 radius/footprint、最大速度/加速度、速度门控上限和障碍配置必须按实车校准，
尤其是云台自旋时速度所在的参考坐标系。

若已有正常工作的自有导航栈，只启动 `decision.launch.py`，并按导航栈形态选择接入模式（完整步骤与桥接代码见 [接入自有导航栈](navigation-integration.md)）：
- **自有 Jazzy Nav2**：默认 `enable_executor:=true enable_velocity_guard:=true`；让原 Nav2 输出经平滑/碰撞监控后进入 `nav_cmd_vel`，由 `velocity_guard` 独占 `cmd_vel`，并将规则禁区掩码挂入 global/local costmap `KeepoutFilter`（若原规划器膨胀半径小于战术 `robot_radius` 或未开放路径预检 Action，可加 `check_path_before_navigate:=false`）。
- **`PoseStamped` 目标话题规划器或直接订阅 `tactical/task` 的自研主控**：传入 `enable_executor:=false enable_velocity_guard:=false`，避免内置执行器等待 Action Server 或 `velocity_guard` 向 `cmd_vel` 发送零速抢占底盘。

检查 Interface：

```bash
ros2 interface show sentry_interfaces/msg/GameStatus
ros2 interface show sentry_interfaces/msg/RobotStatusArray
ros2 interface show sentry_interfaces/msg/StructureStatus
ros2 interface show sentry_interfaces/msg/ObservationArray
ros2 topic echo /sentry/world/state --once
ros2 topic echo /sentry/tactical/task --once
ros2 lifecycle get /sentry/bt_navigator
ros2 action list -t
```

定位、比赛状态和数据流恢复后，检查故障原因并按需 reset：

```bash
ros2 service call /sentry/executor/reset_fault std_srvs/srv/Trigger '{}'
```

如果 Action 响应丢失导致旧导航所有权无法确认，reset 会返回失败。
应先停止/恢复 Nav2 和执行器，再建立新任务会话，不跳过未决 Goal 检查。

## 录制与回放

```bash
ros2 bag record -o match_01 \
  /sentry/game/status /sentry/robots/status /sentry/structures/status \
  /sentry/perception/tracks /sentry/referee/state /sentry/self_pose /sentry/localization_ok \
  /sentry/world/state /sentry/team/intents /sentry/threat/state \
  /sentry/tactical/task /sentry/tactical/feedback /sentry/tactical/trace \
  /sentry/tf /sentry/tf_static /sentry/odom /sentry/scan /clock
```

做决策回放时，在新的 ROS domain 中只启动 decision：

```bash
export ROS_DOMAIN_ID=79
ros2 run sentry_ros decision --ros-args \
  -r __ns:=/sentry -p use_sim_time:=true \
  -p config_file:=/absolute/path/tactics_rmuc2027.yaml \
  -p own_team:=RED \
  -p trace_path:=/absolute/path/logs/recomputed.jsonl
```

另一终端使用相同 domain，只播放决策输入，避免原任务与重算任务竞争：

```bash
export ROS_DOMAIN_ID=79
ros2 bag play match_01 --clock --topics /sentry/world/state /sentry/tactical/feedback
```

不要同时回放旧 `/clock` 并生成另一套 `/clock`。纯 JSONL 回放更适合精确复现仲裁与反馈顺序；
rosbag 用于检查消息、TF、QoS 和现场原始数据。

## 从参考环境到比赛场地

完整分步实操清单见 [实车上车教程](real-robot.md)，核心步骤如下：

1. 测量并固定 `map` 系、红蓝阵营（`own_team`）、地面机器人官方编号与时钟同步；核对状态话题全部以 `RELIABLE` QoS 发布。
2. 基于 `tactics_rmuc2027.yaml`（`schema_version: 2`）与独立场地文件 `field_*.yaml` 标定真实 `bounds`、凸多边形 `obstacles` / `keepouts`（凹障碍拆分为多个凸多边形）、`structures`、`supply`、多边形/圆形 `zones`、可选偏好点 `points`（非白名单，决策每周期动态生成六类候选点）与 `mirror: auto`，并用 `.venv/bin/sentry-field ... --own-team RED` 和 `--own-team BLUE` 同时静态校验红蓝双边几何与连通性。
3. 用实测栅格地图验证动态候选点、禁区投影掩码与绕行路径。场地中心、建筑实体中心与可达驻守点是不同概念。
4. 确认代价图 `KeepoutFilter`、局部碰撞监控、速度门控与底盘下位机硬件超时停车链均实际生效，并在台架完成雷达断连与 `READY -> RUNNING` 零延迟恢复演练。
5. 按“纯规则低速 → `rl_blend:=0.0` 离线 RL 影子模式记录 → `rl_blend:=0.25` 受约束混合 → 双边模型校验”四阶段逐步上车。

离线 RL 参数、三态 `rl_mirror` 与影子模式见 [离线强化学习](offline-rl.md)。
`rl_blend:=0.0` 只保持原规则任务选择，原规则仍可导航；需要完全离线检查时使用 JSONL 回放。
