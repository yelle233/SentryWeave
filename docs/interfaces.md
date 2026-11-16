# 开放输入输出接口与缺失降级规范

本文档定义 SentryWeave 面向外部定位、雷达站、裁判系统与下游导航控制栈的完整数据契约，包括 ROS 2 话题与 QoS 要求、强类型消息逐字段语义与有效位规则、输出任务租约与闭环反馈协议、雷达/裁判信息缺失时的定量降级机制，以及纯 Python 与 ROS 2 桥接代码示例。

> **定位边界**：`sentry_interfaces` 定义的是**与具体赛季字节流解耦的规范化战术接口**，而非 RoboMaster 官方裁判串口帧（如 `0x0001`/`0x0003`）或特定雷达站私有协议。硬件串口解析、视觉/雷达目标关联与坐标系标定在外部驱动或桥接节点中完成，缺失字段只需将对应 `*_valid` 置为 `false`，决策核心会自动按缺失降级规则运行。

---

## 1. 概述、命名空间与话题总表

### 1.1 命名空间、TF 与时钟约定

- **默认命名空间**：`/sentry`（由启动参数 `namespace:=sentry` 控制）。下表所有相对话题名均位于该命名空间下（例如 `game/status` 实际为 `/sentry/game/status`）。
- **TF 重映射与帧名**：`decision.launch.py` 将 `/tf` 与 `/tf_static` 重映射至命名空间内的 `/sentry/tf` 与 `/sentry/tf_static`，但 **TF 帧名本身不加前缀**，固定使用 `map`（世界坐标系，米/弧度，右手系）、`odom`、`base_link`（可通过参数 `base_frame` 修改车体帧名）。若已有 SLAM/激光定位发布在全局 `/tf`，启动时可传 `namespace:=""` 或将 TF 重映射至同一命名空间（详见 [接入自有导航栈](navigation-integration.md)）。
- **全局时钟**：仿真时钟 `/clock` 始终保持全局路径，不受命名空间重映射影响。
- **双时钟隔离**：观测采样、轨迹外推与任务租约使用 ROS 时钟（`header.stamp`）；执行器动作超时、世界状态心跳与底盘速度门控（`velocity_guard`）独立使用操作系统单调时钟（`STEADY_TIME` / `time.monotonic()`），防止 `/clock` 暂停或跳变导致底盘锁死在最后一条非零速度。

### 1.2 输入话题与 QoS 对照表

`world_model` 节点订阅以下输入源并按 `10 Hz` 融合发布 `world/state`：

| 话题 / TF（相对 `/sentry`） | 消息类型 | 建议频率 | `world_model` 订阅 QoS | 用途与优先级 |
|---|---|---|---|---|
| `game/status` | `sentry_interfaces/GameStatus` | 1–5 Hz | **RELIABLE**, Volatile, KeepLast(1) | 赛季、己方红蓝、比赛阶段、剩余时间、补给区可用性（**推荐主接口**） |
| `robots/status` | `sentry_interfaces/RobotStatusArray` | 1–10 Hz | **RELIABLE**, Volatile, KeepLast(10) | 敌我各机器人血量、血量上限、存活、等级、允许发弹比例（支持裁判串口、雷达站、队内链路多桥接节点并发发布） |
| `structures/status` | `sentry_interfaces/StructureStatus` | 1–5 Hz | **RELIABLE**, Volatile, KeepLast(10) | 双方基地与前哨站血量、存活、护盾（支持多桥接节点并发发布） |
| `perception/tracks` | `sentry_interfaces/ObservationArray` | 5–20 Hz | `SensorDataQoS`（**BEST_EFFORT**, Volatile, KeepLast(5)） | 带稳定 `robot_id` 与原始采样时刻的敌我地面位置观测（本机 `ALLY + self_id` 条目会被自动忽略，防止外部雷达覆盖本机定位） |
| `map -> odom -> base_link`（或 `self_pose`） | TF（或 `geometry_msgs/PoseWithCovarianceStamped`） | 10–50 Hz | TF 标准 / `SensorDataQoS` | 本机权威定位（默认 `self_from_tf:=true` 查 TF；设为 `false` 时订阅 `self_pose`） |
| `localization_ok` | `std_msgs/Bool` | 5–20 Hz | **RELIABLE**, Volatile, KeepLast(1) | 定位健康心跳：`require_localization_health:=true`（`robot.launch.py` 默认开启）时必须持续发布，单调时钟超时 `> 0.5 s` 或 `false` 立即进入 `SAFE_STOP` |
| `referee/state` | `sentry_interfaces/RefereeState` | 1–5 Hz | **RELIABLE**, Volatile, KeepLast(1) | **旧版兼容接口（将废弃）**；同一字段若新接口有效且未超时，始终以新接口为准 |

> **重要警告：ROS 2 DDS QoS 兼容性红线**
>
> 在 ROS 2 DDS 规范中，**`BEST_EFFORT` 发布者无法匹配 `RELIABLE` 订阅者**——两端不会建立连接，`world_model` 会静默收不到任何消息且默认不抛异常！
> - 编写 **`game/status`、`robots/status`、`structures/status`、`localization_ok`**（以及兼容接口 `referee/state`）的桥接节点时，发布端**必须显式使用 `ReliabilityPolicy.RELIABLE` + `DurabilityPolicy.VOLATILE`**（单值状态深度 `1`，数组状态建议深度 `10`）。
> - 仅高频位置传感器流 **`perception/tracks`** 与 **`self_pose`** 使用 `qos_profile_sensor_data`（`BEST_EFFORT`）。
> - 联调时务必执行 `ros2 topic info -v /sentry/game/status` 与 `ros2 topic info -v /sentry/robots/status`，确认发布端与订阅端均为 `Reliability: RELIABLE` 且 `Publisher count >= 1`。

### 1.3 输出与控制话题总表

| 话题 / Action / Service | 类型 | 方向与默认频率 | QoS / 协议 | 说明 |
|---|---|---|---|---|
| `world/state` | `sentry_interfaces/WorldState` | `world_model -> decision, executor`（10 Hz） | **RELIABLE**, Volatile, KeepLast(1) | 融合后的世界快照（轨迹代龄、分字段新鲜状态、运行模式） |
| `tactical/task` | `sentry_interfaces/TacticalTask` | `decision -> executor` / 自研导航栈（默认 2 Hz，临界状态跳变立即触发） | **RELIABLE**, Volatile, KeepLast(1) | 经效用评分、RL 受限混合与防抖仲裁后的带租约战术任务 |
| `tactical/feedback` | `sentry_interfaces/TaskFeedback` | `executor` / 自研导航栈 `-> decision`（事件驱动 + 导航进度） | **RELIABLE**, Volatile, KeepLast(20) | 执行状态回传，驱动失败点 `5.0 s` 冷却与取消重发 |
| `team/intents` | `sentry_interfaces/TeammateIntentArray` | `decision` 对外发布（2 Hz） | **RELIABLE**, Volatile, KeepLast(1) | 友方地面机器人 6 类战术意图概率分布与置信度 |
| `threat/state` | `sentry_interfaces/ThreatState` | `decision` 对外发布（2 Hz） | **RELIABLE**, Volatile, KeepLast(1) | 各 `DEFEND_BASE` 防区的最大威胁分、首要拦截目标与预计到达时间 |
| `tactical/trace` | `std_msgs/String` | `decision` 对外发布（2 Hz） | **RELIABLE**, Volatile, KeepLast(1) | 单行 JSON 决策全量审计记录（含 `missing_info` 与 `policy_info`；配置 `trace_path` 时同步写滚动 JSONL） |
| `compute_path_to_pose` | `nav2_msgs/action/ComputePathToPose` | `executor -> planner_server` | ROS 2 Action | `check_path_before_navigate:=true`（默认）时发送导航前整条路径规则合法性预检 |
| `navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | `executor -> bt_navigator` | ROS 2 Action | `enable_executor:=true`（默认）时由 `executor` 独占管理目标发送、取消与终态确认 |
| `motion/permit` | `sentry_interfaces/MotionPermit` | `executor -> velocity_guard`（20 Hz 单调时钟） | **RELIABLE**, Volatile, KeepLast(1) | 带执行器会话 UUID 与递增序号的运动许可心跳 |
| `cmd_vel` | `geometry_msgs/Twist` | `velocity_guard ->` 底盘驱动（50 Hz 单调时钟） | **RELIABLE**, Volatile, KeepLast(1) | 经 `nav_cmd_vel` 限幅与 `motion/permit` 双看门狗门控后的最终底盘速度 |
| `executor/reset_fault` | `std_srvs/srv/Trigger` | 运维调用 `-> executor` | ROS 2 Service | 在定位与 Nav2 恢复且无未决导航后，复位执行器锁定故障 |

---

## 2. 输入消息逐字段契约与有效位陷阱

### 2.1 通用时间戳与多源增量合并规则

1. **严禁伪造新鲜度**：所有消息中的 `header.stamp` 或 `stamp` 必须是**原始设备采样/解算时刻**，绝不能在串口转发定时器中用 `clock.now()` 反复刷新旧缓存数据的时间戳。`WorldModel` 只接受满足 `-0.05 <= received_at - stamp <= timeout` 的记录（允许最多 `50 ms` 轻微时钟偏斜）；若时钟回退超过 `1 ms`，自动清空全部轨迹与状态缓存并递增 `epoch`。
2. **多源并发按字段合并**：`game/status`、`robots/status`、`structures/status` 内部按字段独立维护 `(value, stamp, source_bits)`。某个桥接节点将未知字段置为 `*_valid=false` 时，在内部解码为 `None`，**不会覆盖**另一桥接节点早先上报且尚未超时的有效值；只有 `stamp >= old_stamp` 的有效字段才会更新该字段，并将 `source` 位掩码按位或累计。因此裁判串口桥（上报己方精确血量/发弹量）与雷达站桥（上报敌方位置与估血）可安全向同一话题并发发布。

### 2.2 `GameStatus.msg`（话题：`game/status`）

单条规范化比赛全局状态：

| 字段 | 类型 / 常量 | 有效位与约束 | 语义与注意事项 |
|---|---|---|---|
| `header` | `std_msgs/Header` | `header.stamp` 必填 | 原始采样时刻，超过 `game_timeout`（默认 `2.0 s`）作过期处理 |
| `season` | `string` | 非空即有效 | 赛季标识，如 `"rmuc2027"`、`"rmuc2026"`；未知留空 `""` |
| `own_team` | `uint8`（`TEAM_UNKNOWN=0`, `RED=1`, `BLUE=2`） | `!= 0` 即有效 | **阵营一致性保护**：若非 `0` 且与节点配置的 `own_team`（`"RED"`/`"BLUE"`）不一致，决策立即触发紧急安全驻守 `HOLD`（`reason_code="TEAM_MISMATCH"`）；未知填 `TEAM_UNKNOWN (0)` |
| `phase` | `uint8`（`UNKNOWN=0`, `READY=1`, `RUNNING=2`, `FINISHED=3`） | `phase_valid` (`bool`) | `phase_valid=false` 或 `phase=UNKNOWN` 视为阶段未知（`HOLD: GAME_STATE_UNKNOWN`）；`READY` 或 `FINISHED` 触发 `HOLD: GAME_NOT_RUNNING`；仅 `RUNNING` 允许机动 |
| `remaining_time` | `float64` | `remaining_time_valid` (`bool`) | 当前阶段剩余秒数（有限且 `>= 0`）；无效时置 `remaining_time_valid=false` |
| `supply_available` | `bool` | `supply_valid` (`bool`) | 己方补给/恢复区当前是否可用；`supply_valid=false` 或值为 `false` 时不生成 `RECOVER` 任务 |
| `source` | `uint32` 位掩码 | `SOURCE_REFEREE=1`, `SOURCE_RADAR=2`, `SOURCE_ONBOARD=4`, `SOURCE_TEAM=8` | 数据来源位掩码，支持按位或组合 |

### 2.3 `RobotStatusArray.msg` 与 `RobotStatus.msg`（话题：`robots/status`）

`RobotStatusArray` 包含 `std_msgs/Header header` 与 `RobotStatus[<=64] robots`（单包最多 **64** 条，`WorldModel` 最多跟踪 `max_tracks=32` 个不同 `robot_id`）。`RobotStatus.msg` 逐字段契约如下：

| 字段 | 类型 / 常量 | 有效位与约束 | 语义与易错陷阱 |
|---|---|---|---|
| `stamp` | `builtin_interfaces/Time` | `0 <= now - stamp <= robot_status_timeout`（默认 `2.0 s`） | 该机器人状态的原始采样时刻 |
| `robot_id` | `uint16`（`1..65535`） | 必填 | **官方机器人编号**：红方 `1, 3, 4, 6, 7`（含工程 `2` 等），蓝方 `101, 103, 104, 106, 107`（含 `102` 等）。当战术文件省略 `self_id` 时，系统按 `own_team` 自动推导 **`RED -> 7`、`BLUE -> 107`** |
| `team` | `uint8`（`UNKNOWN=0`, `ALLY=1`, `ENEMY=2`） | 必须为 `ALLY` 或 `ENEMY` | **陷阱 1（相对阵营）**：表示**相对本车的敌我关系**（友军恒为 `ALLY=1`，敌军恒为 `ENEMY=2`），**绝不是红方=1 / 蓝方=2**！当本车为蓝方 `107` 时，蓝方队友 `101/103` 必须填 `ALLY=1`，红方对手 `1/3/7` 必须填 `ENEMY=2`。`UNKNOWN=0` 或同一 `robot_id` 前后改变 `team` 会被拒绝 |
| `hp` | `float64`（`>= 0`） | `hp_valid` (`bool`) | **陷阱 2（独立的有效位）**：`hp_valid` **仅管辖 `hp`**！只知当前血量不知上限时置 `hp_valid=true, max_hp_valid=false`，内核仍保留 `hp`；当 `alive_valid=false` 时会由 `hp > 0` 推导存活（因此 `hp=0` 能准确判定阵亡）。**未知血量必须置 `hp_valid=false`，切勿填 `hp=0, hp_valid=true`** |
| `max_hp` | `float64`（`> 0`） | `max_hp_valid` (`bool`) | **仅管辖 `max_hp`**。血量比例 `hp_ratio = clamp(hp / max_hp, 0, 1)` 仅在 `hp_valid` 与 `max_hp_valid` **同时为真**（或分包合并后两者均未超时）时才非空；否则 `hp_ratio` 保持 `None`（未知） |
| `alive` | `bool` | `alive_valid` (`bool`) | 显式存活标志；有效时优先于 `hp > 0` 决定 `known_alive` |
| `level` | `uint16` | `level_valid` (`bool`) | 机器人等级；未知置 `level_valid=false` |
| `ammo_ratio` | `float64`（**`[0.0, 1.0]`**） | `ammo_valid` (`bool`) | 剩余允许发弹比例（超出 `[0, 1]` 整条拒绝）。按满额发弹基数折算；敌方或未知弹量必须置 `ammo_valid=false` |
| `source` | `uint32` 位掩码 | `SOURCE_REFEREE=1`, `SOURCE_RADAR=2`, `SOURCE_ONBOARD=4`, `SOURCE_TEAM=8` | 数据来源位掩码 |

### 2.4 `StructureStatus.msg` 与 `StructureState.msg`（话题：`structures/status`）

`StructureStatus` 包含 `std_msgs/Header header` 与 `StructureState[<=16] structures`（单包最多 **16** 条，`WorldModel` 最多维护 16 个结构 ID）。`StructureState.msg` 逐字段契约如下：

| 字段 | 类型 / 常量 | 有效位与约束 | 语义与易错陷阱 |
|---|---|---|---|
| `stamp` | `builtin_interfaces/Time` | `0 <= now - stamp <= structure_timeout`（默认 `5.0 s`） | 该建筑状态的原始采样时刻 |
| `id` | `string`（`1..64` 字符） | 必填 | 标准名称为 `"own_base"`、`"own_outpost"`、`"enemy_base"`、`"enemy_outpost"`，也可使用场地文件 `structures` 段声明的自定义结构名 |
| `hp`, `max_hp` | `float64`（`hp >= 0`, `max_hp > 0`） | **`hp_valid` (`bool`，同时管辖两者)** | **陷阱 3（与 `RobotStatus` 不同！）**：`StructureState.msg` 只有一个 `hp_valid`，**同时覆盖 `hp` 与 `max_hp`**。编码 `hp_valid=true` 时必须保证 `hp >= 0` 且 `max_hp > 0`（若填 `max_hp=0` 且 `hp_valid=true`，整条消息会被 `WorldModel` 拒绝！）。若仅知建筑存活与否或不知 `max_hp`，请置 `hp_valid=false` 并填写 `alive` + `alive_valid=true`，或按当季规则手册一并填入满血 `max_hp` |
| `alive` | `bool` | `alive_valid` (`bool`) | 建筑是否存活；`alive_valid=false` 时若 `hp_valid=true` 则按 `hp > 0` 推导，否则为 `None`（“存活但未确认”） |
| `shield` | `float64`（`>= 0`） | `shield_valid` (`bool`) | 基地虚拟护盾值或比例；未知置 `shield_valid=false` |
| `source` | `uint32` 位掩码 | `SOURCE_*`（`1/2/4/8`） | 数据来源位掩码 |

### 2.5 `ObservationArray.msg` 与 `RobotObservation.msg`（话题：`perception/tracks`）

`ObservationArray` 包含 `std_msgs/Header header` 与 `RobotObservation[<=32] observations`（单包最多 **32** 条）。`RobotObservation.msg` 逐字段与 `world_model` 转换规则如下：

| 字段 | 类型 / 常量 | 有效位与约束 | 语义与坐标转换规则 |
|---|---|---|---|
| `header` | `std_msgs/Header` | `frame_id` 非空；`0 <= now - stamp <= predicted_age`（默认 `0.9 s`） | **采集时刻 TF 查询**：若 `header.frame_id != config.frame`（默认 `"map"`），`world_model` 严格按**采集时刻 `header.stamp`** 查询 TF；若该时刻变换不可用，**直接丢弃该观测，绝不静默回退到最新 TF**！同一目标观测时间戳必须严格递增 |
| `robot_id` | `uint16`（`1..65535`） | 必填 | 阵营内稳定身份 ID。**本机保护**：若 `team == ALLY` 且 `robot_id == self_id`，`world_model` 自动忽略该条目，外部雷达绝不覆盖本机定位权威 |
| `team` | `uint8`（`UNKNOWN=0`, `ALLY=1`, `ENEMY=2`） | 必须为 `ALLY` 或 `ENEMY` | 相对本车的敌我关系（同 `RobotStatus.team`） |
| `pose` | `geometry_msgs/PoseWithCovariance` | `(x, y)` 有限且位于场地 `bounds` 内 | 世界平面位置。若相邻两帧位移超过 `max_speed * dt + jump_slack`（默认 `6.0 * dt + 0.6 m`），首次跳变被暂存拒绝，需新位置处第二帧一致观测确认后才重建轨迹并递增 `generation` |
| `twist` | `geometry_msgs/TwistWithCovariance` | `velocity_valid` (`bool`)，模长 `<= max_speed`（`6.0 m/s`） | 当观测原属非 `map` 移动坐标系并经 TF 转换后，`world_model` **强制清空 `velocity_valid=false`**（不把移动系速度直接旋转充当惯性速度，改由世界系位置历史通过 α-β 滤波重估） |
| `confidence` | `float64` | **`(0.0, 1.0]`**（左开右闭） | 目标检测/跟踪质量分，`<= 0` 或 `> 1` 会被拒绝 |
| `source_mask` | `uint32` | 非负整数 | 传感器来源掩码（如 `SOURCE_RADAR=2`, `SOURCE_ONBOARD=4`, `SOURCE_TEAM=8`） |

### 2.6 本机定位与旧版兼容接口 `RefereeState.msg`

- **本机权威定位（TF 或 `self_pose` + `localization_ok`）**：
  - 默认 `self_from_tf:=true`：`world_model` 每 `0.1 s` 查询 `map <- base_frame` 最新带非零时间戳的变换，**直接使用定位输出的坐标而不做二次轨迹平滑或前向预测外推**（防止绕障拐弯时平滑切入墙体）。
  - 观测起点吸附 vs 目标严格合法：决策生成任何候选目标或执行目标都严格要求 `arena.legal(goal)`（距场地边界、物理障碍与规则禁区 `>= robot_radius`）；而当实车本机当前定位贴近墙沿略小于 `robot_radius`（仍位于 `bounds` 内）时，规划起点通过 `arena.origin(self_pos)` 吸附到最近合法自由栅格，避免因轻微贴边自锁 `LOCALIZATION_INVALID`。
  - `localization_ok`（`std_msgs/Bool`）：实车入口 `robot.launch.py` 默认设 `require_localization_health:=true`；收到 `false` 或单调时钟超过 `0.5 s` 未更新时，立即清空 `self_track` 并置 `WorldState.mode = "SAFE_STOP"`。
- **旧版兼容接口 `referee/state`（`RefereeState.msg`，将废弃）**：
  - 含 `phase` + `phase_valid`、`hp_ratio` + `hp_valid`、`ammo_ratio` + `ammo_valid`、`own_outpost_alive` + `own_outpost_valid`、`enemy_outpost_alive` + `enemy_outpost_valid`、`supply_available` + `supply_valid`、`remaining_time` + `remaining_time_valid`。
  - **字段级覆盖优先级（`sentry_core/inputs.py: view_inputs`）**：对每个语义字段，只要新强类型接口（`GameStatus`、`RobotStatus[self_id]`、`StructureState`）中该字段有效且未超时，**一律以新接口为准**（`sources[field] = "typed"`）；仅在新接口缺失/超时且 `RefereeState` 未超时（`referee_timeout=2.0 s`）时才回退读取旧接口。新建工程请直接发布 `game/status`、`robots/status` 与 `structures/status`。

---

## 3. 输出消息与控制契约

### 3.1 `tactical/task`（`TacticalTask.msg`）—— 租约化战术任务

`decision` 节点以 `RELIABLE` + `VOLATILE` + `KeepLast(1)` 发布 `TacticalTask`（不使用 `TRANSIENT_LOCAL`，避免向迟到重启节点重放过期旧指令）：

| 字段 | 类型 / 常量 | 说明与下游消费契约 |
|---|---|---|
| `header` | `std_msgs/Header` | 发布时刻与世界坐标系 `frame`（默认 `"map"`） |
| `session_id` | `string` | 决策引擎会话 UUID；节点重启、时钟回退或 `world.epoch` 变化时重新生成 |
| `task_id` | `uint64` | 会话内从 `1` 单调递增的任务序号。**续租与重发契约**：同一任务持续胜出且目标坐标漂移 `<= goal_update_distance`（默认 `1.5 m`）时，**`(session_id, task_id)` 与 `target_pose` 保持不变**，仅延长 `valid_until`（续租心跳，下游规划器不应重置当前导航！）；当胜出候选键改变、动态目标移动 `> 1.5 m`（`GOAL_MOVED`）、取消后恢复或 RL 子目标失败回退时，`task_id` 递增 |
| `based_on_world_seq` | `uint64` | 本次评估所依据的 `WorldState.world_seq` |
| `task_type` | `uint8` | `HOLD=0`（驻守/停车）、`SUPPORT=1`（协同进攻/支援）、`INTERCEPT=2`（防区拦截）、`DEFEND=3`（区域布防）、`RECOVER=4`（回补给区恢复） |
| `target_zone_id` | `string` | 目标战术区域 ID（`HOLD` 时为空字符串 `""`） |
| `target_robot_id` + `target_robot_valid` | `uint16` + `bool` | 关联机器人 ID（`SUPPORT` 为协同友方 ID，`INTERCEPT` 为拦截敌方 ID；无关时 `target_robot_valid=false`、`target_robot_id=0`） |
| `goal_id` | `string` | 候选点标识（场地偏好点名，或动态候选点标识；即使坐标经离线 RL 子目标细化，`goal_id` 仍保留原规则候选点 ID） |
| `target_pose` + `target_pose_valid` | `geometry_msgs/PoseStamped` + `bool` | `HOLD` 时 `target_pose_valid=false`；其余任务为 `true`，保证在 `map` 系内满足 `arena.legal(goal)`，四元数编码期望平面朝向 `yaw` |
| `utility` | `float64` | 综合效用分 `[0.0, 1.0]` |
| `confidence` | `float64` | 任务置信度 `[0.0, 1.0]` |
| `urgent` | `bool` | 紧急标志：高威胁拦截（`score >= urgent_threat [0.72]`）、强制恢复（低血/低弹锁存态）及安全原因 `HOLD` 为 `true`，可跳过普通防抖直接抢占 |
| `created_at`, `valid_until` | `builtin_interfaces/Time` | 任务首次创建时刻与租约截止时刻（每次续租刷新为 `world.stamp + task_lease`，默认 **`task_lease = 2.0 s`**；下游若超时未收到有效续租必须停车） |
| `reason_code` | `string` | 原因码：安全类 `FRAME_MISMATCH` / `LOCALIZATION_INVALID` / `GAME_STATE_UNKNOWN` / `GAME_NOT_RUNNING` / `TEAM_MISMATCH` / `ROBOT_DISABLED`；空候选 `NO_FEASIBLE_TASK`；战术类 `AREA_UNCOVERED` / `BASE_THREAT` / `TEAM_SUPPORT_HIGH` / `FIXED_OUTPOST_RULE` / `LOW_RESOURCES` |

### 3.2 `tactical/feedback`（`TaskFeedback.msg`）—— 执行反馈与失败点冷却

下游执行器（内置 `executor` 或外部自定义导航桥接节点）通过 `tactical/feedback`（队列深度 `20`）回传任务执行状态：

- **字段**：`header`、`session_id`、`task_id`、`execution_state`（`IDLE=0`, `ACCEPTED=1`, `RUNNING=2`, `CANCELING=3`, `SUCCEEDED=4`, `FAILED=5`, `CANCELED=6`, `REJECTED=7`）、`remaining_distance_m` + `remaining_distance_valid`、`reason_code`。
- **身份校验**：`DecisionEngine.feedback` 仅处理 `(session_id, task_id)` 与当前活跃任务完全匹配的反馈，自动忽略过时任务的迟到回执。
- **失败冷却与 RL 子目标无损回退**：
  - 当收到 **`FAILED (5)`** 或 **`REJECTED (7)`** 时：
    - 若当前目标为普通规则点，将该 `goal_id` 加入冷却表，在 **`cooldown = 5.0 s`** 内过滤该点（记录 `<goal_id>:FAILURE_COOLDOWN`）并改选次优合法候选点。
    - **若当前任务是由离线 RL 子目标细化后的坐标（`_refined=True`）**：失败归因于细化子目标而非原规则任务——引擎**仅将 `"rl_subgoal"` 冷却 `5.0 s`**（不冷却原规则 `goal_id`），并在下一周期分配新 `task_id` 立即在原规则目标点上恢复该任务。
  - 当收到 **`CANCELED (6)`** 时：清空当前任务引用，确保看门狗取消后恢复时生成新 `task_id`。
  - 当收到 **`SUCCEEDED (4)`** 时：表示机器人已到位进入 `ARRIVED` 驻守态，任务继续正常续租保持驻守。

### 3.3 态势、意图、防区最大威胁与运动许可消息

- **`world/state`（`WorldState.msg`）**：包含 `world_seq`、`epoch`、`mode`（`"FULL"` 所有外部轨迹可用 / `"PARTIAL"` 部分外部轨迹可用 / `"LOCAL_ONLY"` 无可用外部轨迹但本机健康 / `"SAFE_STOP"` 本机定位失效或超时）、`self_robot` (`RobotTrack`) + `self_valid`、`allies[<=32]`、`enemies[<=32]`、`game` + `game_valid`、`robots[<=64]`、`structures[<=16]` 及兼容字段 `referee` + `referee_valid`。其中 `RobotTrack.status` 取 `OBSERVED=1`（`age <= 0.3 s`）、`PREDICTED=2`（`0.3 s < age <= 0.9 s`）、`STALE=3`（`0.9 s < age <= 5.0 s`），`history[<=128]` 保留最近 `history_sec=3.0 s` 内的观测。
- **`team/intents`（`TeammateIntentArray.msg` / `TeammateIntent.msg`）**：对每名轨迹可用的队友输出 `robot_id`、`intent_type`（`UNKNOWN=0, ATTACK_OUTPOST=1, DEFEND_BASE=2, CONTEST_AREA=3, RETREAT=4, TRANSIT=5`）、关联 `zone_id`、6 维归一化后验 `probabilities`、`confidence` 与布尔位 `valid`。
- **`threat/state`（`ThreatState.msg` / `ZoneThreat.msg`）**：对每个 `DEFEND_BASE` 区域输出 `zone_id`、`score`、`enemy_id` + `enemy_valid`、`eta_sec` + `eta_valid`。
  - **`ZoneThreat.score` 计算契约**：`score` **恒取该防区对所有在场敌方单机威胁分的最大值**，直接决定区域防守紧迫度以及 `INTERCEPT` 是否达到紧急抢占阈值 `urgent_threat`（`0.72`）。已知敌方低血量奖励 `enemy_low_hp_bonus * (1 - hp_ratio)`（默认 `enemy_low_hp_bonus = 0.15`）仅加在选敌优先级 `priority` 上，**只改变优先锁定哪台 `enemy_id` 及其 `eta_sec`，绝不会抬高防区本身的 `score`**。
- **`motion/permit`（`MotionPermit.msg`）**：内置 `executor` 按 `20 Hz` 发送 `executor_session`、单调递增 `sequence`、`allowed` 与 `reason`。仅在启动清理完成（`READY`）、Nav2 活跃 Goal 唯一且属于本执行器、世界状态健康且处于 `RUNNING` 时 `allowed=true`。`velocity_guard` 要求 `motion/permit`（超时 `0.3 s`）与 `nav_cmd_vel`（超时 `0.25 s`）同时健康且序号递增，否则立即向 `cmd_vel` 发送零速。
- **`tactical/trace`（`std_msgs/String`）**：单行紧凑 JSON（`schema_version: 1`），包含 `episode_id`、`map_fingerprint`、`world`、`decision`、`feedback_before` 与单步耗时 `decision_ms`（详见 §4.4）。

---

## 4. 统一“未知”语义、超时与雷达信息缺失降级

### 4.1 统一“未知”原则

SentryWeave 在全部数据链路中严格贯彻 **“未知（`None`）既不等于 `0`，也不等于满值”**：

- 未知敌方血量不会触发残血追击加分，也不会把敌方当作 `0` 血阵亡剔除；
- 未知前哨站/基地状态不会被当作“已摧毁”而取消进攻，而是按“存活但未确认”温和折减进攻权重；
- 未知本机血量/发弹量时禁止发起激进的 `SUPPORT` 突击，但仍允许执行 `DEFEND` 与 `INTERCEPT`。

### 4.2 默认超时与轨迹生命周期参数表

以下参数均可在 `tactics_*.yaml` 的 `parameters` 段按实车通信质量调整：

| 参数名 | 默认值 | 作用对象与超时行为 |
|---|---|---|
| `self_timeout` | `0.5 s` | 本机定位（TF / `self_pose`）及 `decision`/`executor` 接受 `WorldState` 的最大允许年龄；`localization_ok` 单调时钟超时同样为 `0.5 s`；超时立即进入安全 `HOLD(LOCALIZATION_INVALID)` |
| `game_timeout` | `2.0 s` | `game/status` 各字段新鲜度窗口；超时后若无未过期兼容 `referee/state`，阶段变为 `UNKNOWN` 并进入安全 `HOLD(GAME_STATE_UNKNOWN)` |
| `robot_status_timeout` | `2.0 s` | `robots/status` 中每台机器人各状态字段（`hp`、`max_hp`、`alive`、`level`、`ammo`）的有效窗口；超时字段自动归为 `None` |
| `structure_timeout` | `5.0 s` | `structures/status` 中每个建筑各状态字段的有效窗口；超时后归为“存活状态未知（`None`）” |
| `referee_timeout` | `2.0 s` | 旧版兼容 `referee/state` 消息的有效窗口 |
| `observed_age` | `0.3 s` | 位置观测年龄 `age <= 0.3 s` 时轨迹状态为 `OBSERVED`（`usable = True`） |
| `predicted_age` | `0.9 s` | `0.3 s < age <= 0.9 s` 时轨迹状态为 `PREDICTED`（对友军/敌军做短时速度外推，置信度按 `exp(-age / predicted_age)` 衰减，`usable = True` 且 `confidence > 0.05`）；拒绝延迟 `> 0.9 s` 的陈旧观测入库 |
| `retention` | `5.0 s` | `0.9 s < age <= 5.0 s` 时轨迹转为 `STALE`（`usable = False`），保留最后已知位置供敌方失联威胁衰减计算；`age > 5.0 s` 彻底移除 |

### 4.3 逐类信息缺失降级行为表

每个决策周期，`sentry_core/inputs.py: assess` 先统计 6 个维度的信息完整度 `completeness`（范围 `[0.0, 1.0]`；注意**已确认阵亡 `known_alive == False` 的机器人自动从期望在场分母 `expected_allies` / `expected_enemies` 中扣除**，不会因对手阵亡离场误判为雷达掉线！），随后按以下规则平滑降级：

| 缺失场景 | 定量降级行为与公式 | `missing_info.degraded` 审计标记 |
|---|---|---|
| **敌方地面位置部分或全部缺失**（雷达掉线 / 遮挡） | ① 处于 `predicted_age..retention`（`0.9..5.0 s`）的 `STALE` 敌方轨迹保留最后已知位置，权重按 `stale_enemy_weight * (1 - (age - predicted_age) / (retention - predicted_age))`（默认 `stale_enemy_weight = 0.5`）线性衰减至 `0`，按保守速度 `0.7 m/s` 估计防区威胁；<br>② 对所有 `DEFEND_BASE` 防区的 `DEFEND` 候选额外增加紧迫度 `boost = missing_enemy_defense_boost * (1 - completeness["enemies"])`（默认 `missing_enemy_defense_boost = 0.25`），自动收缩稳守关键防区 | `STALE_ENEMY:<id>`<br>`ENEMY_POSITIONS_MISSING:defense+<boost>` |
| **敌方血量未知**（无裁判敌方血量或未给 `max_hp`） | 威胁选敌优先级 `priority` 不加 `enemy_low_hp_bonus`（`0.15`），退化为纯空间威胁分选敌；若已知 `hp=0` 或 `alive=false` 则直接剔除该敌方 | `ENEMY_HP_UNKNOWN:<id>` |
| **队友位置缺失或已知阵亡** | ① 忽略过期（`!usable`）或已知阵亡（`known_alive == False`）的队友，针对阵亡队友的支援点直接拒绝（`<goal_id>:ALLY_DOWN`）；<br>② `SUPPORT` 任务的协同项乘缩放因子 `scale = 0.5 + 0.5 * completeness["allies"]`，队友视野不全时降低孤军前压倾向 | `ALLIES_INCOMPLETE:support*<scale>` |
| **前哨站 / 基地存活状态未知** | 视为“存活但未确认”（`alive is None`）：`ATTACK_OUTPOST` 区域的 `SUPPORT` 战略价值乘 `unconfirmed_attack_factor = 0.7`；一旦确认摧毁（`alive == False`）立即拒绝（`<goal_id>:TARGET_DESTROYED`）；若确认 `own_outpost` 被毁，`DEFEND` 战略价值提高 `+0.12` | `STRUCTURE_UNCONFIRMED:<structure>` |
| **本机血量 / 弹量 / 补给可用性未知** | `self_hp` 或 `self_ammo` 为 `None` 时拒绝生成 `SUPPORT` 候选（`<goal_id>:RESOURCES_UNCERTAIN_OR_LOW`），仍保留 `DEFEND` 与 `INTERCEPT`（风险项按中性 `hp=0.5` 计）；`supply_available != True` 时不生成 `RECOVER` | 记录于 `decision.rejected` 与 `missing_info.missing` |
| **队友或敌方位置完整度 `< completeness_threshold`（`0.6`）时离线 RL 降权** | 在线加载离线 RL 模型（`GoalPrior`）时，若 `completeness["allies"] < 0.6` 或 `completeness["enemies"] < 0.6`，自动按 `scale = min(completeness["allies"], completeness["enemies"]) / 0.6` 线性下调本周期混合权重 `effective_blend = rl_blend * scale`；当任一方位置全丢（`0.0`）时 `effective_blend = 0.0`，**自动退回 `SHADOW` 影子模式**（仅记录建议点，既不改变候选排序也不做子目标细化） | `RL_BLEND_REDUCED:INFO_INCOMPLETE:<categories>` |
| **本机定位失效 / 比赛阶段非 `RUNNING` 或未知 / 阵营不匹配 / 本机阵亡** | 立即输出紧急安全驻守 `HOLD`（`urgent=True`，原因码见 §3.1），执行器撤销运动许可使底盘停稳；**一旦安全条件恢复（如定位恢复或比赛从 `READY` 切入 `RUNNING`），首个决策周期跳过 `min_hold`、`switch_margin` 与 `confirm_cycles`，零延迟切换至最优任务，仲裁状态记为 `SAFETY_RECOVERED`** | 体现于 `task.reason` 与恢复周期的 `decision.arbitration = "SAFETY_RECOVERED"` |

### 4.4 `tactical/trace` 中 `missing_info` 与 `policy_info` 真实字段说明

订阅 `/sentry/tactical/trace` 或读取 `trace_path` 生成的 JSONL 文件时，每行 JSON 的 `decision` 对象均包含完整的缺失诊断 `missing_info` 与策略信息 `policy_info`：

```json
{
  "missing_info": {
    "completeness": {
      "game": 1.0,
      "self": 1.0,
      "allies": 0.667,
      "enemies": 0.25,
      "robot_status": 0.429,
      "structures": 0.5
    },
    "missing": [
      "allies.positions 2/3",
      "enemies.positions 1/4",
      "enemies.103.stale",
      "robots.hp 3/7",
      "structure.own_outpost",
      "structure.enemy_base"
    ],
    "rl_blend": {
      "configured": 0.25,
      "effective": 0.10416666666666667,
      "reason": "INFO_INCOMPLETE:enemies"
    },
    "degraded": [
      "STALE_ENEMY:103",
      "ENEMY_HP_UNKNOWN:103",
      "ENEMY_POSITIONS_MISSING:defense+0.19",
      "STRUCTURE_UNCONFIRMED:enemy_outpost",
      "ALLIES_INCOMPLETE:support*0.83",
      "RL_BLEND_REDUCED:INFO_INCOMPLETE:enemies"
    ]
  },
  "policy_info": {
    "algorithm": "iql",
    "model_id": "9f8c...e21a",
    "dataset_id": "rmuc2026_offline_v1",
    "domain_transfer": false,
    "mirror": false,
    "blend": 0.25,
    "effective_blend": 0.10416666666666667,
    "blend_reason": "INFO_INCOMPLETE:enemies",
    "used": true,
    "status": "READY",
    "cache_age": 0.0,
    "proposal": [8.42, 7.35],
    "projection_distance_m": 1.5,
    "projected_goal": [8.06, 7.42],
    "subgoal_refined": true,
    "subgoal_reason": "REFINED",
    "subgoal": [8.06, 7.42],
    "subgoal_base": "defend:own_base:0",
    "model_fallbacks": {
      "intent_model": {
        "mode": "rule",
        "reason": "MAP_FINGERPRINT_MISMATCH",
        "path": "artifacts/intent_hmm.json",
        "model_map_fingerprint": "a1b2...",
        "field_map_fingerprint": "c3d4...",
        "detail": "intent model map fingerprint mismatch; retrain for this map (model a1b2..., field c3d4...); using the rule intent estimator"
      }
    }
  }
}
```

关键字段说明：

- **`missing_info`**：
  - `completeness`：6 个信息类别（`game`, `self`, `allies`, `enemies`, `robot_status`, `structures`）当前的可用率 `[0.0, 1.0]`。
  - `missing`：具体缺失或陈旧的条目清单（如 `enemies.103.stale` 表示 103 号敌车轨迹已超过 `predicted_age` 进入陈旧衰减阶段）。
  - `rl_blend`（仅在配置了 `rl_model` 时出现）：记录配置权重 `configured`、经位置完整度缩放后的 `effective` 以及触发降权的 `reason`。
  - `degraded`：本周期实际生效的所有缺失降级修正项列表（保序去重）。
- **`policy_info`**（详见 [离线强化学习](offline-rl.md)）：
  - `status`：离线 RL 提议状态，取值包括 `"READY"`、`"SHADOW"`（`effective_blend == 0` 时仅记录不干预，同时附带 `proposal_status`）、`"NO_FEASIBLE_TASK"`、`"OUT_OF_FIELD"`、`"NO_DIRECTION"`、`"PROPOSAL_REACHED"`、`"NO_LEGAL_PROJECTION"` 或 `"INVALID_MODEL_OUTPUT"`。
  - `subgoal_refined` 与 `subgoal_reason`：**离线 RL 只细化不篡改合同的审计字段**。仲裁选出胜出任务后，投影合法子目标仅允许细化**非紧急 `SUPPORT` / `DEFEND` 任务**的目标坐标（须落在该任务所属区域内或距原规则目标点 `<= goal_update_distance [1.5 m]`、连通可达，且通过冷却与友军间距 `ally_clearance` 检查）。细化成功时 `subgoal_refined=true`、`subgoal_reason="REFINED"`（或在提议暂不可导引时保持已细化点 `"KEPT:<status>"`），并记录 `subgoal` 与原规则点 `subgoal_base`；未细化时 `subgoal_refined=false`，`subgoal_reason` 明确给出 `"NO_RULE_TASK"`、`"URGENT_TASK"`、`"NOT_REFINABLE:INTERCEPT"`、`"NOT_REFINABLE:RECOVER"`、`"OUTSIDE_TASK_AREA"`、`"UNREACHABLE"`、`"FAILURE_COOLDOWN"`、`"ALLY_OCCUPIED"` 或当前 `status`。
  - `model_fallbacks`（仅当 `intent_model` 或 `utility_model` 发生安全回退时出现）：例如场地启用 `mirror: auto` 切换到蓝方（`own_team=BLUE`）后 `map_fingerprint` 改变，若仍传入仅在红方场地上训练的 HMM 或效用权重文件，节点不会报错停机，而是自动回退至规则意图估计器或战术文件权重，并在每帧 `model_fallbacks` 中保留完整原因。

---

## 5. 代码示例

### 5.1 示例 A：不依赖 ROS 2 的纯 Python 调用（`sentry_core`）

如果团队自研主控直接在 Python 进程内运行，或希望在无 ROS 2 环境下编写单元测试与仿真器闭环，可直接导入 `sentry_core`，构造 `WorldModel` 与强类型状态对象并调用 `DecisionEngine.step(world)`：

```python
"""Pure-Python SentryWeave example without ROS 2 dependencies."""

from sentry_core.config import load_config
from sentry_core.model import (
    SOURCE_ONBOARD,
    SOURCE_RADAR,
    SOURCE_REFEREE,
    GameState,
    Observation,
    Phase,
    RobotStatus,
    StructureState,
    Team,
    Vec2,
)
from sentry_core.policy import DecisionEngine
from sentry_core.tracking import WorldModel


def run_pure_python_demo() -> None:
    # Load RMUC 2027 tactics; BLUE automatically mirrors the field and sets self_id=107.
    config = load_config("src/sentry_bringup/config/tactics_rmuc2027.yaml", own_team="RED")
    assert config.self_id == 7

    world_model = WorldModel(config)
    engine = DecisionEngine(config)
    now = 10.0

    # 1. Normalized referee game status (phase + countdown + supply zone availability).
    world_model.update_game(
        GameState(
            stamp=now,
            season="rmuc2027",
            own_team="RED",
            phase=Phase.RUNNING,
            remaining=360.0,
            supply_available=True,
            source=SOURCE_REFEREE,
        ),
        received_at=now,
    )

    # 2. Robot statuses: Team is always relative to own_team (ALLY / ENEMY).
    world_model.update_robot(
        RobotStatus(
            robot_id=7,
            team=Team.ALLY,
            stamp=now,
            hp=400.0,
            max_hp=400.0,
            alive=True,
            ammo=0.80,
            source=SOURCE_REFEREE,
        ),
        received_at=now,
    )
    world_model.update_robot(
        RobotStatus(
            robot_id=3,
            team=Team.ALLY,
            stamp=now,
            hp=200.0,
            max_hp=250.0,
            alive=True,
            source=SOURCE_REFEREE,
        ),
        received_at=now,
    )
    # Partial radar estimate: current hp is known, max_hp and ammo remain None (unknown).
    world_model.update_robot(
        RobotStatus(
            robot_id=103,
            team=Team.ENEMY,
            stamp=now,
            hp=120.0,
            max_hp=None,
            source=SOURCE_RADAR,
        ),
        received_at=now,
    )

    # 3. Structure status: enemy_outpost alive is known even if exact hp is unknown (None).
    world_model.update_structure(
        StructureState(name="own_base", stamp=now, hp=5000.0, max_hp=5000.0, alive=True,
                       source=SOURCE_REFEREE),
        received_at=now,
    )
    world_model.update_structure(
        StructureState(name="enemy_outpost", stamp=now, alive=True, source=SOURCE_REFEREE),
        received_at=now,
    )

    # 4. Self pose + ally/enemy ground tracks in the map frame.
    world_model.observe(
        Observation(robot_id=7, team=Team.ALLY, position=Vec2(6.571, 7.5), stamp=now,
                    source_mask=SOURCE_ONBOARD),
        received_at=now,
    )
    world_model.observe(
        Observation(robot_id=3, team=Team.ALLY, position=Vec2(14.5, 4.2), stamp=now,
                    velocity=Vec2(1.1, -0.2), confidence=0.95, source_mask=SOURCE_RADAR),
        received_at=now,
    )
    world_model.observe(
        Observation(robot_id=103, team=Team.ENEMY, position=Vec2(7.8, 7.5), stamp=now,
                    velocity=Vec2(-1.2, 0.0), confidence=0.90, source_mask=SOURCE_RADAR),
        received_at=now,
    )

    # 5. Snapshot the world and step the decision engine.
    world = world_model.snapshot(now)
    decision = engine.step(world)
    task = decision.task
    print(
        f"task={task.kind.name} zone={task.zone_id} goal={task.goal} "
        f"reason={task.reason} arbitration={decision.arbitration}"
    )
    print("completeness:", decision.missing_info["completeness"])
    print("degraded:", decision.missing_info["degraded"])


if __name__ == "__main__":
    run_pure_python_demo()
```

### 5.2 示例 B：裁判系统串口 + 雷达站 ROS 2 Python 桥接骨架

以下骨架展示如何在 ROS 2 桥接节点中正确配置 **`RELIABLE` QoS**（用于比赛/机器人/建筑状态）与 **`qos_profile_sensor_data`**（用于位置观测），将官方红蓝机器人编号（`1..7` / `101..107`）转换为相对本车的 `ALLY` / `ENEMY`，并严格区分缺失字段有效位：

> **协议说明**：RoboMaster 官方裁判系统串口协议（帧头 `0xA5`、CRC8/CRC16 校验及各 `cmd_id` 字节偏移）与雷达站队内通信结构会随当季规则手册调整。请将贵队当季串口解包结果填入下方 `on_referee_packet` 与 `on_radar_tracks` 方法中。

```python
"""Skeleton ROS 2 bridge for referee serial packets and radar station tracks."""

import math

from builtin_interfaces.msg import Time
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sentry_interfaces.msg import (
    GameStatus,
    ObservationArray,
    RobotObservation,
    RobotStatus,
    RobotStatusArray,
    StructureState,
    StructureStatus,
)


# CRITICAL: world_model subscribes to state topics with RELIABLE + VOLATILE QoS.
# Never publish game/status, robots/status or structures/status with BEST_EFFORT!
LATEST_RELIABLE_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)
STATUS_ARRAY_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)


def ros_stamp(seconds: float) -> Time:
    sec, nanosec = divmod(round(seconds * 1_000_000_000), 1_000_000_000)
    return Time(sec=sec, nanosec=nanosec)


def relative_team(official_robot_id: int, own_team: str) -> int:
    """Convert official RoboMaster ID (1..7 red, 101..107 blue) to ALLY=1 / ENEMY=2."""
    is_blue_robot = official_robot_id >= 100
    own_is_blue = own_team.upper() == "BLUE"
    return RobotStatus.ALLY if (is_blue_robot == own_is_blue) else RobotStatus.ENEMY


class RefereeAndRadarBridge(Node):
    def __init__(self) -> None:
        super().__init__("referee_radar_bridge")
        self.own_team = self.declare_parameter("own_team", "RED").value.upper()
        self.season = self.declare_parameter("season", "rmuc2027").value
        self.full_ammo_allowance = float(
            self.declare_parameter("full_ammo_allowance", 400.0).value
        )

        self.game_pub = self.create_publisher(GameStatus, "game/status", LATEST_RELIABLE_QOS)
        self.robots_pub = self.create_publisher(
            RobotStatusArray, "robots/status", STATUS_ARRAY_QOS
        )
        self.structures_pub = self.create_publisher(
            StructureStatus, "structures/status", STATUS_ARRAY_QOS
        )
        self.tracks_pub = self.create_publisher(
            ObservationArray, "perception/tracks", qos_profile_sensor_data
        )

    def on_referee_packet(self, sample_time_sec: float, parsed: dict) -> None:
        """Map one decoded referee packet using its original sample_time_sec."""
        stamp = ros_stamp(sample_time_sec)

        # 1. GameStatus: leave *_valid=False for any field absent from this packet.
        game = GameStatus()
        game.header.stamp = stamp
        game.header.frame_id = "map"
        game.season = self.season
        game.own_team = GameStatus.BLUE if self.own_team == "BLUE" else GameStatus.RED
        if "stage" in parsed:
            stage_map = {
                0: GameStatus.READY,
                1: GameStatus.READY,
                2: GameStatus.READY,
                3: GameStatus.READY,
                4: GameStatus.RUNNING,
                5: GameStatus.FINISHED,
            }
            game.phase = stage_map.get(parsed["stage"], GameStatus.UNKNOWN)
            game.phase_valid = game.phase != GameStatus.UNKNOWN
        if "stage_remain_time" in parsed:
            game.remaining_time = float(parsed["stage_remain_time"])
            game.remaining_time_valid = True
        if "supply_available" in parsed:
            game.supply_available = bool(parsed["supply_available"])
            game.supply_valid = True
        game.source = GameStatus.SOURCE_REFEREE
        self.game_pub.publish(game)

        # 2. RobotStatusArray: hp_valid and max_hp_valid are independent!
        robots_msg = RobotStatusArray()
        robots_msg.header.stamp = stamp
        robots_msg.header.frame_id = "map"
        for item in parsed.get("robots", ())[:64]:
            robot = RobotStatus()
            robot.stamp = stamp
            robot.robot_id = int(item["robot_id"])
            robot.team = relative_team(robot.robot_id, self.own_team)
            if item.get("hp") is not None:
                robot.hp = float(item["hp"])
                robot.hp_valid = True
            if item.get("max_hp") is not None and item["max_hp"] > 0:
                robot.max_hp = float(item["max_hp"])
                robot.max_hp_valid = True
            if item.get("projectile_allowance") is not None and self.full_ammo_allowance > 0:
                ratio = float(item["projectile_allowance"]) / self.full_ammo_allowance
                robot.ammo_ratio = min(1.0, max(0.0, ratio))
                robot.ammo_valid = True
            else:
                robot.ammo_valid = False  # Enemy ammo is unknown, never 0!
            robot.source = RobotStatus.SOURCE_REFEREE
            robots_msg.robots.append(robot)
        if robots_msg.robots:
            self.robots_pub.publish(robots_msg)

        # 3. StructureStatus: StructureState.hp_valid covers BOTH hp and max_hp!
        structures_msg = StructureStatus()
        structures_msg.header.stamp = stamp
        structures_msg.header.frame_id = "map"
        for name, info in parsed.get("structures", {}).items():
            st = StructureState()
            st.stamp = stamp
            st.id = name  # "own_base", "own_outpost", "enemy_base", "enemy_outpost"
            hp, max_hp = info.get("hp"), info.get("max_hp")
            if hp is not None and max_hp is not None and max_hp > 0:
                st.hp = float(hp)
                st.max_hp = float(max_hp)
                st.hp_valid = True
            else:
                # If max_hp is unknown, keep hp_valid=False and report alive_valid instead.
                st.hp_valid = False
            if info.get("alive") is not None:
                st.alive = bool(info["alive"])
                st.alive_valid = True
            st.source = StructureState.SOURCE_REFEREE
            structures_msg.structures.append(st)
        if structures_msg.structures:
            self.structures_pub.publish(structures_msg)

    def on_radar_tracks(self, acquisition_time_sec: float, detections: list[dict]) -> None:
        """Publish radar ground positions with their original acquisition_time_sec."""
        array = ObservationArray()
        array.header.stamp = ros_stamp(acquisition_time_sec)
        array.header.frame_id = "map"
        for det in detections[:32]:
            x, y = float(det["x"]), float(det["y"])
            if not (math.isfinite(x) and math.isfinite(y)):
                continue
            obs = RobotObservation()
            obs.header.stamp = array.header.stamp
            obs.header.frame_id = det.get("frame_id", "map")
            obs.robot_id = int(det["robot_id"])
            obs.team = relative_team(obs.robot_id, self.own_team)
            obs.pose.pose.position.x = x
            obs.pose.pose.position.y = y
            obs.pose.pose.orientation.w = 1.0
            obs.velocity_valid = False
            obs.confidence = min(1.0, max(0.05, float(det.get("confidence", 0.85))))
            obs.source_mask = RobotStatus.SOURCE_RADAR
            array.observations.append(obs)
        if array.observations:
            self.tracks_pub.publish(array)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RefereeAndRadarBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
```

---

## 6. 延伸阅读

- **下游导航与控制栈对接**：三种接入模式（标准 Nav2 Action、`PoseStamped` 目标话题桥接、直接消费 `TacticalTask` 或纯 Python）详见 [接入自有导航栈](navigation-integration.md)。
- **实车联调与上车流程**：场地标定、`sentry-field` 红蓝双向校验、台架断连测试与离线 RL 影子模式上车详见 [实车上车教程](real-robot.md)。
- **内部原理与模型训练**：状态机与评分细节见 [系统架构](architecture.md)，离线策略特征与训练流程见 [离线强化学习](offline-rl.md)，ROS 启动与录包回放命令见 [ROS 接口与部署](deployment.md)。
