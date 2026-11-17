# 接入自有导航栈指南

SentryWeave 将**多机态势融合与战术决策（`world_model` + `decision`）**同**底盘路径规划与速度控制（`executor` + `velocity_guard`）**彻底解耦。无论车队已经具备成熟的 ROS 2 Jazzy Nav2 栈、基于点云分割与局部轨迹跟踪的 `PoseStamped` 自研规划器，还是在 C++/Python 主控中统一协调全向小陀螺与云台，都无需修改决策核心代码即可直接接入。

> **输入数据契约**：接入导航栈前，请确保定位、裁判系统与雷达站输入符合 [开放输入输出接口与缺失降级规范](interfaces.md)（特别是比赛/机器人/建筑状态的 `RELIABLE` QoS 要求）。实车标定与分阶段安全联调流程见 [实车上车教程](real-robot.md)。

---

## 1. 接入总览与三种模式选型

`decision.launch.py` 默认启动 `world_model`、`decision`、`executor`、`velocity_guard` 四个节点，并通过启动参数 `enable_executor` 与 `enable_velocity_guard` 支持按需裁剪执行层：

| 接入方式 | 适用团队 | `decision.launch.py` 参数 | 导航执行权与安全门控 |
|---|---|---|---|
| **方式一：标准 Nav2 Action（推荐）** | 使用 ROS 2 Jazzy Nav2（或兼容 `NavigateToPose` Action 的导航栈） | 默认（`enable_executor:=true enable_velocity_guard:=true`） | 内置 `executor` 独占管理取消/终态确认/预检，`velocity_guard` 门控 `nav_cmd_vel -> cmd_vel` |
| **方式二：目标点话题（`PoseStamped`）** | 自研全局/局部规划器接收 `geometry_msgs/PoseStamped` 目标点（如点云分割 + 局部轨迹跟踪栈） | `enable_executor:=false enable_velocity_guard:=false` | 运行本文提供的 ~35 行 `TaskToPoseBridge` 桥接节点，把 `tactical/task` 转为去重 `goal_pose` 与冷却反馈 |
| **方式三：仅输出任务 / 纯 Python 嵌入** | 在自研 C++/Python 主控中统一协调底盘、小陀螺转速与云台，或不使用 ROS | `enable_executor:=false enable_velocity_guard:=false`（或直接 `import sentry_core`） | 下游直接订阅 `tactical/task`（回传可选 `tactical/feedback`），或直接调用 `DecisionEngine.step(World)` |

三种模式的数据流分界如下：

```text
多源输入 (TF / tracks / game / robots / structures)
  └─► world_model (10 Hz world/state)
        └─► decision (2 Hz tactical/task + tactical/trace)
              ├─► [方式一] executor ──(NavigateToPose)──► 自有 Nav2 ──(nav_cmd_vel)──► velocity_guard ──► cmd_vel
              ├─► [方式二] TaskToPoseBridge ──(PoseStamped goal_pose / cancel_goal)──► 自研规划器 ──► 底盘
              └─► [方式三] 自研 C++/Python 主控直接订阅 tactical/task（或进程内调用 DecisionEngine.step）
```

---

## 2. 坐标系、TF 与云台自旋速度系约定

### 2.1 标准坐标链与车体帧名（`map -> odom -> base_link`）

- **世界坐标系（`map`）**：右手笛卡尔坐标系，单位为米与弧度。默认 RMUC 2026/2027 赛季场地以红方一侧左下角为 `(0, 0)`，`+x` 沿 `28 m` 长边指向蓝方，`+y` 沿 `15 m` 宽边，`+z` 竖直向上，平面航向角 `yaw` 从 `+x` 逆时针为正（无论红蓝阵营，世界系定义保持统一；蓝方由 `mirror: auto` 在加载配置时绕场地中心 `(14.0, 7.5)` 自动旋转 180°）。
- **标准 TF 树**：`map -> odom -> base_link`（帧名本身**不带**命名空间前缀，切勿写成 `sentry/map`）。
- **自定义底盘帧名**：若自有里程计或激光 SLAM（如 FAST-LIO2 / Point-LIO）发布的车体坐标系名为 `chassis`、`body` 或 `aft_mapped`，启动 `decision.launch.py` 时传入 `base_frame:=chassis` 即可让 `world_model` 查询 `map -> chassis`。
- **非 TF 定位接入**：若定位模块仅发布带协方差位姿话题而不维护 `map -> base_frame` TF，可设置 `self_from_tf:=false` 并将位姿发布到命名空间内的 `self_pose`（`geometry_msgs/PoseWithCovarianceStamped`，`SensorDataQoS`）。

### 2.2 命名空间 TF 重映射（`/tf -> tf` 即 `/sentry/tf`）

`decision.launch.py`（及仓库参考 `navigation.launch.py`）默认包含重映射 `[("/tf", "tf"), ("/tf_static", "tf_static")]`：在默认 `namespace:=sentry` 下，`world_model` 实际监听的是 **`/sentry/tf`** 与 **`/sentry/tf_static`**。

如果车队现有的 SLAM / Point-LIO / FAST-LIO2 定位节点将变换发布在**全局 `/tf` 与 `/tf_static`**，任选以下一种方式对齐，否则 `world_model` 将因查不到 `map -> base_frame` 而持续处于 `SAFE_STOP`（`HOLD: LOCALIZATION_INVALID`）：

1. **全局命名空间启动决策链（最简）**：启动 `decision.launch.py` 时传入 `namespace:=""`，所有话题与 `/tf` 均直接使用根命名空间。
2. **保持 `/sentry` 命名空间并对齐 TF**：将定位/导航节点的 `/tf`、`/tf_static` 重映射到 `/sentry/tf`、`/sentry/tf_static`，或在自定义顶层 Launch 文件中移除/覆盖 TF 重映射，确保在对应 TF 树上能查通变换：
   ```bash
   # 核对 /sentry/tf 下能否查到非零时间戳的 map -> base_link 变换
   ros2 run tf2_ros tf2_echo map base_link \
     --ros-args -r /tf:=/sentry/tf -r /tf_static:=/sentry/tf_static
   ```

### 2.3 全向小陀螺自旋与速度参考系约定

- **决策层职责边界**：`decision` 只输出 `map` 系下的目标平面位姿 `TacticalTask.target_pose`（含期望驻守朝向 `yaw`），不直接接管云台 `pitch/yaw` 瞄准或小陀螺转速解算。
- **速度门控职责边界**：内置 `velocity_guard`（源码 `src/sentry_ros/sentry_ros/guard_node.py`）假设输入的 `nav_cmd_vel` 与输出的 `cmd_vel` 位于同一车体平面速度系，按 `hypot(linear.x, linear.y) <= max_linear`（节点默认 `1.5 m/s`）等比例限幅线速度，并按 `|angular.z| <= max_angular`（默认 `2.0 rad/s`）截断角速度。
- **云台随动系 / 虚拟航向系（如 `fake_vel_transform`）注意事项**：实车哨兵常采用虚拟随动帧（如 `gimbal_yaw` 或不随底盘高速旋转的 `chassis_fake`）运行 Nav2 局部控制器，再在末端按云台—底盘实时夹角把平移速度分解到底盘轮系并叠加小陀螺自旋角速度 `wz_spin`。采用该架构时：
  1. 推荐让 Nav2 在虚拟不自旋帧输出平移速度 `nav_cmd_vel -> velocity_guard -> cmd_vel`，再由下游底盘解算节点读取受门控的 `cmd_vel`（同时订阅 `motion/permit` 或在 `cmd_vel` 为零且非战斗驻守时关停平移）完成旋转矩阵投影与小陀螺叠加；
  2. 若将 `velocity_guard` 放在已叠加小陀螺高角速度（例如 `6–12 rad/s`）之后的底盘帧，务必同步标定调大 `velocity_guard` 的 `max_angular` 与 `max_linear` 参数，避免正常自旋角速度被截断在 `2.0 rad/s`。

---

## 3. 方式一详解：接入自有 Nav2 栈

如果车队已调通适配本车全向/差速底盘的 ROS 2 Jazzy Nav2 栈，**只需单独启动 `decision.launch.py`**，切勿重复运行仓库自带的参考 `navigation.launch.py` 或 `robot.launch.py`（后两者会额外拉起一套参考 Nav2 实例造成冲突）。

### 3.1 启动命令示例

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# 若自有 Nav2 运行在 /sentry 命名空间：
ros2 launch sentry_bringup decision.launch.py \
  namespace:=sentry \
  config_file:=/abs/path/tactics_rmuc2027.yaml \
  own_team:=RED \
  base_frame:=base_link \
  require_localization_health:=true \
  trace_path:=/abs/path/logs/match.jsonl

# 若自有 Nav2 与定位运行在根命名空间（/navigate_to_pose、全局 /tf）：
ros2 launch sentry_bringup decision.launch.py \
  namespace:="" \
  config_file:=/abs/path/tactics_rmuc2027.yaml \
  own_team:=RED \
  require_localization_health:=true
```

> **提示**：开启 `require_localization_health:=true` 后，定位监测节点必须以 `5–20 Hz`、`RELIABLE` QoS 向 `localization_ok`（`std_msgs/Bool`）持续发布 `true`；一旦超过 `0.5 s` 未更新或收到 `false`，`world_model` 会立即进入 `SAFE_STOP` 撤销运动许可。

### 3.2 Action 与底盘速度链对接表

请将自有 Nav2 栈的 Action 名称与速度话题按以下契约对齐（参考配置见 [nav2.yaml](../src/sentry_bringup/config/nav2.yaml) 与 [navigation.launch.py](../src/sentry_bringup/launch/navigation.launch.py)）：

| 通道名称（相对 `namespace`） | ROS 类型 | 对接关系 | 关键要求 |
|---|---|---|---|
| `navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | `executor` (Client) `-> bt_navigator` (Server) | 命名空间内**仅允许 `executor` 一个客户端发单** |
| `compute_path_to_pose` | `nav2_msgs/action/ComputePathToPose` | `executor` (Client) `-> planner_server` (Server) | `check_path_before_navigate:=true`（默认）时用于发单前路径合法性预检；默认请求 `planner_id="GridBased"`（可通过 `executor` 参数 `-p planner_id:=...` 修改） |
| `cmd_vel_raw -> cmd_vel_smoothed -> nav_cmd_vel` | **`geometry_msgs/msg/Twist`** | `controller_server` / `behavior_server` `-> velocity_smoother -> collision_monitor` | 将自有导航栈平滑与碰撞监控后的最终输出重映射或配置为 **`nav_cmd_vel`**（勿直接写 `cmd_vel`！） |
| `motion/permit` | `sentry_interfaces/msg/MotionPermit` | `executor -> velocity_guard`（20 Hz 单调时钟） | 内部自动连接；超时 `permit_timeout=0.3 s` |
| `nav_cmd_vel -> cmd_vel` | **`geometry_msgs/msg/Twist`** | `collision_monitor -> velocity_guard ->` 底盘驱动（50 Hz 单调时钟） | `nav_cmd_vel` 超时 `command_timeout=0.25 s` 或 `motion/permit.allowed=false` 时立即向 `cmd_vel` 输出零速 |

> **显式使用 `Twist` 而非 `TwistStamped`**：ROS 2 Jazzy Nav2 支持切换 `TwistStamped`，但 SentryWeave 全链路（含 `velocity_guard` 输入 `nav_cmd_vel` 与输出 `cmd_vel`）统一显式使用非带戳的 **`geometry_msgs/msg/Twist`**。请在自有 Nav2 的 `controller_server`、`behavior_server`、`velocity_smoother` 与 `collision_monitor` 中确保 `enable_stamped_cmd_vel: false`。

### 3.3 `check_path_before_navigate` 与膨胀半径对齐

当 `check_path_before_navigate:=true`（默认值）时，`executor` 在向 `NavigateToPose` 发送每个新目标前，会先调用 `ComputePathToPose` 获取全局路径，并用 `arena.legal(p)` 与 `arena.segment_legal(a, b)` 逐点逐段检验**整条全局路径与场地边界、战术障碍物（`obstacles`）及规则禁区（`keepouts`）的距离是否均 `>= robot_radius`**（例如 `tactics_rmuc2027.yaml` 中 `robot_radius: 0.35 m`）。

接入自有 Nav2 时常见两种不匹配情形及处理方式：

1. **自有全局代价图膨胀半径偏小导致 `NO_LEGAL_PATH`**：若自有 `global_costmap` 的 `robot_radius` 或 `inflation_radius` 小于战术配置中的 `robot_radius`（例如代价图允许路径贴墙 `0.28 m` 绕行，而战术图要求 `>= 0.35 m`），`executor` 会判定该全局路径触碰膨胀边界并返回 `TaskFeedback.FAILED (NO_LEGAL_PATH)`，触发候选点轮换冷却。
   - **推荐做法**：调齐自有 `global_costmap` / `local_costmap` 的 `robot_radius` 与 `inflation_radius`（参考 `nav2.yaml` 中 `inflation_radius: 0.6`、禁区膨胀 `0.7`），并按 §3.4 挂载禁区掩码。
2. **自有导航栈未开放 `compute_path_to_pose` 或希望完全交由 Nav2 代价图避障**：可在启动 `decision.launch.py` 时关闭路径预检：
   ```bash
   ros2 launch sentry_bringup decision.launch.py \
     config_file:=/abs/path/tactics_rmuc2027.yaml \
     check_path_before_navigate:=false
   ```
   设为 `false` 后，`Executor` 状态机跳过 `PLANNING` 直接进入 `SENDING`（`NavigateToPose`）；**目标点本身依然经过严格的 `arena.legal(task.goal)` 校验**（非法或落入禁区的目标仍会被立即 `REJECTED`），中途绕障与动态避障则完全交由自有 Nav2 代价图与局部控制器负责。

### 3.4 将规则禁区（`KeepoutFilter`）接入自有 Costmap

**仅保证战术目标点合法是不够的**：比赛场地中的敌方补给区（`enemy_supply`）、敌方集结区（`enemy_assembly`）等规则禁区在地面上并无实体挡板，激光雷达无法扫出障碍。若不在 Nav2 全局与局部代价图中加载禁区掩码，机器人在动态绕障或恢复行为（`backup`）时极易横穿规则禁区判罚！

SentryWeave 在 `sentry_core.keepout` 中提供了零 ROS 依赖的栅格化函数 `render_keepout(config, map_yaml, output_directory)`，能够根据当前 `own_team`（`BLUE` 自动应用 180° 镜像）将越界区域、`obstacles` 与 `keepouts` 渲染为与导航静态图完全对齐的 `keepout.pgm` + `keepout.yaml`：

```python
"""Generate a team-aligned Nav2 KeepoutFilter mask from season tactics and map.yaml."""

from pathlib import Path

from sentry_core.config import load_config
from sentry_core.keepout import render_keepout


def build_team_keepout_mask(
    tactics_yaml: str,
    nav_map_yaml: str,
    out_dir: str,
    own_team: str = "RED",
) -> Path:
    # When own_team="BLUE" and mirror="auto", obstacles and keepouts are mirrored 180 deg.
    config = load_config(tactics_yaml, own_team=own_team)
    keepout_yaml = render_keepout(config, nav_map_yaml, out_dir)
    print(f"Generated keepout mask for {config.own_team} (self_id={config.self_id}): {keepout_yaml}")
    return keepout_yaml
```

生成 `keepout.yaml` 后，参考 [`src/sentry_bringup/launch/navigation.launch.py`](../src/sentry_bringup/launch/navigation.launch.py) 与 [`src/sentry_bringup/config/nav2.yaml`](../src/sentry_bringup/config/nav2.yaml) 将其挂入自有 Nav2：

1. 启动 `nav2_map_server`（节点名 `keepout_mask_server`）加载生成的 `keepout.yaml` 发布到 `keepout_mask` 话题，并启动 `costmap_filter_info_server`（`type: 0`）发布 `costmap_filter_info`；
2. 在 **`global_costmap` 与 `local_costmap`** 中同时启用 `KeepoutFilter` 及紧随其后的独立 `InflationLayer`（因为 `render_keepout` 只覆盖相交栅格单元，车体半径安全膨胀由滤波器后的 `keepout_inflation` 统一施加）：
   ```yaml
   filters: ["keepout_filter", "keepout_inflation"]
   keepout_filter:
     plugin: nav2_costmap_2d::KeepoutFilter
     enabled: true
     filter_info_topic: /sentry/costmap_filter_info
   keepout_inflation:
     plugin: nav2_costmap_2d::InflationLayer
     inflation_radius: 0.7
     cost_scaling_factor: 4.0
   ```

### 3.5 独占导航所有权（Single-Owner）与故障恢复

内置 `executor` 对所选命名空间下的 `navigate_to_pose` 实施严格的单所有者协议：

1. **启动排空（`WAITING -> CANCELING -> DRAINING -> READY`）**：启动时向 `navigate_to_pose/_action/cancel_goal` 发送全零 UUID 取消所有遗留 Goal，并通过 `get_result` 确认全部旧 Goal 进入终态后才开放 `READY`；此期间 `motion/permit.allowed` 恒为 `false`。
2. **外来 Goal 熔断（`STARTUP_OWNERSHIP_FOREIGN_GOAL`）**：运行期间 `executor` 实时监听 `navigate_to_pose/_action/status`。一旦检测到非本执行器发出的活跃 Goal（例如调试人员在 RViz 点击了 **`2D Goal Pose`**、残留了旧赛季行为树节点、或手动执行了 `ros2 action send_goal`），`executor` 会立即触发 **`STARTUP_OWNERSHIP_FOREIGN_GOAL`** 锁死故障、主动取消当前导航，并将 `motion/permit.allowed` 置为 `false` 令 `velocity_guard` 立即锁零速！
3. **故障解除方式**：
   - **普通运行期锁定故障**（如暂时性 `PLANNING_TIMEOUT`、`SENDING_TIMEOUT`、`CLOCK_REWIND`，此时 `startup_state == "READY"`）：待定位与 Nav2 恢复正常且无未决导航后，调用复位服务即可恢复接单：
     ```bash
     ros2 service call /sentry/executor/reset_fault std_srvs/srv/Trigger "{}"
     ```
   - **所有权冲突故障**（`STARTUP_OWNERSHIP_FOREIGN_GOAL` 或启动排空失败，此时 `startup_state == "FAULT"`）：为防止未知外部客户端仍在后台发单，`/sentry/executor/reset_fault` 会拒绝原地复位并返回 `"Startup ownership unresolved; restore Nav2 and restart the executor"`。此时必须先关闭抢占发单的外部进程/RViz 工具，再重启 `executor` 重新完成启动排空确认。

---

## 4. 方式二详解：`PoseStamped` 目标话题桥接

许多 RoboMaster 哨兵队伍使用点云地面分割（如 Patchwork++ / terrain_analysis）加局部路径跟随器（如 `localPlanner` / FAR Planner / 自研 MPC），只接收 `geometry_msgs/msg/PoseStamped` 目标点话题并直接向底盘输出控制量，不使用 Nav2 `NavigateToPose` Action。

### 4.1 为什么必须关闭内置 `executor` 与 `velocity_guard`

在方式二下启动 `decision.launch.py` 时，**必须显式传入 `enable_executor:=false enable_velocity_guard:=false`**：

- 若保留 `enable_executor:=true`，内置 `executor` 会一直阻塞在等待 `navigate_to_pose` Action Server 的 `STARTUP_WAITING` 状态，并持续广播 `motion/permit.allowed = false`；
- 若保留 `enable_velocity_guard:=true`，`velocity_guard` 会以 **50 Hz 单调时钟向 `cmd_vel` 持续发送零速 `Twist(0, 0, 0)`**，与自研局部控制器输出的底盘速度直接抢占冲突，导致底盘抽搐或无法移动！

启动决策链命令：

```bash
ros2 launch sentry_bringup decision.launch.py \
  namespace:=sentry \
  config_file:=/abs/path/tactics_rmuc2027.yaml \
  own_team:=RED \
  enable_executor:=false \
  enable_velocity_guard:=false \
  trace_path:=/abs/path/logs/match.jsonl
```

### 4.2 完整可运行桥接节点源码（`task_to_pose_bridge.py`）

将以下脚本保存为 `task_to_pose_bridge.py` 并运行。该节点完整实现了：
1. 以 `RELIABLE` + `KeepLast(1)` 订阅 `tactical/task`；
2. 记录 `last_identity = (msg.session_id, msg.task_id)`，**过滤 2 Hz 续租心跳**，仅在任务首次下发、切换或动态目标移动超 `1.5 m`（`task_id` 递增）时向 `goal_pose` 发布一次 `PoseStamped`，防止反复重置自研规划器；
3. 当收到 `TacticalTask.HOLD`、`!msg.target_pose_valid` 或 `valid_until` 租约超时（默认 `2.0 s`）时，向 `cancel_goal` 发布停车/取消信号；
4. 可选订阅自研规划器的不可达/卡死信号 `planner_failed`，向 `tactical/feedback` 回传 `TaskFeedback.FAILED` 以激活决策核心 `5.0 s` 失败点自动冷却（含离线 RL 子目标无损回退）。

```python
#!/usr/bin/env python3
"""Bridge SentryWeave TacticalTask leases to geometry_msgs/PoseStamped goals."""

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sentry_interfaces.msg import TacticalTask, TaskFeedback
from std_msgs.msg import Empty

LATEST = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.VOLATILE)
FEEDBACK = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.VOLATILE)


class TaskToPoseBridge(Node):
    def __init__(self):
        super().__init__("task_to_pose_bridge")
        self.last_identity, self.valid_until, self.moving = None, 0.0, False
        self.goal_pub = self.create_publisher(PoseStamped, "goal_pose", LATEST)
        self.cancel_pub = self.create_publisher(Empty, "cancel_goal", LATEST)
        self.feedback_pub = self.create_publisher(TaskFeedback, "tactical/feedback", FEEDBACK)
        self.create_subscription(TacticalTask, "tactical/task", self._on_task, LATEST)
        self.create_subscription(Empty, "planner_failed", self._on_failed, LATEST)
        self.create_timer(0.1, self._check_lease)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _stop(self) -> None:
        if self.moving:
            self.cancel_pub.publish(Empty())
            self.moving = False

    def _on_task(self, msg: TacticalTask) -> None:
        until = msg.valid_until.sec + msg.valid_until.nanosec / 1e9
        if msg.task_type == TacticalTask.HOLD or not msg.target_pose_valid or until <= self._now():
            self.last_identity = (msg.session_id, msg.task_id)
            return self._stop()
        self.valid_until = until
        identity = (msg.session_id, msg.task_id)
        if identity != self.last_identity:
            self.last_identity, self.moving = identity, True
            self.goal_pub.publish(msg.target_pose)

    def _check_lease(self) -> None:
        if self.moving and self._now() >= self.valid_until:
            self.last_identity = None
            self._stop()

    def _on_failed(self, _: Empty) -> None:
        if not self.moving or self.last_identity is None:
            return
        self._stop()
        fb = TaskFeedback(session_id=self.last_identity[0], task_id=self.last_identity[1],
                          execution_state=TaskFeedback.FAILED, reason_code="CUSTOM_PLANNER_FAILED")
        fb.header.stamp, fb.header.frame_id = self.get_clock().now().to_msg(), "map"
        self.feedback_pub.publish(fb)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TaskToPoseBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
```

启动桥接节点并通过 ROS 2 重映射对接自研规划器话题：

```bash
python3 task_to_pose_bridge.py --ros-args \
  -r __ns:=/sentry \
  -r goal_pose:=/way_point_goal \
  -r cancel_goal:=/stop_command \
  -r planner_failed:=/planner_stuck
```

> **安全提醒**：方式二关闭了内置 `velocity_guard`，请务必在自研局部控制器与底盘下位机中保留硬件级指令超时看门狗，确保收到 `cancel_goal` 或上位机断连时立即刹停底盘平移。

---

## 5. 方式三详解：直接消费 `TacticalTask` 或纯 Python 调用 `sentry_core`

### 5.1 自研 C++ / Python 主控直接订阅 `tactical/task`

如果车队拥有统一的兵种主控进程（例如在同一状态机内联动底盘轨迹规划、高低速小陀螺切换、云台扫描扇区与摩擦轮状态），推荐以 `enable_executor:=false enable_velocity_guard:=false` 启动 `decision.launch.py`，在主控中直接订阅 `/sentry/tactical/task`（及可选 `/sentry/threat/state`、`/sentry/team/intents`）：

1. **订阅 QoS**：`ReliabilityPolicy::RELIABLE` + `DurabilityPolicy::VOLATILE` + `KeepLast(1)`。
2. **去重续租**：以 `(session_id, task_id)` 作为唯一任务标识。标识不变时仅刷新本地租约截止时间 `valid_until`（不打断正在执行的轨迹）；标识变化时切换目标位姿 `target_pose`。
3. **语义联动**：
   - `task_type == HOLD (0)` 或 `!target_pose_valid` 或当前时钟 `>= valid_until`：立即令底盘驻停（可根据 `reason_code` 区分赛前 `GAME_NOT_RUNNING` 禁动与赛中驻守）；
   - `task_type` 取 `SUPPORT (1)` / `INTERCEPT (2)` / `DEFEND (3)` / `RECOVER (4)`，结合 `urgent`、`target_robot_id`（`target_robot_valid`）与 `target_zone_id`，主控可自主分配平移功率与小陀螺角速度预算，并将云台预瞄扇区指向关联目标或高威胁防区。
4. **闭环反馈（可选但强烈推荐）**：当局部规划判定目标点被动态障碍堵死时，向 `/sentry/tactical/feedback`（`RELIABLE`, `KeepLast(20)`）发送一条具有相同 `session_id` 与 `task_id`、`execution_state = FAILED (5)` 的 `TaskFeedback`，决策引擎会自动将该 `goal_id` 冷却 `5.0 s` 并立即下发次优可达点。

### 5.2 不依赖 ROS 2 的纯 Python 进程内调用（`sentry_core`）

`sentry_core` 仅依赖 `numpy` 与 `PyYAML`，不导入任何 `rclpy` 模块。若车队主控使用自定义消息总线（如 ZeroMQ、LCM、共享内存或纯 Python 进程），可直接在主循环中实例化 `WorldModel` 与 `DecisionEngine`：

```python
"""Embed SentryWeave directly in a pure-Python master loop without ROS 2."""

from sentry_core.config import load_config
from sentry_core.execution import VelocityGate
from sentry_core.model import GameState, Observation, Phase, RobotStatus, Team, Vec2
from sentry_core.policy import DecisionEngine
from sentry_core.tracking import WorldModel


def step_embedded_controller() -> None:
    config = load_config("src/sentry_bringup/config/tactics_rmuc2027.yaml", own_team="BLUE")
    assert config.self_id == 107 and config.mirrored is True

    world_model = WorldModel(config)
    engine = DecisionEngine(config)
    gate = VelocityGate(command_timeout=0.25, permit_timeout=0.30, max_linear=1.5, max_angular=2.0)

    now = 20.0
    world_model.update_game(
        GameState(stamp=now, season="rmuc2027", own_team="BLUE", phase=Phase.RUNNING,
                  remaining=300.0, supply_available=True),
        received_at=now,
    )
    world_model.update_robot(
        RobotStatus(robot_id=107, team=Team.ALLY, stamp=now, hp=400.0, max_hp=400.0,
                    alive=True, ammo=0.9),
        received_at=now,
    )
    world_model.observe(
        Observation(robot_id=107, team=Team.ALLY, position=Vec2(21.429, 7.5), stamp=now),
        received_at=now,
    )

    decision = engine.step(world_model.snapshot(now))
    task = decision.task
    if task.kind.name != "HOLD" and task.goal is not None:
        gate.permit(True, now)
        gate.command(0.8, 0.0, 0.0, now)
        # If the custom planner reports the goal blocked, trigger 5.0 s goal cooldown:
        engine.feedback(task.session_id, task.task_id, "FAILED", now + 0.1)
    vx, vy, wz = gate.output(now)
    print(f"task={task.kind.name} goal={task.goal} cmd=({vx:.2f}, {vy:.2f}, {wz:.2f})")


if __name__ == "__main__":
    step_embedded_controller()
```

更完整的多源状态输入（含队友/敌方轨迹、建筑状态与缺失有效位处理）见 [开放输入输出接口 §5.1](interfaces.md#51-示例-a不依赖-ros-2-的纯-python-调用sentry_core)。

---

## 6. 观测起点吸附 vs 目标严格合法（为什么贴墙不会误锁 `LOCALIZATION_INVALID`）

实车与高保真物理仿真中经常遇到一个几何边界矛盾：战术配置设 `robot_radius: 0.35 m` 作为安全规划裕度，但哨兵在高速过弯、斜坡上下行、轮系打滑或激光定位存在 `2–4 cm` 抖动时，车体中心实测坐标很容易短暂落到距路沿或台阶多边形 `0.31 m`（`< 0.35 m`）处。

如果对“当前观测起点”与“待下发目标点”套用同一把标尺，一旦车体轻微贴边就会触发定位非法保护而锁死原地，永远无法自主驶离路沿。为此，`sentry_core/geometry.py` 严格区分了两类几何判据：

| 检查对象 | 调用方法 | 几何判据与行为 |
|---|---|---|
| **所有候选目标与执行目标**（规则偏好点、动态候选点、RL 细化子目标、`Executor._valid_task`） | `arena.legal(goal)` | **严格合法**：目标坐标必须位于 `bounds` 内缩 `robot_radius` 范围内，且严格落在所有按 `robot_radius` 外扩后的物理障碍（`obstacles`）与规则禁区（`keepouts`）凸多边形之外；绝不向墙体内部、膨胀边缘或规则禁区下发目标 |
| **实测机器人当前位姿**（本机 `self_track.position` 与敌我轨迹区域路由起点） | `arena.origin(pos)` | **边界内起点吸附**：只要 `arena.in_bounds(pos)` 为真（仍在 `[0, 28] × [0, 15] m` 场地外框内且数值有限）：<br>① 若本身满足 `arena.legal(pos)` 且位于自由连通栅格上，直接使用原坐标；<br>② 若因贴近路沿落入 `< robot_radius` 膨胀带，自动将其路由起点吸附到**最近的合法自由栅格中心**（分辨率 `grid_resolution = 0.1 m`），并把实测点到吸附点的精确欧氏补偿距离计入路径长度 |

因此：
- 实车贴近矮墙或坡道边缘 `< robot_radius` 时，**不会误报 `LOCALIZATION_INVALID`**，决策仍能正常规划安全的脱困或机动目标点；
- 仅当定位输出跑出场地 `bounds`、含 `NaN/Inf`、超时（`> self_timeout [0.5 s]`）或 `localization_ok=false` 时，才会立即进入 `HOLD(LOCALIZATION_INVALID)` 安全驻停。

---

## 7. 上线前 10 项核对清单与常见排障表

### 7.1 上线前 10 项核对清单

- [ ] **1. QoS 策略全绿**：`ros2 topic info -v` 确认 `game/status`、`robots/status`、`structures/status`、`localization_ok` 的发布端均为 **`RELIABLE`**（非 `BEST_EFFORT`），且 `Publisher count >= 1`。
- [ ] **2. TF 连通且时间戳新鲜**：在目标命名空间（`/sentry/tf` 或全局 `/tf`）能查通 `map -> base_frame`，变换时间戳随当前 ROS 时钟实时递增且延迟 `< 0.5 s`。
- [ ] **3. 红蓝阵营与自车 ID 一致**：启动参数 `own_team:=RED`（自动推导 `self_id=7`）或 `own_team:=BLUE`（自动推导 `self_id=107` 且 180° 镜像场地）与 `GameStatus.own_team` 一致；`RobotStatus.team` / `RobotObservation.team` 填写的是相对本车的 `ALLY=1 / ENEMY=2`。
- [ ] **4. 原始采样时间戳真实**：串口与雷达桥接消息中的 `header.stamp` / `stamp` 使用真实采样时刻，未在定时转发循环中用 `clock.now()` 伪造刷新旧包。
- [ ] **5. 建筑与机器人有效位正确**：`RobotStatus` 独立设置 `hp_valid` 与 `max_hp_valid`；`StructureState` 在 `hp_valid=true` 时已同时填入非负 `hp` 与正数 `max_hp`（未知 `max_hp` 时改用 `alive + alive_valid=true`）。
- [ ] **6. 双边场地语义校验通过**：`.venv/bin/sentry-field <tactics.yaml> --own-team RED` 与 `--own-team BLUE` 均返回 `"ok": true, "problems": []`。
- [ ] **7. 规则禁区已挂入全局与局部代价图**：自有 Nav2 的 `global_costmap` 与 `local_costmap` 均已加载对应红蓝阵营的 `KeepoutFilter` 掩码与 `InflationLayer`（或自研规划器已导入等效禁区多边形）。
- [ ] **8. 执行层启动开关匹配选型**：方式一确认速度链走 `Twist`（`enable_stamped_cmd_vel: false`）并汇入 `nav_cmd_vel`；方式二/三已传 `enable_executor:=false enable_velocity_guard:=false`。
- [ ] **9. 导航动作无外部抢占**：自主比赛期间已关闭 RViz `2D Goal Pose`、旧行为树节点及手工发单脚本，保证 `executor` 独占 `navigate_to_pose`。
- [ ] **10. 底盘硬件看门狗实测有效**：架空轮系或低速场测下，人为断开上位机通信或停止 `velocity_guard` / 自研控制器时，下位机能在百毫秒级内切断底盘电机输出。

### 7.2 常见故障现象与排障表

| 症状 / 原因码 | 根因定位 | 诊断命令与修复措施 |
|---|---|---|
| 话题 `echo` 看似有发布，但 `world/state` 无比赛/血量数据，任务停在 **`GAME_STATE_UNKNOWN`** | 上游桥接节点用 `BEST_EFFORT` 发布 `game/status` 或 `robots/status`，被 `world_model` 的 `RELIABLE` 订阅端静默拒收；或 `header.stamp` 与当前 ROS 时钟偏差 `> 2.0 s` | 执行 `ros2 topic info -v /sentry/game/status` 检查两端 Reliability 是否均为 `RELIABLE`；核对系统时钟 / `/clock` 同步 |
| `motion/permit.allowed` 恒为 `false`，`reason` 显示 **`STARTUP_WAITING`** | ① 自有 Nav2 未激活生命周期或运行在不同命名空间（如 `/navigate_to_pose` vs `/sentry/navigate_to_pose`）；<br>② 采用方式二/三自研规划器但忘记关闭内置 `executor` | ① 检查 `ros2 action list` 与 `ros2 lifecycle get /sentry/bt_navigator`；<br>② 非 Nav2 Action 接入必须传 `enable_executor:=false enable_velocity_guard:=false` |
| 任务频繁报 **`NO_LEGAL_PATH`**（`TaskFeedback.FAILED`），候选点依次进入 5 s 冷却 | ① 自有 Nav2 未提供 `compute_path_to_pose` 或 `planner_id` 不叫 `"GridBased"`；<br>② 自有全局代价图膨胀半径小于战术文件 `robot_radius`（`0.35 m`），规划出的贴墙路径未通过 `arena.legal(p)` 预检 | ① 修改 `executor` 参数 `planner_id`；<br>② 调齐 `global_costmap` 的 `inflation_radius` 并挂载 `KeepoutFilter`，或在 `decision.launch.py` 设置 `check_path_before_navigate:=false` |
| 运行中突然急停，`executor` 报错 **`STARTUP_OWNERSHIP_FOREIGN_GOAL`** | RViz `2D Goal Pose`、旧决策节点或命令行同时向 `navigate_to_pose` 发送了未授权 Goal，触发单所有者安全熔断 | 停止外部发单源，然后**重启 `executor` 节点**重新完成启动排空（注意：普通超时故障可调 `/sentry/executor/reset_fault` 复位，所有权冲突必须重启 `executor`） |
| 任务锁死在 `HOLD`，`reason_code` 为 **`TEAM_MISMATCH`** | `GameStatus.own_team`（`RED=1` / `BLUE=2`）与启动参数 `own_team`（`"RED"` / `"BLUE"`）相反；或误把 `RobotStatus.team` 按红=1/蓝=2 编码导致自车状态未被识别为 `ALLY` | 确保启动参数 `own_team` 与当场红蓝一致（未确定时 `GameStatus.own_team` 填 `TEAM_UNKNOWN=0`），并确保所有友军恒填 `ALLY=1`、敌军恒填 `ENEMY=2` |
| `world/state.mode` 为 **`SAFE_STOP`**，任务为 **`LOCALIZATION_INVALID`** | ① SLAM 发布在全局 `/tf` 而 `world_model` 监听 `/sentry/tf`，或车体帧名不是 `base_link`；<br>② 开启了 `require_localization_health:=true` 但未发 `RELIABLE` 的 `localization_ok=true`；<br>③ 定位坐标飞出 `bounds` `[0, 28] × [0, 15]` | ① 传 `namespace:=""` 或对齐 TF 重映射与 `base_frame:=...`；<br>② 检查 `ros2 topic echo /sentry/localization_ok` 频率 `>= 5 Hz`；<br>③ 核对建图原点与红方左下角 `(0, 0)` 对齐（注意轻微贴墙 `< robot_radius` 会自动吸附，不会触发此错误） |
