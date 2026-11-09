# ROS 接口与部署

## 运行前的坐标和时间约定

默认 namespace 为 `/sentry`，战场坐标为 `map`，车体为 `base_link`，里程计为 `odom`。
TF 主题为 `/sentry/tf` 与 `/sentry/tf_static`，帧名不重复添加 namespace。
设备驱动应与这个约定一致，或者在启动中显式重映射。`/clock` 始终全局。

位姿以米、角度以弧度、资源以 `[0,1]` 比例表示。源采样时刻必须在同一 ROS 时钟域；
不允许用每次转发的接收时刻刷新旧雷达数据。TF 转换使用观测时刻的变换，失败则丢弃，
不悄悄回退到“最新 TF”。移动坐标系中的速度不被当成世界速度，转换后从位置历史重估。

## 输入 Interface

以下路径都在所选 namespace 内：

| Topic / TF | 消息 | 约定 |
|---|---|---|
| `perception/tracks` | `sentry_interfaces/ObservationArray` | 每个元素有稳定 ID、阵营、源 stamp、frame、位姿、质量；速度可选 |
| `self_pose` | `geometry_msgs/PoseWithCovarianceStamped` | `self_from_tf=false` 时使用 |
| `map -> odom -> base_link` | TF | `self_from_tf=true` 时使用最新有时间戳的定位链 |
| `localization_ok` | `std_msgs/Bool` | 实车必须持续发布；false 或超时会撤销自身有效性 |
| `referee/state` | `sentry_interfaces/RefereeState` | phase 与各字段有效位必须明确设置 |
| `scan` | `sensor_msgs/LaserScan` | 默认 Nav2 障碍与碰撞监控输入 |
| `odom` | `nav_msgs/Odometry` | Nav2 的底盘里程计 |

阵营编码是相对己方的 `ALLY=1 / ENEMY=2`，不是红蓝颜色。ID 在各阵营内唯一；
同一 ID 的身份若不能保持，必须在上游完成关联。`self_id` 默认 7，应修改为实际哨兵 ID。
最多保留 32 个原始观测/数组及 128 个历史观测/轨迹，避免输入无限增长。

裁判适配器需要填写：

```text
header.stamp                 原始裁判状态时刻
phase + phase_valid          UNKNOWN / READY / RUNNING / FINISHED
hp_ratio + hp_valid
ammo_ratio + ammo_valid
own_outpost_alive + own_outpost_valid
enemy_outpost_alive + enemy_outpost_valid
supply_available + supply_valid
remaining_time + remaining_time_valid
```

这些是本工程的规范化 Interface，**不是 RoboMaster 官方串口数据包**。
裁判/雷达驱动应将设备消息映射到这些字段；缺失字段令有效位为 false。
官方字节协议和硬件驱动由部署方接入。可用字段、通信链路和更新频率以当季规则为准。

## 输出与控制

| Topic / Action / Service | 用途 |
|---|---|
| `world/state` | 带质量、年龄和原始轨迹历史的 World |
| `team/intents` | 概率意图，包含 UNKNOWN 和 valid |
| `threat/state` | 分防区威胁 |
| `tactical/task` | 最终经仲裁的有期限任务 |
| `tactical/feedback` | 接受、运行、取消、到位、失败 |
| `tactical/trace` | 完整 JSON 解释与回放数据 |
| `compute_path_to_pose` | Executor 的最新可达性预检 |
| `navigate_to_pose` | Executor 独占的 Nav2 目标 |
| `motion/permit` | 带会话和序号的运动许可心跳 |
| `executor/reset_fault` | `std_srvs/Trigger`，解除已可安全清除的执行故障 |
| `cmd_vel` | 最终给底盘的 Twist |

状态与任务使用 Reliable/Volatile/KeepLast(1)，原始传感器用 SensorData/BestEffort。
任务不使用 transient-local 重放。执行器用会话、递增 ID、有效期和 immutable 目标定义
处理丢包、重启及迟到消息。同一 ID 不能改变目标，仅能续租。

同时保留今年的决策节点、RViz Goal 控制或另一套 Action Client 会破坏独占假设；
运行自主决策时应让 Nav2 只接收本执行器的任务。遥控接管应在底盘控制权仲裁处实现，
并先撤销自主运动许可。
启动执行器意味着接管所选 namespace 的导航：它会取消该 Action Server 上遗留的 Goal，
等待全部终态后再开放新任务。启动清理超时或被拒绝时保持停止，并要求恢复 Nav2 后重启执行器。

## 实车启动

由设备驱动提供定位、里程计、障碍扫描、定位健康和规范化裁判/雷达数据后，在仓库根目录执行：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch sentry_bringup robot.launch.py \
  namespace:=sentry \
  map:=/absolute/path/season_map.yaml \
  config_file:=/absolute/path/season_tactics.yaml \
  trace_path:=/absolute/path/logs/match.jsonl
```

`robot.launch.py` 默认叠加 Jazzy MPPI Omni；`nav2.yaml` 本身是差速 RPP 参考配置。
控制器、机器人 radius/footprint、最大速度/加速度、速度门控上限和障碍配置必须按实车校准，
尤其是云台自旋时速度所在的参考坐标系。

若已有正常工作的 Jazzy Nav2，只启动 `decision.launch.py`；让原 Nav2 输出经平滑/碰撞监控后
进入 `nav_cmd_vel`，由 guard 独占 `cmd_vel`。规则禁区仍需接入原导航的 global/local costmap，
不能只检查战术目标点。仓库 `navigation.launch.py` 展示自动生成并加载禁区掩码的完整实现。

检查 Interface：

```bash
ros2 interface show sentry_interfaces/msg/ObservationArray
ros2 interface show sentry_interfaces/msg/RefereeState
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
  /sentry/perception/tracks /sentry/referee/state /sentry/self_pose \
  /sentry/world/state /sentry/team/intents /sentry/threat/state \
  /sentry/tactical/task /sentry/tactical/feedback /sentry/tactical/trace \
  /sentry/tf /sentry/tf_static /sentry/odom /sentry/scan /clock
```

做决策回放时，在新的 ROS domain 中只启动 decision：

```bash
export ROS_DOMAIN_ID=79
ros2 run sentry_ros decision --ros-args \
  -r __ns:=/sentry -p use_sim_time:=true \
  -p config_file:=/absolute/path/season_tactics.yaml \
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

1. 测量并固定 `map` 系、机器人 ID 与时间同步；记录更新频率和丢包分布。
2. 在 `tactics.yaml` 填写真实 bounds、障碍/禁区矩形、语义区域、候选点和恢复可用规则。
   多个矩形可以组成保守的禁区近似；当前实现不宣称自动理解任意场地语义。
3. 用真实地图验证全部候选点和绕行路径。场地中心、目标中心与可到达驻守点是不同概念。
4. 确认地图掩码、局部碰撞监控、速度门控、底盘硬件超时链实际生效。
5. 先用规则模式跑回放，再做低速实车场景；有真实标注与对照证据后再评估 HMM 或离线 RL。

离线 RL 参数与影子模式见 [离线强化学习](offline-rl.md)。
`rl_blend:=0.0` 只保持原规则任务选择，原规则仍可导航；需要完全离线检查时使用 JSONL 回放。
