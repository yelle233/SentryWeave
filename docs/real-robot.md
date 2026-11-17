# 实车上车教程

本教程面向已备好实车底盘、激光定位（或自有导航栈）、裁判串口与雷达站通信的战队工程人员，给出从**赛季场地标定校验 → 台架通信与掉线演练 → 四阶段安全上车（规则低速 → 影子模式 → 受限混合 → 双边模型校验）→ 机载耗时与日志体检**的分步实操闭环。

> **前置阅读**：
> - 上游话题字段、有效位与 **`RELIABLE` QoS 红线**：见 [开放输入输出接口与缺失降级规范](interfaces.md)；
> - 对接自有 Nav2、`PoseStamped` 规划器或自研 C++/Python 主控：见 [接入自有导航栈指南](navigation-integration.md)。

---

## 第一步：标定赛季场地语义与战术参数

SentryWeave 采用**“场地几何语义”与“车体战术参数”双文件分离**设计（以默认 RMUC 2027 配置为例）：

| 配置文件 | Schema 版本 | 职责范围与核心字段 |
|---|---|---|
| [`field_rmuc2027.yaml`](../src/sentry_bringup/config/field_rmuc2027.yaml) | `field_version: 1` | 以**红方视角**声明纯场地语义：`bounds`（`[0.0, 0.0, 28.0, 15.0]`）、物理不可通行障碍 `obstacles`（凸多边形或矩形）、规则禁区 `keepouts`（如 `enemy_supply`、`enemy_assembly`）、四座核心建筑 `structures`（`own_base`、`own_outpost`、`enemy_base`、`enemy_outpost`）、己方补给恢复区 `supply`、圆形或多边形战术区域 `zones`（覆盖 `DEFEND_BASE`、`ATTACK_OUTPOST`、高地与纵深跟进区 `enemy_front` 等 `CONTEST_AREA`）、非白名单偏好点 `points`，以及自动红蓝中心对称开关 `mirror: auto` |
| [`tactics_rmuc2027.yaml`](../src/sentry_bringup/config/tactics_rmuc2027.yaml) | `schema_version: 2` | 引用 `field: field_rmuc2027.yaml`，定义坐标系 `frame: map`、默认阵营 `own_team: RED`（故意省略 `self_id`，由系统按 **`RED -> 7`、`BLUE -> 107`** 自动推导）、可选目标点发布范围 `goal_scope`（`full` / `own_half_and_highland` / `own_half`）、实车外接圆半径 `robot_radius`、6 维效用权重 `weights`，以及超时/防抖/恢复阈值与期望地面人数 `parameters` |

### 1.1 关键标定原则（切勿把场地文件当成固定点位表）

1. **场地文件只声明语义，绝不限制移动范围**：
   `field_*.yaml` 中的 `points` 只是可选的额外偏好点（例如 `own_fortress`），**绝不是目标点白名单**！每个决策周期，`GoalGenerator` 会根据实时友军推进方向、敌方突袭方位与区域几何，在哨兵当前所在的合法连通自由空间内**动态生成拦截（`intercept`）、扇形布防（`defense`）、侧翼/后卫支援（`support`）、区域控制（`contest`）与补给恢复（`recovery`）候选点**，并自动投影到最近合法可达坐标。
2. **凹障碍物必须拆分为多个凸多边形**：
   可见图路由与向量化碰撞检测要求 `obstacles` 和 `keepouts` 中的每个多边形均为**严格凸多边形**。实测场地上遇到 L 形护栏、U 形掩体或带凸耳的高地台阶时，请沿内角拆成若干个相邻或重叠的凸四边形/凸多边形（参考 `field_rmuc2027.yaml` 中高地与隧道立柱的拆法）。若启用 `mirror: auto`，红蓝对称建筑与物理障碍需满足绕场地中心 `(14.0, 7.5)` 的对称公差 `mirror_tolerance`（默认 `0.05 m`）。
3. **实测车体外接圆半径填入 `robot_radius`**：
   用卷尺测量本队哨兵底盘（含防撞条/装甲板最外沿）中心到最远角点的外接圆半径并留出 `2–3 cm` 控制跟踪余量填入 `robot_radius`（参考值 `0.35 m`），同时与自有导航栈代价图的膨胀半径对齐。
4. **按当季赛制填写地面人数 `expected_allies` / `expected_enemies`**：
   在 `tactics_*.yaml` 的 `parameters` 段中，`expected_allies` 为**除本哨兵外**己方预期上场的地面机器人数（RMUC 常规为 `3`：英雄 + 双步兵或工程；RMUL 3v3 请按实际编制下调），`expected_enemies` 为敌方预期在场地面机器人数（RMUC 常规为 `4`）。两个参数必须为整数，用于准确计算雷达视野完整度 `missing_info.completeness`（已由裁判系统确认阵亡的机器人会自动从期望分母中扣除）。
5. **按底盘爬坡能力与赛场纪律配置目标发布范围 `goal_scope`**：
   可在 `tactics_*.yaml`（顶层或 `parameters:` 段）或 ROS 2 Launch 参数 `goal_scope:=...` 中从三档活动范围中任选（依据 `own_base` 相对场地中线 `mid_x` 的方位，对 `RED` 与 `mirror: auto` 下的 `BLUE` 自动对称生效）：
   - `full`（默认）：允许发布全场合法连通目标点（包括 `enemy_outpost` 破损后随队友推进至敌方半场 `enemy_front`）；
   - `own_half_and_highland`：仅允许己方半场与名称含 `highland` 的高地区域（如 `central_highland`），禁止跨中线进入敌方腹地；
   - `own_half`：仅允许己方半场平地且**禁止进入高地区域**（适合底盘不爬坡或稳守半场战术）。越界规则候选、越界 RL 子目标均会以 `OUTSIDE_GOAL_SCOPE` 拒绝，同时 `navigation.launch.py` 生成的 `keepout.pgm` 掩码也会将禁行半场/高地涂黑锁死。

### 1.2 运行 `sentry-field` 同时校验红蓝双方与连通可达性

修改场地或战术 YAML 后，上车前**必须同时校验 `RED` 与 `BLUE` 两个阵营**。`sentry-field` 会一次性检查多边形凸性、建筑完整性、`mirror: auto` 中心镜像对称性、偏好点碰撞，以及所有战术区域和偏好点在 `0.1 m` 膨胀自由栅格上的四连通可达性：

```bash
.venv/bin/sentry-field src/sentry_bringup/config/tactics_rmuc2027.yaml --own-team RED
.venv/bin/sentry-field src/sentry_bringup/config/tactics_rmuc2027.yaml --own-team BLUE
```

标准通过输出示例（重点核对 `"ok": true`、`"problems": []`，以及 `RED -> self_id: 7, mirrored: false` 与 `BLUE -> self_id: 107, mirrored: true`）：

```json
{
  "file": "src/sentry_bringup/config/tactics_rmuc2027.yaml",
  "own_team": "BLUE",
  "ok": true,
  "problems": [],
  "season": "rmuc2027",
  "site": "rmuc2027_arena1",
  "self_id": 107,
  "mirror": "auto",
  "mirrored": true,
  "bounds": [0.0, 0.0, 28.0, 15.0],
  "counts": {
    "obstacles": 75,
    "keepouts": 2,
    "structures": 4,
    "zones": 6,
    "points": 1
  }
}
```

> 若单独校验裸场地文件（如 `field_rmuc2027.yaml`），可附加 `--robot-radius 0.35` 指定校验所用的车体半径。只要返回码非 `0` 或 `problems` 非空，严禁直接上车！

---

## 第二步：接入裁判串口与雷达站并做台架掉线测试

### 2.1 台架安全准备与决策链启动

**安全红线**：首次联调外部定位、裁判串口与雷达站桥接节点时，**务必将哨兵底盘架空（四轮离地）或断开底盘电调动力电源**！

启动传感器/串口桥接节点与决策链（若此时尚未启动底盘导航栈，可先加 `enable_executor:=false enable_velocity_guard:=false` 聚焦验证态势融合与决策输出；传入 `own_team:=RED` 或 `own_team:=BLUE` 切换当场红蓝）：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch sentry_bringup decision.launch.py \
  namespace:=sentry \
  config_file:="$PWD/src/sentry_bringup/config/tactics_rmuc2027.yaml" \
  own_team:=RED \
  goal_scope:=own_half_and_highland \
  enable_executor:=false \
  enable_velocity_guard:=false \
  trace_path:=/tmp/bench_check.jsonl
```

### 2.2 核对 DDS QoS 匹配、自车编号推导与 `0x0120` / 小陀螺输出话题

另开终端，逐一检查核心输入与 2027 控制输出话题的节点连接与 QoS 策略：

```bash
ros2 topic info -v /sentry/game/status
ros2 topic info -v /sentry/robots/status
ros2 topic info -v /sentry/structures/status
ros2 topic info -v /sentry/perception/tracks
ros2 topic info -v /sentry/referee/sentry_cmd
ros2 topic info -v /sentry/chassis/spin
```

逐项确认：
1. `/sentry/game/status`、`/sentry/robots/status`、`/sentry/structures/status`（以及开启健康检查时的 `/sentry/localization_ok`）的发布端与订阅端均为 **`Reliability: RELIABLE`** 且 `Publisher count >= 1`（若发布端误用 `BEST_EFFORT`，`world_model` 将收不到任何状态消息！）；
2. `/sentry/perception/tracks` 为 `BEST_EFFORT`（`SensorDataQoS`），可同时承载常规裁判 `0x020B`（1 Hz 己方地面机器人 UWB 位置保底）、本机云台视觉（`SOURCE_ONBOARD=4`）与雷达站全局轨迹（`SOURCE_RADAR=2`）；
3. `/sentry/referee/sentry_cmd`（`sentry_interfaces/SentryRefereeCmd`）与 `/sentry/chassis/spin`（`std_msgs/Float32`）发布端均为 `RELIABLE`，串口发送桥接节点与云台/底盘随动主控的订阅 QoS 也应匹配；
4. 执行 `ros2 topic echo /sentry/world/state --once`，确认 `self_valid: true`、`self_robot.robot_id` 在 `own_team:=RED` 时自动为 `7`（在 `own_team:=BLUE` 时自动为 `107`），且队友/敌方 `team` 字段均正确映射为相对本车的 `ALLY (1)` / `ENEMY (2)`。

### 2.3 台架掉线、开赛跳变与 `0x0120` 姿态 / 响应式小陀螺四项必测演练

保持实时观察 `/sentry/tactical/trace`（单行 JSON 审计流）、`/sentry/referee/sentry_cmd` 与 `/sentry/chassis/spin`，在台架上依次验证以下四项赛场核心行为：

1. **拔掉雷达站通信线（保留常规机载裁判串口 `0x0001..0x020D`）**：
   - 核对 7 维信息完整度边界：常规机载裁判直连的 `ally_status`（`0x0003` 己方队友血量）、`structures`（`0x0003` 双方基地/前哨站血量）及 `0x020B` 1 Hz 友方位置保持可用，而依赖雷达视觉与 §5.7.4 信息波的 `enemies` 和 `enemy_status` 在超时后降至 `0.0`；
   - 在断流后 `0.9..5.0 s` 内，`missing_info.degraded` 出现 `STALE_ENEMY:<id>`；超过 `5.0 s` 后出现 **`ENEMY_POSITIONS_MISSING:defense+0.25`**（自动抬高基地防区驻防紧迫度）；若挂载离线 RL 模型，同步出现 **`RL_BLEND_REDUCED:INFO_INCOMPLETE:...`**（`effective_blend` 随视野完整度线性降至 `0.0` 影子模式）。
2. **停发雷达 §5.7.4 敌方估血或前哨站血量（置对应 `*_valid=false`）**：
   - 观察 `degraded` 记录 `ENEMY_HP_UNKNOWN:<id>` 与 `STRUCTURE_UNCONFIRMED:enemy_outpost`；
   - 确认决策将未知字段严格视为 `None`，既不会把未知血量敌车当作 `0` 血阵亡剔除，也不会把状态未确认的敌方前哨站当作已击毁而放弃推进。
3. **验证 `/sentry/referee/sentry_cmd`（`0x0120` 姿态切换与 `5.0 s` 冷却）**：
   - 无威胁巡航或驻守时，默认输出 `posture: "move"`（`posture_code: 3`，`raw_sentry_cmd = (3 << 21) | 1`），保留 `"attack"` 与 `"defense"` 各 `180.0 s` 未衰减额度；
   - 模拟高枪口热量（`0x0202` `heat_ratio >= 0.75`）或近敌受击，确认 `referee/sentry_cmd` 切换至 `"attack"`（`1`）或 `"defense"`（`2`），且在随后 **`5.0 s` 冷却窗口**（`policy_info.posture.cooldown_remaining > 0`）内严格锁定 `COOLDOWN_LOCKED` 不发生高频抖动；切回 `READY` 安全态时 `posture_enhanced` 立即强制归 `false`，绝不空耗 `3×15.0 s` 强化预算。
4. **验证 `/sentry/chassis/spin` 受击 `< 100 ms` 瞬时响应与 `READY` 安全归零**：
   - 在 `RUNNING` 阶段轻敲本机一块装甲板触发 `0x0206`（或模拟本机有效血量下降触发自动合成兜底）：确认 `decision` 跳过 `2 Hz` 等待，在 **`< 100 ms`** 内于 `/sentry/chassis/spin` 立即输出 **`+7.0` 或 `-7.0` `rad/s`**（`policy_info.spin.reason = "REACTIVE_HIT"`），并在持续受击或每隔 `2.2 s` 时自动正负变向；
   - 将 `GameStatus.phase` 切回 `READY`（或断开 `localization_ok`）：确认 `/sentry/tactical/task` 立即进入紧急 `HOLD`，**`/sentry/chassis/spin` 同一周期立即归零为 `0.0`**（`UNSAFE:GAME_NOT_RUNNING`）；再次切入 `RUNNING` 的首个周期以 `arbitration = "SAFETY_RECOVERED"` 零延迟恢复出击。

### 2.4 底盘小陀螺两种实车接线模式

| 实车下位机架构 | `velocity_guard` 参数配置 | 话题流向与安全边界 |
|---|---|---|
| **标准云台随动解算架构（推荐默认）** | `inject_spin_into_cmd_vel:=false`（默认） | `velocity_guard` 仅门控平动导航速度 `nav_cmd_vel -> cmd_vel`；下位机/云台随动节点直接订阅 `/sentry/chassis/spin` 叠加车体自旋并在云台陀螺仪回路做前馈解耦（安全 `HOLD` 时 `chassis/spin` 恒为 `0.0`） |
| **无独立随动解算层的全向底盘** | `inject_spin_into_cmd_vel:=true`（可调 `max_spin_angular:=8.0`、`spin_timeout:=1.0`） | `velocity_guard` 订阅 `/sentry/chassis/spin`，**仅在 `MotionPermit.allowed == True` 且许可未超时（`<= 0.3 s`）时**将限幅后的自旋角速度注入 `cmd_vel.angular.z`（到位停稳后仍可原地自旋驻守）；一旦 `MotionPermit` 撤销或超时，单调时钟看门狗立即强制向 `cmd_vel` 输出 `(0.0, 0.0, 0.0)` |

---

## 第三步：四阶段安全上车流程

台架数据链与掉线降级全部验证无误后，放下底盘进入场地，严格按以下四个阶段由浅入深放开控制权：

```text
阶段 1：纯规则低速验证 ──► 阶段 2：离线 RL 影子模式 (rl_blend=0.0)
                                    │
                                    ▼ 赛后 sentry-rl replay 离线复盘确认
阶段 4：双边 HMM/效用校验 ◄── 阶段 3：受约束混合模式 (rl_blend=0.25, rl_mirror=auto)
```

### 阶段 1：纯规则低速验证（核验坐标方向、禁区掩码与硬件看门狗）

首次场地带电行走时**不加载任何学习模型**（`rl_model:="" intent_model:="" utility_model:=""`），并将底盘速度上限收紧至安全低速：

1. 若使用内置 `velocity_guard`（方式一），注意 `max_linear`（默认 `1.5 m/s`）与 `max_angular`（默认 `2.0 rad/s`）是 `velocity_guard` 节点的启动 ROS 参数而非 `decision.launch.py` 顶层参数：首测需要临时压低至 `0.5 m/s` 与 `0.8 rad/s` 时，可在启动 `decision.launch.py` 时传 `enable_velocity_guard:=false`，再单独以 `ros2 run sentry_ros velocity_guard --ros-args -r __ns:=/sentry -p max_linear:=0.5 -p max_angular:=0.8`（或通过节点 YAML 参数文件 `--params-file`）启动门控节点；同时检查 Nav2 代价图已正确加载红/蓝阵营对应的 `KeepoutFilter` 规则禁区掩码；
2. 触发一次 `DEFEND` 或 `RECOVER` 任务，目视确认目标点在场地上的物理方位、车体移动方向及避障行为完全正确；
3. **必做底盘断连测试**：在低速移动中途人为断开上位机与底盘下位机通信（或停止速度发送节点），确认下位机硬件看门狗能在百毫秒级内可靠锁死零速（`velocity_guard` 是上位机最后一道软件门控，绝不能替代下位机硬件看门狗）。

### 阶段 2：离线 RL 影子模式上车（`rl_blend:=0.0`）

纯规则全速跑通后，切勿直接盲开 RL 权重！先以 **`rl_blend:=0.0`** 挂载导出的 `actor.json` 并开启 `trace_path` 跑完整场训练赛：

```bash
ros2 launch sentry_bringup decision.launch.py \
  namespace:=sentry \
  config_file:=/abs/path/tactics_rmuc2027.yaml \
  own_team:=RED \
  require_localization_health:=true \
  rl_model:=/abs/path/artifacts/rl/runs/iql-seed7/actor.json \
  rl_blend:=0.0 \
  rl_mirror:=auto \
  trace_path:=/abs/path/logs/match_shadow.jsonl
```

- **影子模式安全契约（v1 `73 -> 2` 与 v2 `85 -> 6` 双版本通用）**：当 `rl_blend:=0.0` 时，`policy_info.status` 固定为 `"SHADOW"`——**实车导航移动、2027 哨兵姿态与响应式小陀螺 100% 仍由阶段 1 已验证的规则策略驱动**（模型既不改变规则候选效用排序、不细化任何导航目标点，也不修改姿态或强化开关），纯 NumPy 策略仅在每帧同步计算短期位移建议（v2 模型还会将三姿态概率 `posture_probs` 与强化概率 `enhanced_prob` 写入 `match_shadow.jsonl` 的 `decision.policy_info`）。
- **赛后离线复盘**：训练赛结束后，在开发机上用 `.venv-rl/bin/sentry-rl replay` 重放实车录制的真实世界快照，零风险评估若开启 `blend=0.25` 时模型对候选排序与子目标细化的具体影响（注意显式指定实车所用的 `--config`，未传时 CLI 默认读取演示配置 `tactics.yaml`；若该局为蓝方对局，传入的战术 YAML 需同样设置 `own_team: BLUE` 以匹配轨迹中的 `map_fingerprint`）：

```bash
.venv-rl/bin/sentry-rl replay \
  --actor artifacts/rl/runs/iql-seed7/actor.json \
  --config src/sentry_bringup/config/tactics_rmuc2027.yaml \
  --trace /abs/path/logs/match_shadow.jsonl \
  --blend 0.25 \
  --mirror auto
```

### 阶段 3：受约束混合模式（`rl_blend:=0.25 rl_mirror:=auto`）

复盘确认模型提议方向与场地战术一致、无异常拉扯后，正式切换至受约束混合模式：

```bash
ros2 launch sentry_bringup decision.launch.py \
  namespace:=sentry \
  config_file:=/abs/path/tactics_rmuc2027.yaml \
  own_team:=BLUE \
  require_localization_health:=true \
  rl_model:=/abs/path/artifacts/rl/runs/iql-seed7/actor.json \
  rl_blend:=0.25 \
  rl_mirror:=auto \
  trace_path:=/abs/path/logs/match_blend.jsonl
```

上线混合模式时需掌握以下三项工程细则：

1. **严格的只细化不越权边界（导航 + v2 姿态受约束混合）**：离线 RL 先在相同推进步长上对可行规则候选做路线一致性打分混合；经防抖仲裁选出胜出任务后，投影合法子目标**仅允许微调非紧急 `SUPPORT` / `DEFEND` 任务的目标坐标**（必须落在该任务区域内或距原规则点 `<= 1.5 m`、连通可达，并通过冷却与友军 `0.85 m` 间距检查），**绝不改变胜出任务的类型、战术区域、关联机器人或紧急标志**。当挂载 v2（`85 -> 6`）模型且未触发重伤硬生存保护（`SURVIVAL_DEFENSE`）时，`PosturePolicy` 在 **`5.0 s` 切换冷却、`180.0 s` 衰减保护与 `3×15.0 s` 强化上限**规则屏障内按 `effective_blend` 混合模型姿态与强化偏好（安全 `HOLD` 或非交火/非远距紧急机动时严禁开启强化）。一旦细化导航点失败，引擎仅将 `rl_subgoal` 冷却 `5.0 s`，下一周期立即恢复原规则目标点。
2. **场地尺寸检查与跨赛季迁移说明**：
   - `tactics_rmuc2027.yaml` 与 `tactics_rmuc2026.yaml` 场地外框均为 **`28.0 × 15.0 m`**，与 RMUC 2026 官方离线数据集尺寸完全相同，因此启动时**无需**开启 `rl_allow_transfer:=true`（仅在 `20 × 12 m` 演示配置 `tactics.yaml` 上实验时才需传 `rl_allow_transfer:=true` 或 `--allow-domain-transfer`）。
   - 但需清醒认识：将基于 2026 区域赛数据训练的 `actor.json` 部署于 2027 赛季新地形属于**跨赛季地形迁移**，依赖 `TacticalMap` 可见图投影与规则屏障兜底；实车比赛推荐结合阶段 2 实录数据持续迭代验证。
3. **`rl_mirror` 红蓝自适应与 `ros2 run` 字符串引号写法**：
   - `rl_mirror` 支持 `"auto"`（默认，跟随场地 `mirror` 状态：蓝方镜像场地自动对 RL 观测特征与输出位移做 180° 中心对称变换，**同一个 `actor.json` 红蓝双方直接通用**）、`"true"` 与 `"false"`。
   - `decision.launch.py` 内部已声明 `value_type=str`；若不经 Launch 文件而直接用 **`ros2 run sentry_ros decision`** 传 `'true'` 或 `'false'`，**必须加内层单引号**防止 ROS 2 命令行将其解析为布尔型 `bool` 报错：
     ```bash
     ros2 run sentry_ros decision --ros-args \
       -r __ns:=/sentry \
       -p config_file:=/abs/path/tactics_rmuc2027.yaml \
       -p own_team:=BLUE \
       -p rl_model:=/abs/path/actor.json \
       -p rl_blend:=0.25 \
       -p "rl_mirror:='true'"
     ```

### 阶段 4：可选 HMM 意图模型 / 效用权重双边训练与 `model_fallbacks` 检查

除离线 RL 模型 `rl_model` 外，`decision` 还支持挂载可选的队友意图 HMM 模型（`intent_model`）与闭环调优效用权重（`utility_model`，训练流程见 [规则与学习基线](baselines.md)）。双边比赛部署这两类可选产物时请注意 **`map_fingerprint` 红蓝差异**：

- **为什么红蓝指纹不同**：`Config.map_fingerprint` 对场地多边形、建筑、区域与偏好点坐标计算 SHA-256。启用 `mirror: auto` 时，`own_team:=BLUE` 加载后的几何坐标已绕场地中心旋转 180°，因此**蓝方 `map_fingerprint` 与红方不同**。
- **预期内的安全回退（`MAP_FINGERPRINT_MISMATCH`）**：若将仅在红方配置下拟合的 `intent_model` 或 `utility_model` 直接用于 `own_team:=BLUE`，节点启动时**绝不会崩溃或阻断比赛**，而是打印告警日志：
  ```text
  [WARN] [sentry.decision]: intent_model falls back to rule mode (MAP_FINGERPRINT_MISMATCH): ...
  ```
  自动回退到内置规则意图估计器或 `tactics_*.yaml` 中的默认权重，并在每帧轨迹 `decision.policy_info["model_fallbacks"]` 中记录回退详情。在日志中看到该提示属于正常的安全保护。
- **红蓝双边生效做法**：若希望红蓝双方场次均启用学习到的 HMM 或效用权重，请分别基于红方配置（`own_team: RED`）和蓝方配置（`own_team: BLUE`）各训练导出一份产物（如 `intent_red.json` / `intent_blue.json`），在赛前启动脚本中随 `own_team` 对应选取；离线 RL 的 `actor.json` 则无需分红蓝重训，保持 `rl_mirror:=auto` 即可双边通用。

---

## 第四步：机载耗时与资源健康检查

### 4.1 建立车载电脑单步决策耗时基线

首次部署到车载计算平台（如 Intel NUC、算力板或工控机）时，直接运行仓库自带基准测试脚本测量本机 CPU 上的纯核心耗时与峰值堆内存：

```bash
.venv/bin/python tools/benchmark_decision.py --out /tmp/onboard-latency.json
```

若已导出离线 RL 模型，可附加 `--rl-model artifacts/rl/runs/iql-seed7/actor.json` 一并测量含 `GoalPrior` 推理与子目标投影的单步耗时。

健康参考范围（对照仓库实测基线 [`benchmarks/decision-latency.json`](../benchmarks/decision-latency.json) 与 [`benchmarks/runtime-footprint.json`](../benchmarks/runtime-footprint.json)，详见 [实验与性能基准](benchmarks.md)）：
- 在包含 **77 个障碍/禁区多边形、149 个可见图角点**的 RMUC 2027 场地上，`DecisionEngine.step` 正常单步耗时为**数毫秒至十余毫秒**（x86_64 基线 `p50 ≈ 2.4 ms`、`p95 ≈ 16.5 ms`），远小于 `2 Hz` 决策周期（**`500 ms`** 预算），纯核心单核 CPU 占比约 `1.2%`；
- 纯 NumPy 离线 RL 单次前向推理（`NumpyNavigationPolicy.predict`）：MLP（BC/IQL，约 `535 KB`）仅需微秒级（`~6–17 μs`），Decision Transformer（`context=8`，约 `2.8 MB`）不到 `1 ms`（`~0.29–0.73 ms`），车端无需安装 PyTorch 或占用自瞄 GPU；
- ROS 2 `decision` 节点在 `2 Hz` 在线运行（含 DDS 消息编解码与滚动 JSONL 写入）单核 CPU 占用约 `3.2%–3.7%`，常驻内存 RSS 约 `72–80 MB`。

### 4.2 赛后比赛日志一键健康检查脚本

每局比赛或训练赛结束后，可用以下 Python 脚本直接扫描 `trace_path` 生成的 JSONL 日志（滚动文件上限 `20 MB × 4`），自动检查机载单步耗时 `decision_ms` 是否逼近 `500 ms` 周期红线、统计各传感器链路平均完整度、汇总降级事件与模型指纹回退告警：

```python
#!/usr/bin/env python3
"""Summarize onboard decision latency, sensor completeness and fallbacks from match.jsonl."""

from collections import Counter
import json
from pathlib import Path
import sys

import numpy as np

from sentry_core.model import TaskType


def inspect_match_trace(trace_file: str | Path) -> dict:
    rows = [
        json.loads(line)
        for line in Path(trace_file).read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    if not rows:
        raise ValueError(f"Empty trace file: {trace_file}")

    latencies = [float(r["decision_ms"]) for r in rows if "decision_ms" in r]
    degraded_counts: Counter[str] = Counter()
    arbitrations: Counter[str] = Counter()
    tasks: Counter[str] = Counter()
    completeness_sums: Counter[str] = Counter()
    fallbacks: dict = {}

    for row in rows:
        dec = row["decision"]
        kind = dec["task"]["kind"]
        task_name = TaskType(int(kind)).name if isinstance(kind, int) else str(kind)
        tasks[task_name] += 1
        arbitrations[dec["arbitration"]] += 1
        missing_info = dec.get("missing_info") or {}
        for key, val in (missing_info.get("completeness") or {}).items():
            completeness_sums[key] += float(val)
        for tag in missing_info.get("degraded") or ():
            degraded_counts[tag.split(":", 1)[0]] += 1
        policy_info = dec.get("policy_info") or {}
        if "model_fallbacks" in policy_info:
            fallbacks.update(policy_info["model_fallbacks"])

    n = len(rows)
    summary = {
        "steps": n,
        "decision_ms": (
            {
                "p50": round(float(np.percentile(latencies, 50)), 2),
                "p95": round(float(np.percentile(latencies, 95)), 2),
                "max": round(float(max(latencies)), 2),
                "budget_ok": float(np.percentile(latencies, 95)) < 100.0,
            }
            if latencies
            else None
        ),
        "mean_completeness": {k: round(v / n, 3) for k, v in completeness_sums.items()},
        "tasks": dict(tasks),
        "safety_recovered_count": arbitrations.get("SAFETY_RECOVERED", 0),
        "degraded_events": dict(degraded_counts),
        "model_fallbacks": list(fallbacks),
    }
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return summary


if __name__ == "__main__":
    if len(sys.argv) == 2:
        inspect_match_trace(sys.argv[1])
```

将上述代码保存为 `inspect_trace.py` 后，赛后只需执行 ` .venv/bin/python inspect_trace.py /abs/path/logs/match.jsonl`：若 `decision_ms.p95` 稳定在数十毫秒以内（`budget_ok: true`）、比赛期间无非预期的 `LOCALIZATION_INVALID` 或 `TEAM_MISMATCH`，即可放心投入正式正赛。
