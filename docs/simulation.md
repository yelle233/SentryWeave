# 仿真、验收与日志

默认场地为自建的 20×12 m 参考环境。机器人具有物理碰撞、差速底盘、激光和里程计；
队友/敌方由场景节点提供观测。它用于决策与导航集成，不模拟完整比赛裁判或射击过程。

## Docker

首次克隆后，在仓库根目录构建：

```bash
docker build -t sentryweave:jazzy -f docker/Dockerfile .
mkdir -p artifacts
docker run --rm -it --name sentryweave-demo \
  -e LIBGL_ALWAYS_SOFTWARE=1 \
  -v "$PWD/artifacts:/ws/artifacts" \
  sentryweave:jazzy bash tools/ros_exec.bash \
  ros2 launch sentry_bringup simulation.launch.py \
  scenario:=team_push trace_path:=/ws/artifacts/decisions.jsonl
```

所有 ROS 运行依赖都在镜像中，宿主机不需要安装 ROS。
软件渲染可以在无 GPU 环境运行。代码修改后重建镜像，权重修改后重启节点。

可用场景：

| 名称 | 内容 |
|---|---|
| `team_push` | 队友推进 |
| `base_raid` | 防区受威胁 |
| `conflict` | 进攻和回防冲突 |
| `ambiguous` | 意图不明 |
| `dropout` | 外部位置数据停止 |
| `blocked` | 轻量仿真中的一次导航失败注入 |
| `phase_change` | 阶段/前哨站状态变化 |
| `low_hp` | 资源下降 |
| `localization_loss` | 定位健康中断 |
| `id_jump` | 轨迹身份跳变 |

各场景在运动学与物理层覆盖的故障不完全相同；例如轻量 `blocked` 注入规划失败，
不能视为真实 Gazebo 障碍物移动实验。

## 自动物理验收

无需训练模型：

```bash
docker run --rm -e LIBGL_ALWAYS_SOFTWARE=1 \
  -v "$PWD/artifacts:/ws/artifacts" \
  sentryweave:jazzy bash tools/ros_exec.bash \
  python3 tools/gazebo_acceptance.py --out artifacts/physics
```

模型训练完成后可启用 RL：

```bash
docker run --rm -e LIBGL_ALWAYS_SOFTWARE=1 \
  -v "$PWD/artifacts:/ws/artifacts" \
  sentryweave:jazzy bash tools/ros_exec.bash \
  python3 tools/gazebo_acceptance.py \
  --rl-model /ws/artifacts/rl/runs/iql-seed7/actor.json \
  --rl-blend 0.25 --allow-domain-transfer --out artifacts/rl/physics
```

工具创建独立 Gazebo partition，并使用默认 ROS domain 78。
并行运行多个验收实例时，为每个实例指定不同的 `--domain`。
输出包括 `physics.json`、JUnit XML、launch 日志和 JSONL 决策。
验收检查传感器、有效 World、禁区掩码和 SUPPORT 到位；RL 模式还要求模型实际参与评分。

## 本机 Jazzy

先安装 ROS 2 Jazzy，再安装运行依赖：

```bash
sudo apt update
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-ros-gz \
  ros-jazzy-robot-state-publisher ros-jazzy-tf2-geometry-msgs \
  python3-numpy python3-yaml python3-pytest python3-colcon-common-extensions
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch sentry_bringup simulation.launch.py gui:=true scenario:=conflict
```

不要跨宿主机/容器复用带绝对路径的 colcon 构建目录。
ROS 使用系统 Python，离线学习使用独立 `.venv-rl`。

## Docker GUI（可选）

Linux X11/XWayland 桌面可使用显示转发。先确认 `DISPLAY` 非空并安装 `xhost`：

```bash
echo "$DISPLAY"
xhost +si:localuser:root
docker run --rm -it --name sentryweave-gui \
  -e DISPLAY -e QT_X11_NO_MITSHM=1 -e LIBGL_ALWAYS_SOFTWARE=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
  -v "$PWD/artifacts:/ws/artifacts" \
  sentryweave:jazzy bash tools/ros_exec.bash \
  ros2 launch sentry_bringup simulation.launch.py gui:=true scenario:=team_push
xhost -si:localuser:root
```

最后一条命令在关闭仿真后撤销显示授权。
不同桌面环境可能需要额外配置，自动验收优先使用无 GUI 路径。

## 观察运行状态

对命名为 `sentryweave-demo` 的运行容器：

```bash
docker exec sentryweave-demo bash tools/ros_exec.bash ros2 topic echo /sentry/tactical/task --once
docker exec sentryweave-demo bash tools/ros_exec.bash ros2 topic echo /sentry/tactical/feedback --once
docker exec sentryweave-demo bash tools/ros_exec.bash ros2 topic echo /sentry/motion/permit --once
docker exec sentryweave-demo bash tools/ros_exec.bash ros2 lifecycle get /sentry/bt_navigator
```

Nav2 应进入 active；开始阶段等待 TF 属于正常初始化过程。
到位驻守、HOLD、旧任务取消或数据失效时，运动许可可能为 false。
发生保护时结合 `reason_code`、World 有效性和时间戳定位原因，不绕过门控。

暂停或重置仿真会影响 `/clock`。恢复后若旧 Goal 终态不能确认，应按
[部署指南](deployment.md) 清理故障或重启导航链。

## 日志与回放

`trace_path` 开启有界 JSONL 日志，单文件上限 20 MB，保留三个历史文件。
日志包含输入、候选、模型建议、拒绝原因和执行反馈。

```bash
.venv/bin/sentry-train replay --trace artifacts/demo/conflict.jsonl
```

RL 日志使用对应模型：

```bash
.venv-rl/bin/sentry-rl replay \
  --actor artifacts/rl/runs/iql-seed7/actor.json \
  --trace artifacts/rl/physics/decisions.jsonl --blend 0.25 --allow-domain-transfer
```

原始记录只存在于使用者生成的 `artifacts/` 中。
公开仓库的汇总实验数据见 [benchmarks](benchmarks.md)。
