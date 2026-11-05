# PolarBear / RMU 集成与 Jazzy 迁移

## 实际上游与迁移取舍

固定提交见 [`integrations/polarbear/upstream.repos`](../integrations/polarbear/upstream.repos)，
上游项目使用 Apache-2.0。
`rmu_gazebo_simulator` 是 **Ignition Fortress**，不是 Gazebo Classic；
本工程目标是 **Harmonic（gz-sim 8）**。

| 旧实现 | 本工程的处理 |
|---|---|
| `gz_version=6`、`ignition::gazebo`、`libignition-gazebo-*` | Harmonic 启动与 `gz::sim` / `gz-sim-*` 插件 |
| 桥接 `ignition.msgs.*` | 转为 `gz.msgs.*`，保留原 ROS 主题名的转换产物 |
| Humble 内置 BT 库清单 | 移除；Jazzy 自动注册内置库 |
| Nav2 类名 `package/Class` | 对标准 Nav2 插件改为 `package::Class` |
| 旧 BT XML / 恢复节点 | 替换为本工程实际运行的 BT.CPP4 XML |
| 自定义 Omni PID、IntensityVoxelLayer、BackUpFreeSpace | 用 Jazzy 标准控制器、LaserScan 障碍层与标准行为替代 |
| point_lio、Livox、small_gicp、gimbal fake frame | 作为真实定位/感知 Adapter 保留 Interface；不混编未经验证的 Humble 子模块 |
| 原 RMOSS 射击/比赛裁判插件 | 参考验证链用规范化裁判/敌我观测注入，不声称已移植比赛物理 |
| `Twist` / `TwistStamped` 版本差异 | Jazzy 显式使用 Twist，全链一致 |

SentryWeave 提供 Jazzy 标准插件的替代运行路径。自定义 Gazebo 插件需要单独做源码/API 迁移；
导入器会拒绝无法自动适配的插件。

## 导入

```bash
.venv/bin/python tools/import_polarbear.py --fetch
```

或使用已有、干净且固定到所需提交的完整 checkout：

```bash
.venv/bin/python tools/import_polarbear.py \
  --nav /absolute/path/pb2025_sentry_nav \
  --sim /absolute/path/rmu_gazebo_simulator \
  --out third_party/polarbear_jazzy
```

输出包含 `worlds/`、`models/`、`maps/`、`nav2_jazzy.yaml`、
`upstream_bridge_jazzy.yaml`、原许可证和 `import_manifest.json`。
脚本检查提交、工作树、mesh/地图完整性，拒绝覆盖已存在的输出；需重新导入时选择新输出目录。
`COLCON_IGNORE` 防止第三方旧包被递归构建。默认不递归下载硬件驱动子模块。

迁移后的导航配置可以先在本项目参考场地完成独立验收：

```bash
bash tools/ros_exec.bash python3 tools/gazebo_acceptance.py \
  --params-file third_party/polarbear_jazzy/nav2_jazzy.yaml \
  --out artifacts/polarbear-navigation
```

这会实际加载迁移后的全局规划器配置，使用 Jazzy 下的参考底盘、执行器和禁区过滤。

## 世界坐标与导航地图对齐

不能把原 `.pgm` 地图原点当成 Gazebo 世界原点。例如核验的 `rmuc_2025.yaml` 的栅格原点是
`[-3.58, -9.44, 0]`；它只表示图像栅格在 map 中的位置，不给出 world→map 变换。

在 Gazebo 世界和导航地图中选取至少两个不同的对应物理点，推荐三个以上分散锚点。
JSON 格式如下，填写实测坐标，单位都是米：

```text
[
  {"world": [世界坐标x, 世界坐标y], "map": [地图坐标x, 地图坐标y]},
  ...
]
```

拟合刚体变换并检查残差：

```bash
.venv/bin/python tools/fit_map_transform.py /absolute/path/survey_anchors.json --max-rmse 0.2
```

工具输出平移、旋转和各点误差；拒绝退化点或超过容差的拟合，不擅自缩放场地。
`world_to_map` 只作用于仿真真值定位。实车 LIO/GICP 已给出 map 系时不要再重复应用。

在标定好战术地图和变换后运行，例如 RMUC 2025：

```bash
ros2 launch sentry_bringup simulation.launch.py \
  world:="$PWD/third_party/polarbear_jazzy/worlds/rmuc_2025_world.sdf" \
  extra_resource_path:="$PWD/third_party/polarbear_jazzy/models" \
  map:="$PWD/third_party/polarbear_jazzy/maps/rmuc_2025.yaml" \
  params_file:="$PWD/third_party/polarbear_jazzy/nav2_jazzy.yaml" \
  config_file:=/absolute/path/calibrated_rmuc_2025_tactics.yaml \
  world_to_map:='[实测tx, 实测ty, 实测yaw]' \
  gui:=true
```

命令中的三项实测值必须替换为拟合输出。导入时 `--spawn X Y YAW` 决定参考机器人在
Gazebo 世界的初始位置，应选择场地内可通行位置。场地与地图可以成功加载，并不证明点位已经对齐。

## 接已有 PolarBear 定位栈

使用已经适配 Jazzy 的 point_lio / small_gicp / terrain_analysis 时：

- 统一 `map/odom/base_link` 与 `odometry` 等主题，或者修改 Nav2/decision 参数使其一致；
- 原 `gimbal_yaw_fake` 与底盘实际运动方向之间的速度变换必须保留；
- 在最终速度变换之后接入门控，避免 `fake_vel_transform` 与 guard 同时向同一底盘主题发速度；
- 雷达点云中的高度/intensity 语义不能简单转换成标准 obstacle layer 的参数名。
  参考配置用 LaserScan；实车可换成经过 Jazzy 编译测试的点云/地形层；
- 保留原许可证与源码版本记录。未获得传感器和驱动的实际 Interface 前，不把仿真真值定位带到实车。

Harmonic SDF 静态检查时，模型查找路径使用 `SDF_PATH`；启动 Gazebo 时使用 `GZ_SIM_RESOURCE_PATH`。
`gz_frame_id` 是 Gazebo 的传感器扩展，SDFormat 独立校验器可能提示扩展标签，
实际桥接应检查 LaserScan 的 `header.frame_id` 是否为 `lidar_link`。
