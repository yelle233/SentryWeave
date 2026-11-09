# 入门：从首次克隆到训练与运行

本教程面向首次使用 SentryWeave 的开发者。主路径为 Ubuntu 24.04 / Python 3.12，
训练使用独立虚拟环境，ROS 2 Jazzy 通过 Docker 运行。

源码不附带比赛数据库、训练权重或预构建镜像。完成相应步骤后，这些文件会生成在本地。

## 1. 获取工程

```bash
git clone https://github.com/yelle233/SentryWeave.git "$HOME/SentryWeave"
cd "$HOME/SentryWeave"
ls README.md pyproject.toml docker/Dockerfile
```

后续命令默认在仓库根目录执行。新终端先执行 `cd "$HOME/SentryWeave"`；
如果克隆到其他目录，使用对应路径。

- `#` 开头是注释。
- 行尾 `\` 表示命令在下一行继续，后面不要加空格。
- `Ctrl+C` 停止当前前台程序。
- `.venv/bin/...` 或 `.venv-rl/bin/...` 明确选择 Python 环境，不要求先激活虚拟环境。
- Docker 中工程目录是 `/ws`，宿主机路径不会直接变成容器路径。

## 2. 先运行规则决策

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -e '.[dev]'
.venv/bin/python -m pytest tests -q

.venv/bin/sentry-train simulate \
  --scenario conflict --seed 101 --duration 30 \
  --trace artifacts/demo/conflict.jsonl
.venv/bin/sentry-train replay --trace artifacts/demo/conflict.jsonl
```

正常输出包含任务序列、覆盖指标和切换次数。`conflict` 同时提供队友推进与敌方威胁。
没有 ROS/PyTorch 时，相应测试会显示 skipped；跳过的部分没有被该次运行验证。

任务编码为 HOLD=0、SUPPORT=1、INTERCEPT=2、DEFEND=3、RECOVER=4。
模型之后只参与这些合法候选的评分，导航执行器仍管理实际移动。

## 3. 建立训练环境

术语：

| 名称 | 含义 |
|---|---|
| observation | 决策时可获得的状态 |
| action | 由轨迹构建的短期导航子目标 |
| reward | 行为结果的评分 |
| step | 一次参数更新，不是比赛的一秒或机器人迈一步 |
| checkpoint | 某次更新后保存的模型 |
| validation / test | 验证集负责模型选择，测试集负责独立报告 |

BC 模仿日志；IQL 使用奖励学习价值和策略；DT 根据历史和回报条件生成动作。
离线训练不需要 ROS 或 Docker，也不控制真实机器人。

```bash
sudo apt install -y python3-venv python3-pip curl
python3 -m venv .venv-rl
.venv-rl/bin/python -m pip install --upgrade pip
```

GPU 路线先运行 `nvidia-smi`。兼容 CUDA 12.8 的 NVIDIA 环境可使用：

```bash
.venv-rl/bin/python -m pip install torch==2.9.1 \
  --index-url https://download.pytorch.org/whl/cu128 --timeout 600
```

CPU 路线选择以下命令，二选一：

```bash
.venv-rl/bin/python -m pip install torch==2.9.1 \
  --index-url https://download.pytorch.org/whl/cpu --timeout 600
```

然后安装工程：

```bash
.venv-rl/bin/python -m pip install -e '.[dev,rl]'
.venv-rl/bin/sentry-rl --help
```

如果使用 uv 创建环境，可以用 `uv pip install --python .venv-rl/bin/python ...` 管理依赖。
uv 环境可能没有 pip，不能仅凭 `No module named pip` 判断环境损坏。

检查设备：

```bash
.venv-rl/bin/python - <<'PY'
import torch
print("PyTorch:", torch.__version__)
print("CUDA available:", torch.cuda.is_available())
if torch.cuda.is_available():
    print("GPU:", torch.cuda.get_device_name(0))
    x = torch.randn(128, 128, device="cuda")
    print("GPU matrix result:", (x @ x).norm().item())
PY
```

PyTorch wheel 带有所需 CUDA 运行库，系统仍需兼容驱动。
`--device auto` 会选择可用 CUDA，否则使用 CPU。
应预留数十 GB 给依赖、Docker 镜像和实验产物；数据库本身约 1.10 GiB。

## 4. 下载和构建数据集

```bash
.venv-rl/bin/sentry-rl fetch --out artifacts/rl/raw
.venv-rl/bin/sentry-rl build \
  --db artifacts/rl/raw/extracted/rmuc_2026_region_dataset.sqlite \
  --out artifacts/rl/dataset \
  --roles mixed --horizon 5 --gamma 0.99 --seed 29
.venv-rl/bin/sentry-rl inspect --data artifacts/rl/dataset
```

下载器校验官方附件。`build` 按整场比赛分组，生成 train/validation/test；
`mixed` 构建步兵3、步兵4和哨兵，后续训练可选择其中的兵种。

```text
artifacts/rl/dataset/
├── metadata.json
├── train.npz
├── validation.npz
├── test.npz
└── *.episodes.json
```

已存在 `metadata.json` 时，直接 inspect 复用，或选择新的构建输出目录。
不要把相邻帧随机打散后再划分训练/测试。

## 5. 用小训练检查流程

```bash
.venv-rl/bin/sentry-rl train \
  --data artifacts/rl/dataset --algorithm bc --roles infantry \
  --steps 300 --batch-size 128 --eval-every 100 \
  --seed 7 --device auto --out artifacts/rl/runs/smoke-bc
```

预期看到 `bc step=100 ...` 等验证进度，结束后检查：

```bash
ls artifacts/rl/runs/smoke-bc
python3 -m json.tool artifacts/rl/runs/smoke-bc/report.json
```

`optimizer_steps` 应为 300。小训练只检查流程，不作为策略效果证据。
如果目录已有完整实验，改用新 `--out`，不要删除整个数据目录。

## 6. 训练 BC → IQL，并加入 DT 对照

BC：

```bash
.venv-rl/bin/sentry-rl train \
  --data artifacts/rl/dataset --algorithm bc --roles infantry \
  --steps 30000 --batch-size 512 --eval-every 1000 \
  --seed 7 --device auto --out artifacts/rl/runs/bc-seed7
```

BC 完成后再训练 IQL：

```bash
.venv-rl/bin/sentry-rl train \
  --data artifacts/rl/dataset --algorithm iql --roles infantry \
  --init-actor artifacts/rl/runs/bc-seed7/actor.json \
  --steps 30000 --batch-size 512 --eval-every 1000 \
  --seed 7 --device auto --out artifacts/rl/runs/iql-seed7
```

DT：

```bash
.venv-rl/bin/sentry-rl train \
  --data artifacts/rl/dataset --algorithm dt --roles infantry \
  --steps 15000 --batch-size 64 --context 8 --eval-every 1000 \
  --seed 7 --device auto --out artifacts/rl/runs/dt-seed7
```

`infantry` 使用人控步兵记录；是否能迁移到哨兵由独立测试判断。
哨兵日志基线可改为 `--roles sentry`，BC 与 IQL 应使用一致的训练角色和网络尺寸。

在第二个终端查看进度：

```bash
cd "$HOME/SentryWeave"
tail -f artifacts/rl/runs/iql-seed7/metrics.jsonl
```

- 首次验证前可能暂时没有控制台输出，可查看日志和 GPU 占用。
- 在训练终端按 `Ctrl+C` 会中断训练；在日志终端按下只停止 `tail`。
- 当前 CLI 没有完整优化器断点续训；`--init-actor` 只初始化 actor。
- 显存不足时降低 batch；DT 还可缩短 context，并用新目录记录参数。
- 训练更久不一定更好，使用验证集选出的 `actor.json`。

## 7. 识别模型产物

| 文件 | 用途 |
|---|---|
| `actor.json` | 验证集选出的部署模型 |
| `actor_last.json` | 最后一次更新的 actor |
| `network.safetensors` | 最佳 checkpoint 的完整网络参数 |
| `config.json` | 参数、数据 ID、版本与设备 |
| `metrics.jsonl` | 训练和验证记录 |
| `report.json` | 最佳步数、实际更新数及耗时 |

给 ROS 的 `rl_model` 填 `actor.json`。
`best_step=1000`、`optimizer_steps=30000` 表示后续更新未改善选定验证指标，不能据此认为训练提前停止。
不同算法/随机种子应使用不同实验目录；评测用相同输出目录时会更新旧报告。

## 8. 独立评测与轻量闭环

```bash
.venv-rl/bin/sentry-rl evaluate \
  --data artifacts/rl/dataset \
  --actors artifacts/rl/runs/bc-seed7/actor.json \
           artifacts/rl/runs/iql-seed7/actor.json \
           artifacts/rl/runs/dt-seed7/actor.json \
  --out artifacts/rl/evaluation --device auto
```

打开生成的 `comparison.md` 和 `comparison.json`：

- `infantry` 为同兵种测试，`sentry_transfer` 为跨兵种测试。
- `endpoint_error_m` 是导航目标的预测误差，不是定位误差或 Nav2 到位误差。
- 同时看移动样本、静止样本、零位移基线和恒速基线。
- 选择模型使用验证集；测试集用于最终报告，不能反复据其调参再称为独立测试。

轻量闭环：

```bash
.venv-rl/bin/sentry-rl closed-loop \
  --actors artifacts/rl/runs/iql-seed7/actor.json \
  --config src/sentry_bringup/config/tactics.yaml \
  --seeds 701,702,703 --duration 30 --blend 0.25 \
  --allow-domain-transfer --out artifacts/rl/closed-loop.json
```

检查非法目标、失效定位下运动、支援/防区覆盖、任务切换和 `rl_rank_changes`。
这是脚本化运动学场景，不是完整比赛结果。
FQE 的使用和限制见 [离线强化学习](offline-rl.md)。

## 9. 构建 ROS 2 决策链

ROS 2 Jazzy 与 Nav2 依赖都在 Docker 镜像中，宿主机不需要安装 ROS。先确认 Docker 服务和权限：

```bash
docker version
```

应同时显示 Client 和 Server。如果全新 Ubuntu 尚未安装 Docker，可使用：

```bash
sudo apt update
sudo apt install -y docker.io
sudo systemctl enable --now docker
sudo usermod -aG docker "$USER"
```

修改用户组后注销并重新登录。已有正常工作的 Docker CE 等安装时，无需另外安装 `docker.io`。

构建镜像，并在容器中运行测试：

```bash
docker build -t sentryweave:main -f docker/Dockerfile .
docker run --rm sentryweave:main bash tools/ros_exec.bash python3 -m pytest tests -q
```

容器中会实际运行 ROS Action 与 Nav2 控制器配置测试；镜像不含 PyTorch，训练相关测试显示 skipped。
代码修改后需要重新构建镜像。

`decision.launch.py` 启动 `world_model`、`decision`、`executor` 和 `velocity_guard`，
不包含定位、感知、Nav2 或仿真。查看全部参数：

```bash
docker run --rm sentryweave:main bash tools/ros_exec.bash \
  ros2 launch sentry_bringup decision.launch.py --show-args
```

## 10. 在决策链中加载模型

```bash
mkdir -p artifacts/live
docker run --rm -it --name sentryweave-decision \
  -v "$PWD/artifacts:/ws/artifacts" \
  sentryweave:main bash tools/ros_exec.bash \
  ros2 launch sentry_bringup decision.launch.py \
  rl_model:=/ws/artifacts/rl/runs/iql-seed7/actor.json \
  rl_blend:=0.25 rl_allow_transfer:=true \
  trace_path:=/ws/artifacts/live/decisions.jsonl
```

宿主机 `artifacts/...` 在容器中对应 `/ws/artifacts/...`。
日志出现 `Decision ready: intent=rule, goal=iql` 表示模型已加载；
`rl_allow_transfer` 显式允许 28×15 m 官方数据模型用于默认 `tactics.yaml` 的 20×12 m 示例场地。
尚未接入定位、裁判和 Nav2 时，任务保持 HOLD，运动许可为 false。

第二个终端检查：

```bash
docker exec sentryweave-decision bash tools/ros_exec.bash ros2 topic echo /sentry/tactical/task --once
docker exec sentryweave-decision bash tools/ros_exec.bash ros2 topic echo /sentry/motion/permit --once
```

查看 `tactical/trace` 或 JSONL 中的 `policy_info`，了解建议、模型 ID 和回退原因。
`used=false` 可能表示小位移、建议已到达或约束回退，不一定是加载失败。

将 `rl_blend:=0.0` 可启用影子模式；**原规则仍可驱动机器人**。
完全不控制机器人时使用离线回放。
更换模型后重启节点，不会自动热加载磁盘文件。
在运行终端按 `Ctrl+C` 停止；也可在第二个终端发送 SIGINT：

```bash
docker kill --signal=SIGINT sentryweave-decision
```

使用相同配置与模型回放记录：

```bash
.venv-rl/bin/sentry-rl replay \
  --actor artifacts/rl/runs/iql-seed7/actor.json \
  --trace artifacts/live/decisions.jsonl \
  --config src/sentry_bringup/config/tactics.yaml \
  --blend 0.25 --allow-domain-transfer
```

使用蓝方变换时加 `--mirror`；若原运行加载 HMM/效用模型，也提供对应参数。

## 11. 接入实车与导航栈

设备侧需要定位、里程计、障碍输入、定位健康和规范化雷达/裁判消息。
模型只消费这些状态，不直接解析官方串口字节或相机图片。

先按 [部署指南](deployment.md) 对齐：

1. `map/odom/base_link`、源时间戳和地面机器人 ID；
2. 实际场地 bounds、禁区与候选点；
3. 底盘速度、加速度、footprint、速度仲裁和硬件超时；
4. 比赛阶段、资源有效位和补给条件；
5. 训练坐标与实际地图的单位、轴向、原点和侧别。

已有导航栈时只启动 `decision.launch.py`：导航栈提供 `navigate_to_pose` 与 `compute_path_to_pose`，
速度输出接到 `nav_cmd_vel`，由 `velocity_guard` 独占 `cmd_vel`。
`robot.launch.py` 则同时启动参考 Nav2 与决策链。

模型尺寸检查失败时，应处理坐标/数据域问题，不能只修改数字绕过检查。
2027 等新赛季即使场地尺寸相同，也需要重新验证地图、规则和数据分布。

## 常见问题

| 问题 | 处理方向 |
|---|---|
| 找不到 `sentry-rl` | 回仓库根目录，检查 `.venv-rl` 并重新安装 editable 工程 |
| `No module named pip` | 使用 uv 管理该环境，或通过 `python -m ensurepip --upgrade` 补齐 |
| CUDA 不可用 | 验证驱动与 PyTorch wheel；也可使用 CPU |
| 已有数据/实验目录 | 复用正确数据，或使用新输出目录，保留原实验 |
| warm start 尺寸不匹配 | BC 与 IQL 的 hidden 必须一致，默认 128 |
| actor 与 dataset ID 不一致 | 用训练时对应的数据评测，不手改校验字段 |
| 容器找不到模型 | 检查挂载、工程根目录和 `/ws/artifacts` 路径 |
| 代码修改未生效 | 重新构建镜像；更换权重后重新启动节点 |
| 机器人 HOLD | 检查定位、裁判阶段、时间戳、有效位和数据新鲜度 |
| 到位后不重发 Goal | 驻守任务保持相同身份是正常行为 |
| 暂停/重置仿真后停止 | 检查时钟与执行故障，恢复后按部署文档清理或重启 |

当前结果支持研究、影子记录与受约束实验，尚不支持直接宣称战术收益提升。
更多说明见 [实验基线](benchmarks.md)、[部署](deployment.md)、[离线学习](offline-rl.md)。
