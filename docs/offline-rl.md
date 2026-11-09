# 离线强化学习

SentryWeave 提供 BC、IQL 和 Decision Transformer 的导航子目标学习。
首次使用可按 [入门教程](getting-started.md) 执行完整流程。

## 数据合同

原始数据来自 [RMUC 2026 区域赛公开数据](https://bbs.robomaster.com/article/1936220)。
下载器固定官方附件并核对校验值，数据库只读。

| 内容 | 定义 |
|---|---|
| observation | 73 维：自己位置/速度/HP、规则倒计时、前哨站有效位、距离排名的敌我地面机器人槽位 |
| action | 最长 5 s 后的归一化 XY 位移，由日志轨迹构造 |
| reward | 队伍级伤害、建筑保护/攻击、自己的 HP 损失、死亡和末端胜负 |
| transition | 不重叠宏转移，按实际时长累计折扣奖励，bootstrap 为 `gamma ** duration` |
| split | `(赛区, 场次号)` 整组划分，所有局、阵营、兵种、生命片段保持同组 |

敌方 HP、经济、热量、学校名与胜方不进入观测。累计发弹不作为剩余弹量。
位置缺失标记 `(0,0)` 在蓝方镜像前处理；没有无限前向填充或未来最大 HP 补值。
时间使用规则时长的倒计时，不使用实际结束时刻泄漏未来。

宏动作是轨迹代理，不是真实导航命令记录。追踪中断、跳变和死亡会分段；
部分死亡末端缺少位置，无法构造动作，截断偏差需要在评测中说明。

## 构建数据

```bash
.venv-rl/bin/sentry-rl fetch --out artifacts/rl/raw
.venv-rl/bin/sentry-rl build \
  --db artifacts/rl/raw/extracted/rmuc_2026_region_dataset.sqlite \
  --out artifacts/rl/dataset --roles mixed --horizon 5 --gamma 0.99 --seed 29
.venv-rl/bin/sentry-rl inspect --data artifacts/rl/dataset
```

`mixed` 包含步兵3/4与哨兵；训练可指定 `infantry`、`sentry` 或具体地面角色 ID。
改变数据、特征、折扣或宏时长后，应生成新数据目录和实验目录。

## 训练

环境安装见入门教程。以下命令使用独立虚拟环境：

```bash
.venv-rl/bin/sentry-rl train \
  --data artifacts/rl/dataset --algorithm bc --roles infantry \
  --steps 30000 --batch-size 512 --seed 7 --device auto \
  --out artifacts/rl/runs/bc-seed7

.venv-rl/bin/sentry-rl train \
  --data artifacts/rl/dataset --algorithm iql --roles infantry \
  --init-actor artifacts/rl/runs/bc-seed7/actor.json \
  --steps 30000 --batch-size 512 --seed 7 --device auto \
  --out artifacts/rl/runs/iql-seed7

.venv-rl/bin/sentry-rl train \
  --data artifacts/rl/dataset --algorithm dt --roles infantry \
  --steps 15000 --batch-size 64 --context 8 --seed 7 --device auto \
  --out artifacts/rl/runs/dt-seed7
```

BC 使用 tanh-Gaussian 模仿；IQL 使用 twin Q、target Q、expectile V 和停止梯度的优势加权回归。
IQL 的 Q 只在记录动作上训练。DT 使用因果 `(return,state,action)` token；
当前 state 不可读取当前 action 标签，上下文不跨轨迹片段。

DT 训练使用已观测片段 RTG，截断片段的完整未来回报未知。
验证/部署采用训练集确定的固定预算，不读取测试局真实 RTG。
训练中的通道遮挡模拟缺失观测，并保持邻居排名槽位语义。

默认 checkpoint 依据验证集移动/静止平衡终点误差选择，反映轨迹拟合，不等同于策略价值最优。
训练只读取 train/validation；完整优化器断点续训未实现。

## 产物与导出

```text
config.json             配置、数据 ID、版本与设备
metrics.jsonl           loss、Q/V、优势权重、验证记录
network.safetensors     最佳 checkpoint 网络参数
actor.json              最佳推理模型
actor_last.json         最后一步推理模型
report.json             实际步数、最佳步数、参数变化与耗时
```

训练权重使用 safetensors，在线模型是经过 schema 校验的 JSON。
MLP/Transformer 都提供 NumPy 实现，并有 PyTorch 一致性测试；运行端不依赖 PyTorch/CUDA。

## 独立评测

```bash
.venv-rl/bin/sentry-rl evaluate \
  --data artifacts/rl/dataset \
  --actors artifacts/rl/runs/bc-seed7/actor.json \
           artifacts/rl/runs/iql-seed7/actor.json \
           artifacts/rl/runs/dt-seed7/actor.json \
  --out artifacts/rl/evaluation --device auto
```

报告区分步兵同域和哨兵迁移，包含终点误差、移动/静止分层、越界率、零位移与因果恒速基线。
区间按整场比赛重采样。输入遮挡用于检测模型依赖，不能作为合作价值的因果证明。

FQE：

```bash
.venv-rl/bin/sentry-rl fqe \
  --data artifacts/rl/dataset --actor artifacts/rl/runs/iql-seed7/actor.json \
  --steps 8000 --seeds 17,23 --device auto \
  --out artifacts/rl/evaluation/fqe-iql.json
```

FQE 仅支持同域 Markov BC/IQL，在训练数据拟合 critic，在测试片段初始状态评估原始 actor。
它不评估投影后的控制器，也不是胜率。DT 需要历史状态评估器，不能使用此无历史 FQE。
与 BC 的差值是诊断信息，不保证消除分布偏差。

## 在线使用

```text
模型短期建议
  → 比较相同行进距离上的可达候选路点
  → 混合原效用
  → 防抖 / 紧急仲裁
  → Nav2
```

模型不新增任意目标，不覆盖资源、定位、禁区和导航所有权约束。
已到达/越过或方向不充分的建议回到原效用，防止旧建议将机器人向后拉。

| 参数 | 含义 |
|---|---|
| `rl_model` | actor.json 路径；空值使用原策略 |
| `rl_blend` | 0 为影子；0.25 为混合起点；1 为可行候选内模型主导 |
| `rl_mirror` | 是否将当前地图做蓝方 180° 变换 |
| `rl_allow_transfer` | 显式允许场地尺寸不一致，用于研究性域迁移 |

影子模式保留规则任务，规则仍可导航。`policy_info`、`rule_utility` 和
`learned_utility` 记录模型影响。DT 历史只记录已经发生的位移，不记录尚未执行的未来动作。

示例场地与官方数据尺寸不同，运行集成实验需要显式选择域迁移：

```bash
ros2 launch sentry_bringup decision.launch.py \
  rl_model:="$PWD/artifacts/rl/runs/iql-seed7/actor.json" \
  rl_blend:=0.25 rl_allow_transfer:=true \
  trace_path:="$PWD/artifacts/rl/decisions.jsonl"
```

实车还需确认地图单位、原点、轴向、侧别和当季规则；尺寸匹配不能证明数据分布匹配。

## 闭环与回放

```bash
.venv-rl/bin/sentry-rl closed-loop \
  --actors artifacts/rl/runs/iql-seed7/actor.json \
  --seeds 701,702,703 --blend 0.25 --allow-domain-transfer \
  --out artifacts/rl/closed-loop.json

.venv-rl/bin/sentry-rl replay \
  --actor artifacts/rl/runs/iql-seed7/actor.json \
  --trace artifacts/rl/decisions.jsonl \
  --allow-domain-transfer --blend 0.25
```

回放需要匹配配置、混合比例和侧别；必要时提供 `--mirror`、
`--intent-model` 和 `--utility-model`。日志位置来自原记录，不能将反事实回放当成新策略的闭环收益。

公开的参考实验设置与结果见 [实验基线](benchmarks.md)。
