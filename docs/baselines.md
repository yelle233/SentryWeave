# 规则、HMM 与效用权重基线

本页介绍 HMM 与效用权重基线。官方比赛数据的 BC/IQL/Decision Transformer
训练见 [离线强化学习](offline-rl.md)。

## 训练的对象

原有基线训练两部分：

1. **意图 HMM**：从轨迹特征学习六类意图的 Gaussian emission 和转移概率，验证集选择温度。
2. **战术效用权重**：用交叉熵搜索（CEM）在闭环场景中优化六个权重；硬约束、时效规则、
   取消协议和禁区不参加学习，模型不能把它们“学没了”。

这不是端到端强化学习，也不输出底盘速度。底盘控制和避障由 Nav2 承担。
若以后加入 PPO 等策略，替换的是候选任务的评分 Interface，继续保留同样的约束与执行器。

## 可复现的合成数据流程

所有命令在仓库根目录运行；非默认战术地图使用命令前的全局参数：

```bash
.venv/bin/sentry-train --config /absolute/path/tactics.yaml generate \
  --episodes-per-class 40 --steps 80 --seed 110 --out artifacts/data/custom.jsonl
```

默认示例直接执行 README 的完整训练流程。训练生成：

| 文件 | 内容 |
|---|---|
| `intent.jsonl` | 整段轨迹、采样时刻、特征、人工/生成标签、分组、地图指纹 |
| `intent.json` | 特征版本、标签顺序、均值/方差、转移矩阵、先验、温度 |
| `intent.report.json` | 分组清单、验证与测试准确率、macro-F1、NLL、Brier、ECE、混淆矩阵 |
| `utility.json` | 有效权重、训练/验证种子、搜索历史与验证收益 |
| `comparison.json/.md` | 同场景同种子的策略对照、逐场结果 |

按**整段轨迹的 group_id** 划分 60%/20%/20%，同组不跨集合。生成器同一个重复种子的六类
轨迹共享 group_id。真实数据应以整场比赛/独立采集任务分组，不能把相邻帧随机打散。
温度、权重选择只使用验证集；独立评测种子不能出现在权重搜索中。

HMM 六个类别都需要至少两个训练样本，实际训练应远多于此。JSON 加载器会拒绝不兼容特征、
错误地图指纹、非有限参数、负方差和非法概率矩阵。改变场地几何后需要重新生成特征和训练。

## 真实数据工作流

在线启动时开启有界 JSONL 日志（每个文件最大 20 MB，最多保留 3 个历史文件）：

```bash
ros2 launch sentry_bringup decision.launch.py namespace:=sentry \
  config_file:=/absolute/path/tactics.yaml \
  trace_path:=/absolute/path/logs/match_01.jsonl
```

JSONL 包含完整 World、每个队友的特征/概率、所有可行候选、拒绝原因、仲裁结果和执行反馈。
日志内的 `episode_id` 是运行会话，标注时应照抄。采样时间使用 **源观测时间**，
不是读文件时的系统时间。记录至少五个独立比赛/采集组后才做分组训练。

人工按录像和队伍复盘填写区间。区间为左闭右开：

```json
[
  {"episode_id": "日志中的会话ID", "robot_id": 1,
   "start": 12.0, "end": 24.0, "label": "ATTACK_OUTPOST"},
  {"episode_id": "日志中的会话ID", "robot_id": 1,
   "start": 24.0, "end": 28.0, "label": "UNKNOWN"},
  {"episode_id": "日志中的会话ID", "robot_id": 1,
   "start": 28.0, "end": 40.0, "label": "RETREAT"}
]
```

上面的时刻只是格式示例，实车日志通常是 ROS 系统时钟秒数。提取器只采用明确标注的样本，
拒绝重叠标签；未标注不等于 UNKNOWN。重复观测去重，失联、身份重建或长空隙会分段。

```bash
.venv/bin/sentry-train --config /absolute/path/tactics.yaml extract \
  --trace /absolute/path/logs/match_01.jsonl \
  --annotations /absolute/path/labels/match_01.json \
  --out artifacts/data/match_01.jsonl
```

提取多个独立比赛后，可以用 `cat artifacts/data/match_*.jsonl > artifacts/data/real.jsonl`
合并，再执行 `train-intent`。训练报告应分别统计合成、真实验证和真实测试数据。
不要通过给模型读取录像标签来评估在线决策；运行时只能使用观测特征。

## 场景与公平对照

| 场景名 | 覆盖行为 |
|---|---|
| `team_push` | 队友持续进攻，哨兵寻找补充支援点 |
| `base_raid` | 敌人逼近己方区域 |
| `conflict` | 进攻机会与防区威胁同时存在 |
| `ambiguous` | 静止/意图不明，不强猜战术 |
| `dropout` | 雷达停止更新，旧意图失效 |
| `blocked` | 一次导航不可达，点位冷却与备选 |
| `phase_change` | 前哨站状态与比赛阶段转换 |
| `low_hp` | 资源不足且补给有条件可用 |
| `localization_loss` | 自定位短时中断 |
| `id_jump` | 身份轨迹突变与重建 |

轻量仿真传感器以 10 Hz 更新，战术层默认 2 Hz；加入位置噪声、延迟和丢帧，
用同一执行协议模拟规划、到位与失败。奖励来自实际位置的覆盖、资源和移动/切换代价，
不会因为任务名字叫 SUPPORT 就直接给奖励。

`fixed` 是只依据自身资源、前哨站状态的固定策略对照。
比较实际比赛收益时，应接入参赛队伍已有的行为树作为对照策略。
`rule` 用规则意图和原配置权重；`learned` 用 HMM 和可选训练权重；
`no_hysteresis` 是移除普通仲裁保持/确认的消融。`evaluate --utility` 只改 learned，
不会悄悄修改控制组。所有策略使用同样的场景、初始状态和观测随机种子。

评测按场景分析支援覆盖、防区覆盖、响应延迟、切换次数、导航成功/失败、
非法目标、失效数据驱动行为。没有威胁事件或没有响应时延值为 null，不能当作零延迟。
平均覆盖是连续距离代理分数，不是官方得分或比赛胜率。

## 回放与验证层次

```bash
.venv/bin/sentry-train simulate --scenario blocked --seed 303 --duration 30 \
  --trace artifacts/replay/blocked.jsonl
.venv/bin/sentry-train replay --trace artifacts/replay/blocked.jsonl
```

在线日志的 `feedback_before` 在对应决策之前应用，轻量模拟的 `feedback` 在该轮执行之后应用。
这样冷却和取消恢复的时间顺序不会在回放中反转。时间倒退后映射新的会话身份；
更换策略进行对比回放时，原本属于不同任务/目标的反馈不会套到新任务上。
回放中的机器人位置仍来自原始记录，不能把这种对比当成新策略的闭环对局结果。

离线测试：

```bash
.venv/bin/python -m pytest tests -q
.venv/bin/ruff check src tests tools
```

ROS Action 集成（需已经构建并 source 工作区）：

```bash
bash tools/ros_exec.bash python3 -m pytest tests/integration/test_ros_actions.py -q
```

合成数据准确率、轻量场景收益和接入导航栈后的实际导航结果应分开报告。
