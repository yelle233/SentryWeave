# 实验基线与验证范围

本页记录可复现的参考实验，不提供比赛胜率或官方认证结论。
机器可读汇总见 [`benchmarks/rmuc2026-navigation.json`](../benchmarks/rmuc2026-navigation.json)；
仓库不分发原始比赛记录、逐帧预测或训练权重。

## 协议

实验记录日期为 2026-11-05，使用公开 RMUC 2026 数据。
SQLite SHA-256 为 `53ac64efeaf570c0e21d8c3724dcefcb0123196b3c7ef963183a911863f724a7`。

| 集合 | 场次组 | 局数 | 宏转移 |
|---|---:|---:|---:|
| train | 186 | 431 | 186,690 |
| validation | 39 | 87 | 37,807 |
| test | 41 | 95 | 41,368 |

按 `(赛区, 场次号)` 分组，split seed=29。
主训练使用 121,741 条步兵宏转移；同域测试 27,591 条，哨兵迁移测试 13,777 条。
观测 73 维、动作 2 维、宏时长最长 5 s，每秒 gamma=0.99。
这些定义不同于上游完整交战动作任务，指标不能直接与其目标分类准确率比较。

BC/IQL 各 30,000 更新、batch=512；DT 15,000 更新、batch=64、context=8。
三个方法均使用 seeds 7/19/43。IQL actor 从同 seed BC 初始化。
checkpoint 由验证集移动/静止平衡终点误差选择，测试集不参与模型选择。

参考环境：Python 3.12.3、PyTorch 2.9.1+cu128、RTX 5070 12 GB。
详细结构和实验设置保存在汇总 JSON 中。

## 独立轨迹测试

平均终点误差，单位米，较小表示更接近日志中的后续位置：

| 策略 | 步兵总体 | 步兵移动样本 | 哨兵总体 | 哨兵移动样本 |
|---|---:|---:|---:|---:|
| 零位移 | 2.1480 | 3.1403 | 0.8513 | 2.2562 |
| 因果恒速 | 3.2609 | — | 1.8570 | — |
| BC | 1.9043 | 2.4700 | 1.2232 | 2.0918 |
| IQL | 1.9384 | 2.4758 | 1.2862 | 2.0997 |
| DT | 2.0086 | 2.5910 | 1.1696 | 2.0717 |

步兵总体误差的 seed 标准差分别为 0.0090、0.0021、0.0168 m。
哨兵只有约 31.25% 的宏步移动超过 0.5 m，零位移基线很强；
较低总体误差不能单独证明更好的战术。

IQL 在该轨迹拟合口径下未超过 BC。该结果也不能反向证明 IQL 在所有战术收益上更差：
轨迹拟合与真实收益是不同目标，目前缺少支持更强结论的闭环证据。

## FQE

同域 BC/IQL 各拟合两个 FQE critic（seeds 17/23，各 8,000 更新）。
相对同训练 seed BC，IQL 的均值差约为 +0.0284 和 +0.0003。
这些变化受部分可观测性、截断和动作支撑集影响，不构成胜率提升证据。
未对 DT 使用无历史的 Markov FQE。

## 受约束的闭环

在 20×12 m 运动学参考环境中使用 seeds 701/702/703，10 类场景、每次 30 s。
这是显式域迁移，不能代替官方场地评测。
用于闭环的 BC/IQL/DT 分别是 seed19/43/7，
依据完整步兵验证集选出；独立测试结果未参与这项选择。

| 策略，blend=0.25 | 代理奖励 | 支援覆盖 | 防区覆盖 | 平均切换 |
|---|---:|---:|---:|---:|
| fixed | 1.1058 | 0.8434 | 0.0696 | 0.70 |
| rule | 0.9004 | 0.3021 | 0.8965 | 3.70 |
| BC prior | 0.9024 | 0.3036 | 0.9140 | 3.80 |
| IQL prior | 0.8861 | 0.3071 | 0.8401 | 3.87 |
| DT prior | 0.8070 | 0.2603 | 0.8330 | 5.07 |

blend=1.0 时，BC/IQL/DT 的代理奖励为 0.7229/0.7122/0.5849。
非法目标和失效定位下运动计数均为 0，主要来自约束实现，不能归功为模型自行学会安全规则。
这些结果支持保留规则基线与影子/混合模式，而非直接全面替换控制策略。

## Jazzy 物理仿真集成

以下结果在作者的物理仿真环境中测得（Nav2 1.3.13、Gazebo Harmonic 8.15.0、BehaviorTree.CPP 4.10.0），
该环境不属于本仓库。NumPy actor 在没有 PyTorch 的运行镜像中完成以下验收：

| 指标 | IQL | DT |
|---|---:|---:|
| 实际位移 | 12.058 m | 12.075 m |
| 模型评分次数 | 22 | 6 |
| 到位任务 | SUPPORT | SUPPORT |
| 执行故障 | 0 | 0 |

验证覆盖激光、里程计、World、禁区掩码、模型实际评分和 NavigateToPose 终态。
底盘为差速参考模型；MPPI Omni 已做参数配置验证，未据此宣称全向实车动力学已验证。
队友为脚本观测，未模拟完整对战。

## 复现

按 [离线强化学习](offline-rl.md) 构建数据并训练。
完整协议需要分别运行三个 seed，并保持同一数据划分：

```bash
for seed in 7 19 43; do
  .venv-rl/bin/sentry-rl train --data artifacts/rl/dataset \
    --algorithm bc --roles infantry --steps 30000 --batch-size 512 --seed "$seed" \
    --out "artifacts/rl/runs/bc-seed$seed"
  .venv-rl/bin/sentry-rl train --data artifacts/rl/dataset \
    --algorithm iql --roles infantry --steps 30000 --batch-size 512 --seed "$seed" \
    --init-actor "artifacts/rl/runs/bc-seed$seed/actor.json" \
    --out "artifacts/rl/runs/iql-seed$seed"
  .venv-rl/bin/sentry-rl train --data artifacts/rl/dataset \
    --algorithm dt --roles infantry --steps 15000 --batch-size 64 --context 8 --seed "$seed" \
    --out "artifacts/rl/runs/dt-seed$seed"
done
.venv-rl/bin/sentry-rl evaluate --data artifacts/rl/dataset \
  --actors artifacts/rl/runs/*/actor.json --out artifacts/rl/evaluation
```

已有完整实验目录时请换目录。硬件、依赖版本和实现更新可能影响数值；
报告应附带实际 commit、配置、数据校验值和运行日志。
