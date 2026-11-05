# 第三方源码与数据

RMJC 自身源码按 [Apache-2.0](LICENSE) 发布。该许可证不改变以下材料的原有权利或使用条件。

## PolarBear 导航与 RMU 仿真

- <https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav>
- <https://github.com/SMBU-PolarBear-Robotics-Team/rmu_gazebo_simulator>

两项目使用 Apache-2.0。固定版本见
[`integrations/polarbear/upstream.repos`](integrations/polarbear/upstream.repos)。
资产导入工具在本地生成 `third_party/`，保留上游许可证、修改声明及导入清单。
第三方场地和旧驱动子模块没有直接纳入本仓库源码。

## RMUC-OfflineRL

参考项目：<https://github.com/Harkerbest/RMUC-OfflineRL>。

Copyright (c) 2026 Navigator Team, Beijing Normal-Hong Kong Baptist University (BNBU)。
上游 MIT 文本和固定提交见
[`integrations/rmuc_offlinerl/`](integrations/rmuc_offlinerl/)。
RMJC 的导航学习实现使用适配在线接口的观测与动作定义，实验数字不能直接替代上游不同任务的结果。

## RMUC 2026 比赛数据

官方来源：<https://bbs.robomaster.com/article/1936220>。
比赛数据权利归 DJI/RoboMaster 相关权利人所有。
官方说明将数据用于战队策略训练，并明确其不具备规则效力或机器人竞技奖评价意义。

仓库不包含比赛数据库、原始轨迹或训练权重。下载工具只在使用者本地获取公开附件并校验，
使用者应遵守数据来源的适用条款。`benchmarks/` 只包含汇总分析指标和实验设置。
官方场地图片、规则手册内容与第三方商标也不随项目源码许可证重新授权。

## ROS 2、Navigation2 与 Gazebo

这些依赖通过系统包或容器安装，遵循各自许可证。项目保留标准 ROS 2 / Nav2 接口，
不会将依赖的许可证替换为 RMJC 的许可证。

RoboMaster、RMUC 及相关名称属于对应权利人。使用这些名称仅用于说明适用领域与数据来源，
不表示官方授权、认可或背书。
