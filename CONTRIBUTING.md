# 参与贡献

感谢参与 SentryWeave。项目由未来战队维护，接受决策算法、数据处理、ROS 适配、仿真、测试和文档改进。
问题与讨论优先使用 [GitHub Issues](https://github.com/yelle233/SentryWeave/issues)；
其他联系可发送至 [1376982098@qq.com](mailto:1376982098@qq.com)，维护者为 [yelle233](https://github.com/yelle233)。

## 开发环境

```bash
git clone https://github.com/yelle233/SentryWeave.git
cd SentryWeave
python3 -m venv .venv
.venv/bin/python -m pip install -e '.[dev]'
```

算法训练使用单独的 `.venv-rl`，安装步骤见 [入门教程](docs/getting-started.md)。
ROS 运行使用 Ubuntu 24.04 / Jazzy，推荐先通过 Docker 运行 ROS 测试。

## 修改约定

- `main` 只包含决策、强化学习和通用 ROS 接入代码；物理仿真环境与针对特定导航栈的测试不进入 `main`。
- 决策、特征编码和执行协议放在 `sentry_core`，避免复制到训练脚本或 ROS 回调中。
- 时间戳表示原始观测时刻；缺失值使用有效位，不能伪装成零血量或已摧毁目标。
- 模型只能重排合法候选，不能绕过资源、定位、禁区或导航所有权约束。
- 更改特征、动作、坐标或模型格式时，更新 schema/version、导出器和推理一致性测试。
- 数据划分保持整场独立；模型选择不使用测试集。实验报告说明数据域、基线及局限。
- 使用小而聚焦的变更；文档和配置命令应能在首次克隆的环境中执行。

## 验证

普通代码：

```bash
.venv/bin/ruff check src tests tools
.venv/bin/python -m pytest tests -q
```

离线学习：

```bash
.venv-rl/bin/python -m pytest tests/test_offline_data.py \
  tests/test_offline_learning.py tests/test_rl_policy.py -q
```

已 source 的 Jazzy 工作区：

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
python3 -m pytest tests -q
```

纯文档修改检查链接、代码块语法和实际 CLI 参数即可。
没有相应环境的测试会被跳过，应在 PR 中说明，不能把 skipped 计为通过。

## 提交内容

PR 应说明问题、最终行为、验证命令和剩余限制。涉及算法结果时附上运行配置、随机种子、
数据校验信息和汇总指标；原始数据可在获取许可后由审阅者按官方渠道下载。

不提交：

- 官方比赛数据库、原始轨迹、录包和未获再分发许可的场地图；
- 训练权重、虚拟环境、Docker/colcon 缓存及运行日志；
- 凭据、设备私有配置、个人绝对路径或内部会话记录。

这些文件应保存在被忽略的 `artifacts/`、`third_party/` 等目录。
公开分析结果可放入 `benchmarks/`，并说明来源、生成方法和解释范围。

贡献按照项目 Apache-2.0 许可证提交。引入第三方内容时保留原作者声明，并更新
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。
