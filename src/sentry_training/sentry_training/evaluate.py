"""Matched-seed policy evaluation and deterministic world replay."""

import json
from pathlib import Path

import numpy as np

from sentry_core.policy import DecisionEngine
from sentry_core.serde import world_from_dict
from .fit import write_json
from .simulation import SCENARIOS, rollout


def evaluate(config, policies=("fixed", "rule"), seeds=(101, 102, 103),
             scenarios=SCENARIOS, duration=30., model=None, output=None, learned_config=None):
    runs = [rollout(learned_config if policy == "learned" and learned_config else config,
                    scenario, seed, duration=duration, policy=policy, model=model)
            for policy in policies for scenario in scenarios for seed in seeds]
    metrics = ("reward", "switches", "navigation_arrivals", "navigation_failures",
               "illegal_goals", "unsafe_motion_steps", "stale_support_steps",
               "support_coverage", "defense_coverage", "threat_response_sec", "decision_p95_ms")
    summary = {}
    for policy in policies:
        rows = [row for row in runs if row["policy"] == policy]
        summary[policy] = {}
        for metric in metrics:
            values = [r[metric] for r in rows if r[metric] is not None]
            summary[policy][metric] = (float(np.mean(values)) if values else None)
        summary[policy]["runs"] = len(rows)
    report = {
        "schema_version": 1, "environment": "kinematic_tactical_surrogate",
        "map_fingerprint": config.map_fingerprint,
        "seed_pairs": list(seeds), "duration": duration, "summary": summary, "runs": runs,
        "limitations": "Coverage scores are surrogate metrics; no measured match win rate.",
    }
    if output:
        write_json(output, report)
        lines = ["# 场景评测", "", "同一策略使用相同的场景和随机种子。数值来自轻量运动学仿真。",
                 "", "| 策略 | 奖励 | 支援覆盖 | 防区覆盖 | 切换次数 | 非法目标 |",
                 "|---|---:|---:|---:|---:|---:|"]
        for policy, row in summary.items():
            def fmt(name):
                return f"{row[name]:.3f}" if row[name] is not None else "无机会"
            lines.append(f"| {policy} | {fmt('reward')} | {fmt('support_coverage')} | "
                         f"{fmt('defense_coverage')} | {fmt('switches')} | {fmt('illegal_goals')} |")
        Path(output).with_suffix(".md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    return report


def replay(config, path, model=None, policy="rule", goal_prior=None):
    engine = None
    identities = {}
    previous = None
    sequence = []
    decisions = 0
    with open(path, encoding="utf-8") as stream:
        for line in stream:
            record = json.loads(line)
            if record.get("schema_version") != 1:
                raise ValueError("unsupported trace format")
            if record.get("map_fingerprint", config.map_fingerprint) != config.map_fingerprint:
                raise ValueError("trace map fingerprint mismatch")
            if engine is None:
                session = record.get("decision", {}).get("task", {}).get("session_id", "replay")
                engine = DecisionEngine(config, model, session_id=session, baseline=policy == "fixed",
                                        hysteresis=policy != "no_hysteresis", goal_prior=goal_prior)
            def apply_feedback(items):
                for feedback in items:
                    identity = identities.get((feedback["session_id"], feedback["task_id"]))
                    if identity is not None:
                        engine.feedback(*identity, feedback["state"], feedback["stamp"])
            apply_feedback(record.get("feedback_before", []))
            decision = engine.step(world_from_dict(record["world"]))
            task = decision.task
            recorded = record.get("decision", {}).get("task")
            if recorded:
                key = (recorded["kind"], recorded["zone_id"], recorded["target_id"], recorded["goal_id"])
                goal = {"x": task.goal.x, "y": task.goal.y} if task.goal else None
                # Map session IDs across clock resets. For counterfactual policy replay,
                # do not apply results for a different task/goal to the new decision.
                if key == task.key and recorded["goal"] == goal:
                    identities[(recorded["session_id"], recorded["task_id"])] = task.identity
                    if len(identities) > 4096:
                        del identities[next(iter(identities))]
            if task.identity != previous:
                sequence.append(f"{task.kind.name}:{task.zone_id}:{task.goal_id}")
                previous = task.identity
            apply_feedback(record.get("feedback", []))
            decisions += 1
    return {"decisions": decisions, "task_sequence": sequence}
