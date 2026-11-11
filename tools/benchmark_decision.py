#!/usr/bin/env python3
"""Measure DecisionEngine.step latency, CPU time and memory on the bundled fields.

Example: ``.venv/bin/python tools/benchmark_decision.py --out benchmarks/decision-latency.json``
"""

import argparse
import json
import math
import os
from pathlib import Path
import platform
import resource
import time
import tracemalloc

import numpy as np

from sentry_core.config import load_config
from sentry_core.geometry import TacticalMap
from sentry_core.model import GameState, Observation, Phase, RobotStatus, StructureState, Team, Vec2
from sentry_core.policy import DecisionEngine
from sentry_core.rl_policy import GoalPrior
from sentry_core.tracking import WorldModel

ROOT = Path(__file__).resolve().parents[1]
CONFIGS = {"demo": "tactics.yaml", "rmuc2027": "tactics_rmuc2027.yaml",
           "rmuc2026": "tactics_rmuc2026.yaml"}


def _between(a: Vec2, b: Vec2, fraction: float) -> Vec2:
    return Vec2(a.x + (b.x - a.x) * fraction, a.y + (b.y - a.y) * fraction)


def scenario(config, steps: int, dt: float = 0.5):
    """Deterministic worlds: three allies push the enemy outpost, four enemies raid."""
    arena = TacticalMap(config)
    base = config.structure("own_base").position
    outpost = config.structure("enemy_outpost").position
    raid = config.structure("enemy_base").position
    own = arena.nearest_legal(_between(base, outpost, 0.15))
    allies = [arena.nearest_legal(_between(base, outpost, f), own) for f in (0.4, 0.55, 0.7)]
    enemies = [arena.nearest_legal(_between(raid, base, f), own) for f in (0.3, 0.45, 0.6, 0.75)]
    model = WorldModel(config)
    for k in range(steps):
        now = 10.0 + k * dt
        for sample in range(5):
            stamp = now - dt + (sample + 1) * dt / 5
            model.observe(Observation(config.self_id, Team.ALLY, own, stamp), stamp)
            for i, p in enumerate(allies):
                allies[i] = arena.advance(p, outpost, 1.2 * dt / 5) if arena.legal(outpost) else p
                model.observe(Observation(i + 1, Team.ALLY, allies[i], stamp), stamp)
            for i, p in enumerate(enemies):
                target = arena.nearest_legal(base, own)
                enemies[i] = arena.advance(p, target, 1.0 * dt / 5)
                model.observe(Observation(101 + i, Team.ENEMY, enemies[i], stamp), stamp)
        model.update_game(GameState(now, config.season, config.own_team, Phase.RUNNING,
                                    max(0.0, 420.0 - k * dt), True), now)
        model.update_robot(RobotStatus(config.self_id, Team.ALLY, now, 300.0, 400.0, True,
                                       ammo=0.8, source=1), now)
        for name in ("own_base", "own_outpost", "enemy_base", "enemy_outpost"):
            model.update_structure(StructureState(name, now, alive=True, source=1), now)
        yield model.snapshot(now)


def measure(config, steps: int, prior_path=None) -> dict:
    def engine():
        prior = GoalPrior.load(prior_path, config, allow_transfer=True) if prior_path else None
        return DecisionEngine(config, session_id="benchmark", goal_prior=prior)

    worlds = list(scenario(config, steps))
    started = time.perf_counter()
    decider = engine()
    setup_ms = (time.perf_counter() - started) * 1000
    wall, cpu, tasks = [], [], {}
    for world in worlds:
        w0, c0 = time.perf_counter(), time.process_time()
        decision = decider.step(world)
        wall.append((time.perf_counter() - w0) * 1000)
        cpu.append((time.process_time() - c0) * 1000)
        tasks[decision.task.kind.name] = tasks.get(decision.task.kind.name, 0) + 1
    tracemalloc.start()
    decider = engine()
    for world in worlds[:50]:
        decider.step(world)
    _, peak = tracemalloc.get_traced_memory()
    tracemalloc.stop()

    def summary(values):
        return {"p50": float(np.percentile(values, 50)), "p95": float(np.percentile(values, 95)),
                "max": float(max(values)), "mean": float(np.mean(values))}

    return {"steps": steps, "setup_ms": setup_ms, "step_wall_ms": summary(wall),
            "step_cpu_ms": summary(cpu), "cpu_share_at_2hz": float(np.mean(cpu)) / 500.0,
            "tracemalloc_peak_kib": peak / 1024, "tasks": tasks,
            "obstacles": len(config.obstacles) + len(config.keepouts),
            "graph_corners": len(TacticalMap(config).corners)}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--configs", default=",".join(CONFIGS))
    parser.add_argument("--steps", type=int, default=200)
    parser.add_argument("--rl-model", help="optional exported actor.json to include GoalPrior")
    parser.add_argument("--out")
    args = parser.parse_args(argv)
    results = {}
    for name in args.configs.split(","):
        config = load_config(ROOT / "src/sentry_bringup/config" / CONFIGS[name])
        results[name] = measure(config, args.steps, args.rl_model)
    report = {
        "schema_version": 1, "tool": "tools/benchmark_decision.py",
        "recorded_on": time.strftime("%Y-%m-%d"), "policy": "rl" if args.rl_model else "rule",
        "environment": {"python": platform.python_version(), "numpy": np.__version__,
                        "machine": platform.machine(), "processor": platform.processor(),
                        "cpu_count": os.cpu_count()},
        "max_rss_kib": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
        "results": results,
        "limitations": "Synthetic tracks on the tactical map; Nav2, ROS transport and real "
                       "sensor load are not included.",
    }
    text = json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)
    if args.out:
        Path(args.out).parent.mkdir(parents=True, exist_ok=True)
        Path(args.out).write_text(text + "\n", encoding="utf-8")
    print(text)
    return 0 if all(math.isfinite(r["step_wall_ms"]["p95"]) for r in results.values()) else 1


if __name__ == "__main__":
    raise SystemExit(main())
