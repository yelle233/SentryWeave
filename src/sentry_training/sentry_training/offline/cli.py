"""Official-data BC/IQL/DT workflow. Heavy torch imports are restricted to learning commands."""

import argparse
import json
from pathlib import Path

from sentry_core.config import load_config
from sentry_core.navigation_features import NavigationSpec
from .data import BuildOptions, build_dataset, read_split


def _roles(value):
    aliases = {"infantry": (3, 4), "sentry": (7,), "mixed": (3, 4, 7), "hero": (1,), "engineer": (2,)}
    return aliases[value] if value in aliases else tuple(int(v) for v in value.split(","))


def main(argv=None):
    parser = argparse.ArgumentParser(description="RMUC official-data offline navigation learning")
    commands = parser.add_subparsers(dest="command", required=True)
    download = commands.add_parser("fetch")
    download.add_argument("--out", default="artifacts/rl/raw")
    build = commands.add_parser("build")
    build.add_argument("--db", required=True)
    build.add_argument("--out", required=True)
    build.add_argument("--roles", default="mixed")
    build.add_argument("--horizon", type=int, default=5)
    build.add_argument("--gamma", type=float, default=0.99)
    build.add_argument("--seed", type=int, default=29)
    inspect = commands.add_parser("inspect")
    inspect.add_argument("--data", required=True)
    fit = commands.add_parser("train")
    fit.add_argument("--data", required=True)
    fit.add_argument("--algorithm", choices=("bc", "iql", "dt"), required=True)
    fit.add_argument("--out", required=True)
    fit.add_argument("--seed", type=int, default=7)
    fit.add_argument("--steps", type=int, default=30000)
    fit.add_argument("--batch-size", type=int, default=512)
    fit.add_argument("--eval-every", type=int, default=1000)
    fit.add_argument("--roles", default="infantry")
    fit.add_argument("--device", default="auto")
    fit.add_argument("--init-actor")
    fit.add_argument("--hidden", type=int, default=128)
    fit.add_argument("--context", type=int, default=8)
    evaluate = commands.add_parser("evaluate")
    evaluate.add_argument("--data", required=True)
    evaluate.add_argument("--actors", nargs="+", required=True)
    evaluate.add_argument("--out", required=True)
    evaluate.add_argument("--device", default="auto")
    fqe = commands.add_parser("fqe")
    fqe.add_argument("--data", required=True)
    fqe.add_argument("--actor", required=True)
    fqe.add_argument("--out", required=True)
    fqe.add_argument("--steps", type=int, default=8000)
    fqe.add_argument("--seeds", default="17,23")
    fqe.add_argument("--device", default="auto")
    closed = commands.add_parser("closed-loop")
    closed.add_argument("--actors", nargs="+", required=True)
    closed.add_argument("--config", default="src/sentry_bringup/config/tactics.yaml")
    closed.add_argument("--out", required=True)
    closed.add_argument("--seeds", default="401,402,403")
    closed.add_argument("--duration", type=float, default=30.)
    closed.add_argument("--blend", type=float, default=0.25)
    closed.add_argument("--allow-domain-transfer", action="store_true")
    play = commands.add_parser("replay")
    play.add_argument("--actor", required=True)
    play.add_argument("--trace", required=True)
    play.add_argument("--config", default="src/sentry_bringup/config/tactics.yaml")
    play.add_argument("--blend", type=float, default=0.25)
    play.add_argument("--mirror", action="store_true")
    play.add_argument("--intent-model")
    play.add_argument("--utility-model")
    play.add_argument("--allow-domain-transfer", action="store_true")
    args = parser.parse_args(argv)
    if args.command == "fetch":
        from .fetch import fetch
        result = fetch(args.out)
    elif args.command == "build":
        options = BuildOptions(seed=args.seed, gamma=args.gamma, roles=_roles(args.roles),
                               spec=NavigationSpec(horizon=args.horizon))
        result = build_dataset(args.db, args.out, options, progress=print)
    elif args.command == "inspect":
        result = {s: {"transitions": len(data := read_split(args.data, s)),
                      "episodes": len(data.episodes), "metadata": data.metadata["splits"][s]}
                  for s in ("train", "validation", "test")}
    elif args.command == "train":
        from .train import TrainingOptions, train
        options = TrainingOptions(
            algorithm=args.algorithm, steps=args.steps, batch_size=args.batch_size,
            eval_every=args.eval_every, seed=args.seed, roles=_roles(args.roles),
            device=args.device, hidden=args.hidden, context=args.context,
        )
        result = train(args.data, args.out, options, initial_actor=args.init_actor, progress=print)
    elif args.command == "evaluate":
        from .evaluate import compare
        result = compare(args.data, args.actors, args.out, args.device)["aggregate"]
    elif args.command == "fqe":
        from .evaluate import evaluate_fqe
        result = evaluate_fqe(args.data, args.actor, args.steps,
                              tuple(int(s) for s in args.seeds.split(",")), args.device, args.out)
    elif args.command == "replay":
        from sentry_core.learned import IntentModel, load_utility
        from sentry_core.rl_policy import GoalPrior
        from sentry_training.evaluate import replay
        config = load_config(args.config)
        if args.utility_model:
            config = load_utility(args.utility_model, config)
        intent = IntentModel.load(args.intent_model, config) if args.intent_model else None
        prior = GoalPrior.load(args.actor, config, args.blend, args.mirror or None,
                               args.allow_domain_transfer)
        result = replay(config, args.trace, model=intent, policy="rl", goal_prior=prior)
    else:
        result = _closed_loop(args)
    print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))


def _closed_loop(args):
    import numpy as np
    from sentry_core.rl_policy import GoalPrior, NumpyNavigationPolicy
    from sentry_training.simulation import SCENARIOS, rollout

    config = load_config(args.config)
    seeds = [int(s) for s in args.seeds.split(",")]
    runs = []
    for baseline in ("fixed", "rule"):
        runs.extend(rollout(config, scenario, seed, duration=args.duration, policy=baseline)
                    for scenario in SCENARIOS for seed in seeds)
    for path in args.actors:
        model = NumpyNavigationPolicy.load(path)
        name = Path(path).parent.name
        for scenario in SCENARIOS:
            for seed in seeds:
                prior = GoalPrior(model, config, args.blend, allow_transfer=args.allow_domain_transfer)
                row = rollout(config, scenario, seed, duration=args.duration, policy="rl", goal_prior=prior)
                row.update(policy=name, algorithm=model.payload["algorithm"],
                           domain_transfer=prior.domain_transfer, blend=args.blend)
                runs.append(row)
    summary = {}
    for policy in sorted({row["policy"] for row in runs}):
        rows = [row for row in runs if row["policy"] == policy]
        summary[policy] = {"runs": len(rows)}
        for metric in ("reward", "support_coverage", "defense_coverage", "switches",
                       "illegal_goals", "unsafe_motion_steps", "rl_used_steps", "rl_rank_changes",
                       "decision_p95_ms"):
            values = [r[metric] for r in rows if r[metric] is not None]
            summary[policy][metric] = float(np.mean(values)) if values else None
    result = {"schema_version": 1, "environment": "synthetic kinematic arena",
              "domain_transfer_opt_in": args.allow_domain_transfer, "seeds": seeds,
              "summary": summary, "runs": runs,
              "limitations": "Different map, scripted peers and surrogate reward; not official match performance."}
    output = Path(args.out)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + "\n")
    return summary


if __name__ == "__main__":
    main()
