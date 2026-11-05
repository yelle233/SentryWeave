"""Offline workflow entry point: ``python -m sentry_training.cli --help``."""

import argparse
import json
from pathlib import Path

from sentry_core.config import load_config
from sentry_core.learned import IntentModel, load_utility
from .dataset import extract_labeled, generate_dataset, read_dataset, write_dataset
from .evaluate import evaluate, replay
from .fit import train_intent, tune_utility, write_json
from .simulation import SCENARIOS, rollout


def _default_config():
    try:
        from ament_index_python.packages import get_package_share_directory
        return str(Path(get_package_share_directory("sentry_bringup")) / "config/tactics.yaml")
    except (ImportError, LookupError):
        return "src/sentry_bringup/config/tactics.yaml"


def main(argv=None):
    parser = argparse.ArgumentParser(description="哨兵意图训练、战术调参、仿真对照与回放")
    parser.add_argument("--config", default=_default_config())
    commands = parser.add_subparsers(dest="command", required=True)
    generate = commands.add_parser("generate", help="generate labeled synthetic trajectories")
    generate.add_argument("--out", required=True)
    generate.add_argument("--episodes-per-class", type=int, default=40)
    generate.add_argument("--steps", type=int, default=80)
    generate.add_argument("--seed", type=int, default=11)
    fit = commands.add_parser("train-intent", help="fit HMM, calibrate on validation, report held-out test")
    fit.add_argument("--data", required=True)
    fit.add_argument("--out", required=True)
    fit.add_argument("--seed", type=int, default=17)
    tune = commands.add_parser("tune-utility", help="closed-loop CEM weight search")
    tune.add_argument("--out", required=True)
    tune.add_argument("--iterations", type=int, default=6)
    tune.add_argument("--population", type=int, default=16)
    tune.add_argument("--seed", type=int, default=31)
    tune.add_argument("--duration", type=float, default=30)
    tune.add_argument("--model")
    test = commands.add_parser("evaluate", help="paired scenario/seed comparison")
    test.add_argument("--out", required=True)
    test.add_argument("--policies", default="fixed,rule")
    test.add_argument("--seeds", default="101,102,103")
    test.add_argument("--scenarios", default=",".join(SCENARIOS))
    test.add_argument("--duration", type=float, default=30)
    test.add_argument("--model")
    test.add_argument("--utility")
    episode = commands.add_parser("simulate", help="one closed-loop episode and JSONL trace")
    episode.add_argument("--scenario", choices=SCENARIOS, default="conflict")
    episode.add_argument("--seed", type=int, default=101)
    episode.add_argument("--duration", type=float, default=30)
    episode.add_argument("--trace", required=True)
    episode.add_argument("--policy", choices=("fixed", "rule", "learned", "no_hysteresis"), default="rule")
    episode.add_argument("--model")
    episode.add_argument("--utility")
    play = commands.add_parser("replay", help="run the same policy over recorded World snapshots")
    play.add_argument("--trace", required=True)
    play.add_argument("--model")
    play.add_argument("--utility")
    play.add_argument("--policy", default="rule", choices=("fixed", "rule", "learned", "no_hysteresis"))
    extract = commands.add_parser("extract", help="convert human-labeled trace intervals to trajectories")
    extract.add_argument("--trace", required=True)
    extract.add_argument("--annotations", required=True)
    extract.add_argument("--out", required=True)
    args = parser.parse_args(argv)
    config = load_config(args.config)
    baseline_config = config
    if getattr(args, "utility", None):
        config = load_utility(args.utility, config)
    model = IntentModel.load(args.model, config) if getattr(args, "model", None) else None
    if args.command == "generate":
        data = generate_dataset(config, args.episodes_per_class, args.seed, args.steps)
        write_dataset(args.out, data)
        result = {"sequences": len(data), "samples": sum(len(s.labels) for s in data),
                  "dataset": args.out, "source": "synthetic"}
    elif args.command == "train-intent":
        result = train_intent(read_dataset(args.data), config, args.out, args.seed)
    elif args.command == "tune-utility":
        result = tune_utility(config, args.iterations, args.population,
                              duration=args.duration, seed=args.seed, model=model)
        write_json(args.out, result)
    elif args.command == "evaluate":
        report = evaluate(baseline_config, tuple(args.policies.split(",")),
                          tuple(int(s) for s in args.seeds.split(",")),
                          tuple(args.scenarios.split(",")), args.duration, model, args.out,
                          learned_config=config)
        result = report["summary"]
    elif args.command == "simulate":
        result = rollout(config, args.scenario, args.seed, args.duration,
                         policy=args.policy, model=model, trace_path=args.trace)
    elif args.command == "replay":
        if args.policy == "learned" and model is None:
            parser.error("learned replay requires --model")
        result = replay(config, args.trace, model, args.policy)
    else:
        data = extract_labeled(config, args.trace, args.annotations)
        write_dataset(args.out, data)
        result = {"labeled_sequences": len(data), "dataset": args.out}
    print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
