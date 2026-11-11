"""Validate a tactics or field file: ``sentry-field FILE [--own-team BLUE]``.

Checks bounds, convex obstacles/keep-outs, points outside obstacles, complete
structures, mirror consistency and grid reachability of zones and points.
"""

import argparse
import json
from pathlib import Path

import yaml

from .config import Config, load_config, sentry_id, validate_config
from .field import field_problems, read_field

DEFAULT_WEIGHTS = (0.22, 0.24, 0.28, 0.12, 0.10, 0.04)


def check_file(path, own_team: str = "RED", robot_radius: float = 0.32) -> dict:
    path, own_team = Path(path), str(own_team).upper()
    report = {"file": str(path), "own_team": own_team, "ok": False, "problems": []}
    try:
        raw = yaml.safe_load(path.read_text(encoding="utf-8"))
        if isinstance(raw, dict) and "field_version" in raw:
            # A bare field file: the sentry id follows the team like tactics without self_id.
            config = validate_config(Config(
                frame="map", self_id=sentry_id(own_team), robot_radius=robot_radius,
                weights=DEFAULT_WEIGHTS, own_team=own_team,
                **read_field(raw, path.parent, own_team)))
        else:
            config = load_config(path, own_team, check=False)
        report["problems"] = field_problems(config, reachability=True)
    # Malformed or missing files are reported as problems, not tracebacks.
    except (AttributeError, KeyError, OSError, TypeError, ValueError, yaml.YAMLError) as error:
        report["problems"] = [f"{type(error).__name__}: {error}"]
        return report
    report.update(
        ok=not report["problems"], season=config.season, site=config.site,
        self_id=config.self_id, mirror=config.mirror, mirrored=config.mirrored,
        bounds=list(config.bounds),
        counts={"obstacles": len(config.obstacles), "keepouts": len(config.keepouts),
                "structures": len(config.structures), "zones": len(config.zones),
                "points": len(config.points)},
    )
    return report


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="校验战术/场地语义文件")
    parser.add_argument("file")
    parser.add_argument("--own-team", choices=("RED", "BLUE"), default="RED")
    parser.add_argument("--robot-radius", type=float, default=0.32,
                        help="robot radius for bare field files")
    args = parser.parse_args(argv)
    report = check_file(args.file, args.own_team, args.robot_radius)
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
