#!/usr/bin/env python3
"""Start an isolated physics stack, verify navigation, and cleanly stop owned processes."""

import argparse
import os
from pathlib import Path
import signal
import subprocess
import sys
import uuid


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", default="artifacts/gazebo")
    parser.add_argument("--domain", type=int, default=78)
    parser.add_argument("--params-file", help="also validate a migrated upstream Nav2 configuration")
    parser.add_argument("--rl-model", help="exported NumPy actor JSON")
    parser.add_argument("--rl-blend", type=float, default=0.25)
    parser.add_argument("--allow-domain-transfer", action="store_true")
    args = parser.parse_args()
    if not 0 <= args.domain <= 232:
        parser.error("ROS domain must be in [0,232]")
    directory = Path(args.out).resolve()
    directory.mkdir(parents=True, exist_ok=True)
    env = {**os.environ, "ROS_DOMAIN_ID": str(args.domain),
           "GZ_PARTITION": f"sentryweave-acceptance-{uuid.uuid4().hex}",
           "SENTRYWEAVE_GAZEBO_TEST": "1", "SENTRYWEAVE_GAZEBO_REPORT": str(directory / "physics.json")}
    if args.rl_model:
        env["SENTRYWEAVE_RL_EXPECTED"] = "1"
    with (directory / "launch.log").open("w") as log:
        command = ["ros2", "launch", "sentry_bringup", "simulation.launch.py", "scenario:=team_push",
                   f"trace_path:={directory / 'decisions.jsonl'}"]
        if args.params_file:
            command.append(f"params_file:={Path(args.params_file).resolve()}")
        if args.rl_model:
            command.extend([
                f"rl_model:={Path(args.rl_model).resolve()}", f"rl_blend:={args.rl_blend}",
                f"rl_allow_transfer:={str(args.allow_domain_transfer).lower()}",
            ])
        launch = subprocess.Popen(
            command,
            stdout=log, stderr=subprocess.STDOUT, env=env, start_new_session=True,
        )
        try:
            result = subprocess.run(
                [sys.executable, "-m", "pytest", "tests/integration/test_gazebo_smoke.py",
                 "-q", "--tb=short", f"--junitxml={directory / 'pytest.xml'}"],
                env=env, timeout=110,
            )
            return result.returncode
        finally:
            if launch.poll() is None:
                launch.send_signal(signal.SIGINT)
                try:
                    launch.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(launch.pid, signal.SIGKILL)
                    launch.wait()


if __name__ == "__main__":
    sys.exit(main())
