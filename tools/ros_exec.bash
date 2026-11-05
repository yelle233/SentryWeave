#!/usr/bin/env bash
# Execute a command with Jazzy and this workspace sourced, including in Docker.
set -e
source /opt/ros/jazzy/setup.bash
sentryweave_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ -f "$sentryweave_root/install/setup.bash" ]]; then
    source "$sentryweave_root/install/setup.bash"
fi
exec "$@"
