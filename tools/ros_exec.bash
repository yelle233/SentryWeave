#!/usr/bin/env bash
# Execute a command with Jazzy and this workspace sourced, including in Docker.
set -e
source /opt/ros/jazzy/setup.bash
rmjc_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ -f "$rmjc_root/install/setup.bash" ]]; then
    source "$rmjc_root/install/setup.bash"
fi
exec "$@"
