# Paths and environment shared by the scripts. Source it, do not run it:
#
#   source scripts/env.sh
#
# Set EXTERNAL before sourcing to keep CARLA and SimLingo somewhere other than external/ (which may be a
# symlink to carla-track-control's, to share the downloads).

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export REPO
export EXTERNAL="${EXTERNAL:-$REPO/external}"
export CARLA_ROOT="${CARLA_ROOT:-$EXTERNAL/carla}"
export CARLA_PORT="${CARLA_PORT:-2000}"
# Python 3.8 (the CARLA 0.9.15 client only exists for 3.7 and 3.8): SimLingo's environment, where the
# leaderboard and both agents run (scripts/install_simlingo.sh makes it)
export PY="$EXTERNAL/simlingo_venv/bin/python"

# the repository's own module (expertctl) and the agents; nothing else, so a sourced ROS 2 setup cannot leak in
export PYTHONPATH="$REPO/build/python:$REPO"
export PATH="$HOME/.local/bin:$PATH"

# Vulkan (CARLA's renderer) needs a runtime directory, which an ssh session without a desktop does not have
if [[ -z "${XDG_RUNTIME_DIR:-}" || ! -d "${XDG_RUNTIME_DIR:-}" ]]; then
    export XDG_RUNTIME_DIR="/tmp/runtime-$(id -u)"
    mkdir -p "$XDG_RUNTIME_DIR"
    chmod 700 "$XDG_RUNTIME_DIR"
fi
