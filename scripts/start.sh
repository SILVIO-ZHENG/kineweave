#!/usr/bin/env bash
# Launch local simulation from any directory.
set -eu
cd -- "$(dirname -- "$0")/.."
exec .venv/bin/python -m teleop_v2 serve --driver sim --robot "${1:-g2_24}" "${@:2}"
