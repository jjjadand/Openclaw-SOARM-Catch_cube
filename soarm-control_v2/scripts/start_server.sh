#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LEROBOT_ROOT="${LEROBOT_ROOT:-${HOME}/lerobot}"

export SOARM_API_HOST="${SOARM_API_HOST:-127.0.0.1}"
export SOARM_API_PORT="${SOARM_API_PORT:-8000}"
export SOARM_PORT="${SOARM_PORT:-/dev/ttyACM0}"
export SOARM_ID="${SOARM_ID:-openclaw_soarm}"
export SOARM_SKIP_CALIBRATION="${SOARM_SKIP_CALIBRATION:-1}"
if [[ -d "${LEROBOT_ROOT}" ]]; then
  export PYTHONPATH="${LEROBOT_ROOT}:${PYTHONPATH:-}"
fi

exec ~/anaconda3/bin/conda run -n lerobot python "${SCRIPT_DIR}/soarm_api.py"
