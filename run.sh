#!/usr/bin/env bash
# Entrypoint for calibrated-wheeled-odometry.
set -euo pipefail

cd "$(dirname "$0")"

VENV_NAME="venv"
if [ ! -x "${VENV_NAME}/bin/python" ]; then
    ./setup.sh
fi

# shellcheck disable=SC1091
source "${VENV_NAME}/bin/activate"

export PYTHONPATH="${PWD}/src${PYTHONPATH:+:${PYTHONPATH}}"
exec python -m main "$@"
