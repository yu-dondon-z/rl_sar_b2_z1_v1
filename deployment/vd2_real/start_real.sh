#!/usr/bin/env bash
set -euo pipefail
export OMP_WAIT_POLICY=PASSIVE
export OPENBLAS_NUM_THREADS=1
vd2_package_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ -x "$vd2_package_dir/.venv/bin/python" ]]; then
    vd2_python="$vd2_package_dir/.venv/bin/python"
elif [[ -n "${VIRTUAL_ENV:-}" && -x "$VIRTUAL_ENV/bin/python" ]]; then
    vd2_python="$VIRTUAL_ENV/bin/python"
else
    vd2_python="python3"
fi
exec "$vd2_python" "$vd2_package_dir/tools/launch_real.py" "$@"
