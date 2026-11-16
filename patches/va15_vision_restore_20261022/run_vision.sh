#!/usr/bin/env bash
set -euo pipefail
kit_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
vision_python="${RL_SAR_VISION_PYTHON:-$(command -v python3)}"
exec "${vision_python}" "${kit_dir}/session.py" "$@"
