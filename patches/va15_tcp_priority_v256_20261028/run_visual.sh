#!/usr/bin/env bash
set -euo pipefail
kit="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec /usr/bin/python3 "${kit}/session.py" "$@"
