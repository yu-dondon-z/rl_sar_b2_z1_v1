#!/usr/bin/env bash
# Historical launcher; original options retained, not the VD13 entry.
printf '[历史入口] debug_vd5_gait_compare.sh：仅供旧版本复现；当前 VD13 见 docs/README.md。\n' >&2
# Offline matched MuJoCo/Isaac runs, sequentially to avoid GPU interference.
set -euo pipefail
vd5_probe_root="$(cd -- "$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")/../../.." && pwd)"
vd5_probe_engine="${1:-both}"
if [[ "$#" -gt 0 ]]; then shift; fi
case "$vd5_probe_engine" in mujoco|isaac|both) ;; *) printf 'Usage: bash scripts/debug_vd5_gait_compare.sh [both|mujoco|isaac] [--duration 60 ...]\n'; exit 2;; esac
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1
if [[ "$vd5_probe_engine" != isaac ]]; then
    MUJOCO_GL=egl /home/dong/anaconda3/envs/crack_vision/bin/python \
        "$vd5_probe_root/scripts/diagnose_vd5_gait.py" --engine mujoco "$@"
fi
if [[ "$vd5_probe_engine" != mujoco ]]; then
    vd5_probe_cache="$(mktemp -d /dev/shm/vd5_gait_isaac_XXXXXXXX)"
    mkdir -p "$vd5_probe_cache/cache" "$vd5_probe_cache/tmp" "$vd5_probe_cache/mpl" "$vd5_probe_cache/assets"
    export XDG_CACHE_HOME="$vd5_probe_cache/cache" TMPDIR="$vd5_probe_cache/tmp"
    export MPLCONFIGDIR="$vd5_probe_cache/mpl" VD3_ASSET_CACHE="$vd5_probe_cache/assets"
    export PYTHONPATH="/home/dong/b2_z1_VD2/source/b2_z1${PYTHONPATH:+:$PYTHONPATH}"
    /home/dong/anaconda3/envs/env_isaaclab/bin/python \
        "$vd5_probe_root/scripts/diagnose_vd5_gait.py" --engine isaac --device cuda:0 \
        '--kit_args=--/persistent/isaac/asset_root/cloud=/home/dong/isaacsim_assets/Assets/Isaac/4.5' "$@"
fi
