#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
asset_dir="${repo_dir}/assets/b2_z1_no_gun_description"
target_dir="${repo_dir}/src/rl_sar_zoo/b2_z1_no_gun_description"

if [[ ! -d "${asset_dir}/mjcf/assets" ]]; then
    echo "Missing B2+Z1 packaged assets: ${asset_dir}" >&2
    exit 1
fi
if [[ ! -d "${repo_dir}/src/rl_sar_zoo/.git" && ! -f "${repo_dir}/src/rl_sar_zoo/.git" ]]; then
    echo "Initialize submodules first: git submodule update --init --recursive" >&2
    exit 1
fi
if [[ -e "${target_dir}" ]]; then
    echo "Model directory already exists; leaving it unchanged: ${target_dir}"
    exit 0
fi
cp -a "${asset_dir}" "${target_dir}"
echo "Installed B2+Z1 model assets: ${target_dir}"
