#!/usr/bin/env bash
# Install only the operator-confirmed no-tool GetDown candidate, never run control.
set -euo pipefail
repo=/home/unitree/rl_sar_b2_z1
stage=$(realpath -- "${1:?explicit staging directory required}")
case "$stage" in "$repo"/diagnostics/parallel_getdown_20261021_*) ;; *) exit 2 ;; esac
cd "$repo"
if pgrep -af '[r]l_real_b2_z1_no_gun|[r]un_b2_z1_real(_vision)?[.]sh'; then
    echo 'Refusing deployment while a B2 control/launcher process exists.' >&2
    exit 3
fi
check_hash() {
    test "$(sha256sum "$1" | awk '{print $1}')" = "$2" || {
        echo "Unexpected source/config drift: $1" >&2; exit 4;
    }
}
check_protected() {
    check_hash policy/b2_z1_no_gun/VA15/config.yaml 8faa51628b3704ce3e7e3a384f1c0eb8bb59b7dcf5a0069992069efed8aab8d1
    check_hash policy/b2_z1_no_gun/VA15/policy.pt 8dfe4bcad4f0f2edd5d51858ef6e7f5ef9403c2db967cad8f45759e4bdb71098
    check_hash src/rl_sar/library/core/rl_sdk/va15_controlled_stop.hpp 85c448f829a7ebabd9d9b928698cac3ccaa791a1d8c7eb69bc3eb15998816386
    check_hash diagnostics/independent_state_20261020_L7VGqH/va15_independent_state.hpp e76a733a3428ebaaa27b792b4005a69aac18e0eac2eb0824d9762379a707a3f9
}
check_protected
check_hash src/rl_sar/src/rl_real_b2_z1_no_gun.cpp 5228618f2be9094feb55cd5d8d6a1acd048792821600cfcf5adf28fbd31f64c8
check_hash src/rl_sar/fsm_robot/fsm_b2_z1_no_gun.hpp 435b18eddc2381c53ce7e31ab93386eaa86393e5c9eeb37841a942649e131cf3
check_hash "$stage/fsm_b2_z1_no_gun.hpp" 5ff38043ccab5ceeaeec09bf5828ac26da2b093a72a725d9cd43f132a0a91d94
check_hash "$stage/rl_real_b2_z1_no_gun.cpp" 6cff6770d520e95ef2cd395959bacfbcffacf6c19da185329787339f01666a6f
backup=$(mktemp -d "$repo/deployment_backups/parallel_getdown_20261021_XXXXXX")
echo "Recoverable backup: $backup"
checks=$(mktemp -d "$stage/install_checks_XXXXXX")
echo "Offline check reports: $checks"
targets=(src/rl_sar/fsm_robot/fsm_b2_z1_no_gun.hpp
    src/rl_sar/src/rl_real_b2_z1_no_gun.cpp policy/b2_z1_no_gun/VA15/manifest.json
    cmake_build_va15/bin/rl_real_b2_z1_no_gun)
cp --parents -- "${targets[@]}" "$backup/"
restore_on_error() {
    result=$?
    if (( result != 0 )); then
        for item in "${targets[@]}"; do cp -- "$backup/$item" "$repo/$item"; done
        echo "Candidate check failed; original source, manifest and binary restored from $backup" >&2
    fi
}
trap restore_on_error EXIT
cp -- "$stage/fsm_b2_z1_no_gun.hpp" src/rl_sar/fsm_robot/fsm_b2_z1_no_gun.hpp
cp -- "$stage/rl_real_b2_z1_no_gun.cpp" src/rl_sar/src/rl_real_b2_z1_no_gun.cpp
cp -- "$stage/manifest.json" policy/b2_z1_no_gun/VA15/manifest.json
if ! cmake --build cmake_build_va15 --target rl_real_b2_z1_no_gun -j2 >"$checks/build.log" 2>&1; then
    tail -70 "$checks/build.log"; exit 5
fi
if ! ./scripts/run_b2_z1_real.sh --offline-check >"$checks/offline_check.log" 2>&1; then
    tail -70 "$checks/offline_check.log"; exit 6
fi
for test_source in "$stage/test_parallel_getdown.cpp" \
    "$stage/test_parallel_stop_gate.cpp" "$repo/scripts/test_va15_controlled_stop.cpp" \
    "$repo/scripts/test_va15_stand_gains.cpp"; do
    python3 "$stage/verify_board_candidate.py" --repo "$repo" --test-source "$test_source" \
        --output "$checks/native_$(basename "$test_source" .cpp)"
done
check_protected
check_hash src/rl_sar/fsm_robot/fsm_b2_z1_no_gun.hpp 5ff38043ccab5ceeaeec09bf5828ac26da2b093a72a725d9cd43f132a0a91d94
check_hash src/rl_sar/src/rl_real_b2_z1_no_gun.cpp 6cff6770d520e95ef2cd395959bacfbcffacf6c19da185329787339f01666a6f
trap - EXIT
sed -n '1,100p' "$checks/offline_check.log"
echo 'PASS: board offline checks only; no control startup, DDS or Z1 initialization.'
echo 'Dynamic motion remains gated. New parallel GetDown is not a hardware acceptance.'
