#!/usr/bin/env bash
# Sourced by the real launcher; no action is performed by sourcing this file.
ensure_local_z1_controller() {
    local repo_dir="$1"
    shift
    local disabled=false z1_ip=127.0.0.1 z1_port=8071
    local -a options=("$@") pids=()
    local i next
    for ((i=0;i<${#options[@]};++i)); do
        case "${options[i]}" in
            --disable-z1) disabled=true ;;
            --z1-ip|--z1-to-port)
                next=$((i+1))
                if ((next>=${#options[@]})); then
                    echo "${options[i]} requires a value" >&2;return 2
                fi
                if [[ "${options[i]}" == --z1-ip ]]; then z1_ip="${options[next]}";
                else z1_port="${options[next]}";fi
                i=$next ;;
        esac
    done
    [[ "$disabled" == true ]] && return 0
    if [[ "$z1_ip" == 192.168.123.110 ]]; then
        echo "Use the local Z1 SDK endpoint 127.0.0.1, not the arm lower-controller address." >&2
        return 2
    fi
    if [[ "$z1_ip" != 127.0.0.1 && "$z1_ip" != localhost ]]; then
        echo "Using explicit external Z1 SDK endpoint $z1_ip:$z1_port; local auto-start skipped."
        return 0
    fi
    if [[ ! "$z1_port" =~ ^[0-9]+$ ]] || ((${#z1_port}>5)); then
        echo "Invalid Z1 UDP port: $z1_port" >&2;return 2
    fi
    z1_port=$((10#$z1_port))
    if ((z1_port<1 || z1_port>65535)); then
        echo "Invalid Z1 UDP port: $z1_port" >&2;return 2
    fi
    command -v ss >/dev/null || { echo "ss is required to check the local Z1 UDP socket" >&2;return 1; }
    local candidate controller_stat
    while IFS= read -r candidate; do
        [[ -n "$candidate" ]] || continue
        controller_stat=$(ps -o stat= -p "$candidate" 2>/dev/null || true)
        if [[ -z "$controller_stat" || "$controller_stat" =~ ^[[:space:]]*Z ]]; then
            echo "Ignoring exited/zombie Z1 controller PID=$candidate."
            continue
        fi
        pids+=("$candidate")
    done < <(pgrep -x z1_ctrl || true)
    if ((${#pids[@]}>1)); then
        echo "Multiple z1_ctrl processes exist; refusing to start another: ${pids[*]}" >&2;return 1
    fi
    local pid log_file="" controller_root controller_binary
    if ((${#pids[@]}==1)); then
        pid="${pids[0]}"
        echo "Reusing local Z1 controller, PID=$pid."
    else
        controller_root="${RL_SAR_Z1_CONTROLLER_ROOT:-${HOME}/z1_controller}"
        controller_binary="$controller_root/build/z1_ctrl"
        [[ -x "$controller_binary" ]] || { echo "Missing Z1 controller: $controller_binary" >&2;return 1; }
        command -v setsid >/dev/null || { echo "setsid is required to isolate the Z1 controller from terminal Ctrl+C" >&2;return 1; }
        mkdir -p "$repo_dir/outputs/z1_controller"
        log_file="$repo_dir/outputs/z1_controller/z1_$(date +%Y%m%d_%H%M%S)_$$.log"
        pushd "$controller_root/build" >/dev/null || return 1
        # nohup only handles SIGHUP. The SDK controller installs a SIGINT
        # handler, so it must also leave the real program's terminal group.
        # Keep it alive while the real program requests PASSIVE at shutdown.
        nohup setsid "$controller_binary" </dev/null >"$log_file" 2>&1 &
        pid=$!
        popd >/dev/null || return 1
        printf '%s\n' "$pid" > "$repo_dir/outputs/z1_controller/launcher_started.pid"
        echo "Started local Z1 controller in isolated background session, PID=$pid; log=$log_file"
    fi
    for ((i=0;i<50;++i)); do
        if ! kill -0 "$pid" 2>/dev/null; then
            echo "Z1 controller exited before SDK UDP became ready." >&2
            [[ -z "$log_file" ]] || tail -15 "$log_file" >&2
            return 1
        fi
        if ss -H -uanp "( sport = :$z1_port )" 2>/dev/null | grep -Fq "pid=$pid,"; then
            echo "Local Z1 controller ready on UDP $z1_port."
            return 0
        fi
        sleep .1
    done
    echo "Z1 PID $pid has no confirmed SDK UDP socket $z1_port; B2 control was not started." >&2
    [[ -z "$log_file" ]] || tail -15 "$log_file" >&2
    return 1
}
