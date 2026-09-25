#!/usr/bin/env bash
set -euo pipefail

valid_ipv4() {
    local value="$1" octet
    local -a octets

    [[ "$value" =~ ^[0-9]{1,3}(\.[0-9]{1,3}){3}$ ]] || return 1
    IFS=. read -r -a octets <<<"$value"
    [[ "${#octets[@]}" -eq 4 ]] || return 1
    for octet in "${octets[@]}"; do
        ((10#$octet <= 255)) || return 1
    done
    [[ "$value" != 0.0.0.0 && "$value" != 127.* ]]
}

discover_ros_ip() {
    local ip_bin route_output candidate
    local -a candidates

    ip_bin="${X3PLUS_IP_BIN:-$(command -v ip || true)}"
    if [[ -z "$ip_bin" || ! -x "$ip_bin" ]]; then
        echo "navigation startup failed: the ip command is unavailable" >&2
        return 1
    fi

    # Route lookup selects the address the kernel would advertise off-host. It
    # does not send a packet. This is preferable to taking the first hostname
    # address, which can select Docker or another secondary interface.
    if route_output="$($ip_bin -4 route get 1.1.1.1 2>/dev/null)"; then
        candidate="$(awk '{for (i=1; i<=NF; i++) if ($i == "src") {print $(i+1); exit}}' <<<"$route_output")"
        if valid_ipv4 "$candidate"; then
            printf '%s\n' "$candidate"
            return 0
        fi
    fi

    # A robot may be on an isolated LAN with no default route. Accept the sole
    # global IPv4 address, but fail closed when multiple interfaces are viable.
    mapfile -t candidates < <(
        "$ip_bin" -4 -o addr show up scope global 2>/dev/null |
            awk '{split($4, address, "/"); print address[1]}'
    )
    if [[ "${#candidates[@]}" -eq 1 ]] && valid_ipv4 "${candidates[0]}"; then
        printf '%s\n' "${candidates[0]}"
        return 0
    fi

    echo "navigation startup failed: cannot choose one reachable ROS_IP" >&2
    echo "set ROS_IP explicitly in /etc/default/x3plus-navigation" >&2
    return 1
}

main() {
    if [[ -n "${ROS_IP:-}" ]]; then
        if ! valid_ipv4 "$ROS_IP"; then
            echo "navigation startup failed: invalid ROS_IP override: $ROS_IP" >&2
            return 2
        fi
    else
        ROS_IP="$(discover_ros_ip)" || return 2
    fi
    export ROS_IP
    unset ROS_HOSTNAME

    echo "navigation ROS_IP=$ROS_IP"

    source /opt/ros/melodic/setup.bash
    source /home/jetson/ROS/X3/yahboomcar_ws/devel/setup.bash
    exec python3 -u \
        /home/jetson/ROS/X3/yahboomcar_ws/src/yahboomcar_bringup/scripts/ai_motor_server_P0.py
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
    main "$@"
fi
