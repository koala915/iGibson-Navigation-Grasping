#!/usr/bin/env bash
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
tmp_dir="$(mktemp -d)"
trap 'rm -rf -- "$tmp_dir"' EXIT

device="$tmp_dir/myserial"
: >"$device"

make_helper() {
    local name="$1" body="$2"
    printf '#!/usr/bin/env bash\n%s\n' "$body" >"$tmp_dir/$name"
    chmod +x "$tmp_dir/$name"
}

expect_status() {
    local expected="$1"
    shift
    set +e
    "$@" >/dev/null 2>&1
    local actual=$?
    set -e
    if [[ "$actual" -ne "$expected" ]]; then
        echo "expected exit $expected, got $actual: $*" >&2
        exit 1
    fi
}

make_helper fuser-free 'exit 1'
make_helper fuser-owned 'printf "4321\\n"; exit 0'
make_helper fuser-error 'printf "permission denied\\n" >&2; exit 1'
make_helper fuser-broken 'exit 4'

X3PLUS_FUSER_BIN="$tmp_dir/fuser-free" \
    "$here/check-serial-owner.sh" "$device" >/dev/null
expect_status 3 env X3PLUS_FUSER_BIN="$tmp_dir/fuser-owned" \
    "$here/check-serial-owner.sh" "$device"
expect_status 2 env X3PLUS_FUSER_BIN="$tmp_dir/fuser-error" \
    "$here/check-serial-owner.sh" "$device"
expect_status 2 env X3PLUS_FUSER_BIN="$tmp_dir/fuser-broken" \
    "$here/check-serial-owner.sh" "$device"
expect_status 2 env X3PLUS_FUSER_BIN="$tmp_dir/missing" \
    "$here/check-serial-owner.sh" "$device"
expect_status 2 "$here/check-serial-owner.sh" "$tmp_dir/missing-device"

# Source only the resolver functions; the script's main guard prevents ROS
# setup files and the motor server from being executed by this test.
source "$here/start-navigation.sh"
valid_ipv4 192.168.0.42
! valid_ipv4 127.0.0.1
! valid_ipv4 999.1.1.1
! valid_ipv4 nonsense

make_helper ip-route 'printf "1.1.1.1 via 192.168.0.1 dev wlan0 src 192.168.0.42 uid 1000\\n"'
resolved="$(X3PLUS_IP_BIN="$tmp_dir/ip-route" discover_ros_ip)"
[[ "$resolved" == 192.168.0.42 ]]

make_helper ip-one-address '
if [[ "$*" == *"route get"* ]]; then exit 2; fi
printf "2: wlan0 inet 10.0.0.8/24 brd 10.0.0.255 scope global wlan0\\n"
'
resolved="$(X3PLUS_IP_BIN="$tmp_dir/ip-one-address" discover_ros_ip)"
[[ "$resolved" == 10.0.0.8 ]]

make_helper ip-ambiguous '
if [[ "$*" == *"route get"* ]]; then exit 2; fi
printf "2: wlan0 inet 10.0.0.8/24 brd 10.0.0.255 scope global wlan0\\n"
printf "3: eth0 inet 192.168.0.8/24 brd 192.168.0.255 scope global eth0\\n"
'
expect_status 1 env X3PLUS_IP_BIN="$tmp_dir/ip-ambiguous" \
    bash -c "source '$here/start-navigation.sh'; discover_ros_ip"

# Run main() end to end against stand-in ROS setup files. The first one reads
# a variable it never sets, as ROS's real profile.d/1.ros_distro.sh does with
# ROS_DISTRO: main() must source it with nounset off, then reach the server.
# The resolver tests above never got this far, which is how that bug shipped.
printf '%s\n' 'export SEEN_ROS="${ROS_DISTRO_NEVER_SET}ros"' >"$tmp_dir/ros-setup.bash"
printf '%s\n' 'export SEEN_WS=ws' >"$tmp_dir/ws-setup.bash"
make_helper fake-python 'printf "server %s %s args=%s\\n" "$SEEN_ROS" "$SEEN_WS" "$*"'
started="$(ROS_IP=192.168.0.42 \
    X3PLUS_ROS_SETUP="$tmp_dir/ros-setup.bash" \
    X3PLUS_WS_SETUP="$tmp_dir/ws-setup.bash" \
    X3PLUS_MOTOR_SERVER=/opt/motor_server.py \
    X3PLUS_PYTHON="$tmp_dir/fake-python" \
    "$here/start-navigation.sh")"
[[ "$started" == *"server ros ws args=-u /opt/motor_server.py"* ]]
expect_status 2 env ROS_IP=999.0.0.1 "$here/start-navigation.sh"

echo "systemd helper tests passed"
