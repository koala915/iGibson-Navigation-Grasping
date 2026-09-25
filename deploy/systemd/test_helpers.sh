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

echo "systemd helper tests passed"
