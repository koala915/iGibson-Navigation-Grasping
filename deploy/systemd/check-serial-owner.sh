#!/usr/bin/env bash
set -euo pipefail

# Exit 0 only when the device exists and no process owns it.  fuser uses the
# same non-zero status for "no owner" and some failures, so stderr must also be
# inspected instead of blindly negating its exit code.

main() {
    local device fuser_bin tmp_dir status stdout stderr
    device="${1:-/dev/myserial}"

    if [[ ! -e "$device" ]]; then
        echo "serial-owner check failed: device does not exist: $device" >&2
        return 2
    fi

    fuser_bin="${X3PLUS_FUSER_BIN:-$(command -v fuser || true)}"
    if [[ -z "$fuser_bin" || ! -x "$fuser_bin" ]]; then
        echo "serial-owner check failed: fuser is unavailable" >&2
        return 2
    fi

    tmp_dir="$(mktemp -d)" || {
        echo "serial-owner check failed: cannot create temporary directory" >&2
        return 2
    }
    set +e
    "$fuser_bin" "$device" >"$tmp_dir/stdout" 2>"$tmp_dir/stderr"
    status=$?
    set -e
    stdout="$(<"$tmp_dir/stdout")"
    stderr="$(<"$tmp_dir/stderr")"
    rm -rf -- "$tmp_dir"

    case "$status" in
        0)
            echo "serial device is already owned: $device" >&2
            [[ -z "$stderr" ]] || printf '%s\n' "$stderr" >&2
            [[ -z "$stdout" ]] || printf '%s\n' "$stdout" >&2
            return 3
            ;;
        1)
            if [[ -n "$stderr" ]]; then
                echo "serial-owner check failed while inspecting $device:" >&2
                printf '%s\n' "$stderr" >&2
                return 2
            fi
            echo "serial device is free: $device"
            return 0
            ;;
        *)
            echo "serial-owner check failed: fuser exited with status $status" >&2
            [[ -z "$stderr" ]] || printf '%s\n' "$stderr" >&2
            return 2
            ;;
    esac
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
    main "$@"
fi
