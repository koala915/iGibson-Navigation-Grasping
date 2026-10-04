#!/usr/bin/env bash
# Rear camera only; never stop grasp-vision or take its arm camera.
set -euo pipefail
HOST=${1:?usage: start_rear_approach_stream.sh WINDOWS_IPV4 [PORT]}
PORT=${2:-5600}
[[ "$HOST" =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]] || exit 2
[[ "$PORT" =~ ^[0-9]+$ ]] && (( PORT > 0 && PORT < 65536 )) || exit 2
REAR=/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera_SN0001-video-index0
ARM=/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera-video-index0
[[ -e "$REAR" && -e "$ARM" ]] || { echo 'camera identities missing'; exit 2; }
[[ "$(readlink -f "$REAR")" != "$(readlink -f "$ARM")" ]] || exit 2
if fuser "$REAR" >/dev/null 2>&1; then
    echo 'rear camera already owned; refusing to open a second reader'
    exit 2
fi
exec gst-launch-1.0 -e v4l2src device="$REAR" do-timestamp=true \
  ! 'video/x-raw,format=YUY2,width=640,height=480,framerate=30/1' \
  ! videoconvert ! 'video/x-raw,format=I420' \
  ! nvvidconv ! 'video/x-raw(memory:NVMM),format=NV12' \
  ! nvv4l2h264enc bitrate=2000000 insert-sps-pps=true iframeinterval=30 \
  ! h264parse ! rtph264pay config-interval=1 pt=96 \
  ! udpsink host="$HOST" port="$PORT" sync=false async=false
