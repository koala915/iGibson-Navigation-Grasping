#!/bin/bash
# Fallback search: when E1 sees nothing, hand the robot to the three-pose scan.
#
# The scanner is an exclusive phase by design -- it owns the camera AND the
# servo bus and exits before the PPO controller starts -- so it cannot run
# while the resident services hold those devices. Rather than teach the service
# to hand its serial port back and forth (a seam the controller does not
# document), this stops both services, runs the launcher's normal scan+grasp
# flow, and brings the services back. The fallback therefore pays the full
# ~23 s startup; the common E1 case never touches this script.
#
# --unlock-unvalidated-scan stays an explicit, visible flag here because what it
# unlocks is still true: the LEFT/RIGHT poses' real-robot mapping has not been
# verified to 1 cm, so a rotated-view target is an experiment, not a procedure.
set -e

V23=/home/jetson/Documents/deploy_jetson2/grasp/v23
PY=/home/jetson/grasp_venv/bin/python3

echo "[graspscan] stopping the resident services (they hold the camera and /dev/myserial)"
sudo systemctl stop grasp-vision grasp-service

restore() {
    echo "[graspscan] restarting the resident services"
    sudo systemctl start grasp-service grasp-vision
    echo "[graspscan] the service needs ~30 s to reload PyBullet before graspctl works"
}
trap restore EXIT

cd "$V23"
echo "[graspscan] running the three-pose scan + grasp"
"$PY" -u jetson_one_command_grasp.py \
    --three-pose-scan \
    --unlock-unvalidated-scan \
    --floor-finger-error-mm 15 \
    --jaw-track-fraction 0.5 \
    --tcp-forward-error-mm 20 \
    "$@"
