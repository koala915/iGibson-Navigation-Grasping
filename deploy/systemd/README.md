# X3Plus `/dev/myserial` ownership

`grasp-service.service` and `x3plus-navigation.service` are mutually exclusive.
They must never run at the same time because both open the Rosmaster controller
through `/dev/myserial`.

The units carry both `Conflicts=` and an ordering edge
(`grasp-service.service` is `Before=x3plus-navigation.service`, equivalently
navigation is `After=grasp-service.service`). When switching modes, systemd
therefore finishes stopping the current owner before it starts the new one.
The grasp drop-in also refuses to start if an unmanaged process still has
`/dev/myserial` open. The arm's posture-change delay is extra margin, not part
of the ownership guarantee.

Since G2 (2026-09-28) `g2-chassis.conf` makes the grasp service drive the wheels
too, on TCP 7000 with the same velocity protocol as
`integration/sugarbox_rl_motor_server.py`. The approach controller therefore
needs no mode switch at all; `x3plus-navigation` is only for the older
`{"action","speed"}` tools and maintenance. Two things to know:

- `systemctl stop/restart grasp-service` zeroes the wheels before the process
  exits (SIGTERM takes the Ctrl+C path). Verified on the robot, wheels off the
  ground, 3/3.
- The unit allows 3 starts in 300 s. A fourth `restart` inside that window
  leaves the service `failed (start-limit-hit)`; recover with
  `sudo systemctl reset-failed grasp-service && sudo systemctl start grasp-service grasp-vision`.

Navigation chooses `ROS_IP` from the kernel's default IPv4 route at each
startup. On an isolated LAN with no default route, it accepts the only global
IPv4 address. Multiple viable addresses are treated as ambiguous and stop the
service. An operator can override the choice in `/etc/default/x3plus-navigation`:

```bash
ROS_IP=192.168.0.201
```

Install on the Jetson:

```bash
sudo install -m 0644 x3plus-navigation.service /etc/systemd/system/
sudo install -d -m 0755 /etc/systemd/system/grasp-service.service.d
sudo install -m 0644 grasp-service.service.d/serial-owner.conf \
  grasp-service.service.d/g2-chassis.conf \
  /etc/systemd/system/grasp-service.service.d/
sudo install -d -m 0755 /usr/local/libexec/x3plus
sudo install -m 0755 check-serial-owner.sh start-navigation.sh \
  /usr/local/libexec/x3plus/
sudo systemctl daemon-reload
sudo systemd-analyze verify \
  /etc/systemd/system/x3plus-navigation.service \
  /etc/systemd/system/grasp-service.service
```

The portable helper tests use fake `ip` and `fuser` commands, so they are safe
to run without a robot or serial device:

```bash
bash deploy/systemd/test_helpers.sh
```

Switch to navigation mode (the ROS master must already be running):

```bash
sudo systemctl start x3plus-navigation.service
```

Start the TG30 with the robot-specific measured transform, rather than the
vendor `TG_test.launch` (whose height and yaw do not match this installation):

```bash
source /opt/ros/melodic/setup.bash
source ~/ydlidar_ws/devel/setup.bash
roslaunch /path/to/repo/deploy/ros/x3plus_tg30_navigation.launch
```

The transform was measured on the real robot on 2026-09-27.  The laser centre
is approximately 0.10 m forward of the chassis centre and its scan plane is
0.095 m above the floor.  A board physically in front appeared at raw scan
angle 180 degrees, so the launch applies a pi-radian yaw.  Verify the live tree
before mapping or AMCL:

```bash
rosrun tf tf_echo base_footprint laser
rostopic hz /scan
```

The expected transform is translation `[0.100, 0.000, 0.095]`, yaw `180 deg`,
with `/scan` close to 10 Hz.

Switch back to the resident grasp mode:

```bash
sudo systemctl start grasp-service.service
sudo systemctl start grasp-vision.service
```

Select exactly one boot owner. The deployed default remains grasp mode:

```bash
# Grasp at boot (current default)
sudo systemctl disable x3plus-navigation.service
sudo systemctl enable grasp-service.service grasp-vision.service

# Navigation at boot (only after roscore is also made persistent)
sudo systemctl disable grasp-service.service grasp-vision.service
sudo systemctl enable x3plus-navigation.service
```

Verify that there is exactly one owner:

```bash
sudo fuser -v /dev/myserial
systemctl --no-pager --full status \
  x3plus-navigation.service grasp-service.service grasp-vision.service
```

`check-serial-owner.sh` exits `0` only for a confirmed free device, `3` when an
owner exists, and `2` when the check itself cannot be trusted. A missing
`fuser`, permission failure, or unexpected tool error therefore blocks startup
instead of being mistaken for "no owner". The unit runs this check as root via
systemd's `+` command prefix so it can see processes owned by other users.

After exercising both switch directions, verify the stop completed before the
next start. `systemctl start` waits for the transaction, so no extra `sleep` is
required:

```bash
journalctl -b -o short-monotonic \
  -u x3plus-navigation.service -u grasp-service.service
systemctl show -p Before -p After -p Conflicts \
  x3plus-navigation.service grasp-service.service
```
