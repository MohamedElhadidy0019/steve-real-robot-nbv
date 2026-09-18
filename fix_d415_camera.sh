#!/bin/bash
# Run this after unplugging/replugging the D415 (or whenever the camera node
# logs "No RealSense devices were found!").
#
# Root cause: this devcontainer's /dev is a private tmpfs that does NOT sync
# live USB hotplug events from the host. Every time the D415 is unplugged and
# replugged, the kernel re-enumerates it under NEW major:minor numbers (both
# the raw USB control node under /dev/bus/usb/... and the /dev/videoN nodes),
# but the container keeps stale device nodes pointing at the old numbers.
# This script reads the CURRENT numbers from sysfs and recreates the /dev
# nodes to match, then makes sure the ROS camera node is running so it picks
# the device up on its own retry loop (it retries every ~6s already).

echo "== USB control node (vendor 8086 = Intel) =="
FOUND=0
for d in /sys/bus/usb/devices/*/idVendor; do
  v=$(cat "$d" 2>/dev/null)
  if [ "$v" = "8086" ]; then
    dir=$(dirname "$d")
    bus=$(cat "$dir/busnum" 2>/dev/null)
    dev=$(cat "$dir/devnum" 2>/dev/null)
    majmin=$(cat "$dir/dev" 2>/dev/null)
    [ -z "$majmin" ] && continue
    maj=${majmin%%:*}
    min=${majmin##*:}
    busfmt=$(printf "%03d" "$bus")
    devfmt=$(printf "%03d" "$dev")
    node="/dev/bus/usb/$busfmt/$devfmt"
    if [ ! -e "$node" ]; then
      echo "creating $node (major=$maj minor=$min)"
      sudo mknod "$node" c "$maj" "$min"
      sudo chmod 666 "$node"
    else
      echo "$node already present"
    fi
    FOUND=1
  fi
done
[ "$FOUND" = "0" ] && echo "WARNING: no Intel (8086) USB device found in sysfs - is it plugged in?"

echo "== video4linux nodes =="
for d in /sys/class/video4linux/video*; do
  name=$(cat "$d/name" 2>/dev/null)
  case "$name" in
    *RealSense*)
      majmin=$(cat "$d/dev" 2>/dev/null)
      maj=${majmin%%:*}
      min=${majmin##*:}
      node="/dev/$(basename "$d")"
      if [ ! -e "$node" ]; then
        echo "creating $node (major=$maj minor=$min)"
        sudo mknod "$node" c "$maj" "$min"
        sudo chmod 666 "$node"
      else
        echo "$node already present"
      fi
      ;;
  esac
done

echo "== SDK visibility check =="
timeout 8 rs-enumerate-devices --short || echo "SDK still doesn't see it - check the physical cable/port."

echo "== ROS camera node =="
if pgrep -f "realsense2_camera_node.*d415_calib" >/dev/null 2>&1; then
  echo "camera node already running - it should pick up the device on its own retry loop within ~10s."
else
  echo "no camera node running - launching one."
  source /home/ws/steve_env.sh
  mkdir -p /home/ws/nbv_scratch/logs
  nohup bash -c 'source /home/ws/steve_env.sh; exec ros2 launch realsense2_camera rs_launch.py camera_name:=d415_calib camera_namespace:=d415_calib' \
    > /home/ws/nbv_scratch/logs/d415_camera.log 2>&1 &
  disown
  echo "launched, logs at /home/ws/nbv_scratch/logs/d415_camera.log"
fi

echo "== done =="
