#!/bin/bash
bash clean.sh
source /home/ws/install/setup.bash
ros2 launch neo_mpo_700-2 bringup.launch.py \
  use_mock_arm:=True \
  arm_type:=ur5 \
  disable_scanners:=True \
  d435_enable:=False
