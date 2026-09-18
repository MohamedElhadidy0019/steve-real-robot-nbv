#!/bin/bash
source /home/ws/install/setup.bash

RVIZ_CFG=/home/ws/install/neo_mpo_700-2/share/neo_mpo_700-2/configs/rviz/robot_description_rviz.rviz

if [ -f "$RVIZ_CFG" ]; then
  rviz2 -d "$RVIZ_CFG"
else
  rviz2
fi
