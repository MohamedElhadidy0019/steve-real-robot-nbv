#!/bin/bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
--packages-skip spacenav wiimote wiimote_msgs neo_local_planner2 ps3joy neo_sick_s300-2 \
--continue-on-error 2>&1 | tail -30