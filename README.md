# Installing the workspace

This tutorial provides a quick procedure to set up a worspace using Tiago-Pro in ROS2 Jazzy and Gazebo Harmonic with tools developped by Gepetto team. 

## Prerequisite

  You need a machine running linux with docker installed. See https://docs.docker.com/engine/install/ubuntu for docker installation instructions.

```bash
# basic launch 
ros2 launch tiago_pro_gazebo tiago_pro_gazebo.launch.py \
                is_public_sim:=True \
                world_name:=empty \
                arm_type_left:=no-arm \
                end_effector_right:=no-end-effector \
                end_effector_left:=no-end-effector \
                tuck_arm:=False \
                gazebo_version:=gazebo
```

In another terminal, launch :
```bash
ros2 launch tiago_pro_lfc_bringup switch_to_lfc_controllers.launch.py 
# and then
ros2 launch tiago_pro_lfc_bringup pd_plus_controller.launch.py
# You should see Tiago pro arm moving !
```

License
---------

This software is distributed under the terms of both the MIT license and the Apache License (Version 2.0).

See LICENSE-APACHE and LICENSE-MIT for details.

Contributors
---------------
Clément Pène