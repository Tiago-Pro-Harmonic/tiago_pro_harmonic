# Installing the workspace

This organisation provides a quick procedure to set up a worspace using Tiago-Pro in ROS2 Jazzy and Gazebo Harmonic with tools developped by Gepetto team. 

  1. Clone this repository 

```
git clone https://github.com/Tiago-Pro-Harmonic/tiago_pro_harmonic.git
cd tiago_pro_harmonic
```

  2. Download a lightweight web browser

```
wget https://github.com/goastian/midori-desktop/releases/download/v11.6/midori_11.6-1_amd64.deb
```
  
  3. Download ros2 repositories

```
mkdir ros2_ws/src
cd ros2_ws/src

vcs import < ../dependencies/controls.repos 
vcs import < ../dependencies/tiago_pro_dependencies.repos 

cd -
```

  
  4. Build the docker image

```
docker build --build-arg DOCKER_USER=`id -u` --build-arg DOCKER_GROUP=`id -g` -t hpp:tuto -f Dockerfile .
```

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