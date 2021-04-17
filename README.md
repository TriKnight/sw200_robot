# sw200_robot
# install nvidia driver
```
ubuntu-drivers devices
sudo ubuntu-drivers autoinstall
```
## install
```
sudo apt-get install ros-melodic-gazebo-ros-control

sudo apt-get install ros-melodic-interactive-marker-twist-server

sudo apt-get install ros-melodic-twist-mux

sudo apt-get install ros-melodic-realsense2-*

 sudo apt-get install ros-melodic-serial
 ```


Gazebo demo (existing map)
--------------------------

```bash
### gazebo:
roslaunch sw200_gazebo sw200_maze_world.launch
rosservice call /gazebo/unpause_physics   # or click the "start" button in the Gazebo GUI

### localization:
roslaunch sw200_navigation amcl.launch initial_pose_x:=10.0 initial_pose_y:=10.0
# or alternatively: roslaunch sw200_gazebo fake_localization.launch delta_x:=-10.0 delta_y:=-10.0

# navigation:
roslaunch sw200_navigation start_planner.launch \
    map_file:=$(rospack find sw200_gazebo)/maps/maze.yaml \
    virtual_walls_map_file:=$(rospack find sw200_gazebo)/maps/maze_virtual_walls.yaml
rviz -d $(rospack find sw200_navigation)/rviz/navigation.rviz
```
