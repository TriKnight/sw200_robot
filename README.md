# sw200_robot
# install nvidia driver
```
ubuntu-drivers devices
sudo ubuntu-drivers autoinstall
```
## install
```
sudo apt-get install ros-melodic-joint-state-publisher-gui

sudo apt-get install ros-melodic-gazebo-ros-control

sudo apt-get install ros-melodic-interactive-marker-twist-server

sudo apt-get install ros-melodic-twist-mux

sudo apt-get install ros-melodic-realsense2-*

sudo apt-get install ros-melodic-serial
 
sudo apt-get install libsvm-dev
 
git clone -b melodic https://github.com/spencer-project/spencer_people_tracking
 
git clone -b melodic-devel https://github.com/mdrwiega/depth_nav_tools
 
sudo apt-get install libsdl-image1.2-dev
 
sudo apt-get install libsdl-dev

sudo apt-get install ros-melodic-robot-nav-rviz-plugins

//Realsense
sudo apt-get install librealsense2-dbg

sudo apt-get install librealsense2-dev 

sudo apt-get install librealsense2-utils

// Navigation
git clone -b melodic-devel https://github.com/ros-planning/navigation_experimental

git clone -b melodic-devel https://github.com/ros-planning/navigation

sudo apt-get install ros-melodic-sbpl*

git clone -b noetic-devel https://github.com/rst-tu-dortmund/teb_local_planner.git

git clone https://github.com/rst-tu-dortmund/costmap_converter.git

git clone https://github.com/magazino/move_base_flex


 ```
## Install PIP
```
sudo apt-get install python-pip
pip install vpython
```
## Install dependences
Update source. This step is necessary for either binary or source install.
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update -qq
```
For a source install
```
# use rosdep to install all dependencies (including ROS itself)
sudo apt-get update -qq
sudo apt-get install -qq -y python-rosdep
sudo rosdep init
rosdep update
rosdep install --from-paths ./ -i -y --rosdistro melodic

# build all packages in the catkin workspace
source /opt/ros/melodic/setup.bash
catkin_init_workspace
cd ~/catkin_ws
catkin_make -DCMAKE_BUILD_TYPE=RelWithDebugInfo
```
## Open CV using

Check open CV version, we use open CV 3 version
```
dpkg -l | grep libopencv
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

Gazebo demo (mapping)
---------------------

```bash
### gazebo:
roslaunch sw200_gazebo mir_maze_world.launch
rosservice call /gazebo/unpause_physics   # or click the "start" button in the Gazebo GUI

### mapping:
roslaunch sw200_navigation hector_mapping.launch

# navigation:
roslaunch sw200_navigation move_base.xml with_virtual_walls:=false
rviz -d $(rospack find sw200_navigation)/rviz/navigation.rviz
```

Auto docking
--------------------------
### Lidar Based Auto Docking

General autodocking package for ROS based robots. hehe. :)
Note: your dock must have the same cross sectional dimensions as the fetch charging dock.stl file.  Front face is 300mm long, Each side is 100mm long, at 45 degree angle

### Setup/Installation
 * Firstly, clone this package into your catkin/src folder using git clone. 
 * Next, use catkin_make to compile the package.
 * Once the package has been compiled, open auto_dock.launch(which is located in the launch folder) and modify the remap statements as neccessary. 
 * **to="scan" (scan could be replaced by the name of your laserscan topic ie. laser_scan)**
 * **to="cmd_vel" (cmd_vel could be replaced by a custom name for sending velocity commands ie. dock_cmd_vel)**
 * Next, in the params folder, open *docking_parameters.yaml* and tune the parameters as neccessary. Detailed descriptions of the    parameters can be found in the same document.
 * The docking program subscribes to the topic **battery_voltage** of type **std_msgs/Float32** to obtain the current batteries    charge state in (Volts). Do ensure you have a source to publish to this topic.
 
 * To run the autodock program, type **roslaunch fetch_open_auto_dock auto_dock.launch**
 
### General Testing
* To test the docking program without having to publish to **battery_voltage** and set the far goal parameters, drive/place the robot about 1 metre away from the dock with the robot pointing towards it and run the following lines:

```
roslaunch fetch_open_auto_dock auto_dock.launch
```

 * Followed by 
```
rosrun fetch_open_auto_dock dock_on_button.py
```
 
* To test the undocking program by itself **(warning: robot will reverse and spin 180 degrees)** run the following lines:

```
roslaunch fetch_open_auto_dock auto_dock.launch
 
rosrun fetch_open_auto_dock undock_on_button.py
```

# General Notes
* It is recommended to place the near goal about 1-1.5 metres away from the dock and have the robot point towards the dock. 
* Obstacle avoidance is not implemented here. Ensure dock area is clear before docking robot. 



