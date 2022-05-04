# -ECE4310-Service-Robot-
Source code for the project Service Robot of cource ECE4310 at CUHK-Shenzhen. I am the only student to complete all the tasks.

# Dependencies
 
 Ubuntu 20.04
 
 ROS noetic: AMCL, move_base, Gmapping, hector mapping, RTAB mapping packages
 
 OpenCV
 
Augment Reality Tag

PCL

To install the ros packages, run:

```
$ sudo apt-get install ros-noetic-ecl-threads ros-noetic-openslam-gmapping ros-noetic-hector-slam ros-noetic-map-server
$ sudo apt-get install ros-noetic-navigation ros-noetic-pointcloud-to-laserscan ros-noetic-rosbridge-server
$ sudo apt-get install ros-noetic-rtabmap ros-noetic-rtabmap-ros
$ sudo apt install ros-noetic-pcl-ros
$ sudo apt install pcl-tools
$ sudo apt install ros-noetic-octamap-msgs
$ sudo apt-get install ros-noetic-vision-opencv libopencv-dev python-opencv
$ sudo apt-get install ros-${ROS_DISTRO}-perception
```

# Usage

1. Download the package and run 
```
cd catkin_ws
catkin_make
source devel/setup.bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:<your-ws-
path>/src/spark1_description/model
```
Remember to indicate your workspace path!

2. The project uses Gazebo to do simulation. To run the simulation environment, run
```
roslaunch spark1_description spark_gazebo_easy.launch
```
3. The robot uses ar tag. To detect the tag, run
```
roslaunch ar_track_alvar ar_track_depth.launch
```
4. The robot uses amcl to do localization and move_base to navigation. I creates a map using RTAB for amcl and move_base. To run these component, tun
```
roslaunch spark1_description spark_map_load.launch
roslaunch spark_navigation amcl.launch
roslaunch spark_navigation move_base.launch
```

Note for this step:

a. In alternative, you can
```
roslaunch spark_navigation move_base_mapless_demo.launch
```
to see the effect without the map.

b. Remember to change the path to the map in your own computer! Pay attention to these files: spark1_description/launch/spark_map_load.launch, spark_rtabmap/scripts/test_map.yaml

5. Finally, to run the robot, run 
```
rosrun spark1_description service_robot
```
