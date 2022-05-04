#ubuntu16.04&ubuntu18.04

1.将spark1_description文件夹放在catkin_ws工作空间的src文件夹下

2.进行编译

3.source devel/setup.bash

4.roslaunch spark1_description gazebe_spark.launch  打开gazebo仿真和rviz

5.rviz中设置marobot1/base_link ,添加机器人模型插件，在TFprefix中填入marobot1，即可在Rivz显示
