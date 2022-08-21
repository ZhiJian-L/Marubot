# Marubot
Two-wheel differential drive ROS robot with LiDAR and RGBD camera setup.
Microcontroller: Arduino UNO.

## Operation
1. Bringup Marubot
~~~
roslaunch marubot bringup.launch
~~~
2. Perform SLAM (of your choice)
~~~
roslaunch marubot slam.launch slam:=gmapping
roslaunch marubot slam.launch slam:=hector
roslaunch marubot slam.launch slam:=cartographer
roslaunch marubot slam.launch slam:=slamtoolbox
~~~
