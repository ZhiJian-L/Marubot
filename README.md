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

#
## Unicycle Model
- Move with a desired Forward Velocity (V) and Rotational Velocity (W).
- Forward Velocity (V) is average of Left and Right Wheel Velocities (Vl, Vr). Note that Vl and Vr are in rad/s, while V is in m/s so need multiply radius (R). 
~~~
V = R * (Vr + Vl)/2   ---- (1)
~~~
- Rotational Velocity (W) is differences of wheel velocities divided by radius of rotation. So, consider left wheel is stopped while right wheel moves forward, the robot will rotate and make an arc with radius of L.
~~~
W = R * (Vr - Vl)/L   ---- (2)
~~~
- Simultaneous Substitution of (1) and (2):
~~~
Vr = (2V + wL) / 2R
Vl = (2V - wL) / 2R
~~~

#
### References:
- Apply Coursera Control of Mobile Robots with ROS and ROSbots
