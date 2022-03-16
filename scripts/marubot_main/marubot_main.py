#!/usr/bin/env python2.7

import math
import rospy
import tf2_ros

from geometry_msgs.msg import Pose2D, TransformStamped
from tf.transformations import quaternion_from_euler

from marubot_command import Command
from marubot_diffdrive import DifferentialDrive
from marubot_robot import Robot


class Marubot:

    def __init__(self):

        # Initialize Marubot Node
        rospy.init_node('marubot', anonymous = False)
        self.rate = rospy.Rate(2) #2hz
        
        # Initialize TF Broadcaster
        self.tf = tf2_ros.TransformBroadcaster()
        self.robot_name = rospy.get_name()
        
        # Create Instances
        self.robot = Robot()
        self.command = Command()
        self.diffdrive = DifferentialDrive(self.robot.wheel_base, self.robot.wheel_radius)
        
        # Initialize previous wheel encoder ticks
        self.prev_tick = None
        
        # Shutdown Operations
        rospy.on_shutdown(self.shutdownBase)
        
        # Startup Operations
        rospy.logwarn("Starting up Marubot ... ")
        self.startMarubot()


###### Main Part ###### ###### ###### ######

    # Main Function
    def main(self):

        #
        self.drive_robot()
        
        # 
        self.update_odometry()

        #
        self.publish_pose()


###### Wheel PWM Part ###### ###### ###### ######

    # Run robot according to cmd_vel
    def drive_robot(self):

        # Get cmd_vel data
        cmd_vel = self.command.main()

        # Convert cmd_vel to wheel speeds
        diff_output = self.diffdrive.inv_kinematics(cmd_vel["v"], cmd_vel["w"])

        # Output wheel speeds
        self.robot.set_wheel_speed(diff_output["vr"], diff_output["vl"])

###### Wheel Encoder Part ###### ###### ###### ######

    # Update the pose of the robot
    def update_odometry(self):

        # Get wheel encoder ticks
        ticks = self.robot.get_wheel_ticks()
        
        # Have not seen a wheel encoder message yet so no need to do anything
        if ticks["l"] == None or ticks["r"] == None:
            return
        
        # Robot may not start with encoder count at zero
        if self.prev_tick == None:
            self.prev_tick = {"l": ticks["l"], "r": ticks["r"]}

        # If ticks are the same since last time, then no need to update either
        if ticks["l"] == self.prev_tick["l"] and ticks["r"] == self.prev_tick["r"]:
            return

        # Get current pose from robot
        prev_pose = self.robot.get_pose2D()

        # Compute odometry - kinematics in meters
        L = self.robot.wheel_base;
        meters_per_tick = self.robot.meters_per_tick

        # How far did each wheel move
        wheel_dir = self.robot.get_wheel_dir()
        meters_left = meters_per_tick * (ticks["l"] - self.prev_tick["l"]) * wheel_dir["l"]
        meters_right = meters_per_tick * (ticks["r"] - self.prev_tick["r"]) * wheel_dir["r"]
        meters_center = (meters_right + meters_left) * 0.5

        # Compute new pose
        x_dt = meters_center * math.cos(prev_pose.theta);
        y_dt = meters_center * math.sin(prev_pose.theta);
        theta_dt = (meters_right - meters_left) / L;
            
        new_pose = Pose2D(0.0, 0.0, 0.0)
        new_pose.x = prev_pose.x + x_dt
        new_pose.y = prev_pose.y + y_dt
        theta_tmp = prev_pose.theta + theta_dt
        new_pose.theta = math.atan2( math.sin(theta_tmp), math.cos(theta_tmp) )

        # Update robot with new pose
        self.robot.set_pose2D(new_pose)

        # Update the tick count
        self.prev_tick["r"] = ticks["r"]
        self.prev_tick["l"] = ticks["l"]

    # Get and publish the updated pose
    def publish_pose(self):

        # Broadcast pose as ROS tf
        pose = self.robot.get_pose2D()
        t = TransformStamped()

        t.header.frame_id = "odom"
        t.child_frame_id = self.robot_name

        t.header.stamp = rospy.Time.now()
        t.transform.translation.x = pose.x
        t.transform.translation.y = pose.y
        t.transform.translation.z = 0.0

        q = quaternion_from_euler(0, 0, pose.theta)

        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        
        self.tf.sendTransform(t)

###### Michellaneous Part ###### ###### ###### ######

    # Shutdown Marubot
    def shutdownBase(self):
        
        rospy.logwarn("Shutting down Marubot ... ")
        self.robot.shutdown()

    # Startup Marubot
    def startMarubot(self):

        while not rospy.is_shutdown():
            try: 
                self.main()     
            except:
                pass
            self.rate.sleep()

###### End ###### ###### ###### ######

if __name__ == "__main__":
    Marubot()
