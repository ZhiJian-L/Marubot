#!/usr/bin/env python2.7

import math
import rospy
import threading

from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float32, UInt32

from marubot_pid import PID


class Robot:

    def __init__(self):

    ###### Physical Properties Part ###### ###### ######

        # Marubot's physical properties (m)
        self.wheel_base = rospy.get_param("wheel_base", default = 0.112)
        self.wheel_radius = rospy.get_param("wheel_radius", default = 0.035)

        # Encoder disk physical properties
        self.ticks_per_rev = rospy.get_param("ticks_per_rev", default = 40)
        # self.meters_per_tick = ((math.pi * 2.0 * self.wheel_radius) / (float)(self.ticks_per_rev))
        self.meters_per_tick = 0.0055
        
    ###### Velocity and PWM Part ###### ###### ######

        # Marubot's wheel velocity range (m/s)
        self.vmin = rospy.get_param("wheel/vmin", default = 0.197) # wheel speed @ 0.6 / robot speed ~ 0.040
        self.vmax = rospy.get_param("wheel/vmax", default = 0.256) # 0.256 # wheel speed @ 1.0 / robot speed ~ 0.026

        # Marubot's wheel power (PWM) range
        self.pmin = rospy.get_param("wheel/pmin", default = 0.6)
        self.pmax = rospy.get_param("wheel/pmax", default = 1.0)

    ###### Wheel PWM Publisher Part ###### ###### ######

        # Declare variables for Wheel PWM
        self.cmd_WheelL = Float32()
        self.cmd_WheelR = Float32()

        # Initialize Wheel PWM values
        self.cmd_WheelL.data = 0.0
        self.cmd_WheelR.data = 0.0

        # Create publishers for Wheel PWM
        self.pub_cmd_WheelL = rospy.Publisher('/cmd_WheelL', Float32, queue_size=10)
        self.pub_cmd_WheelR = rospy.Publisher('/cmd_WheelR', Float32, queue_size=10)
        
        # Publish Wheel PWM
        self.pub_cmd_WheelL.publish(self.cmd_WheelL)
        self.pub_cmd_WheelR.publish(self.cmd_WheelR)

    ###### Encoder Ticks Subscriber Part ###### ###### ######

        # Create semaphore for Encoder Ticks
        self.lock_TickL = threading.Semaphore()
        self.lock_TickR = threading.Semaphore()

        # Create subscribers for Encoder Ticks
        self.sub_TickL = rospy.Subscriber("/num_TickL", UInt32, self.cb_encoderTick, (True))
        self.sub_TickR = rospy.Subscriber("/num_TickR", UInt32, self.cb_encoderTick, (False))

        # Initialize Encoder Ticks values
        self.num_TickL = None           # "l": Total left wheel ticks
        self.num_TickR = None           # "r": Total right wheel ticks
        self.time_tick = None           # "t": Current time while getting ticks value

    ###### Odometry Part ###### ###### ######

        # Current robot pose
        self.pose2D = Pose2D(0.0, 0.0, 0.0)

    ###### Wheel velocity Part ###### ###### ###### ######

        # Previous tick value
        self.prev_tick = {"l" : None, "r" : None, "t": None}

        # Current wheel velocity
        self.wheel_vel = {"l": 0.0, "r": 0.0}

    ###### PID Part ###### ###### ######

        # Create Instances
        self.PID = {"l": PID(0.8, 0.001, 0), "r": PID(0.8, 0.001, 0)}

        self.error = {"l": 0.0, "r": 0.0}
        self.control_val = {"l": 0.0, "r": 0.0}
        self.tmp_cmd = {"l": 0.0, "r": 0.0}

    ##### PID DEBUGGING PURPOSE (COMMENT AFTER USE) ##### #####

        # Reference Wheel velocity publisher
        self.vll = Float32()
        self.vrr = Float32()
        self.vll.data = 0.0
        self.vrr.data = 0.0
        self.pub_vll = rospy.Publisher('/vll', Float32, queue_size=10)
        self.pub_vrr = rospy.Publisher('/vrr', Float32, queue_size=10)
        self.pub_vll.publish(self.vll)
        self.pub_vrr.publish(self.vrr)

        # Real Wheel velocity publisher
        self.vel_WheelL = Float32()
        self.vel_WheelR = Float32()
        self.vel_WheelL.data = 0.0
        self.vel_WheelR.data = 0.0
        self.pub_vel_WheelL = rospy.Publisher('/vel_WheelL', Float32, queue_size=10)
        self.pub_vel_WheelR = rospy.Publisher('/vel_WheelR', Float32, queue_size=10)
        self.pub_vel_WheelL.publish(self.vel_WheelL)
        self.pub_vel_WheelR.publish(self.vel_WheelR)
    
###### Wheel PWM Part ###### ###### ###### ######

    # Main Function: Set Wheel's Speed
    def set_wheel_speed(self, vr, vl):

        ###### INPUT PART ######

        # Limit wheel speeds to within the range (-vmax to +vmax)
        vl = max(min(vl, self.vmax), self.vmax * -1.0)
        vr = max(min(vr, self.vmax), self.vmax * -1.0)

        self.cmd_WheelL.data = self.v_to_pwm(vl)
        self.cmd_WheelR.data = self.v_to_pwm(vr)

        # ###### PID PART ######

        # Get current wheel velocity (y)
        # self.get_wheelVelocity() # dont use, got problem

        # # Find the error between output and reference in PWM (e = x - y)
        # self.error["l"] = self.v_to_pwm(vl - self.wheel_vel["l"])
        # self.error["r"] = self.v_to_pwm(vr - self.wheel_vel["r"])

        # # Apply PID control (e * C)
        # for wheel in ["l"]:
        #     self.control_val["l"] = self.PID["l"].output(self.error["l"])
        #     self.tmp_cmd["l"] = self.cmd_WheelL.data + self.control_val["l"]

        #     # Stop motor since it wont turn below self.pmin
        #     if abs(self.tmp_cmd["l"]) < self.pmin:
        #         self.tmp_cmd["l"] = 0.0
        
        # for wheel in ["r"]:
        #     self.control_val["r"] = self.PID["r"].output(self.error["r"])
        #     self.tmp_cmd["r"] = self.cmd_WheelR.data + self.control_val["r"]

        #     # Stop motor since it wont turn below self.pmin
        #     if abs(self.tmp_cmd["r"]) < self.pmin:
        #         self.tmp_cmd["r"] = 0.0


        #         if vl > 0.0:
        #             self.tmp_cmd["l"] += 0.1
        #         elif vl < 0.0:
        #             self.tmp_cmd["l"] -= 0.1
        #         elif vl == 0.0:
        #             self.tmp_cmd["l"] = 0.0
        #         elif vr > 0.0:
        #             self.tmp_cmd["r"] += 0.1
        #         elif vr < 0.0:
        #             self.tmp_cmd["r"] -= 0.1
        #         elif vr == 0.0:
        #             self.tmp_cmd["r"] = 0.0

        # # Limit PWM to within the range (-pmax to +pmax)
        # self.cmd_WheelL.data = max(min(self.tmp_cmd["l"],self.pmax), self.pmax * -1.0)
        # self.cmd_WheelR.data = max(min(self.tmp_cmd["r"],self.pmax), self.pmax * -1.0)

        # # Debug Purpose for PID (Comment after use)
        # rospy.loginfo("vel: L " + str(self.wheel_vel["l"]) + " / R :" + str(self.wheel_vel["r"]) )
        # rospy.logwarn("err: L " + str(self.error["l"]) + " / R :" + str(self.error["r"]))
        # rospy.loginfo("con: L " + str(self.control_val["l"]) + " / R :" + str(self.control_val["r"]))
        # rospy.loginfo("pow: L " + str(self.cmd_WheelL) + " / R : " + str(self.cmd_WheelR))
        # rospy.loginfo(" --- ")
        # # Debug Purpose for PID (Comment after use)
        
        # Debug Purpose for PID (Comment after use)
        self.vll.data = vl
        self.vrr.data = vr
        self.pub_vll.publish(self.vll)
        self.pub_vrr.publish(self.vrr)
        # Debug Purpose for PID (Comment after use)

        # Publish the Wheel PWM
        self.pub_cmd_WheelR.publish(self.cmd_WheelR)
        self.pub_cmd_WheelL.publish(self.cmd_WheelL)

    # Convert wheel velocity to PWM
    def v_to_pwm(self, v):

        # Absolute velocity value
        av = abs(v)

        # Interpolation
        v_ratio = ((av)/(self.vmax))
        cmd_wheel = v_ratio * self.pmax

        # Assertion (Debug purpose)
        assert(cmd_wheel <= 1.0)
        assert(cmd_wheel >= 0.0)

        # Take care of wheel direction
        if v < 0:
            cmd_wheel *= -1.0

        return cmd_wheel

###### Wheel Encoder Part ###### ###### ###### ######

    # Main Function: Return the current Number of Ticks when called     
    def get_wheel_ticks(self):

        ticks = {}

        # Get semaphore
        self.lock_TickL.acquire()
        self.lock_TickR.acquire()

        # Get number of ticks
        ticks["l"] = self.num_TickL
        ticks["r"] = self.num_TickR
        ticks["t"] = self.time_tick

        # Release semaphore
        self.lock_TickL.release()
        self.lock_TickR.release()

        return ticks

    # Main Function: Return the current Wheel's Direction when called
    def get_wheel_dir(self):

        wheel_dir = {"l": 1.0, "r": 1.0}

        if self.cmd_WheelL.data < 0.0:
            wheel_dir["l"] = -1.0

        if self.cmd_WheelR.data < 0.0:
            wheel_dir["r"] = -1.0

        return wheel_dir

    # Wheel Encoder Subscriber Callback
    def cb_encoderTick(self, ticks, is_left_wheel):

        if is_left_wheel:
            self.lock_TickL.acquire()
            self.time_Tick = rospy.Time.now()
            self.num_TickL = ticks.data
            self.lock_TickL.release()
        else:
            self.lock_TickR.acquire()
            self.time_tick = rospy.Time.now()
            self.num_TickR = ticks.data
            self.lock_TickR.release()

###### Odometry Part ###### ###### ###### ######

    # Main Function: Return the current pose when called
    def get_pose2D(self):

        return self.pose2D

    # Main Function: Update the current pose
    def set_pose2D(self, pose2D):

        self.pose2D.x = pose2D.x
        self.pose2D.y = pose2D.y
        self.pose2D.theta = pose2D.theta

###### Wheel velocity Part ###### ###### ###### ######

    # Get the wheel velocity
    def get_wheelVelocity(self):

        ticks = self.get_wheel_ticks()
        dir = self.get_wheel_dir()

        if self.prev_tick["r"] != None and self.prev_tick["l"] != None and self.prev_tick["t"] != None:
            tick_duration = ticks["t"] - self.prev_tick["t"]

            if tick_duration.nsecs != 0:
                inv_sec = 1000000000.0 / (float)(tick_duration.nsecs)
                
            for wheel in ["l", "r"]:
                self.wheel_vel[wheel] = (float)(ticks[wheel] - self.prev_tick[wheel]) * inv_sec * self.meters_per_tick * dir[wheel]

        self.prev_tick["r"] = ticks["r"]
        self.prev_tick["l"] = ticks["l"]
        self.prev_tick["t"] = ticks["t"]
        
        ##### Debug purpose for PID (Comment after use)
        self.vel_WheelL.data = self.wheel_vel["l"]
        self.vel_WheelR.data = self.wheel_vel["r"]
        self.pub_vel_WheelL.publish(self.vel_WheelL)
        self.pub_vel_WheelR.publish(self.vel_WheelR)
        ##### Debug purpose for PID (Comment after use)

        return self.wheel_vel

###### Michellaneous Part ###### ###### ###### ######

    # Shutdown Operations
    def shutdown(self):

        # Stop the wheels
        self.cmd_WheelL.data = 0.0
        self.cmd_WheelR.data = 0.0
        self.pub_cmd_WheelL.publish(self.cmd_WheelL)
        self.pub_cmd_WheelR.publish(self.cmd_WheelR)

###### End ###### ###### ###### ######
