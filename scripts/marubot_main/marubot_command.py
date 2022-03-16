#!/usr/bin/env python2.7

import rospy

from geometry_msgs.msg import Twist

class Command:

    def __init__(self):

        # Subscribe to <node>/cmd_vel
        rospy.Subscriber("/cmd_vel", Twist, self.cb_cmdVel)

        # Declare forward velocity,v and angular velocity,w
        self.v = 0
        self.w = 0

    # Main Function
    def main(self):
        
        # Return cmd_vel data
        output = {"v": self.v, "w": self.w}

        return output

    # Cmd_vel Subscriber Callback
    def cb_cmdVel(self, data):
        
        # Store cmd_vel data
        self.v = data.linear.x
        self.w = data.angular.z