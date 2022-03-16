#!/usr/bin/env python2.7

class DifferentialDrive:

    def __init__(self, wheel_base, wheel_radius):
        
        # Initialize variables
        self.wheel_base = wheel_base 
        self.wheel_radius = wheel_radius

    # Main Function: Convert (v and w) to (vl and vr)
    def inv_kinematics(self, v, w):

        # Initialize variables (m/rad)
        L = self.wheel_base
        R = self.wheel_radius
        
        # Unicycle to Differential Drive
        vr = ((2.0 * v) + (w * L)) / (2.0 * R)
        vl = ((2.0 * v) - (w * L)) / (2.0 * R)

        # Return wheel speeds (m/s)
        return {"vl": vl, "vr": vr}
