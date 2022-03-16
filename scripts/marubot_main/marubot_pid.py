#!/usr/bin/env python2.7

class PID:

    def __init__(self, Kp, Ki, Kd):

        # PID parameters
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        # Accumulated error
        self.sum_error = 0.0

        # Previous error
        self.prev_error = 0.0

    # Reset the PID
    def reset(self):

        # Clear
        self.sum_error = 0.0
        self.prev_error = 0.0

    # Main Function: PID Adjustment Output
    def output(self, error):

        # Accumulated error
        self.sum_error += error

        # PID Formula
        control_val = (self.Kp * error) + (self.Ki * self.sum_error) + (self.Kd * (error - self.prev_error))

        # Store the error value
        self.prev_error = error

        return control_val