#!/usr/bin/env python3

"""
2D Controller Class to be used for the CARLA waypoint follower demo.
"""

import cutils
import numpy as np

class Controller2D(object):
    def __init__(self, waypoints):
        self.vars                = cutils.CUtils()
        self._current_x          = 0
        self._current_y          = 0
        self._current_yaw        = 0
        self._current_speed      = 0
        self._desired_speed      = 0
        self._current_frame      = 0
        self._current_timestamp  = 0
        self._start_control_loop = False
        self._set_throttle       = 0
        self._set_brake          = 0
        self._set_steer          = 0
        self._waypoints          = waypoints
        self._conv_rad_to_steer  = 180.0 / 70.0 / np.pi
        self._pi                 = np.pi
        self._2pi                = 2.0 * np.pi

    def update_values(self, x, y, yaw, speed, timestamp, frame):
        self._current_x         = x
        self._current_y         = y
        self._current_yaw       = yaw
        self._current_speed     = speed
        self._current_timestamp = timestamp
        self._current_frame     = frame
        if self._current_frame:
            self._start_control_loop = True

    def update_desired_speed(self):
        min_idx       = 0
        min_dist      = float("inf")
        desired_speed = 0
        for i in range(len(self._waypoints)):
            dist = np.linalg.norm(np.array([
                    self._waypoints[i][0] - self._current_x,
                    self._waypoints[i][1] - self._current_y]))
            if dist < min_dist:
                min_dist = dist
                min_idx = i
        if min_idx < len(self._waypoints)-1:
            desired_speed = self._waypoints[min_idx][2]
        else:
            desired_speed = self._waypoints[-1][2]
        self._desired_speed = desired_speed

    def update_waypoints(self, new_waypoints):
        self._waypoints = new_waypoints

    def get_commands(self):
        return self._set_throttle, self._set_steer, self._set_brake

    def set_throttle(self, input_throttle):
        # Clamp the throttle command to valid bounds
        throttle           = np.fmax(np.fmin(input_throttle, 1.0), 0.0)
        self._set_throttle = throttle

    def set_steer(self, input_steer_in_rad):
        # Covnert radians to [-1, 1]
        input_steer = self._conv_rad_to_steer * input_steer_in_rad

        # Clamp the steering command to valid bounds
        steer           = np.fmax(np.fmin(input_steer, 1.0), -1.0)
        self._set_steer = steer

    def set_brake(self, input_brake):
        # Clamp the steering command to valid bounds
        brake           = np.fmax(np.fmin(input_brake, 1.0), 0.0)
        self._set_brake = brake

    def update_controls(self):
        ######################################################
        # RETRIEVE SIMULATOR FEEDBACK
        ######################################################
        x               = self._current_x
        y               = self._current_y
        yaw             = self._current_yaw
        v               = self._current_speed
        self.update_desired_speed()
        v_desired       = self._desired_speed
        t               = self._current_timestamp
        waypoints       = self._waypoints
        throttle_output = 0
        steer_output    = 0
        brake_output    = 0

        ######################################################
        ######################################################
        # MODULE 7: DECLARE USAGE VARIABLES HERE
        ######################################################
        ######################################################
        """
            Use 'self.vars.create_var(<variable name>, <default value>)'
            to create a persistent variable (not destroyed at each iteration).
            This means that the value can be stored for use in the next
            iteration of the control loop.

            Example: Creation of 'v_previous', default value to be 0
            self.vars.create_var('v_previous', 0.0)

            Example: Setting 'v_previous' to be 1.0
            self.vars.v_previous = 1.0

            Example: Accessing the value from 'v_previous' to be used
            throttle_output = 0.5 * self.vars.v_previous
        """
        self.vars.create_var('v_previous', 0.0)
        self.vars.create_var('t_previous', 0.0)
        self.vars.create_var('throttle_previous', 0.0)
        self.vars.create_var('int_value', 0.0)
        self.vars.create_var('last_error', 0.0)
        kp = 10
        ki = 1
        kd = .01

        # Skip the first frame to store previous values properly
        if self._start_control_loop:
            """
                Controller iteration code block.

                Controller Feedback Variables:
                    x               : Current X position (meters)
                    y               : Current Y position (meters)
                    yaw             : Current yaw pose (radians)
                    v               : Current forward speed (meters per second)
                    t               : Current time (seconds)
                    v_desired       : Current desired speed (meters per second)
                                      (Computed as the speed to track at the
                                      closest waypoint to the vehicle.)
                    waypoints       : Current waypoints to track
                                      (Includes speed to track at each x,y
                                      location.)
                                      Format: [[x0, y0, v0],
                                               [x1, y1, v1],
                                               ...
                                               [xn, yn, vn]]
                                      Example:
                                          waypoints[2][1]: 
                                          Returns the 3rd waypoint's y position

                                          waypoints[5]:
                                          Returns [x5, y5, v5] (6th waypoint)
                
                Controller Output Variables:
                    throttle_output : Throttle output (0 to 1)
                    steer_output    : Steer output (-1.22 rad to 1.22 rad)
                    brake_output    : Brake output (0 to 1)
            """

            ######################################################
            ######################################################
            # MODULE 7: IMPLEMENTATION OF LONGITUDINAL CONTROLLER HERE
            
            throttle_output = 0
            brake_output    = 0
            
            # Lonngitudinal control has two types of control, high-level and low-level
            # For high level controller we use PID method to maintain and generate desired Accelration:
            # X''(desired Acc) = Kp(Vref - V)+Ki integraation (Vref - v) + Kd d(Vref - v)/dt
            # For Low level controller we Convert desired Acceleration to engine torque then to throttle angle
            # we first translate desired Acc. to torque to avoid making the problem non-linear.
            # T (engine torque) = (Je/(r_eff*GR)) * X'' + Tload
            # we then maps the output torque to engine map to get the desired throttle.
            
            step_time = t - self.vars.t_previous
            delta_V = v_desired - v
            
            
            integral_term = self.vars.int_value + delta_V * step_time
            derivative_term = (delta_V - self.vars.last_error) / step_time
            
            PID = kp * delta_V + ki * integral_term + kd * derivative_term
            # I couldn't find mapping function for conversion
            # but referring to the module again I found out that sometimes we can neglect the Low_Level controller
            # So we can directly set the throttle output to PID output.
            # Also I've noticed (After testing the simulator multiple times that there's no brake
            # And the car slow down by itself when you pull your finger off (Not pressing (W)), 
            # So clearly there's no need for any brake commands.
            
            if PID > 0:
                throttle_output = PID
                brake_output = 0
            else:
                throttle_output = 0
                brake_output = PID
            
            ######################################################
            ######################################################
            
            """
                Implement a longitudinal controller here. Remember that you can
                access the persistent variables declared above here. For
                example, can treat self.vars.v_previous like a "global variable".
            """
            ######################################################
            ######################################################
            # MODULE 7: IMPLEMENTATION OF LATERAL CONTROLLER HERE
            ######################################################
            ######################################################
            """
                Implement a lateral controller here. Remember that you can
                access the persistent variables declared above here. For
                example, can treat self.vars.v_previous like a "global variable".
            """
            
            # Change the steer output with the lateral controller. 
            steer_output    = 0
            
            # we have two methods to perform lateral control
            # 1- pure pursuit, 2- stanley control.
            # here I will use stanley control which was developed by stanford team.
            # It defines an intuitive steering laws to:
            # 1- correct heading error ,2- correct position error, 3- obey max steering angle position.
            # It also depends on the measure depending on the front axle.
            # please note that an important thing in stanley, that if it requires an angle which is
            # greater than steering angle, It will set steering angle to the max.
            
            K_e = .3
            K_s = 10
            
            # heading error
            yaw_original_path = np.arctan2(waypoints[-1][1]-waypoints[0][1], waypoints[-1][0]-waypoints[0][0])
            yaw_diff = yaw_original_path - yaw 
            if yaw_diff > np.pi:
                yaw_diff -= 2 * np.pi
            if yaw_diff < - np.pi:
                yaw_diff += 2 * np.pi
            
            # cross track error
            # First I will get difference between current points and desired path points
            current_xy = np.array([x, y])
            cross_track_error = np.min(np.sum((current_xy - np.array(waypoints)[:, :2])**2, axis=1))
            
            yaw_cross_track = np.arctan2(y-waypoints[0][1], x-waypoints[0][0])
            yaw_diff_path = yaw_original_path - yaw_cross_track

            if yaw_diff_path > np.pi:
                yaw_diff_path -= 2 * np.pi
            if yaw_diff_path < - np.pi:
                yaw_diff_path += 2 * np.pi
            if yaw_diff_path > 0:
                cross_track_error = abs(cross_track_error)
            else:
                cross_track_error = - abs(cross_track_error)
            
            # I used K_s as an improvement for stanley control
            yaw_diff_crosstrack = np.arctan(K_e * cross_track_error / (K_s + v))
            print(cross_track_error, yaw_diff, yaw_diff_crosstrack)
            # Control: = heading error + crosstrack error.
            expected_steering_output = yaw_diff + yaw_diff_crosstrack
            if expected_steering_output > np.pi:
                expected_steering_output -= 2 * np.pi
            if expected_steering_output < - np.pi:
                expected_steering_output += 2 * np.pi
                
            # steering limits => -1.22 to 1.22 (in radians, from left to right)
            expected_steering_output = min(1.22, expected_steering_output)
            expected_steering_output = max(-1.22, expected_steering_output)

            # 4. update
            steer_output = expected_steering_output
            
            
            ######################################################
            # SET CONTROLS OUTPUT
            ######################################################
            self.set_throttle(throttle_output)  # in percent (0 to 1)
            self.set_steer(steer_output)        # in rad (-1.22 to 1.22)
            self.set_brake(brake_output)        # in percent (0 to 1)

        ######################################################
        ######################################################
        # MODULE 7: STORE OLD VALUES HERE (ADD MORE IF NECESSARY)
        ######################################################
        ######################################################
        """
            Use this block to store old values (for example, we can store the
            current x, y, and yaw values here using persistent variables for use
            in the next iteration)
        """
        self.vars.v_previous = v  # Store forward speed to be used in next step
        self.vars.t_previous = t
        self.vars.int_val = integral_term
        self.vars.throttle_previous = throttle_output
