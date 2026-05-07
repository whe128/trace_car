#!/usr/bin/env python3
#conding: utf-8

import rclpy
from rclpy.node import Node
from trace_car.msg import Perception
import math
from std_msgs.msg import Float64MultiArray
import numpy as np

# fixed parameters
WHEEL_DIAMETER_MM = 64     # mm
WHEEL_BASE_MM = 225.75     # mm, the distance between two rear wheels
CAR_WIDTH_MM = 153         # mm, the distance between two front wheels
CAR_LENGTH_MM = 280        # mm, the distance between front and rear axle
MM_TO_RAD_RATIO = 2 / WHEEL_DIAMETER_MM

CCD_LEN = 128
CCD_CENTER = int((CCD_LEN - 1) // 2)

CENTER_ERROR_HIS_LEN = 15          # the length of the history error list for steer control, can be used for more complex control method like sliding window average or even learning based method
LOCK_CROSS_ERROR_AVG_LEN = 3      # the length of the history error list to calculate the average center error for cross when lock, can be used to improve the guess of the center when lock the cross
CROSS_COUNT_TRIGGER = 4           # the count trigger to consider the cross is really detected, can be used to avoid the noise of perception in cross detection
CLOSE_SEE_DISTANCE_MM = 400  # mm
CROSS_DISTANCE_MM = 550      # mm

MAX_STEER = 35            # degree
MAX_SPEED = 3000          # mm/s, the max speed of the car
MAX_STEER_RAD = math.radians(MAX_STEER)
MAX_SPEED_RAD = MAX_SPEED * MM_TO_RAD_RATIO
STEER_DT = 0.01 # from the ccd node timer, 10 ms
SPEED_DT = 0.01 # from the controller node timer, 10 ms

# adjustable parameters
STOP_DISTANCE_MM = 250.0                  # mm, the distance that the car need to go to consider fully stop, can be used to make the stop lock more stable and avoid the noise of perception in stop detection
MAX_STEER_RATE_DEGREE = 360.0       # degree/s, the max steer change rate, for steer dynamics
MAX_SPEED_UP_RATE_MM = 1000.0         # mm/s^2, the max speed change rate, for speed dynamics
MAX_SPEED_DOWN_RATE_MM = 3500.0       # mm/s^2, the max speed change rate when decelerate, for speed dynamics, can be larger than up rate for more safety
CROSS_LOCK_ERROR_DECAY_EXP = 1.5
STEER_KP = 0.0074
STEER_KD = 0.005
STRAIGHT_SPEED_MM = 2500.0
TURN_SPEED_MM = 1600.0
CROSS_SPEED_MM = 1600.0


# control start after several count to init the system
STRAT_COUNT_TRIGGER = 20 # in timer, 20 * 10 ms = 0.2s

class ControllerCore():

    def __init__(self, node):
        self.node = node
        node.declare_parameter('MAX_STEER_RATE_DEGREE', MAX_STEER_RATE_DEGREE)

        node.declare_parameter('MAX_SPEED_UP_RATE_MM', MAX_SPEED_UP_RATE_MM)
        node.declare_parameter('MAX_SPEED_DOWN_RATE_MM', MAX_SPEED_DOWN_RATE_MM)

        node.declare_parameter('CROSS_LOCK_ERROR_DECAY_EXP', CROSS_LOCK_ERROR_DECAY_EXP)
        node.declare_parameter('STEER_KP', STEER_KP)
        node.declare_parameter('STEER_KD', STEER_KD)
        node.declare_parameter('STRAIGHT_SPEED_MM', STRAIGHT_SPEED_MM)
        node.declare_parameter('TURN_SPEED_MM', TURN_SPEED_MM)
        node.declare_parameter('CROSS_SPEED_MM', CROSS_SPEED_MM)
        node.declare_parameter('STOP_DISTANCE_MM', STOP_DISTANCE_MM)

        # control the start of the system
        self.has_started = False

        self.center_error_history = np.zeros(CENTER_ERROR_HIS_LEN)

        self.locked_cross_center_error = 0

        # rad
        self.cross_start_odom_rad = -1
        self.stop_odom_record_enable = False
        self.stop_start_odom_rad = -1
        self.cross_odom_increment_rad = 0
        self.stop_odom_increment_rad = 0

        self.received_road_type = None
        self.locked_road_type = None
        self.cross_guess_center_error = 0

        self.black_locked_error = None

        self.start_count = 0


        # wheel control rad/s
        self.output_speed = 0   # send to simulator
        self.output_steer = 0.0

        self.steer_left = 0.0
        self.steer_right = 0.0
        self.speed_left = 0.0
        self.speed_right = 0.0


        self.target_speed = 0.0
        self.target_steer = 0.0

    def on_reset(self):
        self.has_started = False

        self.center_error_history = np.zeros(CENTER_ERROR_HIS_LEN)

        self.locked_cross_center_error = 0

        self.cross_start_odom_rad = -1
        self.stop_odom_record_enable = False
        self.stop_start_odom_rad = -1
        self.cross_odom_increment_rad = 0
        self.stop_odom_increment_rad = 0

        self.received_road_type = None
        self.locked_road_type = None
        self.cross_guess_center_error = 0

        self.black_locked_error = None

        self.start_count = 0

        # wheel control rad/s
        self.output_speed = 0.0   # send to simulator
        self.output_steer = 0.0

        self.steer_left = 0.0
        self.steer_right = 0.0
        self.speed_left = 0.0
        self.speed_right = 0.0


        self.target_speed = 0.0
        self.target_steer = 0.0

    def ccd_steer_pid(self, error, error_prev):
        ''' pd control for steering, output the steer angle in rad
            error: ccd center error in pixel
            error_prev: previous error for derivative calculation

            return: output steer angle in rad
        '''

        steer_kp = self.node.get_parameter('STEER_KP').value
        steer_kd = self.node.get_parameter('STEER_KD').value

        # P
        p = steer_kp * error

        # D
        d = steer_kd * (error - error_prev)

        # output
        steer = p + d

        return steer

    def apply_steer_dynamics(self):
        max_steer_rate_degree = self.node.get_parameter('MAX_STEER_RATE_DEGREE').value

        # for simulator, the steer can directly reach the target
        max_steer_change = math.radians(max_steer_rate_degree) * STEER_DT

        delta = self.target_steer - self.output_steer

        delta = max(min(delta, max_steer_change), -max_steer_change)

        output_steer = self.output_steer + delta
        output_steer = max(-MAX_STEER_RAD, min(output_steer, MAX_STEER_RAD))

        self.output_steer = output_steer


    def apply_speed_dynamics(self):
        max_speed_up_rate_mm = self.node.get_parameter('MAX_SPEED_UP_RATE_MM').value
        max_speed_down_rate_mm = self.node.get_parameter('MAX_SPEED_DOWN_RATE_MM').value
        # for simulator, the speed can directly reach the target
        max_speed_up_change_rad   = max_speed_up_rate_mm   * MM_TO_RAD_RATIO * SPEED_DT
        max_speed_down_change_rad = max_speed_down_rate_mm * MM_TO_RAD_RATIO * SPEED_DT

        delta = self.target_speed - self.output_speed
        if delta > 0:
            # accelerate, use the min to limit the speed change
            delta = min(delta, max_speed_up_change_rad)
        else:
            # decelerate, use the max to limit the speed change
            delta = max(delta, -max_speed_down_change_rad)

        self.output_speed = max(0.0, min(self.output_speed + delta, MAX_SPEED_RAD))



    def ackman_steer_calculate(self):
        steer = self.output_steer

        if abs(steer) < 0.005:
            # if steer is very small, consider it as straight to avoid the noise of steer when straight
            self.steer_left = 0.0
            self.steer_right = 0.0
            return

        turn_radius  = WHEEL_BASE_MM / math.tan(steer)
        kappa = 1 / turn_radius
        if abs(kappa) < 1e-6:
            # if kappa is very small, consider it as straight to avoid the noise of steer when straight
            self.steer_left = 0.0
            self.steer_right = 0.0
            return

        self.steer_left = math.atan(WHEEL_BASE_MM / (turn_radius - 0.5 * CAR_WIDTH_MM))
        self.steer_right = math.atan(WHEEL_BASE_MM / (turn_radius + 0.5 * CAR_WIDTH_MM))


    def ackman_speed_calculate(self):
        speed = self.output_speed

        kappa  = math.tan(self.target_steer) / WHEEL_BASE_MM

        self.speed_left = speed * (1 - 0.5 * CAR_WIDTH_MM * kappa)
        self.speed_right = speed * (1 + 0.5 * CAR_WIDTH_MM * kappa)


    def start_step(self):
        # init state, wait for a few frames to start
        if self.start_count < STRAT_COUNT_TRIGGER:
            self.has_started = False
            self.start_count += 1
        else:
            self.has_started = True

    def get_road_type(self):
        if self.locked_road_type is not None:
            return self.locked_road_type
        else:
            return self.received_road_type

    def steer_control_logic(self, received_road_type, received_center_error):
        cross_lock_error_decay_exp = self.node.get_parameter('CROSS_LOCK_ERROR_DECAY_EXP').value
        straight_speed_mm = self.node.get_parameter('STRAIGHT_SPEED_MM').value
        turn_speed_mm = self.node.get_parameter('TURN_SPEED_MM').value
        cross_speed_mm = self.node.get_parameter('CROSS_SPEED_MM').value

        # 100 hz run the steer control logic based on the perception result
        self.received_road_type = received_road_type
        # update the error for pd control
        self.center_error_history[1:] = self.center_error_history[:-1]
        self.center_error_history[0] = received_center_error

        # use my guess center error for cross when locked
        if self.locked_road_type == Perception.CROSS:
            cross_guess_center_error_pre = self.cross_guess_center_error

            ratio = self.cross_odom_increment_rad / (CROSS_DISTANCE_MM * MM_TO_RAD_RATIO)
            ratio = min(max(ratio, 0.0), 1.0)

            self.cross_guess_center_error = self.locked_cross_center_error * (1 - ratio) ** cross_lock_error_decay_exp

            steer_by_error = self.ccd_steer_pid(self.cross_guess_center_error, cross_guess_center_error_pre)
        elif received_road_type == Perception.BLACK:
            if self.black_locked_error is None:
                # need to lock at biggest steer
                if np.mean(self.center_error_history) > 0:
                    self.black_locked_error = int( CCD_LEN // 2 )
                else:
                    self.black_locked_error = int(-( CCD_LEN // 2 ))

            steer_by_error = self.ccd_steer_pid(self.black_locked_error, self.black_locked_error)

        else:
            steer_by_error = self.ccd_steer_pid(self.center_error_history[0], self.center_error_history[1])


        self.target_steer = max(-MAX_STEER_RAD, min(steer_by_error, MAX_STEER_RAD))

        if self.locked_road_type is None:
            # no lock and consider the road type for control
            if received_road_type == Perception.STRAIGHT:
                self.target_speed = straight_speed_mm * MM_TO_RAD_RATIO

            elif received_road_type == Perception.TURN:
                self.target_speed = turn_speed_mm * MM_TO_RAD_RATIO

            elif received_road_type == Perception.CROSS:
                # lock can only unlock after odometer
                # unlock in wheel control logic after odometer record enough
                self.locked_road_type = Perception.CROSS

                # use the center error history to guess the center error
                # current center error must be in cross, not reliable
                # first several error in history should not consider

                start_index = 1 + CROSS_COUNT_TRIGGER
                end_index = min(start_index + LOCK_CROSS_ERROR_AVG_LEN, CENTER_ERROR_HIS_LEN)

                # make average nearby
                self.locked_cross_center_error = np.mean(self.center_error_history[start_index: end_index])


            elif received_road_type == Perception.STOP:
                # just set the flag to record the odometer
                # will lock stop after odometer record enough
                # lock step is in wheel control logic
                self.stop_odom_record_enable = True

            elif received_road_type == Perception.BLACK:
                self.target_speed = turn_speed_mm * MM_TO_RAD_RATIO
            else:
                self.target_steer = 0.0
                self.target_speed = 0.0
        else:
            # check the lock
            if self.locked_road_type == Perception.CROSS:
                # pow n reduce throught the odometer increment
                self.target_steer = steer_by_error
                self.target_speed = cross_speed_mm * MM_TO_RAD_RATIO

            elif self.locked_road_type == Perception.STOP:
                self.target_speed = 0.0

        self.apply_steer_dynamics()

        self.ackman_steer_calculate()

        return self.steer_left, self.steer_right

    def wheel_control_logic(self, pos_L, pos_R):
        stop_distance_mm = self.node.get_parameter('STOP_DISTANCE_MM').value
        # 100 hz run the speed control logic based on the current speed
        avg_pos = 0.5 * (pos_L + pos_R)



        if self.stop_odom_record_enable:
            # record the stop odometer
            if self.stop_start_odom_rad < 0:
                # record the start
                self.stop_start_odom_rad = avg_pos
            else:
                # already have start, record the increment
                self.stop_odom_increment_rad = avg_pos - self.stop_start_odom_rad
                stop_distance_mm = self.node.get_parameter('STOP_DISTANCE_MM').value
                # need go through certain distance (see distance + car length to make sure the car has fully stop) to lock the stop
                # stop odometer distance
                if self.stop_odom_increment_rad >=  stop_distance_mm * MM_TO_RAD_RATIO :
                    # lock the stop after record enough odometer
                    self.locked_road_type = Perception.STOP
                    self.stop_odom_record_enable = False
                    self.stop_start_odom_rad = -1
                    self.stop_odom_increment_rad = 0
        elif self.locked_road_type == Perception.CROSS:
            # record the cross odometer
            if self.cross_start_odom_rad < 0:
                # record the start
                self.cross_start_odom_rad = avg_pos
                self.cross_odom_increment_rad = 0
            else:
                # already have start, record the increment
                self.cross_odom_increment_rad = avg_pos - self.cross_start_odom_rad

                # need go through certain distance to unlock the cross
                if self.cross_odom_increment_rad >= CROSS_DISTANCE_MM * MM_TO_RAD_RATIO:
                    # unlock the cross after record enough odometer
                    self.locked_road_type = None
                    self.cross_start_odom_rad = -1
                    self.cross_odom_increment_rad = 0

        if self.locked_road_type == Perception.STOP:
            self.output_speed = 0.0
            self.speed_left = 0.0
            self.speed_right = 0.0

        else:
            # speed control, store the current output for incremental pid
            self.apply_speed_dynamics()
            self.ackman_speed_calculate()
        return self.speed_left, self.speed_right
