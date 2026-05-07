#!/usr/bin/env python3
#conding: utf-8

import rclpy
from rclpy.node import Node
from trace_car.msg import Perception, CarInfo, GuiControl
import numpy as np
from std_msgs.msg import Float64MultiArray
from controller_core import ControllerCore
from sensor_msgs.msg import JointState
import argparse


TIMER_PERIOD = 0.01 # seconds, 100 hz

class ControllerNode(Node):

    def __init__(self, gui=False):
        super().__init__("controller_node")
        self.gui = gui
        self.ControllerCore = ControllerCore(self)
        self.timer = self.create_timer(TIMER_PERIOD, self.timer_callback)

        # subscribe   20 hz from ccd node publisher
        # receive the perception result
        # call back run the steer control
        self.create_subscription(
            Perception,
            '/perception',
            self.perception_callback,
            10
        )

        # subscribe from the joint states
        # receive the current speed
        # call back for speed control
        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # publisher for control command to car
        self.wheel_pub = self.create_publisher(
            Float64MultiArray,
            '/wheel_velocity_controller/commands',
            10
        )
        self.steer_pub = self.create_publisher(
            Float64MultiArray,
            '/steer_position_controller/commands',
            10
        )

        self.car_info_msg = CarInfo()  # for GUI display
        self.car_info_pub = self.create_publisher(CarInfo, "/car_info", 10)

        # if use GUI, publish ccd detection result to gui
        if self.gui:
            self.system_paused = True
            self.create_subscription(
                GuiControl,
                "/gui_control",
                self.gui_control_callback,
                5
            )

            self.get_logger().info("Controller Node started [GUI mode]")
        else:
            self.system_paused = False
            self.get_logger().info("Controller Node started")

    def on_reset(self):
        self.ControllerCore.on_reset()
        self.send_steer(0.0, 0.0)
        self.send_wheel(0.0, 0.0)


    def send_wheel(self, left: float, right: float):
        msg = Float64MultiArray()
        msg.data = [left, right]
        self.wheel_pub.publish(msg)

    def send_steer(self, left_angle: float, right_angle: float):
        msg = Float64MultiArray()
        msg.data = [left_angle, right_angle]
        self.steer_pub.publish(msg)

    def timer_callback(self):
        if self.system_paused:
            return

        # step to start the system, wait for a few frames to start
        if not self.ControllerCore.has_started:
            self.ControllerCore.start_step()
        else:
            if self.gui and self.ControllerCore.received_road_type is not None:
                # publish ccd detection result to GUI
                self.car_info_msg.target_steer = self.ControllerCore.target_steer
                self.car_info_msg.target_speed = self.ControllerCore.target_speed
                self.car_info_msg.steer_left = self.ControllerCore.steer_left
                self.car_info_msg.steer_right = self.ControllerCore.steer_right
                self.car_info_msg.speed_left = self.ControllerCore.speed_left
                self.car_info_msg.speed_right = self.ControllerCore.speed_right

                self.car_info_msg.output_steer = self.ControllerCore.output_steer
                self.car_info_msg.output_speed = self.ControllerCore.output_speed

                self.car_info_msg.road_type = self.ControllerCore.get_road_type()
                self.car_info_pub.publish(self.car_info_msg)

    def perception_callback(self, msg):
        if self.system_paused:
            return

        # 100 hz from ccd node publisher
        if not self.ControllerCore.has_started:
            self.send_steer(0.0, 0.0)
            return

        # steer control logic
        road_type = msg.road_type
        center_error = msg.center_error

        steer_L, steer_R = self.ControllerCore.steer_control_logic(road_type, center_error)
        self.send_steer(steer_L, steer_R)

    def joint_state_callback(self, msg):
        if self.system_paused:
            return

        # 100 hz from gazebo joint state publisher
        if not self.ControllerCore.has_started:
            self.send_wheel(0.0, 0.0)
            return

        # speed control logic
        joint_map_pos = dict(zip(msg.name, msg.position))

        # can used for how far the car has gone
        pos_wheel_L = joint_map_pos['wheel_L_back_to_body']
        pos_wheel_R = joint_map_pos['wheel_R_back_to_body']

        wheel_L, wheel_R = self.ControllerCore.wheel_control_logic(pos_wheel_L, pos_wheel_R)
        self.send_wheel(wheel_L, wheel_R)

    def gui_control_callback(self, msg):
        # receive control command from GUI
        if msg.command == GuiControl.START:
            self.get_logger().info("[CMD] STARTED")
            self.system_paused = False

        elif msg.command == GuiControl.PAUSE:
            self.get_logger().info("[CMD] PAUSED")
            self.system_paused = True
        elif msg.command == GuiControl.RESET:
            self.get_logger().info("[CMD] RESETTING – car returning to spawn …")
            self.system_paused = True
            self.on_reset()


# =========================
# main
# =========================
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--gui",
                        action="store_true",
                        help="Whether to use GUI control")
    args, _ = parser.parse_known_args()

    rclpy.init()

    node = ControllerNode(args.gui)

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
