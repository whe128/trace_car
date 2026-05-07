#!/usr/bin/env python3
# coding: utf-8

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from ccd_processor import CCDProcessor
from trace_car.msg import Perception, Detection, GuiControl
import argparse

# adjustable parameters
SHOW_IMAGE = False
SHOW_RAW_IMAGE = True

# fixed parameters
INIT_COUNT = 10
SHOW_IMAGE_W = 640
SHOW_IMAGE_H = 150
CCD_LEN = 128
CCD_CENTER = int((CCD_LEN - 1) // 2)
FRAME_BUFFER_SIZE = 4
SHOW_EDGE_W = 3
TIMER_PERIOD = 0.01  # seconds, 100 hz

BLUE = (255, 0, 0)
GREEN = (0, 255, 0)
RED = (0, 0, 255)

class CCDNode(Node):
    def __init__(self, gui=False):
        super().__init__("ccd_node")
        self.gui = gui
        self.show_image_enable = SHOW_IMAGE

        self.bridge = CvBridge()
        self.ccd_processor = CCDProcessor(self)
        self.perception_msg = Perception()

        # timer for publish to controller and show image
        # 100 hz
        self.timer = self.create_timer(TIMER_PERIOD, self.timer_callback)

        # images
        self.close_img_init_count = 0
        self.close_img_raw = None
        self.close_img = None
        self.close_img_buffer = np.zeros((FRAME_BUFFER_SIZE, CCD_LEN), dtype=np.uint8)


        self.far_img_init_count = 0
        self.far_img_raw = None
        self.far_img = None
        self.far_img_buffer = np.zeros((FRAME_BUFFER_SIZE, CCD_LEN), dtype=np.uint8)


        # subscribe
        self.create_subscription(
            Image,
            "/ccd/ccd_close/image_raw",
            self.ccd_close_callback,
            1
        )

        self.create_subscription(
            Image,
            "/ccd/ccd_far/image_raw",
            self.ccd_far_callback,
            1
        )


        # publisher for perception result to controller
        self.perception_pub = self.create_publisher(Perception, "/perception", 10)

        # window
        if self.show_image_enable:
            if SHOW_RAW_IMAGE:
                cv2.namedWindow("CCD_raw")
            cv2.namedWindow("CCD_filtered")

        if self.gui:
            self.system_paused = True
            # if use GUI, publish ccd detection result to gui
            self.ccd_detection_msg = Detection()  # for GUI display
            self.ccd_detection_pub = self.create_publisher(Detection, "/ccd_detection", 10)

            self.create_subscription(
                GuiControl,
                "/gui_control",
                self.gui_control_callback,
                5
            )
            self.get_logger().info("CCD Node started [GUI mode]")

        else:
            self.system_paused = False
            self.get_logger().info("CCD Node started")

    def on_reset(self):
        # clear the buffer in ccd processor
        self.ccd_processor.on_reset()

        # clear image buffers and reset counters
        self.close_img_init_count = 0
        self.close_img_raw = None
        self.close_img = None
        self.close_img_buffer = np.zeros((FRAME_BUFFER_SIZE, CCD_LEN), dtype=np.uint8)

        self.far_img_init_count = 0
        self.far_img_raw = None
        self.far_img = None
        self.far_img_buffer = np.zeros((FRAME_BUFFER_SIZE, CCD_LEN), dtype=np.uint8)

    # =========================
    # display
    # =========================
    def show_image(self):
        if not self.show_image_enable:
            return

        if SHOW_RAW_IMAGE and self.close_img_raw is not None and self.far_img_raw is not None:
            # ======= for raw images =======
            # resize
            close_vis = cv2.resize(self.close_img_raw, (SHOW_IMAGE_W, SHOW_IMAGE_H), interpolation=cv2.INTER_NEAREST)
            far_vis   = cv2.resize(self.far_img_raw,   (SHOW_IMAGE_W, SHOW_IMAGE_H), interpolation=cv2.INTER_NEAREST)

            # to BGR
            close_vis = cv2.cvtColor(close_vis, cv2.COLOR_GRAY2BGR)
            far_vis   = cv2.cvtColor(far_vis,   cv2.COLOR_GRAY2BGR)

            # label
            cv2.putText(close_vis, "ccd_close_raw", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, GREEN, 2)
            cv2.putText(far_vis, "ccd_far_raw", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, GREEN, 2)

            # gap
            h, w = close_vis.shape[:2]
            gap = np.ones((30, w, 3), dtype=np.uint8) * 255
            combined = cv2.vconcat([close_vis, gap, far_vis])

            cv2.imshow("CCD_raw", combined)


        # ======= for filtered images =======
        if self.close_img is None or self.far_img is None:
            return

        # resize
        close_vis = cv2.resize(self.close_img, (SHOW_IMAGE_W, SHOW_IMAGE_H), interpolation=cv2.INTER_NEAREST)
        far_vis   = cv2.resize(self.far_img,   (SHOW_IMAGE_W, SHOW_IMAGE_H), interpolation=cv2.INTER_NEAREST)
        # to BGR
        close_vis = cv2.cvtColor(close_vis, cv2.COLOR_GRAY2BGR)
        far_vis   = cv2.cvtColor(far_vis,   cv2.COLOR_GRAY2BGR)

        # draw detected line on far image
        scale_factor = SHOW_IMAGE_W / CCD_LEN
        # close CCD

        center_x = int(self.ccd_processor.center_close_his[0] * scale_factor)  # scale to visualization
        cv2.line(close_vis, (center_x, 0), (center_x, SHOW_IMAGE_H), RED, SHOW_EDGE_W)  # red for center

        if self.ccd_processor.left_edge_close >= 0:
            left_x = int(self.ccd_processor.left_edge_close * scale_factor)
            cv2.line(close_vis, (left_x, 0), (left_x, SHOW_IMAGE_H), BLUE, SHOW_EDGE_W)      # blue for left edge

        if self.ccd_processor.right_edge_close >= 0:
            right_x = int(self.ccd_processor.right_edge_close * scale_factor)
            cv2.line(close_vis, (right_x, 0), (right_x, SHOW_IMAGE_H), GREEN, SHOW_EDGE_W)    # green for right edge

        # far CCD
        center_x = int(self.ccd_processor.center_far_his[0] * scale_factor)  # scale to visualization
        cv2.line(far_vis, (center_x, 0), (center_x, SHOW_IMAGE_H), RED, SHOW_EDGE_W)  # red for center

        if self.ccd_processor.left_edge_far >= 0:
            left_x = int(self.ccd_processor.left_edge_far * scale_factor)
            cv2.line(far_vis, (left_x, 0), (left_x, SHOW_IMAGE_H), BLUE, SHOW_EDGE_W)      # blue for left edge

        if self.ccd_processor.right_edge_far >= 0:
            right_x = int(self.ccd_processor.right_edge_far * scale_factor)
            cv2.line(far_vis, (right_x, 0), (right_x, SHOW_IMAGE_H), GREEN, SHOW_EDGE_W)    # green for right edge



        # label
        cv2.putText(close_vis, "ccd_close", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, GREEN, 2)
        cv2.putText(far_vis, "ccd_far", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, GREEN, 2)

        # avg brightness
        cv2.putText(close_vis,
            "avg_brightness: {:.1f}".format(self.ccd_processor.avg_brightness_close),
            (380, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            GREEN,
            2)
        cv2.putText(far_vis,
            "avg_brightness: {:.1f}".format(self.ccd_processor.avg_brightness_far),
            (380, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            GREEN,
            2)

        # road width
        if self.ccd_processor.trace_width_close > 0:
            trace_width = self.ccd_processor.trace_width_close
            cv2.putText(close_vis,
                "road_width: {}".format(trace_width),
                (150, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                GREEN,
                2)
        if self.ccd_processor.trace_width_far > 0:
            trace_width = self.ccd_processor.trace_width_far
            cv2.putText(far_vis,
                "road_width: {}".format(trace_width),
                (150, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                GREEN,
                2)
        # gap
        h, w = close_vis.shape[:2]
        gap = np.ones((30, w, 3), dtype=np.uint8) * 255
        combined = cv2.vconcat([close_vis, gap, far_vis])

        cv2.imshow("CCD_filtered", combined)

        cv2.waitKey(1)

    # =========================
    # callbacks
    # =========================
    def ccd_close_callback(self, msg):
        if self.system_paused:
            return

        try:
            self.close_img_raw = self.bridge.imgmsg_to_cv2(msg, 'mono8')
            self.close_img_buffer[1:] = self.close_img_buffer[:-1]
            # median filter for each frame
            self.close_img_buffer[0] = self.ccd_processor.median_filter(self.close_img_raw)

            # after collecting enough frames, do line detection and show results
            self.close_img_init_count += 1
            if self.close_img_init_count < INIT_COUNT:
                return

            # median filter for temporal
            self.close_img = np.median(self.close_img_buffer, axis=0, keepdims=True).astype(np.uint8)
            self.ccd_processor.analyse_ccd_close(self.close_img)
        except Exception as e:
            self.get_logger().error(str(e))

    def ccd_far_callback(self, msg):
        if self.system_paused:
            return

        try:
            self.far_img_raw = self.bridge.imgmsg_to_cv2(msg, 'mono8')
            self.far_img_buffer[1:] = self.far_img_buffer[:-1]
            # median filter for each frame
            self.far_img_buffer[0] = self.ccd_processor.median_filter(self.far_img_raw)

            # after collecting enough frames, do line detection and show results
            self.far_img_init_count += 1
            if self.far_img_init_count < INIT_COUNT:
                return

            # median filter for temporal
            self.far_img = np.median(self.far_img_buffer, axis=0, keepdims=True).astype(np.uint8)
            self.ccd_processor.analyse_ccd_far(self.far_img)
        except Exception as e:
            self.get_logger().error(str(e))

    def timer_callback(self):
        if self.system_paused:
            return

        # each time period, run the perception analysis to update perception result, and publish to controller
        self.ccd_processor.perception_analyse()

        if self.ccd_processor.road_type is None:
            return

        self.perception_msg.road_type = self.ccd_processor.road_type
        self.perception_msg.center_error = CCD_CENTER - self.ccd_processor.center_mean_close
        self.perception_pub.publish(self.perception_msg)
        self.show_image()

        if self.gui and (self.far_img is not None) and (self.close_img is not None):
            # publish ccd detection result to GUI
            self.ccd_detection_msg.far_img = self.far_img.flatten().astype(np.uint8).tolist()
            self.ccd_detection_msg.far_left_edge = self.ccd_processor.left_edge_far
            self.ccd_detection_msg.far_right_edge = self.ccd_processor.right_edge_far
            self.ccd_detection_msg.far_center = int(self.ccd_processor.center_far_his[0])
            self.ccd_detection_msg.far_road_width = self.ccd_processor.trace_width_far
            self.ccd_detection_msg.far_avg_brightness = self.ccd_processor.avg_brightness_far

            self.ccd_detection_msg.close_img = self.close_img.flatten().astype(np.uint8).tolist()
            self.ccd_detection_msg.close_left_edge = self.ccd_processor.left_edge_close
            self.ccd_detection_msg.close_right_edge = self.ccd_processor.right_edge_close
            self.ccd_detection_msg.close_center = int(self.ccd_processor.center_close_his[0])
            self.ccd_detection_msg.close_road_width = self.ccd_processor.trace_width_close
            self.ccd_detection_msg.close_avg_brightness = self.ccd_processor.avg_brightness_close

            self.ccd_detection_pub.publish(self.ccd_detection_msg)

    def gui_control_callback(self, msg):
        # receive control command from GUI
        if msg.command == GuiControl.START:
            self.system_paused = False
        elif msg.command == GuiControl.PAUSE:
            self.system_paused = True
        elif msg.command == GuiControl.RESET:
            self.system_paused = True
            self.on_reset()
        elif msg.command == GuiControl.SHOW_IMAGE:
            self.show_image_enable = True
            self.system_paused = False
        elif msg.command == GuiControl.HIDE_IMAGE:
            cv2.destroyAllWindows()
            self.show_image_enable = False


# =========================
# main
# =========================
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--gui",
                        action="store_true",
                        help="Whether to use GUI control")

    parser.add_argument("--show",
                    action="store_true",
                    help="Whether to show image")
    args, _ = parser.parse_known_args()

    rclpy.init()

    node = CCDNode(gui=args.gui)

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
