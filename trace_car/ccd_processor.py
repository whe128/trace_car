#!/usr/bin/env python3
# coding: utf-8

from cv_bridge import CvBridge
import cv2
import numpy as np
from trace_car.msg import Perception
# adjustable parameters
MIN_LIGHT_WIDTH_RATIO = 0.08
BLACK_THRESHOLD = 15
WHITE_THRESHOLD = 215
STOP_LIGHT_THRESHOLD = 50
STRAIGHT_LIGHT_THRESHOLD = 16
STOP_LINE_WIDTH_RATIO = 0.35
EDGE_LIGHT_THRESHOLD_RATIO = 0.25

# fixed parameters
SMOOTH_W = 3
DIFF_W = 3
CENTER_HIS_LEN = 3
MEDIAN_FILTER_W = 5


STRAIGHT_COUNT_TRIGGER = 8
TURN_COUNT_TRIGGER = 4
CROSS_COUNT_TRIGGER = 4
STOP_COUNT_TRIGGER = 2
BALCK_COUNT_TRIGGER = 4


FIND_EDGE_FROM_CENTER = False

CCD_LEN = 128
CCD_CENTER = int((CCD_LEN - 1) // 2)

class CCDProcessor():
    def __init__(self, node):
        self.node = node
        node.declare_parameter('MIN_LIGHT_WIDTH_RATIO', MIN_LIGHT_WIDTH_RATIO)
        node.declare_parameter('BLACK_THRESHOLD', BLACK_THRESHOLD)
        node.declare_parameter('WHITE_THRESHOLD', WHITE_THRESHOLD)
        node.declare_parameter('STOP_LIGHT_THRESHOLD', STOP_LIGHT_THRESHOLD)
        node.declare_parameter('STRAIGHT_LIGHT_THRESHOLD', STRAIGHT_LIGHT_THRESHOLD)
        node.declare_parameter('STOP_LINE_WIDTH_RATIO', STOP_LINE_WIDTH_RATIO)
        node.declare_parameter('EDGE_LIGHT_THRESHOLD_RATIO', EDGE_LIGHT_THRESHOLD_RATIO)


        # close CCD recognition parameters
        self.center_close_his = np.full(CENTER_HIS_LEN, CCD_CENTER, dtype=np.uint8)
        self.center_mean_close = CCD_CENTER
        self.left_edge_close = -1
        self.right_edge_close = -1
        self.trace_width_close = -1
        self.avg_brightness_close = 0

        # far CCD recognition parameters
        self.center_far_his = np.full(CENTER_HIS_LEN, CCD_CENTER, dtype=np.uint8)
        self.center_mean_far = CCD_CENTER
        self.left_edge_far = -1
        self.right_edge_far = -1
        self.trace_width_far = -1
        self.avg_brightness_far = 0

        # perception result
        self.road_type = None

        self.straight_count = 0
        self.turn_count = 0
        self.stop_count = 0
        self.cross_count = 0
        self.black_count = 0

    def on_reset(self):
        self.center_close_his = np.full(CENTER_HIS_LEN, CCD_CENTER, dtype=np.uint8)
        self.center_mean_close = CCD_CENTER
        self.left_edge_close = -1
        self.right_edge_close = -1
        self.trace_width_close = -1
        self.avg_brightness_close = 0

        self.center_far_his = np.full(CENTER_HIS_LEN, CCD_CENTER, dtype=np.uint8)
        self.center_mean_far = CCD_CENTER
        self.left_edge_far = -1
        self.right_edge_far = -1
        self.trace_width_far = -1
        self.avg_brightness_far = 0

        self.road_type = None

        self.straight_count = 0
        self.turn_count = 0
        self.stop_count = 0
        self.cross_count = 0
        self.black_count = 0

    def median_filter(self, image):
        """
        image: mono8, shape = (1, N) or (N,)
        return: filtered image
        """
        line = image[0] if len(image.shape) > 1 else image
        pad_w = MEDIAN_FILTER_W // 2
        padded = np.pad(line, (pad_w, pad_w), mode='edge')

        out = np.zeros_like(image, dtype=np.uint8)

        for k in range(len(line)):
            window = padded[k:k+5]
            out[0][k] = np.median(window).astype(np.uint8)

        return out

    def analyse_ccd_close(self, image):
        """
        image: mono8, shape = (1, N) or (N,)
        update close CCD recognition parameters
        """
        self.avg_brightness_close, line_check_result = self.detect_line(image, self.center_mean_close, self.trace_width_close, max_change_avoid_enable = True)

        if line_check_result is not None:
            left, right, center, trace_width = line_check_result
            self.left_edge_close = left
            self.right_edge_close = right
            if self.trace_width_close < 0 and self.left_edge_close >= 0 and self.right_edge_close >= 0:
                self.trace_width_close = trace_width
            # update history
            self.center_close_his[1:] = self.center_close_his[:-1]
            self.center_close_his[0] = center
            self.center_mean_close = int(np.mean(self.center_close_his))
        else:
            self.left_edge_close = -1
            self.right_edge_close = -1

        # print("close CCD: left edge = {}, right edge = {}, center = {}, trace width = {}, avg brightness = {}".format(
        #     self.left_edge_close, self.right_edge_close, self.center_close_his[0], self.trace_width_close, self.avg_brightness_close
        # ))

    def analyse_ccd_far(self, image):
        """
        image: mono8, shape = (1, N) or (N,)
        update far CCD recognition parameters
        """
        self.avg_brightness_far, line_check_result = self.detect_line(image, self.center_mean_far, self.trace_width_far, max_change_avoid_enable = False)

        if line_check_result is not None:
            left, right, center, trace_width = line_check_result
            self.left_edge_far = left
            self.right_edge_far = right
            if self.trace_width_far < 0 and self.left_edge_far >= 0 and self.right_edge_far >= 0:
                self.trace_width_far = trace_width
            # update history
            self.center_far_his[1:] = self.center_far_his[:-1]
            self.center_far_his[0] = center
            self.center_mean_far = int(np.mean(self.center_far_his))
        else:
            self.left_edge_far = -1
            self.right_edge_far = -1

        # print("far CCD: left edge = {}, right edge = {}, center = {}, trace width = {}, avg brightness = {}".format(
        #     self.left_edge_far, self.right_edge_far, self.center_far_his[0], self.trace_width_far, self.avg_brightness_far
        # ))

    def detect_line(self, image, avg_center, init_trace_width, max_change_avoid_enable):
        """
        image: mono8, shape = (1, N) or (N,)
        center_mean_far: int, mean of center position history, used for edge detection
        check_trace_width: whether to check trace width, if True, only return result when trace width is reasonable

        output:
            avg_brightness: int
            tuple: line_check_result
                - left_edge: int
                - right_edge: int
                - center: int
                - trace_width: int
        """
        line = image[0] if len(image.shape) > 1 else image
        N = len(line)

        avg_brightness  = int(np.mean(line))
        black_threshold = self.node.get_parameter('BLACK_THRESHOLD').value
        white_threshold = self.node.get_parameter('WHITE_THRESHOLD').value

        if avg_brightness < black_threshold or avg_brightness > white_threshold:
            return avg_brightness, None

        smooth = np.zeros(N, dtype=np.uint8)


        for i in range(N):
            l = max(0, i - SMOOTH_W)
            r = min(N, i + SMOOTH_W + 1)
            smooth[i] = int(np.mean(line[l:r]))

        # right - left diff
        diff = np.zeros(N)
        for i in range(DIFF_W, N - DIFF_W):
            left = np.mean(smooth[i - DIFF_W:i])
            right = np.mean(smooth[i + 1:i + DIFF_W + 1])

            diff[i] = int(right - left)


        # edge detection
        edge_light_threshold_ratio = self.node.get_parameter('EDGE_LIGHT_THRESHOLD_RATIO').value
        edge_thread = (np.max(smooth) - np.min(smooth)) * edge_light_threshold_ratio

        left_edge = -1
        right_edge = -1
        center = avg_center
        trace_width = -1

        if FIND_EDGE_FROM_CENTER:
            # left edge
            for i in range(avg_center, DIFF_W - 1, -1):
                if diff[i] > edge_thread:
                    left_edge = i
                    break
            for i in range(avg_center, N - DIFF_W):
                if diff[i] < -edge_thread:
                    right_edge = i
                    break

            if left_edge >= 0 and right_edge >= 0:
                # both edges detected, calculate center
                center = (left_edge + right_edge) // 2
                trace_width = right_edge - left_edge
            elif left_edge >=0 and right_edge < 0:
                # only left edge detected, estimate right edge
                center = left_edge + init_trace_width // 2 if init_trace_width > 0 else left_edge + 20
                center = min(center, N - 1)
            elif left_edge < 0 and right_edge >= 0:
                # only right edge detected, estimate left edge
                center = right_edge - init_trace_width // 2 if init_trace_width > 0 else right_edge - 20
                center = max(center, 0)
            else:
                # both edges detected, calculate center
                center = avg_center
        else:
            # list[(edge_position, is_up_edge)]
            edges = []

            pre_diff = 0
            start_continuous_edge = -1

            # find all up and down edges
            for i in range(DIFF_W, N - DIFF_W):
                diff_val = diff[i]

                if diff_val > edge_thread:
                    # up edge
                    if start_continuous_edge < 0:
                        # find the start edge, but not start to detect continuous edge
                        start_continuous_edge = i
                    else:
                        # already find the start edge, continue to detect continuous edge
                        if pre_diff <= edge_thread:
                            # continuous eddge is down, but current is up, stop continuous edge detection, record the down edge
                            middle = (start_continuous_edge + i) // 2
                            edges.append((middle, False))
                            start_continuous_edge = i

                        # else continuous edge is up, continue to detect

                elif diff_val < -edge_thread:
                    # down edge
                    if start_continuous_edge < 0:
                        # find the start edge, but not start to detect continuous edge
                        start_continuous_edge = i
                    else:
                        # already find the start edge, continue to detect continuous edge
                        if pre_diff >= -edge_thread:
                            # continuous eddge is up, but current is down, stop continuous edge detection, record the up edge
                            middle = (start_continuous_edge + i) // 2
                            edges.append((middle, True))
                            start_continuous_edge = i

                        # else continuous edge is down, continue to detect

                else:
                    # not edge, stop continuous edge detection
                    if start_continuous_edge >= 0:
                        if pre_diff > edge_thread:
                            # previous edge is up edge, record the continuous up edge
                            middle = (start_continuous_edge + i) // 2
                            edges.append((middle, True))
                            start_continuous_edge = -1
                        elif pre_diff < -edge_thread:
                            # previous edge is down edge, record the continuous down edge
                            middle = (start_continuous_edge + i) // 2
                            edges.append((middle, False))
                            start_continuous_edge = -1

                # update pre_diff
                pre_diff = diff_val

            # finish edge detection, and still have continuous edge, record the edge
            if start_continuous_edge >= 0:
                middle = (start_continuous_edge + (N - 1)) // 2
                if pre_diff > edge_thread:
                    # previous edge is up edge, record the continuous up edge
                    edges.append((middle, True))
                elif pre_diff < -edge_thread:
                    # previous edge is down edge, record the continuous down edge
                    edges.append((middle, False))
            # find all light windows
            light_windows = []
            is_pre_up_edge = None
            up_edge = -1

            for edge_pose, is_up_edge in edges:
                if is_up_edge:
                    # up edge
                    if is_pre_up_edge is None or is_pre_up_edge == False:
                        # no previous edge, or previous is down
                        is_pre_up_edge = True
                        up_edge = edge_pose

                    # else previous is up, ignore

                else:
                    # down edge
                    if is_pre_up_edge is None:
                        # no previous edge, first finded the down edge
                        is_pre_up_edge = False
                        # append the light window with left edge = 0
                        light_windows.append((0, edge_pose))
                    elif is_pre_up_edge == True:
                        # previous edge is up edge, find down edge,
                        # record the light window
                        is_pre_up_edge = False
                        light_windows.append((up_edge, edge_pose))


            # check the end window when reach the end
            if is_pre_up_edge == True:
                # previous edge is up edge,
                # but no down edge until the end, record the light window with right edge = N - 1
                light_windows.append((up_edge, N - 1))


            # find the window that contains avg_center
            temp_left = -1
            temp_right = -1
            min_center_diff = N
            min_light_width_ratio = self.node.get_parameter('MIN_LIGHT_WIDTH_RATIO').value


            min_light_width = int(init_trace_width * min_light_width_ratio) if init_trace_width > 0 else 3

            for left, right in light_windows:
                window_center = (left + right) // 2
                center_diff = abs(window_center - avg_center)
                # if the window is too narrow, and is not boarder window, ignore it
                if right - left < min_light_width and left > 0 and right < N - 1:
                    continue

                # find the closest window center to avg_center
                if center_diff < min_center_diff:
                    temp_left = left
                    temp_right = right
                    min_center_diff = center_diff

            if temp_left >= 0 and temp_right >= 0:
                if temp_left > 0 and temp_right < N - 1:
                    # both edges detected, calculate center
                    left_edge = temp_left
                    right_edge = temp_right
                    center = (left_edge + right_edge) // 2
                    trace_width = right_edge - left_edge
                elif temp_left > 0 and temp_right == N - 1:
                    # only left edge detected, estimate right edge
                    left_edge = temp_left
                    center = left_edge + init_trace_width // 2 if init_trace_width > 0 else left_edge + 20
                elif temp_left == 0 and temp_right < N - 1:
                    # only right edge detected, estimate left edge
                    right_edge = temp_right
                    center = right_edge - init_trace_width // 2 if init_trace_width > 0 else right_edge - 20
                else:
                    # both edges missed, calculate center based on avg_center
                    center = avg_center
            else:
                # no edge detected, return None
                center = avg_center

            # max center change
            if abs(center - avg_center) > CCD_LEN // 3 and self.road_type != Perception.BLACK and max_change_avoid_enable:
                center = avg_center

            center = max(0, min(center, N - 1))
        return avg_brightness, (left_edge, right_edge, center, trace_width)

    def perception_analyse(self):
        """
        analyse perception result based on recognition parameters, and update self.perception_msg

        return: new perception result (road type)
        """
        stop_line_width_ratio = self.node.get_parameter('STOP_LINE_WIDTH_RATIO').value
        stop_light_threshold = self.node.get_parameter('STOP_LIGHT_THRESHOLD').value
        black_threshold = self.node.get_parameter('BLACK_THRESHOLD').value
        white_threshold = self.node.get_parameter('WHITE_THRESHOLD').value
        straight_light_threshold = self.node.get_parameter('STRAIGHT_LIGHT_THRESHOLD').value


        left_edge = self.left_edge_close
        right_edge = self.right_edge_close
        center = self.center_mean_close
        trace_width = self.trace_width_close
        avg_brightness = self.avg_brightness_close

        # road type in condition: previous
        # road type in assignment, new road type

        # see the stop line, the width less than original width



        if right_edge - left_edge < stop_line_width_ratio * trace_width \
            and left_edge >= 0 and right_edge >= 0 and trace_width > 0 \
            and avg_brightness < stop_light_threshold\
            and (
                self.road_type == Perception.STRAIGHT \
                or self.road_type == Perception.STOP \
                or (
                    # if not stable of straight, still consider as stop
                    abs(self.center_mean_close - CCD_CENTER) < straight_light_threshold  \
                    and abs(self.center_mean_far - CCD_CENTER) < straight_light_threshold \
                )):
            self.stop_count += 1
            self.straight_count = 0
            self.turn_count = 0
            self.cross_count = 0
            self.black_count = 0
            if self.stop_count >= STOP_COUNT_TRIGGER:
                self.road_type = Perception.STOP

        elif avg_brightness < black_threshold:
            self.black_count += 1
            self.straight_count = 0
            self.turn_count = 0
            self.cross_count = 0
            self.stop_count = 0
            if self.black_count >= BALCK_COUNT_TRIGGER:
                self.road_type = Perception.BLACK

        elif avg_brightness > white_threshold or (left_edge < 0 and right_edge < 0 and avg_brightness > 0.8 * white_threshold):
            # no line may cross or black, need to avoid black
            self.cross_count += 1
            self.straight_count = 0
            self.turn_count = 0
            self.stop_count = 0
            self.black_count = 0
            if self.cross_count >= CROSS_COUNT_TRIGGER:
                self.road_type = Perception.CROSS


        elif abs(center - CCD_CENTER) < straight_light_threshold and abs(self.center_mean_far - CCD_CENTER) < straight_light_threshold:
            # straight road
            self.straight_count += 1
            self.turn_count = 0
            self.stop_count = 0
            self.cross_count = 0
            self.black_count = 0
            if self.straight_count >= STRAIGHT_COUNT_TRIGGER:
                self.road_type = Perception.STRAIGHT

        else:
            # turn road
            self.turn_count += 1
            self.straight_count = 0
            self.stop_count = 0
            self.cross_count = 0
            self.black_count = 0
            if self.turn_count >= TURN_COUNT_TRIGGER:
                self.road_type = Perception.TURN
