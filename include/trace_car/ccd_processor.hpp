#pragma once
// coding: utf-8

#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <algorithm>
#include <cmath>
#include <optional>
#include <utility>

#include "trace_car/msg/perception.hpp"

// fixed parameters
constexpr int FRAME_BUFFER_SIZE  = 4;
constexpr int SMOOTH_W           = 3;
constexpr int DIFF_W             = 3;
constexpr int CENTER_HIS_LEN     = 4;
constexpr int MEDIAN_FILTER_W    = 5;
constexpr int CCD_LEN            = 128;
constexpr int CCD_CENTER         = (CCD_LEN - 1) / 2;

constexpr int STRAIGHT_COUNT_TRIGGER = 8;
constexpr int TURN_COUNT_TRIGGER     = 4;
constexpr int CROSS_COUNT_TRIGGER    = 4;
constexpr int STOP_COUNT_TRIGGER     = 2;
constexpr int BLACK_COUNT_TRIGGER    = 4;

constexpr bool FIND_EDGE_FROM_CENTER = false;


// adjustable parameters (defaults)
constexpr int    BLACK_THRESHOLD         = 15;
constexpr int    WHITE_THRESHOLD         = 225;
constexpr int    STOP_LIGHT_THRESHOLD    = 50;
constexpr int    STRAIGHT_CENTER_ERROR_THRESHOLD = 16;

constexpr double MIN_LIGHT_WIDTH_RATIO   = 0.08;
constexpr double STOP_LINE_WIDTH_RATIO   = 0.35;
constexpr double EDGE_LIGHT_THRESHOLD_RATIO = 0.25;


struct LineResult {
    int avg_brightness = 0;
    bool line_detected = false;
    int left_edge = -1;
    int right_edge = -1;
    int center = -1;
    int width = -1;
};

class CCDProcessor{
public:
    // constructor
    explicit CCDProcessor(rclcpp::Node* node);

    void on_reset();
    void median_filter(const uint8_t* raw_img, uint8_t* filtered_img);
    void temporal_median_filter(const uint8_t fram_buffer[FRAME_BUFFER_SIZE][CCD_LEN], std::array<uint8_t, CCD_LEN>& filtered_img);
    void analyse_ccd_close(const uint8_t* img);
    void analyse_ccd_far(const uint8_t* img);
    LineResult detect_line(const uint8_t* img, int avg_center, int init_trace_width, bool max_change_avoid_enable);
    void perception_analyse();

    int road_type = -1;

    uint8_t center_close_his[CENTER_HIS_LEN];
    int center_close_his_index = 0;   // current buff index
    int center_mean_close = CCD_CENTER;
    int left_edge_close = -1;
    int right_edge_close = -1;
    int trace_width_close = -1;
    int avg_brightness_close = 0;

    uint8_t center_far_his[CENTER_HIS_LEN];
    int center_far_his_index = 0;
    int center_mean_far = CCD_CENTER;
    int left_edge_far = -1;
    int right_edge_far = -1;
    int trace_width_far = -1;
    int avg_brightness_far = 0;

private:
    rclcpp::Node* node_;
    int straight_count_ = 0;
    int turn_count_ = 0;
    int stop_count_ = 0;
    int cross_count_ = 0;
    int black_count_ = 0;
};
