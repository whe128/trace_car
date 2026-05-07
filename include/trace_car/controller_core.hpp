#pragma once
// coding: utf-8

#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <cmath>
#include <utility>
#include "trace_car/msg/perception.hpp"

// ---- fixed parameters ----
constexpr double WHEEL_DIAMETER_MM  = 64.0;
constexpr double WHEEL_BASE_MM      = 225.75;
constexpr double CAR_WIDTH_MM       = 153.0;
constexpr double CAR_LENGTH_MM      = 280.0;
constexpr double MM_TO_RAD_RATIO    = 2.0 / WHEEL_DIAMETER_MM;

constexpr int CENTER_ERROR_HIS_LEN  = 15;
constexpr int LOCK_CROSS_ERROR_AVG_LEN = 3;
constexpr int CROSS_COUNT_TRIGGER   = 4;   // renamed to avoid clash with ccd_processor
constexpr int CLOSE_SEE_DISTANCE_MM = 400;
constexpr int CROSS_DISTANCE_MM     = 550;

constexpr int CCD_LEN            = 128;
constexpr int CCD_CENTER         = (CCD_LEN - 1) / 2;

constexpr double MAX_STEER      = 35.0;  // degrees
constexpr double MAX_SPEED      = 4000.0; // mm/s
constexpr double MAX_STEER_RAD  = MAX_STEER * M_PI / 180.0;
constexpr double MAX_SPEED_RAD  = MAX_SPEED * MM_TO_RAD_RATIO;
constexpr double STEER_DT       = 0.01;
constexpr double SPEED_DT       = 0.01;

constexpr int START_COUNT_TRIGGER = 20;    // 20 * 0.01 ms = 0.2s

// ---- adjustable defaults ----
constexpr double CROSS_LOCK_ERROR_DECAY_EXP = 0.5;
constexpr double MAX_STEER_RATE_DEGREE    = 360.0;    // max steering change rate in degree/s^2
constexpr double MAX_SPEED_UP_RATE_MM     = 2000.0;   // max acceleration in mm/s^2
constexpr double MAX_SPEED_DOWN_RATE_MM   = 3500.0;   // max acceleration in mm/s^2
constexpr double STEER_KP                 = 0.0074;
constexpr double STEER_KD                 = 0.005;
constexpr double STRAIGHT_SPEED_MM        = 2500.0;
constexpr double TURN_SPEED_MM            = 1600.0;
constexpr double CROSS_SPEED_MM           = 1600.0;
constexpr double STOP_DISTANCE_MM         = 250.0; // the distance that the car need to go to consider fully stop, can be used to make the stop lock more stable and avoid the noise of perception in stop detection));




class ControllerCore{
public:
    explicit ControllerCore(rclcpp::Node* node);
    void on_reset();
    void start_step();
    int get_road_type();

    std::pair<double, double> steer_control_logic(int road_type, int center_error);
    std::pair<double, double> wheel_control_logic(double pos_L, double pos_R);

    bool    has_started = false;
    double  target_steer = 0.0;
    double  target_speed = 0.0;
    double  steer_L = 0.0;
    double  steer_R = 0.0;
    double  speed_L = 0.0;
    double  speed_R = 0.0;

    double output_steer = 0.0;
    double output_speed = 0.0;

private:
    double ccd_steer_pid(double error, double error_prev);

    void   apply_steer_dynamics();
    void   apply_speed_dynamics();

    void ackman_steer_calculate();
    void ackman_speed_calculate();

    rclcpp::Node* node_;

    int start_count_ = 0;

    int center_error_his_[CENTER_ERROR_HIS_LEN] = {};
    int center_error_his_index_ = 0;

    double locked_cross_center_error_ = 0;
    double cross_guess_center_error_ = 0;

    double cross_start_odom_rad_ = -1;
    double cross_odom_increment_rad_ = 0;

    bool   stop_odom_record_enable_ = false;
    double stop_start_odom_rad_ = -1;

    int    received_road_type_ = -1;
    int    locked_road_type_ = -1;

    bool   black_locked_error_has_value_ = false;
    int    black_locked_error_ = 0;
};
