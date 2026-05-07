// coding: utf-8
#include <trace_car/controller_core.hpp>

using Perception = trace_car::msg::Perception;


// ---------------------------------------------------------------------------
ControllerCore::ControllerCore(rclcpp::Node* node){
    node_ = node;

    node_->declare_parameter("CROSS_LOCK_ERROR_DECAY_EXP",  CROSS_LOCK_ERROR_DECAY_EXP);
    node_->declare_parameter("STOP_DISTANCE_MM",            STOP_DISTANCE_MM);
    node_->declare_parameter("MAX_STEER_RATE_DEGREE",       MAX_STEER_RATE_DEGREE);
    node_->declare_parameter("MAX_SPEED_UP_RATE_MM",        MAX_SPEED_UP_RATE_MM);
    node_->declare_parameter("MAX_SPEED_DOWN_RATE_MM",      MAX_SPEED_DOWN_RATE_MM);
    node_->declare_parameter("STEER_KP",                    STEER_KP);
    node_->declare_parameter("STEER_KD",                    STEER_KD);
    node_->declare_parameter("STRAIGHT_SPEED_MM",           STRAIGHT_SPEED_MM);
    node_->declare_parameter("TURN_SPEED_MM",               TURN_SPEED_MM);
    node_->declare_parameter("CROSS_SPEED_MM",              CROSS_SPEED_MM);
}

void ControllerCore::on_reset() {
    has_started = false;
    target_steer = 0.0;
    target_speed = 0.0;
    steer_L = 0.0;
    steer_R = 0.0;
    speed_L = 0.0;
    speed_R = 0.0;
    output_steer = 0.0;
    output_speed = 0.0;

    start_count_ = 0;

    for (int i = 0; i < CENTER_ERROR_HIS_LEN; i++) {
        center_error_his_[i] = 0;
    }
    center_error_his_index_ = 0;

    locked_cross_center_error_ = 0;
    cross_guess_center_error_ = 0;

    cross_start_odom_rad_ = -1;
    cross_odom_increment_rad_ = 0;

    stop_odom_record_enable_ = false;
    stop_start_odom_rad_ = -1;

    received_road_type_ = -1;
    locked_road_type_ = -1;

    black_locked_error_has_value_ = false;
    black_locked_error_ = 0;
}

// ---------------------------------------------------------------------------
void ControllerCore::start_step()
{
    if (start_count_ < START_COUNT_TRIGGER) {
        has_started = false;
        ++start_count_;
    } else {
        has_started = true;
    }
}

// ---------------------------------------------------------------------------
int ControllerCore::get_road_type()
{
    if (locked_road_type_ >= 0) {
        return locked_road_type_;
    } else {
        return received_road_type_;
    }
}

// ---------------------------------------------------------------------------
double ControllerCore::ccd_steer_pid(double error, double error_prev)
{
    double kp = node_->get_parameter("STEER_KP").as_double();
    double kd = node_->get_parameter("STEER_KD").as_double();
    return kp * error + kd * (error - error_prev);
}

// ---------------------------------------------------------------------------
void ControllerCore::apply_steer_dynamics()
{
    double max_steer_rate_degree = node_->get_parameter("MAX_STEER_RATE_DEGREE").as_double();
    double max_change_rad = max_steer_rate_degree * M_PI / 180.0 * STEER_DT;
    double delta = std::clamp(target_steer - output_steer, -max_change_rad, max_change_rad);
    output_steer = std::clamp(output_steer + delta, -MAX_STEER_RAD, MAX_STEER_RAD);
}

// ---------------------------------------------------------------------------
void ControllerCore::apply_speed_dynamics()
{
    double max_speed_up_rate_mm = node_->get_parameter("MAX_SPEED_UP_RATE_MM").as_double();
    double max_speed_down_rate_mm = node_->get_parameter("MAX_SPEED_DOWN_RATE_MM").as_double();

    double max_speed_up_change = max_speed_up_rate_mm * MM_TO_RAD_RATIO * SPEED_DT;
    double max_speed_down_change = max_speed_down_rate_mm * MM_TO_RAD_RATIO * SPEED_DT;

    double delta = target_speed - output_speed;

    if (delta > 0){
        // accelerate, use the min to limit the speed change
        delta = std::min(delta, max_speed_up_change);
    } else {
        // decelerate, use the max to limit the speed change
        delta = std::max(delta, -max_speed_down_change);
    }

    output_speed = std::clamp(output_speed + delta, 0.0, MAX_SPEED_RAD);
}

// ---------------------------------------------------------------------------
void ControllerCore::ackman_steer_calculate()
{
    double steer = output_steer;
    if (std::abs(steer) < 0.005) {
        // if steer is very small, consider it as straight to avoid the noise of steer when straight
        steer_L = 0.0;
        steer_R = 0.0;
        return;
    }


    double turn_radius = WHEEL_BASE_MM / std::tan(steer);
    double kappa = 1.0 / turn_radius;
    if (std::abs(kappa) < 1e-6) {
        // if kappa is very small, consider it as straight to avoid the noise of steer when straight
        steer_L = 0.0;
        steer_R = 0.0;
        return;
    }

    steer_L  = std::atan(WHEEL_BASE_MM / (turn_radius - 0.5 * CAR_WIDTH_MM));
    steer_R = std::atan(WHEEL_BASE_MM / (turn_radius + 0.5 * CAR_WIDTH_MM));
}

// ---------------------------------------------------------------------------
void ControllerCore::ackman_speed_calculate()
{
    double kappa = std::tan(target_steer) / WHEEL_BASE_MM;
    speed_L  = output_speed * (1.0 - 0.5 * CAR_WIDTH_MM * kappa);
    speed_R = output_speed * (1.0 + 0.5 * CAR_WIDTH_MM * kappa);
}

// ---------------------------------------------------------------------------
std::pair<double,double> ControllerCore::steer_control_logic(int road_type, int center_error){
    // 100 hz run the steer control logic based on the perception result
    double cross_lock_error_decay_exp = node_->get_parameter("CROSS_LOCK_ERROR_DECAY_EXP").as_double();
    double straight_speed_mm  = node_->get_parameter("STRAIGHT_SPEED_MM").as_double();
    double turn_speed_mm      = node_->get_parameter("TURN_SPEED_MM").as_double();
    double cross_speed_mm     = node_->get_parameter("CROSS_SPEED_MM").as_double();

    received_road_type_ = road_type;

    int center_error_prev;
    // update error history
    center_error_prev = center_error_his_[center_error_his_index_];
    center_error_his_index_ = (center_error_his_index_ + 1) % CENTER_ERROR_HIS_LEN;
    center_error_his_[center_error_his_index_] = center_error;


    // clear the black locked error value
    if (road_type != Perception::BLACK) {
        // if not see black, reset the black lock
        black_locked_error_has_value_ = false;
        black_locked_error_ = 0;
    }


    // use guess center error for cross when locked
    double steer_by_error;
    double cross_guess_center_error_pre;
    double ratio;


    // steer control logic:
    if (locked_road_type_ == Perception::CROSS) {
        // if locked in cross, use the guess error which decays from the locked error when first locked
        cross_guess_center_error_pre = cross_guess_center_error_;
        ratio = cross_odom_increment_rad_ / (CROSS_DISTANCE_MM * MM_TO_RAD_RATIO);
        ratio = std::clamp(ratio, 0.0, 1.0);

        cross_guess_center_error_ = locked_cross_center_error_ * std::pow(1.0 - ratio, cross_lock_error_decay_exp);
        steer_by_error = ccd_steer_pid(cross_guess_center_error_, cross_guess_center_error_pre);
    } else if (road_type == Perception::BLACK) {
        // if see black, lock the steer to max steer to one direction based on the error when first see the black
        if (!black_locked_error_has_value_) {
            black_locked_error_has_value_ = true;
            int mean_error = 0;
            for (int i = 0; i < CENTER_ERROR_HIS_LEN; ++i) {
                mean_error += center_error_his_[i];
            }
            mean_error /= CENTER_ERROR_HIS_LEN;

            // set the max error
            if (mean_error > 0) {
                black_locked_error_ = CCD_LEN / 2;
            } else {
                black_locked_error_ = - (CCD_LEN / 2);
            }
        }
        // use the locked error for steer control when see black, which is the max steer to one direction based on the error when first see the black
        steer_by_error = ccd_steer_pid(black_locked_error_, black_locked_error_);
    } else {
        // normally, use the current error for control
        steer_by_error = ccd_steer_pid(center_error, center_error_prev);
    }

    target_steer = std::clamp(steer_by_error, -MAX_STEER_RAD, MAX_STEER_RAD);

    if (locked_road_type_ < 0){
        // no lock and consider the road type for control
        if (road_type == Perception::STRAIGHT) {
            target_speed = straight_speed_mm * MM_TO_RAD_RATIO;
        } else if (road_type == Perception::TURN) {
            target_speed = turn_speed_mm * MM_TO_RAD_RATIO;
        } else if (road_type == Perception::CROSS) {
            // lock can only unlock after odometer
            // unlock in wheel control logic after odometer record enough
            locked_road_type_ = Perception::CROSS;

            // use the center error history to guess the center error
            // current center error must be in cross, not reliable
            // first several error in history should not consider
            int mean_error = 0;
            int start = 1 + CROSS_COUNT_TRIGGER; // at least use the first several history error to make the average more stable
            int end = std::min(start + LOCK_CROSS_ERROR_AVG_LEN, CENTER_ERROR_HIS_LEN);
            int index;

            for (int i = start; i < end; ++i) {
                index = (center_error_his_index_ - i + CENTER_ERROR_HIS_LEN) % CENTER_ERROR_HIS_LEN;
                mean_error += center_error_his_[index];
            }
            mean_error /= (end - start);
            locked_cross_center_error_ = mean_error;

        } else if (road_type == Perception::STOP) {
            // just set the flag to record the odometer
            // will lock stop after odometer record enough
            // lock step is in wheel control logic
            stop_odom_record_enable_ = true;
        } else if (road_type == Perception::BLACK) {
            target_speed = turn_speed_mm * MM_TO_RAD_RATIO;
        } else {
            target_speed = 0.0;
            target_speed = 0.0;
        }
    } else {
        // check the lock
        if (locked_road_type_ == Perception::CROSS) {
            // pow n reduce throught the odometer increment
            target_speed = cross_speed_mm * MM_TO_RAD_RATIO;
        } else if (locked_road_type_ == Perception::STOP) {
            target_speed = 0.0;
        }
    }

    apply_steer_dynamics();
    ackman_steer_calculate();
    return {steer_L, steer_R};
}

// ---------------------------------------------------------------------------
std::pair<double,double> ControllerCore::wheel_control_logic(double pos_L, double pos_R)
{
    // 100 hz run the speed control logic based on the current speed
    double stop_distance_mm = node_->get_parameter("STOP_DISTANCE_MM").as_double();
    double avg_pos = 0.5 * (pos_L + pos_R);


    if (stop_odom_record_enable_) {
        //record the stop odometer
        if (stop_start_odom_rad_ < 0) {
            stop_start_odom_rad_ = avg_pos;
        } else {
            // already have start, record the increment
            // need go through certain distance (see distance + car length to make sure the car has fully stop) to lock the stop
            // stop odometer distance
            if (avg_pos - stop_start_odom_rad_ >= stop_distance_mm * MM_TO_RAD_RATIO) {
                // lock the stop after record enough odometer
                locked_road_type_       = Perception::STOP;
                stop_odom_record_enable_ = false;
                stop_start_odom_rad_    = -1;
            }
        }
    } else if (locked_road_type_ == Perception::CROSS){
        // record the cross odometer
        if (cross_start_odom_rad_ < 0) {
            // record the start
            cross_start_odom_rad_ = avg_pos;
            cross_odom_increment_rad_ = 0;
        } else {
            // already have start, record the increment
            cross_odom_increment_rad_ = avg_pos - cross_start_odom_rad_;

            // need go through certain distance to unlock the cross
            if (cross_odom_increment_rad_ >= CROSS_DISTANCE_MM * MM_TO_RAD_RATIO){
                // unlock the cross after record enough odometer
                locked_road_type_ = -1;
                cross_start_odom_rad_ = -1;
                cross_odom_increment_rad_ = 0;
            }
        }
    }

    if (locked_road_type_ == Perception::STOP){
        // if locked in stop, keep speed 0
        output_speed = 0.0;
        speed_L = 0.0;
        speed_R = 0.0;
    } else {
        // otherwise, calculate the speed based on the current target steer and target speed
        apply_speed_dynamics();
        ackman_speed_calculate();
    }
    return {speed_L, speed_R};

}
