#include <trace_car/ccd_processor.hpp>

using Perception = trace_car::msg::Perception;
// ---------------------------------------------------------------------------
CCDProcessor::CCDProcessor(rclcpp::Node* node){
    node_ = node;

    node_->declare_parameter("MIN_LIGHT_WIDTH_RATIO",    MIN_LIGHT_WIDTH_RATIO);
    node_->declare_parameter("BLACK_THRESHOLD",          BLACK_THRESHOLD);
    node_->declare_parameter("WHITE_THRESHOLD",          WHITE_THRESHOLD);
    node_->declare_parameter("STOP_LIGHT_THRESHOLD",     STOP_LIGHT_THRESHOLD);
    node_->declare_parameter("STRAIGHT_LIGHT_THRESHOLD", STRAIGHT_LIGHT_THRESHOLD);
    node_->declare_parameter("STOP_LINE_WIDTH_RATIO",    STOP_LINE_WIDTH_RATIO);
    node_->declare_parameter("EDGE_LIGHT_THRESHOLD_RATIO",  EDGE_LIGHT_THRESHOLD_RATIO);

    // init center_history
    for (int i = 0; i < CENTER_HIS_LEN; i++) {
        center_close_his[i] = CCD_CENTER;
        center_far_his[i] = CCD_CENTER;
    }
}

void CCDProcessor::on_reset() {
    // reset the history and index
    for (int i = 0; i < CENTER_HIS_LEN; i++) {
        center_close_his[i] = CCD_CENTER;
        center_far_his[i] = CCD_CENTER;
    }

    center_close_his_index = 0;   // current buff index
    center_mean_close = CCD_CENTER;
    left_edge_close = -1;
    right_edge_close = -1;
    trace_width_close = -1;
    avg_brightness_close = 0;

    center_far_his_index = 0;
    center_mean_far = CCD_CENTER;
    left_edge_far = -1;
    right_edge_far = -1;
    trace_width_far = -1;
    avg_brightness_far = 0;

    road_type = -1;

    straight_count_ = 0;
    turn_count_ = 0;
    stop_count_ = 0;
    cross_count_ = 0;
    black_count_ = 0;
}

// ---------------------------------------------------------------------------
void CCDProcessor::median_filter(const uint8_t* raw_img, uint8_t* filtered_img)
{
    int pad_w = MEDIAN_FILTER_W / 2;
    uint8_t window[MEDIAN_FILTER_W];
    // apply median filter
    for (int i = 0; i < CCD_LEN; i++)
    {
        int idx = 0;

        // build window
        for (int j = -pad_w; j <= pad_w; j++){
            int pos = i + j;
            // edge padding
            if (pos < 0) pos = 0;
            if (pos >= CCD_LEN) pos = CCD_LEN - 1;

            window[idx] = raw_img[pos];
            idx++;
        }
        // sort window and get median
        std::nth_element(window, window + pad_w, window + MEDIAN_FILTER_W);

        // store median value to filtered image
        filtered_img[i] = window[pad_w];
    }
}

// ---------------------------------------------------------------------------
void CCDProcessor::temporal_median_filter(const uint8_t fram_buffer[FRAME_BUFFER_SIZE][CCD_LEN], std::array<uint8_t, CCD_LEN>& filtered_img)
{
    // apply median filter on frame buffer
    uint8_t window[FRAME_BUFFER_SIZE];

    for (int i = 0; i < CCD_LEN; i++)
    {
        // build window, raw direction
        for (int f = 0; f < FRAME_BUFFER_SIZE; f++){
            window[f] = fram_buffer[f][i];
        }
        // sort window and get median
        std::nth_element(window, window + FRAME_BUFFER_SIZE / 2, window + FRAME_BUFFER_SIZE);
        // store median value to filtered image
        filtered_img[i] = window[FRAME_BUFFER_SIZE / 2];
    }
}

// ---------------------------------------------------------------------------
void CCDProcessor:: analyse_ccd_close(const uint8_t* img){
    LineResult line_result = detect_line(img, center_mean_close, trace_width_close, true);

    // update history
    avg_brightness_close = line_result.avg_brightness;

    if (!line_result.line_detected) {
        // no line detected, just update brightness and return
        left_edge_close = -1;
        right_edge_close = -1;
        return;
    }

    left_edge_close = line_result.left_edge;
    right_edge_close = line_result.right_edge;
    if (trace_width_close < 0 && line_result.width > 0 && left_edge_close >= 0 && right_edge_close >= 0) {
        trace_width_close = line_result.width;
    }

    // update center history and mean
    center_close_his_index = (center_close_his_index + 1) % CENTER_HIS_LEN;
    center_close_his[center_close_his_index] = line_result.center;

    // mean
    int sum = 0;
    for (int i = 0; i < CENTER_HIS_LEN; i++) {
        sum += center_close_his[i];
    }
    center_mean_close = sum / CENTER_HIS_LEN;
}

// ---------------------------------------------------------------------------
void CCDProcessor:: analyse_ccd_far(const uint8_t* img){
    LineResult line_result = detect_line(img, center_mean_far, trace_width_far, false);

    // update history
    avg_brightness_far = line_result.avg_brightness;

    if (!line_result.line_detected) {
        // no line detected, just update brightness and return
        left_edge_far = -1;
        right_edge_far = -1;
        return;
    }

    left_edge_far = line_result.left_edge;
    right_edge_far = line_result.right_edge;
    if (trace_width_far < 0 && line_result.width > 0 && left_edge_far >= 0 && right_edge_far >= 0) {
        trace_width_far = line_result.width;
    }
    // update center history and mean
    center_far_his_index = (center_far_his_index + 1) % CENTER_HIS_LEN;
    center_far_his[center_far_his_index] = line_result.center;

    int sum = 0;
    for (int i = 0; i < CENTER_HIS_LEN; i++) {
        sum += center_far_his[i];
    }
    center_mean_far = sum / CENTER_HIS_LEN;
}
// ---------------------------------------------------------------------------
LineResult CCDProcessor::detect_line(
    const uint8_t* img,
    int avg_center,
    int init_trace_width,
    bool max_change_avoid_enable
)
{
    LineResult line_result;

    // ---------- ros parameter ----------
    int black_threshold = node_->get_parameter("BLACK_THRESHOLD").as_int();
    int white_threshold = node_->get_parameter("WHITE_THRESHOLD").as_int();

    // ---------- avg brightness ----------
    int avg_brightness = 0;
    for (int i = 0; i < CCD_LEN; i++) {
        avg_brightness += img[i];
    }
    avg_brightness /= CCD_LEN;
    line_result.avg_brightness = avg_brightness;

    // too dark or too bright, consider as no line
    if (avg_brightness < black_threshold || avg_brightness > white_threshold) {
        // no update to line_result, just return
        return line_result;
    }

    // ---------- smooth ----------
    uint8_t smooth[CCD_LEN] = {};
    int l, r, sum;
    for (int i = 0; i < CCD_LEN; i++) {
        l = std::max(0, i - SMOOTH_W);
        r = std::min(CCD_LEN, i + SMOOTH_W + 1);
        sum = 0;
        for (int j = l; j < r; j++) {
            sum += img[j];
        }
        smooth[i] = sum / (r - l);
    }

    // ---------- diff (right - left) ----------
    int diff[CCD_LEN] = {};
    int left_sum;
    int right_sum;
    for (int i = DIFF_W; i < CCD_LEN - DIFF_W; i++) {
        left_sum = 0;
        right_sum = 0;
        for (int j = i - DIFF_W; j < i; j++) {
            left_sum += smooth[j];
        }
        for (int j = i + 1; j <= i + DIFF_W; j++) {
            right_sum += smooth[j];
        }

        diff[i] = (right_sum - left_sum) / DIFF_W;
    }

    // ---------- edge threshold ----------
    double edge_light_threshold_ratio = node_->get_parameter("EDGE_LIGHT_THRESHOLD_RATIO").as_double();
    int max_smooth = smooth[0];
    int min_smooth = smooth[0];
    for (int i = 0; i < CCD_LEN; i++) {
        if (smooth[i] > max_smooth) max_smooth = smooth[i];
        if (smooth[i] < min_smooth) min_smooth = smooth[i];
    }
    double edge_threshold = (max_smooth - min_smooth) * edge_light_threshold_ratio;

    int  left_edge  = -1;
    int  right_edge = -1;
    int  center     = avg_center;
    int  trace_width = -1;

    // ---------- find edges ----------
    if (FIND_EDGE_FROM_CENTER) {
        // ---------- scan from center outward ----------
        // find left edge
        for(int i = avg_center; i >= DIFF_W; i--) {
            if (diff[i] > edge_threshold) {
                left_edge = i;
                break;
            }
        }
        // find right edge
        for (int i = avg_center; i < CCD_LEN - DIFF_W; i++) {
            if (diff[i] < -edge_threshold) {
                right_edge = i;
                break;
            }
        }

        if (left_edge >= 0 && right_edge >= 0) {
            // both edges found, calculate trace width and center
            center = (left_edge + right_edge) / 2;
            trace_width = right_edge - left_edge;
        } else if (left_edge >= 0 && right_edge < 0) {
            // only left edge found, estimate right edge and trace width
            center = (init_trace_width > 0)
                ? (left_edge + init_trace_width / 2)
                : (left_edge + 20);
            center = std::min(center, CCD_LEN - 1);
        } else if (left_edge < 0 && right_edge >= 0) {
            // only right edge found, estimate left edge and trace width
            center = (init_trace_width > 0)
                ? (right_edge - init_trace_width / 2)
                : (right_edge - 20);
            center = std::max(center, 0);
        } else {
            // no edge found, keep center and width unchanged
            center = avg_center;
        }
    } else {
        // ---------- full scan, collect all up/down edges ----------
        // each entry: {position, is_up_edge}

        std::vector<std::pair<int, bool>> edges;
        double diff_val_pre = 0;
        double diff_val = 0;

        int start_continuous_edge = -1;
        int continuous_middle = -1;
        for (int i = DIFF_W; i < CCD_LEN - DIFF_W; i++) {
            // update diff and diff_pre
            diff_val_pre = diff_val;
            diff_val = diff[i];

            if (diff_val > edge_threshold){
                // up edge
                if (start_continuous_edge < 0) {
                    // find the start edge, but not start to detect continuous edge
                    start_continuous_edge = i;
                } else {
                    // already find the start edge, and now still in the continuous edge, do nothing
                    if (diff_val_pre <= edge_threshold) {
                        // continuous eddge is down, but current is up, stop continuous edge detection, record the down edge
                        continuous_middle = (start_continuous_edge + i) / 2;
                        edges.push_back({continuous_middle, false});
                        start_continuous_edge = i;
                    }
                    // else continuous edge is up, continue to detect
                }
            } else if (diff_val < -edge_threshold) {
                // down edge
                if (start_continuous_edge < 0) {
                    // find the start edge, but not start to detect continuous edge
                    start_continuous_edge = i;
                } else {
                    // already find the start edge, and now still in the continuous edge, do nothing
                    if (diff_val_pre >= -edge_threshold) {
                        // continuous eddge is up, but current is down, stop continuous edge detection, record the up edge
                        continuous_middle = (start_continuous_edge + i) / 2;
                        edges.push_back({continuous_middle, true});
                        start_continuous_edge = i;
                    }
                    // else continuous edge is down, continue to detect
                }
            } else {
                // not edge, stop continuous edge detection
                if (start_continuous_edge >= 0) {
                    if (diff_val_pre > edge_threshold) {
                        // previous edge is up edge, record the continuous up edge
                        continuous_middle = (start_continuous_edge + i) / 2;
                        edges.push_back({continuous_middle, true});
                        start_continuous_edge = -1;
                    } else if (diff_val_pre < -edge_threshold) {
                        // previous edge is down edge, record the continuous down edge
                        continuous_middle = (start_continuous_edge + i) / 2;
                        edges.push_back({continuous_middle, false});
                        start_continuous_edge = -1;
                    }
                }
            }
        }

        if (start_continuous_edge >= 0) {
            // finish edge detection, and still have continuous edge, record the edge
            continuous_middle = (start_continuous_edge + (CCD_LEN - 1)) / 2;
            if (diff_val_pre > edge_threshold) {
                // previous edge is up edge, record the continuous up edge
                edges.push_back({continuous_middle, true});
            } else if (diff_val_pre < -edge_threshold) {
                // previous edge is down edge, record the continuous down edge
                edges.push_back({continuous_middle, false});
            }
        }

        // ---------- build light windows from edges ----------
        std::vector<std::pair<int, int>> light_windows; // {left_edge, right_edge}
        std::optional<bool> is_pre_up_edge = std::nullopt;
        int up_edge = -1;

        for (auto& [edge_pose, is_up_edge] : edges) {
            if (is_up_edge) {
                // up edge
                if (!is_pre_up_edge.has_value() || !is_pre_up_edge.value()) {
                    // no previous edge, or previous is down
                    is_pre_up_edge = true;
                    up_edge = edge_pose;
                }

                // else previous is up, ignore
            } else {
                // down edge
                if (!is_pre_up_edge.has_value()){
                    // no previous edge, first finded the down edge
                    is_pre_up_edge = false;
                    light_windows.push_back({0, edge_pose});
                } else if (is_pre_up_edge.value()) {
                    //previous edge is up edge, find down edge,
                    // record the light window
                    is_pre_up_edge = false;
                    light_windows.push_back({up_edge, edge_pose});
                }
            }
        }

        // check the end window when reach the end
        if (is_pre_up_edge.has_value() && is_pre_up_edge.value()) {
            // last edge is up edge, but no down edge found, record the light window from last up edge to the end
            light_windows.push_back({up_edge, CCD_LEN - 1});
        }

        // ---------- find best window near avg_center ----------
        int temp_left = -1;
        int temp_right = -1;
        int min_center_diff = CCD_LEN;
        int min_light_width_ratio = node_->get_parameter("MIN_LIGHT_WIDTH_RATIO").as_double();
        int min_light_width = init_trace_width > 0
                            ? static_cast<int>(init_trace_width * min_light_width_ratio)
                            : 3;

        for (auto& [left, right] : light_windows) {
            int window_center = (left + right) / 2;
            int center_diff = std::abs(window_center - avg_center);

            // skip very narrow non-border windows
            if (right - left < min_light_width && left > 0 && right < CCD_LEN - 1) {
                continue;
            }

            if (center_diff < min_center_diff) {
                // better than previous best window, and also wider than minimum light width, update best window
                temp_left = left;
                temp_right = right;
                min_center_diff = center_diff;
            }
        }

        if (temp_left >= 0 && temp_right >= 0) {
            if (temp_left > 0 && temp_right < CCD_LEN - 1) {
                // both edges detected, calculate center
                left_edge = temp_left;
                right_edge = temp_right;
                center = (left_edge + right_edge) / 2;
                trace_width = right_edge - left_edge;
            } else if (temp_left > 0 && temp_right == CCD_LEN - 1) {
                // only left edge detected, estimate right edge
                left_edge = temp_left;
                center = (init_trace_width > 0)
                    ? (left_edge + init_trace_width / 2)
                    : (left_edge + 20);
            } else if (temp_left == 0 && temp_right < CCD_LEN - 1) {
                // only right edge detected, estimate left edge
                right_edge = temp_right;
                center = (init_trace_width > 0)
                    ? (right_edge - init_trace_width / 2)
                    : (right_edge - 20);
            } else {
                // both edges are border, consider as no edge detected, return None
                center = avg_center;
            }
        } else {
            //no edge detected, return None
            center = avg_center;
        }

        if (max_change_avoid_enable && road_type != Perception::BLACK) {
            // ---------- max change avoid ----------
                // impossible change
            if (std::abs(center - avg_center) > CCD_LEN / 3) {
                // center jump too large, consider as wrong detection, return avg_center
                center = avg_center;
            } else if (center - avg_center > CCD_LEN / 4) {
                center = avg_center + CCD_LEN / 4;
            } else if (avg_center - center > CCD_LEN / 4) {
                center = avg_center - CCD_LEN / 4;
            }
        }

        center = std::clamp(center, 0, CCD_LEN - 1);
    }

    // update line_result
    line_result.line_detected = true;
    line_result.left_edge = left_edge;
    line_result.right_edge = right_edge;
    line_result.center = center;
    line_result.width = trace_width;

    return line_result;
}

// ---------------------------------------------------------------------------
void CCDProcessor::perception_analyse()
{
    double stop_line_width_ratio    = node_->get_parameter("STOP_LINE_WIDTH_RATIO").as_double();
    int    stop_light_threshold     = node_->get_parameter("STOP_LIGHT_THRESHOLD").as_int();
    int    black_threshold          = node_->get_parameter("BLACK_THRESHOLD").as_int();
    int    white_threshold          = node_->get_parameter("WHITE_THRESHOLD").as_int();
    int    straight_light_threshold = node_->get_parameter("STRAIGHT_LIGHT_THRESHOLD").as_int();

    int left_edge    = left_edge_close;
    int right_edge   = right_edge_close;
    int center       = center_mean_close;
    int trace_width  = trace_width_close;
    int avg_brightness = avg_brightness_close;

    // road type in condition: previous
    // road type in assignment, new road type
    // see the stop line, the width less than original width

    // ---- STOP ----
    if (right_edge >= 0 && left_edge >= 0 && trace_width > 0
        && right_edge - left_edge < static_cast<int>(stop_line_width_ratio * trace_width)
        && avg_brightness < stop_light_threshold
        && (
            road_type == Perception::STRAIGHT
            || road_type == Perception::TURN
            || (
                std::abs(center_mean_close - CCD_CENTER) < straight_light_threshold
                && std::abs(center_mean_far - CCD_CENTER) < straight_light_threshold
            )
        )
    ){
        stop_count_++;
        straight_count_ = 0;
        turn_count_ = 0;
        cross_count_ = 0;
        black_count_ = 0;
        if (stop_count_ >= STOP_COUNT_TRIGGER) {
            road_type = Perception::STOP;
        }
    }
    // ---- BLACK ----
    else if (avg_brightness < black_threshold) {
        black_count_++;
        straight_count_ = 0;
        turn_count_ = 0;
        cross_count_ = 0;
        stop_count_ = 0;
        if (black_count_ >= BLACK_COUNT_TRIGGER) {
            road_type = Perception::BLACK;
        }
    }
    // ---- CROSS ----
    else if (avg_brightness > white_threshold
        || (
            left_edge < 0 && right_edge >= 0
                && avg_brightness > static_cast<int>(0.8 * white_threshold)
        )
    ) {
        cross_count_++;
        straight_count_ = 0;
        turn_count_ = 0;
        stop_count_ = 0;
        black_count_ = 0;
        if (cross_count_ >= CROSS_COUNT_TRIGGER) {
            road_type = Perception::CROSS;
        }
    }
    // ---- STRAIGHT ----
    else if ( std::abs(center - CCD_CENTER) < straight_light_threshold
        && std::abs(center_mean_far - CCD_CENTER) < straight_light_threshold
    ) {
        straight_count_++;
        turn_count_ = 0;
        cross_count_ = 0;
        stop_count_ = 0;
        black_count_ = 0;
        if (straight_count_ >= STRAIGHT_COUNT_TRIGGER) {
            road_type = Perception::STRAIGHT;
        }
    }
    else {
        // other cases, consider as turn
        turn_count_++;
        straight_count_ = 0;
        cross_count_ = 0;
        stop_count_ = 0;
        black_count_ = 0;
        if (turn_count_ >= TURN_COUNT_TRIGGER) {
            road_type = Perception::TURN;
        }
    }
}
