// coding: utf-8
// ccd_node.cpp  –  ROS 2 node that drives two CCD line sensors

# define USE_OPENCV 1 // for visualization only, can be removed if not needed

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <algorithm>

#include "trace_car/ccd_processor.hpp"
#include "trace_car/msg/perception.hpp"
#include "trace_car/msg/detection.hpp"
#include "trace_car/msg/gui_control.hpp"

// ---- adjustable ----
constexpr bool SHOW_IMAGE     = false;
constexpr bool SHOW_RAW_IMAGE = true;

// ---- fixed ----
constexpr int    INIT_COUNT        = 10;
constexpr int    SHOW_IMAGE_W      = 640;
constexpr int    SHOW_IMAGE_H      = 150;
constexpr int    SHOW_EDGE_W       = 3;
constexpr double TIMER_PERIOD      = 0.01;   // 100 Hz

# if USE_OPENCV
#include <opencv2/opencv.hpp>

const cv::Scalar BLUE  = {255,   0,   0};
const cv::Scalar GREEN = {  0, 255,   0};
const cv::Scalar RED   = {  0,   0, 255};
# endif

// buffer shape: (FRAME_BUFFER_SIZE, CCD_LEN), returns 1-D vector


class CCDNode : public rclcpp::Node{
public:
    CCDNode(bool gui):
        Node("ccd_node")
    {
        gui_ = gui;

        ccd_processor_ = std::make_unique<CCDProcessor>(this);

        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(TIMER_PERIOD),
            std::bind(&CCDNode::timer_callback, this)
        );

        // subscribe to CCD images
        ccd_close_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/ccd/ccd_close/image_raw",
            1,
            std::bind(&CCDNode::ccd_close_callback, this, std::placeholders::_1)
        );

        ccd_far_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/ccd/ccd_far/image_raw",
            1,
            std::bind(&CCDNode::ccd_far_callback, this, std::placeholders::_1)
        );

        // publisher for perception
        perception_pub_ = this->create_publisher<trace_car::msg::Perception>(
            "/perception",
            10
        );

        // windows for visualization
        if (show_image_)
        {
          if (SHOW_RAW_IMAGE) cv::namedWindow("CCD_raw");
          cv::namedWindow("CCD_filtered");
        }

        if (gui_){
            system_paused_ = true;
            // if use GUI, publish ccd detection result to gui
            ccd_detection_pub_ = this->create_publisher<trace_car::msg::Detection>(
            "/ccd_detection",
            10);

            gui_control_sub_ = this->create_subscription<trace_car::msg::GuiControl>(
                "/gui_control",
                5,
                std::bind(&CCDNode::gui_control_callback, this, std::placeholders::_1)
            );

            RCLCPP_INFO(this->get_logger(), "CCD Node started [GUI mode]");
        } else{
            RCLCPP_INFO(this->get_logger(), "CCD Node started.");
        }
    }

private:
    void on_reset()
    {
        ccd_processor_->on_reset();

        for (int i = 0; i < CCD_LEN; i++) {
            close_img_raw_[i] = 0;
            close_img_filtered_[i] = 0;

            far_img_raw_[i] = 0;
            far_img_filtered_[i] = 0;
        }

        for (int f = 0; f < FRAME_BUFFER_SIZE; f++) {
            for (int i = 0; i < CCD_LEN; i++) {
                close_img_buffer_[f][i] = 0;
                far_img_buffer_[f][i] = 0;
            }
        }


        close_img_init_count_ = 0;
        far_img_init_count_ = 0;

        close_img_buffer_index_ = 0;
        far_img_buffer_index_ = 0;
    }
#if USE_OPENCV
    void show_image()
    {

        if (!show_image_) return;

        if (SHOW_RAW_IMAGE)
        {
            cv::Mat close_vis, far_vis;

            // transform uint8_t array to cv::Mat for visualization
            cv::Mat close_img_raw_mat(1, CCD_LEN, CV_8UC1, close_img_raw_);
            cv::Mat far_img_raw_mat(1, CCD_LEN, CV_8UC1, far_img_raw_);

            cv::resize(close_img_raw_mat, close_vis, {SHOW_IMAGE_W, SHOW_IMAGE_H}, 0, 0, cv::INTER_NEAREST);
            cv::resize(far_img_raw_mat,   far_vis,   {SHOW_IMAGE_W, SHOW_IMAGE_H}, 0, 0, cv::INTER_NEAREST);
            cv::cvtColor(close_vis, close_vis, cv::COLOR_GRAY2BGR);
            cv::cvtColor(far_vis,   far_vis,   cv::COLOR_GRAY2BGR);
            cv::putText(close_vis, "ccd_close_raw", {10,30}, cv::FONT_HERSHEY_SIMPLEX, 0.8, GREEN, 2);
            cv::putText(far_vis,   "ccd_far_raw",   {10,30}, cv::FONT_HERSHEY_SIMPLEX, 0.8, GREEN, 2);
            int w = close_vis.cols;
            cv::Mat gap(30, w, CV_8UC3, cv::Scalar(255,255,255));
            cv::Mat combined;
            cv::vconcat(std::vector<cv::Mat>{close_vis, gap, far_vis}, combined);
            cv::imshow("CCD_raw", combined);
        }

        cv::Mat close_vis, far_vis;

        cv::Mat close_img_filtered_mat(1, CCD_LEN, CV_8UC1, close_img_filtered_.data());
        cv::Mat far_img_filtered_mat(1, CCD_LEN, CV_8UC1, far_img_filtered_.data());

        cv::resize(close_img_filtered_mat, close_vis, {SHOW_IMAGE_W, SHOW_IMAGE_H}, 0, 0, cv::INTER_NEAREST);
        cv::resize(far_img_filtered_mat,   far_vis,   {SHOW_IMAGE_W, SHOW_IMAGE_H}, 0, 0, cv::INTER_NEAREST);
        cv::cvtColor(close_vis, close_vis, cv::COLOR_GRAY2BGR);
        cv::cvtColor(far_vis,   far_vis,   cv::COLOR_GRAY2BGR);

        double scale = static_cast<double>(SHOW_IMAGE_W) / CCD_LEN;

        //close CCD annotations
        int cx = static_cast<int>(ccd_processor_->center_close_his[ccd_processor_->center_close_his_index] * scale);
        cv::line(close_vis, {cx,0}, {cx,SHOW_IMAGE_H}, RED,   SHOW_EDGE_W);
        if (ccd_processor_->left_edge_close >= 0) {
            int lx = static_cast<int>(ccd_processor_->left_edge_close * scale);
            cv::line(close_vis, {lx,0}, {lx,SHOW_IMAGE_H}, BLUE,  SHOW_EDGE_W);
        }
        if (ccd_processor_->right_edge_close >= 0) {
            int rx = static_cast<int>(ccd_processor_->right_edge_close * scale);
            cv::line(close_vis, {rx,0}, {rx,SHOW_IMAGE_H}, GREEN, SHOW_EDGE_W);
        }

        // far CCD annotations
        int fx = static_cast<int>(ccd_processor_->center_far_his[ccd_processor_->center_far_his_index] * scale);
        cv::line(far_vis, {fx,0}, {fx,SHOW_IMAGE_H}, RED,   SHOW_EDGE_W);
        if (ccd_processor_->left_edge_far >= 0) {
            int lx = static_cast<int>(ccd_processor_->left_edge_far * scale);
            cv::line(far_vis, {lx,0}, {lx,SHOW_IMAGE_H}, BLUE,  SHOW_EDGE_W);
        }
        if (ccd_processor_->right_edge_far >= 0) {
            int rx = static_cast<int>(ccd_processor_->right_edge_far * scale);
            cv::line(far_vis, {rx,0}, {rx,SHOW_IMAGE_H}, GREEN, SHOW_EDGE_W);
        }

        // title
        cv::putText(close_vis, "ccd_close", {10,30}, cv::FONT_HERSHEY_SIMPLEX, 0.8, GREEN, 2);
        cv::putText(far_vis,   "ccd_far",   {10,30}, cv::FONT_HERSHEY_SIMPLEX, 0.8, GREEN, 2);

        //trace width
        if (ccd_processor_->trace_width_close > 0)
            cv::putText(close_vis,
                "road_width: " + std::to_string(ccd_processor_->trace_width_close),
                {150,30}, cv::FONT_HERSHEY_SIMPLEX, 0.7, GREEN, 2);
        if (ccd_processor_->trace_width_far > 0)
            cv::putText(far_vis,
                "road_width: " + std::to_string(ccd_processor_->trace_width_far),
                {150,30}, cv::FONT_HERSHEY_SIMPLEX, 0.7, GREEN, 2);
        //brightness
        cv::putText(close_vis,
            "avg_brightness: " + std::to_string(ccd_processor_->avg_brightness_close),
            {380,30}, cv::FONT_HERSHEY_SIMPLEX, 0.7, GREEN, 2);
        cv::putText(far_vis,
            "avg_brightness: " + std::to_string(ccd_processor_->avg_brightness_far),
            {380,30}, cv::FONT_HERSHEY_SIMPLEX, 0.7, GREEN, 2);

        cv::Mat gap(30, SHOW_IMAGE_W, CV_8UC3, cv::Scalar(255,255,255));
        cv::Mat combined;
        cv::vconcat(std::vector<cv::Mat>{close_vis, gap, far_vis}, combined);
        cv::imshow("CCD_filtered", combined);
        cv::waitKey(1);
    }
#endif

    // ------ callbacks
    void ccd_close_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        if (system_paused_) return;

        if (msg->encoding != "mono8" and msg->width != CCD_LEN)
        {
            RCLCPP_ERROR(this->get_logger(), "Unexpected image encoding or width: %s, %d", msg->encoding.c_str(), msg->width);
            return;
        }
        // get the raw image
        std::memcpy(close_img_raw_, msg->data.data(), CCD_LEN * sizeof(uint8_t));

        // median filter
        close_img_buffer_index_ = (close_img_buffer_index_ + 1) % FRAME_BUFFER_SIZE;
        ccd_processor_->median_filter(close_img_raw_, close_img_buffer_[close_img_buffer_index_]);

        close_img_init_count_++;

        if (close_img_init_count_ < INIT_COUNT) return;

        // median filter on frame buffer
        ccd_processor_->temporal_median_filter(close_img_buffer_, close_img_filtered_);

        // analyze the image and publish perception
        ccd_processor_->analyse_ccd_close(close_img_filtered_.data());
    }

    void ccd_far_callback(const sensor_msgs::msg::Image::SharedPtr msg){
        if (system_paused_) return;

        if (msg->encoding != "mono8" and msg->width != CCD_LEN)
        {
            RCLCPP_ERROR(this->get_logger(), "Unexpected image encoding or width: %s, %d", msg->encoding.c_str(), msg->width);
            return;
        }
        // get the raw image
        std::memcpy(far_img_raw_, msg->data.data(), CCD_LEN * sizeof(uint8_t));

        // median filter
        far_img_buffer_index_ = (far_img_buffer_index_ + 1) % FRAME_BUFFER_SIZE;
        ccd_processor_->median_filter(far_img_raw_, far_img_buffer_[far_img_buffer_index_]);
        far_img_init_count_++;

        if (far_img_init_count_ < INIT_COUNT) return;

        // median filter on frame buffer
        ccd_processor_->temporal_median_filter(far_img_buffer_, far_img_filtered_);

        // analyze the image and publish perception
        ccd_processor_->analyse_ccd_far(far_img_filtered_.data());
    }

    void timer_callback(){
        if (system_paused_) return;

        ccd_processor_->perception_analyse();
        if (ccd_processor_->road_type < 0) {
            return;
        }
        trace_car::msg::Perception msg;
        msg.road_type = ccd_processor_->road_type;
        msg.center_error = CCD_CENTER - ccd_processor_->center_mean_close;
        perception_pub_->publish(msg);

        if (gui_) {
            trace_car::msg::Detection detection_msg;

            detection_msg.far_img = far_img_filtered_;
            detection_msg.far_left_edge = ccd_processor_->left_edge_far;
            detection_msg.far_right_edge = ccd_processor_->right_edge_far;
            detection_msg.far_center = ccd_processor_->center_far_his[ccd_processor_->center_far_his_index];
            detection_msg.far_road_width = ccd_processor_->trace_width_far;
            detection_msg.far_avg_brightness = ccd_processor_->avg_brightness_far;

            detection_msg.close_img = close_img_filtered_;
            detection_msg.close_left_edge = ccd_processor_->left_edge_close;
            detection_msg.close_right_edge = ccd_processor_->right_edge_close;
            detection_msg.close_center = ccd_processor_->center_close_his[ccd_processor_->center_close_his_index];
            detection_msg.close_road_width = ccd_processor_->trace_width_close;
            detection_msg.close_avg_brightness = ccd_processor_->avg_brightness_close;

            ccd_detection_pub_->publish(detection_msg);
        }


#if USE_OPENCV
        show_image();
#endif
    }

    void gui_control_callback(const trace_car::msg::GuiControl::SharedPtr msg)
    {
        switch (msg->command) {
            case trace_car::msg::GuiControl::START:
                system_paused_ = false;
                break;
            case trace_car::msg::GuiControl::PAUSE:
                system_paused_ = true;
                break;
            case trace_car::msg::GuiControl::RESET:
                system_paused_ = true;
                on_reset();
                break;
            case trace_car::msg::GuiControl::SHOW_IMAGE:
                show_image_ = true;
                system_paused_ = false;
                break;
            case trace_car::msg::GuiControl::HIDE_IMAGE:
                cv::destroyAllWindows();
                show_image_ = false;
                break;
        }
    }
    bool gui_ = false;
    bool system_paused_ = false;
    bool show_image_ = SHOW_IMAGE;
    // -- members ---
    std::unique_ptr<CCDProcessor> ccd_processor_;
    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Image>> ccd_close_sub_;
    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Image>> ccd_far_sub_;
    std::shared_ptr<rclcpp::Publisher<trace_car::msg::Perception>> perception_pub_;

    // use for gui
    std::shared_ptr<rclcpp::Publisher<trace_car::msg::Detection>> ccd_detection_pub_;
    std::shared_ptr<rclcpp::Subscription<trace_car::msg::GuiControl>> gui_control_sub_;


    rclcpp::TimerBase::SharedPtr timer_;

    // images
    int     close_img_init_count_ = 0;
    uint8_t close_img_raw_[CCD_LEN] = {};
    uint8_t close_img_buffer_[FRAME_BUFFER_SIZE][CCD_LEN] = {};
    std::array<uint8_t, CCD_LEN> close_img_filtered_ = {};


    int     far_img_init_count_ = 0;
    uint8_t far_img_raw_[CCD_LEN] = {};
    uint8_t far_img_buffer_[FRAME_BUFFER_SIZE][CCD_LEN] = {};
    std::array<uint8_t, CCD_LEN> far_img_filtered_ = {};


    // update index for frame buffer
    int close_img_buffer_index_ = 0;
    int far_img_buffer_index_ = 0;
};


// ---------- main function ----------
int main(int argc, char *argv[])
{
    bool use_gui = false;
    for(int i = 1; i < argc; i++) {
        std::string arg(argv[i]);
        if (arg == "--gui") {
            use_gui = true;
        }
    }

    rclcpp::init(argc, argv);
    auto node = std::make_shared<CCDNode>(use_gui);
    rclcpp::spin(node);
    rclcpp::shutdown();
    cv::destroyAllWindows();
    return 0;
}
