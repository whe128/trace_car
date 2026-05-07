#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "trace_car/msg/perception.hpp"
#include "trace_car/msg/car_info.hpp"
#include "trace_car/controller_core.hpp"
#include "trace_car/msg/gui_control.hpp"

constexpr double CTRL_TIMER_PERIOD = 0.01;  // 100 Hz

class ControllerNode : public rclcpp::Node{
public:
    ControllerNode(bool gui)
    : Node("controller_node")
    {
        gui_ = gui;
        controller_core_ = std::make_unique<ControllerCore>(this);

        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(CTRL_TIMER_PERIOD),
            std::bind(&ControllerNode::timer_callback, this)
        );

        // subscriber for perception
        perception_sub_ = this->create_subscription<trace_car::msg::Perception>(
            "/perception",
            10,
            std::bind(&ControllerNode::perception_callback, this, std::placeholders::_1)
        );

        joint_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states",
            10,
            std::bind(&ControllerNode::joint_states_callback, this, std::placeholders::_1)
        );

        // publisher for steer and wheel commands
        steer_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/steer_position_controller/commands",
            10
        );
        wheel_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/wheel_velocity_controller/commands",
            10
        );


        car_info_pub = this->create_publisher<trace_car::msg::CarInfo>(
            "/car_info",
            10
        );

        if (gui_) {
            system_paused_ = true;
            gui_control_sub_ = this->create_subscription<trace_car::msg::GuiControl>(
                "/gui_control",
                5,
                std::bind(&ControllerNode::gui_control_callback, this, std::placeholders::_1)
            );

            RCLCPP_INFO(this->get_logger(), "Controller Node started [GUI mode].");
        } else {
            RCLCPP_INFO(this->get_logger(), "Controller Node started.");
        }
    }

private:
    void on_reset() {
        controller_core_->on_reset();
        send_steer(0.0, 0.0);
        send_wheel(0.0, 0.0);
    }

    void send_wheel(double left, double right)
    {
        std_msgs::msg::Float64MultiArray msg;
        msg.data = {left, right};
        wheel_pub_->publish(msg);
    }

    void send_steer(double left_angle, double right_angle)
    {
        std_msgs::msg::Float64MultiArray msg;
        msg.data = {left_angle, right_angle};
        steer_pub_->publish(msg);
    }

    void timer_callback(){
        if (system_paused_) return;

        if (!controller_core_->has_started) {
            controller_core_->start_step();
        } else {
            // publish car info for GUI display
            if (controller_core_->get_road_type() >= 0) {
                trace_car::msg::CarInfo car_info_msg;

                car_info_msg.target_steer = controller_core_->target_steer;
                car_info_msg.target_speed = controller_core_->target_speed;
                car_info_msg.steer_left   = controller_core_->steer_L;
                car_info_msg.steer_right  = controller_core_->steer_R;
                car_info_msg.speed_left   = controller_core_->speed_L;
                car_info_msg.speed_right  = controller_core_->speed_R;

                car_info_msg.output_steer = controller_core_->output_steer;
                car_info_msg.output_speed = controller_core_->output_speed;

                car_info_msg.road_type = controller_core_->get_road_type();
                car_info_pub->publish(car_info_msg);
            }
        }
    }

    void perception_callback(const trace_car::msg::Perception::SharedPtr msg)
    {
        if (system_paused_) return;

        if (!controller_core_->has_started) {
            send_steer(0.0, 0.0);
            return;
        }

        // 100 hz from ccd node publisher
        int road_type = msg->road_type;
        int center_error = msg->center_error;

        // steer control
        auto [steer_L, steer_R] = controller_core_->steer_control_logic(road_type, center_error);
        send_steer(steer_L, steer_R);
    }

    void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (system_paused_) return;

        // 100 hz from gazebo joint state publisher
        if (!controller_core_->has_started) {
            send_wheel(0.0, 0.0);
            return;
        }

        double wheel_pos_L_ = 0.0;
        double wheel_pos_R_ = 0.0;

        for (size_t i = 0; i < msg->name.size(); i++)
        {
            if (msg->name[i] == "wheel_L_back_to_body")
                wheel_pos_L_ = msg->position[i];
            else if (msg->name[i] == "wheel_R_back_to_body")
                wheel_pos_R_ = msg->position[i];
        }

        // wheel speed control
        auto [wheel_L, wheel_R] = controller_core_->wheel_control_logic(wheel_pos_L_, wheel_pos_R_);
        send_wheel(wheel_L, wheel_R);
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

        }
    }

    bool gui_ = false;
    bool system_paused_ = false;
    std::unique_ptr<ControllerCore> controller_core_;

    std::shared_ptr<rclcpp::Subscription<trace_car::msg::Perception>> perception_sub_;
    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::JointState>> joint_states_sub_;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64MultiArray>> steer_pub_;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64MultiArray>> wheel_pub_;

    // use for gui
    std::shared_ptr<rclcpp::Publisher<trace_car::msg::CarInfo>> car_info_pub;
    std::shared_ptr<rclcpp::Subscription<trace_car::msg::GuiControl>> gui_control_sub_;


    rclcpp::TimerBase::SharedPtr timer_;
};

// ---- main ----
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
    auto node = std::make_shared<ControllerNode>(use_gui);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
