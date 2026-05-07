#include <gazebo/gui/GuiPlugin.hh>
#include <gazebo/gui/GuiIface.hh>
#include <gazebo/rendering/UserCamera.hh>
#include <gazebo/rendering/Scene.hh>
#include <gazebo/rendering/RenderEvents.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <thread>

#include "trace_car/msg/car_info.hpp"

# define USE_DELTA_TRACKING 0

namespace gazebo
{
class CameraViewPlugin : public GUIPlugin
{
public:
  CameraViewPlugin() : GUIPlugin()
  {
    // hide the plugin in the GUI
    this->setStyleSheet("QWidget { background-color: transparent; }");
     this->resize(0, 0);

    // init ROS2
    if(!rclcpp::ok()) rclcpp::init(0, nullptr);

    node_ = std::make_shared<rclcpp::Node>("gazebo_camera_view_plugin");

    cmd_sub_ = node_->create_subscription<std_msgs::msg::String>(
      "/camera_view_cmd", 10,
      std::bind(&CameraViewPlugin::OnMsg, this, std::placeholders::_1));

    car_info_sub_ = node_->create_subscription<trace_car::msg::CarInfo>(
      "/car_info", 10,
      [this](const trace_car::msg::CarInfo::SharedPtr msg) {
        output_steer_ = msg->output_steer;
      });

    // ROS spin in a separate thread
    ros_thread_ = std::thread([this]() {
      rclcpp::spin(node_);
    });

    this->preRendder_ = rendering::Events::ConnectCameraPreRender (
      std::bind(&CameraViewPlugin::OnPreRender, this));

    std::cout << "[CameraViewPlugin] Loaded." << std::endl;
  }

  ~CameraViewPlugin()
  {
    rclcpp::shutdown();
    if (ros_thread_.joinable())
      ros_thread_.join();
  }

private:
    /*
        TrackVisual("car")     follow the car
        TrackedVisual()        get the tracked entity
        SetTrackPosition()     set offset position from the tracked entity
        SetTrackInheritYaw(true)   inherit the yaw rotation of the tracked entity
        TrackVisual("")             stop tracking
    */
  void OnMsg(const std_msgs::msg::String::SharedPtr msg)
  {
    std::string cmd = msg->data;

    {
      std::lock_guard<std::mutex> lock(pose_mutex_);

      if (cmd == "global") {
        global_pendingFrames_ = 1;
        view_mode_ = 0;
      } else if (cmd == "global_track") {
        global_pendingFrames_ = 0;
        view_mode_ = 1;
      } else if (cmd == "first") {
        global_pendingFrames_ = 0;
        view_mode_ = 2;
      } else if (cmd == "third") {
        global_pendingFrames_ = 0;
        view_mode_ = 3;
#if USE_DELTA_TRACKING
        init_cam_yaw_ = true;
#endif
      } else {
        global_pendingFrames_ = 0;
        view_mode_ = -1;
        return;
      }

      pending_pose_ = ignition::math::Pose3d(
        cam_table[view_mode_][0],
        cam_table[view_mode_][1],
        cam_table[view_mode_][2],
        cam_table[view_mode_][3],
        cam_table[view_mode_][4],
        cam_table[view_mode_][5]
      );

      pending_euler_ = pending_pose_.Rot().Euler();
    }
  }

  void OnPreRender()
  {
    std::lock_guard<std::mutex> lock(pose_mutex_);

#if USE_DELTA_TRACKING
    common::Time current_time = common::Time::GetWallTime();
    double dt = (current_time - last_update_time_).Double();
    last_update_time_ = current_time;
# endif

    if (view_mode_ < 0) return;

    auto cam = gui::get_active_camera();
    if (!cam) return;


    if (view_mode_== 0){
        if (global_pendingFrames_ <= 0) return;
        --global_pendingFrames_;
        if (global_pendingFrames_ == 0) cam->SetWorldPose(pending_pose_);
    } else {
        // global track, first, third, need fresh the pose every frame
        auto scene = cam->GetScene();
        if (!scene) return;

        auto visual = scene->GetVisual("car");
        if (!visual) return;


        auto car_pose = visual->WorldPose();

# if USE_DELTA_TRACKING
        if (init_cam_yaw_) {
            auto cam_pose = cam->WorldPose();
            auto& q2 = cam_pose.Rot();
            cam_yaw_cached_ = std::atan2(
                2.0 * (q2.W() * q2.Z() + q2.X() * q2.Y()),
                1.0 - 2.0 * (q2.Y() * q2.Y() + q2.Z() * q2.Z())
            );

            init_cam_yaw_ = false;
        }
# endif

        if (view_mode_ == 1) {
            // global track
            ignition::math::Pose3d car_trans_only(
                car_pose.Pos(),
                ignition::math::Quaterniond::Identity
            );

            cam->SetWorldPose(car_trans_only * pending_pose_);

        } else if (view_mode_ == 2) {
            // first, third
            cam->SetWorldPose(car_pose * pending_pose_);
        } else if (view_mode_ == 3) {
            // third
#if !USE_DELTA_TRACKING
              // Low-pass Filter, LPF
              filtered_steer_ = 0.05 * output_steer_ + 0.95 * filtered_steer_;
              ignition::math::Pose3d steer_pose(
                      ignition::math::Vector3d(0, 0, 0),
                      ignition::math::Quaterniond(0, 0, -filtered_steer_ * 1.5)
              );
              cam->SetWorldPose(car_pose * steer_pose * pending_pose_);
# else

              auto& q = car_pose.Rot();
              double car_yaw = std::atan2(
                  2.0 * (q.W() * q.Z() + q.X() * q.Y()),
                  1.0 - 2.0 * (q.Y() * q.Y() + q.Z() * q.Z())
              );


              double raw_delta =  car_yaw - 0 * output_steer_ - cam_yaw_cached_; // desired yaw change to follow the car's steering

              // avoid sudden jump when crossing the -180/180 degree boundary
              if (raw_delta > M_PI) raw_delta -= 2 * M_PI;
              else if (raw_delta < -M_PI) raw_delta += 2 * M_PI;

              double yaw_rate = 50.0; // how fast the camera yaw changes to follow the desired offset
              double yaw_delta = std::clamp(raw_delta, -5.0 * dt, yaw_rate * dt); // limit the yaw change per frame
              cam_yaw_cached_ += yaw_delta; // update the cached camera yaw

              if (cam_yaw_cached_ > M_PI)       cam_yaw_cached_ -= 2 * M_PI;
              else if (cam_yaw_cached_ < -M_PI) cam_yaw_cached_ += 2 * M_PI;

              // pos offset
              ignition::math::Vector3d offset_vec = pending_pose_.Pos();
              double cos_yaw, sin_yaw;
              __builtin_sincos(cam_yaw_cached_, &sin_yaw, &cos_yaw);
              ignition::math::Vector3d rotated_offset(
                  offset_vec.X() * cos_yaw - offset_vec.Y() * sin_yaw,
                  offset_vec.X() * sin_yaw + offset_vec.Y() * cos_yaw,
                  offset_vec.Z()
              );


              ignition::math::Pose3d new_cam_pose(
                    car_pose.Pos() + rotated_offset,
                    ignition::math::Quaterniond(
                      pending_euler_.X(),
                      pending_euler_.Y(),
                      cam_yaw_cached_
                    )
                  );

              cam->SetWorldPose(new_cam_pose);
#endif
        }
    }
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub_;
  rclcpp::Subscription<trace_car::msg::CarInfo>::SharedPtr car_info_sub_;

  std::thread ros_thread_;

  event::ConnectionPtr preRendder_;
  std::mutex pose_mutex_;
  ignition::math::Pose3d pending_pose_;
  ignition::math::Vector3d pending_euler_;

  int view_mode_ = -1;
  int global_pendingFrames_ = 0;
  double output_steer_ = 0.0;
  double filtered_steer_ = 0.0;

#if USE_DELTA_TRACKING

  bool init_cam_yaw_ = false;
  double cam_yaw_cached_ = 0.0;
  common::Time last_update_time_;
#endif

  double cam_table[4][8] = {
      //  x         y         z        roll   pitch     yaw
      { -3.63895, -6.21077,  3.03218,  0.0,   0.487643,  0.980191},  // global
      { -2.5,     -1.6,      2.5,      0.0,   0.7,     0.569},    // global_track
      { -0.064,    0.0,      0.075,    0.0,   0.194,     0.0},       // first
      { -0.7,      0.0,      0.4,      0.0,   0.35,      0.0}        // third
  };
};

// register this plugin with the simulator
GZ_REGISTER_GUI_PLUGIN(CameraViewPlugin)
}
