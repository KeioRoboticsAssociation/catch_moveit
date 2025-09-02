/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, PickNik Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PickNik Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 ***********************************************************************/

/*      Title     : joystick_servo_example.cpp
 *      Project   : moveit_servo
 *      Created   : 08/07/2020
 *      Author    : Adam Pettinger
 */

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/experimental/buffers/intra_process_buffer.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/qos_event.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/utilities.hpp>
#include <thread>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
// We'll just set up parameters here
const std::string JOY_TOPIC = "/joy";
const std::string GUI_LEFT_TOPIC = "/left_arm_realtime_control";
const std::string GUI_RIGHT_TOPIC = "/right_arm_realtime_control";
const std::string TWIST_TOPIC = "/servo_node/delta_twist_cmds";
const std::string JOINT_TOPIC = "/servo_node/delta_joint_cmds";
const std::string HAND_TRAJECTORY_TOPIC = "/hand_controller/joint_trajectory";
const std::string EEF_FRAME_ID = "EndEffector_1";
const std::string BASE_FRAME_ID = "base_link";

// Enums for button names -> axis/button array index
// For XBOX 1 controller
enum Axis
{
  LEFT_STICK_X = 0,
  LEFT_STICK_Y = 1,
  LEFT_TRIGGER = 2,
  RIGHT_STICK_X = 3,
  RIGHT_STICK_Y = 4,
  RIGHT_TRIGGER = 5,
  D_PAD_X = 6,
  D_PAD_Y = 7
};
enum Button
{
  A = 0,
  B = 1,
  X = 2,
  Y = 3,
  LEFT_BUMPER = 4,
  RIGHT_BUMPER = 5,
  CHANGE_VIEW = 6,
  MENU = 7,
  HOME = 8,
  LEFT_STICK_CLICK = 9,
  RIGHT_STICK_CLICK = 10
};

// Some axes have offsets (e.g. the default trigger position is 1.0 not 0)
// This will map the default values for the axes
std::map<Axis, double> AXIS_DEFAULTS = { { LEFT_TRIGGER, 1.0 }, { RIGHT_TRIGGER, 1.0 } };
std::map<Button, double> BUTTON_DEFAULTS;

// To change controls or setup a new controller, all you should to do is change the above enums and the follow 2
// functions
/** \brief // This converts a joystick axes and buttons array to a TwistStamped or JointJog message
 * @param axes The vector of continuous controller joystick axes
 * @param buttons The vector of discrete controller button values
 * @param twist A TwistStamped message to update in prep for publishing
 * @param joint A JointJog message to update in prep for publishing
 * @return return true if you want to publish a Twist, false if you want to publish a JointJog
 */
bool convertJoyToCmd(const std::vector<float>& axes, const std::vector<int>& buttons,
                     std::unique_ptr<geometry_msgs::msg::TwistStamped>& twist,
                     std::unique_ptr<control_msgs::msg::JointJog>& joint)
{
  // Give joint jogging priority because it is only buttons
  // If any joint jog command is requested, we are only publishing joint commands
  // if (axes[D_PAD_Y] != 0.0)
  // {
  //   joint->joint_names.push_back("Slider_1");
  //   joint->velocities.push_back(axes[D_PAD_Y] > 0 ? axes[D_PAD_Y] : 0.0); // D_PAD_Y up for Slider_1 (closes gripper)
  //   joint->joint_names.push_back("Slider_2");
  //   joint->velocities.push_back(axes[D_PAD_Y] < 0 ? axes[D_PAD_Y] : 0.0); // D_PAD_Y down for Slider_2 (opens gripper)
  //   return false;
  // }

  // The bread and butter: map buttons to twist commands
  twist->twist.linear.z = axes[RIGHT_STICK_Y]; 
  twist->twist.linear.y = axes[RIGHT_STICK_X];

  double lin_x_right = -0.5 * (axes[RIGHT_TRIGGER] - AXIS_DEFAULTS.at(RIGHT_TRIGGER));
  double lin_x_left = 0.5 * (axes[LEFT_TRIGGER] - AXIS_DEFAULTS.at(LEFT_TRIGGER));
  twist->twist.linear.x = lin_x_right + lin_x_left;

  twist->twist.angular.y = axes[LEFT_STICK_Y];
  twist->twist.angular.x = axes[LEFT_STICK_X];

  double roll_positive = -1 * (buttons[RIGHT_BUMPER]);
  double roll_negative = buttons[LEFT_BUMPER];
  twist->twist.angular.z = roll_positive + roll_negative;

  return true;
}

/** \brief // This should update the frame_to_publish_ as needed for changing command frame via controller
 * @param frame_name Set the command frame to this
 * @param buttons The vector of discrete controller button values
 */
void updateCmdFrame(std::string& frame_name, const std::vector<int>& buttons)
{
  if (buttons[CHANGE_VIEW] && frame_name == EEF_FRAME_ID)
    frame_name = BASE_FRAME_ID;
  else if (buttons[MENU] && frame_name == BASE_FRAME_ID)
    frame_name = EEF_FRAME_ID;
}

namespace moveit_servo
{

class JoyToServoPub : public rclcpp::Node
{
public:
  JoyToServoPub(const rclcpp::NodeOptions& options)
    : Node("joystick_servo", options), frame_to_publish_(BASE_FRAME_ID)
  {
    // Setup pub/sub
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        JOY_TOPIC, rclcpp::SystemDefaultsQoS(),
        [this](const sensor_msgs::msg::Joy::ConstSharedPtr& msg) { return joyCB(msg); });

    // GUIからのTwistメッセージを受信するサブスクライバーを追加
    gui_left_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        GUI_LEFT_TOPIC, rclcpp::SystemDefaultsQoS(),
        [this](const geometry_msgs::msg::Twist::ConstSharedPtr& msg) { return guiLeftCB(msg); });

    gui_right_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        GUI_RIGHT_TOPIC, rclcpp::SystemDefaultsQoS(),
        [this](const geometry_msgs::msg::Twist::ConstSharedPtr& msg) { return guiRightCB(msg); });

    twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(TWIST_TOPIC, rclcpp::SystemDefaultsQoS());
    joint_pub_ = this->create_publisher<control_msgs::msg::JointJog>(JOINT_TOPIC, rclcpp::SystemDefaultsQoS());
    hand_cmd_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(HAND_TRAJECTORY_TOPIC, rclcpp::SystemDefaultsQoS());

    // Create a service client to start the ServoNode
    servo_start_client_ = this->create_client<std_srvs::srv::Trigger>("/servo_node/start_servo");

    RCLCPP_INFO(get_logger(), "Waiting for Servo service...");
    servo_start_client_->wait_for_service(std::chrono::seconds(5));
    RCLCPP_INFO(get_logger(), "Servo service found. Starting Servo...");
    servo_start_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    RCLCPP_INFO(get_logger(), "Joystick teleop node started.");
  }

  void joyCB(const sensor_msgs::msg::Joy::ConstSharedPtr& msg)
  {
    // Create the messages we might publish
    // ▼▼▼ ハンド制御のロジックをここに追加 ▼▼▼
    // Bボタンで開く、Aボタンで閉じる
    if (msg->buttons[B] == 1 && last_b_button_state_ == 0)
    {
      publishHandCommand(true); // 開く
    }
    else if (msg->buttons[A] == 1 && last_a_button_state_ == 0)
    {
      publishHandCommand(false); // 閉じる
    }
    last_a_button_state_ = msg->buttons[A];
    last_b_button_state_ = msg->buttons[B];
    auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
    auto joint_msg = std::make_unique<control_msgs::msg::JointJog>();

    // This call updates the frame for twist commands
    updateCmdFrame(frame_to_publish_, msg->buttons);

    // Convert the joystick message to Twist or JointJog and publish
    if (convertJoyToCmd(msg->axes, msg->buttons, twist_msg, joint_msg))
    {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Publishing Twist command.");
      // publish the TwistStamped
      twist_msg->header.frame_id = frame_to_publish_;
      twist_msg->header.stamp = this->now();
      twist_pub_->publish(std::move(twist_msg));
    }
    else
    {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Publishing JointJog command.");
      // publish the JointJog
      joint_msg->header.stamp = this->now();
      joint_msg->header.frame_id = BASE_FRAME_ID;
      joint_pub_->publish(std::move(joint_msg));
    }
  }

  // GUIからの左アーム制御コールバック
  void guiLeftCB(const geometry_msgs::msg::Twist::ConstSharedPtr& msg)
  {
    auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
    twist_msg->header.frame_id = "world"; // 左アームの基準フレーム
    twist_msg->header.stamp = this->now();
    twist_msg->twist = *msg;
    
    RCLCPP_DEBUG(get_logger(), "Publishing GUI left arm Twist command.");
    twist_pub_->publish(std::move(twist_msg));
  }

  // GUIからの右アーム制御コールバック
  void guiRightCB(const geometry_msgs::msg::Twist::ConstSharedPtr& msg)
  {
    auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
    twist_msg->header.frame_id = "world"; // 右アームの基準フレーム
    twist_msg->header.stamp = this->now();
    twist_msg->twist = *msg;
    
    RCLCPP_DEBUG(get_logger(), "Publishing GUI right arm Twist command.");
    twist_pub_->publish(std::move(twist_msg));
  }

private:
  void publishHandCommand(bool open)
  {
    auto traj_msg = std::make_unique<trajectory_msgs::msg::JointTrajectory>();
    
    // ▼▼▼ 制御する関節を Slider_1 のみに減らす ▼▼▼
    traj_msg->joint_names = {"Slider_1"};

    trajectory_msgs::msg::JointTrajectoryPoint point;
    if (open)
    {
      RCLCPP_INFO(this->get_logger(), "Opening hand...");
      // ▼▼▼ Slider_1 の目標角度だけを指定 ▼▼▼
      point.positions = {0.024}; // 開いた状態の角度
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Closing hand...");
      // ▼▼▼ Slider_1 の目標角度だけを指定 ▼▼▼
      point.positions = {0.0}; // 閉じた状態の角度
    }
    point.time_from_start = rclcpp::Duration::from_seconds(0.5);

    traj_msg->points.push_back(point);
    hand_cmd_pub_->publish(std::move(traj_msg));
  }
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr gui_left_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr gui_right_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr hand_cmd_pub_;
  int last_a_button_state_ = 0;
  int last_b_button_state_ = 0;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servo_start_client_;

  std::string frame_to_publish_;
};  // class JoyToServoPub

}  // namespace moveit_servo

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<moveit_servo::JoyToServoPub>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
