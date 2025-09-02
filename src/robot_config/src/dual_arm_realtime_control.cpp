#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>

class DualArmRealtimeController : public rclcpp::Node
{
public:
    DualArmRealtimeController() : Node("dual_arm_realtime_controller")
    {
        // MoveIt設定
        left_arm_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "left_arm");
        right_arm_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "right_arm");
        
        // 動作パラメータ設定
        left_arm_group_->setMaxVelocityScalingFactor(0.5);
        left_arm_group_->setMaxAccelerationScalingFactor(0.5);
        right_arm_group_->setMaxVelocityScalingFactor(0.5);
        right_arm_group_->setMaxAccelerationScalingFactor(0.5);
        
        // リアルタイム制御のためのタイマー（制御周期: 50Hz）
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&DualArmRealtimeController::controlLoop, this));
        
        // Subscriberの設定
        left_arm_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/left_arm_realtime_control", 10,
            std::bind(&DualArmRealtimeController::leftArmCallback, this, std::placeholders::_1));
        
        right_arm_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/right_arm_realtime_control", 10,
            std::bind(&DualArmRealtimeController::rightArmCallback, this, std::placeholders::_1));
        
        // 初期化
        left_arm_target_velocity_ = geometry_msgs::msg::Twist();
        right_arm_target_velocity_ = geometry_msgs::msg::Twist();
        
        // 現在のポーズを取得
        updateCurrentPoses();
        
        RCLCPP_INFO(this->get_logger(), "Dual Arm Realtime Controller initialized");
        RCLCPP_INFO(this->get_logger(), "Left arm planning frame: %s", left_arm_group_->getPlanningFrame().c_str());
        RCLCPP_INFO(this->get_logger(), "Right arm planning frame: %s", right_arm_group_->getPlanningFrame().c_str());
        RCLCPP_INFO(this->get_logger(), "Left arm end effector: %s", left_arm_group_->getEndEffectorLink().c_str());
        RCLCPP_INFO(this->get_logger(), "Right arm end effector: %s", right_arm_group_->getEndEffectorLink().c_str());
    }

private:
    void leftArmCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(velocity_mutex_);
        left_arm_target_velocity_ = *msg;
        left_arm_last_command_time_ = this->now();
        
        RCLCPP_DEBUG(this->get_logger(), "Left arm velocity command: linear(%.3f, %.3f, %.3f), angular(%.3f, %.3f, %.3f)",
                    msg->linear.x, msg->linear.y, msg->linear.z,
                    msg->angular.x, msg->angular.y, msg->angular.z);
    }
    
    void rightArmCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(velocity_mutex_);
        right_arm_target_velocity_ = *msg;
        right_arm_last_command_time_ = this->now();
        
        RCLCPP_DEBUG(this->get_logger(), "Right arm velocity command: linear(%.3f, %.3f, %.3f), angular(%.3f, %.3f, %.3f)",
                    msg->linear.x, msg->linear.y, msg->linear.z,
                    msg->angular.x, msg->angular.y, msg->angular.z);
    }
    
    void controlLoop()
    {
        std::lock_guard<std::mutex> lock(velocity_mutex_);
        
        auto current_time = this->now();
        
        // コマンドタイムアウトチェック（1秒間コマンドがない場合は停止）
        bool left_arm_timeout = (current_time - left_arm_last_command_time_).seconds() > 1.0;
        bool right_arm_timeout = (current_time - right_arm_last_command_time_).seconds() > 1.0;
        
        // 左アームの制御
        if (!left_arm_timeout && isVelocityNonZero(left_arm_target_velocity_))
        {
            moveArmWithVelocity(left_arm_group_, left_arm_current_pose_, left_arm_target_velocity_, "left");
        }
        
        // 右アームの制御
        if (!right_arm_timeout && isVelocityNonZero(right_arm_target_velocity_))
        {
            moveArmWithVelocity(right_arm_group_, right_arm_current_pose_, right_arm_target_velocity_, "right");
        }
    }
    
    bool isVelocityNonZero(const geometry_msgs::msg::Twist& velocity)
    {
        const double threshold = 1e-6;
        return (std::abs(velocity.linear.x) > threshold ||
                std::abs(velocity.linear.y) > threshold ||
                std::abs(velocity.linear.z) > threshold ||
                std::abs(velocity.angular.x) > threshold ||
                std::abs(velocity.angular.y) > threshold ||
                std::abs(velocity.angular.z) > threshold);
    }
    
    void moveArmWithVelocity(std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_group,
                           geometry_msgs::msg::Pose& current_pose,
                           const geometry_msgs::msg::Twist& velocity,
                           const std::string& arm_name)
    {
        try
        {
            // 制御周期での移動量を計算（dt = 0.02秒）
            double dt = 0.02;
            
            // 線形速度を位置に積分
            current_pose.position.x += velocity.linear.x * dt;
            current_pose.position.y += velocity.linear.y * dt;
            current_pose.position.z += velocity.linear.z * dt;
            
            // 角速度を姿勢に積分
            if (std::abs(velocity.angular.z) > 1e-6)
            {
                // ヨー角のみの回転を適用
                tf2::Quaternion q_current, q_delta, q_new;
                tf2::fromMsg(current_pose.orientation, q_current);
                
                // ヨー角の変化量
                double delta_yaw = velocity.angular.z * dt;
                q_delta.setRPY(0, 0, delta_yaw);
                
                // 新しい姿勢を計算
                q_new = q_current * q_delta;
                q_new.normalize();
                
                current_pose.orientation = tf2::toMsg(q_new);
            }
            
            // 安全範囲チェック
            if (!isPositionSafe(current_pose.position, arm_name))
            {
                RCLCPP_WARN(this->get_logger(), "%s arm position out of safe range, stopping movement", arm_name.c_str());
                return;
            }
            
            // MoveItでポーズ目標を設定
            arm_group->setPoseTarget(current_pose);
            
            // 動作計画と実行
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            bool success = (arm_group->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            
            if (success)
            {
                arm_group->asyncExecute(plan);
            }
            else
            {
                RCLCPP_DEBUG(this->get_logger(), "%s arm planning failed", arm_name.c_str());
                // プランニングが失敗した場合は現在のポーズを再取得
                updateCurrentPose(arm_group, current_pose);
            }
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error in %s arm movement: %s", arm_name.c_str(), e.what());
            updateCurrentPose(arm_group, current_pose);
        }
    }
    
    bool isPositionSafe(const geometry_msgs::msg::Point& position, const std::string& arm_name)
    {
        // 安全範囲の定義（実際のロボットの作業空間に合わせて調整）
        const double x_min = -1.0, x_max = 1.0;
        const double y_min = -1.0, y_max = 1.0;
        const double z_min = 0.0, z_max = 1.5;
        
        return (position.x >= x_min && position.x <= x_max &&
                position.y >= y_min && position.y <= y_max &&
                position.z >= z_min && position.z <= z_max);
    }
    
    void updateCurrentPoses()
    {
        updateCurrentPose(left_arm_group_, left_arm_current_pose_);
        updateCurrentPose(right_arm_group_, right_arm_current_pose_);
    }
    
    void updateCurrentPose(std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_group,
                          geometry_msgs::msg::Pose& current_pose)
    {
        try
        {
            current_pose = arm_group->getCurrentPose().pose;
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to get current pose: %s", e.what());
        }
    }
    
    // メンバ変数
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> left_arm_group_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> right_arm_group_;
    
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr left_arm_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr right_arm_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    
    geometry_msgs::msg::Twist left_arm_target_velocity_;
    geometry_msgs::msg::Twist right_arm_target_velocity_;
    
    geometry_msgs::msg::Pose left_arm_current_pose_;
    geometry_msgs::msg::Pose right_arm_current_pose_;
    
    rclcpp::Time left_arm_last_command_time_;
    rclcpp::Time right_arm_last_command_time_;
    
    std::mutex velocity_mutex_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<DualArmRealtimeController>();
    
    // MoveItの初期化を待つ
    rclcpp::sleep_for(std::chrono::seconds(2));
    
    RCLCPP_INFO(node->get_logger(), "Starting dual arm realtime control...");
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}
