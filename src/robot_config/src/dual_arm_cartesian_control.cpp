#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/srv/get_cartesian_path.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>

class DualArmCartesianController : public rclcpp::Node
{
public:
    DualArmCartesianController() : Node("dual_arm_cartesian_controller")
    {
        // パラメータ設定
        this->declare_parameter("control_frequency", 20.0);  // 20Hz
        this->declare_parameter("velocity_scale", 0.1);      // 速度スケール
        this->declare_parameter("timeout_duration", 0.5);    // タイムアウト時間（秒）
        
        control_frequency_ = this->get_parameter("control_frequency").as_double();
        velocity_scale_ = this->get_parameter("velocity_scale").as_double();
        timeout_duration_ = this->get_parameter("timeout_duration").as_double();
        
        // MoveIt設定
        left_arm_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "left_arm");
        right_arm_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "right_arm");
        
        // Cartesian制御のパラメータ設定
        left_arm_group_->setMaxVelocityScalingFactor(0.3);
        left_arm_group_->setMaxAccelerationScalingFactor(0.3);
        right_arm_group_->setMaxVelocityScalingFactor(0.3);
        right_arm_group_->setMaxAccelerationScalingFactor(0.3);
        
        // Cartesian制御の精度設定
        left_arm_group_->setPlanningTime(0.1);  // 短い計画時間
        right_arm_group_->setPlanningTime(0.1);
        
        // リアルタイム制御のためのタイマー
        auto timer_period = std::chrono::duration<double>(1.0 / control_frequency_);
        control_timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(timer_period),
            std::bind(&DualArmCartesianController::controlLoop, this));
        
        // Subscriberの設定
        left_arm_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/left_arm_realtime_control", 10,
            std::bind(&DualArmCartesianController::leftArmCallback, this, std::placeholders::_1));
        
        right_arm_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/right_arm_realtime_control", 10,
            std::bind(&DualArmCartesianController::rightArmCallback, this, std::placeholders::_1));
        
        // 初期化
        left_arm_target_velocity_ = geometry_msgs::msg::Twist();
        right_arm_target_velocity_ = geometry_msgs::msg::Twist();
        left_arm_last_command_time_ = this->now();
        right_arm_last_command_time_ = this->now();
        
        RCLCPP_INFO(this->get_logger(), "Dual Arm Cartesian Controller initialized");
        RCLCPP_INFO(this->get_logger(), "Control frequency: %.1f Hz", control_frequency_);
        RCLCPP_INFO(this->get_logger(), "Velocity scale: %.3f", velocity_scale_);
        RCLCPP_INFO(this->get_logger(), "Left arm end effector: %s", left_arm_group_->getEndEffectorLink().c_str());
        RCLCPP_INFO(this->get_logger(), "Right arm end effector: %s", right_arm_group_->getEndEffectorLink().c_str());
    }

private:
    void leftArmCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(velocity_mutex_);
        left_arm_target_velocity_ = *msg;
        left_arm_last_command_time_ = this->now();
        
        RCLCPP_DEBUG(this->get_logger(), "Left arm velocity: linear(%.3f, %.3f, %.3f), angular(%.3f, %.3f, %.3f)",
                    msg->linear.x, msg->linear.y, msg->linear.z,
                    msg->angular.x, msg->angular.y, msg->angular.z);
    }
    
    void rightArmCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(velocity_mutex_);
        right_arm_target_velocity_ = *msg;
        right_arm_last_command_time_ = this->now();
        
        RCLCPP_DEBUG(this->get_logger(), "Right arm velocity: linear(%.3f, %.3f, %.3f), angular(%.3f, %.3f, %.3f)",
                    msg->linear.x, msg->linear.y, msg->linear.z,
                    msg->angular.x, msg->angular.y, msg->angular.z);
    }
    
    void controlLoop()
    {
        std::lock_guard<std::mutex> lock(velocity_mutex_);
        
        auto current_time = this->now();
        
        // コマンドタイムアウトチェック
        bool left_arm_timeout = (current_time - left_arm_last_command_time_).seconds() > timeout_duration_;
        bool right_arm_timeout = (current_time - right_arm_last_command_time_).seconds() > timeout_duration_;
        
        // 左アームの制御
        if (!left_arm_timeout && isVelocityNonZero(left_arm_target_velocity_))
        {
            executeCartesianVelocity(left_arm_group_, left_arm_target_velocity_, "left");
        }
        
        // 右アームの制御
        if (!right_arm_timeout && isVelocityNonZero(right_arm_target_velocity_))
        {
            executeCartesianVelocity(right_arm_group_, right_arm_target_velocity_, "right");
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
    
    void executeCartesianVelocity(std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_group,
                                 const geometry_msgs::msg::Twist& velocity,
                                 const std::string& arm_name)
    {
        try
        {
            // 現在のエンドエフェクタポーズを取得
            geometry_msgs::msg::PoseStamped current_pose = arm_group->getCurrentPose();
            
            // 制御周期での移動量を計算
            double dt = 1.0 / control_frequency_;
            
            // 目標ポーズを計算
            geometry_msgs::msg::Pose target_pose = current_pose.pose;
            
            // 線形速度を適用
            target_pose.position.x += velocity.linear.x * velocity_scale_ * dt;
            target_pose.position.y += velocity.linear.y * velocity_scale_ * dt;
            target_pose.position.z += velocity.linear.z * velocity_scale_ * dt;
            
            // 角速度を適用（ヨー角のみ）
            if (std::abs(velocity.angular.z) > 1e-6)
            {
                tf2::Quaternion q_current, q_delta, q_new;
                tf2::fromMsg(target_pose.orientation, q_current);
                
                double delta_yaw = velocity.angular.z * velocity_scale_ * dt;
                q_delta.setRPY(0, 0, delta_yaw);
                
                q_new = q_current * q_delta;
                q_new.normalize();
                
                target_pose.orientation = tf2::toMsg(q_new);
            }
            
            // 安全範囲チェック
            if (!isPositionSafe(target_pose.position, arm_name))
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                    "%s arm position out of safe range", arm_name.c_str());
                return;
            }
            
            // Cartesian経路を作成
            std::vector<geometry_msgs::msg::Pose> waypoints;
            waypoints.push_back(target_pose);
            
            // Cartesian経路の計画
            moveit_msgs::msg::RobotTrajectory trajectory;
            const double jump_threshold = 0.0;  // ジャンプ検出を無効化
            const double eef_step = 0.01;       // エンドエフェクタステップサイズ
            
            double fraction = arm_group->computeCartesianPath(
                waypoints, eef_step, jump_threshold, trajectory);
            
            if (fraction > 0.5)  // 50%以上の経路が計画できた場合
            {
                // 動作計画の作成と実行
                moveit::planning_interface::MoveGroupInterface::Plan plan;
                plan.trajectory_ = trajectory;
                
                // 非同期実行（ブロッキングしない）
                arm_group->asyncExecute(plan);
                
                RCLCPP_DEBUG(this->get_logger(), "%s arm cartesian path fraction: %.2f", arm_name.c_str(), fraction);
            }
            else
            {
                RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                    "%s arm cartesian planning failed, fraction: %.2f", arm_name.c_str(), fraction);
            }
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                "Error in %s arm cartesian control: %s", arm_name.c_str(), e.what());
        }
    }
    
    bool isPositionSafe(const geometry_msgs::msg::Point& position, const std::string& arm_name)
    {
        // 安全範囲の定義（実際のロボットの作業空間に合わせて調整）
        const double x_min = -1.2, x_max = 1.2;
        const double y_min = -1.2, y_max = 1.2;
        const double z_min = 0.0, z_max = 1.8;
        
        bool safe = (position.x >= x_min && position.x <= x_max &&
                    position.y >= y_min && position.y <= y_max &&
                    position.z >= z_min && position.z <= z_max);
        
        if (!safe)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                "%s arm position (%.3f, %.3f, %.3f) is outside safe bounds", 
                                arm_name.c_str(), position.x, position.y, position.z);
        }
        
        return safe;
    }
    
    // メンバ変数
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> left_arm_group_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> right_arm_group_;
    
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr left_arm_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr right_arm_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    
    geometry_msgs::msg::Twist left_arm_target_velocity_;
    geometry_msgs::msg::Twist right_arm_target_velocity_;
    
    rclcpp::Time left_arm_last_command_time_;
    rclcpp::Time right_arm_last_command_time_;
    
    std::mutex velocity_mutex_;
    
    // パラメータ
    double control_frequency_;
    double velocity_scale_;
    double timeout_duration_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<DualArmCartesianController>();
    
    // MoveItの初期化を待つ
    rclcpp::sleep_for(std::chrono::seconds(2));
    
    RCLCPP_INFO(node->get_logger(), "Starting dual arm cartesian control...");
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}
