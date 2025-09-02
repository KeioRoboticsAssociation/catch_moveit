#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <chrono>
#include <memory>

class SimpleDualArmController : public rclcpp::Node
{
public:
    SimpleDualArmController() : Node("simple_dual_arm_controller")
    {
        RCLCPP_INFO(this->get_logger(), "Starting Simple Dual Arm Controller initialization...");
        
        // パラメータ設定
        this->declare_parameter("control_frequency", 10.0);
        this->declare_parameter("velocity_scale", 0.05);
        this->declare_parameter("timeout_duration", 1.0);
        
        control_frequency_ = this->get_parameter("control_frequency").as_double();
        velocity_scale_ = this->get_parameter("velocity_scale").as_double();
        timeout_duration_ = this->get_parameter("timeout_duration").as_double();
        
        // 初期化フラグ
        left_arm_initialized_ = false;
        right_arm_initialized_ = false;
        
        // 初期化用タイマー
        init_timer_ = this->create_wall_timer(
            std::chrono::seconds(3),
            std::bind(&SimpleDualArmController::initializeMoveGroups, this));
        
        // Subscriberの設定
        left_arm_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/left_arm_realtime_control", 10,
            std::bind(&SimpleDualArmController::leftArmCallback, this, std::placeholders::_1));
        
        right_arm_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/right_arm_realtime_control", 10,
            std::bind(&SimpleDualArmController::rightArmCallback, this, std::placeholders::_1));
        
        // 初期化
        left_arm_target_velocity_ = geometry_msgs::msg::Twist();
        right_arm_target_velocity_ = geometry_msgs::msg::Twist();
        left_arm_last_command_time_ = this->now();
        right_arm_last_command_time_ = this->now();
        
        RCLCPP_INFO(this->get_logger(), "Simple Dual Arm Controller initialized");
        RCLCPP_INFO(this->get_logger(), "Control frequency: %.1f Hz", control_frequency_);
        RCLCPP_INFO(this->get_logger(), "Velocity scale: %.3f", velocity_scale_);
    }

private:
    void initializeMoveGroups()
    {
        try
        {
            RCLCPP_INFO(this->get_logger(), "Initializing MoveIt move groups...");
            
            // MoveIt設定 - より安全な初期化
            if (!left_arm_initialized_)
            {
                try
                {
                    left_arm_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "left_arm");
                    left_arm_group_->setMaxVelocityScalingFactor(0.2);
                    left_arm_group_->setMaxAccelerationScalingFactor(0.2);
                    left_arm_group_->setPlanningTime(1.0);
                    left_arm_initialized_ = true;
                    RCLCPP_INFO(this->get_logger(), "Left arm group initialized successfully");
                }
                catch (const std::exception& e)
                {
                    RCLCPP_WARN(this->get_logger(), "Failed to initialize left arm group: %s", e.what());
                }
            }
            
            if (!right_arm_initialized_)
            {
                try
                {
                    right_arm_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "right_arm");
                    right_arm_group_->setMaxVelocityScalingFactor(0.2);
                    right_arm_group_->setMaxAccelerationScalingFactor(0.2);
                    right_arm_group_->setPlanningTime(1.0);
                    right_arm_initialized_ = true;
                    RCLCPP_INFO(this->get_logger(), "Right arm group initialized successfully");
                }
                catch (const std::exception& e)
                {
                    RCLCPP_WARN(this->get_logger(), "Failed to initialize right arm group: %s", e.what());
                }
            }
            
            // 両方のアームが初期化されたら制御ループを開始
            if (left_arm_initialized_ && right_arm_initialized_)
            {
                RCLCPP_INFO(this->get_logger(), "Both arms initialized, starting control loop");
                
                // 制御タイマーを開始
                auto timer_period = std::chrono::duration<double>(1.0 / control_frequency_);
                control_timer_ = this->create_wall_timer(
                    std::chrono::duration_cast<std::chrono::milliseconds>(timer_period),
                    std::bind(&SimpleDualArmController::controlLoop, this));
                
                // 初期化タイマーを停止
                init_timer_->cancel();
                
                // エンドエフェクタ情報を表示
                RCLCPP_INFO(this->get_logger(), "Left arm end effector: %s", 
                           left_arm_group_->getEndEffectorLink().c_str());
                RCLCPP_INFO(this->get_logger(), "Right arm end effector: %s", 
                           right_arm_group_->getEndEffectorLink().c_str());
            }
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error during MoveIt initialization: %s", e.what());
        }
    }
    
    void leftArmCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        if (!left_arm_initialized_)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                "Left arm not yet initialized, ignoring command");
            return;
        }
        
        std::lock_guard<std::mutex> lock(velocity_mutex_);
        left_arm_target_velocity_ = *msg;
        left_arm_last_command_time_ = this->now();
        
        RCLCPP_DEBUG(this->get_logger(), "Left arm velocity: linear(%.3f, %.3f, %.3f), angular(%.3f, %.3f, %.3f)",
                    msg->linear.x, msg->linear.y, msg->linear.z,
                    msg->angular.x, msg->angular.y, msg->angular.z);
    }
    
    void rightArmCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        if (!right_arm_initialized_)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                "Right arm not yet initialized, ignoring command");
            return;
        }
        
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
        if (left_arm_initialized_ && !left_arm_timeout && isVelocityNonZero(left_arm_target_velocity_))
        {
            executeSimpleMovement(left_arm_group_, left_arm_target_velocity_, "left");
        }
        
        // 右アームの制御
        if (right_arm_initialized_ && !right_arm_timeout && isVelocityNonZero(right_arm_target_velocity_))
        {
            executeSimpleMovement(right_arm_group_, right_arm_target_velocity_, "right");
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
    
    void executeSimpleMovement(std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_group,
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
            
            // 線形速度を適用（小さなステップサイズ）
            target_pose.position.x += velocity.linear.x * velocity_scale_ * dt;
            target_pose.position.y += velocity.linear.y * velocity_scale_ * dt;
            target_pose.position.z += velocity.linear.z * velocity_scale_ * dt;
            
            // 角速度を適用（ヨー角のみ）
            if (std::abs(velocity.angular.z) > 1e-6)
            {
                tf2::Quaternion q_current, q_delta, q_new;
                tf2::fromMsg(target_pose.orientation, q_current);
                
                double delta_yaw = velocity.angular.z * velocity_scale_ * dt * 0.5; // より小さな回転
                q_delta.setRPY(0, 0, delta_yaw);
                
                q_new = q_current * q_delta;
                q_new.normalize();
                
                target_pose.orientation = tf2::toMsg(q_new);
            }
            
            // 安全範囲チェック
            if (!isPositionSafe(target_pose.position, arm_name))
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                    "%s arm position out of safe range", arm_name.c_str());
                return;
            }
            
            // 単純なポーズ目標設定
            arm_group->setPoseTarget(target_pose);
            arm_group->asyncMove(); // 非同期実行
            
            RCLCPP_DEBUG(this->get_logger(), "%s arm moving to new position", arm_name.c_str());
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                "Error in %s arm movement: %s", arm_name.c_str(), e.what());
        }
    }
    
    bool isPositionSafe(const geometry_msgs::msg::Point& position, const std::string& arm_name)
    {
        // より緩い安全範囲
        const double x_min = -2.0, x_max = 2.0;
        const double y_min = -2.0, y_max = 2.0;
        const double z_min = -0.5, z_max = 2.0;
        
        bool safe = (position.x >= x_min && position.x <= x_max &&
                    position.y >= y_min && position.y <= y_max &&
                    position.z >= z_min && position.z <= z_max);
        
        return safe;
    }
    
    // メンバ変数
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> left_arm_group_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> right_arm_group_;
    
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr left_arm_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr right_arm_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr init_timer_;
    
    geometry_msgs::msg::Twist left_arm_target_velocity_;
    geometry_msgs::msg::Twist right_arm_target_velocity_;
    
    rclcpp::Time left_arm_last_command_time_;
    rclcpp::Time right_arm_last_command_time_;
    
    std::mutex velocity_mutex_;
    
    // パラメータ
    double control_frequency_;
    double velocity_scale_;
    double timeout_duration_;
    
    // 初期化フラグ
    bool left_arm_initialized_;
    bool right_arm_initialized_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<SimpleDualArmController>();
    
    RCLCPP_INFO(node->get_logger(), "Starting simple dual arm control...");
    
    try
    {
        rclcpp::spin(node);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(node->get_logger(), "Exception in main: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}
