#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <mutex>
#include <memory>

class DualArmServoController : public rclcpp::Node
{
public:
    DualArmServoController() : Node("dual_arm_servo_controller")
    {
        RCLCPP_INFO(this->get_logger(), "Starting Dual Arm Servo Controller...");
        
        // パラメータ設定
        this->declare_parameter("velocity_scale", 1.0);
        this->declare_parameter("timeout_duration", 0.5);  // 短いタイムアウト
        this->declare_parameter("left_servo_topic", "/left_servo_node/delta_twist_cmds");
        this->declare_parameter("right_servo_topic", "/right_servo_node/delta_twist_cmds");
        this->declare_parameter("min_z_limit", 0.1);  // z座標の下限値（5cm）

        velocity_scale_ = this->get_parameter("velocity_scale").as_double();
        timeout_duration_ = this->get_parameter("timeout_duration").as_double();
        left_servo_topic_ = this->get_parameter("left_servo_topic").as_string();
        right_servo_topic_ = this->get_parameter("right_servo_topic").as_string();
        min_z_limit_ = this->get_parameter("min_z_limit").as_double();

        // TF2設定
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 位置キャッシュの初期化
        left_ee_z_cache_ = min_z_limit_ + 0.1;
        right_ee_z_cache_ = min_z_limit_ + 0.1;
        last_position_update_ = this->now();
        
        // 簡単な状態管理：Poseコマンドが実行中かどうかだけ
        
        // MoveIt Servoへのパブリッシャー設定
        left_servo_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            left_servo_topic_, rclcpp::SystemDefaultsQoS());
        right_servo_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            right_servo_topic_, rclcpp::SystemDefaultsQoS());
        
        // WebアプリからのTwist入力を受信するSubscriber
        left_arm_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/left_arm_realtime_control", 100,
            std::bind(&DualArmServoController::leftArmCallback, this, std::placeholders::_1));
        
        right_arm_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/right_arm_realtime_control", 100,
            std::bind(&DualArmServoController::rightArmCallback, this, std::placeholders::_1));
        
        // シンプルな制御ループタイマー
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50), // 20Hz
            std::bind(&DualArmServoController::controlLoop, this));
        
        // 初期化
        left_arm_target_velocity_ = geometry_msgs::msg::Twist();
        right_arm_target_velocity_ = geometry_msgs::msg::Twist();
        left_arm_last_command_time_ = this->now();
        right_arm_last_command_time_ = this->now();
        
        RCLCPP_INFO(this->get_logger(), "Simple Dual Arm Servo Controller initialized (realtime only)");
        RCLCPP_INFO(this->get_logger(), "Velocity scale: %.3f", velocity_scale_);
    }

private:
    void updateEndEffectorPositions()
    {
        auto current_time = this->now();

        // 100Hz でキャッシュを更新（10ms間隔）
        if ((current_time - last_position_update_).seconds() < 0.01) {
            return;
        }

        try {
            // 左アーム
            geometry_msgs::msg::TransformStamped left_transform =
                tf_buffer_->lookupTransform("world", "left_EndEffector_1", tf2::TimePointZero, tf2::durationFromSec(0.005));
            left_ee_z_cache_ = left_transform.transform.translation.z;
        } catch (const tf2::TransformException& ex) {
            // 前回の値を維持
        }

        try {
            // 右アーム
            geometry_msgs::msg::TransformStamped right_transform =
                tf_buffer_->lookupTransform("world", "right_EndEffector_1", tf2::TimePointZero, tf2::durationFromSec(0.005));
            right_ee_z_cache_ = right_transform.transform.translation.z;
        } catch (const tf2::TransformException& ex) {
            // 前回の値を維持
        }

        last_position_update_ = current_time;
    }

    double getCurrentEndEffectorZ(const std::string& arm_name)
    {
        updateEndEffectorPositions();

        if (arm_name == "left") {
            return left_ee_z_cache_;
        } else if (arm_name == "right") {
            return right_ee_z_cache_;
        }

        return min_z_limit_ + 0.1; // フォールバック
    }

    geometry_msgs::msg::Twist applyZLimit(const geometry_msgs::msg::Twist& input_velocity, const std::string& arm_name)
    {
        geometry_msgs::msg::Twist output_velocity = input_velocity;

        // エンドエフェクタの現在z座標を取得
        double current_z = getCurrentEndEffectorZ(arm_name);

        // より厳しい制限：下限に近づいたら段階的に減速、下限では完全停止
        if (current_z <= min_z_limit_) {
            // 下限以下：下方向移動を完全にブロック
            if (input_velocity.linear.z < 0.0) {
                output_velocity.linear.z = 0.0;
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                                    "%s arm at z-limit (%.3f <= %.3f). Blocking downward movement.",
                                    arm_name.c_str(), current_z, min_z_limit_);
            }
        } else if (current_z <= min_z_limit_ + 0.02) {
            // 下限+2cm以内：下方向速度を大幅に減速
            if (input_velocity.linear.z < 0.0) {
                output_velocity.linear.z = input_velocity.linear.z * 0.1;  // 10%に減速
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                    "%s arm approaching z-limit (%.3f). Reducing downward speed.",
                                    arm_name.c_str(), current_z);
            }
        }

        return output_velocity;
    }

    void leftArmCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(velocity_mutex_);

        // Z制限を適用
        left_arm_target_velocity_ = applyZLimit(*msg, "left");
        left_arm_last_command_time_ = this->now();

        RCLCPP_DEBUG(this->get_logger(), "Left arm velocity: linear(%.3f, %.3f, %.3f), angular(%.3f, %.3f, %.3f)",
                    msg->linear.x, msg->linear.y, msg->linear.z,
                    msg->angular.x, msg->angular.y, msg->angular.z);
    }
    
    void rightArmCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(velocity_mutex_);

        // Z制限を適用
        right_arm_target_velocity_ = applyZLimit(*msg, "right");
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
        geometry_msgs::msg::Twist left_cmd = left_arm_timeout ? 
            geometry_msgs::msg::Twist() : left_arm_target_velocity_;
        publishServoCommand(left_servo_pub_, left_cmd, "world");
        
        // 右アームの制御
        geometry_msgs::msg::Twist right_cmd = right_arm_timeout ? 
            geometry_msgs::msg::Twist() : right_arm_target_velocity_;
        publishServoCommand(right_servo_pub_, right_cmd, "world");
    }
    
    void publishServoCommand(rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher,
                           const geometry_msgs::msg::Twist& velocity,
                           const std::string& frame_id)
    {
        auto twist_stamped = std::make_unique<geometry_msgs::msg::TwistStamped>();
        
        // 速度スケーリングを適用
        twist_stamped->twist.linear.x = velocity.linear.x * velocity_scale_;
        twist_stamped->twist.linear.y = velocity.linear.y * velocity_scale_;
        twist_stamped->twist.linear.z = velocity.linear.z * velocity_scale_;
        
        twist_stamped->twist.angular.x = velocity.angular.x * velocity_scale_;
        twist_stamped->twist.angular.y = velocity.angular.y * velocity_scale_;
        twist_stamped->twist.angular.z = velocity.angular.z * velocity_scale_;
        
        // ヘッダー設定
        twist_stamped->header.stamp = this->now();
        twist_stamped->header.frame_id = frame_id;
        
        // パブリッシュ
        publisher->publish(std::move(twist_stamped));
    }
    
    // メンバ変数（シンプル化）
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr left_servo_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr right_servo_pub_;
    
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr left_arm_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr right_arm_sub_;
    
    rclcpp::TimerBase::SharedPtr control_timer_;
    
    geometry_msgs::msg::Twist left_arm_target_velocity_;
    geometry_msgs::msg::Twist right_arm_target_velocity_;
    
    rclcpp::Time left_arm_last_command_time_;
    rclcpp::Time right_arm_last_command_time_;
    
    std::mutex velocity_mutex_;
    
    // パラメータ
    double velocity_scale_;
    double timeout_duration_;
    std::string left_servo_topic_;
    std::string right_servo_topic_;
    double min_z_limit_;

    // TF2
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // 位置キャッシュ
    double left_ee_z_cache_;
    double right_ee_z_cache_;
    rclcpp::Time last_position_update_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<DualArmServoController>();
    
    RCLCPP_INFO(node->get_logger(), "Starting dual arm servo control...");
    
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
