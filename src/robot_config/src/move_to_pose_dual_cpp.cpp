#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>
#include <memory>
#include <chrono>

class MoveToPoseDualCpp : public rclcpp::Node
{
public:
    MoveToPoseDualCpp()
        : Node("move_to_pose_dual_cpp", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
    {
        // 左アームの初期化スレッド
        left_init_thread_ = std::thread([this]() {
            RCLCPP_INFO(this->get_logger(), "Initializing MoveGroupInterface for left_arm...");
            left_move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(rclcpp::Node::SharedPtr(this, [](rclcpp::Node*){}), "left_arm");
            left_move_group_interface_->setEndEffectorLink("left_EndEffector_1");
            RCLCPP_INFO(this->get_logger(), "MoveGroupInterface for left_arm initialized.");

            // 左アーム用RPYサブスクライバ
            left_rpy_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
                "/left_target_pose_rpy", 10, std::bind(&MoveToPoseDualCpp::left_rpy_callback, this, std::placeholders::_1));

            RCLCPP_INFO(this->get_logger(), "Ready to receive targets on /left_target_pose_rpy");
        });

        // 右アームの初期化スレッド
        right_init_thread_ = std::thread([this]() {
            RCLCPP_INFO(this->get_logger(), "Initializing MoveGroupInterface for right_arm...");
            right_move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(rclcpp::Node::SharedPtr(this, [](rclcpp::Node*){}), "right_arm");
            right_move_group_interface_->setEndEffectorLink("right_EndEffector_1");
            RCLCPP_INFO(this->get_logger(), "MoveGroupInterface for right_arm initialized.");

            // 右アーム用RPYサブスクライバ
            right_rpy_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
                "/right_target_pose_rpy", 10, std::bind(&MoveToPoseDualCpp::right_rpy_callback, this, std::placeholders::_1));

            RCLCPP_INFO(this->get_logger(), "Ready to receive targets on /right_target_pose_rpy");
        });

        // Attach/detach subscribers
        left_attach_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/left_attach_object", 10, std::bind(&MoveToPoseDualCpp::left_attach_callback, this, std::placeholders::_1));
        left_detach_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/left_detach_object", 10, std::bind(&MoveToPoseDualCpp::left_detach_callback, this, std::placeholders::_1));
        right_attach_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/right_attach_object", 10, std::bind(&MoveToPoseDualCpp::right_attach_callback, this, std::placeholders::_1));
        right_detach_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/right_detach_object", 10, std::bind(&MoveToPoseDualCpp::right_detach_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Ready to receive attach/detach commands");
        
        // Initialize PlanningSceneInterface after node initialization
        planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
    }

    ~MoveToPoseDualCpp()
    {
        if (left_init_thread_.joinable())
        {
            left_init_thread_.join();
        }
        if (right_init_thread_.joinable())
        {
            right_init_thread_.join();
        }
    }

private:
    void left_rpy_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received target on /left_target_pose_rpy");
        if (msg->data.size() != 6)
        {
            RCLCPP_ERROR(this->get_logger(), "Expected 6 values for [x, y, z, roll, pitch, yaw], but got %zu", msg->data.size());
            return;
        }

        // RPYからクォータニオンに変換
        tf2::Quaternion q;
        q.setRPY(msg->data[3], msg->data[4], msg->data[5]);

        // PoseStampedメッセージを作成
        geometry_msgs::msg::PoseStamped target_pose;
        target_pose.header.frame_id = "world"; // 基準フレーム
        target_pose.header.stamp = this->get_clock()->now();
        target_pose.pose.position.x = msg->data[0];
        target_pose.pose.position.y = msg->data[1];
        target_pose.pose.position.z = msg->data[2];
        target_pose.pose.orientation = tf2::toMsg(q);

        move_to_pose(left_move_group_interface_, target_pose);
    }

    void right_rpy_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received target on /right_target_pose_rpy");
        if (msg->data.size() != 6)
        {
            RCLCPP_ERROR(this->get_logger(), "Expected 6 values for [x, y, z, roll, pitch, yaw], but got %zu", msg->data.size());
            return;
        }

        // RPYからクォータニオンに変換
        tf2::Quaternion q;
        q.setRPY(msg->data[3], msg->data[4], msg->data[5]);

        // PoseStampedメッセージを作成
        geometry_msgs::msg::PoseStamped target_pose;
        target_pose.header.frame_id = "world"; // 基準フレーム
        target_pose.header.stamp = this->get_clock()->now();
        target_pose.pose.position.x = msg->data[0];
        target_pose.pose.position.y = msg->data[1];
        target_pose.pose.position.z = msg->data[2];
        target_pose.pose.orientation = tf2::toMsg(q);

        move_to_pose(right_move_group_interface_, target_pose);
    }

    void move_to_pose(std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface, const geometry_msgs::msg::PoseStamped& target_pose)
    {
        if (!move_group_interface) {
            RCLCPP_ERROR(this->get_logger(), "MoveGroupInterface not initialized yet.");
            return;
        }

        move_group_interface->setPoseTarget(target_pose);

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group_interface->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success)
        {
            RCLCPP_INFO(this->get_logger(), "Planner found a plan, executing it.");
            move_group_interface->execute(my_plan);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Planning failed.");
        }
    }

    void left_attach_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Attaching object '%s' to left arm", msg->data.c_str());
        attachMeshObject(msg->data, "left");
    }

    void left_detach_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Detaching object '%s' from left arm", msg->data.c_str());
        detachMeshObject(msg->data, "left");
    }

    void right_attach_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Attaching object '%s' to right arm", msg->data.c_str());
        attachMeshObject(msg->data, "right");
    }

    void right_detach_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Detaching object '%s' from right arm", msg->data.c_str());
        detachMeshObject(msg->data, "right");
    }

    void attachMeshObject(const std::string& object_id, const std::string& arm_name)
    {
        if (arm_name == "left" && left_move_group_interface_) {
            // Simple attach without extra touch links to avoid collision issues
            std::vector<std::string> touch_links = {"left_EndEffector_1"};
            left_move_group_interface_->attachObject(object_id, "left_EndEffector_1", touch_links);
            
            // Set planning parameters for attached object
            left_move_group_interface_->setGoalPositionTolerance(0.01); // 1cm tolerance
            left_move_group_interface_->setGoalOrientationTolerance(0.1); // ~6 degree tolerance
            left_move_group_interface_->setPlanningTime(10.0); // More planning time
            
            RCLCPP_INFO(this->get_logger(), "Attached mesh object '%s' to left arm", object_id.c_str());
        } else if (arm_name == "right" && right_move_group_interface_) {
            std::vector<std::string> touch_links = {"right_EndEffector_1"};  
            right_move_group_interface_->attachObject(object_id, "right_EndEffector_1", touch_links);
            
            // Set planning parameters for attached object
            right_move_group_interface_->setGoalPositionTolerance(0.01);
            right_move_group_interface_->setGoalOrientationTolerance(0.1);
            right_move_group_interface_->setPlanningTime(10.0);
            
            RCLCPP_INFO(this->get_logger(), "Attached mesh object '%s' to right arm", object_id.c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Cannot attach object: MoveGroupInterface not initialized for %s arm", arm_name.c_str());
        }
    }

    void detachMeshObject(const std::string& object_id, const std::string& arm_name)
    {
        if (arm_name == "left" && left_move_group_interface_) {
            left_move_group_interface_->detachObject(object_id);
            
            // Reset planning parameters to default values after detach
            left_move_group_interface_->setGoalPositionTolerance(0.001); // Back to 1mm precision
            left_move_group_interface_->setGoalOrientationTolerance(0.01); // Back to ~0.6 degrees
            left_move_group_interface_->setPlanningTime(5.0); // Back to default planning time
            
            // Clear any cached planning scene state
            left_move_group_interface_->clearPoseTargets();
            left_move_group_interface_->stop(); // Stop any ongoing motion
            
            // Small delay to ensure planning scene is updated
            rclcpp::sleep_for(std::chrono::milliseconds(100));
            
            RCLCPP_INFO(this->get_logger(), "Detached mesh object '%s' from left arm and reset planning parameters", object_id.c_str());
        } else if (arm_name == "right" && right_move_group_interface_) {
            right_move_group_interface_->detachObject(object_id);
            
            // Reset planning parameters to default values after detach
            right_move_group_interface_->setGoalPositionTolerance(0.001);
            right_move_group_interface_->setGoalOrientationTolerance(0.01);
            right_move_group_interface_->setPlanningTime(5.0);
            
            // Clear any cached planning scene state
            right_move_group_interface_->clearPoseTargets();
            right_move_group_interface_->stop(); // Stop any ongoing motion
            
            // Small delay to ensure planning scene is updated
            rclcpp::sleep_for(std::chrono::milliseconds(100));
            
            RCLCPP_INFO(this->get_logger(), "Detached mesh object '%s' from right arm and reset planning parameters", object_id.c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Cannot detach object: MoveGroupInterface not initialized for %s arm", arm_name.c_str());
        }
    }

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr left_rpy_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr right_rpy_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr left_attach_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr left_detach_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr right_attach_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr right_detach_sub_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> left_move_group_interface_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> right_move_group_interface_;
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
    std::thread left_init_thread_;
    std::thread right_init_thread_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveToPoseDualCpp>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}