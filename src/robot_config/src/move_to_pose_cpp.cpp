#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>
#include <memory>

class MoveToPoseCpp : public rclcpp::Node
{
public:
    MoveToPoseCpp()
        : Node("move_to_pose_cpp", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
    {
        init_thread_ = std::thread([this]() {
            RCLCPP_INFO(this->get_logger(), "Initializing MoveGroupInterface...");
            auto node_ptr = shared_from_this();
            move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_ptr, "arm");
            move_group_interface_->setEndEffectorLink("EndEffector_1");
            RCLCPP_INFO(this->get_logger(), "MoveGroupInterface initialized.");

            // PoseStamped用
            pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/target_pose", 10, std::bind(&MoveToPoseCpp::pose_callback, this, std::placeholders::_1));
            
            // RPY用
            rpy_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
                "/target_pose_rpy", 10, std::bind(&MoveToPoseCpp::rpy_callback, this, std::placeholders::_1));

            RCLCPP_INFO(this->get_logger(), "Ready to receive targets on /target_pose and /target_pose_rpy");
        });
    }

    ~MoveToPoseCpp()
    {
        if (init_thread_.joinable())
        {
            init_thread_.join();
        }
    }

private:
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received target on /target_pose");
        move_to_pose(*msg);
    }

    void rpy_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received target on /target_pose_rpy");
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

        move_to_pose(target_pose);
    }

    void move_to_pose(const geometry_msgs::msg::PoseStamped& target_pose)
    {
        if (!move_group_interface_) {
            RCLCPP_ERROR(this->get_logger(), "MoveGroupInterface not initialized yet.");
            return;
        }

        move_group_interface_->setPoseTarget(target_pose);

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group_interface_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success)
        {
            RCLCPP_INFO(this->get_logger(), "Planner found a plan, executing it.");
            move_group_interface_->execute(my_plan);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Planning failed.");
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr rpy_sub_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
    std::thread init_thread_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveToPoseCpp>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}