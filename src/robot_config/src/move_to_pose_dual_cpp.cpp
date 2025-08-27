#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/collision_detection/collision_common.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>
#include <memory>
#include <chrono>
#include <set>
#include <functional>

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
            left_move_group_interface_->setPlanningPipelineId("ompl");
            left_move_group_interface_->setPlannerId("RRTConnect");
            RCLCPP_INFO(this->get_logger(), "MoveGroupInterface for left_arm initialized.");

            // 左ハンド（グリッパー）の初期化
            RCLCPP_INFO(this->get_logger(), "Initializing MoveGroupInterface for left_hand...");
            left_hand_move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(rclcpp::Node::SharedPtr(this, [](rclcpp::Node*){}), "left_hand");
            left_hand_move_group_interface_->setPlanningPipelineId("pilz_industrial_motion_planner");
            left_hand_move_group_interface_->setPlannerId("PTP");
            RCLCPP_INFO(this->get_logger(), "MoveGroupInterface for left_hand initialized.");

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
            right_move_group_interface_->setPlanningPipelineId("ompl");
            right_move_group_interface_->setPlannerId("RRTConnect");
            RCLCPP_INFO(this->get_logger(), "MoveGroupInterface for right_arm initialized.");

            // 右ハンド（グリッパー）の初期化
            RCLCPP_INFO(this->get_logger(), "Initializing MoveGroupInterface for right_hand...");
            right_hand_move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(rclcpp::Node::SharedPtr(this, [](rclcpp::Node*){}), "right_hand");
            right_hand_move_group_interface_->setPlanningPipelineId("pilz_industrial_motion_planner");
            right_hand_move_group_interface_->setPlannerId("PTP");
            RCLCPP_INFO(this->get_logger(), "MoveGroupInterface for right_hand initialized.");

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

        // Arm up/down subscribers
        left_arm_up_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/left_arm_up", 10, std::bind(&MoveToPoseDualCpp::left_arm_up_callback, this, std::placeholders::_1));
        left_arm_down_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/left_arm_down", 10, std::bind(&MoveToPoseDualCpp::left_arm_down_callback, this, std::placeholders::_1));
        right_arm_up_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/right_arm_up", 10, std::bind(&MoveToPoseDualCpp::right_arm_up_callback, this, std::placeholders::_1));
        right_arm_down_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/right_arm_down", 10, std::bind(&MoveToPoseDualCpp::right_arm_down_callback, this, std::placeholders::_1));

        // Gripper open/close subscribers
        left_arm_open_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/left_arm_open", 10, std::bind(&MoveToPoseDualCpp::left_arm_open_callback, this, std::placeholders::_1));
        left_arm_close_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/left_arm_close", 10, std::bind(&MoveToPoseDualCpp::left_arm_close_callback, this, std::placeholders::_1));
        right_arm_open_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/right_arm_open", 10, std::bind(&MoveToPoseDualCpp::right_arm_open_callback, this, std::placeholders::_1));
        right_arm_close_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/right_arm_close", 10, std::bind(&MoveToPoseDualCpp::right_arm_close_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Ready to receive attach/detach, arm up/down, and gripper open/close commands");
        
        // Initialize PlanningSceneInterface after node initialization
        planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
        
        // Subscribe to planning scene for dynamic object tracking
        planning_scene_sub_ = this->create_subscription<moveit_msgs::msg::PlanningScene>(
            "/planning_scene", 10, std::bind(&MoveToPoseDualCpp::planning_scene_callback, this, std::placeholders::_1));
        
        // Subscribers for coordinate dictionary registration
        register_coordinate_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/register_coordinate", 10, std::bind(&MoveToPoseDualCpp::register_coordinate_callback, this, std::placeholders::_1));
        
        // Initialize object positions by getting current planning scene
        std::thread([this]() {
            // Wait a bit for initialization
            std::this_thread::sleep_for(std::chrono::seconds(2));
            initialize_object_positions();
        }).detach();
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

        // Store the received pose for arm up/down functionality
        last_left_pose_rpy_ = msg->data;
        left_pose_received_ = true;

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

        // Store the received pose for arm up/down functionality
        last_right_pose_rpy_ = msg->data;
        right_pose_received_ = true;

        move_to_pose(right_move_group_interface_, target_pose);
    }

    void move_to_pose(std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface, const geometry_msgs::msg::PoseStamped& target_pose)
    {
        if (!move_group_interface) {
            RCLCPP_ERROR(this->get_logger(), "MoveGroupInterface not initialized yet.");
            return;
        }

        move_group_interface->setPoseTarget(target_pose);
        
        // プランナーをOMPLのRRTConnectに指定
        move_group_interface->setPlanningPipelineId("ompl");
        move_group_interface->setPlannerId("RRTConnect");

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
            // Include all gripper links as touch links to avoid collision with attached object
            std::vector<std::string> touch_links = {
                "left_EndEffector_1", 
                "left_left_finger_1", 
                "left_right_finger_1"
            };
            left_move_group_interface_->attachObject(object_id, "left_EndEffector_1", touch_links);
            
            // Set planning parameters for attached object
            left_move_group_interface_->setGoalPositionTolerance(0.0001); // 1cm tolerance
            left_move_group_interface_->setGoalOrientationTolerance(0.001); // ~6 degree tolerance
            left_move_group_interface_->setPlanningTime(10.0); // More planning time
            
            RCLCPP_INFO(this->get_logger(), "Attached mesh object '%s' to left arm with collision-aware touch links", object_id.c_str());
        } else if (arm_name == "right" && right_move_group_interface_) {
            std::vector<std::string> touch_links = {
                "right_EndEffector_1", 
                "right_left_finger_1", 
                "right_right_finger_1"
            };
            right_move_group_interface_->attachObject(object_id, "right_EndEffector_1", touch_links);
            
            // Set planning parameters for attached object
            right_move_group_interface_->setGoalPositionTolerance(0.0001);
            right_move_group_interface_->setGoalOrientationTolerance(0.001);
            right_move_group_interface_->setPlanningTime(10.0);
            
            RCLCPP_INFO(this->get_logger(), "Attached mesh object '%s' to right arm", object_id.c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Cannot attach object: MoveGroupInterface not initialized for %s arm", arm_name.c_str());
        }
    }

    void detachMeshObject(const std::string& object_id, const std::string& arm_name)
    {
        if (arm_name == "left" && left_move_group_interface_) {
            // First detach from robot
            left_move_group_interface_->detachObject(object_id);
            
            // Then remove completely from world using PlanningSceneInterface
            if (planning_scene_interface_) {
                std::vector<std::string> object_ids = {object_id};
                planning_scene_interface_->removeCollisionObjects(object_ids);
                RCLCPP_INFO(this->get_logger(), "Removed mesh object '%s' from world", object_id.c_str());
            }
            
            // Reset planning parameters to default values after detach
            left_move_group_interface_->setGoalPositionTolerance(0.001); // Back to 1mm precision
            left_move_group_interface_->setGoalOrientationTolerance(0.01); // Back to ~0.6 degrees
            left_move_group_interface_->setPlanningTime(5.0); // Back to default planning time
            
            // Clear any cached planning scene state
            left_move_group_interface_->clearPoseTargets();
            left_move_group_interface_->stop(); // Stop any ongoing motion
            
            // Small delay to ensure planning scene is updated
            rclcpp::sleep_for(std::chrono::milliseconds(200));
            
            RCLCPP_INFO(this->get_logger(), "Detached and removed mesh object '%s' from left arm", object_id.c_str());
        } else if (arm_name == "right" && right_move_group_interface_) {
            // First detach from robot
            right_move_group_interface_->detachObject(object_id);
            
            // Then remove completely from world using PlanningSceneInterface
            if (planning_scene_interface_) {
                std::vector<std::string> object_ids = {object_id};
                planning_scene_interface_->removeCollisionObjects(object_ids);
                RCLCPP_INFO(this->get_logger(), "Removed mesh object '%s' from world", object_id.c_str());
            }
            
            // Reset planning parameters to default values after detach
            right_move_group_interface_->setGoalPositionTolerance(0.001);
            right_move_group_interface_->setGoalOrientationTolerance(0.01);
            right_move_group_interface_->setPlanningTime(5.0);
            
            // Clear any cached planning scene state
            right_move_group_interface_->clearPoseTargets();
            right_move_group_interface_->stop(); // Stop any ongoing motion
            
            // Small delay to ensure planning scene is updated
            rclcpp::sleep_for(std::chrono::milliseconds(200));
            
            RCLCPP_INFO(this->get_logger(), "Detached and removed mesh object '%s' from right arm", object_id.c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Cannot detach object: MoveGroupInterface not initialized for %s arm", arm_name.c_str());
        }
    }

    void left_arm_up_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Left arm up command received");
        if (!left_pose_received_) {
            RCLCPP_ERROR(this->get_logger(), "No left pose received yet. Cannot execute arm_up.");
            return;
        }
        move_arm_z("left", arm_up_z_value_);
    }

    void left_arm_down_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Left arm down command received");
        if (!left_pose_received_) {
            RCLCPP_ERROR(this->get_logger(), "No left pose received yet. Cannot execute arm_down.");
            return;
        }
        move_arm_z("left", arm_down_z_value_);
    }

    void right_arm_up_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Right arm up command received");
        if (!right_pose_received_) {
            RCLCPP_ERROR(this->get_logger(), "No right pose received yet. Cannot execute arm_up.");
            return;
        }
        move_arm_z("right", arm_up_z_value_);
    }

    void right_arm_down_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Right arm down command received");
        if (!right_pose_received_) {
            RCLCPP_ERROR(this->get_logger(), "No right pose received yet. Cannot execute arm_down.");
            return;
        }
        move_arm_z("right", arm_down_z_value_);
    }

    void move_arm_z(const std::string& arm_name, double target_z)
    {
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface;
        std::vector<double> current_pose_rpy;

        if (arm_name == "left") {
            move_group_interface = left_move_group_interface_;
            current_pose_rpy = last_left_pose_rpy_;
        } else if (arm_name == "right") {
            move_group_interface = right_move_group_interface_;
            current_pose_rpy = last_right_pose_rpy_;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Invalid arm name: %s", arm_name.c_str());
            return;
        }

        if (!move_group_interface) {
            RCLCPP_ERROR(this->get_logger(), "MoveGroupInterface not initialized for %s arm", arm_name.c_str());
            return;
        }

        // Create target pose with modified z-coordinate
        geometry_msgs::msg::PoseStamped target_pose;
        target_pose.header.frame_id = "world";
        target_pose.header.stamp = this->get_clock()->now();
        target_pose.pose.position.x = current_pose_rpy[0];
        target_pose.pose.position.y = current_pose_rpy[1];
        target_pose.pose.position.z = target_z;  // Set absolute z value

        // Convert RPY to quaternion
        tf2::Quaternion q;
        q.setRPY(current_pose_rpy[3], current_pose_rpy[4], current_pose_rpy[5]);
        target_pose.pose.orientation = tf2::toMsg(q);

        // Use Pilz LIN planner for straight line motion
        move_group_interface->setPlanningPipelineId("pilz_industrial_motion_planner");
        move_group_interface->setPlannerId("LIN");
        move_group_interface->setPoseTarget(target_pose);

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group_interface->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success) {
            RCLCPP_INFO(this->get_logger(), "Pilz LIN planner found a plan for %s arm z-movement, executing it.", arm_name.c_str());
            move_group_interface->execute(my_plan);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Pilz LIN planning failed for %s arm z-movement.", arm_name.c_str());
        }
    }

    void left_arm_open_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Left arm open command received");
        set_gripper_position("left", 0.0, 0.0);
        
        // Detach objects when gripper opens
        detach_attached_objects("left");
    }

    void left_arm_close_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Left arm close command received");
        
        // Set gripper position with completion callback
        set_gripper_position("left", 0.026, -0.026, [this]() {
            RCLCPP_INFO(this->get_logger(), "Gripper movement completed, checking for objects to attach");
            check_for_grasp_attachment("left");
        });
    }

    void right_arm_open_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Right arm open command received");
        set_gripper_position("right", 0.0, 0.0);
        
        // Detach objects when gripper opens
        detach_attached_objects("right");
    }

    void right_arm_close_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Right arm close command received");
        
        // Set gripper position with completion callback
        set_gripper_position("right", 0.026, -0.026, [this]() {
            RCLCPP_INFO(this->get_logger(), "Gripper movement completed, checking for objects to attach");
            check_for_grasp_attachment("right");
        });
    }

    void set_gripper_position(const std::string& arm_name, double slider1_value, double slider2_value, std::function<void()> completion_callback = nullptr)
    {
        RCLCPP_INFO(this->get_logger(), "Setting %s gripper position directly: slider1=%f, slider2=%f", 
                   arm_name.c_str(), slider1_value, slider2_value);
        
        // Create joint trajectory message
        auto trajectory_msg = trajectory_msgs::msg::JointTrajectory();
        
        // Set joint names
        trajectory_msg.joint_names = {arm_name + "_Slider_1", arm_name + "_Slider_2"};
        
        // Create trajectory point
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = {slider1_value, slider2_value};
        point.time_from_start = rclcpp::Duration::from_seconds(0.1);
        
        trajectory_msg.points.push_back(point);
        
        // Publish to joint trajectory controller
        std::string controller_topic = "/" + arm_name + "_hand_controller/joint_trajectory";
        
        // Create publisher (we'll create it once and reuse)
        if (arm_name == "left") {
            if (!left_gripper_pub_) {
                left_gripper_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(controller_topic, 10);
            }
            left_gripper_pub_->publish(trajectory_msg);
        } else if (arm_name == "right") {
            if (!right_gripper_pub_) {
                right_gripper_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(controller_topic, 10);
            }
            right_gripper_pub_->publish(trajectory_msg);
        }
        
        RCLCPP_INFO(this->get_logger(), "Published joint trajectory to %s", controller_topic.c_str());
        
        // Execute callback after trajectory duration (small buffer for completion)
        if (completion_callback) {
            std::thread([completion_callback]() {
                std::this_thread::sleep_for(std::chrono::milliseconds(200)); // 0.2 seconds
                completion_callback();
            }).detach();
        }
    }

    void initialize_object_positions()
    {
        RCLCPP_INFO(this->get_logger(), "Initializing object positions from planning scene");
        
        // Create service client for getting planning scene
        auto client = this->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");
        
        if (!client->wait_for_service(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "GetPlanningScene service not available");
            return;
        }
        
        auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
        request->components.components = moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_NAMES |
                                        moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY;
        
        auto future = client->async_send_request(request);
        
        if (future.wait_for(std::chrono::seconds(5)) == std::future_status::ready) {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "Got planning scene with %zu collision objects", 
                       response->scene.world.collision_objects.size());
            
            std::lock_guard<std::mutex> lock(object_mutex_);
            
            for (const auto& collision_obj : response->scene.world.collision_objects) {
                RCLCPP_INFO(this->get_logger(), "Found object: %s", collision_obj.id.c_str());
                
                if (!collision_obj.primitive_poses.empty()) {
                    std::array<double, 3> obj_pos = {
                        collision_obj.primitive_poses[0].position.x,
                        collision_obj.primitive_poses[0].position.y,
                        collision_obj.primitive_poses[0].position.z
                    };
                    object_mesh_positions_[collision_obj.id] = obj_pos;
                    RCLCPP_INFO(this->get_logger(), "Registered object '%s' at (%.3f, %.3f, %.3f)", 
                               collision_obj.id.c_str(), obj_pos[0], obj_pos[1], obj_pos[2]);
                } else if (!collision_obj.mesh_poses.empty()) {
                    std::array<double, 3> obj_pos = {
                        collision_obj.mesh_poses[0].position.x,
                        collision_obj.mesh_poses[0].position.y,
                        collision_obj.mesh_poses[0].position.z
                    };
                    object_mesh_positions_[collision_obj.id] = obj_pos;
                    RCLCPP_INFO(this->get_logger(), "Registered mesh object '%s' at (%.3f, %.3f, %.3f)", 
                               collision_obj.id.c_str(), obj_pos[0], obj_pos[1], obj_pos[2]);
                }
            }
            
            RCLCPP_INFO(this->get_logger(), "Object positions initialized: %zu objects registered", object_mesh_positions_.size());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to get planning scene");
        }
    }
    
    void planning_scene_callback(const moveit_msgs::msg::PlanningScene::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Planning scene callback triggered with %zu collision objects", msg->world.collision_objects.size());
        
        // Update object mesh positions from planning scene
        std::lock_guard<std::mutex> lock(object_mutex_);
        
        for (const auto& collision_obj : msg->world.collision_objects) {
            RCLCPP_INFO(this->get_logger(), "Processing object: %s", collision_obj.id.c_str());
            
            if (!collision_obj.primitive_poses.empty()) {
                std::array<double, 3> obj_pos = {
                    collision_obj.primitive_poses[0].position.x,
                    collision_obj.primitive_poses[0].position.y,
                    collision_obj.primitive_poses[0].position.z
                };
                object_mesh_positions_[collision_obj.id] = obj_pos;
            } else if (!collision_obj.mesh_poses.empty()) {
                std::array<double, 3> obj_pos = {
                    collision_obj.mesh_poses[0].position.x,
                    collision_obj.mesh_poses[0].position.y,
                    collision_obj.mesh_poses[0].position.z
                };
                object_mesh_positions_[collision_obj.id] = obj_pos;
            }
        }
    }
    
    void register_coordinate_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        // Format: [key_x, key_y, key_z, obj1_x, obj1_y, obj1_z, obj2_x, obj2_y, obj2_z]
        if (msg->data.size() >= 9) {
            std::array<double, 3> key_pos = {msg->data[0], msg->data[1], msg->data[2]};
            std::array<double, 3> obj1_pos = {msg->data[3], msg->data[4], msg->data[5]};
            std::array<double, 3> obj2_pos = {msg->data[6], msg->data[7], msg->data[8]};
            
            std::lock_guard<std::mutex> lock(coordinate_dict_mutex_);
            grasp_coordinate_dictionary_[key_pos] = std::make_pair(obj1_pos, obj2_pos);
            
            RCLCPP_INFO(this->get_logger(), "Registered grasp coordinate: key(%.3f, %.3f, %.3f) -> obj1(%.3f, %.3f, %.3f) + obj2(%.3f, %.3f, %.3f)", 
                       key_pos[0], key_pos[1], key_pos[2], obj1_pos[0], obj1_pos[1], obj1_pos[2], obj2_pos[0], obj2_pos[1], obj2_pos[2]);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Invalid coordinate registration: expected 9 values [key_x,y,z, obj1_x,y,z, obj2_x,y,z], got %zu", msg->data.size());
        }
    }
    
    double calculate3DDistance(const std::array<double, 3>& pos1, const std::array<double, 3>& pos2)
    {
        double dx = pos1[0] - pos2[0];
        double dy = pos1[1] - pos2[1];
        double dz = pos1[2] - pos2[2];
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }
    
    std::string find_nearest_object_by_position(const std::array<double, 3>& target_pos, double tolerance = 0.05)
    {
        std::lock_guard<std::mutex> lock(object_mutex_);
        
        RCLCPP_INFO(this->get_logger(), "Searching for object near position (%.3f, %.3f, %.3f)", target_pos[0], target_pos[1], target_pos[2]);
        RCLCPP_INFO(this->get_logger(), "Object mesh positions has %zu entries", object_mesh_positions_.size());
        
        std::string nearest_object = "";
        double min_distance = tolerance;
        
        for (const auto& obj_entry : object_mesh_positions_) {
            const std::string& obj_name = obj_entry.first;
            const std::array<double, 3>& obj_pos = obj_entry.second;
            
            double distance = calculate3DDistance(target_pos, obj_pos);
            RCLCPP_INFO(this->get_logger(), "Object '%s' at (%.3f, %.3f, %.3f), distance: %.3f", 
                       obj_name.c_str(), obj_pos[0], obj_pos[1], obj_pos[2], distance);
            
            if (distance < min_distance) {
                min_distance = distance;
                nearest_object = obj_name;
                RCLCPP_INFO(this->get_logger(), "New nearest object: %s (distance: %.3f)", obj_name.c_str(), distance);
            }
        }
        
        if (nearest_object.empty()) {
            RCLCPP_WARN(this->get_logger(), "No object found within tolerance %.3f", tolerance);
        }
        
        return nearest_object;
    }
    
    void check_for_grasp_attachment(const std::string& arm_name)
    {
        RCLCPP_INFO(this->get_logger(), "Checking for EndEffector_1 collision-based attachment for %s arm", arm_name.c_str());
        
        // Check collision with gripper links (fingers and end effector)
        std::vector<std::string> gripper_links = {
            arm_name + "_EndEffector_1",
            arm_name + "_left_finger_1", 
            arm_name + "_right_finger_1"
        };
        
        auto move_group = (arm_name == "left") ? left_move_group_interface_ : right_move_group_interface_;
        if (!move_group) {
            RCLCPP_WARN(this->get_logger(), "Move group not initialized for %s", arm_name.c_str());
            return;
        }
        
        // Get current robot state
        moveit::core::RobotStatePtr current_state;
        try {
            current_state = move_group->getCurrentState(10.0);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get current state: %s", e.what());
            return;
        }
        
        if (!current_state) {
            RCLCPP_ERROR(this->get_logger(), "Current state is null");
            return;
        }
        
        // Create a planning scene for collision checking
        planning_scene::PlanningScenePtr planning_scene_ptr = std::make_shared<planning_scene::PlanningScene>(current_state->getRobotModel());
        planning_scene_ptr->getCurrentStateNonConst() = *current_state;
        
        // Get collision objects and add them to the planning scene
        std::map<std::string, moveit_msgs::msg::CollisionObject> collision_objects_map = planning_scene_interface_->getObjects();
        
        for (const auto& obj_pair : collision_objects_map) {
            planning_scene_ptr->processCollisionObjectMsg(obj_pair.second);
        }
        
        std::vector<std::string> objects_to_attach;
        
        // Check each object for collision with EndEffector_1
        for (const auto& obj_pair : collision_objects_map) {
            const std::string& obj_name = obj_pair.first;
            
            // Skip large objects like field, walls
            if (obj_name.find("field") != std::string::npos || 
                obj_name.find("wall") != std::string::npos ||
                obj_name.find("box_from_8points") != std::string::npos) {
                continue;
            }
            
            // Check collision between EndEffector_1 and this object
            collision_detection::CollisionRequest collision_request;
            collision_detection::CollisionResult collision_result;
            
            collision_request.contacts = true;
            collision_request.max_contacts = 100;
            collision_request.max_contacts_per_pair = 10;
            collision_request.verbose = false;
            
            planning_scene_ptr->checkCollision(collision_request, collision_result, *current_state);
            
            if (collision_result.collision) {
                // Check if EndEffector_1 is involved in any collision
                for (const auto& contact_pair : collision_result.contacts) {
                    const std::string& link1 = contact_pair.first.first;
                    const std::string& link2 = contact_pair.first.second;
                    
                    // Check if any gripper link is colliding with this object
                    bool gripper_collision = false;
                    std::string colliding_link;
                    
                    for (const std::string& gripper_link : gripper_links) {
                        if ((link1 == gripper_link && link2 == obj_name) ||
                            (link2 == gripper_link && link1 == obj_name)) {
                            gripper_collision = true;
                            colliding_link = gripper_link;
                            break;
                        }
                    }
                    
                    if (gripper_collision) {
                        RCLCPP_INFO(this->get_logger(), "Collision detected between %s and %s", colliding_link.c_str(), obj_name.c_str());
                        objects_to_attach.push_back(obj_name);
                        break;
                    }
                }
            }
        }
        
        // Attach only objects that are actually colliding with EndEffector_1
        for (const std::string& obj_name : objects_to_attach) {
            try {
                attachMeshObject(obj_name, arm_name);
                RCLCPP_INFO(this->get_logger(), "Auto-attached %s to %s EndEffector_1 based on collision", obj_name.c_str(), arm_name.c_str());
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "Failed to attach %s: %s", obj_name.c_str(), e.what());
            }
        }
        
        if (objects_to_attach.empty()) {
            RCLCPP_WARN(this->get_logger(), "No objects found colliding with %s gripper links", arm_name.c_str());
        }
    }
    
    void detach_attached_objects(const std::string& arm_name)
    {
        // Get list of currently attached objects and detach them
        RCLCPP_INFO(this->get_logger(), "Detaching objects from %s arm (gripper opened)", arm_name.c_str());
        
        // For now, we'll try to detach common object names
        for (int i = 1; i <= 20; i++) {
            std::string obj_name = "object_" + std::to_string(i);
            try {
                detachMeshObject(obj_name, arm_name);
            } catch (...) {
                // Ignore errors for objects that aren't attached
            }
        }
    }

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr left_rpy_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr right_rpy_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr left_attach_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr left_detach_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr right_attach_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr right_detach_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr left_arm_up_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr left_arm_down_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr right_arm_up_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr right_arm_down_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr left_arm_open_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr left_arm_close_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr right_arm_open_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr right_arm_close_sub_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> left_move_group_interface_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> right_move_group_interface_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> left_hand_move_group_interface_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> right_hand_move_group_interface_;
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
    std::thread left_init_thread_;
    std::thread right_init_thread_;
    
    // Store last received poses for arm up/down functionality
    std::vector<double> last_left_pose_rpy_;
    std::vector<double> last_right_pose_rpy_;
    bool left_pose_received_ = false;
    bool right_pose_received_ = false;
    
    // Configurable z-coordinate values for arm up/down
    double arm_up_z_value_ = 0.2;   // Absolute z coordinate for arm up position
    double arm_down_z_value_ = 0.086; // Absolute z coordinate for arm down position
    
    // Object mesh tracking and coordinate dictionary
    rclcpp::Subscription<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr register_coordinate_sub_;
    
    std::mutex object_mutex_;
    std::map<std::string, std::array<double, 3>> object_mesh_positions_;
    
    std::mutex coordinate_dict_mutex_;
    std::map<std::array<double, 3>, std::pair<std::array<double, 3>, std::array<double, 3>>> grasp_coordinate_dictionary_;
    
    // Direct joint trajectory publishers for grippers
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr left_gripper_pub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr right_gripper_pub_;
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