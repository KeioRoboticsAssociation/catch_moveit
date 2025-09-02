#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <thread>
#include <memory>
#include <chrono>
#include <mutex>

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
            left_hand_move_group_interface_->setPlanningPipelineId("ompl");
            left_hand_move_group_interface_->setPlannerId("RRTConnect");
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
            right_hand_move_group_interface_->setPlanningPipelineId("ompl");
            right_hand_move_group_interface_->setPlannerId("RRTConnect");
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
        
        // Initialize action clients for direct trajectory execution
        left_arm_action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
            this, "/left_arm_controller/follow_joint_trajectory");
        right_arm_action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
            this, "/right_arm_controller/follow_joint_trajectory");
            
        // Initialize servo control service clients for simple switching
        left_servo_start_client_ = this->create_client<std_srvs::srv::Trigger>(
            "/left_servo_node/start_servo");
        left_servo_stop_client_ = this->create_client<std_srvs::srv::Trigger>(
            "/left_servo_node/stop_servo");
        right_servo_start_client_ = this->create_client<std_srvs::srv::Trigger>(
            "/right_servo_node/start_servo");
        right_servo_stop_client_ = this->create_client<std_srvs::srv::Trigger>(
            "/right_servo_node/stop_servo");
            
        RCLCPP_INFO(this->get_logger(), "Simple servo switching system initialized");
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
    // Simple servo control helper functions
    void stopServo(const std::string& arm_name)
    {
        auto client = (arm_name == "left") ? left_servo_stop_client_ : right_servo_stop_client_;
        if (!client->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(), "Servo stop service not available for %s arm", arm_name.c_str());
            return;
        }
        
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto future = client->async_send_request(request);
        
        if (future.wait_for(std::chrono::seconds(2)) == std::future_status::ready) {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "Stopped %s servo: %s", arm_name.c_str(), 
                       response->success ? "Success" : response->message.c_str());
        }
    }
    
    void startServo(const std::string& arm_name)
    {
        auto client = (arm_name == "left") ? left_servo_start_client_ : right_servo_start_client_;
        if (!client->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(), "Servo start service not available for %s arm", arm_name.c_str());
            return;
        }
        
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto future = client->async_send_request(request);
        
        if (future.wait_for(std::chrono::seconds(2)) == std::future_status::ready) {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "Started %s servo: %s", arm_name.c_str(), 
                       response->success ? "Success" : response->message.c_str());
        }
    }

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

        std::string arm_name = (move_group_interface->getName() == "left_arm") ? "left" : "right";
        RCLCPP_INFO(this->get_logger(), "move_to_pose called for %s arm", arm_name.c_str());
        
        executeCommand(move_group_interface, target_pose, arm_name);
    }
    
    void executeCommand(std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface, const geometry_msgs::msg::PoseStamped& target_pose, const std::string& arm_name)
    {
        // 非同期実行のためのスレッドを作成
        std::thread([this, move_group_interface, target_pose, arm_name]() {
            RCLCPP_INFO(this->get_logger(), "Executing pose command for %s arm with servo coordination", arm_name.c_str());
            
            // Stop servo for clean pose execution
            stopServo(arm_name);
            std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Brief pause
            
            // Apply dynamic collision avoidance settings
            applyDynamicCollisionAvoidance(move_group_interface, arm_name);
            
            move_group_interface->setPoseTarget(target_pose);
            
            // プランナーをOMPLのRRTConnectに指定
            move_group_interface->setPlanningPipelineId("ompl");
            move_group_interface->setPlannerId("RRTConnect");
            
            // デフォルトの高速設定（他のアーム動作中でない場合）
            if (!((arm_name == "left" && right_arm_executing_) || (arm_name == "right" && left_arm_executing_))) {
                move_group_interface->setPlanningTime(1.0);     // 高速プランニング
                move_group_interface->setNumPlanningAttempts(1); // 1回のみ
            }

            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            bool success = (move_group_interface->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

            if (success)
            {
                RCLCPP_INFO(this->get_logger(), "Planner found a plan for %s, executing via direct trajectory.", arm_name.c_str());
                execute_trajectory_directly(move_group_interface, my_plan, arm_name);  // 直接軌道送信
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Planning failed for %s.", arm_name.c_str());
                stopTrajectoryTracking(arm_name);
                
                // Restart servo even after planning failure for continuous control
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
                startServo(arm_name);
                RCLCPP_INFO(this->get_logger(), "Servo restarted for %s arm after planning failure", arm_name.c_str());
            }
        }).detach();  // スレッドをデタッチして非同期実行
    }

    void execute_trajectory_directly(
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface,
        const moveit::planning_interface::MoveGroupInterface::Plan& plan,
        const std::string& arm_name)
    {
        // アクションクライアントを選択
        auto action_client = (arm_name == "left") ? left_arm_action_client_ : right_arm_action_client_;
        
        if (!action_client) {
            RCLCPP_ERROR(this->get_logger(), "Action client not initialized for %s arm", arm_name.c_str());
            return;
        }

        // アクションサーバーを待機
        if (!action_client->wait_for_action_server(std::chrono::seconds(1))) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available for %s arm", arm_name.c_str());
            return;
        }

        // ゴールを作成
        auto goal_msg = FollowJointTrajectory::Goal();
        goal_msg.trajectory = plan.trajectory_.joint_trajectory;

        // 非同期で送信
        auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            [this, arm_name](auto) {
                RCLCPP_INFO(this->get_logger(), "Goal accepted by %s arm controller", arm_name.c_str());
            };
        send_goal_options.result_callback =
            [this, arm_name](const rclcpp_action::ClientGoalHandle<FollowJointTrajectory>::WrappedResult & result) {
                RCLCPP_INFO(this->get_logger(), "Trajectory execution completed for %s arm", arm_name.c_str());
                stopTrajectoryTracking(arm_name);
                
                // Restart servo after trajectory completion for seamless switching
                std::this_thread::sleep_for(std::chrono::milliseconds(200)); // Brief pause
                startServo(arm_name);
                RCLCPP_INFO(this->get_logger(), "Servo restarted for %s arm - ready for realtime control", arm_name.c_str());
            };

        action_client->async_send_goal(goal_msg, send_goal_options);
        RCLCPP_INFO(this->get_logger(), "Trajectory sent directly to %s arm controller", arm_name.c_str());
        
        // Update trajectory tracking for collision avoidance
        updateTrajectoryTracking(plan, arm_name);
    }

    void updateTrajectoryTracking(const moveit::planning_interface::MoveGroupInterface::Plan& plan, const std::string& arm_name)
    {
        std::lock_guard<std::mutex> lock(trajectory_mutex_);
        auto now = std::chrono::steady_clock::now();
        
        if (arm_name == "left") {
            current_left_plan_ = plan;
            left_arm_executing_ = true;
            left_execution_start_ = now;
        } else {
            current_right_plan_ = plan;
            right_arm_executing_ = true;
            right_execution_start_ = now;
        }
        
        RCLCPP_INFO(this->get_logger(), "Updated trajectory tracking for %s arm", arm_name.c_str());
    }

    void stopTrajectoryTracking(const std::string& arm_name)
    {
        std::lock_guard<std::mutex> lock(trajectory_mutex_);
        
        if (arm_name == "left") {
            left_arm_executing_ = false;
        } else {
            right_arm_executing_ = false;
        }
        
        RCLCPP_INFO(this->get_logger(), "Stopped trajectory tracking for %s arm", arm_name.c_str());
    }
    

    void applyDynamicCollisionAvoidance(std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface, const std::string& arm_name)
    {
        std::lock_guard<std::mutex> lock(trajectory_mutex_);
        
        // Check if the other arm is executing
        bool other_arm_executing = (arm_name == "left") ? right_arm_executing_ : left_arm_executing_;
        
        if (other_arm_executing) {
            RCLCPP_INFO(this->get_logger(), "Other arm is executing, applying dynamic collision avoidance for %s arm", arm_name.c_str());
            
            // Get the other arm's current trajectory
            auto other_plan = (arm_name == "left") ? current_right_plan_ : current_left_plan_;
            auto other_start_time = (arm_name == "left") ? right_execution_start_ : left_execution_start_;
            
            // Calculate elapsed time since other arm started executing
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - other_start_time);
            
            // Apply more conservative collision checking
            move_group_interface->setGoalPositionTolerance(0.001);   // Balanced tolerance
            move_group_interface->setGoalOrientationTolerance(0.001); // Relaxed orientation
            move_group_interface->setPlanningTime(2.0);             // Sufficient planning time
            move_group_interface->setNumPlanningAttempts(3);        // Fewer attempts for speed
            
            RCLCPP_INFO(this->get_logger(), "Applied enhanced collision avoidance settings for %s arm (other arm executing for %ld ms)", 
                       arm_name.c_str(), elapsed.count());
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
        move_arm_z("left", arm_up_z_value_);
    }

    void left_arm_down_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Left arm down command received");
        move_arm_z("left", arm_down_z_value_);
    }

    void right_arm_up_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Right arm up command received");
        move_arm_z("right", arm_up_z_value_);
    }

    void right_arm_down_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Right arm down command received");
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

        // Execute z-movement directly
        RCLCPP_INFO(this->get_logger(), "Executing z-movement command for %s arm", arm_name.c_str());
        executeZMoveCommand(move_group_interface, target_pose, arm_name, target_z);
    }
    
    void executeZMoveCommand(std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface, 
                            const geometry_msgs::msg::PoseStamped& target_pose, const std::string& arm_name, double target_z)
    {
        // 非同期実行でz-movementを実行
        std::thread([this, move_group_interface, target_pose, arm_name]() {
            RCLCPP_INFO(this->get_logger(), "Executing %s arm z-movement with servo coordination", arm_name.c_str());
            
            // Stop servo for clean z-movement execution
            stopServo(arm_name);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            
            // Apply dynamic collision avoidance settings
            applyDynamicCollisionAvoidance(move_group_interface, arm_name);
            
            // Use Pilz LIN planner for straight line motion
            move_group_interface->setPlanningPipelineId("pilz_industrial_motion_planner");
            move_group_interface->setPlannerId("LIN");
            move_group_interface->setPoseTarget(target_pose);

            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            bool success = (move_group_interface->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

            if (success) {
                RCLCPP_INFO(this->get_logger(), "Pilz LIN planner found a plan for %s arm z-movement, executing via direct trajectory.", arm_name.c_str());
                execute_trajectory_directly(move_group_interface, my_plan, arm_name);  // 直接軌道送信
            } else {
                RCLCPP_ERROR(this->get_logger(), "Pilz LIN planning failed for %s arm z-movement.", arm_name.c_str());
                stopTrajectoryTracking(arm_name);
                
                // Restart servo after planning failure
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
                startServo(arm_name);
                RCLCPP_INFO(this->get_logger(), "Servo restarted for %s arm after z-movement planning failure", arm_name.c_str());
            }
        }).detach();
    }

    void left_arm_open_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Left arm open command received");
        set_gripper_position("left", 0.0, 0.0);
    }

    void left_arm_close_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Left arm close command received");
        set_gripper_position("left", 0.024, -0.024);
    }

    void right_arm_open_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Right arm open command received");
        set_gripper_position("right", 0.0, 0.0);
    }

    void right_arm_close_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Right arm close command received");
        set_gripper_position("right", 0.024, -0.024);
    }

    void set_gripper_position(const std::string& arm_name, double slider1_value, double slider2_value)
    {
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> hand_move_group_interface;
        
        if (arm_name == "left") {
            hand_move_group_interface = left_hand_move_group_interface_;
        } else if (arm_name == "right") {
            hand_move_group_interface = right_hand_move_group_interface_;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Invalid arm name: %s", arm_name.c_str());
            return;
        }

        if (!hand_move_group_interface) {
            RCLCPP_ERROR(this->get_logger(), "MoveGroupInterface not initialized for %s hand", arm_name.c_str());
            return;
        }

        // Get joint names to understand the structure
        std::vector<std::string> joint_names = hand_move_group_interface->getJointNames();
        RCLCPP_INFO(this->get_logger(), "Joint names for %s hand:", arm_name.c_str());
        for (size_t i = 0; i < joint_names.size(); i++) {
            RCLCPP_INFO(this->get_logger(), "  [%zu]: %s", i, joint_names[i].c_str());
        }
        
        // Create joint target map for specific gripper joints
        std::map<std::string, double> joint_targets;
        
        // Set specific gripper joint names based on arm
        std::string slider1_name = arm_name + "_Slider_1";
        std::string slider2_name = arm_name + "_Slider_2";
        
        joint_targets[slider1_name] = slider1_value;
        joint_targets[slider2_name] = slider2_value;
        
        RCLCPP_INFO(this->get_logger(), "Setting %s hand gripper: %s=%f, %s=%f", 
                   arm_name.c_str(), slider1_name.c_str(), slider1_value, 
                   slider2_name.c_str(), slider2_value);
        
        // Use named targets for gripper control
        hand_move_group_interface->setJointValueTarget(joint_targets);
        
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (hand_move_group_interface->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        
        if (success) {
            RCLCPP_INFO(this->get_logger(), "Gripper position plan found for %s hand, executing asynchronously...", arm_name.c_str());
            // 非同期実行のためのスレッドを作成
            std::thread([this, hand_move_group_interface, my_plan, arm_name]() {
                hand_move_group_interface->execute(my_plan);
                RCLCPP_INFO(this->get_logger(), "Gripper execution completed for %s hand", arm_name.c_str());
            }).detach();
        } else {
            RCLCPP_ERROR(this->get_logger(), "Gripper position planning failed for %s hand", arm_name.c_str());
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
    
    // Action clients for direct trajectory execution
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr left_arm_action_client_;
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr right_arm_action_client_;
    
    // Track current trajectories for collision avoidance
    std::mutex trajectory_mutex_;
    moveit::planning_interface::MoveGroupInterface::Plan current_left_plan_;
    moveit::planning_interface::MoveGroupInterface::Plan current_right_plan_;
    bool left_arm_executing_ = false;
    bool right_arm_executing_ = false;
    std::chrono::steady_clock::time_point left_execution_start_;
    std::chrono::steady_clock::time_point right_execution_start_;
    
    // Servo control service clients for simple switching
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr left_servo_start_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr left_servo_stop_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr right_servo_start_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr right_servo_stop_client_;
    
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