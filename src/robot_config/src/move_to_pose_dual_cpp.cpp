#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
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
        // Subscribe to pose targets early to avoid missing first messages
        left_rpy_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/left_target_pose_rpy", 10, std::bind(&MoveToPoseDualCpp::left_rpy_callback, this, std::placeholders::_1));
        right_rpy_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/right_target_pose_rpy", 10, std::bind(&MoveToPoseDualCpp::right_rpy_callback, this, std::placeholders::_1));
        // Dual composite target (12 elements)
        dual_rpy_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/dual_target_pose_rpy", 10, std::bind(&MoveToPoseDualCpp::dual_rpy_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Ready to receive targets on /left_target_pose_rpy and /right_target_pose_rpy");

        // Joint state readiness gate
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            [this](const sensor_msgs::msg::JointState::SharedPtr) {
                if (!joint_state_ready_) {
                    joint_state_ready_ = true;
                    RCLCPP_INFO(this->get_logger(), "Joint states received - system ready");
                    attemptFlushPending();
                }
            }
        );

        // Parameter to enable/disable local PlanningSceneMonitor (heavy)
        enable_local_psm_ = this->declare_parameter<bool>("enable_local_planning_scene_monitor", false);

        // 左アームの初期化スレッド
        left_init_thread_ = std::thread([this]() {
            RCLCPP_INFO(this->get_logger(), "Initializing MoveGroupInterface for left_arm...");
            left_move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(rclcpp::Node::SharedPtr(this, [](rclcpp::Node*){}), "left_arm");
            left_move_group_interface_->setEndEffectorLink("left_EndEffector_1");
            left_move_group_interface_->setPlanningPipelineId("ompl");
            left_move_group_interface_->setPlannerId("RRTConnectkConfigDefault");
            RCLCPP_INFO(this->get_logger(), "MoveGroupInterface for left_arm initialized.");

            // 左ハンド（グリッパー）の初期化
            RCLCPP_INFO(this->get_logger(), "Initializing MoveGroupInterface for left_hand...");
            left_hand_move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(rclcpp::Node::SharedPtr(this, [](rclcpp::Node*){}), "left_hand");
            left_hand_move_group_interface_->setPlanningPipelineId("ompl");
            left_hand_move_group_interface_->setPlannerId("RRTConnect");
            RCLCPP_INFO(this->get_logger(), "MoveGroupInterface for left_hand initialized.");

            // 初期化中に受信したターゲットがあればフラッシュ実行
            if (left_pose_received_ && last_left_pose_rpy_.size() == 6 && left_move_group_interface_) {
                tf2::Quaternion q;
                q.setRPY(last_left_pose_rpy_[3], last_left_pose_rpy_[4], last_left_pose_rpy_[5]);
                geometry_msgs::msg::PoseStamped target_pose;
                target_pose.header.frame_id = "world";
                target_pose.header.stamp = this->get_clock()->now();
                target_pose.pose.position.x = last_left_pose_rpy_[0];
                target_pose.pose.position.y = last_left_pose_rpy_[1];
                target_pose.pose.position.z = last_left_pose_rpy_[2];
                target_pose.pose.orientation = tf2::toMsg(q);
                RCLCPP_INFO(this->get_logger(), "Flushing buffered left target pose after initialization");
                move_to_pose(left_move_group_interface_, target_pose);
                left_pose_received_ = false; // 一度消化
            }

            // 初回プランナウォームアップ
            prewarmPlanner(left_move_group_interface_, "left");
        });

        // 右アームの初期化スレッド
        right_init_thread_ = std::thread([this]() {
            RCLCPP_INFO(this->get_logger(), "Initializing MoveGroupInterface for right_arm...");
            right_move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(rclcpp::Node::SharedPtr(this, [](rclcpp::Node*){}), "right_arm");
            right_move_group_interface_->setEndEffectorLink("right_EndEffector_1");
            right_move_group_interface_->setPlanningPipelineId("ompl");
            right_move_group_interface_->setPlannerId("RRTConnectkConfigDefault");
            RCLCPP_INFO(this->get_logger(), "MoveGroupInterface for right_arm initialized.");

            // 右ハンド（グリッパー）の初期化
            RCLCPP_INFO(this->get_logger(), "Initializing MoveGroupInterface for right_hand...");
            right_hand_move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(rclcpp::Node::SharedPtr(this, [](rclcpp::Node*){}), "right_hand");
            right_hand_move_group_interface_->setPlanningPipelineId("ompl");
            right_hand_move_group_interface_->setPlannerId("RRTConnect");
            RCLCPP_INFO(this->get_logger(), "MoveGroupInterface for right_hand initialized.");

            // 初期化中に受信したターゲットがあればフラッシュ実行
            if (right_pose_received_ && last_right_pose_rpy_.size() == 6 && right_move_group_interface_) {
                tf2::Quaternion q;
                q.setRPY(last_right_pose_rpy_[3], last_right_pose_rpy_[4], last_right_pose_rpy_[5]);
                geometry_msgs::msg::PoseStamped target_pose;
                target_pose.header.frame_id = "world";
                target_pose.header.stamp = this->get_clock()->now();
                target_pose.pose.position.x = last_right_pose_rpy_[0];
                target_pose.pose.position.y = last_right_pose_rpy_[1];
                target_pose.pose.position.z = last_right_pose_rpy_[2];
                target_pose.pose.orientation = tf2::toMsg(q);
                RCLCPP_INFO(this->get_logger(), "Flushing buffered right target pose after initialization");
                move_to_pose(right_move_group_interface_, target_pose);
                right_pose_received_ = false; // 一度消化
            }

            // 初回プランナウォームアップ
            prewarmPlanner(right_move_group_interface_, "right");
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

        // Defer heavy initializations until after the executor starts spinning
        post_init_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            [this]() {
                do_post_init();
                // run only once
                post_init_timer_->cancel();
            }
        );
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
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;

    void do_post_init()
    {
        // Initialize PlanningSceneInterface
        planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

        // Optionally initialize local Planning Scene Monitor (skip by default to speed startup)
        if (enable_local_psm_) {
            planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
                shared_from_this(), "robot_description");
            if (planning_scene_monitor_) {
                planning_scene_monitor_->startSceneMonitor();
                planning_scene_monitor_->startWorldGeometryMonitor();
                planning_scene_monitor_->startStateMonitor();
                RCLCPP_INFO(this->get_logger(), "Planning Scene Monitor initialized and started");
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to initialize Planning Scene Monitor");
            }
        } else {
            RCLCPP_INFO(this->get_logger(), "Local PlanningSceneMonitor disabled (enable with param enable_local_planning_scene_monitor=true)");
        }

        // Initialize action clients for direct trajectory execution
        left_arm_action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
            this, "/left_arm_controller/follow_joint_trajectory");
        right_arm_action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
            this, "/right_arm_controller/follow_joint_trajectory");

        // Proactively connect to action servers to reduce first-goal latency
        if (left_arm_action_client_) {
            left_arm_action_client_->wait_for_action_server(std::chrono::seconds(5));
        }
        if (right_arm_action_client_) {
            right_arm_action_client_->wait_for_action_server(std::chrono::seconds(5));
        }

        actions_ready_ = true;
        attemptFlushPending();

        // Initialize servo control service clients for simple switching
        left_servo_start_client_ = this->create_client<std_srvs::srv::Trigger>(
            "/left_servo_node/start_servo");
        left_servo_stop_client_ = this->create_client<std_srvs::srv::Trigger>(
            "/left_servo_node/stop_servo");
        right_servo_start_client_ = this->create_client<std_srvs::srv::Trigger>(
            "/right_servo_node/start_servo");
        right_servo_stop_client_ = this->create_client<std_srvs::srv::Trigger>(
            "/right_servo_node/stop_servo");

        // Initialize TF2 for direct pose retrieval
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        RCLCPP_INFO(this->get_logger(), "Simple servo switching system initialized");
    }

    // Simple servo control helper functions
    void stopServo(const std::string& arm_name)
    {
        auto client = (arm_name == "left") ? left_servo_stop_client_ : right_servo_stop_client_;
        if (!client->wait_for_service(std::chrono::milliseconds(100))) {
            RCLCPP_WARN(this->get_logger(), "Servo stop service not available for %s arm", arm_name.c_str());
            return;
        }
        
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto future = client->async_send_request(request);
        
        if (future.wait_for(std::chrono::milliseconds(500)) == std::future_status::ready) {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "Stopped %s servo: %s", arm_name.c_str(), 
                       response->success ? "Success" : response->message.c_str());
        }
    }
    
    void startServo(const std::string& arm_name)
    {
        auto client = (arm_name == "left") ? left_servo_start_client_ : right_servo_start_client_;
        if (!client->wait_for_service(std::chrono::milliseconds(100))) {
            RCLCPP_WARN(this->get_logger(), "Servo start service not available for %s arm", arm_name.c_str());
            return;
        }
        
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto future = client->async_send_request(request);
        
        if (future.wait_for(std::chrono::milliseconds(500)) == std::future_status::ready) {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "Started %s servo: %s", arm_name.c_str(), 
                       response->success ? "Success" : response->message.c_str());
        }
    }

    void dual_rpy_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (msg->data.size() != 12) {
            RCLCPP_ERROR(this->get_logger(), "Dual target requires 12 values, got %zu", msg->data.size());
            return;
        }
        // left
        tf2::Quaternion ql; ql.setRPY(msg->data[3], msg->data[4], msg->data[5]);
        geometry_msgs::msg::PoseStamped l_pose;
        l_pose.header.frame_id = "world"; l_pose.header.stamp = this->get_clock()->now();
        l_pose.pose.position.x = msg->data[0]; l_pose.pose.position.y = msg->data[1]; l_pose.pose.position.z = msg->data[2];
        l_pose.pose.orientation = tf2::toMsg(ql);
        // right
        tf2::Quaternion qr; qr.setRPY(msg->data[9], msg->data[10], msg->data[11]);
        geometry_msgs::msg::PoseStamped r_pose;
        r_pose.header.frame_id = "world"; r_pose.header.stamp = this->get_clock()->now();
        r_pose.pose.position.x = msg->data[6]; r_pose.pose.position.y = msg->data[7]; r_pose.pose.position.z = msg->data[8];
        r_pose.pose.orientation = tf2::toMsg(qr);

        // Execute strictly synchronized planning/execution
        planAndExecuteSynchronized(l_pose, r_pose);
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

        // Attempt synchronized execution with right arm if available
        handleIncomingTargetPose("left", target_pose);
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

        // Attempt synchronized execution with left arm if available
        handleIncomingTargetPose("right", target_pose);
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
            std::this_thread::sleep_for(std::chrono::milliseconds(60)); // Brief pause to ensure servo fully stops
            
            // Apply dynamic collision avoidance settings
            applyDynamicCollisionAvoidance(move_group_interface, arm_name);
            
            // Fresh state and frame, then set target
            try { move_group_interface->setStartStateToCurrentState(); } catch (...) {}
            move_group_interface->setPoseReferenceFrame("world");
            move_group_interface->setPoseTarget(target_pose);
            
            // OMPL only, immediate planning
            bool success = false;
            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            move_group_interface->setPlanningPipelineId("ompl");
            move_group_interface->setPlanningTime(0.25);
            move_group_interface->setNumPlanningAttempts(2);
            // Position tolerance 0.1mm
            move_group_interface->setGoalPositionTolerance(0.0001);
            move_group_interface->setGoalOrientationTolerance(0.1); // relax orientation tol for solvability
            move_group_interface->setMaxVelocityScalingFactor(0.6);
            move_group_interface->setMaxAccelerationScalingFactor(0.6);
            // 探索方法が異なるプランナーの組み合わせ（軽量順）
            std::vector<std::string> planners = {"RRTConnectkConfigDefault", "BKPIECEkConfigDefault", "PRMkConfigDefault"};
            
            for (const auto& planner : planners) {
                move_group_interface->setPlannerId(planner);
                // Keep the relaxed tolerances set above
                success = (move_group_interface->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
                if (success) {
                    RCLCPP_INFO(this->get_logger(), "%s planner succeeded for %s (OMPL)", planner.c_str(), arm_name.c_str());
                    break;
                } else {
                    RCLCPP_WARN(this->get_logger(), "%s planner failed for %s, trying next...", planner.c_str(), arm_name.c_str());
                }
            }

            // Escalate planning time/attempts if fast path failed completely
            if (!success) {
                move_group_interface->setPlanningTime(1.0);
                move_group_interface->setNumPlanningAttempts(6);
                for (const auto& planner : planners) {
                    move_group_interface->setPlannerId(planner);
                    if (move_group_interface->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
                        success = true;
                        RCLCPP_INFO(this->get_logger(), "Escalated planning succeeded (OMPL %s) for %s", planner.c_str(), arm_name.c_str());
                        break;
                    }
                }
            }

            if (success)
            {
                RCLCPP_INFO(this->get_logger(), "Planner found a plan for %s, executing via direct trajectory.", arm_name.c_str());
                execute_trajectory_directly(move_group_interface, my_plan, arm_name);
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "All planners failed for %s.", arm_name.c_str());
                
                stopTrajectoryTracking(arm_name);
                
                // Restart servo even after planning failure for continuous control
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
                startServo(arm_name);
                RCLCPP_INFO(this->get_logger(), "Servo restarted for %s arm after planning failure", arm_name.c_str());
            }
        }).detach();  // スレッドをデタッチして非同期実行
    }

    // 初回のプランニング遅延を抑えるための軽いウォームアップ
    void prewarmPlanner(const std::shared_ptr<moveit::planning_interface::MoveGroupInterface>& mgi,
                        const std::string& arm_name)
    {
        if (!mgi) return;
        try {
            mgi->setStartStateToCurrentState();
            geometry_msgs::msg::PoseStamped current_pose = mgi->getCurrentPose();
            mgi->setPoseReferenceFrame("world");
            mgi->clearPoseTargets();
            mgi->setPoseTarget(current_pose);
            mgi->setPlanningPipelineId("ompl");
            mgi->setPlannerId("RRTConnectkConfigDefault");
            mgi->setPlanningTime(0.2);
            mgi->setNumPlanningAttempts(1);
            moveit::planning_interface::MoveGroupInterface::Plan warm_plan;
            (void) mgi->plan(warm_plan); // 結果は使わない。プラグイン/シーンの初期化を済ませる目的
            RCLCPP_INFO(this->get_logger(), "Prewarmed planner for %s arm", arm_name.c_str());
        } catch (...) {
            RCLCPP_WARN(this->get_logger(), "Prewarm planner failed for %s arm (ignored)", arm_name.c_str());
        }
    }

    // --- Dual-arm synchronization: pair nearly-simultaneous targets ---
    void handleIncomingTargetPose(const std::string& arm_name, const geometry_msgs::msg::PoseStamped& target_pose)
    {
        const auto now_tp = std::chrono::steady_clock::now();
        // Larger pairing window to ensure dual commands synchronize reliably
        const auto window = std::chrono::milliseconds(1000);

        bool should_sync = false;
        geometry_msgs::msg::PoseStamped l_pose, r_pose;

        {
            std::lock_guard<std::mutex> lk(sync_mutex_);
            if (arm_name == "left") {
                pending_left_pose_ = target_pose;
                pending_left_time_ = now_tp;
                has_pending_left_pose_ = true;
            } else {
                pending_right_pose_ = target_pose;
                pending_right_time_ = now_tp;
                has_pending_right_pose_ = true;
            }

            if (has_pending_left_pose_ && has_pending_right_pose_) {
                auto dt = (pending_left_time_ > pending_right_time_) ? (pending_left_time_ - pending_right_time_) : (pending_right_time_ - pending_left_time_);
                if (dt <= window) {
                    l_pose = pending_left_pose_;
                    r_pose = pending_right_pose_;
                    has_pending_left_pose_ = false;
                    has_pending_right_pose_ = false;
                    should_sync = true;
                }
            }
        }

        if (should_sync) {
            std::thread([this, l_pose, r_pose]() {
                planAndExecuteSynchronized(l_pose, r_pose);
            }).detach();
            return;
        }

        // Single-arm fallback after the pairing window: if partner didn't arrive, execute singly
        std::thread([this, arm_name, target_pose, now_tp, window]() {
            std::this_thread::sleep_for(window);
            bool still_unpaired = false;
            {
                std::lock_guard<std::mutex> lk(sync_mutex_);
                if (arm_name == "left") {
                    still_unpaired = has_pending_left_pose_ && (pending_left_time_ == now_tp);
                    if (still_unpaired) has_pending_left_pose_ = false;
                } else {
                    still_unpaired = has_pending_right_pose_ && (pending_right_time_ == now_tp);
                    if (still_unpaired) has_pending_right_pose_ = false;
                }
            }
            if (still_unpaired) {
                auto mgi = (arm_name == "left") ? left_move_group_interface_ : right_move_group_interface_;
                move_to_pose(mgi, target_pose);
            }
        }).detach();
    }

    void planAndExecuteSynchronized(const geometry_msgs::msg::PoseStamped& left_pose,
                                    const geometry_msgs::msg::PoseStamped& right_pose)
    {
        RCLCPP_INFO(this->get_logger(), "Planning synchronized dual-arm motion (short-time)");

        // Stop both servos and wait briefly
        stopServo("left");
        stopServo("right");
        std::this_thread::sleep_for(std::chrono::milliseconds(60));

        auto plan_one = [this](std::shared_ptr<moveit::planning_interface::MoveGroupInterface> mgi,
                               const geometry_msgs::msg::PoseStamped& pose) -> bool
        {
            try { mgi->setStartStateToCurrentState(); } catch (...) {}
            mgi->setPoseReferenceFrame("world");
            mgi->clearPoseTargets();
            mgi->setPoseTarget(pose);
            // OMPL only with 0.1mm position tolerance
            mgi->setPlanningPipelineId("ompl");
            mgi->setPlannerId("RRTConnectkConfigDefault");
            mgi->setGoalPositionTolerance(0.0001);
            mgi->setGoalOrientationTolerance(0.0001);
            mgi->setPlanningTime(0.1);
            mgi->setNumPlanningAttempts(20);
            mgi->setMaxVelocityScalingFactor(0.8);
            mgi->setMaxAccelerationScalingFactor(0.8);
            if (mgi->plan(plan_buffer_) == moveit::core::MoveItErrorCode::SUCCESS) return true;
            // Escalate
            mgi->setPlanningTime(0.1);
            mgi->setPlannerId("PRMkConfigDefault");
            return (mgi->plan(plan_buffer_) == moveit::core::MoveItErrorCode::SUCCESS);
        };

        moveit::planning_interface::MoveGroupInterface::Plan left_plan, right_plan;
        bool left_ok = false, right_ok = false;

        if (left_move_group_interface_) {
            left_ok = plan_one(left_move_group_interface_, left_pose);
            if (left_ok) left_plan = plan_buffer_;
        }
        if (right_move_group_interface_) {
            right_ok = plan_one(right_move_group_interface_, right_pose);
            if (right_ok) right_plan = plan_buffer_;
        }

        if (!left_ok && !right_ok) {
            RCLCPP_ERROR(this->get_logger(), "Both synchronized plans failed. Aborting.");
            startServo("left");
            startServo("right");
            return;
        }

        auto send_one = [this](const std::string& arm,
                               rclcpp_action::Client<FollowJointTrajectory>::SharedPtr client,
                               const moveit::planning_interface::MoveGroupInterface::Plan& plan)
        {
            if (!client) {
                RCLCPP_ERROR(this->get_logger(), "Action client missing for %s", arm.c_str());
                startServo(arm);
                return;
            }
            if (!client->wait_for_action_server(std::chrono::seconds(5))) {
                RCLCPP_ERROR(this->get_logger(), "Action server not available for %s arm", arm.c_str());
                startServo(arm);
                return;
            }
            auto goal = FollowJointTrajectory::Goal();
            goal.trajectory = plan.trajectory_.joint_trajectory;
            auto options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
            options.goal_response_callback = [this, arm](auto){ RCLCPP_INFO(this->get_logger(), "[SYNC] Goal accepted by %s", arm.c_str()); };
            options.result_callback = [this, arm](auto){ RCLCPP_INFO(this->get_logger(), "[SYNC] Execution done for %s", arm.c_str()); stopTrajectoryTracking(arm); std::this_thread::sleep_for(std::chrono::milliseconds(80)); startServo(arm); };
            client->async_send_goal(goal, options);
            updateTrajectoryTracking(plan, arm);
        };

        // Ensure truly simultaneous start by scheduling same future start time
        auto now = this->get_clock()->now();
        // Add small latency margin to allow both goals to be sent before start
        auto start_time = now + rclcpp::Duration::from_seconds(0.2);
        if (left_ok) {
            left_plan.trajectory_.joint_trajectory.header.stamp = start_time;
        }
        if (right_ok) {
            right_plan.trajectory_.joint_trajectory.header.stamp = start_time;
        }

        if (left_ok)  send_one("left",  left_arm_action_client_,  left_plan);
        if (right_ok) send_one("right", right_arm_action_client_, right_plan);

        RCLCPP_INFO(this->get_logger(), "Synchronized dual-arm trajectories sent (short planning)");
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
            // Stop済みであれば必ず再開しておく
            startServo(arm_name);
            return;
        }

        // アクションサーバーを待機（初回起動安定化のため余裕を持つ）
        if (!action_client->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available for %s arm", arm_name.c_str());
            // サーバ未準備時もServoを再開して、後続の操作を阻害しない
            startServo(arm_name);
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
                std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Brief pause
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
            RCLCPP_INFO(this->get_logger(), "Other arm is executing, using Planning Scene Monitor for real-time collision avoidance");
            
            // Planning Scene Monitorから最新のロボット状態を取得
            if (planning_scene_monitor_) {
                planning_scene_monitor::LockedPlanningSceneRO locked_scene(planning_scene_monitor_);
                if (locked_scene) {
                    // 最新のロボット状態を開始状態に設定
                    moveit::core::RobotState current_state = locked_scene->getCurrentState();
                    move_group_interface->setStartState(current_state);
                    
                    RCLCPP_INFO(this->get_logger(), "Updated robot state from Planning Scene Monitor for %s arm", arm_name.c_str());
                }
            }
            
            // より慎重（ただし過度に厳密でない）なプランニング設定
            move_group_interface->setGoalPositionTolerance(0.004);
            move_group_interface->setGoalOrientationTolerance(0.04);
            move_group_interface->setPlanningTime(0.6);
            move_group_interface->setNumPlanningAttempts(3);
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
        RCLCPP_INFO(this->get_logger(), "Left arm up command received, calling move_arm_z with z=%.3f", arm_up_z_value_);
        move_arm_z("left", arm_up_z_value_);
        RCLCPP_INFO(this->get_logger(), "move_arm_z call completed");
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
        RCLCPP_INFO(this->get_logger(), "move_arm_z called for %s arm with target_z=%.3f", arm_name.c_str(), target_z);
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface;

        if (arm_name == "left") {
            move_group_interface = left_move_group_interface_;
        } else if (arm_name == "right") {
            move_group_interface = right_move_group_interface_;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Invalid arm name: %s", arm_name.c_str());
            return;
        }

        if (!move_group_interface) {
            RCLCPP_ERROR(this->get_logger(), "MoveGroupInterface not initialized for %s arm", arm_name.c_str());
            return;
        }

        // Get current pose using TF2 directly from EndEffector frame
        geometry_msgs::msg::PoseStamped current_pose;
        std::string target_frame = arm_name + "_EndEffector_1";
        
        try {
            // Get transform from world to EndEffector
            geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
                "world", target_frame, tf2::TimePointZero, tf2::durationFromSec(1.0));
                
            // Convert transform to PoseStamped
            current_pose.header.frame_id = "world";
            current_pose.header.stamp = this->get_clock()->now();
            current_pose.pose.position.x = transform.transform.translation.x;
            current_pose.pose.position.y = transform.transform.translation.y;
            current_pose.pose.position.z = transform.transform.translation.z;
            current_pose.pose.orientation = transform.transform.rotation;
            
            RCLCPP_INFO(this->get_logger(), "Got current pose via TF2 for %s arm: x=%.3f, y=%.3f, z=%.3f", 
                       arm_name.c_str(), current_pose.pose.position.x, 
                       current_pose.pose.position.y, current_pose.pose.position.z);
                       
        } catch (const tf2::TransformException & ex) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get current pose via TF2 for %s arm: %s", arm_name.c_str(), ex.what());
            return;
        }

        // Create target pose with modified z-coordinate, keeping current x, y, and orientation
        geometry_msgs::msg::PoseStamped target_pose = current_pose;
        target_pose.header.stamp = this->get_clock()->now();
        target_pose.pose.position.z = target_z;  // Set absolute z value

        // Execute z-movement directly
        RCLCPP_INFO(this->get_logger(), "Executing z-movement for %s arm from z=%.3f to z=%.3f", 
                   arm_name.c_str(), current_pose.pose.position.z, target_z);
        RCLCPP_INFO(this->get_logger(), "Target pose: x=%.3f, y=%.3f, z=%.3f, qx=%.3f, qy=%.3f, qz=%.3f, qw=%.3f",
                   target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z,
                   target_pose.pose.orientation.x, target_pose.pose.orientation.y, 
                   target_pose.pose.orientation.z, target_pose.pose.orientation.w);
        executeZMoveCommand(move_group_interface, target_pose, arm_name, target_z);
    }
    
    void executeZMoveCommand(std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface, 
                            const geometry_msgs::msg::PoseStamped& target_pose, const std::string& arm_name, double target_z)
    {
        // 非同期実行でz-movementを実行
        std::thread([this, move_group_interface, target_pose, arm_name]() {
            RCLCPP_INFO(this->get_logger(), "Executing %s arm z-movement with servo coordination", arm_name.c_str());
            
            // Stop servo for clean pose execution
            stopServo(arm_name);
            // std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Brief pause - removed for faster response
            
            // Configure Pilz LIN planner with simple settings
            move_group_interface->setPlanningPipelineId("pilz_industrial_motion_planner");
            move_group_interface->setPlannerId("LIN");
            move_group_interface->setPlanningTime(0.1);
            move_group_interface->setNumPlanningAttempts(3);
            
            // Check current state before planning
            geometry_msgs::msg::PoseStamped current_pose_check = move_group_interface->getCurrentPose();
            RCLCPP_INFO(this->get_logger(), "Pre-planning check - Current: x=%.3f, y=%.3f, z=%.3f", 
                       current_pose_check.pose.position.x, current_pose_check.pose.position.y, current_pose_check.pose.position.z);
            
            move_group_interface->setPoseTarget(target_pose);
            
            RCLCPP_INFO(this->get_logger(), "About to plan Pilz LIN motion for %s arm", arm_name.c_str());

            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            bool success = (move_group_interface->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

            if (success) {
                RCLCPP_INFO(this->get_logger(), "Pilz LIN planner found a plan for %s arm z-movement, executing via direct trajectory.", arm_name.c_str());
                execute_trajectory_directly(move_group_interface, my_plan, arm_name);
            } else {
                RCLCPP_WARN(this->get_logger(), "Pilz LIN planning failed for %s arm z-movement, trying OMPL RRTConnect.", arm_name.c_str());
                
                // Fallback to OMPL RRTConnect planner
                move_group_interface->setPlanningPipelineId("ompl");
                move_group_interface->setPlannerId("RRTConnect");
                move_group_interface->setPlanningTime(5.0);
                move_group_interface->setNumPlanningAttempts(3);
                move_group_interface->setPoseTarget(target_pose);
                
                bool ompl_success = (move_group_interface->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
                
                if (ompl_success) {
                    RCLCPP_INFO(this->get_logger(), "OMPL RRTConnect found a plan for %s arm z-movement, executing via direct trajectory.", arm_name.c_str());
                    execute_trajectory_directly(move_group_interface, my_plan, arm_name);
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Both Pilz LIN and OMPL RRTConnect planning failed for %s arm z-movement.", arm_name.c_str());
                    stopTrajectoryTracking(arm_name);
                    
                    // Restart servo even after planning failure for continuous control
                    std::this_thread::sleep_for(std::chrono::milliseconds(200));
                    startServo(arm_name);
                    RCLCPP_INFO(this->get_logger(), "Servo restarted for %s arm after z-movement planning failure", arm_name.c_str());
                }
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
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr dual_rpy_sub_;
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
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> left_move_group_interface_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> right_move_group_interface_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> left_hand_move_group_interface_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> right_hand_move_group_interface_;
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
    std::thread left_init_thread_;
    std::thread right_init_thread_;
    rclcpp::TimerBase::SharedPtr post_init_timer_;
    bool enable_local_psm_ = false;
    
    // Store last received poses for arm up/down functionality
    std::vector<double> last_left_pose_rpy_;
    std::vector<double> last_right_pose_rpy_;
    bool left_pose_received_ = false;
    bool right_pose_received_ = false;
    bool joint_state_ready_ = false;
    bool actions_ready_ = false;
    std::vector<double> pending_dual_pose_;
    bool has_pending_dual_pose_ = false;
    
    // Configurable z-coordinate values for arm up/down
    double arm_up_z_value_ = 0.2;   // Absolute z coordinate for arm up position
    double arm_down_z_value_ = 0.086; // Absolute z coordinate for arm down position
    
    // Action clients for direct trajectory execution
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

    // Dual-arm sync state
    std::mutex sync_mutex_;
    geometry_msgs::msg::PoseStamped pending_left_pose_;
    geometry_msgs::msg::PoseStamped pending_right_pose_;
    bool has_pending_left_pose_ = false;
    bool has_pending_right_pose_ = false;
    std::chrono::steady_clock::time_point pending_left_time_;
    std::chrono::steady_clock::time_point pending_right_time_;

    // Shared plan buffer for small lambdas
    moveit::planning_interface::MoveGroupInterface::Plan plan_buffer_;
    
    // Servo control service clients for simple switching
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr left_servo_start_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr left_servo_stop_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr right_servo_start_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr right_servo_stop_client_;
    
    // TF2 for direct pose retrieval
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    bool isReady() const { return joint_state_ready_ && actions_ready_; }

    void attemptFlushPending()
    {
        if (!isReady()) return;
        // Flush dual if pending
        bool do_dual = false;
        std::vector<double> dual;
        {
            std::lock_guard<std::mutex> lk(sync_mutex_);
            if (has_pending_dual_pose_ && pending_dual_pose_.size() == 12) {
                dual = pending_dual_pose_;
                has_pending_dual_pose_ = false;
                do_dual = true;
            }
        }
        if (do_dual) {
            tf2::Quaternion ql; ql.setRPY(dual[3], dual[4], dual[5]);
            geometry_msgs::msg::PoseStamped l_pose;
            l_pose.header.frame_id = "world"; l_pose.header.stamp = this->get_clock()->now();
            l_pose.pose.position.x = dual[0]; l_pose.pose.position.y = dual[1]; l_pose.pose.position.z = dual[2];
            l_pose.pose.orientation = tf2::toMsg(ql);
            tf2::Quaternion qr; qr.setRPY(dual[9], dual[10], dual[11]);
            geometry_msgs::msg::PoseStamped r_pose;
            r_pose.header.frame_id = "world"; r_pose.header.stamp = this->get_clock()->now();
            r_pose.pose.position.x = dual[6]; r_pose.pose.position.y = dual[7]; r_pose.pose.position.z = dual[8];
            r_pose.pose.orientation = tf2::toMsg(qr);
            planAndExecuteSynchronized(l_pose, r_pose);
        }
    }
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
