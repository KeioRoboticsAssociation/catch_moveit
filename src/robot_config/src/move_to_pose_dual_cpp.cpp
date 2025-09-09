#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
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
#include <sensor_msgs/msg/joint_state.hpp>
#include <thread>
#include <memory>
#include <chrono>
#include <mutex>
#include <optional>
#include <cmath>
#include <iomanip>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <numeric>
#include <algorithm>

class MoveToPoseDualCpp : public rclcpp::Node
{
public:
    MoveToPoseDualCpp()
        : Node("move_to_pose_dual_cpp", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
    {
        // Parameters
        this->declare_parameter<bool>("strict_dual_sync", true); // 両腕の計画が揃うまで実行しない
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
            right_move_group_interface_->setPlannerId("RRTConnectkConfigDefault");
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
        
        // Initialize Planning Scene Monitor for real-time robot state updates
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
            
        // Initialize TF2 for direct pose retrieval
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        // Direct joint states control subscribers
        left_direct_joints_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/left_direct_joints", 10, std::bind(&MoveToPoseDualCpp::left_direct_joints_callback, this, std::placeholders::_1));
        right_direct_joints_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/right_direct_joints", 10, std::bind(&MoveToPoseDualCpp::right_direct_joints_callback, this, std::placeholders::_1));
        
        // Joint states recording (1000Hz - high precision like original)
        joint_states_recording_timer_ = this->create_wall_timer(
            std::chrono::microseconds(1000), // 1000Hz = 1ms
            std::bind(&MoveToPoseDualCpp::recordJointStatesCallback, this));
        
        // Joint states subscribers for high-frequency recording
        joint_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&MoveToPoseDualCpp::jointStatesCallback, this, std::placeholders::_1));
            
        // YAML trajectory loading subscribers
        left_load_yaml_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/left_load_trajectory", 10, std::bind(&MoveToPoseDualCpp::left_load_yaml_callback, this, std::placeholders::_1));
        right_load_yaml_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/right_load_trajectory", 10, std::bind(&MoveToPoseDualCpp::right_load_yaml_callback, this, std::placeholders::_1));
        
        // Unified trajectory loading subscriber
        unified_load_yaml_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/dual_load_trajectory", 10, std::bind(&MoveToPoseDualCpp::unified_load_yaml_callback, this, std::placeholders::_1));
        
        // Joint trajectory publishers
        left_joint_trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/left_arm_controller/joint_trajectory", 10);
        right_joint_trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/right_arm_controller/joint_trajectory", 10);
        
        // 1000Hz recording control subscribers
        start_recording_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/start_joint_recording", 10, std::bind(&MoveToPoseDualCpp::startRecordingCallback, this, std::placeholders::_1));
        stop_recording_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/stop_joint_recording", 10, std::bind(&MoveToPoseDualCpp::stopRecordingCallback, this, std::placeholders::_1));
            
        RCLCPP_INFO(this->get_logger(), "Simple servo switching system initialized");
        RCLCPP_INFO(this->get_logger(), "Direct joint states control ready on /left_direct_joints and /right_direct_joints");
        RCLCPP_INFO(this->get_logger(), "YAML trajectory loading ready on /left_load_trajectory, /right_load_trajectory, and /dual_load_trajectory");
        RCLCPP_INFO(this->get_logger(), "🎯 1000Hz precision joint states recording ready on /start_joint_recording and /stop_joint_recording");
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

        // Try synchronized execution with right arm if available
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

        // Try synchronized execution with left arm if available
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
            // Allow a brief pause to ensure servo node actually stops and state settles
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            
            // Apply dynamic collision avoidance settings
            applyDynamicCollisionAvoidance(move_group_interface, arm_name);
            
            // Always plan from the true current robot state
            try {
                move_group_interface->setStartStateToCurrentState();
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "Failed to set start state to current for %s arm: %s", arm_name.c_str(), e.what());
            }

            // Clear previous pose targets to avoid residual constraints
            move_group_interface->clearPoseTargets();
            move_group_interface->setPoseTarget(target_pose);
            
            // プランナーをOMPLのRRTConnectに指定
            move_group_interface->setPlanningPipelineId("ompl");
            // 探索方法が異なるプランナーの組み合わせ
            // std::vector<std::string> planners = {"RRTConnectkConfigDefault", "PRMkConfigDefault", "ESTkConfigDefault", "BKPIECEkConfigDefault"};
            std::vector<std::string> planners = {"RRTStarkConfigDefault"};
            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            bool success = false;
            
            for (const auto& planner : planners) {
                move_group_interface->setPlannerId(planner);
                // 全プランナーで統一された高精度設定
                move_group_interface->setGoalPositionTolerance(0.000000001);   // 0.1mm精度
                move_group_interface->setGoalOrientationTolerance(0.0000001); // 0.1mm精度
                move_group_interface->setNumPlanningAttempts(10);

                success = (move_group_interface->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
                if (success) {
                    RCLCPP_INFO(this->get_logger(), "%s planner succeeded for %s with 0.001 tolerance", planner.c_str(), arm_name.c_str());
                    break;
                } else {
                    RCLCPP_WARN(this->get_logger(), "%s planner failed for %s, trying next...", planner.c_str(), arm_name.c_str());
                }
            }

            if (success)
            {
                // CHOMP最適化を無効化（10秒の遅延を防ぐため）
                RCLCPP_INFO(this->get_logger(), "Planner found a plan for %s, executing directly (optimization disabled for speed)", arm_name.c_str());
                
                // 軌道を自動保存（統合記録は軽量化）
                saveTrajectoryToYaml(my_plan, arm_name);
                
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

    void execute_trajectory_directly(
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface,
        const moveit::planning_interface::MoveGroupInterface::Plan& plan,
        const std::string& arm_name)
    {
        // Start automatic recording when trajectory execution begins
        startJointStatesRecording();
        
        // アクションクライアントを選択
        auto action_client = (arm_name == "left") ? left_arm_action_client_ : right_arm_action_client_;
        
        if (!action_client) {
            RCLCPP_ERROR(this->get_logger(), "Action client not initialized for %s arm", arm_name.c_str());
            is_recording_joint_states_ = false;
            return;
        }

        // アクションサーバーを待機
        if (!action_client->wait_for_action_server(std::chrono::milliseconds(100))) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available for %s arm", arm_name.c_str());
            is_recording_joint_states_ = false;
            return;
        }

        // ゴールを作成（先頭に現在状態のポイントを追加してスナップ回避）
        auto goal_msg = FollowJointTrajectory::Goal();
        goal_msg.trajectory = plan.trajectory_.joint_trajectory;

        try {
            // 取得した現在関節値を先頭ポイントとして追加
            std::vector<double> current_positions;
            current_positions = move_group_interface->getCurrentJointValues();

            if (!goal_msg.trajectory.joint_names.empty() &&
                !goal_msg.trajectory.points.empty() &&
                current_positions.size() == goal_msg.trajectory.points[0].positions.size()) {

                const auto& first = goal_msg.trajectory.points.front();
                double diff_norm = 0.0;
                for (size_t i = 0; i < first.positions.size(); ++i) {
                    double d = first.positions[i] - current_positions[i];
                    diff_norm += d * d;
                }

                if (diff_norm > 1e-8) {
                    trajectory_msgs::msg::JointTrajectoryPoint start_pt;
                    start_pt.positions = current_positions;
                    start_pt.time_from_start = rclcpp::Duration::from_seconds(0.0);

                    // 挿入し、元の先頭が0秒なら少し先へずらす
                    auto original_first = goal_msg.trajectory.points.front();
                    if (original_first.time_from_start.sec == 0 && original_first.time_from_start.nanosec == 0) {
                        original_first.time_from_start = rclcpp::Duration::from_seconds(0.05);
                        goal_msg.trajectory.points[0] = original_first;
                    }
                    goal_msg.trajectory.points.insert(goal_msg.trajectory.points.begin(), start_pt);
                }
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Failed to prepend current state point for %s arm: %s", arm_name.c_str(), e.what());
        }

        // 非同期で送信
        auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            [this, arm_name](auto) {
                RCLCPP_INFO(this->get_logger(), "Goal accepted by %s arm controller", arm_name.c_str());
            };
        send_goal_options.feedback_callback =
            [this, arm_name](rclcpp_action::ClientGoalHandle<FollowJointTrajectory>::SharedPtr,
                             const std::shared_ptr<const FollowJointTrajectory::Feedback>) {
                // Start unified recording at the moment feedback arrives (movement started)
                if (!is_recording_unified_) {
                    startUnifiedTrajectoryRecording("dual");
                }
            };
        send_goal_options.result_callback =
            [this, arm_name](const rclcpp_action::ClientGoalHandle<FollowJointTrajectory>::WrappedResult & result) {
                RCLCPP_INFO(this->get_logger(), "Trajectory execution completed for %s arm", arm_name.c_str());
                
                // Stop automatic recording when trajectory execution completes
                stopJointStatesRecording(arm_name);
                
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

        // If neither arm is executing, stop unified recording immediately
        if (!left_arm_executing_ && !right_arm_executing_) {
            if (is_recording_unified_) {
                stopUnifiedTrajectoryRecording();
            }
        }
    }

    // ---- Strict synchronization for dual-arm starts ----
    void handleIncomingTargetPose(const std::string& arm_name, const geometry_msgs::msg::PoseStamped& target_pose)
    {
        const auto now_tp = std::chrono::steady_clock::now();
        const bool strict_sync = this->get_parameter("strict_dual_sync").as_bool();

        bool should_plan_sync = false;
        geometry_msgs::msg::PoseStamped left_pose, right_pose;

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
                // strictモード: 到着時間差に関係なく揃い次第実行
                left_pose = pending_left_pose_;
                right_pose = pending_right_pose_;
                has_pending_left_pose_ = false;
                has_pending_right_pose_ = false;
                should_plan_sync = true;
            }
        }

        if (should_plan_sync) {
            std::thread([this, left_pose, right_pose]() {
                planAndExecuteSynchronized(left_pose, right_pose);
            }).detach();
            return;
        }

        // strictモードでは相手が来るまで待機（単独実行しない）
        if (!strict_sync) {
            // 非strictモードのみ単独実行のフォールバック
            std::thread([this, arm_name, target_pose, now_tp]() {
                std::this_thread::sleep_for(std::chrono::milliseconds(220));
                bool still_unpaired = false;
                {
                    std::lock_guard<std::mutex> lk(sync_mutex_);
                    if (arm_name == "left") {
                        still_unpaired = has_pending_left_pose_ && (!has_pending_right_pose_ || (now_tp == pending_left_time_));
                        if (still_unpaired) has_pending_left_pose_ = false;
                    } else {
                        still_unpaired = has_pending_right_pose_ && (!has_pending_left_pose_ || (now_tp == pending_right_time_));
                        if (still_unpaired) has_pending_right_pose_ = false;
                    }
                }
                if (still_unpaired) {
                    if (arm_name == "left") move_to_pose(left_move_group_interface_, target_pose);
                    else move_to_pose(right_move_group_interface_, target_pose);
                }
            }).detach();
        }
    }

    void planAndExecuteSynchronized(const geometry_msgs::msg::PoseStamped& left_pose,
                                    const geometry_msgs::msg::PoseStamped& right_pose)
    {
        RCLCPP_INFO(this->get_logger(), "Planning synchronized dual-arm motion");

        // Stop both servos briefly
        stopServo("left");
        stopServo("right");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // Prepare planners (simple, consistent settings)
        auto plan_one = [this](std::shared_ptr<moveit::planning_interface::MoveGroupInterface> mgi,
                               const geometry_msgs::msg::PoseStamped& pose,
                               const std::string& arm) -> std::optional<moveit::planning_interface::MoveGroupInterface::Plan>
        {
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            try { mgi->setStartStateToCurrentState(); } catch (...) {}
            mgi->clearPoseTargets();
            mgi->setPoseTarget(pose);
            mgi->setPlanningPipelineId("ompl");
            mgi->setPlannerId("RRTstarKConfigDefault");
            mgi->setGoalPositionTolerance(0.00001);
            mgi->setGoalOrientationTolerance(0.00001);
            mgi->setPlanningTime(0.1);
            mgi->setNumPlanningAttempts(10);
            bool ok = (mgi->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
            if (!ok) return std::nullopt;
            return plan;
        };

        auto left_plan_opt = plan_one(left_move_group_interface_, left_pose, "left");
        auto right_plan_opt = plan_one(right_move_group_interface_, right_pose, "right");

        if (!left_plan_opt || !right_plan_opt) {
            RCLCPP_WARN(this->get_logger(), "Synchronized planning: one or both plans failed. Falling back to individual execution.");
            if (left_plan_opt) execute_trajectory_directly(left_move_group_interface_, *left_plan_opt, "left");
            if (right_plan_opt) execute_trajectory_directly(right_move_group_interface_, *right_plan_opt, "right");
            return;
        }

        // Build synchronized goals with common future stamp and prepend current state point
        auto now_ros = this->get_clock()->now();
        rclcpp::Time start_time = now_ros + rclcpp::Duration::from_seconds(0.25);

        auto make_goal = [this, start_time](std::shared_ptr<moveit::planning_interface::MoveGroupInterface> mgi,
                                            const moveit::planning_interface::MoveGroupInterface::Plan& plan,
                                            const std::string& arm) -> FollowJointTrajectory::Goal
        {
            FollowJointTrajectory::Goal goal;
            goal.trajectory = plan.trajectory_.joint_trajectory;
            // header stamp unified
            goal.trajectory.header.stamp = start_time;

            // prepend current state point if necessary
            try {
                std::vector<double> curr = mgi->getCurrentJointValues();
                if (!goal.trajectory.points.empty() && curr.size() == goal.trajectory.points[0].positions.size()) {
                    const auto& f = goal.trajectory.points.front();
                    double dn = 0.0;
                    for (size_t i=0;i<f.positions.size();++i){ double d=f.positions[i]-curr[i]; dn+=d*d; }
                    if (dn > 1e-8) {
                        trajectory_msgs::msg::JointTrajectoryPoint p0;
                        p0.positions = curr;
                        p0.time_from_start = rclcpp::Duration::from_seconds(0.0);
                        auto fmod = f;
                        if (fmod.time_from_start.sec==0 && fmod.time_from_start.nanosec==0) {
                            fmod.time_from_start = rclcpp::Duration::from_seconds(0.05);
                            goal.trajectory.points[0] = fmod;
                        }
                        goal.trajectory.points.insert(goal.trajectory.points.begin(), p0);
                    }
                }
            } catch (...) {}

            return goal;
        };

        auto left_goal = make_goal(left_move_group_interface_, *left_plan_opt, "left");
        auto right_goal = make_goal(right_move_group_interface_, *right_plan_opt, "right");

        // Send both goals
        auto send_one = [this](const std::string& arm,
                               rclcpp_action::Client<FollowJointTrajectory>::SharedPtr client,
                               const FollowJointTrajectory::Goal& goal,
                               const moveit::planning_interface::MoveGroupInterface::Plan& plan)
        {
            if (!client->wait_for_action_server(std::chrono::seconds(2))) {
                RCLCPP_ERROR(this->get_logger(), "Action server not available for %s arm", arm.c_str());
                return;
            }
            auto options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
            options.goal_response_callback = [this, arm](auto){ RCLCPP_INFO(this->get_logger(), "[SYNC] Goal accepted by %s", arm.c_str()); };
            options.feedback_callback = [this](auto, const std::shared_ptr<const FollowJointTrajectory::Feedback>) {
                if (!is_recording_unified_) {
                    startUnifiedTrajectoryRecording("dual");
                }
            };
            options.result_callback = [this, arm](auto){ RCLCPP_INFO(this->get_logger(), "[SYNC] Execution done for %s", arm.c_str()); stopTrajectoryTracking(arm); std::this_thread::sleep_for(std::chrono::milliseconds(100)); startServo(arm); };
            client->async_send_goal(goal, options);
            updateTrajectoryTracking(plan, arm);
        };

        send_one("left", left_arm_action_client_, left_goal, *left_plan_opt);
        send_one("right", right_arm_action_client_, right_goal, *right_plan_opt);

        RCLCPP_INFO(this->get_logger(), "Synchronized dual-arm trajectories sent with common start time");
    }
    
    // 軌道最適化関数
    bool optimizeTrajectory(
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface,
        const moveit::planning_interface::MoveGroupInterface::Plan& original_plan,
        moveit::planning_interface::MoveGroupInterface::Plan& optimized_plan,
        const std::string& arm_name)
    {
        // 超高速最適化開始（ログ最小限）
        
        // 最適化手法のリスト（優先順位順・高速化のためCHOMPのみ）
        std::vector<std::string> optimization_methods = {"chomp"};  // STOMPはより時間がかかるため除外
        
        for (const auto& method : optimization_methods) {
            // ログは最小限に（高速化のため）
            
            try {
                // プランニングパイプラインを最適化手法に切り替え
                move_group_interface->setPlanningPipelineId(method);
                
                // 最適化のためのパラメータ設定（超高速・軽量化）
                if (method == "chomp") {
                    move_group_interface->setPlanningTime(0.05);  // CHOMP超高速最適化
                    move_group_interface->setNumPlanningAttempts(1);  // 1回のみの試行
                    move_group_interface->setMaxVelocityScalingFactor(1.0);
                    move_group_interface->setMaxAccelerationScalingFactor(1.0);
                } else if (method == "stomp") {
                    move_group_interface->setPlanningTime(0.05); // STOMP超高速最適化
                    move_group_interface->setNumPlanningAttempts(1);  // 1回のみの試行
                    move_group_interface->setMaxVelocityScalingFactor(1.0);
                    move_group_interface->setMaxAccelerationScalingFactor(1.0);
                }
                
                // 元の軌道の終点を目標として設定
                const auto& last_point = original_plan.trajectory_.joint_trajectory.points.back();
                move_group_interface->setJointValueTarget(last_point.positions);
                
                // 軌道計画実行
                bool success = (move_group_interface->plan(optimized_plan) == moveit::core::MoveItErrorCode::SUCCESS);
                
                if (success) {
                    // 最適化成功（計算は省略して高速化）
                    RCLCPP_INFO(this->get_logger(), "%s optimization ✓ for %s arm", method.c_str(), arm_name.c_str());
                    return true;
                } else {
                    // 失敗時は静かに次へ（高速化のため）
                    continue;
                }
                
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Exception during %s optimization for %s arm: %s", 
                            method.c_str(), arm_name.c_str(), e.what());
            }
        }
        
        // 最適化失敗（静かに元の軌道を使用）
        return false;
    }
    
    // 軌道の長さを計算する補助関数
    double calculateTrajectoryLength(const moveit::planning_interface::MoveGroupInterface::Plan& plan)
    {
        double total_length = 0.0;
        const auto& points = plan.trajectory_.joint_trajectory.points;
        
        for (size_t i = 1; i < points.size(); ++i) {
            double segment_length = 0.0;
            for (size_t j = 0; j < points[i].positions.size(); ++j) {
                double diff = points[i].positions[j] - points[i-1].positions[j];
                segment_length += diff * diff;
            }
            total_length += std::sqrt(segment_length);
        }
        
        return total_length;
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
            
            // より慎重なプランニング設定
            move_group_interface->setGoalPositionTolerance(0.0001);
            move_group_interface->setGoalOrientationTolerance(0.0001);
            move_group_interface->setPlanningTime(0.3);
            move_group_interface->setNumPlanningAttempts(2);
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
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            
            // Configure Pilz LIN planner with simple settings
            move_group_interface->setPlanningPipelineId("pilz_industrial_motion_planner");
            move_group_interface->setPlannerId("LIN");
            move_group_interface->setPlanningTime(0.1);
            move_group_interface->setNumPlanningAttempts(3);
            
            // Check current state before planning
            geometry_msgs::msg::PoseStamped current_pose_check = move_group_interface->getCurrentPose();
            RCLCPP_INFO(this->get_logger(), "Pre-planning check - Current: x=%.3f, y=%.3f, z=%.3f", 
                       current_pose_check.pose.position.x, current_pose_check.pose.position.y, current_pose_check.pose.position.z);
            
            // Always plan from current state
            try { move_group_interface->setStartStateToCurrentState(); } catch (...) {}

            move_group_interface->clearPoseTargets();
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
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
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
    std::mutex save_mutex_;
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
    
    // TF2 for direct pose retrieval
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // Direct joint states control subscribers
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr left_direct_joints_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr right_direct_joints_sub_;
    
    // YAML trajectory loading subscribers
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr left_load_yaml_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr right_load_yaml_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr unified_load_yaml_sub_;
    
    // Joint trajectory publishers for direct control
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr left_joint_trajectory_pub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr right_joint_trajectory_pub_;
    
    // 1000Hz recording control subscribers
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr start_recording_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr stop_recording_sub_;
    
    // High-frequency joint states recording with precision timestamps
    rclcpp::TimerBase::SharedPtr joint_states_recording_timer_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
    sensor_msgs::msg::JointState::SharedPtr latest_joint_states_;
    std::vector<sensor_msgs::msg::JointState> recorded_joint_states_;
    std::mutex joint_states_mutex_;
    bool is_recording_joint_states_ = false;

    // --- Dual-arm strict sync buffers ---
    geometry_msgs::msg::PoseStamped pending_left_pose_;
    geometry_msgs::msg::PoseStamped pending_right_pose_;
    std::chrono::steady_clock::time_point pending_left_time_;
    std::chrono::steady_clock::time_point pending_right_time_;
    bool has_pending_left_pose_ = false;
    bool has_pending_right_pose_ = false;
    std::mutex sync_mutex_;
    
    // Direct joint states control callbacks
    void left_direct_joints_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (msg->data.size() != 7) {
            RCLCPP_ERROR(this->get_logger(), "Left arm requires exactly 7 joint values, received %zu", msg->data.size());
            return;
        }
        
        executeDirectJoints("left", msg->data);
    }
    
    void right_direct_joints_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (msg->data.size() != 7) {
            RCLCPP_ERROR(this->get_logger(), "Right arm requires exactly 7 joint values, received %zu", msg->data.size());
            return;
        }
        
        executeDirectJoints("right", msg->data);
    }
    
    // Direct joint states execution function
    void executeDirectJoints(const std::string& arm_name, const std::vector<double>& joint_values)
    {
        RCLCPP_INFO(this->get_logger(), "Executing direct joint states for %s arm", arm_name.c_str());
        
        // Create a simple trajectory with just the target joint positions
        trajectory_msgs::msg::JointTrajectory trajectory;
        
        // Set joint names based on arm
        if (arm_name == "left") {
            trajectory.joint_names = {"left_Revolute_1", "left_Revolute_2", "left_Revolute_3", 
                                     "left_Revolute_4", "left_Revolute_5", "left_Revolute_6", "left_Revolute_7"};
        } else {
            trajectory.joint_names = {"right_Revolute_1", "right_Revolute_2", "right_Revolute_3", 
                                     "right_Revolute_4", "right_Revolute_5", "right_Revolute_6", "right_Revolute_7"};
        }
        
        // Create trajectory point
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = joint_values;
        point.time_from_start = rclcpp::Duration::from_seconds(2.0);  // 2秒で移動
        trajectory.points.push_back(point);
        
        // Create action goal
        auto goal = FollowJointTrajectory::Goal();
        goal.trajectory = trajectory;
        
        // Select appropriate action client
        auto action_client = (arm_name == "left") ? left_arm_action_client_ : right_arm_action_client_;
        
        if (!action_client->wait_for_action_server(std::chrono::seconds(1))) {
            RCLCPP_ERROR(this->get_logger(), "Action server for %s arm not available", arm_name.c_str());
            return;
        }
        
        // Track execution and send with feedback
        moveit::planning_interface::MoveGroupInterface::Plan direct_plan;
        direct_plan.trajectory_.joint_trajectory = trajectory;
        updateTrajectoryTracking(direct_plan, arm_name);

        auto options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
        options.feedback_callback = [this](auto, const std::shared_ptr<const FollowJointTrajectory::Feedback>) {
            if (!is_recording_unified_) startUnifiedTrajectoryRecording("dual");
        };
        options.result_callback = [this, arm_name](auto){ RCLCPP_INFO(this->get_logger(), "Execution done for %s (direct joints)", arm_name.c_str()); stopTrajectoryTracking(arm_name); std::this_thread::sleep_for(std::chrono::milliseconds(100)); startServo(arm_name); };
        action_client->async_send_goal(goal, options);
        RCLCPP_INFO(this->get_logger(), "Direct joint trajectory sent to %s arm controller", arm_name.c_str());
        
        // Save the direct joint trajectory to YAML for future use
        saveTrajectoryToYaml(direct_plan, arm_name + "_direct");
    }
    
    // YAML trajectory loading callbacks
    void left_load_yaml_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        loadAndExecuteTrajectory(msg->data, "left");
    }
    
    void right_load_yaml_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        loadAndExecuteTrajectory(msg->data, "right");
    }
    
    // Unified trajectory loading callback
    void unified_load_yaml_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        loadAndExecuteUnifiedTrajectory(msg->data);
    }
    
    // Load and execute unified trajectory from YAML file (both arms simultaneously)
    void loadAndExecuteUnifiedTrajectory(const std::string& filename)
    {
        RCLCPP_INFO(this->get_logger(), "Loading unified trajectory from %s for both arms", filename.c_str());
        
        try {
            YAML::Node yaml_node = YAML::LoadFile(filename);
            
            // Check if it's a unified dual-arm trajectory
            if (!yaml_node["unified_dual_arm"] || !yaml_node["unified_dual_arm"]["trajectory"]) {
                RCLCPP_ERROR(this->get_logger(), "File %s is not a valid unified dual-arm trajectory", filename.c_str());
                return;
            }
            
            auto trajectory_data = yaml_node["unified_dual_arm"]["trajectory"];
            
            // Create separate trajectories for left and right arms
            trajectory_msgs::msg::JointTrajectory left_trajectory, right_trajectory;
            
            // Set joint names for each arm
            left_trajectory.joint_names = {
                "left_Revolute_1", "left_Revolute_2", "left_Revolute_3", 
                "left_Revolute_4", "left_Revolute_5", "left_Revolute_6"
            };
            right_trajectory.joint_names = {
                "right_Revolute_1", "right_Revolute_2", "right_Revolute_3", 
                "right_Revolute_4", "right_Revolute_5", "right_Revolute_6"
            };
            
            // Extract points from unified trajectory
            if (trajectory_data["points"]) {
                for (const auto& point_data : trajectory_data["points"]) {
                    trajectory_msgs::msg::JointTrajectoryPoint left_point, right_point;
                    
                    // Extract positions and velocities
                    if (point_data["positions"] && point_data["positions"].size() >= 12) {
                        // Left arm: indices 0-5
                        for (int i = 0; i < 6; ++i) {
                            left_point.positions.push_back(point_data["positions"][i].as<double>());
                        }
                        // Right arm: indices 6-11  
                        for (int i = 6; i < 12; ++i) {
                            right_point.positions.push_back(point_data["positions"][i].as<double>());
                        }
                    }
                    
                    if (point_data["velocities"] && point_data["velocities"].size() >= 12) {
                        // Left arm: indices 0-5
                        for (int i = 0; i < 6; ++i) {
                            left_point.velocities.push_back(point_data["velocities"][i].as<double>());
                        }
                        // Right arm: indices 6-11
                        for (int i = 6; i < 12; ++i) {
                            right_point.velocities.push_back(point_data["velocities"][i].as<double>());
                        }
                    }
                    
                    // Set time from start
                    if (point_data["time_from_start"]) {
                        rclcpp::Duration duration(
                            point_data["time_from_start"]["sec"].as<int32_t>(),
                            point_data["time_from_start"]["nanosec"].as<uint32_t>()
                        );
                        left_point.time_from_start = duration;
                        right_point.time_from_start = duration;
                    }
                    
                    left_trajectory.points.push_back(left_point);
                    right_trajectory.points.push_back(right_point);
                }
            }
            
            RCLCPP_INFO(this->get_logger(), "Successfully loaded unified trajectory with %zu points", 
                       left_trajectory.points.size());
            
            // Set trajectory headers
            left_trajectory.header.stamp = this->get_clock()->now();
            left_trajectory.header.frame_id = "world";
            right_trajectory.header.stamp = this->get_clock()->now();
            right_trajectory.header.frame_id = "world";
            
            // Publish trajectories simultaneously
            left_joint_trajectory_pub_->publish(left_trajectory);
            right_joint_trajectory_pub_->publish(right_trajectory);
            
            RCLCPP_INFO(this->get_logger(), "🤖✅ Published unified dual-arm trajectory to controllers");
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load unified trajectory: %s", e.what());
        }
    }
    
    // Load and execute trajectory from YAML file
    void loadAndExecuteTrajectory(const std::string& filename, const std::string& arm_name)
    {
        RCLCPP_INFO(this->get_logger(), "Loading trajectory from %s for %s arm", filename.c_str(), arm_name.c_str());
        
        moveit::planning_interface::MoveGroupInterface::Plan loaded_plan;
        if (loadTrajectoryFromYaml(filename, loaded_plan)) {
            RCLCPP_INFO(this->get_logger(), "Successfully loaded trajectory with %zu points for %s arm", 
                       loaded_plan.trajectory_.joint_trajectory.points.size(), arm_name.c_str());
            
            // 直接アクションサーバーを使用して実行
            executeTrajectoryDirectly(loaded_plan.trajectory_.joint_trajectory, arm_name);
            
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to load trajectory from %s for %s arm", filename.c_str(), arm_name.c_str());
        }
    }
    
    // Direct trajectory execution using action server
    void executeTrajectoryDirectly(const trajectory_msgs::msg::JointTrajectory& trajectory, const std::string& arm_name)
    {
        RCLCPP_INFO(this->get_logger(), "Executing trajectory directly for %s arm with %zu points", 
                   arm_name.c_str(), trajectory.points.size());
        
        // Create action goal
        auto goal = FollowJointTrajectory::Goal();
        goal.trajectory = trajectory;
        
        // Select appropriate action client
        auto action_client = (arm_name == "left") ? left_arm_action_client_ : right_arm_action_client_;
        
        if (!action_client->wait_for_action_server(std::chrono::seconds(2))) {
            RCLCPP_ERROR(this->get_logger(), "Action server for %s arm not available", arm_name.c_str());
            return;
        }
        
        // Print trajectory details for debugging
        RCLCPP_INFO(this->get_logger(), "Joint names: [%s]", 
                   std::accumulate(trajectory.joint_names.begin(), trajectory.joint_names.end(), std::string{},
                   [](const std::string& a, const std::string& b) { return a.empty() ? b : a + ", " + b; }).c_str());
        
        if (!trajectory.points.empty()) {
            const auto& first_point = trajectory.points[0];
            RCLCPP_INFO(this->get_logger(), "First point positions: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                       first_point.positions[0], first_point.positions[1], first_point.positions[2],
                       first_point.positions[3], first_point.positions[4], first_point.positions[5], first_point.positions[6]);
        }
        
        // Track execution state to allow proper unified recording stop
        moveit::planning_interface::MoveGroupInterface::Plan pseudo_plan;
        pseudo_plan.trajectory_.joint_trajectory = trajectory;
        updateTrajectoryTracking(pseudo_plan, arm_name);

        // Send goal with feedback to start recording when motion starts
        auto options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
        options.feedback_callback = [this](auto, const std::shared_ptr<const FollowJointTrajectory::Feedback>) {
            if (!is_recording_unified_) startUnifiedTrajectoryRecording("dual");
        };
        options.result_callback = [this, arm_name](auto){ RCLCPP_INFO(this->get_logger(), "Execution done for %s (direct)", arm_name.c_str()); stopTrajectoryTracking(arm_name); std::this_thread::sleep_for(std::chrono::milliseconds(100)); startServo(arm_name); };
        action_client->async_send_goal(goal, options);
        RCLCPP_INFO(this->get_logger(), "Trajectory goal sent to %s arm controller", arm_name.c_str());
    }
    
    // Joint states callback for high-frequency recording
    void jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(joint_states_mutex_);
        latest_joint_states_ = msg;
        
        // 統合記録がアクティブな場合、両アームのjoint_statesを記録
        std::lock_guard<std::mutex> unified_lock(unified_recording_mutex_);
        if (is_recording_unified_) {
            unified_joint_states_recording_.push_back(*msg);
        }
    }
    
    // 1000Hz recording callback（最適化版）
    void recordJointStatesCallback()
    {
        // 早期リターンで処理軽減
        if (!is_recording_joint_states_ && !is_recording_unified_) {
            return;
        }
        
        // latest_joint_states_の取得は最小限のロック時間で
        sensor_msgs::msg::JointState current_state;
        {
            std::lock_guard<std::mutex> lock(joint_states_mutex_);
            if (!latest_joint_states_) {
                return;
            }
            current_state = *latest_joint_states_; // コピー
        }
        
        // 従来の個別記録
        if (is_recording_joint_states_) {
            recorded_joint_states_.push_back(current_state);
        }
        
        // 統合記録（try_lockで非ブロッキング）
        if (is_recording_unified_) {
            std::unique_lock<std::mutex> unified_lock(unified_recording_mutex_, std::try_to_lock);
            if (unified_lock.owns_lock()) {
                unified_joint_states_recording_.push_back(current_state);
            }
        }
    }
    
    
    // Start simple joint states recording
    void startJointStatesRecording()
    {
        std::lock_guard<std::mutex> lock(joint_states_mutex_);
        recorded_joint_states_.clear();
        is_recording_joint_states_ = true;
        RCLCPP_INFO(this->get_logger(), "🎯 Started recording joint states at 1000Hz");
    }
    
    // Stop recording and save simple trajectory to YAML
    void stopJointStatesRecording(const std::string& arm_name)
    {
        std::lock_guard<std::mutex> lock(joint_states_mutex_);
        is_recording_joint_states_ = false;
        
        if (recorded_joint_states_.empty()) {
            RCLCPP_WARN(this->get_logger(), "No joint states recorded");
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "🎯 Stopped recording: %zu joint states (%.3fs total)", 
                   recorded_joint_states_.size(), recorded_joint_states_.size() * 0.001);
        
        // Create trajectory with original 1000Hz timing (1ms intervals)  
        trajectory_msgs::msg::JointTrajectory trajectory;
        trajectory.joint_names = recorded_joint_states_[0].name;
        
        for (size_t i = 0; i < recorded_joint_states_.size(); ++i) {
            trajectory_msgs::msg::JointTrajectoryPoint point;
            point.positions = recorded_joint_states_[i].position;
            
            // Original 1ms intervals to match recording frequency
            point.time_from_start = rclcpp::Duration::from_seconds(i * 0.001);
            
            trajectory.points.push_back(point);
        }
        
        // Save to YAML
        moveit::planning_interface::MoveGroupInterface::Plan recorded_plan;
        recorded_plan.trajectory_.joint_trajectory = trajectory;
        saveTrajectoryToYaml(recorded_plan, arm_name);
    }
    
    // Recording control callbacks
    void startRecordingCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        startJointStatesRecording();
    }
    
    void stopRecordingCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        stopJointStatesRecording(msg->data); // msg->data should contain arm name (e.g., "left" or "right")
    }

    // Convert trajectory to 1000Hz precision
    trajectory_msgs::msg::JointTrajectory resampleTrajectoryTo1000Hz(const trajectory_msgs::msg::JointTrajectory& original_trajectory) {
        trajectory_msgs::msg::JointTrajectory resampled_trajectory;
        resampled_trajectory.joint_names = original_trajectory.joint_names;
        
        if (original_trajectory.points.empty()) {
            return resampled_trajectory;
        }
        
        // Calculate total trajectory time
        double total_time = original_trajectory.points.back().time_from_start.sec + 
                           original_trajectory.points.back().time_from_start.nanosec * 1e-9;
        
        // Generate 1000Hz samples (every 0.001 seconds)
        const double dt = 0.001; // 1000Hz
        int num_samples = static_cast<int>(total_time / dt) + 1;
        
        RCLCPP_INFO(this->get_logger(), "Resampling trajectory from %zu points to %d points at 1000Hz over %.2f seconds", 
                   original_trajectory.points.size(), num_samples, total_time);
        
        for (int i = 0; i < num_samples; ++i) {
            double target_time = i * dt;
            trajectory_msgs::msg::JointTrajectoryPoint interpolated_point;
            
            // Find surrounding points in original trajectory
            size_t idx = 0;
            for (size_t j = 0; j < original_trajectory.points.size() - 1; ++j) {
                double point_time = original_trajectory.points[j].time_from_start.sec + 
                                   original_trajectory.points[j].time_from_start.nanosec * 1e-9;
                if (point_time <= target_time) {
                    idx = j;
                } else {
                    break;
                }
            }
            
            if (idx >= original_trajectory.points.size() - 1) {
                // Use last point
                interpolated_point = original_trajectory.points.back();
            } else {
                // Linear interpolation between points
                const auto& p1 = original_trajectory.points[idx];
                const auto& p2 = original_trajectory.points[idx + 1];
                
                double t1 = p1.time_from_start.sec + p1.time_from_start.nanosec * 1e-9;
                double t2 = p2.time_from_start.sec + p2.time_from_start.nanosec * 1e-9;
                
                double alpha = (target_time - t1) / (t2 - t1);
                if (alpha < 0) alpha = 0;
                if (alpha > 1) alpha = 1;
                
                // Interpolate positions
                interpolated_point.positions.resize(p1.positions.size());
                for (size_t j = 0; j < p1.positions.size(); ++j) {
                    interpolated_point.positions[j] = p1.positions[j] + alpha * (p2.positions[j] - p1.positions[j]);
                }
                
                // Interpolate velocities if available
                if (!p1.velocities.empty() && !p2.velocities.empty()) {
                    interpolated_point.velocities.resize(p1.velocities.size());
                    for (size_t j = 0; j < p1.velocities.size(); ++j) {
                        interpolated_point.velocities[j] = p1.velocities[j] + alpha * (p2.velocities[j] - p1.velocities[j]);
                    }
                }
            }
            
            // Set time
            interpolated_point.time_from_start = rclcpp::Duration::from_seconds(target_time);
            resampled_trajectory.points.push_back(interpolated_point);
        }
        
        return resampled_trajectory;
    }

    // 統合軌道記録用の変数
    std::vector<sensor_msgs::msg::JointState> unified_joint_states_recording_;
    std::mutex unified_recording_mutex_;
    bool is_recording_unified_ = false;
    
    // 統合された軌道記録を開始（軽量版）
    void startUnifiedTrajectoryRecording(const std::string& arm_name) {
        // 非同期で記録開始（メインスレッドをブロックしない）
        std::thread([this, arm_name]() {
            std::lock_guard<std::mutex> lock(unified_recording_mutex_);
            if (!is_recording_unified_) {
                unified_joint_states_recording_.clear();
                unified_joint_states_recording_.reserve(10000); // メモリ予約でパフォーマンス向上
                is_recording_unified_ = true;
                RCLCPP_INFO(this->get_logger(), "🔴 Started unified recording (%s)", arm_name.c_str());
            }
        }).detach();
    }
    
    // 統合された軌道記録を停止して保存
    void stopUnifiedTrajectoryRecording() {
        std::lock_guard<std::mutex> lock(unified_recording_mutex_);
        if (is_recording_unified_) {
            is_recording_unified_ = false;
            RCLCPP_INFO(this->get_logger(), "⏹️ Stopped unified recording, recorded %zu states", 
                       unified_joint_states_recording_.size());
            
            if (!unified_joint_states_recording_.empty()) {
                saveUnifiedTrajectoryToYaml(unified_joint_states_recording_);
            }
        }
    }
    
    // 統合軌道をYAMLに保存
    void saveUnifiedTrajectoryToYaml(const std::vector<sensor_msgs::msg::JointState>& recorded_states) {
        try {
            auto now = this->get_clock()->now();
            std::time_t time_t = now.seconds();
            std::tm* tm = std::localtime(&time_t);
            
            std::stringstream ss;
            ss << "/home/a/ws_moveit2/src/robot_config/path/trajectory_"
               << std::put_time(tm, "%Y%m%d_%H%M%S") << ".yaml";
            std::string filename = ss.str();
            
            // 統合された軌道を作成
            trajectory_msgs::msg::JointTrajectory unified_trajectory;
            
            // 両アームの全ジョイント名を定義（固定順序）
            unified_trajectory.joint_names = {
                "left_Revolute_1", "left_Revolute_2", "left_Revolute_3", 
                "left_Revolute_4", "left_Revolute_5", "left_Revolute_6",
                "right_Revolute_1", "right_Revolute_2", "right_Revolute_3", 
                "right_Revolute_4", "right_Revolute_5", "right_Revolute_6"
            };
            
            // 各記録されたjoint_stateを統合軌道に変換
            for (size_t i = 0; i < recorded_states.size(); ++i) {
                const auto& state = recorded_states[i];
                trajectory_msgs::msg::JointTrajectoryPoint point;
                
                // 12個のジョイント値を順序通りに配置
                point.positions.resize(12, 0.0);
                point.velocities.resize(12, 0.0);
                
                // 記録されたjoint_stateから対応するジョイント値を抽出
                for (size_t j = 0; j < state.name.size(); ++j) {
                    const std::string& joint_name = state.name[j];
                    auto it = std::find(unified_trajectory.joint_names.begin(), 
                                       unified_trajectory.joint_names.end(), joint_name);
                    if (it != unified_trajectory.joint_names.end()) {
                        size_t index = std::distance(unified_trajectory.joint_names.begin(), it);
                        if (j < state.position.size()) point.positions[index] = state.position[j];
                        if (j < state.velocity.size()) point.velocities[index] = state.velocity[j];
                    }
                }
                
                point.time_from_start = rclcpp::Duration::from_seconds(i * 0.001); // 1000Hz = 1ms間隔
                unified_trajectory.points.push_back(point);
            }
            
            // YAMLファイルに保存
            YAML::Node root_node;
            YAML::Node trajectory_node;
            
            // 軌道情報
            trajectory_node["trajectory_info"]["type"] = "unified_dual_arm";
            trajectory_node["trajectory_info"]["timestamp"] = static_cast<int64_t>(now.nanoseconds());
            trajectory_node["trajectory_info"]["total_points"] = unified_trajectory.points.size();
            trajectory_node["trajectory_info"]["sampling_rate"] = "1000Hz";
            trajectory_node["trajectory_info"]["duration_seconds"] = 
                unified_trajectory.points.empty() ? 0.0 : 
                (unified_trajectory.points.back().time_from_start.sec + 
                 unified_trajectory.points.back().time_from_start.nanosec * 1e-9);
            
            // ジョイント名
            YAML::Node joint_names_node;
            for (const auto& name : unified_trajectory.joint_names) {
                joint_names_node.push_back(name);
            }
            trajectory_node["trajectory"]["joint_names"] = joint_names_node;
            
            // 軌道ポイント
            YAML::Node points_node;
            for (const auto& point : unified_trajectory.points) {
                YAML::Node point_node;
                
                YAML::Node positions;
                for (const auto& pos : point.positions) {
                    positions.push_back(pos);
                }
                point_node["positions"] = positions;
                
                YAML::Node velocities;
                for (const auto& vel : point.velocities) {
                    velocities.push_back(vel);
                }
                point_node["velocities"] = velocities;
                
                point_node["time_from_start"]["sec"] = point.time_from_start.sec;
                point_node["time_from_start"]["nanosec"] = point.time_from_start.nanosec;
                
                points_node.push_back(point_node);
            }
            trajectory_node["trajectory"]["points"] = points_node;
            
            root_node["unified_dual_arm"] = trajectory_node;
            
            // ファイル保存
            std::ofstream file(filename);
            file << root_node;
            file.close();
            
            RCLCPP_INFO(this->get_logger(), "🤖✅ Unified dual-arm trajectory saved: %s (%zu points, %.2f seconds)", 
                       filename.c_str(), unified_trajectory.points.size(),
                       trajectory_node["trajectory_info"]["duration_seconds"].as<double>());
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to save unified trajectory: %s", e.what());
        }
    }
    
    // 従来の個別保存機能（軽量化版）
    void saveTrajectoryToYaml(const moveit::planning_interface::MoveGroupInterface::Plan& plan, const std::string& arm_name) {
        // 個別保存は非同期で実行（軌道実行をブロックしない）
        std::thread([this, plan, arm_name]() {
            std::lock_guard<std::mutex> lock(save_mutex_);
        try {
            auto now = this->get_clock()->now();
            std::time_t time_t = now.seconds();
            std::tm* tm = std::localtime(&time_t);
            
            std::stringstream ss;
            ss << "/home/a/ws_moveit2/src/robot_config/path/individual_" << arm_name << "_"
               << std::put_time(tm, "%Y%m%d_%H%M%S") << ".yaml";
            std::string filename = ss.str();
            
            YAML::Node root_node;
            YAML::Node arm_trajectory_node;
            
            trajectory_msgs::msg::JointTrajectory resampled_trajectory = resampleTrajectoryTo1000Hz(plan.trajectory_.joint_trajectory);
            
            arm_trajectory_node["trajectory_info"]["arm_name"] = arm_name;
            arm_trajectory_node["trajectory_info"]["timestamp"] = static_cast<int64_t>(now.nanoseconds());
            arm_trajectory_node["trajectory_info"]["original_points"] = plan.trajectory_.joint_trajectory.points.size();
            arm_trajectory_node["trajectory_info"]["resampled_points"] = resampled_trajectory.points.size();
            
            YAML::Node joint_names;
            for (const auto& name : resampled_trajectory.joint_names) {
                joint_names.push_back(name);
            }
            arm_trajectory_node["trajectory"]["joint_names"] = joint_names;
            
            YAML::Node points;
            for (const auto& point : resampled_trajectory.points) {
                YAML::Node point_node;
                
                YAML::Node positions;
                for (const auto& pos : point.positions) {
                    positions.push_back(pos);
                }
                point_node["positions"] = positions;
                
                if (!point.velocities.empty()) {
                    YAML::Node velocities;
                    for (const auto& vel : point.velocities) {
                        velocities.push_back(vel);
                    }
                    point_node["velocities"] = velocities;
                }
                
                point_node["time_from_start"]["sec"] = point.time_from_start.sec;
                point_node["time_from_start"]["nanosec"] = point.time_from_start.nanosec;
                
                points.push_back(point_node);
            }
            arm_trajectory_node["trajectory"]["points"] = points;
            
            root_node[arm_name] = arm_trajectory_node;
            
            std::ofstream file(filename);
            file << root_node;
            file.close();
            
            RCLCPP_INFO(this->get_logger(), "📝 Individual %s trajectory saved: %s (%zu→%zu points)", 
                       arm_name.c_str(), filename.c_str(), 
                       plan.trajectory_.joint_trajectory.points.size(), resampled_trajectory.points.size());
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to save individual trajectory: %s", e.what());
        }
        }).detach(); // 非同期実行
    }
    
    // YAMLファイルから軌道を読み込み
    bool loadTrajectoryFromYaml(const std::string& filename, moveit::planning_interface::MoveGroupInterface::Plan& plan) {
        try {
            YAML::Node yaml_node = YAML::LoadFile(filename);
            
            // ジョイント名を復元
            plan.trajectory_.joint_trajectory.joint_names.clear();
            for (const auto& name : yaml_node["trajectory"]["joint_names"]) {
                plan.trajectory_.joint_trajectory.joint_names.push_back(name.as<std::string>());
            }
            
            // 軌道ポイントを復元
            plan.trajectory_.joint_trajectory.points.clear();
            for (const auto& point_yaml : yaml_node["trajectory"]["points"]) {
                trajectory_msgs::msg::JointTrajectoryPoint point;
                
                // 位置データ復元
                for (const auto& pos : point_yaml["positions"]) {
                    point.positions.push_back(pos.as<double>());
                }
                
                // 速度データ復元（存在する場合）
                if (point_yaml["velocities"]) {
                    for (const auto& vel : point_yaml["velocities"]) {
                        point.velocities.push_back(vel.as<double>());
                    }
                }
                
                // 時間データ復元
                point.time_from_start.sec = point_yaml["time_from_start"]["sec"].as<int32_t>();
                point.time_from_start.nanosec = point_yaml["time_from_start"]["nanosec"].as<uint32_t>();
                
                plan.trajectory_.joint_trajectory.points.push_back(point);
            }
            
            RCLCPP_INFO(this->get_logger(), "Trajectory loaded from: %s", filename.c_str());
            return true;
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load trajectory from %s: %s", filename.c_str(), e.what());
            return false;
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
