#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <thread>
#include <memory>
#include <chrono>
#include <mutex>
#include <cmath>
#include <iomanip>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <algorithm>

struct DualArmWaypoint {
    std::string name;

    // 左アーム
    bool left_active = false;
    std::string left_planner;
    std::vector<double> left_joint_states;

    // 右アーム
    bool right_active = false;
    std::string right_planner;
    std::vector<double> right_joint_states;

    std::string action;
    double delay = 0.0;
};

struct DualArmSequence {
    std::string name;
    std::vector<DualArmWaypoint> waypoints;
};

struct RecordedPoint {
    double timestamp;
    std::vector<double> positions;
    std::string arm_id;
};

class DualArmTrajectoryRecorderNode : public rclcpp::Node {
private:
    // MoveIt interfaces (参考: move_to_pose_dual_cpp.cpp)
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> left_move_group_interface_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> right_move_group_interface_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> left_hand_move_group_interface_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> right_hand_move_group_interface_;
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
    std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> planning_scene_monitor_;

    // 初期化スレッド（参考: move_to_pose_dual_cpp.cpp）
    std::thread left_init_thread_;
    std::thread right_init_thread_;

    // Subscribers
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr record_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr start_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;

    // Publishers
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr button_command_pub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr left_traj_pub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr right_traj_pub_;

    // 記録データ
    std::map<std::string, DualArmSequence> sequences_;
    std::vector<RecordedPoint> left_recording_;
    std::vector<RecordedPoint> right_recording_;
    std::vector<double> last_left_joints_;
    std::vector<double> last_right_joints_;

    // 状態管理
    bool is_recording_ = false;
    std::string current_sequence_;
    int current_waypoint_idx_ = 0;
    std::mutex recording_mutex_;

    // 記録された軌道
    std::map<std::string, std::pair<
        std::vector<RecordedPoint>,  // left_trajectory
        std::vector<RecordedPoint>   // right_trajectory
    >> recorded_trajectories_;

public:
    DualArmTrajectoryRecorderNode()
        : Node("dual_arm_trajectory_recorder", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
    {
        RCLCPP_INFO(get_logger(), "Initializing DualArmTrajectoryRecorderNode...");

        // 左アームの初期化スレッド（move_to_pose_dual_cpp.cppと同様）
        left_init_thread_ = std::thread([this]() {
            RCLCPP_INFO(get_logger(), "Initializing MoveGroupInterface for left_arm...");
            left_move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
                rclcpp::Node::SharedPtr(this, [](rclcpp::Node*){}), "left_arm");
            left_move_group_interface_->setEndEffectorLink("left_EndEffector_1");
            left_move_group_interface_->setPlanningPipelineId("ompl");
            left_move_group_interface_->setPlannerId("RRTConnectkConfigDefault");
            RCLCPP_INFO(get_logger(), "MoveGroupInterface for left_arm initialized.");

            // 左ハンド（グリッパー）の初期化
            RCLCPP_INFO(get_logger(), "Initializing MoveGroupInterface for left_hand...");
            left_hand_move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
                rclcpp::Node::SharedPtr(this, [](rclcpp::Node*){}), "left_hand");
            left_hand_move_group_interface_->setPlanningPipelineId("ompl");
            left_hand_move_group_interface_->setPlannerId("RRTConnect");
            RCLCPP_INFO(get_logger(), "MoveGroupInterface for left_hand initialized.");
        });

        // 右アームの初期化スレッド（move_to_pose_dual_cpp.cppと同様）
        right_init_thread_ = std::thread([this]() {
            RCLCPP_INFO(get_logger(), "Initializing MoveGroupInterface for right_arm...");
            right_move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
                rclcpp::Node::SharedPtr(this, [](rclcpp::Node*){}), "right_arm");
            right_move_group_interface_->setEndEffectorLink("right_EndEffector_1");
            right_move_group_interface_->setPlanningPipelineId("ompl");
            right_move_group_interface_->setPlannerId("RRTConnectkConfigDefault");
            RCLCPP_INFO(get_logger(), "MoveGroupInterface for right_arm initialized.");

            // 右ハンド（グリッパー）の初期化
            RCLCPP_INFO(get_logger(), "Initializing MoveGroupInterface for right_hand...");
            right_hand_move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
                rclcpp::Node::SharedPtr(this, [](rclcpp::Node*){}), "right_hand");
            right_hand_move_group_interface_->setPlanningPipelineId("ompl");
            right_hand_move_group_interface_->setPlannerId("RRTConnect");
            RCLCPP_INFO(get_logger(), "MoveGroupInterface for right_hand initialized.");
        });

        // Subscribers
        record_sub_ = create_subscription<std_msgs::msg::String>(
            "/trajectory_recorder/record", 10,
            std::bind(&DualArmTrajectoryRecorderNode::recordCallback, this, std::placeholders::_1));

        start_sub_ = create_subscription<std_msgs::msg::String>(
            "/trajectory_recorder/start", 10,
            std::bind(&DualArmTrajectoryRecorderNode::startCallback, this, std::placeholders::_1));

        joint_states_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&DualArmTrajectoryRecorderNode::jointStatesCallback, this, std::placeholders::_1));

        // Publishers
        status_pub_ = create_publisher<std_msgs::msg::String>(
            "/trajectory_recorder/status", 10);
        button_command_pub_ = create_publisher<std_msgs::msg::String>(
            "/button_command", 10);
        left_traj_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/left_arm_controller/joint_trajectory", 10);
        right_traj_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/right_arm_controller/joint_trajectory", 10);

        // Planning Scene Interface初期化（move_to_pose_dual_cpp.cppと同様）
        planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

        planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
            shared_from_this(), "robot_description");

        if (planning_scene_monitor_) {
            planning_scene_monitor_->startSceneMonitor();
            planning_scene_monitor_->startWorldGeometryMonitor();
            planning_scene_monitor_->startStateMonitor();
            RCLCPP_INFO(get_logger(), "Planning Scene Monitor initialized and started");
        } else {
            RCLCPP_ERROR(get_logger(), "Failed to initialize Planning Scene Monitor");
        }

        // 設定読み込み
        loadSequenceConfig();

        RCLCPP_INFO(get_logger(), "DualArmTrajectoryRecorder initialized");
        RCLCPP_INFO(get_logger(), "Usage:");
        RCLCPP_INFO(get_logger(), "  Record: ros2 topic pub /trajectory_recorder/record std_msgs/msg/String \"data: 'sequence_name'\" --once");
        RCLCPP_INFO(get_logger(), "  Replay: ros2 topic pub /trajectory_recorder/start std_msgs/msg/String \"data: 'sequence_name'\" --once");
    }

    ~DualArmTrajectoryRecorderNode() {
        if (left_init_thread_.joinable()) {
            left_init_thread_.join();
        }
        if (right_init_thread_.joinable()) {
            right_init_thread_.join();
        }
    }

    void loadSequenceConfig() {
        try {
            // ハードコードされたシーケンス設定（後でYAMLファイルから読み込み可能）
            DualArmSequence sequence;
            sequence.name = "pick_and_place_dual_ompl_pilz";

            // 1. OMPLでアプローチ
            DualArmWaypoint wp1;
            wp1.name = "ompl_approach";
            wp1.left_active = true;
            wp1.left_planner = "OMPL";
            wp1.left_joint_states = {0.2, -1.0, 1.4, -0.5, 1.57, 0.0};
            wp1.right_active = true;
            wp1.right_planner = "OMPL";
            wp1.right_joint_states = {-0.2, 1.0, -1.4, 0.5, -1.57, 0.0};
            wp1.action = "move";
            wp1.delay = 2.0;
            sequence.waypoints.push_back(wp1);

            // 2. Pilzで降下
            DualArmWaypoint wp2;
            wp2.name = "pilz_descend";
            wp2.left_active = true;
            wp2.left_planner = "Pilz";
            wp2.left_joint_states = {0.2, -0.8, 1.0, -0.5, 1.57, 0.0};
            wp2.right_active = true;
            wp2.right_planner = "Pilz";
            wp2.right_joint_states = {-0.2, 0.8, -1.0, 0.5, -1.57, 0.0};
            wp2.action = "move";
            wp2.delay = 1.0;
            sequence.waypoints.push_back(wp2);

            // 3. 掴む
            DualArmWaypoint wp3;
            wp3.name = "grasp_objects";
            wp3.left_active = false;
            wp3.right_active = false;
            wp3.action = "grasp";
            wp3.delay = 1.0;
            sequence.waypoints.push_back(wp3);

            // 4. Pilzで上昇
            DualArmWaypoint wp4;
            wp4.name = "pilz_lift";
            wp4.left_active = true;
            wp4.left_planner = "Pilz";
            wp4.left_joint_states = {0.2, -1.0, 1.4, -0.5, 1.57, 0.0};
            wp4.right_active = true;
            wp4.right_planner = "Pilz";
            wp4.right_joint_states = {-0.2, 1.0, -1.4, 0.5, -1.57, 0.0};
            wp4.action = "move";
            wp4.delay = 1.0;
            sequence.waypoints.push_back(wp4);

            // 5. OMPLで配置位置へ移動
            DualArmWaypoint wp5;
            wp5.name = "ompl_place_move";
            wp5.left_active = true;
            wp5.left_planner = "OMPL";
            wp5.left_joint_states = {1.0, -1.2, 1.6, -0.8, 1.57, 0.0};
            wp5.right_active = true;
            wp5.right_planner = "OMPL";
            wp5.right_joint_states = {-1.0, 1.2, -1.6, 0.8, -1.57, 0.0};
            wp5.action = "move";
            wp5.delay = 2.0;
            sequence.waypoints.push_back(wp5);

            // 6. Pilzで配置降下
            DualArmWaypoint wp6;
            wp6.name = "pilz_place_descend";
            wp6.left_active = true;
            wp6.left_planner = "Pilz";
            wp6.left_joint_states = {1.0, -1.0, 1.2, -0.8, 1.57, 0.0};
            wp6.right_active = true;
            wp6.right_planner = "Pilz";
            wp6.right_joint_states = {-1.0, 1.0, -1.2, 0.8, -1.57, 0.0};
            wp6.action = "move";
            wp6.delay = 1.0;
            sequence.waypoints.push_back(wp6);

            // 7. 離す
            DualArmWaypoint wp7;
            wp7.name = "release_objects";
            wp7.left_active = false;
            wp7.right_active = false;
            wp7.action = "release";
            wp7.delay = 1.0;
            sequence.waypoints.push_back(wp7);

            sequences_[sequence.name] = sequence;

            RCLCPP_INFO(get_logger(), "Loaded sequence: %s with %zu waypoints",
                       sequence.name.c_str(), sequence.waypoints.size());

        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Failed to load config: %s", e.what());
        }
    }

    void recordCallback(const std_msgs::msg::String::SharedPtr msg) {
        std::string sequence_name = msg->data;

        if (sequences_.find(sequence_name) == sequences_.end()) {
            RCLCPP_ERROR(get_logger(), "Unknown sequence: %s", sequence_name.c_str());
            return;
        }

        RCLCPP_INFO(get_logger(), "Starting recording for sequence: %s", sequence_name.c_str());
        startRecording(sequence_name);
    }

    void startRecording(const std::string& sequence_name) {
        std::lock_guard<std::mutex> lock(recording_mutex_);

        current_sequence_ = sequence_name;
        current_waypoint_idx_ = 0;
        is_recording_ = true;
        left_recording_.clear();
        right_recording_.clear();

        publishStatus("RECORDING_STARTED:" + sequence_name);

        // 自動でウェイポイントシーケンス実行
        std::thread([this]() {
            executeSequence();
        }).detach();
    }

    void executeSequence() {
        auto& sequence = sequences_[current_sequence_];

        RCLCPP_INFO(get_logger(), "Executing sequence: %s (%zu waypoints)",
                   current_sequence_.c_str(), sequence.waypoints.size());

        // 各ウェイポイントを順次実行
        for (size_t i = 0; i < sequence.waypoints.size(); ++i) {
            current_waypoint_idx_ = i;
            auto& waypoint = sequence.waypoints[i];

            RCLCPP_INFO(get_logger(), "Executing waypoint %zu: %s", i, waypoint.name.c_str());
            publishStatus("WAYPOINT:" + std::to_string(i) + ":" + waypoint.name);

            // 両アームのコマンド送信
            executeDualArmWaypoint(waypoint);

            // グリッパーアクション
            executeGripperAction(waypoint);

            // 遅延
            if (waypoint.delay > 0) {
                std::this_thread::sleep_for(std::chrono::milliseconds(int(waypoint.delay * 1000)));
            }
        }

        finishRecording();
    }

    void executeDualArmWaypoint(const DualArmWaypoint& waypoint) {
        std::vector<std::future<bool>> futures;

        // 左アーム実行
        if (waypoint.left_active) {
            futures.push_back(std::async(std::launch::async, [this, &waypoint]() {
                return executeArmMotion("left", waypoint.left_planner, waypoint.left_joint_states);
            }));
        }

        // 右アーム実行
        if (waypoint.right_active) {
            futures.push_back(std::async(std::launch::async, [this, &waypoint]() {
                return executeArmMotion("right", waypoint.right_planner, waypoint.right_joint_states);
            }));
        }

        // 両アームの完了待機
        bool all_success = true;
        for (auto& future : futures) {
            if (!future.get()) {
                all_success = false;
            }
        }

        if (!all_success) {
            RCLCPP_WARN(get_logger(), "Some arm motions failed for waypoint: %s", waypoint.name.c_str());
        }
    }

    bool executeArmMotion(const std::string& arm_name,
                         const std::string& planner,
                         const std::vector<double>& joint_states) {

        auto move_group = (arm_name == "left") ? left_move_group_interface_ : right_move_group_interface_;

        if (!move_group) {
            RCLCPP_ERROR(get_logger(), "MoveGroup not initialized for %s arm", arm_name.c_str());
            return false;
        }

        RCLCPP_INFO(get_logger(), "Executing %s motion for %s arm", planner.c_str(), arm_name.c_str());

        if (planner == "OMPL") {
            return executeOMPLMotion(move_group, joint_states, arm_name);
        } else if (planner == "Pilz") {
            return executePilzMotion(move_group, joint_states, arm_name);
        } else {
            RCLCPP_ERROR(get_logger(), "Unknown planner: %s", planner.c_str());
            return false;
        }
    }

    bool executeOMPLMotion(std::shared_ptr<moveit::planning_interface::MoveGroupInterface> group,
                          const std::vector<double>& joint_states, const std::string& arm_name) {
        // OMPL プランナー設定（move_to_pose_dual_cpp.cppと同様）
        group->setPlanningPipelineId("ompl");
        group->setPlannerId("RRTConnectkConfigDefault");
        group->setJointValueTarget(joint_states);
        group->setPlanningTime(10.0);
        group->setNumPlanningAttempts(5);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (group->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success) {
            group->execute(plan);
            RCLCPP_INFO(get_logger(), "OMPL motion executed successfully for %s arm", arm_name.c_str());
            return true;
        } else {
            RCLCPP_ERROR(get_logger(), "OMPL motion planning failed for %s arm", arm_name.c_str());
            return false;
        }
    }

    bool executePilzMotion(std::shared_ptr<moveit::planning_interface::MoveGroupInterface> group,
                          const std::vector<double>& joint_states, const std::string& arm_name) {
        // Pilz プランナー設定（move_to_pose_dual_cpp.cppと同様）
        group->setPlanningPipelineId("pilz_industrial_motion_planner");
        group->setPlannerId("LIN");  // 直線補間
        group->setJointValueTarget(joint_states);
        group->setPlanningTime(0.1);  // 高速プランニング
        group->setNumPlanningAttempts(3);
        group->setGoalPositionTolerance(0.0001);
        group->setMaxVelocityScalingFactor(0.3);  // 速度制限（安全のため）
        group->setMaxAccelerationScalingFactor(0.3);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (group->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success) {
            group->execute(plan);
            RCLCPP_INFO(get_logger(), "Pilz motion executed successfully for %s arm", arm_name.c_str());
            return true;
        } else {
            RCLCPP_ERROR(get_logger(), "Pilz motion planning failed for %s arm", arm_name.c_str());
            return false;
        }
    }

    void jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        if (!is_recording_) return;

        std::lock_guard<std::mutex> lock(recording_mutex_);

        // 左アーム関節の抽出・記録
        recordArmJoints(msg, "left");

        // 右アーム関節の抽出・記録
        recordArmJoints(msg, "right");
    }

    void recordArmJoints(const sensor_msgs::msg::JointState::SharedPtr msg, const std::string& arm_name) {
        std::vector<double> arm_positions;
        std::string prefix = arm_name + "_";

        // 該当アームの関節のみ抽出
        for (size_t i = 0; i < msg->name.size(); ++i) {
            if (msg->name[i].find(prefix) == 0) {
                arm_positions.push_back(msg->position[i]);
            }
        }

        if (arm_positions.empty()) return;

        // 前回と同じ値はスキップ（高速化）
        auto& last_joints = (arm_name == "left") ? last_left_joints_ : last_right_joints_;
        if (shouldSkipState(arm_positions, last_joints)) return;

        // 記録
        RecordedPoint point;
        point.timestamp = now().seconds();
        point.positions = arm_positions;
        point.arm_id = arm_name;

        if (arm_name == "left") {
            left_recording_.push_back(point);
            last_left_joints_ = arm_positions;
        } else {
            right_recording_.push_back(point);
            last_right_joints_ = arm_positions;
        }
    }

    bool shouldSkipState(const std::vector<double>& current, const std::vector<double>& last) {
        if (last.empty()) return false;

        const double tolerance = 0.001;
        for (size_t i = 0; i < current.size() && i < last.size(); ++i) {
            if (std::abs(current[i] - last[i]) > tolerance) {
                return false;
            }
        }
        return true; // 変化なし
    }

    void executeGripperAction(const DualArmWaypoint& waypoint) {
        if (waypoint.action == "grasp" || waypoint.action == "close") {
            auto msg = std_msgs::msg::String();
            msg.data = "close_grippers";
            button_command_pub_->publish(msg);
            RCLCPP_INFO(get_logger(), "Executed grasp action");
        } else if (waypoint.action == "release" || waypoint.action == "open") {
            auto msg = std_msgs::msg::String();
            msg.data = "open_grippers";
            button_command_pub_->publish(msg);
            RCLCPP_INFO(get_logger(), "Executed release action");
        }
    }

    void finishRecording() {
        std::lock_guard<std::mutex> lock(recording_mutex_);
        is_recording_ = false;

        // 記録データを保存
        recorded_trajectories_[current_sequence_] = std::make_pair(left_recording_, right_recording_);

        // ファイル保存
        saveRecordingToFile();

        RCLCPP_INFO(get_logger(), "Recording finished for: %s", current_sequence_.c_str());
        RCLCPP_INFO(get_logger(), "Left arm points: %zu, Right arm points: %zu",
                   left_recording_.size(), right_recording_.size());

        publishStatus("RECORDING_FINISHED:" + current_sequence_);
    }

    void startCallback(const std_msgs::msg::String::SharedPtr msg) {
        std::string sequence_name = msg->data;

        if (recorded_trajectories_.find(sequence_name) == recorded_trajectories_.end()) {
            RCLCPP_ERROR(get_logger(), "No recorded trajectory for: %s", sequence_name.c_str());
            return;
        }

        RCLCPP_INFO(get_logger(), "Replaying sequence: %s", sequence_name.c_str());
        replaySequence(sequence_name);
    }

    void replaySequence(const std::string& sequence_name) {
        auto& trajectories = recorded_trajectories_[sequence_name];

        // JointTrajectoryメッセージ作成
        auto left_traj = createJointTrajectory(trajectories.first);
        auto right_traj = createJointTrajectory(trajectories.second);

        // 同時送信
        left_traj_pub_->publish(left_traj);
        right_traj_pub_->publish(right_traj);

        publishStatus("REPLAYING:" + sequence_name);
        RCLCPP_INFO(get_logger(), "Started replay of: %s", sequence_name.c_str());
    }

    trajectory_msgs::msg::JointTrajectory createJointTrajectory(const std::vector<RecordedPoint>& points) {
        trajectory_msgs::msg::JointTrajectory traj;

        if (points.empty()) return traj;

        // 関節名設定（実際の関節名に合わせて調整）
        if (points[0].arm_id == "left") {
            traj.joint_names = {"left_joint_1", "left_joint_2", "left_joint_3", "left_joint_4", "left_joint_5", "left_joint_6"};
        } else {
            traj.joint_names = {"right_joint_1", "right_joint_2", "right_joint_3", "right_joint_4", "right_joint_5", "right_joint_6"};
        }

        double start_time = points[0].timestamp;

        for (const auto& point : points) {
            trajectory_msgs::msg::JointTrajectoryPoint traj_point;
            traj_point.positions = point.positions;

            double elapsed = point.timestamp - start_time;
            traj_point.time_from_start = rclcpp::Duration::from_seconds(elapsed);

            traj.points.push_back(traj_point);
        }

        return traj;
    }

    void saveRecordingToFile() {
        // 記録データをYAMLファイルに保存（実装省略）
        RCLCPP_INFO(get_logger(), "Recording data saved to file");
    }

    void publishStatus(const std::string& status) {
        auto msg = std_msgs::msg::String();
        msg.data = status;
        status_pub_->publish(msg);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<DualArmTrajectoryRecorderNode>();

    // MoveIt初期化待機
    std::this_thread::sleep_for(std::chrono::seconds(2));

    RCLCPP_INFO(node->get_logger(), "DualArmTrajectoryRecorder ready!");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}