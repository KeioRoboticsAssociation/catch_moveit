#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <memory>
#include <chrono>
#include <mutex>
#include <vector>
#include <cmath>

struct RecordedJointState {
    double timestamp;
    std::vector<std::string> joint_names;
    std::vector<double> positions;
    std::vector<double> velocities;
};

class SimpleTrajectoryRecorder : public rclcpp::Node {
private:
    // Subscribers
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr record_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr stop_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr replay_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;

    // Publishers
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr left_traj_pub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr right_traj_pub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr left_hand_pub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr right_hand_pub_;

    // Recording data
    std::vector<RecordedJointState> recorded_data_;
    std::vector<double> last_joint_positions_;
    std::vector<std::string> last_joint_names_;

    // State management
    bool is_recording_ = false;
    std::mutex recording_mutex_;
    double recording_start_time_;
    std::string current_recording_name_;

    // Tolerance for detecting changes
    const double CHANGE_TOLERANCE = 1e-15;  // rad

public:
    SimpleTrajectoryRecorder() : Node("simple_trajectory_recorder") {
        // Subscribers
        record_sub_ = create_subscription<std_msgs::msg::String>(
            "/record", 10,
            std::bind(&SimpleTrajectoryRecorder::recordCallback, this, std::placeholders::_1));

        stop_sub_ = create_subscription<std_msgs::msg::String>(
            "/stop", 10,
            std::bind(&SimpleTrajectoryRecorder::stopCallback, this, std::placeholders::_1));

        replay_sub_ = create_subscription<std_msgs::msg::String>(
            "/replay", 10,
            std::bind(&SimpleTrajectoryRecorder::replayCallback, this, std::placeholders::_1));

        joint_states_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&SimpleTrajectoryRecorder::jointStatesCallback, this, std::placeholders::_1));

        // Publishers
        status_pub_ = create_publisher<std_msgs::msg::String>("/recorder/status", 10);
        left_traj_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/left_arm_controller/joint_trajectory", 10);
        right_traj_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/right_arm_controller/joint_trajectory", 10);
        left_hand_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/left_hand_controller/joint_trajectory", 10);
        right_hand_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/right_hand_controller/joint_trajectory", 10);

        RCLCPP_INFO(get_logger(), "SimpleTrajectoryRecorder initialized");
        publishStatus("READY - Send /record to start recording");

        printUsageInstructions();
    }

    void printUsageInstructions() {
        RCLCPP_INFO(get_logger(), "=== Simple Trajectory Recorder ===");
        RCLCPP_INFO(get_logger(), "Commands:");
        RCLCPP_INFO(get_logger(), "  Start recording: ros2 topic pub /record std_msgs/msg/String \"data: 'my_trajectory'\" --once");
        RCLCPP_INFO(get_logger(), "  Stop recording:  ros2 topic pub /stop std_msgs/msg/String \"data: ''\" --once");
        RCLCPP_INFO(get_logger(), "  Replay:          ros2 topic pub /replay std_msgs/msg/String \"data: 'my_trajectory.yaml'\" --once");
        RCLCPP_INFO(get_logger(), "");
        RCLCPP_INFO(get_logger(), "Workflow:");
        RCLCPP_INFO(get_logger(), "  1. Send /record command");
        RCLCPP_INFO(get_logger(), "  2. Use GUI buttons to execute your trajectory");
        RCLCPP_INFO(get_logger(), "  3. Send /stop command when finished");
        RCLCPP_INFO(get_logger(), "  4. Send /replay command to replay the trajectory");
    }

    void recordCallback(const std_msgs::msg::String::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(recording_mutex_);

        if (is_recording_) {
            RCLCPP_WARN(get_logger(), "Already recording! Send /stop first.");
            return;
        }

        current_recording_name_ = msg->data;
        if (current_recording_name_.empty()) {
            current_recording_name_ = "trajectory_" + std::to_string(std::time(nullptr));
        }

        is_recording_ = true;
        recorded_data_.clear();
        last_joint_positions_.clear();
        last_joint_names_.clear();
        recording_start_time_ = now().seconds();

        RCLCPP_INFO(get_logger(), "Started recording: %s", current_recording_name_.c_str());
        publishStatus("RECORDING: " + current_recording_name_ + " - Use GUI buttons, then send /stop");
    }

    void stopCallback(const std_msgs::msg::String::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(recording_mutex_);

        if (!is_recording_) {
            RCLCPP_WARN(get_logger(), "Not currently recording!");
            return;
        }

        is_recording_ = false;

        if (recorded_data_.empty()) {
            RCLCPP_WARN(get_logger(), "No data recorded!");
            publishStatus("READY - No data was recorded");
            return;
        }

        // Save to YAML file
        std::string filename = "/home/a/ws_moveit2/src/robot_config/path/" + current_recording_name_ + ".yaml";
        saveToYaml(filename);

        RCLCPP_INFO(get_logger(), "Recording stopped. Saved %zu points to %s",
                   recorded_data_.size(), filename.c_str());
        publishStatus("SAVED: " + filename + " (" + std::to_string(recorded_data_.size()) + " points)");
    }

    void replayCallback(const std_msgs::msg::String::SharedPtr msg) {
        std::string filename = msg->data;
        if (filename.empty()) {
            RCLCPP_ERROR(get_logger(), "Please specify filename to replay");
            return;
        }

        // Add .yaml extension if not present
        if (filename.find(".yaml") == std::string::npos && filename.find(".yml") == std::string::npos) {
            filename += ".yaml";
        }

        // Add full path if not already included
        if (filename.find("/") == std::string::npos) {
            filename = "/home/a/ws_moveit2/src/robot_config/path/" + filename;
        }

        RCLCPP_INFO(get_logger(), "Replaying trajectory from: %s", filename.c_str());

        if (loadAndReplay(filename)) {
            publishStatus("REPLAYING: " + filename);
        } else {
            publishStatus("REPLAY_FAILED: " + filename);
        }
    }

    void jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        if (!is_recording_) return;

        std::lock_guard<std::mutex> lock(recording_mutex_);

        // Check if joint states have changed significantly
        if (!hasSignificantChange(msg)) return;

        // Record the joint state with original timestamp first
        RecordedJointState recorded_state;
        recorded_state.timestamp = now().seconds() - recording_start_time_;
        recorded_state.joint_names = msg->name;
        recorded_state.positions = msg->position;
        recorded_state.velocities = msg->velocity;

        recorded_data_.push_back(recorded_state);

        // Update last known state
        last_joint_names_ = msg->name;
        last_joint_positions_ = msg->position;

        RCLCPP_DEBUG(get_logger(), "Recorded state at t=%.3f (%zu joints)",
                    recorded_state.timestamp, msg->name.size());
    }

    bool hasSignificantChange(const sensor_msgs::msg::JointState::SharedPtr msg) {
        // First recording - always record
        if (last_joint_positions_.empty()) return true;

        // Check if joint names match (simple check)
        if (msg->name.size() != last_joint_names_.size()) return true;

        // Check for significant position changes
        for (size_t i = 0; i < msg->position.size() && i < last_joint_positions_.size(); ++i) {
            if (std::abs(msg->position[i] - last_joint_positions_[i]) > CHANGE_TOLERANCE) {
                return true;
            }
        }

        return false; // No significant change detected
    }

    void saveToYaml(const std::string& filename) {
        YAML::Node root;

        // Metadata
        root["trajectory_name"] = current_recording_name_;
        root["recorded_at"] = std::time(nullptr);
        root["total_points"] = recorded_data_.size();
        root["duration"] = recorded_data_.empty() ? 0.0 : recorded_data_.back().timestamp;

        // Trajectory data
        YAML::Node points;
        for (const auto& state : recorded_data_) {
            YAML::Node point;
            point["time"] = state.timestamp;
            point["joint_names"] = state.joint_names;
            point["positions"] = state.positions;
            if (!state.velocities.empty()) {
                point["velocities"] = state.velocities;
            }
            points.push_back(point);
        }
        root["trajectory_points"] = points;

        // Write to file
        std::ofstream file(filename);
        file << root;
        file.close();

        RCLCPP_INFO(get_logger(), "Saved trajectory to: %s", filename.c_str());
    }

    bool loadAndReplay(const std::string& filename) {
        try {
            YAML::Node root = YAML::LoadFile(filename);

            if (!root["trajectory_points"]) {
                RCLCPP_ERROR(get_logger(), "Invalid trajectory file format");
                return false;
            }

            auto points = root["trajectory_points"];
            RCLCPP_INFO(get_logger(), "Loading %zu trajectory points", points.size());

            // Separate left and right arm/hand trajectories
            std::vector<RecordedJointState> left_trajectory;
            std::vector<RecordedJointState> right_trajectory;
            std::vector<RecordedJointState> left_hand_trajectory;
            std::vector<RecordedJointState> right_hand_trajectory;

            for (const auto& point : points) {
                RecordedJointState state;
                state.timestamp = point["time"].as<double>();
                state.joint_names = point["joint_names"].as<std::vector<std::string>>();
                state.positions = point["positions"].as<std::vector<double>>();

                if (point["velocities"]) {
                    state.velocities = point["velocities"].as<std::vector<double>>();
                }

                // Check if this contains left/right arm/hand joints
                bool has_left_arm = false, has_right_arm = false;
                bool has_left_hand = false, has_right_hand = false;

                for (const auto& joint_name : state.joint_names) {
                    if (joint_name.find("left_Revolute_") == 0) has_left_arm = true;
                    if (joint_name.find("right_Revolute_") == 0) has_right_arm = true;
                    if (joint_name.find("left_Slider_") == 0) has_left_hand = true;
                    if (joint_name.find("right_Slider_") == 0) has_right_hand = true;
                }

                if (has_left_arm) left_trajectory.push_back(state);
                if (has_right_arm) right_trajectory.push_back(state);
                if (has_left_hand) left_hand_trajectory.push_back(state);
                if (has_right_hand) right_hand_trajectory.push_back(state);
            }

            // Create and publish trajectory messages
            if (!left_trajectory.empty()) {
                auto left_traj_msg = createTrajectoryMessage(left_trajectory, "left_Revolute");
                left_traj_pub_->publish(left_traj_msg);
                RCLCPP_INFO(get_logger(), "Published left arm trajectory (%zu points)", left_trajectory.size());
            }

            if (!right_trajectory.empty()) {
                auto right_traj_msg = createTrajectoryMessage(right_trajectory, "right_Revolute");
                right_traj_pub_->publish(right_traj_msg);
                RCLCPP_INFO(get_logger(), "Published right arm trajectory (%zu points)", right_trajectory.size());
            }

            if (!left_hand_trajectory.empty()) {
                auto left_hand_msg = createTrajectoryMessage(left_hand_trajectory, "left_Slider");
                left_hand_pub_->publish(left_hand_msg);
                RCLCPP_INFO(get_logger(), "Published left hand trajectory (%zu points)", left_hand_trajectory.size());
            }

            if (!right_hand_trajectory.empty()) {
                auto right_hand_msg = createTrajectoryMessage(right_hand_trajectory, "right_Slider");
                right_hand_pub_->publish(right_hand_msg);
                RCLCPP_INFO(get_logger(), "Published right hand trajectory (%zu points)", right_hand_trajectory.size());
            }

            return true;

        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Failed to load trajectory file: %s", e.what());
            return false;
        }
    }

    trajectory_msgs::msg::JointTrajectory createTrajectoryMessage(
        const std::vector<RecordedJointState>& trajectory, const std::string& arm_prefix) {

        trajectory_msgs::msg::JointTrajectory traj_msg;

        if (trajectory.empty()) return traj_msg;

        // Extract joint names for this arm (only Revolute joints for controllers)
        std::vector<std::string> arm_joint_names;
        std::vector<int> joint_indices;

        for (size_t i = 0; i < trajectory[0].joint_names.size(); ++i) {
            const auto& joint_name = trajectory[0].joint_names[i];
            if (joint_name.find(arm_prefix + "_") == 0) {
                arm_joint_names.push_back(joint_name);
                joint_indices.push_back(i);
            }
        }

        traj_msg.joint_names = arm_joint_names;

        // Create trajectory points with compressed timestamps
        double compressed_time = 0.0;
        double last_original_time = 0.0;
        const double MAX_GAP = 0.51;  // If gap > 0.51s, compress to 0.5s

        for (size_t i = 0; i < trajectory.size(); ++i) {
            const auto& state = trajectory[i];
            trajectory_msgs::msg::JointTrajectoryPoint point;

            // Extract positions for this arm
            for (int idx : joint_indices) {
                if (idx < static_cast<int>(state.positions.size())) {
                    point.positions.push_back(state.positions[idx]);
                }
            }

            // Extract velocities if available
            if (!state.velocities.empty()) {
                for (int idx : joint_indices) {
                    if (idx < static_cast<int>(state.velocities.size())) {
                        point.velocities.push_back(state.velocities[idx]);
                    }
                }
            }

            // Compress timestamps - remove long planning gaps
            if (i == 0) {
                compressed_time = 0.0;
            } else {
                double time_gap = state.timestamp - last_original_time;
                if (time_gap > MAX_GAP) {
                    // Long gap detected (planning time) - compress to 0.5s
                    compressed_time += 0.5;
                    RCLCPP_INFO(get_logger(), "Compressed gap %.3fs -> 0.5s", time_gap);
                } else {
                    // Normal motion - keep real time interval
                    compressed_time += time_gap;
                    if (time_gap > 0.5) {  // Log gaps > 0.5s for debugging
                        RCLCPP_INFO(get_logger(), "Keeping gap %.3fs (under threshold)", time_gap);
                    }
                }
            }

            point.time_from_start = rclcpp::Duration::from_seconds(compressed_time);
            traj_msg.points.push_back(point);

            last_original_time = state.timestamp;

            // Debug: Print first few and last few timestamps
            if (i < 3 || i >= trajectory.size() - 3) {
                RCLCPP_INFO(get_logger(), "Point %zu: original=%.3fs -> compressed=%.3fs",
                           i, state.timestamp, compressed_time);
            }
        }

        return traj_msg;
    }

    void publishStatus(const std::string& status) {
        auto msg = std_msgs::msg::String();
        msg.data = status;
        status_pub_->publish(msg);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleTrajectoryRecorder>());
    rclcpp::shutdown();
    return 0;
}