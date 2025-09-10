#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <yaml-cpp/yaml.h>
#include <string>
#include <unordered_map>

class PoseCommandPublisher : public rclcpp::Node
{
public:
    PoseCommandPublisher() : Node("pose_command_publisher")
    {
        button_command_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/button_command", 10,
            std::bind(&PoseCommandPublisher::buttonCommandCallback, this, std::placeholders::_1));

        // Both-arms atomic command ("<LEFT>|<RIGHT>")
        dual_button_command_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/dual_button_command", 10,
            std::bind(&PoseCommandPublisher::dualButtonCommandCallback, this, std::placeholders::_1));

        left_target_pose_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/left_target_pose_rpy", 10);

        right_target_pose_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/right_target_pose_rpy", 10);

        // Dual-arm composite publisher: [Lx, Ly, Lz, Lr, Lp, Lyaw, Rx, Ry, Rz, Rr, Rp, Ryaw]
        dual_target_pose_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/dual_target_pose_rpy", 10);

        loadPoseConfigurations();

        RCLCPP_INFO(this->get_logger(), "Pose command publisher node started");
    }

private:
    void buttonCommandCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::string command = msg->data;
        
        if (left_pose_configs_.find(command) != left_pose_configs_.end())
        {
            publishLeftPose(command);
        }
        else if (right_pose_configs_.find(command) != right_pose_configs_.end())
        {
            publishRightPose(command);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Unknown command: %s", command.c_str());
        }
    }

    // Format: "<LEFT_CMD>|<RIGHT_CMD>" (whitespace optional)
    void dualButtonCommandCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::string payload = msg->data;

        // Normalize by removing surrounding spaces
        auto trim = [](std::string &s) {
            auto not_space = [](int ch){ return !std::isspace(ch); };
            s.erase(s.begin(), std::find_if(s.begin(), s.end(), not_space));
            s.erase(std::find_if(s.rbegin(), s.rend(), not_space).base(), s.end());
        };

        // Try split by '|', then fallback to '+' to be tolerant
        auto split_once = [](const std::string& s, char delim) -> std::pair<std::string,std::string> {
            auto pos = s.find(delim);
            if (pos == std::string::npos) return {s, std::string{}};
            return {s.substr(0, pos), s.substr(pos+1)};
        };

        std::string left_cmd, right_cmd;
        {
            auto p = split_once(payload, '|');
            left_cmd = p.first;
            right_cmd = p.second;
            if (right_cmd.empty()) {
                // Fallback to '+'
                p = split_once(payload, '+');
                left_cmd = p.first;
                right_cmd = p.second;
            }
        }

        trim(left_cmd);
        trim(right_cmd);

        if (left_cmd.empty() || right_cmd.empty()) {
            RCLCPP_WARN(this->get_logger(), "dual_button_command malformed: '%s' (expected '<LEFT>|<RIGHT>')", payload.c_str());
            return;
        }

        auto l_it = left_pose_configs_.find(left_cmd);
        auto r_it = right_pose_configs_.find(right_cmd);
        if (l_it == left_pose_configs_.end()) {
            RCLCPP_WARN(this->get_logger(), "Unknown LEFT command in dual: %s", left_cmd.c_str());
            return;
        }
        if (r_it == right_pose_configs_.end()) {
            RCLCPP_WARN(this->get_logger(), "Unknown RIGHT command in dual: %s", right_cmd.c_str());
            return;
        }

        // Publish composite dual target to ensure atomic consumption downstream
        std_msgs::msg::Float64MultiArray dual_msg;
        dual_msg.data = l_it->second; // 6 elements
        dual_msg.data.insert(dual_msg.data.end(), r_it->second.begin(), r_it->second.end()); // +6 elements
        if (dual_msg.data.size() == 12) {
            dual_target_pose_pub_->publish(dual_msg);
            RCLCPP_INFO(this->get_logger(), "Published dual composite poses: LEFT='%s', RIGHT='%s'", left_cmd.c_str(), right_cmd.c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Dual message malformed size=%zu (expected 12)", dual_msg.data.size());
        }
    }

    void publishLeftPose(const std::string& command)
    {
        auto pose_config = left_pose_configs_[command];
        auto pose_msg = createPoseFromConfig(pose_config);
        left_target_pose_pub_->publish(pose_msg);
        RCLCPP_INFO(this->get_logger(), "Published left target pose for command: %s", command.c_str());
    }

    void publishRightPose(const std::string& command)
    {
        auto pose_config = right_pose_configs_[command];
        auto pose_msg = createPoseFromConfig(pose_config);
        right_target_pose_pub_->publish(pose_msg);
        RCLCPP_INFO(this->get_logger(), "Published right target pose for command: %s", command.c_str());
    }

    std_msgs::msg::Float64MultiArray createPoseFromConfig(const std::vector<double>& config)
    {
        std_msgs::msg::Float64MultiArray pose_msg;
        pose_msg.data = config;  // [x, y, z, roll, pitch, yaw]
        return pose_msg;
    }

    void loadPoseConfigurations()
    {
        try {
            std::string config_path = "/home/a/ws_moveit2/src/robot_config/config/pose_commands.yaml";
            YAML::Node config = YAML::LoadFile(config_path);

            if (config["left_poses"]) {
                for (const auto& pose : config["left_poses"]) {
                    std::string command = pose.first.as<std::string>();
                    if (pose.second.IsNull() || !pose.second.IsSequence()) {
                        RCLCPP_WARN(this->get_logger(), "Skipping left command '%s' - empty or invalid value", command.c_str());
                        continue;
                    }
                    auto values = pose.second.as<std::vector<double>>();
                    left_pose_configs_[command] = values;
                    RCLCPP_DEBUG(this->get_logger(), "Loaded left pose '%s': [%f, %f, %f, %f, %f, %f]", 
                               command.c_str(), values[0], values[1], values[2], values[3], values[4], values[5]);
                }
            }

            if (config["right_poses"]) {
                for (const auto& pose : config["right_poses"]) {
                    std::string command = pose.first.as<std::string>();
                    if (pose.second.IsNull() || !pose.second.IsSequence()) {
                        RCLCPP_WARN(this->get_logger(), "Skipping command '%s' - empty or invalid value", command.c_str());
                        continue;
                    }
                    auto values = pose.second.as<std::vector<double>>();
                    right_pose_configs_[command] = values;
                    RCLCPP_DEBUG(this->get_logger(), "Loaded right pose '%s': [%f, %f, %f, %f, %f, %f]", 
                               command.c_str(), values[0], values[1], values[2], values[3], values[4], values[5]);
                }
            }

            RCLCPP_INFO(this->get_logger(), "Loaded %lu left poses and %lu right poses", 
                       left_pose_configs_.size(), right_pose_configs_.size());
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load pose configurations: %s", e.what());
            initializeDefaultPoses();
        }
    }

    void initializeDefaultPoses()
    {
        left_pose_configs_["Pose1"] = {0.3, 0.2, 0.5, 0.0, 0.0, 0.0};
        left_pose_configs_["Pose2"] = {0.3, 0.3, 0.5, 0.0, 0.0, 0.0};
        left_pose_configs_["Pose3"] = {0.3, 0.4, 0.5, 0.0, 0.0, 0.0};
        left_pose_configs_["Pose4"] = {0.3, 0.5, 0.5, 0.0, 0.0, 0.0};
        left_pose_configs_["Pose5"] = {0.3, 0.6, 0.5, 0.0, 0.0, 0.0};
        left_pose_configs_["Pose11"] = {0.4, 0.2, 0.5, 0.0, 0.0, 0.0};
        left_pose_configs_["Pose12"] = {0.4, 0.3, 0.5, 0.0, 0.0, 0.0};
        left_pose_configs_["Pose13"] = {0.4, 0.4, 0.5, 0.0, 0.0, 0.0};
        left_pose_configs_["Pose14"] = {0.4, 0.5, 0.5, 0.0, 0.0, 0.0};
        left_pose_configs_["Pose15"] = {0.4, 0.6, 0.5, 0.0, 0.0, 0.0};
        left_pose_configs_["left_initial"] = {0.0, 0.3, 0.4, 0.0, 0.0, 0.0};
        left_pose_configs_["left_goal"] = {0.5, 0.3, 0.4, 0.0, 0.0, 0.0};

        right_pose_configs_["Pose6"] = {0.3, -0.2, 0.5, 0.0, 0.0, 0.0};
        right_pose_configs_["Pose7"] = {0.3, -0.3, 0.5, 0.0, 0.0, 0.0};
        right_pose_configs_["Pose8"] = {0.3, -0.4, 0.5, 0.0, 0.0, 0.0};
        right_pose_configs_["Pose9"] = {0.3, -0.5, 0.5, 0.0, 0.0, 0.0};
        right_pose_configs_["Pose10"] = {0.3, -0.6, 0.5, 0.0, 0.0, 0.0};
        right_pose_configs_["Pose16"] = {0.4, -0.2, 0.5, 0.0, 0.0, 0.0};
        right_pose_configs_["Pose17"] = {0.4, -0.3, 0.5, 0.0, 0.0, 0.0};
        right_pose_configs_["Pose18"] = {0.4, -0.4, 0.5, 0.0, 0.0, 0.0};
        right_pose_configs_["Pose19"] = {0.4, -0.5, 0.5, 0.0, 0.0, 0.0};
        right_pose_configs_["Pose20"] = {0.4, -0.6, 0.5, 0.0, 0.0, 0.0};
        right_pose_configs_["right_initial"] = {0.0, -0.3, 0.4, 0.0, 0.0, 0.0};
        right_pose_configs_["right_goal"] = {0.5, -0.3, 0.4, 0.0, 0.0, 0.0};

        RCLCPP_INFO(this->get_logger(), "Initialized default poses");
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr button_command_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr dual_button_command_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr left_target_pose_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr right_target_pose_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr dual_target_pose_pub_;

    std::unordered_map<std::string, std::vector<double>> left_pose_configs_;
    std::unordered_map<std::string, std::vector<double>> right_pose_configs_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PoseCommandPublisher>());
    rclcpp::shutdown();
    return 0;
}
