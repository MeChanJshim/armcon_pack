#include "Y2RobMotion/runtime_config.hpp"

#include <stdexcept>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

namespace {

template <typename T>
void loadScalarIfPresent(const YAML::Node& node, const char* key, T& value)
{
    if (node[key]) {
        value = node[key].as<T>();
    }
}

template <typename T, std::size_t N>
void loadArrayIfPresent(const YAML::Node& node, const char* key, std::array<T, N>& value)
{
    if (!node[key]) return;
    const YAML::Node values = node[key];
    if (!values.IsSequence() || values.size() != N) {
        throw std::runtime_error(std::string(key) + " must contain exactly " + std::to_string(N) + " values.");
    }
    for (std::size_t i = 0; i < N; ++i) {
        value[i] = values[i].as<T>();
    }
}

YMatrix loadMatrix4(const YAML::Node& node, const YMatrix& fallback)
{
    if (!node) return fallback;
    if (!node.IsSequence() || node.size() != 4) {
        throw std::runtime_error("ee_to_tcp must be a 4x4 sequence.");
    }

    YMatrix out(4, 4);
    for (std::size_t r = 0; r < 4; ++r) {
        const YAML::Node row = node[r];
        if (!row.IsSequence() || row.size() != 4) {
            throw std::runtime_error("ee_to_tcp must be a 4x4 sequence.");
        }
        for (std::size_t c = 0; c < 4; ++c) {
            out[r][c] = row[c].as<double>();
        }
    }
    return out;
}

}  // namespace

RobotRuntimeConfig loadRobotRuntimeConfig(const std::string& path)
{
    RobotRuntimeConfig config;
    const YAML::Node root = YAML::LoadFile(path);
    const YAML::Node node = root["Y2RobMotion"] ? root["Y2RobMotion"] : root;

    if (node["robot"]) {
        const YAML::Node robot = node["robot"];
        loadScalarIfPresent(robot, "number_of_joints", config.number_of_joints);
        loadScalarIfPresent(robot, "name", config.robot_name);
        loadScalarIfPresent(robot, "control_period", config.control_period);
        loadScalarIfPresent(robot, "package_bundle_dir", config.package_bundle_dir);
        loadScalarIfPresent(robot, "trajectory_mode", config.trajectory_mode);
        if (robot["joint_names"]) {
            config.joint_names = robot["joint_names"].as<std::vector<std::string>>();
        }
        config.ee_to_tcp = loadMatrix4(robot["ee_to_tcp"], config.ee_to_tcp);
    }

    if (node["ros"]) {
        const YAML::Node ros = node["ros"];
        loadScalarIfPresent(ros, "test_mode", config.test_mode);
        loadScalarIfPresent(ros, "remapping_enabled", config.remapping_enabled);
        loadScalarIfPresent(ros, "remap_state_topic", config.remap_state_topic);
        loadScalarIfPresent(ros, "remap_command_topic", config.remap_command_topic);
        loadScalarIfPresent(ros, "joy_move_topic_suffix", config.joy_move_topic_suffix);
        loadScalarIfPresent(ros, "camera_teaching_command_topic", config.camera_teaching_command_topic);
    }

    if (node["joystick"]) {
        const YAML::Node joystick = node["joystick"];
        loadArrayIfPresent(joystick, "axis_mapping", config.joystick_axis_mapping);
        loadArrayIfPresent(joystick, "axis_scales", config.joystick_axis_scales);

        if (joystick["camera"]) {
            const YAML::Node camera = joystick["camera"];
            loadArrayIfPresent(camera, "pose_gains", config.camera_pose_gains);
            loadArrayIfPresent(camera, "max_deltas", config.camera_max_deltas);
            loadArrayIfPresent(camera, "deadbands", config.camera_deadbands);
            loadArrayIfPresent(camera, "rate_limits", config.camera_rate_limits);
            loadScalarIfPresent(camera, "command_timeout", config.camera_command_timeout);
        }

        if (joystick["force"]) {
            const YAML::Node force = joystick["force"];
            loadScalarIfPresent(force, "input_axis", config.joystick_force_input_axis);
            loadScalarIfPresent(force, "input_scale", config.joystick_force_input_scale);
            loadScalarIfPresent(force, "target_axis", config.joystick_force_target_axis);
            loadScalarIfPresent(force, "input_neutral", config.joystick_force_input_neutral);
            loadScalarIfPresent(force, "deadband", config.joystick_force_deadband);
        }
    }

    return config;
}

RobotRuntimeConfig loadInstalledRobotRuntimeConfig()
{
    const std::string share_dir = ament_index_cpp::get_package_share_directory("Y2RobMotion");
    return loadRobotRuntimeConfig(share_dir + "/config/y2_rob_motion.yaml");
}
