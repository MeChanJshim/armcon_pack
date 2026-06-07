#pragma once

#include <rclcpp/rclcpp.hpp>
#include <stdio.h>
#include <fstream>
#include <memory>
#include <chrono>
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"
#include "Y2RobMotion/runtime_config.hpp"
#include <filesystem>
#include <iostream>
#include <thread>
#include <atomic>
#include <mutex>
#include <array>
#include <cctype>
#include <algorithm>

constexpr char CP_PATH[]   = "/measured/currentP.txt";
constexpr char CJ_PATH[]   = "/measured/currentJ.txt";
constexpr char CF_PATH[]   = "/measured/currentF.txt";
constexpr char CMDK_PATH[] = "/measured/currentMDK.txt";

constexpr char TP_PATH[] = "/measured/targetP.txt";
constexpr char TJ_PATH[] = "/measured/targetJ.txt";
constexpr char TF_PATH[] = "/measured/targetF.txt";

constexpr char RECORD_START_TOPIC[] = "/ur10skku/record_boolean_R";
constexpr char RECORD_STOP_TOPIC[]  = "/ur10skku/record_boolean_L";

enum class Mode { Continuous, Discrete };

class UrMeasure : public rclcpp::Node
{
public:
    explicit UrMeasure(Mode mode, const RobotRuntimeConfig& config);
    ~UrMeasure();

private:
    // Robot name & mode
    RobotRuntimeConfig config_;
    std::string robot_name_;
    Mode mode_;
    int number_of_joints_ = 6;
    double measure_period_ = 0.04;
    std::string package_path_;

    // File handles
    std::unique_ptr<std::ofstream> cp_file_;
    std::unique_ptr<std::ofstream> cj_file_;
    std::unique_ptr<std::ofstream> cf_file_;
    std::unique_ptr<std::ofstream> cmdk_file_;
    std::unique_ptr<std::ofstream> tp_file_;
    std::unique_ptr<std::ofstream> tj_file_;
    std::unique_ptr<std::ofstream> tf_file_;

    // Latest data buffers (snapshot / periodic logging)
    std::vector<double> last_cj_;
    std::array<double, 6> last_cp_{};
    std::array<double, 6> last_cf_{};
    std::array<double, 3> last_cmdk_{};
    std::vector<double> last_tj_;
    std::array<double, 6> last_tp_{};
    std::array<double, 6> last_tf_{};

    bool have_cj_{false}, have_cp_{false}, have_cf_{false}, have_cmdk_{false},
         have_tj_{false}, have_tp_{false}, have_tf_{false};

    std::mutex buf_mtx_;

    // ROS2 Subscribers
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr current_j_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr current_p_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr current_f_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr current_mdk_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr target_j_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr target_p_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr target_f_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr record_start_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr record_stop_sub_;

    // Timer for main loop
    rclcpp::TimerBase::SharedPtr timer_;

    // Discrete mode trigger & input thread
    std::atomic<bool> snap_trigger_{false};
    std::atomic<bool> continuous_recording_active_{false};
    std::thread input_thread_;

    // Callbacks
    void currentJCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void currentPCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void currentFCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void currentMDKCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void targetJCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void targetPCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void targetFCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void recordStartCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void recordStopCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void timerCallback();

    // Helpers
    bool initializeFiles();
    void startInputThread();
    void requestStartOrSnapshot(const char* source);
    void requestStop(const char* source);
    void writeDiscreteSnapshot();
};

// Constructor
UrMeasure::UrMeasure(Mode mode, const RobotRuntimeConfig& config)
    : Node("ur_measure_node"),
      config_(config),
      robot_name_(config.robot_name),
      mode_(mode),
      number_of_joints_(config.number_of_joints),
      measure_period_(config.control_period * 5.0),
      package_path_(config.package_bundle_dir + "/Y2RobMotion"),
      last_cj_(static_cast<std::size_t>(config.number_of_joints), 0.0),
      last_tj_(static_cast<std::size_t>(config.number_of_joints), 0.0)
{
    if (!initializeFiles()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize files. Exiting.");
        rclcpp::shutdown();
        return;
    }

    // Topics
    std::string current_j_topic   = robot_name_ + "/currentJ";
    std::string current_p_topic   = robot_name_ + "/currentP";
    std::string current_f_topic   = robot_name_ + "/currentF";
    std::string current_mdk_topic = robot_name_ + "/currentMDK";
    std::string target_j_topic    = robot_name_ + "/targetJ";
    std::string target_p_topic    = robot_name_ + "/targetP";
    std::string target_f_topic    = robot_name_ + "/targetF";

    // Subscribers (callbacks only buffer the latest values)
    current_j_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        current_j_topic, 10,
        std::bind(&UrMeasure::currentJCallback, this, std::placeholders::_1));

    current_p_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        current_p_topic, 10,
        std::bind(&UrMeasure::currentPCallback, this, std::placeholders::_1));

    current_f_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        current_f_topic, 10,
        std::bind(&UrMeasure::currentFCallback, this, std::placeholders::_1));

    current_mdk_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        current_mdk_topic, 10,
        std::bind(&UrMeasure::currentMDKCallback, this, std::placeholders::_1));

    target_j_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        target_j_topic, 10,
        std::bind(&UrMeasure::targetJCallback, this, std::placeholders::_1));

    target_p_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        target_p_topic, 10,
        std::bind(&UrMeasure::targetPCallback, this, std::placeholders::_1));

    target_f_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        target_f_topic, 10,
        std::bind(&UrMeasure::targetFCallback, this, std::placeholders::_1));

    record_start_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        RECORD_START_TOPIC, 10,
        std::bind(&UrMeasure::recordStartCallback, this, std::placeholders::_1));

    record_stop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        RECORD_STOP_TOPIC, 10,
        std::bind(&UrMeasure::recordStopCallback, this, std::placeholders::_1));

    // Timer: write at MEASURE_PERIOD (continuous logging rate)
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(measure_period_),
        std::bind(&UrMeasure::timerCallback, this));

    // Mode banner
    if (mode_ == Mode::Continuous) {
        RCLCPP_INFO(this->get_logger(), "\033[32mRecording ready (CONTINUOUS @ MEASURE_PERIOD)\033[0m");
        RCLCPP_INFO(this->get_logger(), "Logging frequency: %.1f Hz", 1.0 / measure_period_);
        RCLCPP_INFO(this->get_logger(),
                    "Press ENTER or publish true to %s to start. Type -1 then ENTER or publish true to %s to stop.",
                    RECORD_START_TOPIC, RECORD_STOP_TOPIC);
    } else {
        RCLCPP_INFO(this->get_logger(), "\033[33mRecording ready (DISCRETE)\033[0m");
        RCLCPP_INFO(this->get_logger(),
                    "Press ENTER or publish true to %s to snapshot; type -1 then ENTER or publish true to %s to exit.",
                    RECORD_START_TOPIC, RECORD_STOP_TOPIC);
    }

    startInputThread();

    RCLCPP_INFO(this->get_logger(), "UrMeasure node initialized for robot: %s", robot_name_.c_str());
}

// Destructor
UrMeasure::~UrMeasure()
{
    if (input_thread_.joinable()) input_thread_.join();

    if (cp_file_ && cp_file_->is_open()) cp_file_->close();
    if (cj_file_ && cj_file_->is_open()) cj_file_->close();
    if (cf_file_ && cf_file_->is_open()) cf_file_->close();
    if (cmdk_file_ && cmdk_file_->is_open()) cmdk_file_->close();
    if (tp_file_ && tp_file_->is_open()) tp_file_->close();
    if (tj_file_ && tj_file_->is_open()) tj_file_->close();
    if (tf_file_ && tf_file_->is_open()) tf_file_->close();

    RCLCPP_INFO(this->get_logger(), "\033[31mProgram was terminated\033[0m");
}

// Initialize files
bool UrMeasure::initializeFiles()
{
    std::string nist_ur_path = package_path_;

    std::string measured_dir = nist_ur_path + "/measured";
    if (!std::filesystem::exists(measured_dir)) {
        try {
            std::filesystem::create_directories(measured_dir);
            RCLCPP_INFO(this->get_logger(), "Created directory: %s", measured_dir.c_str());
        } catch (const std::filesystem::filesystem_error& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create directory %s: %s",
                         measured_dir.c_str(), e.what());
            return false;
        }
    }

    try {
        cp_file_   = std::make_unique<std::ofstream>(nist_ur_path + CP_PATH);
        cj_file_   = std::make_unique<std::ofstream>(nist_ur_path + CJ_PATH);
        cf_file_   = std::make_unique<std::ofstream>(nist_ur_path + CF_PATH);
        cmdk_file_ = std::make_unique<std::ofstream>(nist_ur_path + CMDK_PATH);

        tp_file_   = std::make_unique<std::ofstream>(nist_ur_path + TP_PATH);
        tj_file_   = std::make_unique<std::ofstream>(nist_ur_path + TJ_PATH);
        tf_file_   = std::make_unique<std::ofstream>(nist_ur_path + TF_PATH);

        // ✅ Open check (fixed)
        if (!cp_file_->is_open() || !cj_file_->is_open() ||
            !cf_file_->is_open() || !cmdk_file_->is_open() ||
            !tp_file_->is_open() || !tj_file_->is_open() ||
            !tf_file_->is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Error opening one or more files.");
            return false;
        }

        // Precision
        cp_file_->precision(3);   cj_file_->precision(3);   cf_file_->precision(3);   cmdk_file_->precision(3);
        tp_file_->precision(3);   tj_file_->precision(3);   tf_file_->precision(3);

        cp_file_->setf(std::ios::fixed);   cj_file_->setf(std::ios::fixed);   cf_file_->setf(std::ios::fixed);   cmdk_file_->setf(std::ios::fixed);
        tp_file_->setf(std::ios::fixed);   tj_file_->setf(std::ios::fixed);   tf_file_->setf(std::ios::fixed);

        RCLCPP_INFO(this->get_logger(), "All measurement files opened successfully.");
        return true;

    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Exception while opening files: %s", e.what());
        return false;
    }
}

// Input thread for terminal-based start/snapshot/stop control
void UrMeasure::startInputThread()
{
    input_thread_ = std::thread([this]() {
        std::string line;
        while (rclcpp::ok()) {
            if (!std::getline(std::cin, line)) {
                rclcpp::shutdown();
                break;
            }
            if (line == "-1") {
                requestStop("terminal");
            } else if (line.empty()) {
                requestStartOrSnapshot("terminal");
            } else {
                if (mode_ == Mode::Discrete) {
                    std::cout << "[Discrete] Press ENTER to record, or -1 then ENTER to exit.\n";
                } else {
                    std::cout << "[Continuous] Press ENTER to start, or -1 then ENTER to exit.\n";
                }
            }
        }
    });
}

// ---- Subscriber callbacks (buffer only, no file write)
void UrMeasure::currentJCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    std::lock_guard<std::mutex> lk(buf_mtx_);
    for (int i = 0; i < number_of_joints_ && i < static_cast<int>(msg->data.size()); i++)
        last_cj_[i] = msg->data[i];
    have_cj_ = true;
}

void UrMeasure::currentPCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    std::lock_guard<std::mutex> lk(buf_mtx_);
    for (int i = 0; i < 6 && i < static_cast<int>(msg->data.size()); i++)
        last_cp_[i] = msg->data[i];
    have_cp_ = true;
}

void UrMeasure::currentFCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    std::lock_guard<std::mutex> lk(buf_mtx_);
    for (int i = 0; i < 6 && i < static_cast<int>(msg->data.size()); i++)
        last_cf_[i] = msg->data[i];
    have_cf_ = true;
}

void UrMeasure::currentMDKCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    std::lock_guard<std::mutex> lk(buf_mtx_);
    for (int i = 0; i < 3 && i < static_cast<int>(msg->data.size()); i++)
        last_cmdk_[i] = msg->data[i];
    have_cmdk_ = true;
}

void UrMeasure::targetJCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    std::lock_guard<std::mutex> lk(buf_mtx_);
    for (int i = 0; i < number_of_joints_ && i < static_cast<int>(msg->data.size()); i++)
        last_tj_[i] = msg->data[i];
    have_tj_ = true;
}

void UrMeasure::targetPCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    std::lock_guard<std::mutex> lk(buf_mtx_);
    for (int i = 0; i < 6 && i < static_cast<int>(msg->data.size()); i++)
        last_tp_[i] = msg->data[i];
    have_tp_ = true;
}

void UrMeasure::targetFCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    std::lock_guard<std::mutex> lk(buf_mtx_);
    for (int i = 0; i < 6 && i < static_cast<int>(msg->data.size()); i++)
        last_tf_[i] = msg->data[i];
    have_tf_ = true;
}

void UrMeasure::recordStartCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (!msg->data) {
        return;
    }
    requestStartOrSnapshot("topic");
}

void UrMeasure::recordStopCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (!msg->data) {
        return;
    }
    requestStop("topic");
}

void UrMeasure::requestStartOrSnapshot(const char* source)
{
    if (mode_ == Mode::Discrete) {
        snap_trigger_.store(true);
        RCLCPP_INFO(this->get_logger(), "Discrete snapshot requested via %s.", source);
        return;
    }

    const bool was_active = continuous_recording_active_.exchange(true);
    if (!was_active) {
        RCLCPP_INFO(this->get_logger(), "Continuous recording started via %s.", source);
    } else {
        RCLCPP_INFO(this->get_logger(), "Continuous recording is already active.");
    }
}

void UrMeasure::requestStop(const char* source)
{
    if (!rclcpp::ok()) {
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Recording stop requested via %s. Shutting down...", source);
    rclcpp::shutdown();
}

// ---- Discrete snapshot writer (writes once when triggered)
void UrMeasure::writeDiscreteSnapshot()
{
    std::lock_guard<std::mutex> lk(buf_mtx_);

    if (cj_file_ && cj_file_->is_open() && have_cj_) {
        for (int i = 0; i < number_of_joints_; ++i) *cj_file_ << last_cj_[i] << "\t";
        *cj_file_ << "\n"; cj_file_->flush();
    }
    if (cp_file_ && cp_file_->is_open() && have_cp_) {
        for (int i = 0; i < 6; ++i) *cp_file_ << last_cp_[i] << "\t";
        *cp_file_ << "\n"; cp_file_->flush();
    }
    if (cf_file_ && cf_file_->is_open() && have_cf_) {
        for (int i = 0; i < 6; ++i) *cf_file_ << last_cf_[i] << "\t";
        *cf_file_ << "\n"; cf_file_->flush();
    }
    if (cmdk_file_ && cmdk_file_->is_open() && have_cmdk_) {
        // ✅ FIX: last_cmdk_ is size 3
        for (int i = 0; i < 3; ++i) *cmdk_file_ << last_cmdk_[i] << "\t";
        *cmdk_file_ << "\n"; cmdk_file_->flush();
    }

    if (tj_file_ && tj_file_->is_open() && have_tj_) {
        for (int i = 0; i < number_of_joints_; ++i) *tj_file_ << last_tj_[i] << "\t";
        *tj_file_ << "\n"; tj_file_->flush();
    }
    if (tp_file_ && tp_file_->is_open() && have_tp_) {
        for (int i = 0; i < 6; ++i) *tp_file_ << last_tp_[i] << "\t";
        *tp_file_ << "\n"; tp_file_->flush();
    }
    if (tf_file_ && tf_file_->is_open() && have_tf_) {
        for (int i = 0; i < 6; ++i) *tf_file_ << last_tf_[i] << "\t";
        *tf_file_ << "\n"; tf_file_->flush();
    }

    RCLCPP_INFO(this->get_logger(), "Snapshot saved (discrete).");
}

// Timer callback
void UrMeasure::timerCallback()
{
    static int counter = 0;
    counter++;

    if (mode_ == Mode::Discrete) {
        if (snap_trigger_.exchange(false)) {
            writeDiscreteSnapshot();
        }
        return;
    }

    if (!continuous_recording_active_.load()) {
        return;
    }

    // ✅ Continuous: log periodically at MEASURE_PERIOD
    std::lock_guard<std::mutex> lk(buf_mtx_);

    if (cj_file_ && cj_file_->is_open() && have_cj_) {
        for (int i = 0; i < number_of_joints_; ++i) *cj_file_ << last_cj_[i] << "\t";
        *cj_file_ << "\n"; cj_file_->flush();
    }
    if (cp_file_ && cp_file_->is_open() && have_cp_) {
        for (int i = 0; i < 6; ++i) *cp_file_ << last_cp_[i] << "\t";
        *cp_file_ << "\n"; cp_file_->flush();
    }
    if (cf_file_ && cf_file_->is_open() && have_cf_) {
        for (int i = 0; i < 6; ++i) *cf_file_ << last_cf_[i] << "\t";
        *cf_file_ << "\n"; cf_file_->flush();
    }
    if (cmdk_file_ && cmdk_file_->is_open() && have_cmdk_) {
        for (int i = 0; i < 3; ++i) *cmdk_file_ << last_cmdk_[i] << "\t";
        *cmdk_file_ << "\n"; cmdk_file_->flush();
    }

    if (tj_file_ && tj_file_->is_open() && have_tj_) {
        for (int i = 0; i < number_of_joints_; ++i) *tj_file_ << last_tj_[i] << "\t";
        *tj_file_ << "\n"; tj_file_->flush();
    }
    if (tp_file_ && tp_file_->is_open() && have_tp_) {
        for (int i = 0; i < 6; ++i) *tp_file_ << last_tp_[i] << "\t";
        *tp_file_ << "\n"; tp_file_->flush();
    }
    if (tf_file_ && tf_file_->is_open() && have_tf_) {
        for (int i = 0; i < 6; ++i) *tf_file_ << last_tf_[i] << "\t";
        *tf_file_ << "\n"; tf_file_->flush();
    }

    // Optional debug
    if (counter % std::max(1, static_cast<int>(10.0 / measure_period_)) == 0) {
        RCLCPP_DEBUG(this->get_logger(), "Measurement running... (count: %d)", counter);
    }
}

// ---- Main
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    // Startup mode selection (stdin)
    std::cout << "Select mode: (c) continuous, (d) discrete > ";
    std::string mode_input;
    std::getline(std::cin, mode_input);

    Mode mode = Mode::Continuous;
    if (!mode_input.empty() && (std::tolower(static_cast<unsigned char>(mode_input[0])) == 'd')) {
        mode = Mode::Discrete;
    }

    try {
        const auto config = loadInstalledRobotRuntimeConfig();
        auto measure_node = std::make_shared<UrMeasure>(mode, config);

        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(measure_node);

        if (mode == Mode::Discrete) {
            std::cout << "[Discrete] Press ENTER to record a snapshot.\n"
                         "[Discrete] Publish true to " << RECORD_START_TOPIC << " to record a snapshot.\n"
                      << "[Discrete] Type -1 then ENTER or publish true to " << RECORD_STOP_TOPIC << " to exit.\n";
        } else {
            std::cout << "[Continuous] Logging at MEASURE_PERIOD = " << (config.control_period * 5.0)
                      << " sec (" << (1.0 / (config.control_period * 5.0)) << " Hz)\n"
                      << "[Continuous] Press ENTER or publish true to " << RECORD_START_TOPIC << " to start.\n"
                      << "[Continuous] Type -1 then ENTER or publish true to " << RECORD_STOP_TOPIC << " to exit.\n";
        }

        executor.spin();

    } catch (const std::exception& e) {
        std::cerr << "Exception in main: " << e.what() << std::endl;
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
