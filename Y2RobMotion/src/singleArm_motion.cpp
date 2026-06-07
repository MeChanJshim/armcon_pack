#include "Y2RobMotion/robot_motion.hpp"
#include "Y2RobMotion/setup_parameters.hpp"
#include "Y2RobMotion/runtime_config.hpp"

class singleArm_motion
{
    public:
        singleArm_motion(rclcpp::Node::SharedPtr node, const std::string& RB_name, 
            double control_period, int numOfJoint, const YMatrix& HTMEE2TCP);

        bool jointsReceived() const {
            return ROBOTMotion->jointsReceived();
        }
    
        void start(bool start_flag = true) {
            ROBOTMotion->start(start_flag);
        }

    private:
    
        std::unique_ptr<robot_motion> ROBOTMotion;
        double robot_ctrPeriod;
        rclcpp::Node::SharedPtr node_;
        
};

singleArm_motion::singleArm_motion(rclcpp::Node::SharedPtr node, const std::string& RB_name, 
        double control_period, int numOfJoint, const YMatrix& HTMEE2TCP)
: node_(node)
{
    ROBOTMotion = std::make_unique<robot_motion>(node_, RB_name, control_period, numOfJoint, HTMEE2TCP);
}


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    /* ROS node generation as hip instance */

    auto node = std::make_shared<rclcpp::Node>("SingleArm");
    const auto config = loadInstalledRobotRuntimeConfig();
    singleArm_motion robot_SingleArm(
        node,
        config.robot_name,
        config.control_period,
        config.number_of_joints,
        config.ee_to_tcp);

    RCLCPP_INFO(node->get_logger(), "Waiting for joint states...");
    
    /* Node generation */
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    /* Bringup execution check */
    while (rclcpp::ok() && !robot_SingleArm.jointsReceived()) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    RCLCPP_INFO(node->get_logger(), "Joint states received!");
    
    /* Node execution */
    robot_SingleArm.start(true);
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}
