#pragma once

/* Compile-time kinematics selection.
 * Runtime robot/ROS/joystick parameters are loaded from
 * Y2RobMotion/config/y2_rob_motion.yaml.
 */
#define ROBOT_KINEMATICS 2 // 0: KUKA_IIWA, 1: UR10, 2: UR10e(+ Simulation)

/* Kinematics of Robot */
#if (ROBOT_KINEMATICS == 0)
    #include "Y2Kinematics/KinematicsKUKAiiwa.hpp"
    typedef KinematicsKUKAiiwa BaseKinematics;
#elif (ROBOT_KINEMATICS == 1)
    #include "Y2Kinematics/KinematicsUR10.hpp"
    typedef KinematicsUR10 BaseKinematics;
#elif (ROBOT_KINEMATICS == 2)
    #include "Y2Kinematics/KinematicsUR10e.hpp"
    typedef KinematicsUR10e BaseKinematics;
#else
    #error "Invalid ROBOT_KINEMATICS value (0: KUKA, 1: UR10)"
#endif
