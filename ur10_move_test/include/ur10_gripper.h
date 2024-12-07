#ifndef GRIPPER_CONTROL_H
#define GRIPPER_CONTROL_H

#include <ros/ros.h>
#include <robotiq_2f_gripper_control/Robotiq2FGripper_robot_output.h>
#include <unistd.h>
#include <iostream>
//#include "robotiq_modbus_rtu/comModbusRtu.h"
extern int button_white;
extern int gripper_command_copy;
class GripperControl {
public:
    // 构造函数，初始化 ROS 相关内容
    GripperControl(ros::NodeHandle& nh);
    bool initializeModbusRTU();
    // 设置夹爪的闭合程度线性增长
    void closeGripperLinearly();

    // 更新夹爪的控制状态
    void updateGripperState();

private:
    // 发布夹爪控制命令
    ros::Publisher gripper_pub;

    // 夹爪控制命令
    robotiq_2f_gripper_control::Robotiq2FGripper_robot_output gripper_command;

    // 控制夹爪的激活状态
    bool gripper_active;
    int position = 0;
};

#endif


