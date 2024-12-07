#include "ur10_gripper.h"

GripperControl::GripperControl(ros::NodeHandle& nh) {
    // 初始化发布器
    gripper_pub = nh.advertise<robotiq_2f_gripper_control::Robotiq2FGripper_robot_output>("/Robotiq2FGripperRobotOutput", 10);

    // 初始化夹爪命令，激活夹爪，设置速度、力量
    gripper_command.rACT = 0;  // 激活夹爪 
    gripper_command.rGTO = 0;  // 使能夹爪控制
    gripper_command.rSP = 0; // 设置最大速度
    gripper_command.rFR = 0; // 设置最大力量
    gripper_command.rPR = 0;   // 初始位置，夹爪完全打开
    gripper_pub.publish(gripper_command);
    ros::Duration(1).sleep();
    gripper_command.rACT = 1; 
    gripper_command.rGTO = 1;  // 使能夹爪控制
    gripper_command.rSP = 255; // 设置最大速度
    gripper_command.rFR = 150; // 设置最大力量
    gripper_command.rPR = 0;   // 初始位置，夹爪完全打开
    gripper_pub.publish(gripper_command);
    ros::Duration(1).sleep();
    // 初始状态：夹爪未激活
    gripper_active = true;


}

void GripperControl::closeGripperLinearly() {
    if(button_white==1)
    {
        position=position+5;
        gripper_command.rPR = position;
        gripper_command_copy=position;
        if(position>250)
        {
            position=250;
        }
        gripper_pub.publish(gripper_command);
        //ros::Duration(0.1).sleep();
    }
    else if(button_white==0)
    {
        position=position-10;
        if(position<0)
        {
            position=0;
        }
        gripper_command_copy=position;
        gripper_command.rPR = position;
        gripper_pub.publish(gripper_command);
        //ros::Duration(0.1).sleep();
    }
}

void GripperControl::updateGripperState() {
    gripper_active = true;
    if (gripper_active) {
        ROS_INFO("Gripper activated.");
    } else {
        ROS_INFO("Gripper deactivated.");
    }
}

// bool GripperControl::initializeModbusRTU() {
//     const std::string device = "/dev/ttyUSB0"; // 固定路径

//     // 创建 Modbus RTU 客户端
//     robotiq_modbus_rtu::comModbusRtu modbus_client;

//     // 尝试连接到设备
//     if (!modbus_client.connectToDevice(device)) {
//         std::cerr << "Failed to connect to Modbus RTU device at: " << device << std::endl;
//         return false;
//     }

//     std::cout << "Successfully initialized Modbus RTU communication on device: " << device << std::endl;
//     return true;
// }







