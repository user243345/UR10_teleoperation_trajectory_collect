#include "ur10_gripper.h"
#include <ros/ros.h>
#include <robotiq_2f_gripper_control/Robotiq2FGripper_robot_output.h>
#include <unistd.h>
#include <iostream>
int button_white=0;
int gripper_command_copy=0;
int main(int argc, char *argv[])
{
    // 初始化 ROS 节点
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "robot_movement_node");
    ros::NodeHandle nh;
    GripperControl gripper_control(nh);
    ros::Rate r(100);
    // 初始化发布器
    ros::Publisher gripper_pub = nh.advertise<robotiq_2f_gripper_control::Robotiq2FGripper_robot_output>("/Robotiq2FGripperRobotOutput", 10);
    robotiq_2f_gripper_control::Robotiq2FGripper_robot_output gripper_command;
    
    ros::Duration(1).sleep();
    while(ros::ok())
    {
        gripper_control.closeGripperLinearly();
        r.sleep();
        ros::spinOnce();
    }
    /* code */
    return 0;
}
