#ifndef UR10_MOVE_H
#define UR10_MOVE_H

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <vector>
#include <string>
#include <Eigen/Dense>
class TrajectoryController {
private:
    
    std::vector<std::string> joint_names = {"shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"}; // 关节名称
    double default_duration;              // 默认持续时间
    double current_position[6];           // 当前关节位置

public:
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> client; // Action客户端
    // 构造函数，初始化Action客户端
    TrajectoryController(const std::string& action_name, double default_duration = 3.0);

    // 初始化客户端，等待服务器连接
    void initClient();

    // 设置目标位置
    void setTargetPosition(Eigen::Matrix<double,6,1> position);

    // 执行移动
    void executeMovement();

    // 设置默认持续时间
    void setDefaultDuration(double duration);
};


#endif









