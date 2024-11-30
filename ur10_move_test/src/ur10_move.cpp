#include "ur10_move.h"

// 构造函数：初始化Action客户端和默认参数
TrajectoryController::TrajectoryController(const std::string& action_name, double default_duration)
    : client(action_name, true), default_duration(default_duration) {
    // 初始化当前关节位置为零
    std::fill(std::begin(current_position), std::end(current_position), 0.0);
}

// 等待服务器连接
void TrajectoryController::initClient() {
    ROS_INFO("Waiting for action server to start...");
    client.waitForServer();
    ROS_INFO("Action server connected.");
}

// 设置目标位置
void TrajectoryController::setTargetPosition(Eigen::Matrix<double,6,1> position) {
    for (int i = 0; i < 6; ++i) {
        current_position[i] = position[i];
    }
}

// 执行移动
void TrajectoryController::executeMovement() {
    // 创建轨迹目标
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory.joint_names = joint_names;  // 设置关节名称

    // 创建并填充一个轨迹点
    trajectory_msgs::JointTrajectoryPoint point;
    for (int i = 0; i < 6; ++i) {
        point.positions.push_back(current_position[i]); // 填充目标位置
    }
    point.time_from_start = ros::Duration(default_duration); // 设置到达时间
    goal.trajectory.points.push_back(point); // 添加轨迹点

    // 发送轨迹目标
    //ROS_INFO("Sending trajectory goal...");
    client.sendGoal(goal);
    //ROS_INFO("Trajectory goal sent.");
}

// 设置默认持续时间
void TrajectoryController::setDefaultDuration(double duration) {
    default_duration = duration;
}







