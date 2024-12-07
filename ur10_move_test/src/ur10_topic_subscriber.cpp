#include "ur10_topic_subscriber.h"

// 构造函数：初始化订阅者
TopicSubscriber::TopicSubscriber(ros::NodeHandle& nh) {
    //回调函数是类的成员函数 则使用this
    arm_state_sub_ = nh.subscribe<sensor_msgs::JointState>(
        "/joint_states", 1, &TopicSubscriber::armStateUpdateCallback, this);
    touch_pose_sub_ = nh.subscribe<geometry_msgs::PoseStamped>(
        "/phantom/phantom/pose", 1, &TopicSubscriber::touchPoseStateCallback, this);
    touch_button_sub_ = nh.subscribe<omni_msgs::OmniButtonEvent>(
        "/phantom/phantom/button", 1, &TopicSubscriber::touchButtonStateCallback, this);
    touch_joint_sub_ = nh.subscribe<sensor_msgs::JointState>(
        "/phantom/phantom/joint_states", 1, &TopicSubscriber::touchJointStateCallback, this);
    
}

// 显式启动订阅（可选，用于控制流程）
void TopicSubscriber::start() {
    ROS_INFO("Starting topic subscribers...");
}

// 回调函数实现：机械臂关节角状态更新
void TopicSubscriber::armStateUpdateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    //ROS_INFO("Received joint states: %lu joints", msg->position.size());
    current_joint_pos[0]=msg->position[2];
    current_joint_pos[1]=msg->position[1];
    current_joint_pos[2]=msg->position[0];
    current_joint_pos[3]=msg->position[3];
    current_joint_pos[4]=msg->position[4];
    current_joint_pos[5]=msg->position[5];
    if(init_joint_flag==0)
    init_joint_flag=1;
}

// 回调函数实现：Touch 位姿更新
void TopicSubscriber::touchPoseStateCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    // ROS_INFO("Received touch pose: position (%f, %f, %f)", 
    //          msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    P_Touch[0]=msg->pose.position.x;
    P_Touch[1]=msg->pose.position.y;
    P_Touch[2]=msg->pose.position.z;
    R_Touch.x()=msg->pose.orientation.x;
    R_Touch.y()=msg->pose.orientation.y;
    R_Touch.z()=msg->pose.orientation.z;
    R_Touch.w()=msg->pose.orientation.w;
}

// 回调函数实现：Touch 按键状态更新
void TopicSubscriber::touchButtonStateCallback(const omni_msgs::OmniButtonEvent::ConstPtr& msg) {
    // ROS_INFO("Received touch button event: button state (%d, %d)", 
    //          msg->grey_button, msg->white_button);
    button_grey=msg->grey_button;
    button_white=msg->white_button;
    // if(button_grey==0 && init_flag==1)
    // {
    //     init_flag=0;
    // }
}
void TopicSubscriber::touchJointStateCallback(const sensor_msgs::JointState::ConstPtr& msg){
    touch_joint[0]=msg->position[5]; //定轴
    touch_joint[1]=-msg->position[3];
    touch_joint[2]=-msg->position[4];
}


