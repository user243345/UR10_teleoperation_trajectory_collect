#ifndef UR10_TOPIC_SUBSCRIBER_H
#define UR10_TOPIC_SUBSCRIBER_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <omni_msgs/OmniButtonEvent.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include "Eigen/Dense"
extern double current_joint_pos[6];
// extern double current_touch_position[3];
// extern double current_touch_orientation[4];
extern Eigen::Matrix<double,3,1> P_Touch;
extern Eigen::Quaterniond R_Touch;
extern int button_grey,button_white,init_flag;
extern int init_joint_flag;
extern double arm_current_pose[7];
extern Eigen::Matrix<double,3,1> touch_joint;

// 话题订阅者类
class TopicSubscriber {
public:
    // 构造函数
    TopicSubscriber(ros::NodeHandle& nh);

    // 显式启动订阅
    void start();

private:
    // 回调函数
    void armStateUpdateCallback(const sensor_msgs::JointState::ConstPtr& msg);
    void touchPoseStateCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void touchButtonStateCallback(const omni_msgs::OmniButtonEvent::ConstPtr& msg);
    void touchJointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
    // ROS 订阅者和其他成员变量
    ros::Subscriber arm_state_sub_;
    ros::Subscriber touch_pose_sub_;
    ros::Subscriber touch_button_sub_;
    ros::Subscriber touch_joint_sub_;
    tf2_ros::Buffer tf_buffer_; // 如果需要缓冲区
};

//坐标变换监听类
class TFTransformHandler
{
public:
    TFTransformHandler()
        : tfListener(tfBuffer) // 初始化 tf 监听器
    {}

    // 获取指定 frame 的变换信息，带等待时间
    bool getTransform(const std::string& source_frame, const std::string& target_frame,
                      Eigen::Vector3d& translation, Eigen::Quaterniond& rotation, double timeout = 1.0)
    {
        ros::Time start_time = ros::Time::now();
        while ((ros::Time::now() - start_time).toSec() < timeout)
        {
            try
            {
                // 尝试获取变换
                geometry_msgs::TransformStamped transformStamped =
                    tfBuffer.lookupTransform(source_frame, target_frame, ros::Time(0), ros::Duration(0.1));

                // 提取位移信息
                translation.x() = transformStamped.transform.translation.x;
                translation.y() = transformStamped.transform.translation.y;
                translation.z() = transformStamped.transform.translation.z;
                
                // 提取旋转信息
                rotation.x() = transformStamped.transform.rotation.x;
                rotation.y() = transformStamped.transform.rotation.y;
                rotation.z() = transformStamped.transform.rotation.z;
                rotation.w() = transformStamped.transform.rotation.w;
                //确保读取到的数据正确
                if(abs(translation[0])>0.001 && abs(translation[1])>0.001 && abs(translation[2])>0.001)
                {
                    ROS_INFO("末端位置:%2f,%2f,%2f",translation.x(),translation.y(),translation.z());
                    ROS_INFO("末端四元数:%2f,%2f,%2f,%2f",rotation.x(),rotation.y(),rotation.z(),rotation.w());
                    return true; // 成功获取变换
                }
                
            }
            catch (tf2::TransformException& ex)
            {
                // 继续等待
                ros::Duration(0.1).sleep();
            }
        }

        ROS_WARN("TF Transform Exception: Unable to get transform from '%s' to '%s' within timeout",
                 source_frame.c_str(), target_frame.c_str());
        return false;
    }

private:
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;
};

#endif 
