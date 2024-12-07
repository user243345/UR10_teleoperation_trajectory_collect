#ifndef UR10_TELEOPERATION_H
#define UR10_TELEOPERATION_H

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "ur10_inverse_update.h"
#include "ur10_move.h"
#include <string>


extern double current_joint_pos[6];
extern int button_grey,button_white,init_flag;
extern Eigen::Matrix<double,3,1> P_Arm,P_Touch;
extern Eigen::Quaterniond R_Arm,R_Touch;
extern Eigen::Matrix<double,6,1> move_joint_command;
extern int first_flag;
extern double arm_target_pose[7];
extern Eigen::Matrix<double,3,1> touch_joint;

class Teleoperation {
public:
    void init_teleoperation();
    void update_teleoperation();
    void init_move_teleoperation();
    void run();
    void end_move_teleoperation();

    // 计算平移变化
    Eigen::Vector3d calculateTranslationChange(Eigen::Matrix<double,3,1> init_position, Eigen::Matrix<double,3,1> end_position);
    // 计算旋转变化
    Eigen::Quaterniond calculateRotationChange(Eigen::Quaterniond init_orientation, Eigen::Quaterniond end_orientation);
    Eigen::Quaterniond calculateEulerChange(Eigen::Matrix<double,3,1> init_joint, Eigen::Matrix<double,3,1> end_joint);
private:
    Eigen::Matrix<double,3,1> init_arm_position,init_touch_position,delta_pos,init_position,init_touch_joint;
    Eigen::Matrix<double,3,1> current_arm_position,current_touch_position,target_pos;
    Eigen::Quaterniond init_arm_orientation,init_touch_orientation,delta_rotation,init_orientation;
    Eigen::Quaterniond current_arm_orientation,current_touch_orientation,target_rotation;
    double K[3]={1,1,1};
    double scale_factor=0.3; //姿态变换尺度缩放因子
    UR10_Kinetics ur10;
    int orientation_flag=0; // 是否开放姿态变换
    Eigen::Matrix<double,3,1> d_pos_touch,d_pos_arm;
    Eigen::Vector3d target;
    double angle = M_PI / 4; // UR10机械臂的45°修正 
    double quat_angle = -135.0 * M_PI / 180.0; // 转为弧度，顺时针为负
    double cos_half = cos(quat_angle / 2);
    double sin_half = sin(quat_angle / 2);
    Eigen::Matrix<double,3,1> d_Euler,d_Euler_copy;
    
};

#endif 
