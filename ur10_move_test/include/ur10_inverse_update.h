#ifndef UR10_INVERSE_H_2
#define UR10_INVERSE_H_2

#include <iostream>
#include <math.h>
#include <stdio.h>
#include "geometry_msgs/Quaternion.h"
#include <Eigen/Dense>
#include "ros/ros.h"

extern Eigen::Matrix<double,3,1> P_Arm;
extern Eigen::Quaterniond R_Arm;
class UR10_Kinetics{
    private:
    //ur10数据
    const double d[6 + 1] = { 0, 0.1273,0,0,0.163941,0.1157,0.0922 };//第0个不用
    const double a[6] = { 0,-0.612,-0.5723,0,0,0 };
    const double alpha[6] = { 1.570796, 0, 0, 1.570796, -1.570796, 0 };
    const double angle_limits[6]={2*M_PI,2*M_PI,2*M_PI,2*M_PI,2*M_PI,2*M_PI};
    const double PI = M_PI;
    const int NUM_JOINTS = 6;
    Eigen::Matrix<double,3,3> Rotation_Matrix;

    public:
    Eigen::Matrix<double,6,1> ur10_solution_filter_test(const double current[6],Eigen::Matrix<double,8,6> q_solutions);
    Eigen::Matrix<double,3,1> forward(double theta_input[6]);//正向运动学 输出末端三维位置
    Eigen::Matrix<double,8,6> ur10_inverse_Quat(double target_pos[6], Eigen::Quaterniond quat);//逆向运动学 输出八组解
    Eigen::Matrix<double,6,1> ur10_inverse(Eigen::Matrix<double,3,1> target_pos, const double current[6], Eigen::Quaterniond quat); 
    
};

#endif