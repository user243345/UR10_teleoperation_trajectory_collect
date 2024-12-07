#include "ur10_teleoperation.h"

void Teleoperation::init_teleoperation()
{
  //保存机械臂和touch的当前位置
  init_arm_position=P_Arm;
  init_touch_position=P_Touch;
  init_arm_orientation=R_Arm;
  init_touch_orientation=R_Touch;
  init_touch_joint=touch_joint;
  move_joint_command=ur10.ur10_inverse(init_arm_position,current_joint_pos,init_arm_orientation);
}


void Teleoperation::update_teleoperation()
{
  target_pos=calculateTranslationChange(init_touch_position,P_Touch);
  //delta_rotation=calculateRotationChange(init_touch_orientation,R_Touch);
  delta_rotation=calculateEulerChange(init_touch_joint,touch_joint);
  target_rotation=delta_rotation*init_arm_orientation; // 左定右前 左乘是绕固定轴 右乘是绕当前轴
  for(int i=0;i<3;i++)
  {
    arm_target_pose[i]=target_pos[i];
  }
  arm_target_pose[3]=target_rotation.x();
  arm_target_pose[4]=target_rotation.y();
  arm_target_pose[5]=target_rotation.z();
  arm_target_pose[6]=target_rotation.w();
  //std::cout<<arm_target_pose[0]<<std::endl;
  if(orientation_flag==0)
  {
    move_joint_command=ur10.ur10_inverse(target_pos,current_joint_pos,init_arm_orientation);
  }
  else
  {
    move_joint_command=ur10.ur10_inverse(target_pos,current_joint_pos,target_rotation);
  }
  

}

void Teleoperation::init_move_teleoperation()
{
  //yan关节位置：1.5296378135681152, -1.771846119557516, 1.646470069885254, -1.3023131529437464, 1.6049401760101318, 0.6965883374214172
  //yan末端位置：0.206161,-0.535404,0.954031
  //yan四元数：-0.007825,0.019776,-0.379199,-0.925071 xyzw
  //position:0.284905,-0.385701,0.826727
  //orientation:-0.663027,-0.230332,-0.282804,-0.653731
  //倾向于去选择正的四元数解wxyz 赋值顺序是wxyz
  // init_position={0.458715,-0.344205,0.484659};
  // init_orientation={-0.675476,-0.520509,0.029293,-0.521483};
  init_position={0.387899,-0.298762,0.444887};
  init_orientation={0.640170,0.655676,0.246287,0.315617}; //wxyz的赋值顺序
  move_joint_command=ur10.ur10_inverse(init_position,current_joint_pos,init_orientation);

}

void Teleoperation::run()
{
  if(first_flag==0)
  {
    init_move_teleoperation();
    //first_flag=1;
  }
  // 先根据按键是否按下判断初始化
  // 如果按键按下则保存当前位姿
  else if(button_grey==0 && init_flag==1)
  { //松开按键的这一刻
    init_flag=0;
    //发送当前的关节角直接让机械臂急停
    end_move_teleoperation();

  }
  else if(button_grey==1 && init_flag==0)
  {
    init_teleoperation();
    init_flag=1;
  }
  // 如果不是第一次按下进入到更新函数 在更新函数中发送指令
  else if(button_grey==1 && init_flag==1)
  {
    update_teleoperation();
  }

}


void Teleoperation::end_move_teleoperation()
{
  for(int i=0;i<6;i++)
  {
    move_joint_command[i]=current_joint_pos[i];
  }
  
}

Eigen::Vector3d Teleoperation::calculateTranslationChange(Eigen::Matrix<double,3,1> init_position, Eigen::Matrix<double,3,1> end_position)
{
  //计算touch三个轴的平移量变化 Touch的x轴是机械臂的Y轴 Touch的Y轴是机械臂的-X轴 Z轴相同
  for(int i=0;i<3;i++)
  {
    d_pos_touch[i]=end_position[i]-init_position[i];
  }
  d_pos_arm[0]=-d_pos_touch[1];
  d_pos_arm[1]=d_pos_touch[0];
  d_pos_arm[2]=d_pos_touch[2];
  //先将d_pos_arm旋转一个45° 
  Eigen::Vector3d axis(0, 0, 1); // z 轴
  // 创建旋转矩阵
  Eigen::AngleAxisd rotation(angle, axis); // 旋转45度绕z轴
  d_pos_arm=rotation*d_pos_arm;
  //将x轴的平移量变化映射到UR10机械臂的Y轴 将Touch的y轴变化映射到UR10机械臂的-X轴
  for(int i=0;i<3;i++)
  {
    target[i]=init_arm_position[i]+K[i]*d_pos_arm[i];
  }
  //ROS_INFO("target_z:%2f",target[2]);
  return target;
}
// 将四元数转换为欧拉角 (roll, pitch, yaw)
Eigen::Vector3d quaternionToEuler(const Eigen::Quaterniond& q) {
    // 提取四元数分量
    double w = q.w();
    double x = q.x();
    double y = q.y();
    double z = q.z();

    // 计算欧拉角
    double roll = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
    double pitch = asin(2 * (w * y - z * x));
    double yaw = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));

    return Eigen::Vector3d(roll, pitch, yaw); // 返回 [roll, pitch, yaw]
}
// 欧拉角转四元数的函数
Eigen::Quaterniond eulerToQuaternion(double roll, double pitch, double yaw) {
    // 计算每个旋转分量
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    // 计算四元数的各个分量
    Eigen::Quaterniond q;
    q.w() = cr * cp * cy + sr * sp * sy;
    q.x() = sr * cp * cy - cr * sp * sy;
    q.y() = cr * sp * cy + sr * cp * sy;
    q.z() = cr * cp * sy - sr * sp * cy;

    return q;
}
Eigen::Quaterniond Teleoperation::calculateRotationChange(Eigen::Quaterniond init_orientation, Eigen::Quaterniond end_orientation)
{
  // 调整旋转轴为zxy顺序
  // 计算旋转变化四元数 旋转变化有问题 Touch第四个关节对应机械臂的第六个关节 Touch的第五个关节对应机械臂的第四个关节 Touch的第六个关节对应机械臂的第五个关节
  Eigen::Quaterniond rotation_change = end_orientation * init_orientation.inverse();
  // 将旋转变化转换为轴角表示
  Eigen::AngleAxisd angle_axis(rotation_change);
  // 缩小旋转角度
  double scaled_angle = angle_axis.angle() * scale_factor;
  // 使用缩小后的角度重新生成四元数
  Eigen::Quaterniond scaled_quaternion(Eigen::AngleAxisd(scaled_angle, angle_axis.axis()));
  //后续将这个四元数进行坐标转换
  Eigen::Quaterniond rotation_modified=Eigen::Quaterniond(cos_half, 0.0, 0.0, sin_half);
  // Eigen::Quaterniond scaled_quaternion_modified=rotation_modified*scaled_quaternion*rotation_modified.conjugate();
  //将四元数转化为欧拉角
  d_Euler=quaternionToEuler(scaled_quaternion);
  d_Euler_copy=d_Euler;
  //然后将欧拉角的顺序调换
  d_Euler[0]=d_Euler_copy[1];
  d_Euler[1]=d_Euler_copy[0];
  d_Euler[2]=d_Euler_copy[2];
  //将欧拉角转化为四元数
  scaled_quaternion=eulerToQuaternion(d_Euler[0],d_Euler[1],d_Euler[2]);
  //然后再进行坐标转换
  Eigen::Quaterniond scaled_quaternion_modified=rotation_modified*scaled_quaternion*rotation_modified.conjugate();
  return scaled_quaternion;

}

Eigen::Quaterniond Teleoperation::calculateEulerChange(Eigen::Matrix<double,3,1> init_joint, Eigen::Matrix<double,3,1> end_joint)
{
  //计算两次欧拉角的偏差
  // 计算欧拉角的变化量
  double alpha = end_joint[0]-init_joint[0];
  double beta = end_joint[1]-init_joint[1];
  double gamma = end_joint[2]-init_joint[2];

  // 创建旋转矩阵并设置为 ZYX 顺序的欧拉角
  Eigen::Matrix3d delta_touch_rotation;
  delta_touch_rotation = Eigen::AngleAxisd(-alpha, Eigen::Vector3d::UnitZ()) *
                         Eigen::AngleAxisd(-beta, Eigen::Vector3d::UnitY()) *
                         Eigen::AngleAxisd(gamma, Eigen::Vector3d::UnitX());
  //改变欧拉角的赋值顺序
  //将旋转矩阵绕z轴旋转顺时针135°
  // 定义绕 Z 轴顺时针旋转的角度 (单位：弧度)
  double angle_rad = -135.0 * M_PI / 180.0; // 顺时针旋转 135 度，转换为弧度
  // 使用 Eigen::AngleAxisd 生成旋转矩阵
  Eigen::Matrix3d rotation_matrix;
  rotation_matrix = Eigen::AngleAxisd(angle_rad, Eigen::Vector3d::UnitZ());
  delta_touch_rotation=rotation_matrix.transpose()*delta_touch_rotation*rotation_matrix;
  //将旋转矩阵转化为四元数
  Eigen::Quaterniond quaternion(delta_touch_rotation);
  // 将旋转变化转换为轴角表示
  Eigen::AngleAxisd angle_axis(quaternion);
  // 缩小旋转角度
  double scaled_angle = angle_axis.angle() * scale_factor;
  // 使用缩小后的角度重新生成四元数
  Eigen::Quaterniond scaled_quaternion(Eigen::AngleAxisd(scaled_angle, angle_axis.axis()));
  //将四元数传递回逆运动学
  return scaled_quaternion;
}








