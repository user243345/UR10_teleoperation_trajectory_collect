#include "ur10_teleoperation.h"

void Teleoperation::init_teleoperation()
{
  //保存机械臂和touch的当前位置
  init_arm_position=P_Arm;
  init_touch_position=P_Touch;
  init_arm_orientation=R_Arm;
  init_touch_orientation=R_Touch;
  move_joint_command=ur10.ur10_inverse(init_arm_position,current_joint_pos,init_arm_orientation);
}


void Teleoperation::update_teleoperation()
{
  target_pos=calculateTranslationChange(init_touch_position,P_Touch);
  delta_rotation=calculateRotationChange(init_touch_orientation,R_Touch);
  target_rotation=delta_rotation*init_arm_orientation;
  for(int i=0;i<3;i++)
  {
    arm_target_pose[i]=target_pos[i];
  }
  arm_target_pose[3]=target_rotation.x();
  arm_target_pose[4]=target_rotation.y();
  arm_target_pose[5]=target_rotation.z();
  arm_target_pose[6]=target_rotation.w();
  std::cout<<arm_target_pose[0]<<std::endl;
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
  //倾向于去选择正的四元数解wxyz
  // init_position={0.458715,-0.344205,0.484659};
  // init_orientation={-0.675476,-0.520509,0.029293,-0.521483};
  init_position={0.206161,-0.535404,0.754031};
  init_orientation={-0.925071,-0.007825,0.019776,-0.379199};
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
Eigen::Quaterniond Teleoperation::calculateRotationChange(Eigen::Quaterniond init_orientation, Eigen::Quaterniond end_orientation)
{
  // 计算旋转变化四元数
  Eigen::Quaterniond rotation_change = end_orientation * init_orientation.inverse();
  // 将旋转变化转换为轴角表示
  Eigen::AngleAxisd angle_axis(rotation_change);
  // 缩小旋转角度
  double scaled_angle = angle_axis.angle() * scale_factor;
  // 使用缩小后的角度重新生成四元数
  Eigen::Quaterniond scaled_quaternion(Eigen::AngleAxisd(scaled_angle, angle_axis.axis()));
  return scaled_quaternion;
}



