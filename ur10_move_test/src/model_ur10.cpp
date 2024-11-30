#include <ros/ros.h>
#include "ur10_move.h"
#include "ur10_inverse_update.h"

#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/PointStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h" 

//用于接收touch消息
#include "sensor_msgs/JointState.h"
#include "omni_msgs/OmniButtonEvent.h"
#include "gazebo_msgs/LinkStates.h"
using namespace std;
double init_joint_angle[6]={-38,-72,-115,17,50,176},init_joint_rad[6]={-0.739563290272848, -1.4319804350482386, -2.128582779561178, 0.4339379072189331, 1.5360922813415527, 3.071777820587158};
double current_pose[6],P_Touch[3],P_0_Touch[3],R_Touch[4],R_0_Touch[4],P_Arm[3],P_0_Arm[3],R_Arm[4],R_0_Arm[4],result[6],q[6],R_Arm_Copy[4],result_last[6];//存储数据用数组
int start_joint_flag=0,start_touch_flag=0,button1=0,first_flag=0,start_arm_flag=0,init_num=1,test_flag=0,try_flag=0,count=0;//标志位
Eigen::Vector3d forward_pos;
double K[3]={0.7,0.7,0.7};
double target_init[6]={0.203371, -0.436228, 0.816995,0.82,1.175,-0.614},target_pos[6],pos_init[4]={-0.9999997, 0, 0, 0.0007963},scale_factor=0.5;
double result_init[6]={0.5937490342381082, -1.4532574144612664, 1.8960501993508299, 1.125822249745503, 1.5675832594116619, -0.97811};
double init_Quat[4]={0.1594877, -0.6497649, 0.734825, -0.1113622};
Eigen::Quaterniond init_quat;
Eigen::Quaterniond quat_0={-0.1113622, 0.1594877, -0.6497649, 0.734825};
std::vector<std::string> joint_names = {"shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"};// 定义关节名称
bool chushihua;
Eigen::Quaterniond quat_Arm,quat_0_Arm,quat_0_Touch,quat_Touch,relative_rotation,arm_rotation,quat_inv;

// 在程序启动时等待服务器连接一次
void initClient(actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>& client) {
    ROS_INFO("Waiting for action server to start...");
    client.waitForServer();
}

// 封装的移动机器人函数：设置一个轨迹点
void moveToPosition(actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>& client, const double position[6], double duration) 
{
    // // 等待服务端准备就绪
    // ROS_INFO("Waiting for action server to start...");
    // client.waitForServer();
    // 创建轨迹目标
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory.joint_names = joint_names;  // 设定关节名称
    // 创建并填充一个轨迹点
    trajectory_msgs::JointTrajectoryPoint point;
    // 将数组元素添加到轨迹点的positions中
    for (int i = 0; i < 6; i++) 
    {
        point.positions.push_back(position[i]);  // 将数组值逐个添加到向量中
        //point.velocities.push_back(velocity[i]);
    }
    point.time_from_start = ros::Duration(duration);  // 设定到达该点的时间
    goal.trajectory.points.push_back(point);

    // 发送轨迹目标并等待结果
    ROS_INFO("Sending trajectory goal...");
    client.sendGoal(goal);
}

void Arm_state_update(const sensor_msgs::JointState::ConstPtr &msg) // 机械臂关节角更新回调函数
{
    current_pose[0]=msg->position[2];
    current_pose[1]=msg->position[1];
    current_pose[2]=msg->position[0];
    current_pose[3]=msg->position[3];
    current_pose[4]=msg->position[4];
    current_pose[5]=msg->position[5];
    if(start_joint_flag==0)start_joint_flag=1; 
}

void Touch_pose_state(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    P_Touch[0]=msg->pose.position.x;
    P_Touch[1]=msg->pose.position.y;
    P_Touch[2]=msg->pose.position.z;
    R_Touch[0]=msg->pose.orientation.x;
    R_Touch[1]=msg->pose.orientation.y;
    R_Touch[2]=msg->pose.orientation.z;
    R_Touch[3]=msg->pose.orientation.w;
    if(start_touch_flag==0) start_touch_flag=1;
}

void Touch_button_state(const omni_msgs::OmniButtonEvent::ConstPtr &touch_button)
{

  button1 = touch_button->grey_button;
  if(button1==0 && first_flag==1)
  {
    first_flag=0;
    start_arm_flag=3;
    //P_Z=P[2];
  }
}

Eigen::Vector3d calculateTranslationChange(const double init_position[3],const double end_position[3])
{
  double d_pos[3];
  Eigen::Vector3d target;
  for(int i=0;i<3;i++)
  {
    d_pos[i]=end_position[i]-init_position[i];
    target[i]=P_0_Arm[i]+K[i]*d_pos[i];
  }
  
  //ROS_INFO("target_z:%2f",target[2]);
  return target;
}

Eigen::Quaterniond calculateRotationChange(const double init[4], const double end[4])
{
    // 将输入的四元数数组转换为 Eigen 的四元数
    Eigen::Quaterniond init_quat(init[3], init[0], init[1], init[2]);
    Eigen::Quaterniond end_quat(end[3], end[0], end[1], end[2]);
    // 计算旋转变化四元数
    Eigen::Quaterniond rotation_change = end_quat * init_quat.inverse();
    // 将旋转变化转换为轴角表示
    Eigen::AngleAxisd angle_axis(rotation_change);
    // 缩小旋转角度
    double scaled_angle = angle_axis.angle() * scale_factor;
    // 使用缩小后的角度重新生成四元数
    Eigen::Quaterniond scaled_quaternion(Eigen::AngleAxisd(scaled_angle, angle_axis.axis()));
    return scaled_quaternion;
}

int main(int argc, char** argv) {
    // 初始化 ROS 节点
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "robot_movement_node");
    ros::NodeHandle nh;
    
    //-------------------------------------创建话题接收对象-------------------------------------
    tf2_ros::Buffer buffer;
    ros::Subscriber sub0 = nh.subscribe<sensor_msgs::JointState>("/joint_states",20,Arm_state_update);//机械臂关节角
    ros::Subscriber sub1 = nh.subscribe<geometry_msgs::PoseStamped>("/phantom/phantom/pose",20,Touch_pose_state);//touch位姿
    ros::Subscriber sub2 = nh.subscribe<omni_msgs::OmniButtonEvent>("/phantom/phantom/button",20,Touch_button_state);//touch按键
    // for (int i=0;i<6;i++)
    // {
    //     init_joint_rad[i]=(init_joint_angle[i])/180*M_PI;
    // }
    
    // 定义目标关节位置（单个轨迹点）

    //target_position = {0.0, -1.57, 1.57, 0.0, 1.57, 0.0};  // 目标位置（弧度）
    double duration = 3;  // 设定执行该移动的时间（秒）
    // 调用封装的移动函数，只设置一个轨迹点
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> client("/scaled_pos_joint_traj_controller/follow_joint_trajectory", true);
    moveToPosition(client,init_joint_rad, duration);
    UR10_Kinetics ur10;
    Eigen::Matrix<double,8,6> q_solutions;
    Eigen::Matrix<double,6,1> q_filer;
    Eigen::Matrix3d init_touch_Rotation,init_arm_Rotation,delta_R,A,B;
    init_quat.x()=init_Quat[0];init_quat.y()=init_Quat[1];init_quat.z()=init_Quat[2];init_quat.w()=init_Quat[3];
    q_solutions=ur10.ur10_inverse_Quat(target_init,quat_0);
    q_filer=ur10.ur10_solution_filter_test(current_pose,q_solutions);
    ros::Duration(5).sleep();
    cout<<"初始化完成"<<endl;
    ros::Rate r(100);
    //-------------------------------------设置监听对象-------------------------------
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped transformStamped;
    //用于记录时间
    ros::Time prev = ros::Time::now();//用于记录运行时间
    ros::Time now = ros::Time::now();
    ros::Duration dt(0);
    double T=0,dt_s=0;
    initClient(client);
    while(ros::ok())
    {
        try
        {
        // 获取 base 到 wrist_3_link 的变换
        transformStamped = tfBuffer.lookupTransform("base", "wrist_3_link", ros::Time(0));
        P_Arm[0]=transformStamped.transform.translation.x;P_Arm[1]=transformStamped.transform.translation.y;
        P_Arm[2]=transformStamped.transform.translation.z;
        R_Arm[0]=transformStamped.transform.rotation.x;R_Arm[1]=transformStamped.transform.rotation.y;
        R_Arm[2]=transformStamped.transform.rotation.z;R_Arm[3]=transformStamped.transform.rotation.w;
        ROS_INFO("末端姿态:%2f %2f %2f %2f",R_Arm[0],R_Arm[1],R_Arm[2],R_Arm[3]);
        try_flag++;

        //ROS_INFO("末端位置:%2f %2f %2f",P_Arm[0],P_Arm[1],P_Arm[2]);
        if(start_joint_flag==1 && chushihua==false && try_flag>0)
        {
            // init_quat.x()=R_Arm[0];init_quat.y()=R_Arm[1];init_quat.z()=R_Arm[2];init_quat.w()=R_Arm[3];
            q_solutions=ur10.ur10_inverse_Quat(target_init,quat_0);
            q_filer=ur10.ur10_solution_filter_test(current_pose,q_solutions);
            for(int i=0;i<6;i++){result[i]=q_filer(i,0);}
            result[5]=3.07;
            duration=2;
            moveToPosition(client,result, duration);
            ros::Duration(5).sleep();
            ROS_INFO("第二次初始化成功");
            chushihua=true;
            //button1=1;
            try_flag=0;

            // for(int i=0;i<8;i++)
            // ROS_INFO("解:%2f %2f %2f %2f %2f %2f",q_solutions(i,0),q_solutions(i,1),q_solutions(i,2),q_solutions(i,3),q_solutions(i,4),q_solutions(i,5));
            // ros::Duration(1).sleep();
            // if(P_Arm[2]>0.84) {test_flag=0;}
            // if(test_flag==0) {target_init[2]=target_init[2]-0.05;}

            // if(P_Arm[2]<0.60){ test_flag=1;}
            // if(test_flag==1) {target_init[2]=target_init[2]+0.05;}
            // ROS_INFO("机械臂高度:%2f",P_Arm[2]);
            // start_joint_flag=2;
            // break;
        }
        
            
        if(button1==1 && chushihua==true && first_flag==0 && try_flag>1)
        {
          ROS_INFO("初始化末端姿态:%2f %2f %2f %2f",R_Arm[0],R_Arm[1],R_Arm[2],R_Arm[3]);
          copy(R_Arm,R_Arm+4,R_0_Arm);
          copy(P_Arm,P_Arm+3,P_0_Arm);
          ROS_INFO("初始化末端四元数:%2f,%2f,%2f,%2f",R_0_Arm[0],R_0_Arm[1],R_0_Arm[2],R_0_Arm[3]);
          copy(R_Touch,R_Touch+4,R_0_Touch);
          copy(P_Touch,P_Touch+3,P_0_Touch);
          quat_0_Arm.w()=R_0_Arm[3];quat_0_Arm.x()=R_0_Arm[0];quat_0_Arm.y()=R_0_Arm[1];quat_0_Arm.z()=R_0_Arm[2];
          ROS_INFO("赋值后末端四元数:%2f,%2f,%2f,%2f",quat_0_Arm.x(),quat_0_Arm.y(),quat_0_Arm.z(),quat_0_Arm.w());
          ROS_INFO("赋值后三维位置:%2f %2f %2f",P_0_Arm[0],P_0_Arm[1],P_0_Arm[2]);
          //quat_0_Touch.w()=R_0_Touch[3];quat_0_Touch.x()=R_0_Touch[0];quat_0_Touch.y()=R_0_Touch[1];quat_0_Touch.z()=R_0_Touch[2];
          
          for(int i=0;i<3;i++) {target_init[i]=P_0_Arm[i];}
          q_solutions=ur10.ur10_inverse_Quat(target_init,quat_0_Arm);//逆运动学计算,输出为8组６个关节角q_solutions
          //q_solutions=ur_10.ur10_inverse_Rotation(target_pos,init_arm_Rotation);//逆运动学计算,输出为8组６个关节角q_solutions
          q_filer=ur10.ur10_solution_filter_test(current_pose,q_solutions);
          for(int i=0;i<6;i++){result[i]=q_filer(i,0);}
          result[5]=3.07;
          duration=5;
          moveToPosition(client,result, duration);
          first_flag=1;
          for(int i=0;i<6;i++){result_last[i]=q_filer(i,0);}
          try_flag=0;
        }
        if(first_flag==1 && button1==1 && try_flag>1)
        {

          relative_rotation=calculateRotationChange(R_0_Touch,R_Touch);
          arm_rotation = relative_rotation * quat_0_Arm;
          Eigen::Vector3d trans=calculateTranslationChange(P_0_Touch,P_Touch);
          for(int j=0;j<3;j++) {target_pos[j]=trans[j];}
          q_solutions=ur10.ur10_inverse_Quat(target_pos,arm_rotation);//这里的姿态控制不一定需要开启 如果开启请添加姿态阻尼系数
          q_filer=ur10.ur10_solution_filter_test(current_pose,q_solutions);
          for(int i=0;i<6;i++) {result[i]=q_filer(i,0);} result[5]=3.07;
          duration=0.3;
          //时间更新
          now = ros::Time::now();
          dt = now - prev;
          prev = now;
          dt_s = dt.toSec();
          T=T+dt_s;
          cout<<"moveit运行时间间隔:"<<dt_s<<endl;
          moveToPosition(client,result, duration);//看一下是否需要添加速度变量
          //for(int i=0;i<6;i++){result_last[i]=q_filer(i,0);}
        }
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        r.sleep();
        ros::spinOnce();


    }

    return 0;
}















