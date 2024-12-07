#include "ur10_topic_subscriber.h"
#include "ur10_teleoperation.h"
#include "ur10_move.h"
#include "ur10_gripper.h"
#include <iostream>
#include <fstream>  // 引入文件流库
//数据采集的过程中 需要将摄像头和夹爪的数据同步采集 因此我们使用两个文件并行处理的方式 都在接收到第一个omnitouch的信号开始进行接收
//目前不知道需不需要进行ROS的时间线对齐 不过应该不需要

using namespace std;
Eigen::Matrix<double,3,1> init_position={0.206161,-0.535404,0.754031},P_Arm,P_Touch,touch_joint;
Eigen::Quaterniond init_pose(-0.1113622,0.1594877, -0.6497649, 0.734825),R_Arm,R_Touch;
Eigen::Matrix<double,6,1> target_position;
// 借助引入外部变量的方法可以修改参数
double current_joint_pos[6];
int button_grey=0,button_white=0,init_flag=0,init_joint_flag=0,write_flag=0;
Eigen::Matrix<double,6,1> move_joint_command;
int gripper_command_copy=0;
int first_flag=0; // 控制机械臂移动到初始化位置
int count_flag=0,button_white_flag=0,button_white_end_flag=1; // 用于统计计数
//如果该外部变量在其它main文件中没有出现 而.h文件中出现了 则会报错
double arm_target_pose[7] = {0.387899,-0.298762,0.444887,0.655676,0.246287,0.315617,0.640170},arm_current_pose[7];

int main(int argc, char *argv[])
{
    // 初始化 ROS 节点
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "robot_movement_node");
    ros::NodeHandle nh;
    // 创建TrajectoryController对象
    TrajectoryController controller("/scaled_pos_joint_traj_controller/follow_joint_trajectory");
    // 初始化客户端，等待服务器连接
    controller.initClient();
    // 创建订阅器对象
    TopicSubscriber subscriber(nh);
    // TFTransformHandler tfHandler; // TF变换不再接收 替换为正运动学
    // 创建 GripperControl 类的实例
    GripperControl gripper_control(nh);
    // 启动订阅器
    subscriber.start();
    // 设置目标位置
    UR10_Kinetics ik;
    //target_position=ik.ur10_inverse(init_position,current_joint_pos,init_pose);
    Teleoperation ur10;
    ros::Rate r(60); //验证完毕 话题读取和消息发布的频率基本一致
    //用于记录时间
    ros::Time prev = ros::Time::now();//用于记录运行时间
    ros::Time now = ros::Time::now();
    ros::Duration dt(0);
    double T=0,dt_s=0;
    //用于记录实验数据
    ofstream dataFile1;//写入估计位置
    const char* file_path1 = "ur10_trajectory.txt"; // 指定文件路径,每次运行前保存这次数据
    if (remove(file_path1) != 0) { // 尝试删除文件
    cout << "Failed to delete ft_sensor." << endl;} 
    else {cout << "File deleted successfully." << endl;}
    while(ros::ok())
    {
        gripper_control.closeGripperLinearly();
        if(init_joint_flag==1)
        {
            ik.forward(current_joint_pos); //正运动学大概有2-3mm的差距
            ur10.run();
            controller.setTargetPosition(move_joint_command);
            if(first_flag==0)
            {   
                controller.setDefaultDuration(5);
                controller.executeMovement();
                ros::Duration(5).sleep();
                first_flag=1;
            }
            controller.setDefaultDuration(0.1);
            controller.executeMovement();
            count_flag++;
            //时间更新
            now = ros::Time::now();dt = now - prev;prev = now;dt_s = dt.toSec();
            T=T+dt_s;
            if(first_flag==1 && write_flag==1)
            {
                //ROS_INFO("执行到这里了");
                arm_current_pose[0]=P_Arm[0];
                arm_current_pose[1]=P_Arm[1];
                arm_current_pose[2]=P_Arm[2];
                arm_current_pose[3]=R_Arm.x();
                arm_current_pose[4]=R_Arm.y();
                arm_current_pose[5]=R_Arm.z();
                arm_current_pose[6]=R_Arm.w();
                dataFile1.open("ur10_trajectory.txt", ofstream::app);
                //需要保存的数据：时间1维 机械臂三维目标位姿7维 机械臂三维实际位姿7维 机械臂关节角6维
                dataFile1 <<T<<" "<<
                arm_target_pose[0]<<" "<<arm_target_pose[1]<<" "<<arm_target_pose[2]<<" "<<arm_target_pose[3]<<" "<<
                arm_target_pose[4]<<" "<<arm_target_pose[5]<<" "<<arm_target_pose[6]<<" "<<
                arm_current_pose[0]<<" "<<arm_current_pose[1]<<" "<<arm_current_pose[2]<<" "<<arm_current_pose[3]<<" "<<
                arm_current_pose[4]<<" "<<arm_current_pose[5]<<" "<<arm_current_pose[6]<<" "<<
                current_joint_pos[0]<<" "<<current_joint_pos[1]<<" "<<current_joint_pos[2]<<" "<<current_joint_pos[3]<<" "<<
                current_joint_pos[4]<<" "<<current_joint_pos[5]<<" "<<current_joint_pos[6]<<" "<<gripper_command_copy<<" "<<
                endl;
                dataFile1.close();
            }
            //如果白色按键按下 则另起一行重新记录
            if(button_white==1 && button_white_flag==0)
            {
                if(button_white_end_flag%2==1)
                {
                    ROS_INFO("开始采集",button_white_end_flag);
                    write_flag=1;
                }
                if(button_white_end_flag%2==0)
                {
                    ROS_INFO("结束采集",button_white_end_flag);
                    
                    dataFile1.open("ur10_trajectory.txt", ofstream::app);
                    dataFile1<<endl;
                    dataFile1.close();
                    write_flag=0;
                }
                button_white_flag=1;
            }
            if(button_white==0 && button_white_flag==1)
            {
                
                button_white_end_flag=button_white_end_flag+1;
                button_white_flag=0;
            }

            // if(button_white==1 && button_white_flag==1 && button_white_end_flag==1)
            // {
            //     ROS_INFO("结束采集");
            //     dataFile1.open("ur10_trajectory.txt", ofstream::app);
            //     dataFile1<<endl;
            //     dataFile1.close();
            //     button_white_flag=0;
            //     button_white_end_flag=0;
            // }

        }
        
        r.sleep();
        ros::spinOnce();

    }
    return 0;
}



// //时间更新
//             now = ros::Time::now();
//             dt = now - prev;
//             prev = now;
//             dt_s = dt.toSec();
//             T=T+dt_s;
//             cout<<"moveit运行时间间隔:"<<dt_s<<endl;










