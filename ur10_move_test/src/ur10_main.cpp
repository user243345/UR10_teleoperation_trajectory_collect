#include "ur10_topic_subscriber.h"
#include "ur10_teleoperation.h"
#include "ur10_move.h"
#include "ur10_gripper.h"
// 初始状态下六个关节角
// [1.4539947509765625, -1.8614228407489222, 1.7357966899871826, 0.38932549953460693, 0.9975152015686035, 0.08349409699440002]
using namespace std;
Eigen::Matrix<double,3,1> init_position={0.203371, -0.436228, 0.816995},P_Arm,P_Touch;
Eigen::Quaterniond init_pose(-0.1113622,0.1594877, -0.6497649, 0.734825),R_Arm,R_Touch;
Eigen::Matrix<double,6,1> target_position;
// 借助引入外部变量的方法可以修改参数
double current_joint_pos[6];
int button_grey=0,button_white=0,init_flag=0,init_joint_flag=0;
Eigen::Matrix<double,6,1> move_joint_command;
int first_flag=0; // 控制机械臂移动到初始化位置


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
    // 创建坐标变换对象
    TFTransformHandler tfHandler;
    // 启动订阅器
    subscriber.start();
    // 设置目标位置
    UR10_Kinetics ik;
    //先进行初始化 封装为函数
    Teleoperation ur10;
    ros::Rate r(100); //验证完毕 话题读取和消息发布的频率基本一致
    while(ros::ok())
    {
        if(tfHandler.getTransform("base", "wrist_3_link", P_Arm, R_Arm, 2.0))
        {
            ur10.run();
            controller.setTargetPosition(move_joint_command);
            controller.setDefaultDuration(0.1);
            controller.executeMovement();
            if(first_flag==0)
            {ros::Duration(3).sleep();first_flag=1;}
        }
        r.sleep();
        ros::spinOnce();

    }
    ros::spin();
    /* code */
    return 0;
}

    // //用于记录时间
    // ros::Time prev = ros::Time::now();//用于记录运行时间
    // ros::Time now = ros::Time::now();
    // ros::Duration dt(0);
    // double T=0,dt_s=0;

// //时间更新
//             now = ros::Time::now();
//             dt = now - prev;
//             prev = now;
//             dt_s = dt.toSec();
//             T=T+dt_s;
//             cout<<"moveit运行时间间隔:"<<dt_s<<endl;










