from ur_python_pkg import ur10_inverse
import rospy
from ur_python_pkg import ur10_move 
import numpy as np

def read_txt_file(file_path):
    """
    从 txt 文件中读取位置和四元数。
    假设文件以列存储，前 3 列为位置，后 4 列为四元数。
    返回：位置和四元数的列表
    """
    data = np.loadtxt(file_path)
    positions = data[:, 1:4]  # 前三列为位置
    orientations = data[:, 4:8]  # 后四列为四元数
    return positions, orientations
def send_command_list(positions,orientations):
    l=len(positions)
    
    for i in range(0,l):
        if positions[i,0]~=0:
            send(positions[i],orientations[i])
    ros.sleep()


if __name__=='__main__':
    rospy.init_node('test_node')
    ur10_inverse.say_hello_world()
    txt_file_path = "ur10_trajectory_copy.txt"  # 替换为实际路径
    # 读取位姿数据
    positions, orientations = read_txt_file(txt_file_path)
    #print(positions)
    controller = ur10_move.ur10_move("/scaled_pos_joint_traj_controller/follow_joint_trajectory", 2.0)
    controller.sayhello()
    l=len(positions)
    for i in range(0,l):
        print(positions[i])

