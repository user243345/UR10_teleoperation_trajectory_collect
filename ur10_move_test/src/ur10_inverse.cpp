#include "ur10_inverse_update.h"


// 计算两组关节角度之间的欧几里得距离
double calculateDistance(const double current[6], const double solution[6]) {
    double distance = 0.0;
    for (int i = 0; i < 5; i++) {
        distance += std::pow(current[i] - solution[i], 2);
    }
    return std::sqrt(distance);
}

//输入:当前关节位置（数组） 逆解结果（指针）|| 滤波函数 在八组解中选择出距离当前位置最近的逆解
Eigen::Matrix<double,6,1> UR10_Kinetics::ur10_solution_filter_test(const double current[6],Eigen::Matrix<double,8,6> q_solutions)
{
  double solution[6],distance,min_distance;
  //double* optimalSolution= new double[6];
  min_distance=2*M_PI*6;
  int isSolutionValid_flag=1;
  Eigen::Matrix<double,6,1> optimalSolution;
  //遍历8个逆运动学解
  for(int i=0;i<8;i++)
  {
	isSolutionValid_flag=1;
	for(int j=0;j<6;j++)
	{
		if(q_solutions(i,j)<-2*M_PI) q_solutions(i,j)=q_solutions(i,j)+2*M_PI;
		if(q_solutions(i,j)>2*M_PI)  q_solutions(i,j)=q_solutions(i,j)-2*M_PI;
		// if(q_solutions(i,j)<-M_PI) isSolutionValid_flag=0;
		// if(q_solutions(i,j)>M_PI) isSolutionValid_flag=0;
		// if(isSolutionValid_flag==0) break;
		solution[j]=q_solutions(i,j);

	}
    if(isSolutionValid_flag==1)
	{
	distance=calculateDistance(current,solution);
    //std::cout<<i<<"第i个方案的距离"<<distance<<std::endl;
    if(distance < min_distance)
    {
        min_distance=distance;
        for(int m=0;m<6;m++)
        {optimalSolution(m,0)=solution[m];}
    }

	}
	
  }
    return optimalSolution;
};

Eigen::Quaterniond rotationMatrixToQuaternion(const Eigen::Matrix3d& R) {
    // 检查旋转矩阵是否有效
    if (!R.isApprox(R.transpose().inverse()) || std::abs(R.determinant() - 1.0) > 1e-6) {
        throw std::invalid_argument("Input matrix is not a valid rotation matrix");
    }

    // 计算迹
    double trace = R.trace();
    double w, x, y, z;

    if (trace > 0) {
        // 公式适用于迹大于零的情况
        double s = std::sqrt(trace + 1.0) * 2; // s = 4 * w
        w = 0.25 * s;
        x = (R(2, 1) - R(1, 2)) / s;
        y = (R(0, 2) - R(2, 0)) / s;
        z = (R(1, 0) - R(0, 1)) / s;
    } else if (R(0, 0) > R(1, 1) && R(0, 0) > R(2, 2)) {
        // R(0, 0) 为最大值时
        double s = std::sqrt(1.0 + R(0, 0) - R(1, 1) - R(2, 2)) * 2; // s = 4 * x
        w = (R(2, 1) - R(1, 2)) / s;
        x = 0.25 * s;
        y = (R(0, 1) + R(1, 0)) / s;
        z = (R(0, 2) + R(2, 0)) / s;
    } else if (R(1, 1) > R(2, 2)) {
        // R(1, 1) 为最大值时
        double s = std::sqrt(1.0 + R(1, 1) - R(0, 0) - R(2, 2)) * 2; // s = 4 * y
        w = (R(0, 2) - R(2, 0)) / s;
        x = (R(0, 1) + R(1, 0)) / s;
        y = 0.25 * s;
        z = (R(1, 2) + R(2, 1)) / s;
    } else {
        // R(2, 2) 为最大值时
        double s = std::sqrt(1.0 + R(2, 2) - R(0, 0) - R(1, 1)) * 2; // s = 4 * z
        w = (R(1, 0) - R(0, 1)) / s;
        x = (R(0, 2) + R(2, 0)) / s;
        y = (R(1, 2) + R(2, 1)) / s;
        z = 0.25 * s;
    }

    // 构造并返回四元数
    Eigen::Quaterniond quaternion(w, x, y, z);
    quaternion.normalize(); // 确保四元数单位化
    return quaternion;
}
//正运动学，用于检验
Eigen::Matrix<double,3,1> UR10_Kinetics::forward(double theta_input[6])
{
	Eigen::Matrix4d T[6];//为了和theta对应，0不用
	for (int i = 0; i < 6; i++)
	{
		T[i](0, 0) = cos(theta_input[i]);
		T[i](0, 1) = -sin(theta_input[i]) * cos(alpha[i]);
		T[i](0, 2) = sin(theta_input[i]) * sin(alpha[i]);
		T[i](0, 3) = a[i] * cos(theta_input[i]);
		T[i](1, 0) = sin(theta_input[i]);
		T[i](1, 1) = cos(theta_input[i]) * cos(alpha[i]);
		T[i](1, 2) = -cos(theta_input[i]) * sin(alpha[i]);
		T[i](1, 3) = a[i] * sin(theta_input[i]);
		T[i](2, 0) = 0;
		T[i](2, 1) = sin(alpha[i]);
		T[i](2, 2) = cos(alpha[i]);
		T[i](2, 3) = d[i+1];
		T[i](3, 0) = 0;
		T[i](3, 1) = 0;
		T[i](3, 2) = 0;
		T[i](3, 3) = 1;
	}
	Eigen::Matrix4d T06 = T[0] * T[1] * T[2] * T[3] * T[4] * T[5];
	Eigen::Matrix<double,3,1> pos_x_y_z={T06(0, 3),T06(1, 3),T06(2, 3)};
	//std::cout << "检验得:X=" << T06(0, 3) << "    Y=" << T06(1, 3)<< "     Z=" << T06(2, 3) << std::endl;
	P_Arm=pos_x_y_z;
    Rotation_Matrix=T06.block<3, 3>(0, 0);
	R_Arm=rotationMatrixToQuaternion(Rotation_Matrix);
	//ROS_INFO("四元数:%2f,%2f,%2f,%2f",R_Arm.x(),R_Arm.y(),R_Arm.z(),R_Arm.w());
	return pos_x_y_z;
};

//输入：目标位置 返回：Eigen矩阵形式的八组解 仿真环境中误差均在1cm之内
Eigen::Matrix<double,8,6> UR10_Kinetics::ur10_inverse_Quat(double target_pos[6], Eigen::Quaterniond quat) //前一项输入位置，后一项输入四元数
{
	double x=target_pos[0],y=target_pos[1],z=target_pos[2],X=target_pos[3],Y=target_pos[4],Z=target_pos[5];
    Eigen::Matrix4d T06;//末端到极坐标系变换矩阵
	Eigen::Matrix<double,8,6> q_solutions;
	double T[4][4],theta[9][7];
	//Eigen::Quaterniond quat(Rotation[3],Rotation[0],Rotation[1],Rotation[2]);
	
    Eigen::Matrix3d Rotation_Matrix;
	Rotation_Matrix=quat.toRotationMatrix();
	for(int i=0;i<3;i++)
	{
		for(int j=0;j<3;j++)
		{
			T06(i,j)=Rotation_Matrix(i,j);
		}
	}
	T06(0, 3) = x;
	T06(1, 3) = y;
	T06(2, 3) = z;
	T06(3, 3) = 1;

	//前三项是三维位置，后三项是RPY角度 绕静轴RPY的公式来计算旋转矩阵
	// T06(0, 0) = cos(Z) * cos(Y);
	// T06(0, 1) = cos(Z) * sin(X) * sin(Y) - cos(X) * sin(Z);
	// T06(0, 2) = sin(X) * sin(Z) + cos(X) * cos(Z) * sin(Y);
	// T06(0, 3) = -x;
	// T06(1, 0) = cos(Y) * sin(Z);
	// T06(1, 1) = cos(X) * cos(Z) + sin(Z) * sin(Y) * sin(X);
	// T06(1, 2) = cos(X) * sin(Y) * sin(Z) - cos(Z) * sin(X);
	// T06(1, 3) = -y;
	// T06(2, 0) = -sin(Y);
	// T06(2, 1) = cos(Y) * sin(X);
	// T06(2, 2) = cos(X) * cos(Y);
	// T06(2, 3) = z;
	// T06(3, 0) = 0;
	// T06(3, 1) = 0;
	// T06(3, 2) = 0;
	// T06(3, 3) = 1;
	//2.求解
	double A, B, C, R, D, E, F, G, M, N;//用大写字母替代常数
	//注意，由于数组下标从0开始的问题，矩阵第一行第一列的元素是(0,0)
    //theta1 两个解
	A = d[6] * T06(1, 2) - T06(1, 3);
	B = d[6] * T06(0, 2) - T06(0, 3);
	C = d[4];
	R = A*A+B*B;
	//第一个解，赋给一到四组
	theta[1][1] = atan2(A, B) - atan2(C, sqrt(A * A + B * B - C * C));
	//ROS_INFO("jisuan:%2f",A * A + B * B - C * C);
	theta[2][1] = theta[1][1];
	theta[3][1] = theta[1][1];
	theta[4][1] = theta[1][1];
	//第二个解，赋给五到八组
	theta[5][1] = atan2(A, B) - atan2(C, -sqrt(A * A + B * B - C * C));
	theta[6][1] = theta[5][1];
	theta[7][1] = theta[5][1];
	theta[8][1] = theta[5][1];
	//theta5 四个解
	//由theta[1][1]产生的第一个解，赋给一到二组
	A = sin(theta[1][1]) * T06(0, 2) - cos(theta[1][1]) * T06(1, 2);
	theta[1][5] = acos(A);
	theta[2][5] = theta[1][5];
	//由theta[1][1]产生的第二个解，赋给三到四组
	theta[3][5] = -acos(A);
	theta[4][5] = theta[3][5];
	//由theta[5][1]产生的第一个解，赋给五到六组
	A = sin(theta[5][1]) * T06(0, 2) - cos(theta[5][1]) * T06(1, 2);
	theta[5][5] = acos(A);
	theta[6][5] = theta[5][5];
	//由theta[5][1]产生的第二个解，赋给七到八组
	theta[7][5] = -acos(A);
	theta[8][5] = theta[7][5];
	//theta6 四个解
	for (int i = 1; i <= 8; i = i + 2)
	{
		A = (sin(theta[i][1]) * T06(0, 0) - cos(theta[i][1]) * T06(1, 0));
		B = (sin(theta[i][1]) * T06(0, 1) - cos(theta[i][1]) * T06(1, 1));
		C = sin(theta[i][5]);
		D = A * A + B * B - C * C;

		if ((C <= -0.00001) || (C >= 0.000001)) {
			theta[i][6] = atan2(A, B) - atan2(C, 0.00);
			theta[i + 1][6] = atan2(A, B) - atan2(C, 0.00);
		}
		else
		{
			theta[i][6] = 0;
			theta[i + 1][6] = 0;
		}

	}
	//theta3 8组解
	for (int i = 1; i <= 8; i = i + 2)
	{
		C = cos(theta[i][1]) * T06(0, 0) + sin(theta[i][1]) * T06(1, 0);
		D = cos(theta[i][1]) * T06(0, 1) + sin(theta[i][1]) * T06(1, 1);
		E = cos(theta[i][1]) * T06(0, 2) + sin(theta[i][1]) * T06(1, 2);
		F = cos(theta[i][1]) * T06(0, 3) + sin(theta[i][1]) * T06(1, 3);
		G = cos(theta[i][6]) * T06(2, 1) + sin(theta[i][6]) * T06(2, 0);
		A = d[5] * (sin(theta[i][6]) * C + cos(theta[i][6]) * D) - d[6] * E + F;
		B = T06(2, 3) - d[1] - T06(2, 2) * d[6] + d[5] * G;
		//theta3
		if (A * A + B * B <= (a[2 - 1] + a[3 - 1]) * (a[2 - 1] + a[3 - 1])) {
			theta[i][3] = acos((A * A + B * B - a[2 - 1] * a[2 - 1] - a[3 - 1] * a[3 - 1]) / (2 * a[2 - 1] * a[3 - 1]));
            theta[i + 1][3] = -theta[i][3];
		}
		else
		{
			theta[i][3] = 0;
			theta[i + 1][3] = 0;
		}
	}
	//theta2 theta4
	for (int i = 1; i <= 8;  i++)
	{
		C = cos(theta[i][1]) * T06(0, 0) + sin(theta[i][1]) * T06(1, 0);
		D = cos(theta[i][1]) * T06(0, 1) + sin(theta[i][1]) * T06(1, 1);
		E = cos(theta[i][1]) * T06(0, 2) + sin(theta[i][1]) * T06(1, 2);
		F = cos(theta[i][1]) * T06(0, 3) + sin(theta[i][1]) * T06(1, 3);
		G = cos(theta[i][6]) * T06(2, 1) + sin(theta[i][6]) * T06(2, 0);
		A = d[5] * (sin(theta[i][6]) * C + cos(theta[i][6]) * D) - d[6] * E + F;
		B = T06(2, 3) - d[1] - T06(2, 2) * d[6] + d[5] * G;
		M = ((a[3 - 1] * cos(theta[i][3]) + a[2 - 1]) * B - a[3 - 1] * sin(theta[i][3]) * A) / (a[2 - 1] * a[2 - 1] + a[3 - 1] * a[3 - 1] + 2 * a[2 - 1] * a[3 - 1] * cos(theta[i][3]));
		N = (A + a[3 - 1] * sin(theta[i][3]) * M) / (a[3 - 1] * cos(theta[i][3]) + a[2 - 1]);
		theta[i][2] = atan2(M, N);
        
		//theta4
		theta[i][4] = atan2((-sin(theta[i][6]) * C - cos(theta[i][6]) * D), G) - theta[i][2] - theta[i][3] ;
	}
	for(int i=1;i<9;i++)
	{
		for(int j=1;j<7;j++)
		{
			if(theta[i][j]>2*M_PI) theta[i][j]=theta[i][j]-2*M_PI;
			if(theta[i][j]<-2*M_PI) theta[i][j]=theta[i][j]+2*M_PI;
			q_solutions(i-1,j-1)=theta[i][j];
		}
	}
	return q_solutions;
	// //输出并检验
	// for (int i = 1; i <= 8; i++)
	// {
	// 	cout << "第" << i << "组解：" << endl;
	// 	for (int j = 1; j <= 6; j++)
	// 		cout << "theta" << j << "=" << theta[i][j] *180/3.1415926 << "  ";
	// 	cout << endl;
	// 	kinematics(theta[i]);
	// 	cout << endl << endl;
	// }
}
//输入：目标位置 返回：Eigen矩阵形式的八组解 仿真环境中误差均在1cm之内
Eigen::Matrix<double,6,1> UR10_Kinetics::ur10_inverse(Eigen::Matrix<double,3,1> target_pos, const double current[6], Eigen::Quaterniond quat) //前一项输入位置，后一项输入四元数
{
	double x=target_pos[0],y=target_pos[1],z=target_pos[2];
    Eigen::Matrix4d T06;//末端到极坐标系变换矩阵
	Eigen::Matrix<double,8,6> q_solutions;
	double T[4][4],theta[9][7];
	//Eigen::Quaterniond quat(Rotation[3],Rotation[0],Rotation[1],Rotation[2]);
	
    Eigen::Matrix3d Rotation_Matrix;
	Rotation_Matrix=quat.toRotationMatrix();
	for(int i=0;i<3;i++)
	{
		for(int j=0;j<3;j++)
		{
			T06(i,j)=Rotation_Matrix(i,j);
		}
	}
	T06(0, 3) = x;
	T06(1, 3) = y;
	T06(2, 3) = z;
	T06(3, 3) = 1;
	//2.求解
	double A, B, C, R, D, E, F, G, M, N;//用大写字母替代常数
	//注意，由于数组下标从0开始的问题，矩阵第一行第一列的元素是(0,0)
    //theta1 两个解
	A = d[6] * T06(1, 2) - T06(1, 3);
	B = d[6] * T06(0, 2) - T06(0, 3);
	C = d[4];
	R = A*A+B*B;
	//第一个解，赋给一到四组
	theta[1][1] = atan2(A, B) - atan2(C, sqrt(A * A + B * B - C * C));
    // for(int i=0;i<3;i++)
	// ROS_INFO("T:%2f,%2f,%2f",T06(i,0),T06(i,1),T06(i,2));
	// ROS_INFO("jisuan:%2f",A * A + B * B - C * C);
	//ROS_INFO("A:%2f,B:%2f,C:%2f",A,B,C);
	theta[2][1] = theta[1][1];
	theta[3][1] = theta[1][1];
	theta[4][1] = theta[1][1];
	//第二个解，赋给五到八组
	theta[5][1] = atan2(A, B) - atan2(C, -sqrt(A * A + B * B - C * C));
	theta[6][1] = theta[5][1];
	theta[7][1] = theta[5][1];
	theta[8][1] = theta[5][1];
	//theta5 四个解
	//由theta[1][1]产生的第一个解，赋给一到二组
	A = sin(theta[1][1]) * T06(0, 2) - cos(theta[1][1]) * T06(1, 2);
	theta[1][5] = acos(A);
	theta[2][5] = theta[1][5];
	//由theta[1][1]产生的第二个解，赋给三到四组
	theta[3][5] = -acos(A);
	theta[4][5] = theta[3][5];
	//由theta[5][1]产生的第一个解，赋给五到六组
	A = sin(theta[5][1]) * T06(0, 2) - cos(theta[5][1]) * T06(1, 2);
	theta[5][5] = acos(A);
	theta[6][5] = theta[5][5];
	//由theta[5][1]产生的第二个解，赋给七到八组
	theta[7][5] = -acos(A);
	theta[8][5] = theta[7][5];
	//theta6 四个解
	for (int i = 1; i <= 8; i = i + 2)
	{
		A = (sin(theta[i][1]) * T06(0, 0) - cos(theta[i][1]) * T06(1, 0));
		B = (sin(theta[i][1]) * T06(0, 1) - cos(theta[i][1]) * T06(1, 1));
		C = sin(theta[i][5]);
		D = A * A + B * B - C * C;

		if ((C <= -0.00001) || (C >= 0.000001)) {
			theta[i][6] = atan2(A, B) - atan2(C, 0.00);
			theta[i + 1][6] = atan2(A, B) - atan2(C, 0.00);
		}
		else
		{
			theta[i][6] = 0;
			theta[i + 1][6] = 0;
		}

	}
	//theta3 8组解
	for (int i = 1; i <= 8; i = i + 2)
	{
		C = cos(theta[i][1]) * T06(0, 0) + sin(theta[i][1]) * T06(1, 0);
		D = cos(theta[i][1]) * T06(0, 1) + sin(theta[i][1]) * T06(1, 1);
		E = cos(theta[i][1]) * T06(0, 2) + sin(theta[i][1]) * T06(1, 2);
		F = cos(theta[i][1]) * T06(0, 3) + sin(theta[i][1]) * T06(1, 3);
		G = cos(theta[i][6]) * T06(2, 1) + sin(theta[i][6]) * T06(2, 0);
		A = d[5] * (sin(theta[i][6]) * C + cos(theta[i][6]) * D) - d[6] * E + F;
		B = T06(2, 3) - d[1] - T06(2, 2) * d[6] + d[5] * G;
		//theta3
		if (A * A + B * B <= (a[2 - 1] + a[3 - 1]) * (a[2 - 1] + a[3 - 1])) {
			theta[i][3] = acos((A * A + B * B - a[2 - 1] * a[2 - 1] - a[3 - 1] * a[3 - 1]) / (2 * a[2 - 1] * a[3 - 1]));
            theta[i + 1][3] = -theta[i][3];
		}
		else
		{
			theta[i][3] = 0;
			theta[i + 1][3] = 0;
		}
	}
	//theta2 theta4
	for (int i = 1; i <= 8;  i++)
	{
		C = cos(theta[i][1]) * T06(0, 0) + sin(theta[i][1]) * T06(1, 0);
		D = cos(theta[i][1]) * T06(0, 1) + sin(theta[i][1]) * T06(1, 1);
		E = cos(theta[i][1]) * T06(0, 2) + sin(theta[i][1]) * T06(1, 2);
		F = cos(theta[i][1]) * T06(0, 3) + sin(theta[i][1]) * T06(1, 3);
		G = cos(theta[i][6]) * T06(2, 1) + sin(theta[i][6]) * T06(2, 0);
		A = d[5] * (sin(theta[i][6]) * C + cos(theta[i][6]) * D) - d[6] * E + F;
		B = T06(2, 3) - d[1] - T06(2, 2) * d[6] + d[5] * G;
		M = ((a[3 - 1] * cos(theta[i][3]) + a[2 - 1]) * B - a[3 - 1] * sin(theta[i][3]) * A) / (a[2 - 1] * a[2 - 1] + a[3 - 1] * a[3 - 1] + 2 * a[2 - 1] * a[3 - 1] * cos(theta[i][3]));
		N = (A + a[3 - 1] * sin(theta[i][3]) * M) / (a[3 - 1] * cos(theta[i][3]) + a[2 - 1]);
		theta[i][2] = atan2(M, N);
        
		//theta4
		theta[i][4] = atan2((-sin(theta[i][6]) * C - cos(theta[i][6]) * D), G) - theta[i][2] - theta[i][3] ;
	}
	for(int i=1;i<9;i++)
	{
		for(int j=1;j<7;j++)
		{
			if(theta[i][j]>2*M_PI) theta[i][j]=theta[i][j]-2*M_PI;
			if(theta[i][j]<-2*M_PI) theta[i][j]=theta[i][j]+2*M_PI;
			q_solutions(i-1,j-1)=theta[i][j];
		}
		//ROS_INFO("8个方案:%2f,%2f,%2f,%2f,%2f,%2f,%2f,%2f",q_solutions(i-1,0),q_solutions(i-1,1),q_solutions(i-1,2),q_solutions(i-1,3)
		//,q_solutions(i-1,4),q_solutions(i-1,5));
	}
	
	Eigen::Matrix<double,6,1> q_filter;
	q_filter=ur10_solution_filter_test(current,q_solutions);
	//ROS_INFO("选择的解:%2f,%2f,%2f,%2f,%2f,%2f,%2f,%2f",q_filter[0],q_filter[1],q_filter[2],q_filter[3],q_filter[4],q_filter[5]);
	return q_filter;

};

