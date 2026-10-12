#pragma once
#pragma once
#ifdef DAULROBOTCALIBRATION_EXPORTS
#define DAULROBOTCALIBRATION_API __declspec(dllexport)
#else
#define DAULROBOTCALIBRATION_API __declspec(dllimport)
#endif

#pragma once
#include "vector"
#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "Eigen/Core"
#include "unsupported/Eigen/KroneckerProduct"
#include "iostream"
#include "ceres/ceres.h"

using namespace Eigen;
using namespace std;
#define M_PI 3.14159265358979323846   // pi

typedef Eigen::Matrix<double, 6, 1> Vector6d;


class DualRobotCalibration
{
public:
	DAULROBOTCALIBRATION_API DualRobotCalibration();
	DAULROBOTCALIBRATION_API ~DualRobotCalibration();
	//已知量
	std::vector<Vector6d> postureValuesMeasurementRobot;//存储测量机器人x,y,z,rz,ry,rx的值
	std::vector<Vector6d> postureValuesOperationRobot;//存储操作机器人x,y,z,rz,ry,rx的值
	std::vector<Eigen::Matrix4d> T_S_T;//存储操作机器人末端工具坐标系（标定板）到测量传感器的刚体变换矩阵
	std::vector<Eigen::Matrix4d> T_O1_E1;//存储测量机器人末端法兰到基坐标系的刚体变换矩阵
	std::vector<Eigen::Matrix4d> T_O2_E2;//存储操作机器人末端法兰到基坐标系的刚体变换矩阵	
	std::vector<Eigen::Matrix3d> R_S_T;//存储操作机器人末端工具坐标系（标定板）到测量传感器的旋转变换矩阵
	std::vector<Eigen::Matrix3d> R_O1_E1;//存储测量机器人末端法兰到基坐标系的旋转变换矩阵
	std::vector<Eigen::Matrix3d> R_O2_E2;//存储操作机器人末端法兰到基坐标系的旋转变换矩阵	
	std::vector<Eigen::Vector3d> t_S_T;//存储操作机器人末端工具坐标系（标定板）到测量传感器的平移向量
	std::vector<Eigen::Vector3d> t_O1_E1;//存储测量机器人末端法兰到基坐标系的平移向量
	std::vector<Eigen::Vector3d> t_O2_E2;//存储操作机器人末端法兰到基坐标系的平移向量
	//待标定量
	Eigen::Matrix4d T_E1_S;//存储测量机器人测量传感器坐标系到末端法兰的刚体变换矩阵
	Eigen::Matrix4d T_E2_T;//存储操作机器人末端工具坐标系（标定板）到末端法兰的刚体变换矩阵
	Eigen::Matrix4d T_O1_O2;//存储操作机器人末端法兰到测量机器人末端法兰的刚体变换矩阵	
	Eigen::Matrix3d R_E1_S;//存储测量机器人测量传感器坐标系到末端法兰的旋转变换矩阵
	Eigen::Matrix3d R_E2_T;//存储操作机器人末端工具坐标系（标定板）到末端法兰的旋转变换矩阵
	Eigen::Matrix3d R_O1_O2;//存储操作机器人末端法兰到测量机器人末端法兰的旋转变换矩阵	
	Eigen::Vector3d t_E1_S;//存储测量机器人测量传感器坐标系到末端法兰的平移向量
	Eigen::Vector3d t_E2_T;//存储操作机器人末端工具坐标系（标定板）到末端法兰的平移向量
	Eigen::Vector3d t_O1_O2;//存储操作机器人末端法兰到测量机器人末端法兰的平移向量

	//标定过程
	DAULROBOTCALIBRATION_API void calibrationProcess(std::vector<Vector6d> postureValuesMeasurementRobot, std::vector<Vector6d> postureValuesOperationRobot, std::vector<Eigen::Matrix4d> T_S_T_ALL);
	//粗标定过程
	DAULROBOTCALIBRATION_API void coarseCalibration();
	//精标定过程
	DAULROBOTCALIBRATION_API void fineCalibration();

	//标定过程用到的函数
	//欧拉角转旋转矩阵
	DAULROBOTCALIBRATION_API void R_XYZ(Eigen::Matrix3d& R, double alpha, double beta, double gama);
	//封闭解求旋转矩阵
	DAULROBOTCALIBRATION_API void CloseSloveR(Eigen::Matrix3d& R_E1_S, std::vector<Eigen::Matrix3d> R_S_T, std::vector<Eigen::Matrix3d> R_O1_E1, std::vector<Eigen::Matrix3d> R_O2_E2);
	//封闭解求旋转矩阵
	DAULROBOTCALIBRATION_API void CloseSlove(Eigen::MatrixXd& M_ABC, Eigen::Matrix3d R_S_T, Eigen::Matrix3d R_O1_E1, Eigen::Matrix3d R_O2_E2);
	//矩阵转向量
	DAULROBOTCALIBRATION_API void Matrix2Vector(Eigen::Matrix4d M, Vector6d& V);
};

