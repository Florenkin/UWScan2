#pragma once
//Eigen
#include <Eigen/Core>
#include <unsupported/Eigen/KroneckerProduct> 
#include <Eigen/Dense>

//OpenCV
#include<opencv2/opencv.hpp>

//STD
#include <iostream>

//Ceres
#include <ceres/ceres.h>

//�����������������d:\cpp_project\Qtscanner\QuadricSurfaceFitting.cpp d:\cpp_project\Qtscanner\QuadricSurfaceFitting2.cpp
//���룺surFPoints-�����ϵĵ㼯
//�����quaSurfCoeffs-��������ϵ��
using namespace Eigen;
using namespace std;
namespace Rui {
	void PtsCoorTrans(Eigen::MatrixXf surfPts, Eigen::MatrixXf& tranSurfPts, Eigen::Matrix4f& TransMat);

	void PtsCoorTrans2(Eigen::MatrixXf surfPts, Eigen::MatrixXf& tranSurfPts, Eigen::Matrix4f& TransMat);

	//���������
	void QuadricSurfFittingAll(Eigen::MatrixXf ptsMat, Eigen::VectorXf& quaSurfCoeffs, std::vector<double>& fitErrors);

	void QuadricSurfFitting(Eigen::MatrixXf surFPoints, Eigen::VectorXf& quaSurfCoeffs, std::vector<double>& fitErrors);

	void QuadricCoeffsTrans(Eigen::VectorXf quaSurfCoeffs, Eigen::Matrix4f TransMat, Eigen::VectorXf& quaSurfCoeffsTrans);

	void GetQuadricSurfCoeffs(std::vector<Eigen::Vector3f> surfPts, Eigen::VectorXf& quaSurfCoeffs);

	void GetQuadricSurfCoeffs2(std::vector<Eigen::Vector3f> surfPts, Eigen::VectorXf& quaSurfCoeffs, Eigen::VectorXf& TransVec);

	void GetQuadricSurfIntersectPt(Eigen::VectorXf quaSurfCoeffs, Eigen::VectorXf camRay, Eigen::Vector3f& intersectPt);

	//����������⽻��,Z��ΪԼ��
	void GetQuadricSurfIntersectPtNewForm(Eigen::VectorXf quadricSurfCoeffs, Eigen::VectorXf camRay, Eigen::Vector3f& intersectPt, double maxZ, double minZ);

	void GetQuadricSurfIntersectPt_localCal(Eigen::VectorXf quaSurfCoeffs, Eigen::VectorXf camRay, Eigen::Matrix4f TransMat, Eigen::Vector3f& intersectPt);

	//��ȡ�Ż���Ķ�������ϵ��
	float CostPtToQuadricSurface(Eigen::Vector3f pt, Eigen::VectorXf quadricSurfCoeffs);
	void GetOptimalQuadricSurfCoeffs(const std::vector<Eigen::Vector3f>& surfPts, Eigen::VectorXf& quaSurfCoeff, std::vector<double>& fitErrors, Eigen::Matrix4f& TransMatrix, double& maxZ, double& minZ);

	//�Ĵ��������run_newForm
	void QuarticSurfFittingAll_newForm(Eigen::MatrixXf ptsMat, Eigen::VectorXf& quarticSurfCoeffs, std::vector<double>& fitErrors, Eigen::Matrix4f& TransMatrix, double& maxZ, double& minZ);

	//�Ĵ��������run
	void QuarticSurfFittingAll(Eigen::MatrixXf ptsMat, Eigen::VectorXf& quarticSurfCoeffs, std::vector<double>& fitErrors);
	//�����Ĵ��������
	void QuarticSurfFitting(Eigen::MatrixXf surFPoints, Eigen::VectorXf& quarticSurfCoeffs, std::vector<double>& fitErrors);
	//�Ĵ�����ϵ���任
	void QuarticCoeffsTrans(Eigen::VectorXf quarticSurfCoeffs, Eigen::Matrix4f TransMat, Eigen::VectorXf& quarticSurfCoeffsTrans);
	//�Ĵ�������⽻��,Z��ΪԼ��
	void GetQuarticSurfIntersectPtNewForm(Eigen::VectorXf quarticSurfCoeffs, Eigen::VectorXf camRay, Eigen::Vector3f& intersectPt, double maxZ, double minZ);
	//�Ĵ�������⽻��
	void GetQuarticSurfIntersectPt(Eigen::VectorXf quarticSurfCoeffs, Eigen::VectorXf camRay, Eigen::Vector3f& intersectPt);
	//�Ĵ��������x
	void GetQuarticSurfXCoor(Eigen::VectorXf quarticSurfCoeffs, double x, double y, double z, double xslove);
	void GetQuarticSurfYCoor(Eigen::VectorXf quarticSurfCoeffs, double x, double y, double z, double yslove);

	struct CostPtToQuadricFunctor {
		CostPtToQuadricFunctor(Vector3f pt) :pt_(pt) {}

		bool operator()(const double* xCoe, double* residual)const
		{
			VectorXf coe = VectorXf::Zero(10);
			for (int i = 0; i < 10; i++) {
				coe(i) = (float)xCoe[i];
			}
			//coe = coe / coe.norm();
			residual[0] = (double)CostPtToQuadricSurface(pt_, coe);
			return true;
		}

		Vector3f pt_;
	};

	struct CostQuadricCoeffsFunctor {
		CostQuadricCoeffsFunctor() {}

		bool operator()(const double* xCoe, double* residual)const
		{
			VectorXf coe = VectorXf::Zero(10);
			for (int i = 0; i < 10; i++) {
				coe(i) = (float)xCoe[i];
			}
			//coe = coe / coe.norm();
			residual[0] = (double)(coe.norm() - 1e3);
			return true;
		}

		Vector3f pt_;
	};

}
