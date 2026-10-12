#pragma once
//OPENCV
#include <opencv2/opencv.hpp>

//C++
#include <iostream>
#include <fstream>
#include <numeric>
#include <io.h>

//Eigen
#include <Eigen/Core>
#include <Eigen/Dense>

class imgPro {
public:
	int GetOtsuThresh(const cv::Mat srcImg);
	void domainSmooth(cv::Mat srcImg, cv::Mat& dstImg);

	void ImgPreprocess(cv::Mat srcImg, cv::Mat& dstImg, int flag = 0);
	void ImgPreprocess2(cv::Mat srcImg, cv::Mat& dstImg, int flag = 0);

	void hampleFilt(std::vector<cv::Point2f> Pt, std::vector<cv::Point2f> dstPoints);

	void GrayCenterLineExtration(cv::Mat srcImg, std::vector<cv::Point2f>& dstPoints, int flag = 1);

	void GrayCenterLineExtrationCol(cv::Mat srcImg, std::vector<cv::Point2f>& dstPoints);

	void StegerLine(cv::Mat srcImg, std::vector<cv::Point2f>& dstPoints);
	void StegerLine2(cv::Mat srcImg, std::vector<cv::Point2f>& dstPoints);
};

int getImgNo(std::string imgPath);

void GetChessBoardBorder(cv::Mat img, cv::Size boardSize, cv::Vec4i& border);

void GetChessBoardBorder2(cv::Mat img, cv::Size boardSize, cv::Vec4i& border);

void imgPreProcess(cv::Mat img, int thre, cv::Mat& dstImg, std::vector<cv::Vec4i>& dstRanges);

void imgPreProcess_simple(cv::Mat img, int thre, cv::Mat& dstImg, std::vector<cv::Vec4i>& dstRanges);

void imgPreProcess_heng(cv::Mat img, int thre, cv::Mat& dstImg, std::vector<cv::Vec4i>& dstRanges);

bool refract_line(double n0, double n1, Eigen::Vector3d r0, Eigen::Vector3d N, Eigen::Vector3d& r1);

//Eigen::Vector3d GetPlaneInterectPt(Eigen::VectorXd vp, Eigen::Vector4d plane);

void FitPLaneBySVD(Eigen::MatrixXd pts, Eigen::VectorXd& coeff, std::vector<double>& errors);

// Calculate Hessian2D matrix
void Hessian2D(const cv::Mat& I, double sigma, cv::Mat& Dx, cv::Mat& Dy, cv::Mat& Dxx, cv::Mat& Dxy, cv::Mat& Dyy);

// Helper function to compute eigenvalues and eigenvectors
void eig2image(const cv::Mat& Dxx, const cv::Mat& Dxy, const cv::Mat& Dyy, cv::Mat& Lambda1, cv::Mat& Lambda2, cv::Mat& Ix, cv::Mat& Iy);

// Main function to find the laser center
std::vector<cv::Point2d> findLaserCenterSteger(const cv::Mat& inputImage, double bwThr, double sltThr, double stripeWidth = 7.0);

std::vector<cv::Point2d> findLaserCenterSteger_v2_heng(const cv::Mat& inputImage, double bwThr, double sltThr, double stripeWidth = 7.0, bool filter = false);

std::vector<cv::Point2d> findLaserCenterSteger_v2_shu(const cv::Mat& inputImage, double bwThr, double sltThr, double stripeWidth = 7.0, bool filter = false);

std::vector<cv::Point2d> findLaserCenterSteger_v5_heng(const cv::Mat& inputImage, double bwThr, double sltThr, double stripeWidth = 7.0, bool filter = false);

std::vector<cv::Point2d> findLaserCenterSteger_v5_shu(const cv::Mat& inputImage, double bwThr, double sltThr, double stripeWidth = 7.0, bool filter = false);

std::vector<cv::Point2d> findLaserCenterGrayCenter_heng(const cv::Mat& inputImage, double bwThr, double sumThr=100);

std::vector<cv::Point2d> findLaserCenterGrayCenter_shu(const cv::Mat& inputImage, double bwThr);

//std::vector<cv::Point2d> bruteLaserCenter(const cv::Mat& inputImage, double max_val_thres, const std::string& dir, int neighbor_size, double pixel_thres);

std::vector<cv::Point2d> findLaserCenterMultiThres_shu(const cv::Mat& inputImage, double max_val_thres, int neighbor_size, double pixel_thres);

std::vector<cv::Point2d> findLaserCenterMultiThres_heng(const cv::Mat& inputImage, double max_val_thres, int neighbor_size, double pixel_thres);

void GetImageRanges(const cv::Mat& bwImg, std::vector<cv::Vec4i>& dstRanges, double areaThr = 50, int borderexten = 10);

void GetImageRanges_v2(const cv::Mat& bwImg, std::vector<cv::Vec4i>& dstRanges, double areaThr, int borderexten);

std::pair<double, double> gauss_distribution(const cv::Mat& img_gray);