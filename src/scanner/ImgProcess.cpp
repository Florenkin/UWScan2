#include "scanner/ImgProcess.h"

using namespace std;
using namespace cv;
using namespace Eigen;

int imgPro::GetOtsuThresh(const cv::Mat srcImg)
{
	Mat img = srcImg.clone();
	if (srcImg.channels() == 3)
		cvtColor(img, img, COLOR_BGR2GRAY);

	vector<int> hist(256, 0);
	double sumGV = 0;
	for (int i = 0; i < img.rows; i++)
		for (int j = 0; j < img.cols; j++)
		{
			hist[(int)img.at<uchar>(i, j)]++;
			sumGV += (int)img.at<uchar>(i, j);
		}
	int imgSize = img.rows * img.cols;
	int bestThresh = 1;
	double var = 0;
	for (auto i = 1; i < 256; i++)
	{
		int cnt0 = 0;
		int cnt1 = 0;
		double u0 = 0;
		double u1 = 0;
		for (int j = 0; j < i; j++)
		{
			cnt0 += hist[j];
			u0 += hist[j] * j;
		}
		u1 = sumGV - u0;
		cnt1 = img.rows * img.cols - cnt0;
		u0 /= cnt0 + 0.0;
		u1 /= cnt1 + 0.0;
		double w0 = cnt0 / (imgSize + 0.0);//��������
		double w1 = cnt1 / (imgSize + 0.0);//ǰ������
		double g = w0 * w1 * pow((u1 - u0), 2);//����
		if (g > var)
		{
			var = g;
			bestThresh = i;
		}
	}
	return bestThresh;
}

void imgPro::domainSmooth(Mat srcImg, Mat& dstImg)
{
	Mat img = srcImg.clone();
	//namedWindow("img00", WINDOW_NORMAL);
	//imshow("img00", img);

	//����
	int sizeHalf = 3;
	dilate(img, img, getStructuringElement(MORPH_RECT, Size(sizeHalf, sizeHalf)));
	//namedWindow("img0", WINDOW_NORMAL);
	//imshow("img0", img);

	//��ֵ�˲�
	blur(img, img, Size(2 * sizeHalf + 1, 2 * sizeHalf + 1));
	//namedWindow("img1", WINDOW_NORMAL);
	//imshow("img1", img);

	//��ֵ��
	threshold(img, img, 255 * 0.5, 255, THRESH_BINARY);
	//namedWindow("img2", WINDOW_NORMAL);
	//imshow("img2", img);

	//��ʴ
	erode(img, img, getStructuringElement(MORPH_RECT, Size(sizeHalf, sizeHalf)));
	dstImg = img.clone();
	//namedWindow("dstImg", WINDOW_NORMAL);
	//imshow("dstImg", img);
	//waitKey(0);
}

void imgPro::ImgPreprocess(Mat srcImg, Mat& dstImg, int flag) {
	Mat img = srcImg.clone();
	if (srcImg.channels() > 1)
		cvtColor(srcImg, img, COLOR_BGR2GRAY);
	Mat bwImg;
	threshold(img, bwImg, GetOtsuThresh(img), 255, THRESH_BINARY);//50,GetOtsuThresh(img)

	Mat Temp = cv::Mat::zeros(bwImg.rows, bwImg.cols, bwImg.type());
	if (flag == 0) {
		//ȥ����������
		std::vector<std::vector<cv::Point>> contours;
		std::vector<std::vector<cv::Point>>::iterator it;
		std::vector<cv::Vec4i>hierarchy;
		cv::findContours(bwImg, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
		vector<Point2i> finalContour;
		vector<vector<Point2i>> finalContourSet;
		double areaThr = 100;//50
		for (int i = 0; i < contours.size(); i++)
		{
			double area = contourArea(contours[i]);
			if (area > areaThr)//���С��areaThr�Ͳ�Ҫ
				finalContourSet.push_back(contours[i]);
		}
		for (int i = 0; i < finalContourSet.size(); i++)
			cv::drawContours(Temp, finalContourSet, i, cv::Scalar(255), cv::FILLED);
	}
	else if (flag == 1) {
		//ȡ��ͼ������Ĳ���
		std::vector<std::vector<cv::Point>> contours;
		std::vector<std::vector<cv::Point>>::iterator it;
		std::vector<cv::Vec4i>hierarchy;
		cv::findContours(bwImg, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
		vector<Point2i> finalContour;
		vector<vector<Point2i>> finalContourSet;
		double maxlength = 0;
		for (int i = 0; i < contours.size(); i++)
		{
			double length = arcLength(contours[i], 1);
			if (length > maxlength)
			{
				finalContour = contours[i];
				maxlength = length;
			}
		}
		if (maxlength > 0) {
			finalContourSet.push_back(finalContour);
			cv::drawContours(Temp, finalContourSet, 0, cv::Scalar(255), cv::FILLED);
		}
	}
	else
		Temp = bwImg.clone();
	//namedWindow("Temp", WINDOW_NORMAL);
	//imshow("Temp", Temp);
	//waitKey(0);
	morphologyEx(Temp, Temp, MORPH_OPEN, getStructuringElement(MORPH_RECT, Size(3, 3)));
	domainSmooth(Temp, Temp);//����ƽ��
	dstImg = img.mul(Temp / 255);
	GaussianBlur(dstImg, dstImg, Size(3, 3), 2, 2);
	//namedWindow("dstImg", WINDOW_NORMAL);
	//imshow("dstImg", dstImg);
	//waitKey(0);
}
void imgPro::ImgPreprocess2(Mat srcImg, Mat& dstImg, int flag) {
	Mat img = srcImg.clone();
	if (srcImg.channels() > 1)
		cvtColor(srcImg, img, COLOR_BGR2GRAY);
	Mat bwImg;
	threshold(img, bwImg, 100, 255, THRESH_BINARY);//50,100,GetOtsuThresh(img)

	Mat Temp = cv::Mat::zeros(bwImg.rows, bwImg.cols, bwImg.type());
	if (flag == 0) {
		//ȥ����������
		std::vector<std::vector<cv::Point>> contours;
		std::vector<std::vector<cv::Point>>::iterator it;
		std::vector<cv::Vec4i>hierarchy;
		cv::findContours(bwImg, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
		vector<Point2i> finalContour;
		vector<vector<Point2i>> finalContourSet;
		double areaThr = 50;
		for (int i = 0; i < contours.size(); i++)
		{
			double area = contourArea(contours[i]);
			if (area > areaThr)//���С��areaThr�Ͳ�Ҫ
				finalContourSet.push_back(contours[i]);
		}
		for (int i = 0; i < finalContourSet.size(); i++)
			cv::drawContours(Temp, finalContourSet, i, cv::Scalar(255), cv::FILLED);
	}
	else if (flag == 1) {
		//ȡ��ͼ������Ĳ���
		std::vector<std::vector<cv::Point>> contours;
		std::vector<std::vector<cv::Point>>::iterator it;
		std::vector<cv::Vec4i>hierarchy;
		cv::findContours(bwImg, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
		vector<Point2i> finalContour;
		vector<vector<Point2i>> finalContourSet;
		double maxlength = 0;
		for (int i = 0; i < contours.size(); i++)
		{
			double length = arcLength(contours[i], 1);
			if (length > maxlength)
			{
				finalContour = contours[i];
				maxlength = length;
			}
		}
		if (maxlength > 0) {
			finalContourSet.push_back(finalContour);
			cv::drawContours(Temp, finalContourSet, 0, cv::Scalar(255), cv::FILLED);
		}
	}
	else {
		Temp = bwImg.clone();
	}
	//namedWindow("Temp", WINDOW_NORMAL);
	//imshow("Temp", Temp);
	//waitKey(0);
	morphologyEx(Temp, Temp, MORPH_OPEN, getStructuringElement(MORPH_RECT, Size(3, 3)));
	domainSmooth(Temp, Temp);//����ƽ��
	dstImg = img.mul(Temp / 255);
	GaussianBlur(dstImg, dstImg, Size(3, 3), 2, 2);
}

void imgPro::hampleFilt(vector<Point2f> Pt, vector<Point2f> dstPoints) {
	vector<Point2f> hamPoints = Pt;
	int halfWidth = 2;
	vector<Point2f> hamPoints2 = hamPoints;
	for (int i = halfWidth; i < hamPoints.size() - halfWidth; i++)
	{
		//��ȡ����
		vector<Point2f> temp;
		temp.assign(hamPoints.begin() + i - halfWidth, hamPoints.begin() + i + halfWidth + 1);
		//ԭ�����λ��
		double orPos = temp[halfWidth].x;
		//����õ���λ��
		sort(temp.begin(), temp.end(), [&](Point2d a, Point2d b) {return a.x < b.x; });
		double median = temp[halfWidth].x;
		//�õ�������λ��
		for (int j = 0; j < temp.size(); j++)
		{
			temp[j].x = fabs(temp[j].x - median);
		}
		sort(temp.begin(), temp.end(), [&](Point2d a, Point2d b) {return a.x < b.x; });
		double mad = 1.4826 * (temp[halfWidth].x - temp[0].x);//1.4826
		if (fabs(orPos - median) > 2 * mad)
		{
			hamPoints2[i].x = median;
			//cout << hamPoints[i] << endl;
		}
	}
	vector<Point2f> hamPoints3;
	hamPoints3.assign(hamPoints2.begin() + halfWidth, hamPoints2.end() - halfWidth);
	dstPoints = hamPoints3;
}

void imgPro::GrayCenterLineExtration(cv::Mat srcImg, vector<Point2f>& dstPoints, int flag)
{
	Mat img = srcImg.clone();
	if (img.channels() > 1)
		cvtColor(img, img, COLOR_BGR2GRAY);
	vector<Point2f> linePoints;
	for (int i = 0; i < img.rows; i++)
	{
		double sum_gv = 0;
		double sum_gv_pos = 0;
		double thresh = 0;//30,10
		for (int j = 0; j < img.cols; j++)
		{
			double tempV = img.at<uchar>(i, j);
			if (tempV > thresh)
			{
				sum_gv += tempV;
				sum_gv_pos += tempV * j;
			}
			if (flag == 1)//�������µ�����
			{
				if (i >= 2 && i < img.rows - 2) {
					sum_gv += img.at<uchar>(i - 2, j) + img.at<uchar>(i - 1, j) + img.at<uchar>(i + 1, j) + img.at<uchar>(i + 2, j);
					sum_gv_pos += (img.at<uchar>(i - 2, j) + img.at<uchar>(i - 1, j) + img.at<uchar>(i + 1, j) + img.at<uchar>(i + 2, j)) * j;
				}
			}
		}
		if (sum_gv < 200)
			continue;
		double x = sum_gv_pos / sum_gv;
		linePoints.push_back(Point2f(x, i));
	}
	dstPoints = linePoints;
	////����������
	//Mat drawImg;
	//cvtColor(img, drawImg, COLOR_GRAY2BGR);
	//for (int i = 0; i < linePoints.size(); i++) {
	//	cv::circle(drawImg, Point2f(linePoints[i].x, linePoints[i].y), 0.5, Scalar(0, 0, 255));
	//}
	//cv::imwrite(".\\�м���\\������.png", drawImg);
	//namedWindow("centerLine", WINDOW_NORMAL);
	//imshow("centerLine", drawImg);
	//waitKey(0);


	/* ���λҶ����ķ� */
	if (flag == 2) {
		//���������ķ�ʸ
		vector<Point2f> lineNorm;
		for (int i = 0; i < linePoints.size(); i++)
		{
			if (i == 0)
			{
				Point2f P_0 = linePoints[0];
				Point2f P_1 = linePoints[1];
				Point2f subP = P_1 - P_0;
				subP /= sqrt(pow(subP.x, 2) + pow(subP.y, 2));
				if (subP.x - subP.y < 0)
					subP *= -1.0;
				lineNorm.push_back(Point2f(-subP.y, subP.x));
			}
			else if (i == linePoints.size() - 1)
			{
				Point2f P_isub1 = linePoints[linePoints.size() - 2];
				Point2f P_i = linePoints[linePoints.size() - 1];
				Point2f subP = P_i - P_isub1;
				subP /= sqrt(pow(subP.x, 2) + pow(subP.y, 2));
				if (subP.x - subP.y < 0)
					subP *= -1.0;
				lineNorm.push_back(Point2f(-subP.y, subP.x));
			}
			else
			{
				Point2f P_isub1 = linePoints[i - 1];
				Point2f P_i = linePoints[i];
				Point2f P_iplus1 = linePoints[i + 1];
				Point2f sub1 = P_iplus1 - P_i;
				Point2f sub2 = P_i - P_isub1;
				sub1 /= sqrt(pow(sub1.x, 2) + pow(sub1.y, 2));
				sub2 /= sqrt(pow(sub2.x, 2) + pow(sub2.y, 2));
				Point2f tempNorm = sub1 - sub2;
				//cout << i << " tempNorm= " << tempNorm << endl;
				if (norm(tempNorm) < 1e-5)
					tempNorm = { 1.0, 0.0 };
				tempNorm /= sqrt(pow(tempNorm.x, 2) + pow(tempNorm.y, 2));
				if (tempNorm.x + tempNorm.y < 0)
					tempNorm *= -1.0;
				lineNorm.push_back(tempNorm);
			}
			//cout << "lineNorm[" << i << "]: " << lineNorm[i].x << "," << lineNorm[i].y << endl;
		}
		vector<Point2f> newLinePoints;
		for (int i = 0; i < linePoints.size(); i++)
		{
			//if (isnan(lineNorm[i].x))
			//{
			//	newLinePoints.push_back(linePoints[i]);
			//	continue;
			//}
			double step = 0.1;//0.5
			int num = 400;
			int leftNum = 1;
			double sum_gv = 0;
			double sum_gv_posX = 0;
			double sum_gv_posY = 0;
			double thresh = 0;//10
			for (int j = 0; j < num; j++)
			{
				int tempX = linePoints[i].x + lineNorm[i].x * step * (j - 200);
				int tempY = linePoints[i].y + lineNorm[i].y * step * (j - 200);
				if (tempX < 0 || tempY < 0)
					continue;
				if (tempX > img.cols - 1 || tempY > img.rows - 1)
					break;
				double tempV = img.at<uchar>(tempY, tempX);
				if (tempV > thresh)
				{
					sum_gv += tempV;
					sum_gv_posX += tempV * tempX;
					sum_gv_posY += tempV * tempY;
				}
			}
			if (sum_gv < 200)
				continue;
			double dstX = sum_gv_posX / sum_gv;
			double dstY = sum_gv_posY / sum_gv;
			newLinePoints.push_back(Point2f(dstX, dstY));
		}
		dstPoints = newLinePoints;
		////����������
		//Mat drawImg2;
		//cvtColor(img, drawImg2, COLOR_GRAY2BGR);
		//dstPoints = linePoints;
		//for (int i = 0; i < dstPoints.size(); i++) {
		//	cv::circle(drawImg2, Point2f(dstPoints[i].x, dstPoints[i].y), 0.5, Scalar(0, 0, 255));
		//}
		//namedWindow("drawImg2", cv::WINDOW_NORMAL);
		//imshow("drawImg2", drawImg2);
		//waitKey(0);
	}
}

void imgPro::GrayCenterLineExtrationCol(cv::Mat srcImg, vector<Point2f>& dstPoints)
{
	Mat img = srcImg.clone();
	if (img.channels() > 1)
		cvtColor(img, img, COLOR_BGR2GRAY);
	vector<Point2f> linePoints;
	Mat drawImg;
	cvtColor(img, drawImg, COLOR_GRAY2BGR);
	for (int j = 0; j < img.cols; j++)
	{
		double sum_gv = 0;
		double sum_gv_pos = 0;
		double thresh = 0;//30,10
		for (int i = 0; i < img.rows; i++)
		{
			double tempV = img.at<uchar>(i, j);
			if (tempV > thresh)
			{
				sum_gv += tempV;
				sum_gv_pos += tempV * i;
			}
			//�������ҵ�����
			if (j >= 2 && j < img.cols - 2) {
				sum_gv += img.at<uchar>(i, j - 2) + img.at<uchar>(i, j - 1) + img.at<uchar>(i, j + 1) + img.at<uchar>(i, j + 2);
				sum_gv_pos += (img.at<uchar>(i, j - 2) + img.at<uchar>(i, j - 1) + img.at<uchar>(i, j + 1) + img.at<uchar>(i, j + 2)) * i;
			}
		}
		if (sum_gv < 200)
			continue;
		double y = sum_gv_pos / sum_gv;
		linePoints.push_back(Point2f(j, y));

	}
	if (linePoints.size() > 20) //ȥ����β�˵�
	{
		linePoints.erase(linePoints.begin(), linePoints.begin() + 10);
		linePoints.erase(linePoints.end() - 10, linePoints.end());
	}
	//����������
	for (int i = 0; i < linePoints.size(); i++) {
		cv::circle(drawImg, Point2f(linePoints[i].x, linePoints[i].y), 0.5, Scalar(0, 0, 255));
	}
	namedWindow("centerLine", WINDOW_NORMAL);
	imshow("centerLine", drawImg);
	waitKey(0);
	//imwrite(".\\�м���\\����΢��������.png", drawImg);
	dstPoints = linePoints;
}

void imgPro::StegerLine(Mat srcImg, vector<Point2f>& dstPoints)
{
	Mat img = srcImg.clone();
	if (img.channels() == 3)
		cvtColor(img, img, COLOR_BGR2GRAY);
	Mat bwImg;
	threshold(img, bwImg, 0, 255, THRESH_OTSU);
	Mat img1;
	bwImg.convertTo(img1, CV_32F);
	img1 /= 255;

	vector<Point2f> Pt;
	vector<Point2f> Pt1;


	/* ����Hessian����_old */
	//double sigma = 15 / sqrt(3);
	//int sizeH = round(3 * sigma) * 2 + 1;
	//Mat X = Mat::zeros(sizeH, sizeH, CV_64F);
	//Mat Y = Mat::zeros(sizeH, sizeH, CV_64F);
	//Mat DGaussx = Mat::zeros(sizeH, sizeH, CV_64F);
	//Mat DGaussy = Mat::zeros(sizeH, sizeH, CV_64F);
	//Mat DGaussxx = Mat::zeros(sizeH, sizeH, CV_64F);
	//Mat DGaussxy = Mat::zeros(sizeH, sizeH, CV_64F);
	//Mat DGaussyy = Mat::zeros(sizeH, sizeH, CV_64F);
	//for (int i = 0; i < sizeH; i++)
	//{
	//	for (int j = 0; j < sizeH; j++)
	//	{
	//		//X.at<double>(i, j) = j - round(3 * sigma);
	//		//Y.at<double>(i, j) = i - round(3 * sigma);
	//		X.at<double>(i, j) = i - round(3 * sigma);
	//		Y.at<double>(i, j) = i - round(3 * sigma);
	//		double x = X.at<double>(i, j);
	//		double y = Y.at<double>(i, j);
	//		DGaussx.at<double>(i, j) = 1 / (2 * CV_PI * pow(sigma, 4)) * (-x) * exp(-(x * x + y * y) / (2 * sigma * sigma));
	//		DGaussy.at<double>(i, j) = 1 / (2 * CV_PI * pow(sigma, 4)) * (-y) * exp(-(x * x + y * y) / (2 * sigma * sigma));
	//		DGaussxx.at<double>(i, j) = 1 / (2 * CV_PI * pow(sigma, 4)) * (x * x/ (sigma* sigma) - 1) * exp(-(x * x + y * y) / (2 * sigma * sigma));
	//		DGaussxy.at<double>(i, j) = 1 / (2 * CV_PI * pow(sigma, 6)) * (x * y) * exp(-(x * x + y * y) / (2 * sigma * sigma));
	//		DGaussyy.at<double>(j, i) = 1 / (2 * CV_PI * pow(sigma, 4)) * (x * x / (sigma * sigma) - 1) * exp(-(x * x + y * y) / (2 * sigma * sigma));
	//		//cout << i << " " << j << " :  " << DGaussxy.at<double>(i, j) << endl;
	//	}
	//}
	//Mat Dx;
	//Mat Dy;
	//Mat Dxx;
	//Mat Dxy;
	//Mat Dyy;
	//filter2D(img1, Dx, CV_64FC1, DGaussx);
	//filter2D(img1, Dy, CV_64FC1, DGaussy);
	//filter2D(img1, Dxx, CV_64FC1, DGaussxx);
	//filter2D(img1, Dxy, CV_64FC1, DGaussxy);
	//filter2D(img1, Dyy, CV_64FC1, DGaussyy);

	/* ����Hessian���� */
	//���и�˹ƽ��
	double sigma = 7 / sqrt(3);
	int sizeH = round(3 * sigma) * 2 + 1;
	Mat imgGauss;
	GaussianBlur(img1, imgGauss, Size(sizeH, sizeH), sigma, sigma);
	//����Dx��Dy��Dxx��Dxy��Dyy
	Mat DxKnl = (Mat_<float>(1, 2) << -1, 1);
	Mat DyKnl = (Mat_<float>(2, 1) << -1, 1);
	Mat DxxKnl = (Mat_<float>(1, 3) << 1, -2, 1);
	Mat DyyKnl = (Mat_<float>(3, 1) << 1, -2, 1);
	Mat DxyKnl = (Mat_<float>(2, 2) << 1, -1, -1, 1);
	cv::flip(DxKnl, DxKnl, -1);
	cv::flip(DyKnl, DyKnl, -1);
	cv::flip(DxxKnl, DxxKnl, -1);
	cv::flip(DyyKnl, DyyKnl, -1);
	cv::flip(DxyKnl, DxyKnl, -1);
	Mat Dx;
	Mat Dy;
	Mat Dxx;
	Mat Dxy;
	Mat Dyy;
	filter2D(imgGauss, Dx, CV_32F, DxKnl);
	filter2D(imgGauss, Dy, CV_32F, DyKnl);
	filter2D(imgGauss, Dxx, CV_32F, DxxKnl);
	filter2D(imgGauss, Dxy, CV_32F, DxyKnl);
	filter2D(imgGauss, Dyy, CV_32F, DyyKnl);

	Mat saveDx = Dyy.clone();
	cv::normalize(saveDx, saveDx, 255.0, 0.0, NORM_INF);
	saveDx.convertTo(saveDx, CV_8U);
	//imwrite("Dx.png", saveDx);
	/* ����Hessian����ֵ */
	for (int i = 0; i < img.rows; i++)
	{
		for (int j = 0; j < img.cols; j++)
		{
			if (bwImg.at<uchar>(i, j) < 255)
			{
				continue;
			}
			//cout << i << " " << j << "  Dx: " << Dx.at<double>(i, j) << endl;

			Mat hessian(2, 2, CV_32F);
			hessian.at<float>(0, 0) = Dxx.at<float>(i, j);
			hessian.at<float>(0, 1) = Dxy.at<float>(i, j);
			hessian.at<float>(1, 0) = Dxy.at<float>(i, j);
			hessian.at<float>(1, 1) = Dyy.at<float>(i, j);

			Mat eValue;
			Mat eVectors;
			eigen(hessian, eValue, eVectors);

			double nx, ny;
			double fmaxD = 0;
			if (abs(eValue.at<float>(0, 0)) >= abs(eValue.at<float>(1, 0)))  //������ֵ���ʱ��Ӧ����������
			{
				nx = eVectors.at<float>(0, 0);
				ny = eVectors.at<float>(0, 1);
				fmaxD = eValue.at<float>(0, 0);
			}
			else
			{
				nx = eVectors.at<float>(1, 0);
				ny = eVectors.at<float>(1, 1);
				fmaxD = eValue.at<float>(1, 0);
			}
			//cout << "hessian: " << endl << hessian << endl;

			double t = -(nx * Dx.at<float>(i, j) + ny * Dy.at<float>(i, j)) /
				(nx * nx * Dxx.at<float>(i, j) + 2 * nx * ny * Dxy.at<float>(i, j) + ny * ny * Dyy.at<float>(i, j));
			//cout << i << " " << j << "  nx,ny: " << t * nx << " " << t * ny << endl;

			double thr = 0.5;
			if (abs(t * nx) <= thr && abs(t * ny) <= thr && bwImg.at<uchar>(i, j) == 255)
			{
				//Pt.push_back(Point2d(i + t * nx, j + t * ny));
				Pt.push_back(Point2d(j + t * nx, i + t * ny));
				Pt1.push_back(Point2d(j, i));
				//cout << t << "  " << nx << "  " << ny << "  " << t * nx << "  " << t * ny << endl;
			}
		}
	}
	////������ȡ���
	//Mat drawImg;
	//cvtColor(img, drawImg, COLOR_GRAY2BGR);
	//sort(Pt.begin(), Pt.end(), [&](Point2f a, Point2f b) {return a.y < b.y; });
	//for (int i = 0; i < Pt.size(); i++)
	//{
	//	circle(drawImg, Pt[i], 0.5, Scalar(0, 0, 255));
	//	//cout << Pt[i].x << " " << Pt[i].y << endl;
	//}
	//namedWindow("drawImg", WINDOW_NORMAL);
	//imshow("drawImg", drawImg);
	//waitKey(0);
	dstPoints = Pt;
	//if (dstPoints.size() > 20) {
	//	dstPoints.erase(dstPoints.begin(), dstPoints.begin() + 10);
	//	dstPoints.erase(dstPoints.end() - 10, dstPoints.end());
	//}

}

void imgPro::StegerLine2(Mat srcImg, vector<Point2f>& dstPoints)
{
	Mat img = srcImg.clone();
	if (img.channels() == 3)
		cvtColor(img, img, COLOR_BGR2GRAY);
	Mat bwImg;
	threshold(img, bwImg, 0, 255, THRESH_OTSU);
	Mat img1;
	bwImg.convertTo(img1, CV_64F);
	img1 /= 255;

	vector<Point2f> Pt;
	vector<Point2f> Pt1;


	/* ����Hessian����_old */
	//double sigma = 15 / sqrt(3);
	//int sizeH = round(3 * sigma) * 2 + 1;
	//Mat X = Mat::zeros(sizeH, sizeH, CV_64F);
	//Mat Y = Mat::zeros(sizeH, sizeH, CV_64F);
	//Mat DGaussx = Mat::zeros(sizeH, sizeH, CV_64F);
	//Mat DGaussy = Mat::zeros(sizeH, sizeH, CV_64F);
	//Mat DGaussxx = Mat::zeros(sizeH, sizeH, CV_64F);
	//Mat DGaussxy = Mat::zeros(sizeH, sizeH, CV_64F);
	//Mat DGaussyy = Mat::zeros(sizeH, sizeH, CV_64F);
	//for (int i = 0; i < sizeH; i++)
	//{
	//	for (int j = 0; j < sizeH; j++)
	//	{
	//		//X.at<double>(i, j) = j - round(3 * sigma);
	//		//Y.at<double>(i, j) = i - round(3 * sigma);
	//		X.at<double>(i, j) = i - round(3 * sigma);
	//		Y.at<double>(i, j) = i - round(3 * sigma);
	//		double x = X.at<double>(i, j);
	//		double y = Y.at<double>(i, j);
	//		DGaussx.at<double>(i, j) = 1 / (2 * CV_PI * pow(sigma, 4)) * (-x) * exp(-(x * x + y * y) / (2 * sigma * sigma));
	//		DGaussy.at<double>(i, j) = 1 / (2 * CV_PI * pow(sigma, 4)) * (-y) * exp(-(x * x + y * y) / (2 * sigma * sigma));
	//		DGaussxx.at<double>(i, j) = 1 / (2 * CV_PI * pow(sigma, 4)) * (x * x/ (sigma* sigma) - 1) * exp(-(x * x + y * y) / (2 * sigma * sigma));
	//		DGaussxy.at<double>(i, j) = 1 / (2 * CV_PI * pow(sigma, 6)) * (x * y) * exp(-(x * x + y * y) / (2 * sigma * sigma));
	//		DGaussyy.at<double>(j, i) = 1 / (2 * CV_PI * pow(sigma, 4)) * (x * x / (sigma * sigma) - 1) * exp(-(x * x + y * y) / (2 * sigma * sigma));
	//		//cout << i << " " << j << " :  " << DGaussxy.at<double>(i, j) << endl;
	//	}
	//}
	//Mat Dx;
	//Mat Dy;
	//Mat Dxx;
	//Mat Dxy;
	//Mat Dyy;
	//filter2D(img1, Dx, CV_64FC1, DGaussx);
	//filter2D(img1, Dy, CV_64FC1, DGaussy);
	//filter2D(img1, Dxx, CV_64FC1, DGaussxx);
	//filter2D(img1, Dxy, CV_64FC1, DGaussxy);
	//filter2D(img1, Dyy, CV_64FC1, DGaussyy);

	/* ����Hessian���� */
	//���и�˹ƽ��
	double sigma = 15 / sqrt(3);
	int sizeH = round(3 * sigma) * 2 + 1;
	Mat imgGauss;
	GaussianBlur(img1, imgGauss, Size(sizeH, sizeH), sigma, sigma);
	imgGauss.convertTo(imgGauss, CV_64F);
	Mat Dxx, Dyy, Dxy;
	Sobel(imgGauss, Dxx, CV_64F, 2, 0, sizeH);
	Sobel(imgGauss, Dyy, CV_64F, 0, 2, sizeH);
	Sobel(imgGauss, Dxy, CV_64F, 1, 1, sizeH);

	// Compute eigenvalues of Hessian matrix for each pixel
	Mat src = srcImg.clone();
	Mat lambda1(src.size(), CV_64F), lambda2(src.size(), CV_64F), theta(src.size(), CV_64F);
	for (int y = 0; y < src.rows; ++y) {
		for (int x = 0; x < src.cols; ++x) {
			// Hessian matrix for pixel (x, y)
			double Ixx = Dxx.at<double>(y, x);
			double Iyy = Dyy.at<double>(y, x);
			double Ixy = Dxy.at<double>(y, x);

			// Compute the eigenvalues of the Hessian matrix
			double trace = Ixx + Iyy;
			double det = Ixx * Iyy - Ixy * Ixy;
			double temp = sqrt(trace * trace - 4 * det);

			// Eigenvalues (sorted by magnitude)
			lambda1.at<double>(y, x) = 0.5 * (trace + temp);
			lambda2.at<double>(y, x) = 0.5 * (trace - temp);

			// Compute the orientation (angle) of the eigenvector associated with lambda2
			theta.at<double>(y, x) = 0.5 * atan2(2 * Ixy, Ixx - Iyy);
		}
	}

	// Initialize centerLine output image
	Mat centerLine = Mat::zeros(src.size(), CV_8UC1);

	// Detect centerline based on the eigenvalue condition (Steger's method)
	for (int y = 1; y < src.rows - 1; ++y) {
		for (int x = 1; x < src.cols - 1; ++x) {
			double l1 = lambda1.at<double>(y, x);
			double l2 = lambda2.at<double>(y, x);

			// Steger criterion: select pixels where one eigenvalue is close to zero and the other is large
			if (fabs(l2) > fabs(l1) && l1 < 0 && l2 > 0) {
				centerLine.at<uchar>(y, x) = 255;
			}
		}
	}
	// Display results
	namedWindow("Original Image", WINDOW_NORMAL);
	namedWindow("Center Line", WINDOW_NORMAL);
	imshow("Original Image", src);
	imshow("Center Line", centerLine);
	waitKey(0);


	////������ȡ���
	//Mat drawImg;
	//cvtColor(img, drawImg, COLOR_GRAY2BGR);
	//sort(Pt.begin(), Pt.end(), [&](Point2f a, Point2f b) {return a.y < b.y; });
	//for (int i = 0; i < Pt.size(); i++)
	//{
	//	circle(drawImg, Pt[i], 0.5, Scalar(0, 0, 255));
	//	//cout << Pt[i].x << " " << Pt[i].y << endl;
	//}
	//namedWindow("drawImg", WINDOW_NORMAL);
	//imshow("drawImg", drawImg);
	//waitKey(0);
	//dstPoints = Pt;
	//if (dstPoints.size() > 20) {
	//	dstPoints.erase(dstPoints.begin(), dstPoints.begin() + 10);
	//	dstPoints.erase(dstPoints.end() - 10, dstPoints.end());
	//}

}

int getImgNo(std::string imgPath) {
	int pos = imgPath.find_last_of('\\') + 1;
	std::string num = imgPath.substr(pos, imgPath.length() - pos);
	int pos1 = num.find_last_of('.') + 1;
	std::string num1 = num.substr(0, pos1 - 1);
	return stoi(num1);
}

void GetChessBoardBorder(Mat img, Size boardSize, Vec4i& border) {
	Size picSize = img.size();
	vector<Point2f> imgCorners;
	findChessboardCorners(img, boardSize, imgCorners);
	int x, y, width, height;
	int extendX = 50;
	int extendY = 200;
	vector<Point2f> imgSortedCorners = imgCorners;
	/* x�������ұ߽���չextendX */
	std::sort(imgSortedCorners.begin(), imgSortedCorners.begin() + imgSortedCorners.size(), [&](cv::Point2f a, cv::Point2f b)
		{ return a.x < b.x; });
	if (imgSortedCorners[0].x - extendX >= 0)
		x = imgSortedCorners[0].x - extendX;
	else
		x = 0;
	if (imgSortedCorners[imgSortedCorners.size() - 1].x + extendX > picSize.width - 1)
		width = picSize.width - 1 - x;
	else
		width = imgSortedCorners[imgSortedCorners.size() - 1].x + extendX - x;
	//x = imgSortedCorners[0].x;
	//width = imgSortedCorners[imgSortedCorners.size() - 1].x - x;
	/* y�������ұ߽���չextendY */
	std::sort(imgSortedCorners.begin(), imgSortedCorners.begin() + imgSortedCorners.size(), [&](cv::Point2f a, cv::Point2f b)
		{ return a.y < b.y; });
	if (imgSortedCorners[0].y - extendY > 0)
		y = imgSortedCorners[0].y - extendY;
	else
		y = 0;
	if (imgSortedCorners[imgSortedCorners.size() - 1].y + extendY > picSize.height - 1)
		height = picSize.height - 1 - y;
	else
		height = imgSortedCorners[imgSortedCorners.size() - 1].y - y;
	border = Vec4i(x, y, width, height);
	////�����ǵ�
	//Mat drawCorners;
	//cvtColor(img, drawCorners, COLOR_GRAY2BGR);
	//drawChessboardCorners(drawCorners, boardSize, imgCorners, true); // ������ͼƬ�б�ǽǵ�
	//namedWindow("drawCorners", WINDOW_NORMAL);
	//imshow("drawCorners", drawCorners);
	////�����߽�
	//Mat drawBorder;
	//cvtColor(img, drawBorder, COLOR_GRAY2BGR);
	//rectangle(drawBorder, Point(x, y), Point(x + width, y + height), Scalar(0, 0, 255), 2);
	//namedWindow("drawBorder", WINDOW_NORMAL);
	//imshow("drawBorder", drawBorder);
	//waitKey(0);
}

void GetChessBoardBorder2(Mat img, Size boardSize, Vec4i& border) {
	Size picSize = img.size();
	vector<Point2f> imgCorners;
	findChessboardCorners(img, boardSize, imgCorners);
	int x, y, width, height;
	vector<Point2f> imgSortedCorners = imgCorners;
	//������ӵ�ƽ������
	vector<double> xWidth;
	vector<double> yWidth;
	for (int i = 0; i < boardSize.height; i++)
	{
		//vector<Point2f> temp;
		//temp.assign(imgSortedCorners.begin() + i * boardSize.width, imgSortedCorners.begin() + (i + 1) * boardSize.width);
		//sort(temp.begin(), temp.end(), [&](Point2f a, Point2f b) {return a.x < b.x; });
		//double width = temp[temp.size() - 1].x - temp[0].x;
		//xWidth.push_back(width);
		for (int j = 0; j < boardSize.width - 1; j++)
		{
			double width = abs(imgSortedCorners[i * boardSize.width + j + 1].x - imgSortedCorners[i * boardSize.width + j].x);
			xWidth.push_back(width);
		}
	}
	double xWidthMean = std::accumulate(xWidth.begin(), xWidth.end(), 0.0) / xWidth.size();
	int extendX = xWidthMean;
	for (int i = 0; i < boardSize.width; i++)
	{
		;
		for (int j = 0; j < boardSize.height - 1; j++)
		{
			double width = abs(imgSortedCorners[(j + 1) * boardSize.width + i].y - imgSortedCorners[j * boardSize.width + i].y);
			yWidth.push_back(width);
		}
	}
	double yWidthMean = std::accumulate(yWidth.begin(), yWidth.end(), 0.0) / yWidth.size();
	int extendY = yWidthMean;
	/* x�������ұ߽���չextendX */
	std::sort(imgSortedCorners.begin(), imgSortedCorners.begin() + imgSortedCorners.size(), [&](cv::Point2f a, cv::Point2f b)
		{ return a.x < b.x; });
	if (imgSortedCorners[0].x - extendX >= 0)
		x = imgSortedCorners[0].x - extendX;
	else
		x = 0;
	if (imgSortedCorners[imgSortedCorners.size() - 1].x + extendX > picSize.width - 1)
		width = picSize.width - 1 - x;
	else
		width = imgSortedCorners[imgSortedCorners.size() - 1].x + extendX - x;
	/* y�������ұ߽���չextendY */
	std::sort(imgSortedCorners.begin(), imgSortedCorners.begin() + imgSortedCorners.size(), [&](cv::Point2f a, cv::Point2f b)
		{ return a.y < b.y; });
	if (imgSortedCorners[0].y - extendY > 0)
		y = imgSortedCorners[0].y - extendY;
	else
		y = 0;
	if (imgSortedCorners[imgSortedCorners.size() - 1].y + extendY > picSize.height - 1)
		height = picSize.height - 1 - y;
	else
		height = imgSortedCorners[imgSortedCorners.size() - 1].y + extendY - y;
	y = y + extendY * 2;
	height = height - extendY * 2;
	border = Vec4i(x, y, width, height);
	////�����ǵ�
	//Mat drawCorners;
	//cvtColor(img, drawCorners, COLOR_GRAY2BGR);
	//drawChessboardCorners(drawCorners, boardSize, imgCorners, true); // ������ͼƬ�б�ǽǵ�
	//namedWindow("drawCorners", WINDOW_NORMAL);
	//imshow("drawCorners", drawCorners);
	////�����߽�
	//Mat drawBorder;
	//cvtColor(img, drawBorder, COLOR_GRAY2BGR);
	//rectangle(drawBorder, Point(x, y), Point(x + width, y + height), Scalar(0, 0, 255), 2);
	//namedWindow("drawBorder", WINDOW_NORMAL);
	//imshow("drawBorder", drawBorder);
	//waitKey(0);
}

void imgPreProcess(Mat img, int thre, Mat& dstImg, vector<Vec4i>& dstRanges) {
	Mat proImg = img.clone();
	if (proImg.channels() > 1)
		cvtColor(proImg, proImg, COLOR_BGR2GRAY);
	Mat bwImg;
	//cout << "otsu: " << imgp.GetOtsuThresh(proImg) << endl;
	threshold(proImg, bwImg, thre, 255, THRESH_BINARY);//50,100,GetOtsuThresh(img),��ֵ1
	//namedWindow("bwImg", WINDOW_NORMAL);
	//imshow("bwImg", bwImg);
	//waitKey(0);
	//�ҳ�������Ե
	std::vector<std::vector<cv::Point>> contours;
	std::vector<std::vector<cv::Point>> contoursOr;
	std::vector<std::vector<cv::Point>>::iterator it;
	std::vector<cv::Vec4i>hierarchy;
	cv::findContours(bwImg, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
	contoursOr = contours;
	vector<vector<Point2i>> finalContourSet;
	double areaThr = 50;//50,200
	Mat Temp = cv::Mat::zeros(bwImg.rows, bwImg.cols, bwImg.type());
	vector<Vec4i> ranges;
	for (int i = 0; i < contours.size(); i++)
	{
		double area = contourArea(contours[i]);
		if (area > areaThr)//���С��areaThr�Ͳ�Ҫ
		{
			//cout << "area:  " << area << endl;
			finalContourSet.push_back(contours[i]);
			Point2i leftP = *min_element(contours[i].begin(), contours[i].end(), [&](Point2i a, Point2i b) {return a.x < b.x; });
			Point2i RightP = *max_element(contours[i].begin(), contours[i].end(), [&](Point2i a, Point2i b) {return a.x < b.x; });
			Point2i topP = *min_element(contours[i].begin(), contours[i].end(), [&](Point2i a, Point2i b) {return a.y < b.y; });
			Point2i botP = *max_element(contours[i].begin(), contours[i].end(), [&](Point2i a, Point2i b) {return a.y < b.y; });
			//cout << "leftP: " << leftP << endl;
			//cout << "RightP: " << RightP << endl;
			//cout << "topP: " << topP << endl;
			//cout << "botP: " << botP << endl;
			int left = leftP.x;
			int right = RightP.x;
			int top = topP.y;
			int bot = botP.y;

			//sort(contours[i].begin(), contours[i].end(), [&](Point2i a, Point2i b) {return a.x < b.x; });
			//int left = contours[i][0].x;
			//int right = contours[i][contours[i].size() - 1].x;
			//sort(contours[i].begin(), contours[i].end(), [&](Point2i a, Point2i b) {return a.y < b.y; });
			//int top = contours[i][0].y;
			//int bot = contours[i][contours[i].size() - 1].y;

			if (bot - top < (right - left) * 1.0)
				continue;
			cv::drawContours(Temp, contoursOr, i, cv::Scalar(255), cv::FILLED);
			//namedWindow("Temp", WINDOW_NORMAL);
			//imshow("Temp", Temp);
			//waitKey(0);
			ranges.push_back(Vec4i(top, bot, left, right));

		}
	}
	imgPro imgp;
	imgp.domainSmooth(Temp, Temp);//����ƽ��
	//namedWindow("Temp", WINDOW_NORMAL);
	//imshow("temp", Temp);
	//waitKey(0);
	Mat dstImg0 = img.mul(Temp / 255);
	GaussianBlur(dstImg0, dstImg0, Size(3, 3), 2, 2);
	//namedWindow("dstImg", WINDOW_NORMAL);
	//imshow("dstImg", dstImg0);
	//waitKey(1);
	dstImg = dstImg0.clone();
	dstRanges = ranges;
}

void imgPreProcess_simple(Mat img, int thre, Mat& dstImg, vector<Vec4i>& dstRanges) {
	Mat proImg = img.clone();
	GaussianBlur(proImg, proImg, Size(0, 0), 2, 2);
	if (proImg.channels() > 1)
		cvtColor(proImg, proImg, COLOR_BGR2GRAY);
	Mat bwImg;
	//cout << "otsu: " << imgp.GetOtsuThresh(proImg) << endl;
	threshold(proImg, bwImg, thre, 255, THRESH_BINARY);//50,100,GetOtsuThresh(img),��ֵ1
	//namedWindow("bwImg", WINDOW_NORMAL);
	//imshow("bwImg", bwImg);
	//waitKey(0);
	//�ҳ�������Ե
	std::vector<std::vector<cv::Point>> contours;
	std::vector<std::vector<cv::Point>> contoursOr;
	std::vector<std::vector<cv::Point>>::iterator it;
	std::vector<cv::Vec4i>hierarchy;
	cv::findContours(bwImg, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
	contoursOr = contours;
	vector<vector<Point2i>> finalContourSet;
	double areaThr = 50;//50,200
	Mat Temp = cv::Mat::zeros(bwImg.rows, bwImg.cols, bwImg.type());
	vector<Vec4i> ranges;
	for (int i = 0; i < contours.size(); i++)
	{
		double area = contourArea(contours[i]);
		if (area > areaThr)//���С��areaThr�Ͳ�Ҫ
		{
			//cout << "area:  " << area << endl;
			finalContourSet.push_back(contours[i]);
			Point2i leftP = *min_element(contours[i].begin(), contours[i].end(), [&](Point2i a, Point2i b) {return a.x < b.x; });
			Point2i RightP = *max_element(contours[i].begin(), contours[i].end(), [&](Point2i a, Point2i b) {return a.x < b.x; });
			Point2i topP = *min_element(contours[i].begin(), contours[i].end(), [&](Point2i a, Point2i b) {return a.y < b.y; });
			Point2i botP = *max_element(contours[i].begin(), contours[i].end(), [&](Point2i a, Point2i b) {return a.y < b.y; });
			//cout << "leftP: " << leftP << endl;
			//cout << "RightP: " << RightP << endl;
			//cout << "topP: " << topP << endl;
			//cout << "botP: " << botP << endl;
			int left = leftP.x;
			int right = RightP.x;
			int top = topP.y;
			int bot = botP.y;

			//sort(contours[i].begin(), contours[i].end(), [&](Point2i a, Point2i b) {return a.x < b.x; });
			//int left = contours[i][0].x;
			//int right = contours[i][contours[i].size() - 1].x;
			//sort(contours[i].begin(), contours[i].end(), [&](Point2i a, Point2i b) {return a.y < b.y; });
			//int top = contours[i][0].y;
			//int bot = contours[i][contours[i].size() - 1].y;

			//if (bot - top < (right - left) * 1.0)
			//	continue;
			cv::drawContours(Temp, contoursOr, i, cv::Scalar(255), cv::FILLED);
			//namedWindow("Temp", WINDOW_NORMAL);
			//imshow("Temp", Temp);
			//waitKey(0);
			ranges.push_back(Vec4i(top, bot, left, right));

		}
	}
	Mat dstImg0 = proImg.mul(Temp / 255);
	dstImg = dstImg0.clone();
	dstRanges = ranges;
}

void imgPreProcess_heng(Mat img, int thre, Mat& dstImg, vector<Vec4i>& dstRanges) {
	Mat proImg = img.clone();
	if (proImg.channels() > 1)
		cvtColor(proImg, proImg, COLOR_BGR2GRAY);
	Mat bwImg;
	//cout << "otsu: " << imgp.GetOtsuThresh(proImg) << endl;
	threshold(proImg, bwImg, thre, 255, THRESH_BINARY);//50,100,GetOtsuThresh(img),��ֵ1
	//namedWindow("bwImg", WINDOW_NORMAL);
	//imshow("bwImg", bwImg);
	//waitKey(0);
	//�ҳ�������Ե
	std::vector<std::vector<cv::Point>> contours;
	std::vector<std::vector<cv::Point>> contoursOr;
	std::vector<std::vector<cv::Point>>::iterator it;
	std::vector<cv::Vec4i>hierarchy;
	cv::findContours(bwImg, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
	contoursOr = contours;
	vector<vector<Point2i>> finalContourSet;
	double areaThr = 50;//50,200
	Mat Temp = cv::Mat::zeros(bwImg.rows, bwImg.cols, bwImg.type());
	vector<Vec4i> ranges;
	for (int i = 0; i < contours.size(); i++)
	{
		double area = contourArea(contours[i]);
		if (area > areaThr)//���С��areaThr�Ͳ�Ҫ
		{
			//cout << "area:  " << area << endl;
			finalContourSet.push_back(contours[i]);
			Point2i leftP = *min_element(contours[i].begin(), contours[i].end(), [&](Point2i a, Point2i b) {return a.x < b.x; });
			Point2i RightP = *max_element(contours[i].begin(), contours[i].end(), [&](Point2i a, Point2i b) {return a.x < b.x; });
			Point2i topP = *min_element(contours[i].begin(), contours[i].end(), [&](Point2i a, Point2i b) {return a.y < b.y; });
			Point2i botP = *max_element(contours[i].begin(), contours[i].end(), [&](Point2i a, Point2i b) {return a.y < b.y; });
			//cout << "leftP: " << leftP << endl;
			//cout << "RightP: " << RightP << endl;
			//cout << "topP: " << topP << endl;
			//cout << "botP: " << botP << endl;
			int left = leftP.x;
			int right = RightP.x;
			int top = topP.y - 5;
			int bot = botP.y + 5;
			//if (left < 0)
			//	left = 0;
			if (top < 0)
				top = 0;
			//if (right > img.cols - 1)
			//	right = img.cols - 1;
			if (bot > img.rows - 1)
				bot = img.rows - 1;

			//sort(contours[i].begin(), contours[i].end(), [&](Point2i a, Point2i b) {return a.x < b.x; });
			//int left = contours[i][0].x;
			//int right = contours[i][contours[i].size() - 1].x;
			//sort(contours[i].begin(), contours[i].end(), [&](Point2i a, Point2i b) {return a.y < b.y; });
			//int top = contours[i][0].y;
			//int bot = contours[i][contours[i].size() - 1].y;

			if (right - left < (bot - top) * 1.0)
				continue;
			cv::drawContours(Temp, contoursOr, i, cv::Scalar(255), cv::FILLED);
			//namedWindow("Temp", WINDOW_NORMAL);
			//imshow("Temp", Temp);
			//waitKey(0);
			ranges.push_back(Vec4i(top, bot, left, right));

		}
	}
	imgPro imgp;
	imgp.domainSmooth(Temp, Temp);//����ƽ��
	Mat dstImg0 = img.mul(Temp / 255);
	GaussianBlur(dstImg0, dstImg0, Size(3, 3), 2, 2);
	//namedWindow("dstImg", WINDOW_NORMAL);
	//imshow("dstImg", dstImg0);
	//waitKey(0);
	dstImg = dstImg0.clone();
	dstRanges = ranges;
}

bool refract_line(double n0, double n1, Vector3d r0, Vector3d N, Vector3d& r1)
{
	N.normalize();
	r0.normalize();
	//cout << "n" << endl << N << endl;
	//cout << "r0" << endl << r0 << endl;
	double n01 = n0 / n1;
	//cout << "n01" << endl << n01 << endl;
	double c1 = N.dot(r0);
	double c2 = 1 - n01 * n01 * (1 - c1 * c1);
	if (c2 > 0)
	{
		r1 = n01 * r0 - N * (n01 * c1 - sqrt(c2));
		r1.normalize();
		return true;
	}
	else
	{
		return false;
	}
}

//Vector3d GetPlaneInterectPt(VectorXd vp, Vector4d plane) {
//	Vector3d q = vp.head(3);
//	Vector3d n = vp.tail(3);
//	double a = n[0];
//	double b = n[1];
//	double c = n[2];
//	Matrix3d A;
//	Vector3d B;
//	A << b, -a, 0, c, 0, -a, plane[0], plane[1], plane[2];
//	B << q[0] * b - q[1] * a, q[0] * c - q[2] * a, -plane[3];
//	Vector3d pt = A.inverse() * B;
//	return pt;
//}

void FitPLaneBySVD(MatrixXd pts, Eigen::VectorXd& coeff, vector<double>& errors) {
	MatrixXd A = MatrixXd::Zero(pts.rows(), 4);
	VectorXd X = pts.col(0);
	VectorXd Y = pts.col(1);
	VectorXd Z = pts.col(2);
	A.col(0) = X;
	A.col(1) = Y;
	A.col(2) = Z;
	A.col(3) = Eigen::VectorXd::Ones(pts.rows(), 1);
	JacobiSVD<MatrixXd> svd_A(A, ComputeFullU | ComputeFullV);
	VectorXd sol = svd_A.matrixV().col(3);
	coeff = sol;
	coeff = coeff / coeff.block(0, 0, 3, 1).norm();
	for (int i = 0; i < pts.rows(); i++) {
		double x = X[i];
		double y = Y[i];
		//double z1 = -(coeff[0] * x + coeff[1] * y + coeff[3]) / coeff[2];
		//double tError = abs(z1 - Z[i]);
		double z = Z[i];
		double tError = abs(coeff.transpose() * Vector4d(x, y, z, 1));
		//cout << tError << endl;
		errors.push_back(tError);
	}
}

// Calculate Hessian2D matrix
void Hessian2D(const Mat& I, double sigma, Mat& Dx, Mat& Dy, Mat& Dxx, Mat& Dxy, Mat& Dyy) {
	// ��������Χ
	int range = static_cast<int>(round(3 * sigma));
	int size = 2 * range + 1;
	// ��������
	Mat X(size, size, CV_64F), Y(size, size, CV_64F);
	for (int i = -range; i <= range; ++i) {
		for (int j = -range; j <= range; ++j) {
			X.at<double>(i + range, j + range) = j;
			Y.at<double>(i + range, j + range) = i;
		}
	}
	// ���� exp(-(X^2 + Y^2) / (2 * sigma^2))
	Mat X2_Y2 = (X.mul(X) + Y.mul(Y)) / (2 * sigma * sigma);
	Mat expTerm;
	cv::exp(-X2_Y2, expTerm); // OpenCV �� exp ����
	// �����˹����
	Mat DGaussx = (1 / (2 * CV_PI * pow(sigma, 4))) * (-X).mul(expTerm);
	Mat DGaussy = DGaussx.t(); // ת��
	Mat DGaussxx = (1 / (2 * CV_PI * pow(sigma, 4))) * ((X.mul(X) / (sigma * sigma)) - 1).mul(expTerm);
	Mat DGaussxy = (1 / (2 * CV_PI * pow(sigma, 6))) * (X.mul(Y)).mul(expTerm);
	Mat DGaussyy = DGaussxx.t(); // ת��
	cv::flip(DGaussx, DGaussx, -1);
	cv::flip(DGaussy, DGaussy, -1);
	cv::flip(DGaussxx, DGaussxx, -1);
	cv::flip(DGaussxy, DGaussxy, -1);
	// Ӧ�þ���
	filter2D(I, Dx, CV_64F, DGaussx, Point(-1, -1), 0, BORDER_CONSTANT);
	filter2D(I, Dy, CV_64F, DGaussy, Point(-1, -1), 0, BORDER_CONSTANT);
	filter2D(I, Dxx, CV_64F, DGaussxx, Point(-1, -1), 0, BORDER_CONSTANT);
	filter2D(I, Dxy, CV_64F, DGaussxy, Point(-1, -1), 0, BORDER_CONSTANT);
	filter2D(I, Dyy, CV_64F, DGaussyy, Point(-1, -1), 0, BORDER_CONSTANT);
}

// Helper function to compute eigenvalues and eigenvectors
void eig2image(const Mat& Dxx, const Mat& Dxy, const Mat& Dyy, Mat& Lambda1, Mat& Lambda2, Mat& Ix, Mat& Iy) {
	Mat tmp;
	sqrt((Dxx - Dyy).mul(Dxx - Dyy) + 4.0 * Dxy.mul(Dxy), tmp);

	Mat v1x = 2.0 * Dxy;
	Mat v1y = Dyy - Dxx + tmp;

	// Normalize
	Mat mag;
	magnitude(v1x, v1y, mag);
	for (int i = 0; i < v1x.rows; i++) {
		for (int j = 0; j < v1x.cols; j++) {
			if (mag.at<double>(i, j) > 0) {
				v1x.at<double>(i, j) /= mag.at<double>(i, j);
				v1y.at<double>(i, j) /= mag.at<double>(i, j);
			}
		}
	}
	//v1x /= mag;
	//v1y /= mag;

	Mat v2x = -v1y;
	Mat v2y = v1x;

	// Compute eigenvalues
	Lambda1 = 0.5 * (Dxx + Dyy + tmp);
	Lambda2 = 0.5 * (Dxx + Dyy - tmp);

	// Sort eigenvalues by absolute value
	Mat check = abs(Lambda1) < abs(Lambda2);

	Ix = v1x.clone();
	Iy = v1y.clone();

	for (int i = 0; i < check.rows; i++) {
		for (int j = 0; j < check.cols; j++) {
			if (check.at<uchar>(i, j) > 0) {
				Ix.at<double>(i, j) = v2x.at<double>(i, j);
				Iy.at<double>(i, j) = v2y.at<double>(i, j);
			}
		}
	}
}

// Main function to find the laser center
vector<Point2d> findLaserCenterSteger(const Mat& inputImage, double bwThr, double sltThr, double stripeWidth) {
	Mat im;
	inputImage.convertTo(im, CV_64F);
	normalize(im, im, 0, 1, NORM_MINMAX);
	sltThr = sltThr / 255.0;

	Mat bw;
	threshold(im, bw, bwThr, 1, THRESH_BINARY);

	Mat Dx, Dy, Dxx, Dxy, Dyy;
	double sigma = stripeWidth / sqrt(3.0);
	Hessian2D(im, sigma, Dx, Dy, Dxx, Dxy, Dyy);

	Mat Lambda1, Lambda2, Ix, Iy;
	eig2image(Dxx, Dxy, Dyy, Lambda1, Lambda2, Ix, Iy);

	Mat t = -(Dx.mul(Ix) + Dy.mul(Iy)) / (Dxx.mul(Ix.mul(Ix)) + 2.0 * Dxy.mul(Ix.mul(Iy)) + Dyy.mul(Iy.mul(Iy)));

	Mat px = t.mul(Ix);
	Mat py = t.mul(Iy);

	/* �ҵ����������ĵ㣬�����ң�����x */
	//vector<Point2d> laserPoints;
	//vector<Point2i> candidatePoints;
	//for (int x = 0; x < px.cols; ++x) {
	//	for (int y = 0; y < px.rows; ++y) {
	//		if (im.at<double>(y, x) > sltThr && abs(px.at<double>(y, x)) <= 0.5 && abs(py.at<double>(y, x)) <= 0.5) {
	//			laserPoints.emplace_back(x + px.at<double>(y, x), y + py.at<double>(y, x));
	//			candidatePoints.emplace_back(x, y);
	//		}
	//	}
	//}

	//// �Ե����ɸѡ-x
	//vector<Point2d> filteredPoints;  // �洢ɸѡ��ĵ�
	//int n = laserPoints.size();
	//vector<bool> processed(n, false); // ����Ƿ��Ѵ������õ�
	//for (int i = 0; i < n; i++) {
	//	//cout << i << "\t";
	//	if (processed[i])
	//		continue;  // �����ǰ���Ѵ�����������
	//	else
	//		processed[i] = true;  // ��ǵ�ǰ���Ѵ���
	//	// ��ǰ��� y ����
	//	int y = candidatePoints[i].y;
	//	int x = candidatePoints[i].x;
	//	// �ҵ��������ڵ�y������ͬ�ĵ�
	//	int minIndex = i;
	//	double value = abs(px.at<double>(y, x)) + abs(py.at<double>(y, x));
	//	for (int j = i + 1; j < n; j++) {
	//		if (candidatePoints[j].x != x) {
	//			break;  // �����һ����� y ����ȣ�ֱ������ѭ��
	//		}
	//		if (candidatePoints[j].x == x && !processed[j]) {
	//			// ѡ���Ӧ value ֵ��С�ĵ�
	//			if (abs(px.at<double>(candidatePoints[j].y, candidatePoints[j].x)) + abs(py.at<double>(candidatePoints[j].y, candidatePoints[j].x)) < value) {
	//				minIndex = j;
	//				value = abs(px.at<double>(candidatePoints[j].y, candidatePoints[j].x)) + abs(py.at<double>(candidatePoints[j].y, candidatePoints[j].x));
	//			}
	//		}
	//		processed[j] = true;
	//	}
	//	// ����Сֵ��Ӧ�ĵ���뵽�����
	//	filteredPoints.push_back(laserPoints[minIndex]);
	//	//cout << minIndex << "\t" << fixed << setprecision(4) << laserPoints[minIndex].x << "\t" << laserPoints[minIndex].y << endl;
	//}
	//laserPoints = filteredPoints;

	/* �ҵ����������ĵ㣬������, ����y */
	vector<Point2d> laserPoints;
	vector<Point2i> candidatePoints;
	for (int y = 0; y < px.rows; ++y) {
		for (int x = 0; x < px.cols; ++x) {
			if (im.at<double>(y, x) > sltThr && abs(px.at<double>(y, x)) <= 0.5 && abs(py.at<double>(y, x)) <= 0.5) {
				laserPoints.emplace_back(x + px.at<double>(y, x), y + py.at<double>(y, x));
				candidatePoints.emplace_back(x, y);
			}
		}
	}

	//// �Ե����ɸѡ-y
	//vector<Point2d> filteredPoints;  // �洢ɸѡ��ĵ�
	//int n = laserPoints.size();
	//vector<bool> processed(n, false); // ����Ƿ��Ѵ������õ�
	//for (int i = 0; i < n; i++) {
	//	//cout << i << "\t";
	//	if (processed[i]) 
	//		continue;  // �����ǰ���Ѵ�����������
	//	else
	//		processed[i] = true;  // ��ǵ�ǰ���Ѵ���
	//	// ��ǰ��� y ����
	//	int y = candidatePoints[i].y;
	//	int x = candidatePoints[i].x;
	//	// �ҵ��������ڵ�y������ͬ�ĵ�
	//	int minIndex = i;
	//	double value = abs(px.at<double>(y, x)) + abs(py.at<double>(y, x));
	//	for (int j = i + 1; j < n; j++) {
	//		if (candidatePoints[j].y != y) {
	//			break;  // �����һ����� y ����ȣ�ֱ������ѭ��
	//		}
	//		if (candidatePoints[j].y == y && !processed[j]) {
	//			// ѡ���Ӧ value ֵ��С�ĵ�
	//			if (abs(px.at<double>(candidatePoints[j].y, candidatePoints[j].x)) + abs(py.at<double>(candidatePoints[j].y, candidatePoints[j].x)) < value) {
	//				minIndex = j;
	//				value = abs(px.at<double>(candidatePoints[j].y, candidatePoints[j].x)) + abs(py.at<double>(candidatePoints[j].y, candidatePoints[j].x));
	//			}
	//		}
	//		processed[j] = true;
	//	}
	//	// ����Сֵ��Ӧ�ĵ���뵽�����
	//	filteredPoints.push_back(laserPoints[minIndex]);
	//	cout << minIndex << "\t" << fixed << setprecision(4) << laserPoints[minIndex].x << "\t" << laserPoints[minIndex].y << endl;
	//}
	//cout << filteredPoints.size() << "\t" << n << endl;
	//laserPoints = filteredPoints;


	return laserPoints;
}

std::vector<cv::Point2d> findLaserCenterSteger_v2_heng(const cv::Mat& inputImage, double bwThr, double sltThr, double stripeWidth, bool filter) {
	//sltThr = sltThr / 255.0;
	Mat proImg = inputImage.clone();
	if (inputImage.channels() > 1)
		cvtColor(proImg, proImg, COLOR_BGR2GRAY);
	Mat bwImg;
	threshold(proImg, bwImg, bwThr, 255, THRESH_BINARY);//50,100,GetOtsuThresh(img),��ֵ1
	//��ȡRoi
	vector<Vec4i> Rois;
	GetImageRanges(bwImg, Rois, 50, 10);
	//����˹��
	// ��������Χ
	double sigma = stripeWidth / sqrt(3.0);
	int range = static_cast<int>(round(3 * sigma));
	int size = 2 * range + 1;
	// ��������
	Mat X(size, size, CV_64F), Y(size, size, CV_64F);
	for (int i = -range; i <= range; ++i) {
		for (int j = -range; j <= range; ++j) {
			X.at<double>(i + range, j + range) = j;
			Y.at<double>(i + range, j + range) = i;
		}
	}
	// ���� exp(-(X^2 + Y^2) / (2 * sigma^2))
	Mat X2_Y2 = (X.mul(X) + Y.mul(Y)) / (2 * sigma * sigma);
	Mat expTerm;
	cv::exp(-X2_Y2, expTerm); // OpenCV �� exp ����
	// �����˹����
	Mat DGaussx = (1 / (2 * CV_PI * pow(sigma, 4))) * (-X).mul(expTerm);
	Mat DGaussy = DGaussx.t(); // ת��
	Mat DGaussxx = (1 / (2 * CV_PI * pow(sigma, 4))) * ((X.mul(X) / (sigma * sigma)) - 1).mul(expTerm);
	Mat DGaussxy = (1 / (2 * CV_PI * pow(sigma, 6))) * (X.mul(Y)).mul(expTerm);
	Mat DGaussyy = DGaussxx.t(); // ת��
	cv::flip(DGaussx, DGaussx, -1);
	cv::flip(DGaussy, DGaussy, -1);
	cv::flip(DGaussxx, DGaussxx, -1);
	cv::flip(DGaussxy, DGaussxy, -1);
	//�������Roi��Hessian����
	vector<Point2d> linePoints;
	for (int k = 0; k < Rois.size(); k++) {
		Vec4i roi = Rois[k];
		Mat roiImg = proImg(Range(roi[0], roi[1] + 1), Range(roi[2], roi[3] + 1)).clone();
		//namedWindow("roiImg", WINDOW_NORMAL);
		//imshow("roiImg", roiImg);
		//waitKey(0);
		roiImg.convertTo(roiImg, CV_64F);
		Mat Dx, Dy, Dxx, Dxy, Dyy;
		// Ӧ�þ���
		filter2D(roiImg, Dx, CV_64F, DGaussx, Point(-1, -1), 0, BORDER_CONSTANT);
		filter2D(roiImg, Dy, CV_64F, DGaussy, Point(-1, -1), 0, BORDER_CONSTANT);
		filter2D(roiImg, Dxx, CV_64F, DGaussxx, Point(-1, -1), 0, BORDER_CONSTANT);
		filter2D(roiImg, Dxy, CV_64F, DGaussxy, Point(-1, -1), 0, BORDER_CONSTANT);
		filter2D(roiImg, Dyy, CV_64F, DGaussyy, Point(-1, -1), 0, BORDER_CONSTANT);
		//�������ֵ
		Mat tmp;
		sqrt((Dxx - Dyy).mul(Dxx - Dyy) + 4.0 * Dxy.mul(Dxy), tmp);
		Mat v1x = 2.0 * Dxy;
		Mat v1y = Dyy - Dxx + tmp;
		// Normalize
		Mat mag;
		magnitude(v1x, v1y, mag);
		//for (int i = 0; i < v1x.rows; i++) {
		//	for (int j = 0; j < v1x.cols; j++) {
		//		if (mag.at<double>(i, j) > 0) {
		//			v1x.at<double>(i, j) /= mag.at<double>(i, j);
		//			v1y.at<double>(i, j) /= mag.at<double>(i, j);
		//		}
		//	}
		//}
		v1x /= mag;
		v1y /= mag;
		//�ڶ�����������
		Mat v2x = -v1y;
		Mat v2y = v1x;
		// Compute eigenvalues
		Mat Lambda1 = 0.5 * (Dxx + Dyy + tmp);
		Mat Lambda2 = 0.5 * (Dxx + Dyy - tmp);
		// Sort eigenvalues by absolute value
		Mat check = abs(Lambda1) < abs(Lambda2);
		Mat Ix = v1x.clone();
		Mat Iy = v1y.clone();
		v2x.copyTo(Ix, check);
		v2y.copyTo(Iy, check);
		Mat t = -(Dx.mul(Ix) + Dy.mul(Iy)) / (Dxx.mul(Ix.mul(Ix)) + 2.0 * Dxy.mul(Ix.mul(Iy)) + Dyy.mul(Iy.mul(Iy)));
		Mat px = t.mul(Ix);
		Mat py = t.mul(Iy);
		vector<Point2d> laserPoints;
		//vector<Point2i> candidatePoints;
		for (int x = 0; x < px.cols; ++x) {
			for (int y = 0; y < px.rows; ++y) {
				if (roiImg.at<double>(y, x) <= sltThr)
					continue;
				else if (abs(px.at<double>(y, x)) <= 0.5 && abs(py.at<double>(y, x)) <= 0.5) {
					laserPoints.emplace_back(x + px.at<double>(y, x) + roi[2], y + py.at<double>(y, x) + roi[0]);
					//candidatePoints.emplace_back(x, y);
				}
			}
		}
		linePoints.insert(linePoints.end(), laserPoints.begin(), laserPoints.end());
	}
	return linePoints;
}

std::vector<cv::Point2d> findLaserCenterSteger_v2_shu(const cv::Mat& inputImage, double bwThr, double sltThr, double stripeWidth, bool filter) {
	//sltThr = sltThr / 255.0;
	Mat proImg = inputImage.clone();
	if (inputImage.channels() > 1)
		cvtColor(proImg, proImg, COLOR_BGR2GRAY);
	Mat bwImg;
	threshold(proImg, bwImg, bwThr, 255, THRESH_BINARY);//50,100,GetOtsuThresh(img),��ֵ1
	//������
	morphologyEx(bwImg, bwImg, MORPH_OPEN, getStructuringElement(MORPH_RECT, Size(3, 3)));
	//��ȡRoi
	vector<Vec4i> Rois;
	GetImageRanges(bwImg, Rois, 50, 10);
	//����˹��
	// ��������Χ
	double sigma = stripeWidth / sqrt(3.0);
	int range = static_cast<int>(round(3 * sigma));
	int size = 2 * range + 1;
	// ��������
	Mat X(size, size, CV_64F), Y(size, size, CV_64F);
	for (int i = -range; i <= range; ++i) {
		for (int j = -range; j <= range; ++j) {
			X.at<double>(i + range, j + range) = j;
			Y.at<double>(i + range, j + range) = i;
		}
	}
	// ���� exp(-(X^2 + Y^2) / (2 * sigma^2))
	Mat X2_Y2 = (X.mul(X) + Y.mul(Y)) / (2 * sigma * sigma);
	Mat expTerm;
	cv::exp(-X2_Y2, expTerm); // OpenCV �� exp ����
	// �����˹����
	Mat DGaussx = (1 / (2 * CV_PI * pow(sigma, 4))) * (-X).mul(expTerm);
	Mat DGaussy = DGaussx.t(); // ת��
	Mat DGaussxx = (1 / (2 * CV_PI * pow(sigma, 4))) * ((X.mul(X) / (sigma * sigma)) - 1).mul(expTerm);
	Mat DGaussxy = (1 / (2 * CV_PI * pow(sigma, 6))) * (X.mul(Y)).mul(expTerm);
	Mat DGaussyy = DGaussxx.t(); // ת��
	cv::flip(DGaussx, DGaussx, -1);
	cv::flip(DGaussy, DGaussy, -1);
	cv::flip(DGaussxx, DGaussxx, -1);
	cv::flip(DGaussxy, DGaussxy, -1);
	//�������Roi��Hessian����
	vector<Point2d> linePoints;
	//ͳ�Ƹ�Ԫ���Ƿ񱻷���
	Mat visited = Mat::zeros(proImg.size(), CV_8UC1);
	for (int k = 0; k < Rois.size(); k++) {
		Vec4i roi = Rois[k];
		Mat roiImg = proImg(Range(roi[0], roi[1] + 1), Range(roi[2], roi[3] + 1)).clone();
		//Mat tempBwimg = bwImg(Range(roi[0], roi[1] + 1), Range(roi[2], roi[3] + 1));
		//Mat drawproImg;
		//cvtColor(proImg, drawproImg, COLOR_GRAY2BGR);
		//rectangle(drawproImg, Rect(roi[2], roi[0], roi[3] - roi[2], roi[1] - roi[0]), Scalar(0, 0, 255), 2);
		//namedWindow("drawproImg", WINDOW_NORMAL);
		//imshow("drawproImg", drawproImg);
		//namedWindow("roiImg", WINDOW_NORMAL);
		//imshow("roiImg", roiImg);
		//namedWindow("tempBwimg", WINDOW_NORMAL);
		//imshow("tempBwimg", tempBwimg);
		//waitKey(0);
		roiImg.convertTo(roiImg, CV_64F);
		Mat Dx, Dy, Dxx, Dxy, Dyy;
		// Ӧ�þ���
		filter2D(roiImg, Dx, CV_64F, DGaussx, Point(-1, -1), 0, BORDER_CONSTANT);
		filter2D(roiImg, Dy, CV_64F, DGaussy, Point(-1, -1), 0, BORDER_CONSTANT);
		filter2D(roiImg, Dxx, CV_64F, DGaussxx, Point(-1, -1), 0, BORDER_CONSTANT);
		filter2D(roiImg, Dxy, CV_64F, DGaussxy, Point(-1, -1), 0, BORDER_CONSTANT);
		filter2D(roiImg, Dyy, CV_64F, DGaussyy, Point(-1, -1), 0, BORDER_CONSTANT);
		//�������ֵ
		Mat tmp;
		sqrt((Dxx - Dyy).mul(Dxx - Dyy) + 4.0 * Dxy.mul(Dxy), tmp);
		Mat v1x = 2.0 * Dxy;
		Mat v1y = Dyy - Dxx + tmp;
		// Normalize
		Mat mag;
		magnitude(v1x, v1y, mag);
		//for (int i = 0; i < v1x.rows; i++) {
		//	for (int j = 0; j < v1x.cols; j++) {
		//		if (mag.at<double>(i, j) > 0) {
		//			v1x.at<double>(i, j) /= mag.at<double>(i, j);
		//			v1y.at<double>(i, j) /= mag.at<double>(i, j);
		//		}
		//	}
		//}
		v1x /= mag;
		v1y /= mag;
		//�ڶ�����������
		Mat v2x = -v1y;
		Mat v2y = v1x;
		// Compute eigenvalues
		Mat Lambda1 = 0.5 * (Dxx + Dyy + tmp);
		Mat Lambda2 = 0.5 * (Dxx + Dyy - tmp);
		// Sort eigenvalues by absolute value
		Mat check = abs(Lambda1) < abs(Lambda2);
		Mat Ix = v1x.clone();
		Mat Iy = v1y.clone();
		v2x.copyTo(Ix, check);
		v2y.copyTo(Iy, check);
		Mat t = -(Dx.mul(Ix) + Dy.mul(Iy)) / (Dxx.mul(Ix.mul(Ix)) + 2.0 * Dxy.mul(Ix.mul(Iy)) + Dyy.mul(Iy.mul(Iy)));
		Mat px = t.mul(Ix);
		Mat py = t.mul(Iy);
		vector<Point2d> laserPoints;
		vector<Point2i> candidatePoints;
		if (filter) {
			for (int y = 0; y < px.rows; ++y) {
				for (int x = 0; x < px.cols; ++x) {
					if (roiImg.at<double>(y, x) <= sltThr || visited.at<uchar>(y + roi[0], x + roi[2]) > 0 || bwImg.at<uchar>(y + roi[0], x + roi[2]) == 0)
						//if (roiImg.at<double>(y, x) <= sltThr)
						continue;
					else if (abs(px.at<double>(y, x)) <= 0.5 && abs(py.at<double>(y, x)) <= 0.5) {
						laserPoints.emplace_back(x + px.at<double>(y, x) + roi[2], y + py.at<double>(y, x) + roi[0]);
						candidatePoints.emplace_back(x, y);
						visited.at<uchar>(y + roi[0], x + roi[2]) = 255;
					}
				}
			}
			// �Ե����ɸѡ
			vector<Point2d> filteredPoints;  // �洢ɸѡ��ĵ�
			int n = laserPoints.size();
			vector<bool> processed(n, false); // ����Ƿ��Ѵ������õ�
			for (int i = 0; i < n; i++) {
				//cout << i << "\t";
				if (processed[i])
					continue;  // �����ǰ���Ѵ�����������
				else {
					processed[i] = true;  // ��ǵ�ǰ���Ѵ���
				}

				// ��ǰ��� y ����
				int x = candidatePoints[i].x;
				int y = candidatePoints[i].y;
				// �ҵ��������ڵ�y������ͬ�ĵ�
				int minIndex = i;
				////����px+py�Ľ���ɸѡ
				//double value = abs(px.at<double>(y, x)) + abs(py.at<double>(y, x));
				//for (int j = i + 1; j < n; j++) {
				//	if (candidatePoints[j].y != y) {
				//		break;  // �����һ����� y ����ȣ�ֱ������ѭ��
				//	}
				//	if (candidatePoints[j].y == y && !processed[j]) {
				//		// ѡ���Ӧ value ֵ��С�ĵ�
				//		if (abs(px.at<double>(candidatePoints[j].y, candidatePoints[j].x)) + abs(py.at<double>(candidatePoints[j].y, candidatePoints[j].x)) < value) {
				//			minIndex = j;
				//			value = abs(px.at<double>(candidatePoints[j].y, candidatePoints[j].x)) + abs(py.at<double>(candidatePoints[j].y, candidatePoints[j].x));
				//		}
				//	}
				//	processed[j] = true;
				//}
				// ����Сֵ��Ӧ�ĵ���뵽�����

				//����x��λ�ý���ɸѡ
				int num = 1;
				for (int j = i + 1; j < n; j++) {
					if (candidatePoints[j].y != y) {
						break;  // �����һ����� y ����ȣ�ֱ������ѭ��
					}
					if (candidatePoints[j].y == y && !processed[j]) {
						num++;
					}
					processed[j] = true;
				}
				if (num > 2)
					minIndex = i + (num - 1) / 2;
				filteredPoints.push_back(laserPoints[minIndex]);
				//cout << minIndex << "\t" << fixed << setprecision(4) << laserPoints[minIndex].x << "\t" << laserPoints[minIndex].y << endl;
			}
			laserPoints = filteredPoints;
		}
		else {
			for (int y = 0; y < px.rows; ++y) {
				for (int x = 0; x < px.cols; ++x) {
					//cout << x << "\t" << y << endl;
					if (roiImg.at<double>(y, x) <= sltThr || visited.at<uchar>(y + roi[0], x + roi[2]) > 0 || bwImg.at<uchar>(y + roi[0], x + roi[2]) == 0)
						//if (roiImg.at<double>(y, x) <= sltThr)
						continue;
					else if (abs(px.at<double>(y, x)) <= 0.5 && abs(py.at<double>(y, x)) <= 0.5) {
						laserPoints.emplace_back(x + px.at<double>(y, x) + roi[2], y + py.at<double>(y, x) + roi[0]);
						//candidatePoints.emplace_back(x, y);
						visited.at<uchar>(y + roi[0], x + roi[2]) = 255;
					}
				}
			}
		}
		linePoints.insert(linePoints.end(), laserPoints.begin(), laserPoints.end());
	}
	return linePoints;
}

std::vector<cv::Point2d> findLaserCenterSteger_v5_heng(const cv::Mat& inputImage, double bwThr, double sltThr, double stripeWidth, bool filter) {
	//sltThr = sltThr / 255.0;
	Mat proImg = inputImage.clone();
	if (inputImage.channels() > 1)
		cvtColor(proImg, proImg, COLOR_BGR2GRAY);
	Mat bwImg;
	threshold(proImg, bwImg, bwThr, 255, THRESH_BINARY);//50,100,GetOtsuThresh(img),��ֵ1
	//namedWindow("bwImg", WINDOW_NORMAL);
	//imshow("bwImg", bwImg);
	//waitKey(0);

	//��ȡRoi�����Ұ��������������
	vector<Vec4i> Rois;
	//GetImageRanges(bwImg, Rois, 100, 10);
	GetImageRanges_v2(bwImg, Rois, 100, 10);
	//����˹��
	// ��������Χ
	double sigma = stripeWidth / sqrt(3.0);
	int range = static_cast<int>(round(3 * sigma));
	int size = 2 * range + 1;
	// ��������
	Mat X(size, size, CV_64F), Y(size, size, CV_64F);
	for (int i = -range; i <= range; ++i) {
		for (int j = -range; j <= range; ++j) {
			X.at<double>(i + range, j + range) = j;
			Y.at<double>(i + range, j + range) = i;
		}
	}
	// ���� exp(-(X^2 + Y^2) / (2 * sigma^2))
	Mat X2_Y2 = (X.mul(X) + Y.mul(Y)) / (2 * sigma * sigma);
	Mat expTerm;
	cv::exp(-X2_Y2, expTerm); // OpenCV �� exp ����
	// �����˹����
	Mat DGaussx = (1 / (2 * CV_PI * pow(sigma, 4))) * (-X).mul(expTerm);
	Mat DGaussy = DGaussx.t(); // ת��
	Mat DGaussxx = (1 / (2 * CV_PI * pow(sigma, 4))) * ((X.mul(X) / (sigma * sigma)) - 1).mul(expTerm);
	Mat DGaussxy = (1 / (2 * CV_PI * pow(sigma, 6))) * (X.mul(Y)).mul(expTerm);
	Mat DGaussyy = DGaussxx.t(); // ת��
	cv::flip(DGaussx, DGaussx, -1);
	cv::flip(DGaussy, DGaussy, -1);
	cv::flip(DGaussxx, DGaussxx, -1);
	cv::flip(DGaussxy, DGaussxy, -1);
	//�������Roi��Hessian����
	vector<Point2d> linePoints;
	//ͳ�Ƹ�Ԫ�ص�X�Ƿ񱻷���
	Mat visited_X = Mat::zeros(Size(proImg.cols, 1), CV_8UC1);
	for (int k = 0; k < Rois.size(); k++) {
		Vec4i roi = Rois[k];
		Mat roiImg = proImg(Range(roi[0], roi[1] + 1), Range(roi[2], roi[3] + 1)).clone();
		//namedWindow("roiImg", WINDOW_NORMAL);
		//imshow("roiImg", roiImg);
		//waitKey(0);
		roiImg.convertTo(roiImg, CV_64F);
		Mat Dx, Dy, Dxx, Dxy, Dyy;
		// Ӧ�þ���
		filter2D(roiImg, Dx, CV_64F, DGaussx, Point(-1, -1), 0, BORDER_CONSTANT);
		filter2D(roiImg, Dy, CV_64F, DGaussy, Point(-1, -1), 0, BORDER_CONSTANT);
		filter2D(roiImg, Dxx, CV_64F, DGaussxx, Point(-1, -1), 0, BORDER_CONSTANT);
		filter2D(roiImg, Dxy, CV_64F, DGaussxy, Point(-1, -1), 0, BORDER_CONSTANT);
		filter2D(roiImg, Dyy, CV_64F, DGaussyy, Point(-1, -1), 0, BORDER_CONSTANT);
		//�������ֵ
		Mat tmp;
		sqrt((Dxx - Dyy).mul(Dxx - Dyy) + 4.0 * Dxy.mul(Dxy), tmp);
		Mat v1x = 2.0 * Dxy;
		Mat v1y = Dyy - Dxx + tmp;
		// Normalize
		Mat mag;
		magnitude(v1x, v1y, mag);
		v1x /= mag;
		v1y /= mag;
		//�ڶ�����������
		Mat v2x = -v1y;
		Mat v2y = v1x;
		// Compute eigenvalues
		Mat Lambda1 = 0.5 * (Dxx + Dyy + tmp);
		Mat Lambda2 = 0.5 * (Dxx + Dyy - tmp);
		// Sort eigenvalues by absolute value
		Mat check = abs(Lambda1) < abs(Lambda2);
		Mat Ix = v1x.clone();
		Mat Iy = v1y.clone();
		v2x.copyTo(Ix, check);
		v2y.copyTo(Iy, check);
		Mat t = -(Dx.mul(Ix) + Dy.mul(Iy)) / (Dxx.mul(Ix.mul(Ix)) + 2.0 * Dxy.mul(Ix.mul(Iy)) + Dyy.mul(Iy.mul(Iy)));
		Mat px = t.mul(Ix);
		Mat py = t.mul(Iy);
		vector<Point2d> laserPoints;
		vector<Point2i> candidatePoints;
		//��ȡ��ʼ��Steger�����������ϵĵ�
		for (int x = 0; x < px.cols; ++x) {
			for (int y = 0; y < px.rows; ++y) {
				if (roiImg.at<double>(y, x) <= sltThr)
					continue;
				else if (abs(px.at<double>(y, x)) <= 0.5 && abs(py.at<double>(y, x)) <= 0.5) {
					//else if (abs(px.at<double>(y, x)) <= 0.3 && abs(py.at<double>(y, x)) <= 0.3) {
					laserPoints.emplace_back(x + px.at<double>(y, x) + roi[2], y + py.at<double>(y, x) + roi[0]);
					candidatePoints.emplace_back(x, y);
					//cout << x + px.at<double>(y, x) << "\t" << y + py.at<double>(y, x) << endl;
				}
			}
		}
		if (laserPoints.size() < 10) {
			continue;
		}


		//�Ե����ɸѡ
		vector<Point2d> laserPoints_filtered;  // �洢ɸѡ��ĵ�
		if (filter) {
			//1. ����ͳ�Ƹ���X������ͬ�ĸ���
			vector<int> laserXNum;
			int ind = 0;
			int num = 1;
			while (ind < candidatePoints.size() - 1) {
				int x = candidatePoints[ind].x;
				if (candidatePoints[ind + 1].x == x) {
					num++;
				}
				else {
					laserXNum.push_back(num);
					num = 1;
				}
				ind++;
			}
			//2. ��ʼ����ǿ������index
			vector<pair<int, int>> num_index;
			int index = 0;
			for (int i = 0; i < laserXNum.size(); i++) {
				if (laserXNum[i] > 1) {
					num_index.push_back(make_pair(laserXNum[i], -1));
				}
				else {
					num_index.push_back(make_pair(laserXNum[i], index));
				}
				index += laserXNum[i];
			}
			//3. ��ʼ����Ҫ�����ĵ�
			int expNum = 5;
			for (int m = 0; m < expNum; m++) {
				index = 0;
				for (int i = 0; i < laserXNum.size(); i++) {
					if (num_index[i].second > 0 || i == 0 || i == laserXNum.size() - 1) { // ǿ�����ĵ��Լ����˵ĵ�Ͳ����ټ�����
						index += num_index[i].first;
						continue;
					}
					else { // ��Ҫ����ĵ�
						if (num_index[i - 1].second < 0 && num_index[i + 1].second < 0) { //���߶�û�б���
							index += num_index[i].first;
							continue;
						}
						else {
							vector<Point2i> temp;
							temp.insert(temp.end(), candidatePoints.begin() + index, candidatePoints.begin() + index + num_index[i].first);
							int minIndex = index;
							double minDis = 1000;
							if (num_index[i - 1].second > 0 && num_index[i + 1].second > 0) //���߶�������
							{
								for (int j = 0; j < num_index[i].first; j++) {
									double dis = abs(candidatePoints[index + j].y - candidatePoints[index - 1].y) + abs(candidatePoints[index + j].y - candidatePoints[index + num_index[i].first].y);
									if (dis < minDis) {
										minDis = dis;
										minIndex = index + j;
									}
								}
							}
							else if (num_index[i - 1].second > 0 && num_index[i + 1].second < 0) //��߱�����
							{
								for (int j = 0; j < num_index[i].first; j++) {
									double dis = abs(candidatePoints[index + j].y - candidatePoints[index - 1].y);
									if (dis < minDis) {
										minDis = dis;
										minIndex = index + j;
									}
								}

							}
							else if (num_index[i - 1].second < 0 && num_index[i + 1].second > 0) //�ұ߱�����
							{
								for (int j = 0; j < num_index[i].first; j++) {
									double dis = abs(candidatePoints[index + j].y - candidatePoints[index + num_index[i].first].y);
									if (dis < minDis) {
										minDis = dis;
										minIndex = index + j;
									}
								}

							}
							num_index[i].second = minIndex;
							index += num_index[i].first;
						}
					}
				}
			}
			//4. ����ɸѡ��ĵ�
			for (int i = 0; i < num_index.size(); i++) {
				if (num_index[i].second < 0 || visited_X.ptr<uchar>(0)[candidatePoints[num_index[i].second].x + roi[2]] == 1)
				{
					continue;
				}
				visited_X.ptr<uchar>(0)[candidatePoints[num_index[i].second].x + roi[2]] = 1;
				laserPoints_filtered.push_back(Point2d(laserPoints[num_index[i].second].x, laserPoints[num_index[i].second].y));
				//cout << laserPoints[num_index[i].second].x << "\t" << laserPoints[num_index[i].second].y << endl;
			}
		}
		else
		{
			laserPoints_filtered = laserPoints;
		}
		linePoints.insert(linePoints.end(), laserPoints_filtered.begin(), laserPoints_filtered.end());
	}
	return linePoints;
}

std::vector<cv::Point2d> findLaserCenterSteger_v5_shu(const cv::Mat& inputImage, double bwThr, double sltThr, double stripeWidth, bool filter) {
	//sltThr = sltThr / 255.0;
	Mat proImg = inputImage.clone();
	if (inputImage.channels() > 1)
		cvtColor(proImg, proImg, COLOR_BGR2GRAY);
	Mat bwImg;
	threshold(proImg, bwImg, bwThr, 255, THRESH_BINARY);//50,100,GetOtsuThresh(img),��ֵ1
	//namedWindow("bwImg", WINDOW_NORMAL);
	//imshow("bwImg", bwImg);
	//waitKey(0);

	//��ȡRoi�����Ұ��������������
	vector<Vec4i> Rois;
	//GetImageRanges(bwImg, Rois, 100, 10);
	GetImageRanges_v2(bwImg, Rois, 100, 10);
	//����˹��
	// ��������Χ
	double sigma = stripeWidth / sqrt(3.0);
	int range = static_cast<int>(round(3 * sigma));
	int size = 2 * range + 1;
	// ��������
	Mat X(size, size, CV_64F), Y(size, size, CV_64F);
	for (int i = -range; i <= range; ++i) {
		for (int j = -range; j <= range; ++j) {
			X.at<double>(i + range, j + range) = j;
			Y.at<double>(i + range, j + range) = i;
		}
	}
	// ���� exp(-(X^2 + Y^2) / (2 * sigma^2))
	Mat X2_Y2 = (X.mul(X) + Y.mul(Y)) / (2 * sigma * sigma);
	Mat expTerm;
	cv::exp(-X2_Y2, expTerm); // OpenCV �� exp ����
	// �����˹����
	Mat DGaussx = (1 / (2 * CV_PI * pow(sigma, 4))) * (-X).mul(expTerm);
	Mat DGaussy = DGaussx.t(); // ת��
	Mat DGaussxx = (1 / (2 * CV_PI * pow(sigma, 4))) * ((X.mul(X) / (sigma * sigma)) - 1).mul(expTerm);
	Mat DGaussxy = (1 / (2 * CV_PI * pow(sigma, 6))) * (X.mul(Y)).mul(expTerm);
	Mat DGaussyy = DGaussxx.t(); // ת��
	cv::flip(DGaussx, DGaussx, -1);
	cv::flip(DGaussy, DGaussy, -1);
	cv::flip(DGaussxx, DGaussxx, -1);
	cv::flip(DGaussxy, DGaussxy, -1);
	//�������Roi��Hessian����
	vector<Point2d> linePoints;
	//ͳ�Ƹ�Ԫ�ص�X�Ƿ񱻷���
	Mat visited_Y = Mat::zeros(Size(proImg.cols, 1), CV_8UC1);
	for (int k = 0; k < Rois.size(); k++) {
		Vec4i roi = Rois[k];
		Mat roiImg = proImg(Range(roi[0], roi[1] + 1), Range(roi[2], roi[3] + 1)).clone();
		//namedWindow("roiImg", WINDOW_NORMAL);
		//imshow("roiImg", roiImg);
		//waitKey(0);
		roiImg.convertTo(roiImg, CV_64F);
		Mat Dx, Dy, Dxx, Dxy, Dyy;
		// Ӧ�þ���
		filter2D(roiImg, Dx, CV_64F, DGaussx, Point(-1, -1), 0, BORDER_CONSTANT);
		filter2D(roiImg, Dy, CV_64F, DGaussy, Point(-1, -1), 0, BORDER_CONSTANT);
		filter2D(roiImg, Dxx, CV_64F, DGaussxx, Point(-1, -1), 0, BORDER_CONSTANT);
		filter2D(roiImg, Dxy, CV_64F, DGaussxy, Point(-1, -1), 0, BORDER_CONSTANT);
		filter2D(roiImg, Dyy, CV_64F, DGaussyy, Point(-1, -1), 0, BORDER_CONSTANT);
		//�������ֵ
		Mat tmp;
		sqrt((Dxx - Dyy).mul(Dxx - Dyy) + 4.0 * Dxy.mul(Dxy), tmp);
		Mat v1x = 2.0 * Dxy;
		Mat v1y = Dyy - Dxx + tmp;
		// Normalize
		Mat mag;
		magnitude(v1x, v1y, mag);
		v1x /= mag;
		v1y /= mag;
		//�ڶ�����������
		Mat v2x = -v1y;
		Mat v2y = v1x;
		// Compute eigenvalues
		Mat Lambda1 = 0.5 * (Dxx + Dyy + tmp);
		Mat Lambda2 = 0.5 * (Dxx + Dyy - tmp);
		// Sort eigenvalues by absolute value
		Mat check = abs(Lambda1) < abs(Lambda2);
		Mat Ix = v1x.clone();
		Mat Iy = v1y.clone();
		v2x.copyTo(Ix, check);
		v2y.copyTo(Iy, check);
		Mat t = -(Dx.mul(Ix) + Dy.mul(Iy)) / (Dxx.mul(Ix.mul(Ix)) + 2.0 * Dxy.mul(Ix.mul(Iy)) + Dyy.mul(Iy.mul(Iy)));
		Mat px = t.mul(Ix);
		Mat py = t.mul(Iy);
		vector<Point2d> laserPoints;
		vector<Point2i> candidatePoints;
		//��ȡ��ʼ��Steger�����������ϵĵ�
		for (int y = 0; y < px.rows; ++y) {
			for (int x = 0; x < px.cols; ++x) {
				if (roiImg.at<double>(y, x) <= sltThr)
					continue;
				else if (abs(px.at<double>(y, x)) <= 0.5 && abs(py.at<double>(y, x)) <= 0.5) {
					//else if (abs(px.at<double>(y, x)) <= 0.3 && abs(py.at<double>(y, x)) <= 0.3) {
					laserPoints.emplace_back(x + px.at<double>(y, x) + roi[2], y + py.at<double>(y, x) + roi[0]);
					candidatePoints.emplace_back(x, y);
					//cout << x + px.at<double>(y, x) << "\t" << y + py.at<double>(y, x) << endl;
				}
			}
		}
		if (laserPoints.size() < 10) {
			continue;
		}


		//�Ե����ɸѡ
		vector<Point2d> laserPoints_filtered;  // �洢ɸѡ��ĵ�
		if (filter) {
			//1. ����ͳ�Ƹ���Y������ͬ�ĸ���
			vector<int> laserYNum;
			int ind = 0;
			int num = 1;
			while (ind < candidatePoints.size() - 1) {
				int y = candidatePoints[ind].y;
				if (candidatePoints[ind + 1].y == y) {
					num++;
				}
				else {
					laserYNum.push_back(num);
					num = 1;
				}
				ind++;
			}
			cout << "laserXNum.size():" << laserYNum.size() << endl;
			//2. ��ʼ����ǿ������index
			vector<pair<int, int>> num_index;
			int index = 0;
			for (int i = 0; i < laserYNum.size(); i++) {
				if (laserYNum[i] > 1) {
					num_index.push_back(make_pair(laserYNum[i], -1));
				}
				else {
					num_index.push_back(make_pair(laserYNum[i], index));
				}
				index += laserYNum[i];
			}
			//3. ��ʼ����Ҫ�����ĵ�
			int expNum = 5;
			for (int m = 0; m < expNum; m++) {
				index = 0;
				for (int i = 0; i < laserYNum.size(); i++) {
					if (num_index[i].second > 0 || i == 0 || i == laserYNum.size() - 1) { // ǿ�����ĵ��Լ����˵ĵ�Ͳ����ټ�����
						index += num_index[i].first;
						continue;
					}
					else { // ��Ҫ����ĵ�
						if (num_index[i - 1].second < 0 && num_index[i + 1].second < 0) { //���߶�û�б���
							index += num_index[i].first;
							continue;
						}
						else {
							vector<Point2i> temp;
							temp.insert(temp.end(), candidatePoints.begin() + index, candidatePoints.begin() + index + num_index[i].first);
							int minIndex = index;
							double minDis = 1000;
							if (num_index[i - 1].second > 0 && num_index[i + 1].second > 0) //���߶�������
							{
								for (int j = 0; j < num_index[i].first; j++) {
									double dis = abs(candidatePoints[index + j].x - candidatePoints[index - 1].x) + abs(candidatePoints[index + j].x - candidatePoints[index + num_index[i].first].x);
									if (dis < minDis) {
										minDis = dis;
										minIndex = index + j;
									}
								}
							}
							else if (num_index[i - 1].second > 0 && num_index[i + 1].second < 0) //��߱�����
							{
								for (int j = 0; j < num_index[i].first; j++) {
									double dis = abs(candidatePoints[index + j].x - candidatePoints[index - 1].x);
									if (dis < minDis) {
										minDis = dis;
										minIndex = index + j;
									}
								}

							}
							else if (num_index[i - 1].second < 0 && num_index[i + 1].second > 0) //�ұ߱�����
							{
								for (int j = 0; j < num_index[i].first; j++) {
									double dis = abs(candidatePoints[index + j].x - candidatePoints[index + num_index[i].first].x);
									if (dis < minDis) {
										minDis = dis;
										minIndex = index + j;
									}
								}

							}
							num_index[i].second = minIndex;
							index += num_index[i].first;
						}
					}
				}
			}
			//4. ����ɸѡ��ĵ�
			for (int i = 0; i < num_index.size(); i++) {
				if (num_index[i].second < 0 || visited_Y.ptr<uchar>(0)[candidatePoints[num_index[i].second].y + roi[0]] == 1)
				{
					continue;
				}
				visited_Y.ptr<uchar>(0)[candidatePoints[num_index[i].second].y + roi[0]] = 1;
				laserPoints_filtered.push_back(Point2d(laserPoints[num_index[i].second].x, laserPoints[num_index[i].second].y));
				//cout << laserPoints[num_index[i].second].x << "\t" << laserPoints[num_index[i].second].y << endl;
			}
		}
		else
		{
			laserPoints_filtered = laserPoints;
		}
		linePoints.insert(linePoints.end(), laserPoints_filtered.begin(), laserPoints_filtered.end());
	}
	return linePoints;
}


std::vector<cv::Point2d> findLaserCenterGrayCenter_heng(const cv::Mat& inputImage, double bwThr, double sumThr) {
	Mat proImg = inputImage.clone();
	if (inputImage.channels() > 1)
		cvtColor(proImg, proImg, COLOR_BGR2GRAY);
	Mat bwImg;
	vector<Vec4i> ranges;
	int thre = bwThr;
	Mat dstImg;
	//二值化阈值
	imgPreProcess_simple(proImg, thre, dstImg, ranges);
	vector<Point2d> linePoints;
	for (int k = 0; k < ranges.size(); k++)
	{
		for (int j = ranges[k][2]; j < ranges[k][3]; j++)
		{
			double sum_gv = 0;
			double sum_gv_pos = 0;
			double thresh = 30;//30,10,0
			for (int i = ranges[k][0]; i < ranges[k][1]; i++)
			{
				double tempV = dstImg.at<uchar>(i, j);
				if (tempV > thresh)
				{
					sum_gv += tempV;
					sum_gv_pos += tempV * i;
				}
				if (j >= ranges[k][2] + 2 && j < ranges[k][3] - 2) {
					sum_gv += dstImg.at<uchar>(i, j - 2) + dstImg.at<uchar>(i, j - 1) + dstImg.at<uchar>(i, j + 1) + dstImg.at<uchar>(i, j + 2);
					sum_gv_pos += (dstImg.at<uchar>(i, j - 2) + dstImg.at<uchar>(i, j - 1) + dstImg.at<uchar>(i, j + 1) + dstImg.at<uchar>(i, j + 2)) * i;
				}
			}
			if (sum_gv < sumThr)
				continue;
			double y = sum_gv_pos / sum_gv;
			linePoints.push_back(Point2f(j, y));
		}
	}
	return linePoints;
}

std::vector<cv::Point2d> findLaserCenterGrayCenter_shu(const cv::Mat& inputImage, double bwThr) {
	Mat proImg = inputImage.clone();
	if (inputImage.channels() > 1)
		cvtColor(proImg, proImg, COLOR_BGR2GRAY);
	Mat bwImg;
	vector<Vec4i> ranges;
	int thre = bwThr;
	Mat dstImg;
	//imgPreProcess(img, thre, dstImg, ranges);
	imgPreProcess_simple(proImg, thre, dstImg, ranges);
	//��ȡ���ĵ�
	vector<Point2d> linePoints;
	for (int k = 0; k < ranges.size(); k++)
	{
		for (int i = ranges[k][0]; i < ranges[k][1]; i++)
		{
			double sum_gv = 0;
			double sum_gv_pos = 0;
			double thresh = 30;//30,10,0
			for (int j = ranges[k][2]; j < ranges[k][3]; j++)
			{
				double tempV = dstImg.at<uchar>(i, j);
				if (tempV > thresh)
				{
					sum_gv += tempV;
					sum_gv_pos += tempV * j;
				}
				if (i >= ranges[k][0] + 2 && i < ranges[k][1] - 2) {
					sum_gv += dstImg.at<uchar>(i - 2, j) + dstImg.at<uchar>(i - 1, j) + dstImg.at<uchar>(i + 1, j) + dstImg.at<uchar>(i + 2, j);
					sum_gv_pos += (dstImg.at<uchar>(i - 2, j) + dstImg.at<uchar>(i - 1, j) + dstImg.at<uchar>(i + 1, j) + dstImg.at<uchar>(i + 2, j)) * j;
				}
			}
			if (sum_gv < 100)
				continue;
			double x = sum_gv_pos / sum_gv;
			linePoints.push_back(Point2d(x, i));
		}
	}
	////����������
	//Mat drawLaserCenterPtsImg;
	//cvtColor(img, drawLaserCenterPtsImg, COLOR_GRAY2BGR);
	//for (int i = 0; i < linePoints.size(); i++) {
	//	cv::circle(drawLaserCenterPtsImg, Point2f(linePoints[i].x, linePoints[i].y), 0.5, Scalar(0, 0, 255));
	//}
	//namedWindow("centerLine", WINDOW_NORMAL);
	//imshow("centerLine", drawLaserCenterPtsImg);
	//waitKey(0);
	return linePoints;
}

void GetImageRanges(const cv::Mat& bwImg, vector<Vec4i>& dstRanges, double areaThr, int borderexten) {
	//�ҳ�������Ե
	std::vector<std::vector<cv::Point>> contours;
	std::vector<std::vector<cv::Point>> contoursOr;
	std::vector<std::vector<cv::Point>>::iterator it;
	std::vector<cv::Vec4i>hierarchy;
	cv::findContours(bwImg, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
	contoursOr = contours;
	//vector<vector<Point2i>> finalContourSet;
	Mat Temp = cv::Mat::zeros(bwImg.rows, bwImg.cols, bwImg.type());
	vector<Vec4i> ranges;
	for (int i = 0; i < contours.size(); i++)
	{
		double area = contourArea(contours[i]);
		if (area > areaThr)//���С��areaThr�Ͳ�Ҫ
		{
			//cout << "area:  " << area << endl;
			//finalContourSet.push_back(contours[i]);
			Point2i leftP = *min_element(contours[i].begin(), contours[i].end(), [&](Point2i a, Point2i b) {return a.x < b.x; });
			Point2i RightP = *max_element(contours[i].begin(), contours[i].end(), [&](Point2i a, Point2i b) {return a.x < b.x; });
			Point2i topP = *min_element(contours[i].begin(), contours[i].end(), [&](Point2i a, Point2i b) {return a.y < b.y; });
			Point2i botP = *max_element(contours[i].begin(), contours[i].end(), [&](Point2i a, Point2i b) {return a.y < b.y; });
			//cout << "leftP: " << leftP << endl;
			//cout << "RightP: " << RightP << endl;
			//cout << "topP: " << topP << endl;
			//cout << "botP: " << botP << endl;
			int left = leftP.x;
			int right = RightP.x;
			int top = topP.y;
			int bot = botP.y;
			//�߽���չ
			if (left - borderexten < 0)
				left = 0;
			else
				left -= borderexten;
			if (top - borderexten < 0)
				top = 0;
			else
				top -= borderexten;
			if (right + borderexten > bwImg.cols - 1)
				right = bwImg.cols - 1;
			else
				right += borderexten;
			if (bot + borderexten > bwImg.rows - 1)
				bot = bwImg.rows - 1;
			else
				bot += borderexten;

			//sort(contours[i].begin(), contours[i].end(), [&](Point2i a, Point2i b) {return a.x < b.x; });
			//int left = contours[i][0].x;
			//int right = contours[i][contours[i].size() - 1].x;
			//sort(contours[i].begin(), contours[i].end(), [&](Point2i a, Point2i b) {return a.y < b.y; });
			//int top = contours[i][0].y;
			//int bot = contours[i][contours[i].size() - 1].y;

			//if (bot - top < (right - left) * 1.0)
			//	continue;
			//cv::drawContours(Temp, contoursOr, i, cv::Scalar(255), cv::FILLED);
			//namedWindow("Temp", WINDOW_NORMAL);
			//imshow("Temp", Temp);
			//waitKey(0);
			ranges.push_back(Vec4i(top, bot, left, right));

		}
	}
	dstRanges = ranges;
}

void GetImageRanges_v2(const cv::Mat& bwImg, vector<Vec4i>& dstRanges, double areaThr, int borderexten) {
	//�ҳ�������Ե
	std::vector<std::vector<cv::Point>> contours;
	std::vector<std::vector<cv::Point>> contoursOr;
	std::vector<std::vector<cv::Point>>::iterator it;
	std::vector<cv::Vec4i>hierarchy;
	cv::findContours(bwImg, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
	contoursOr = contours;
	//vector<vector<Point2i>> finalContourSet;
	Mat Temp = cv::Mat::zeros(bwImg.rows, bwImg.cols, bwImg.type());
	vector<Vec4i> ranges;
	vector<double> areas;
	for (int i = 0; i < contours.size(); i++)
	{
		double area = contourArea(contours[i]);
		if (area > areaThr)//���С��areaThr�Ͳ�Ҫ
		{
			areas.push_back(area);
			//cout << "area:  " << area << endl;
			//finalContourSet.push_back(contours[i]);
			Point2i leftP = *min_element(contours[i].begin(), contours[i].end(), [&](Point2i a, Point2i b) {return a.x < b.x; });
			Point2i RightP = *max_element(contours[i].begin(), contours[i].end(), [&](Point2i a, Point2i b) {return a.x < b.x; });
			Point2i topP = *min_element(contours[i].begin(), contours[i].end(), [&](Point2i a, Point2i b) {return a.y < b.y; });
			Point2i botP = *max_element(contours[i].begin(), contours[i].end(), [&](Point2i a, Point2i b) {return a.y < b.y; });
			//cout << "leftP: " << leftP << endl;
			//cout << "RightP: " << RightP << endl;
			//cout << "topP: " << topP << endl;
			//cout << "botP: " << botP << endl;
			int left = leftP.x;
			int right = RightP.x;
			int top = topP.y;
			int bot = botP.y;
			//�߽���չ
			if (left - borderexten < 0)
				left = 0;
			else
				left -= borderexten;
			if (top - borderexten < 0)
				top = 0;
			else
				top -= borderexten;
			if (right + borderexten > bwImg.cols - 1)
				right = bwImg.cols - 1;
			else
				right += borderexten;
			if (bot + borderexten > bwImg.rows - 1)
				bot = bwImg.rows - 1;
			else
				bot += borderexten;

			ranges.push_back(Vec4i(top, bot, left, right));
		}
	}
	//�����������
	vector<int> idx(areas.size());
	iota(idx.begin(), idx.end(), 0);
	sort(idx.begin(), idx.end(), [&](int i1, int i2) {return areas[i1] > areas[i2]; });
	vector<Vec4i> ranges_sorted;
	for (int i = 0; i < idx.size(); i++) {
		ranges_sorted.push_back(ranges[idx[i]]);
	}
	dstRanges = ranges_sorted;
}

std::vector<cv::Point2d> bruteLaserCenter_shu(const cv::Mat& inputImage, double bwThr) {
	Mat proImg = inputImage.clone();
	if (inputImage.channels() > 1)
		cvtColor(proImg, proImg, COLOR_BGR2GRAY);
	Mat bwImg;
	vector<Vec4i> ranges;
	int thre = bwThr;
	Mat dstImg;
	imgPreProcess_simple(proImg, thre, dstImg, ranges);
	//��ȡ���ĵ�
	vector<Point2d> linePoints;
	for (int k = 0; k < ranges.size(); k++)
	{
		for (int i = ranges[k][0]; i < ranges[k][1]; i++)
		{
			double sum_gv = 0;
			double sum_gv_pos = 0;
			double thresh = 0.6 * thre;//30,10,0
			for (int j = ranges[k][2]; j < ranges[k][3]; j++)
			{
				double tempV = dstImg.at<uchar>(i, j);
				if (tempV > thresh)
				{
					sum_gv += tempV;
					sum_gv_pos += tempV * j;
				}
				if (i >= ranges[k][0] + 2 && i < ranges[k][1] - 2) {
					sum_gv += dstImg.at<uchar>(i - 2, j) + dstImg.at<uchar>(i - 1, j) + dstImg.at<uchar>(i + 1, j) + dstImg.at<uchar>(i + 2, j);
					sum_gv_pos += (dstImg.at<uchar>(i - 2, j) + dstImg.at<uchar>(i - 1, j) + dstImg.at<uchar>(i + 1, j) + dstImg.at<uchar>(i + 2, j)) * j;
				}
			}
			if (sum_gv < 2 * thre)
				continue;
			double x = sum_gv_pos / sum_gv;
			linePoints.push_back(Point2d(x, i));
		}
	}
	return linePoints;
}

std::vector<cv::Point2d> bruteLaserCenter_heng(const cv::Mat& inputImage, double bwThr) {
	Mat proImg = inputImage.clone();
	if (inputImage.channels() > 1)
		cvtColor(proImg, proImg, COLOR_BGR2GRAY);
	Mat bwImg;
	vector<Vec4i> ranges;
	int thre = bwThr;
	Mat dstImg;
	imgPreProcess_simple(proImg, thre, dstImg, ranges);
	vector<Point2d> linePoints;
	for (int k = 0; k < ranges.size(); k++)
	{
		for (int j = ranges[k][2]; j < ranges[k][3]; j++)
		{
			double sum_gv = 0;
			double sum_gv_pos = 0;
			double thresh = 0.6 * thre;//30,10,0
			for (int i = ranges[k][0]; i < ranges[k][1]; i++)
			{
				double tempV = dstImg.at<uchar>(i, j);
				if (tempV > thresh)
				{
					sum_gv += tempV;
					sum_gv_pos += tempV * i;
				}
				if (j >= ranges[k][2] + 2 && j < ranges[k][3] - 2) {
					sum_gv += dstImg.at<uchar>(i, j - 2) + dstImg.at<uchar>(i, j - 1) + dstImg.at<uchar>(i, j + 1) + dstImg.at<uchar>(i, j + 2);
					sum_gv_pos += (dstImg.at<uchar>(i, j - 2) + dstImg.at<uchar>(i, j - 1) + dstImg.at<uchar>(i, j + 1) + dstImg.at<uchar>(i, j + 2)) * i;
				}
			}
			if (sum_gv < 2 * thre)
				continue;
			double y = sum_gv_pos / sum_gv;
			linePoints.push_back(Point2f(j, y));
		}
	}
	return linePoints;
}

std::vector<cv::Point2d> robustLaserCenter_shu(const cv::Mat& roi, double thresh) {
	std::vector<cv::Point2d> points;

	for (int y = 0; y < roi.rows; y++) {
		std::vector<double> row_vals(roi.cols);
		for (int x = 0; x < roi.cols; x++) {
			row_vals[x] = static_cast<double>(roi.at<uchar>(y, x));
		}

		const double sigma = 1.5;
		std::vector<double> smoothed_vals(roi.cols, 0);
		for (int x = 0; x < roi.cols; x++) {
			double sum = 0, weight_sum = 0;
			for (int k = -5; k <= 5; k++) {
				int idx = x + k;
				if (idx >= 0 && idx < roi.cols) {
					double weight = exp(-k * k / (2 * sigma * sigma));
					sum += row_vals[idx] * weight;
					weight_sum += weight;
				}
			}
			smoothed_vals[x] = (weight_sum > 0) ? sum / weight_sum : 0;
		}

		double max_val = 0;
		int max_x = -1;
		bool in_peak = false;
		int peak_start = -1;
		std::vector<std::pair<int, int>> peak_regions;

		for (int x = 1; x < roi.cols - 1; x++) {
			if (smoothed_vals[x] > smoothed_vals[x - 1] &&
				smoothed_vals[x] > smoothed_vals[x + 1] &&
				smoothed_vals[x] > thresh) {

				if (!in_peak) {
					peak_start = x;
					in_peak = true;
				}

				if (smoothed_vals[x] > max_val) {
					max_val = smoothed_vals[x];
					max_x = x;
				}
			}
			else if (in_peak && smoothed_vals[x] < thresh * 0.7) {
				if (max_x != -1) {
					peak_regions.push_back({ peak_start, x });
				}
				in_peak = false;
				max_val = 0;
				max_x = -1;
			}
		}

		// 处理最后一个峰值
		if (in_peak && max_x != -1) {
			peak_regions.push_back({ peak_start, roi.cols - 1 });
		}

		// 3. 选择最可能的激光区域（最大强度区域）
		if (peak_regions.empty()) continue;

		// 找到强度最大的峰值区域
		double best_peak_intensity = 0;
		int best_peak_index = -1;
		for (size_t i = 0; i < peak_regions.size(); i++) {
			double peak_intensity = 0;
			for (int x = peak_regions[i].first; x <= peak_regions[i].second; x++) {
				peak_intensity += smoothed_vals[x];
			}
			if (peak_intensity > best_peak_intensity) {
				best_peak_intensity = peak_intensity;
				best_peak_index = i;
			}
		}

		if (best_peak_index == -1) continue;

		double sum_weight = 0;
		double sum_x = 0;
		int start_x = peak_regions[best_peak_index].first;
		int end_x = peak_regions[best_peak_index].second;

		double peak_value = 0;
		int peak_x = start_x;
		for (int x = start_x; x <= end_x; x++) {
			if (smoothed_vals[x] > peak_value) {
				peak_value = smoothed_vals[x];
				peak_x = x;
			}
		}

		for (int x = max(0, peak_x - 5); x <= min(roi.cols - 1, peak_x + 5); x++) {
			double val = smoothed_vals[x];
			if (val > thresh * 0.5) {  // 低阈值确保弱信号
				// 高斯加权 - 距离中心越近权重越高
				double dist = abs(x - peak_x);
				double spatial_weight = exp(-dist * dist / 4.0);  // 较窄的权重分布
				double weight = val * spatial_weight;

				sum_weight += weight;
				sum_x += x * weight;
			}
		}

		// 有效点检查
		if (sum_weight > 0) {
			points.push_back(cv::Point2d(sum_x / sum_weight, y));
		}
	}
	return points;
}

std::vector<cv::Point2d> robustLaserCenter_heng(const cv::Mat& roi, double thresh) {
	std::vector<cv::Point2d> points;

	// 利用高斯滤波进行y方向处理
	for (int x = 0; x < roi.cols; x++) {
		std::vector<double> col_vals(roi.rows);
		for (int y = 0; y < roi.rows; y++) {
			col_vals[y] = static_cast<double>(roi.at<uchar>(y, x));
		}

		const double sigma = 1.5;
		std::vector<double> smoothed_vals(roi.rows, 0);
		for (int y = 0; y < roi.rows; y++) {
			double sum = 0, weight_sum = 0;
			for (int k = -5; k <= 5; k++) {
				int idx = y + k;
				if (idx >= 0 && idx < roi.rows) {
					double weight = exp(-k * k / (2 * sigma * sigma));
					sum += col_vals[idx] * weight;
					weight_sum += weight;
				}
			}
			smoothed_vals[y] = (weight_sum > 0) ? sum / weight_sum : 0;
		}

		// 2. 还是假设在y方向上遵从高斯分布
		double max_val = 0;
		int max_y = -1;
		bool in_peak = false;
		int peak_start = -1;
		std::vector<std::pair<int, int>> peak_regions;

		for (int y = 1; y < roi.rows - 1; y++) {
			// 寻找局部极大值
			if (smoothed_vals[y] > smoothed_vals[y - 1] &&
				smoothed_vals[y] > smoothed_vals[y + 1] &&
				smoothed_vals[y] > thresh) {

				if (!in_peak) {
					peak_start = y;
					in_peak = true;
				}

				if (smoothed_vals[y] > max_val) {
					max_val = smoothed_vals[y];
					max_y = y;
				}
			}
			// 结束峰值区域
			else if (in_peak && smoothed_vals[y] < thresh * 0.7) {
				if (max_y != -1) {
					peak_regions.push_back({ peak_start, y });
				}
				in_peak = false;
				max_val = 0;
				max_y = -1;
			}
		}

		// 处理最后一个峰值
		if (in_peak && max_y != -1) {
			peak_regions.push_back({ peak_start, roi.rows - 1 });
		}

		if (peak_regions.empty()) continue;
		// 找到强度最大的峰值区域
		double best_peak_intensity = 0;
		int best_peak_index = -1;
		for (size_t i = 0; i < peak_regions.size(); i++) {
			double peak_intensity = 0;
			for (int y = peak_regions[i].first; y <= peak_regions[i].second; y++) {
				peak_intensity += smoothed_vals[y];
			}
			if (peak_intensity > best_peak_intensity) {
				best_peak_intensity = peak_intensity;
				best_peak_index = i;
			}
		}

		if (best_peak_index == -1) continue;

		// 4. 在选定区域内计算高斯加权重心
		double sum_weight = 0;
		double sum_y = 0;
		int start_y = peak_regions[best_peak_index].first;
		int end_y = peak_regions[best_peak_index].second;

		// 找到峰值点作为中心参考
		double peak_value = 0;
		int peak_y = start_y;
		for (int y = start_y; y <= end_y; y++) {
			if (smoothed_vals[y] > peak_value) {
				peak_value = smoothed_vals[y];
				peak_y = y;
			}
		}

		// 高斯加权重心计算
		for (int y = max(0, peak_y - 5); y <= min(roi.rows - 1, peak_y + 5); y++) {
			double val = smoothed_vals[y];
			if (val > thresh * 0.5) {  // 低阈值确保弱信号
				// 高斯加权 - 距离中心越近权重越高
				double dist = abs(y - peak_y);
				double spatial_weight = exp(-dist * dist / 4.0);  // 较窄的权重分布
				double weight = val * spatial_weight;

				sum_weight += weight;
				sum_y += y * weight;
			}
		}

		// 有效点检查
		if (sum_weight > 0) {
			points.push_back(cv::Point2d(x, sum_y / sum_weight));
		}
	}
	return points;
}



std::pair<double, double> gauss_distribution(const Mat& img_gray) {
	double aver = 0.0;
	double sigma = 0.0;

	int rows = img_gray.rows;
	int cols = img_gray.cols;

	int numPixels = 0;
	double sumPixels = 0.0;

	// 计算均值
	for (int i = 0; i < rows; i++) {
		for (int j = 0; j < cols; j++) {
			if (img_gray.at<uchar>(i, j) > 0) {
				sumPixels += img_gray.at<uchar>(i, j);
				numPixels++;
			}
		}
	}

	if (numPixels > 0) {
		aver = sumPixels / numPixels;
	}
	else {
		// 避免除零，这里根据实际情况处理，比如返回 0 或抛出异常
		aver = 0.0;
	}

	double sumSqDiff = 0.0;

	// 计算标准差
	for (int i = 0; i < rows; i++) {
		for (int j = 0; j < cols; j++) {
			if (img_gray.at<uchar>(i, j) > 0) {
				double diff = img_gray.at<uchar>(i, j) - aver;
				sumSqDiff += diff * diff;
			}
		}
	}

	if (numPixels > 1) {
		sigma = sqrt(sumSqDiff / (numPixels - 1));
	}
	else {
		// 避免除零或除 1，这里根据实际情况处理
		sigma = 0.0;
	}

	return { aver, sigma };
}

std::vector<cv::Point2d> findLaserCenterMultiThres_shu(const cv::Mat& img, double max_val_thres, int neighbor_size, double pixel_thres) {
	// Step1. 以没有阈值的情况下粗提取中心线（竖向版本）
	std::vector<cv::Point2d> centers = bruteLaserCenter_shu(img, 0);
	std::vector<cv::Point2d> center_result(8096);

	std::vector<float> x, y;
	for (const auto& center : centers) {
		if (center.y < 0 || center.y >= img.cols || center.x <= 0 ||
			center.x >= img.rows - 1 || std::isnan(center.x) || std::isnan(center.y))
			continue;  // 这里有意让x不位于图像的最左和最右边界
		x.push_back(center.x);
		y.push_back(center.y);
	}
	if (x.empty()) return {};

	int begin_idx = y[0], end_idx = y[0], begin_i = 0, end_i = 0, count = 0;

	// 线性插值函数（沿竖向方向）
	std::function<float(float, int)> getGrayByLinearInterp = [&](float x_index, int y_index)->float {
		int x_index_left = std::floor(x_index);
		int x_index_right = x_index_left + 1;
		if (x_index_left < 0 || x_index_right >= img.rows) return 0;
		return img.at<uchar>(x_index_left, y_index) * (x_index_right - x_index) +
			img.at<uchar>(x_index_right, y_index) * (x_index - x_index_left);
		};

	float gray_max = getGrayByLinearInterp(x[0], begin_idx);
	float gray_min = gray_max;

	// 遍历所有粗提取的中心点，进行区域分割
	for (int i = 1; i < x.size() - 1; i++) {
		float gray_this = getGrayByLinearInterp(x[i], y[i]);
		gray_max = std::max(gray_this, gray_max);
		gray_min = std::min(gray_this, gray_min);

		// Step2. 按照某些条件分割区间（竖向版本判断条件）
		if (y[i] >= y[i - 1]
			&& std::abs(x[i + 1] - 2 * x[i] + x[i - 1]) / std::pow(y[i] - y[i - 1], 2) <= 10.00f
			&& std::abs((x[i] - x[i - 1]) / (y[i] - y[i - 1])) <= 5.00f
			&& cv::norm(cv::Point2d(y[i] - y[i - 1], x[i] - x[i - 1])) < 15.0
			)
			continue;

		// 不满足连续条件，开始分割区间
		end_idx = y[i - 1];
		end_i = i - 1;

		if (end_i - begin_i < 10) { // 区间的点太少，视作噪声
			begin_idx = y[i];
			begin_i = i;
			gray_max = gray_this;
			gray_min = gray_max;
			continue;
		}

		// Step 3. 分区 + 预处理（调整为竖向区域）
		int x_min = std::max(0, static_cast<int>(std::round(*std::min_element(x.begin() + begin_i, x.begin() + end_i + 1) - 10)));
		int x_max = std::min(static_cast<int>(std::round(*std::max_element(x.begin() + begin_i, x.begin() + end_i + 1) + 10)), img.rows - 1);
		cv::Mat img_clone = img(cv::Rect(x_min, begin_idx, x_max - x_min + 1, end_idx - begin_idx + 1));

		auto [aver, sig] = gauss_distribution(img_clone);
		double thresh = 0;

		// 根据高斯分布的情况设置实际的阈值（与原函数相同）
		if (sig < 15) {
			thresh = std::max(aver * 1.1, std::min(aver + sig * 0.3, 80.0));
		}
		else if (aver > 80) {
			thresh = std::min(aver * 0.8, 200.0);
		}
		else {
			thresh = std::max(aver * 1.2, std::min(aver + sig * 0.7, 100.0));
		}

		std::cout << "Begin_index:" << begin_idx << "  End_index: " << end_idx
			<< "  Thresh:" << thresh << " Aver:" << aver << " Sigma:" << sig;
		std::vector<cv::Point2d> centers_temp = robustLaserCenter_shu(img_clone, thresh);
		std::cout << " CenterPt:" << centers_temp.size() << std::endl;

		// 恢复坐标到原图
		for (auto& pt : centers_temp) {
			pt.x += x_min;
			pt.y += begin_idx;
		}

		// Step5. 将结果填入最终数组
		int length = centers_temp.size();
		if (length >= 10) {
			for (int j = 0; j < length; j++) {
				// 过滤掉太接近边缘的点
				if (centers_temp[j].x < 10 || centers_temp[j].x >= img.rows - 10 ||
					centers_temp[j].y < 10 || centers_temp[j].y >= img.cols - 10)
					continue;
				center_result[count++] = centers_temp[j];
			}
		}

		// 更新下一个区间的起点
		begin_idx = y[i];
		begin_i = i;
		gray_max = getGrayByLinearInterp(x[i], begin_idx);
		gray_min = gray_max;
	}

	// 处理最后一个区间
	if (x.size() - 1 - begin_i >= 10) {
		end_i = x.size() - 2;
		end_idx = y[end_i];
		int x_min = std::max(0, static_cast<int>(std::round(*std::min_element(x.begin() + begin_i, x.begin() + end_i + 1) - 10)));
		int x_max = std::min(static_cast<int>(std::round(*std::max_element(x.begin() + begin_i, x.begin() + end_i + 1) + 10)), img.rows - 1);
		cv::Mat img_clone = img(cv::Rect(x_min, begin_idx, x_max - x_min + 1, end_idx - begin_idx + 1));

		auto [aver, sig] = gauss_distribution(img_clone);
		double thresh = 0;

		if (sig < 15) {
			thresh = std::max(aver * 1.1, std::min(aver + sig * 0.3, 80.0));
		}
		else if (aver > 80) {
			thresh = std::min(aver * 0.8, 200.0);
		}
		else {
			thresh = std::max(aver * 1.2, std::min(aver + sig * 0.7, 100.0));
		}

		std::cout << "Begin_index:" << begin_idx << "  End_index: " << end_idx << "  Thresh:" << thresh << std::endl;
		std::vector<cv::Point2d> centers_temp = robustLaserCenter_shu(img_clone, thresh);
		for (auto& pt : centers_temp) {
			pt.x += x_min;
			pt.y += begin_idx;
		}

		int length = centers_temp.size();
		if (length >= 10) {
			for (int j = 0; j < length; j++) {
				if (centers_temp[j].x < 10 || centers_temp[j].x >= img.rows - 10 ||
					centers_temp[j].y < 10 || centers_temp[j].y >= img.cols - 10)
					continue;
				center_result[count++] = centers_temp[j];
			}
		}
	}

	// 返回有效结果
	return std::vector<cv::Point2d>(center_result.begin(), center_result.begin() + count);
}


std::vector<cv::Point2d> findLaserCenterMultiThres_heng(const cv::Mat& img, double max_val_thres, int neighbor_size, double pixel_thres) {
	//Step1. 以没有无阈值的情况下粗提取中心线
	vector<Point2d> centers = bruteLaserCenter_heng(img, 0);
	vector<cv::Point2d> center_result(8096);

	vector<float> x, y;
	for (const auto& center : centers) {
		if (center.x < 0 || center.x >= img.cols || center.y <= 0 || \
			center.y >= img.rows - 1 || isnan(center.x) || isnan(center.y)) continue;  //这里有意让y不位于图像的顶端和底端
		x.push_back(center.x);
		y.push_back(center.y);
	}
	int begin_idx = x[0], end_idx = x[0], begin_i = 0, end_i = 0, count = 0;

	std::function<float(int, float)> getGrayByLinearInterp = [&](int x_index, float y_index)->float {
		//作线性插值
		int y_index_bottom = floor(y_index);
		int y_index_upper = y_index_bottom + 1;
		return img.at<uchar>(y_index_bottom, x_index) * (y_index_upper - y_index) + img.at<uchar>(y_index_upper, x_index) * (y_index - y_index_bottom);
	};

	float gray_max = getGrayByLinearInterp(begin_idx, y[0]);  //针对粗提取的中心取单线性插值
	float gray_min = gray_max;

	for (int i = 1; i < x.size() - 1; i++) {
		float gray_this = getGrayByLinearInterp(x[i], y[i]);
		gray_max = max(gray_this, gray_max);
		gray_min = min(gray_this, gray_min);

		//Step2. 按照某些条件分割区间:
		//连续条件：
		if (x[i] >= x[i - 1] 
			&& abs(y[i + 1] - 2 * y[i] + y[i - 1]) / pow(x[i] - x[i - 1], 2) <= 10.00f
			&& abs((y[i] - y[i - 1]) / (x[i] - x[i - 1])) <= 5.00f
			//&& gray_max - gray_min <= 50  //像素限制实际效果不好
			&& norm(Point2d(x[i] - x[i - 1], y[i] - y[i - 1])) < 15.0 //相邻点的距离约束
			//&& (i == 1 || abs(atan2(y[i] - y[i - 1], x[i] - x[i- 1]) - atan2(y[i - 1] - y[i - 2], x[i - 1] - x[i - 2]) < 45.0 * CV_PI / 180))
			) continue;

		//不满足以上条件的话，就要开始分割区间了
		end_idx = x[i - 1];
		end_i = i - 1;
		
		if (end_i - begin_i < 10) { //区间的点太少，视作噪声，不值得重建
			begin_idx = x[i];
			begin_i = i;
			gray_max = gray_this;
			gray_min = gray_max;
			continue;
		}

		// Step 3. 分区 + 预处理
		int y_min = max(0, static_cast<int>(round(*min_element(y.begin() + begin_i, y.begin() + end_i + 1) - 10)));
		int y_max = min(static_cast<int>(round(*max_element(y.begin() + begin_i, y.begin() + end_i + 1) + 10)), img.rows - 1);
		Mat img_clone = img(Rect(begin_idx, y_min, end_idx - begin_idx + 1, y_max - y_min + 1));

		auto [aver, sig] = gauss_distribution(img_clone);
		double thresh = 0;

		//根据情况设置实际的阈值
		if (sig < 15) {
			thresh = max(aver * 1.1, min(aver + sig * 0.3, 80.0));
		}
		else if (aver > 80) {
			thresh = min(aver * 0.8, 200.0);
		}
		else {
			thresh = max(aver * 1.2, min(aver + sig * 0.7, 100.0));
		}

		std::cout << "Begin_index:" << begin_idx << "  End_index: " << end_idx << "  Thresh:" << thresh << " Aver:" << aver << " Sigma:" << sig;
		vector<Point2d> centers_temp = robustLaserCenter_heng(img_clone, thresh); std::cout << " CenterPt:" << centers_temp.size() << endl;
		for (auto& pt : centers_temp) {
			pt.x += begin_idx;
			pt.y += y_min;
		}

		// Step 4. 单分区内 B 样条光顺，暂时不实现
		
		// Step5. 将center_temp中的结果填入center_result中
		int length = centers_temp.size();
		if (length >= 10) {
			for (int j = 0; j < length; j++) {
				//太过接近画框的边缘，不太合适
				if (centers_temp[j].x < 10 || centers_temp[j].x >= img.rows - 10 || centers_temp[j].y < 10 || centers_temp[j].y >= img.cols - 10) continue;
				center_result[count++] = centers_temp[j];
			}
		}

		begin_idx = x[i];
		begin_i = i;
		gray_max = getGrayByLinearInterp(begin_idx, y[i]);
		gray_min = gray_max;
	}

	//最后一个区间也需要处理：
	if (x.size() - 1 - begin_i >= 10){
		end_i = x.size() - 2;
		end_idx = x[end_i];
		int y_min = max(0, static_cast<int>(round(*min_element(y.begin() + begin_i, y.begin() + end_i + 1) - 10)));
		int y_max = min(static_cast<int>(round(*max_element(y.begin() + begin_i, y.begin() + end_i + 1) + 10)), img.rows - 1);
		Mat img_clone = img(Rect(begin_idx, y_min, end_idx - begin_idx + 1, y_max - y_min + 1));

		auto [aver, sig] = gauss_distribution(img_clone);
		double thresh = 0;

		if (sig < 15) {
			thresh = max(aver * 1.1, min(aver + sig * 0.3, 80.0));
		}
		else if (aver > 80) {
			thresh = min(aver * 0.8, 200.0);
		}
		else {
			thresh = max(aver * 1.2, min(aver + sig * 0.7, 100.0));
		}

		std::cout << "Begin_index:" << begin_idx << "  End_index: " << end_idx << "  Thresh:" << thresh << std::endl;
		vector<Point2d> centers_temp = robustLaserCenter_heng(img_clone, thresh);
		for (auto& pt : centers_temp) {
			pt.x += begin_idx;
			pt.y += y_min;
		}

		int length = centers_temp.size();
		if (length >= 10) {
			for (int j = 0; j < length; j++) {
				if (centers_temp[j].x < 10 || centers_temp[j].x >= img.rows - 10 || centers_temp[j].y < 10 || centers_temp[j].y >= img.cols - 10) continue;
				center_result[count++] = centers_temp[j];
			}
		}
	}
	std::vector<cv::Point2d> center_result_final(center_result.begin(), center_result.begin() + count);
	return center_result_final;
}
