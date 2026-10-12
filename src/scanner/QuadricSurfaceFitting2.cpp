#include "scanner/RuiQuadricSurfaceFitting.h"
#include <unsupported/Eigen/Polynomials>
#include <numeric>

using namespace Eigen;
using namespace std;

void Rui::GetOptimalQuadricSurfCoeffs(const std::vector<Eigen::Vector3f>& surfPts, Eigen::VectorXf& quaSurfCoeff, std::vector<double>& fitErrors, Eigen::Matrix4f& TransMatrix, double& maxZ, double& minZ) {

	MatrixXf Pts(surfPts.size(), 4);
	for (int i = 0; i < surfPts.size(); i++) {
		Pts(i, 0) = surfPts[i](0);
		Pts(i, 1) = surfPts[i](1);
		Pts(i, 2) = surfPts[i](2);
		Pts(i, 3) = 1;
	}
	//����任
	Eigen::MatrixXf surfPtsTrans;
	Eigen::Matrix4f TransMat;
	//PtsCoorTrans(Pts, surfPtsTrans, TransMat);
	PtsCoorTrans2(Pts, surfPtsTrans, TransMat);
	//cout << "TransMat:\n" << TransMat << endl;
	TransMatrix = TransMat;
	Eigen::VectorXf tempSurfCoeffs;
	//�ҵ�surfPtsTrans�е������Сzֵ
	maxZ = surfPtsTrans.col(2).maxCoeff();
	minZ = surfPtsTrans.col(2).minCoeff();
	vector<double> fitErrors_initialZ;
	//cout << "initial Pts.size:\n" << Pts.rows() << endl;
	//�����������
	QuadricSurfFitting(surfPtsTrans, tempSurfCoeffs, fitErrors_initialZ);

	//ϵ���任
	Eigen::VectorXf dstSurfCoeffs;
	QuadricCoeffsTrans(tempSurfCoeffs, Matrix4f::Identity(4, 4), dstSurfCoeffs);
	cout << "dstSurfCoeffs:\n" << dstSurfCoeffs << endl;
	//�Ż�ǰ��������
	vector<double> fitErrorsInitial;
	for (int i = 0; i < surfPtsTrans.rows(); i++) {
		Vector3f tempPt = Vector3f(surfPtsTrans(i, 0), surfPtsTrans(i, 1), surfPtsTrans(i, 2));
		double dis = (double)CostPtToQuadricSurface(tempPt, dstSurfCoeffs);
		//cout << dis << endl;
		fitErrorsInitial.push_back(abs(dis));
	}
	cout << "initial fitErrors: " << *max_element(fitErrorsInitial.begin(), fitErrorsInitial.end()) << " " << std::accumulate(fitErrorsInitial.begin(), fitErrorsInitial.end(), 0.0) / fitErrorsInitial.size() << endl;

	dstSurfCoeffs = dstSurfCoeffs / dstSurfCoeffs.norm() * 1e3;
	//�����Ż�
	//double x0[10] = { static_cast<double>(dstSurfCoeffs[0]), dstSurfCoeffs[1], dstSurfCoeffs[2], dstSurfCoeffs[3], dstSurfCoeffs[4],
	//				  dstSurfCoeffs[5], dstSurfCoeffs[6], dstSurfCoeffs[7], dstSurfCoeffs[8], dstSurfCoeffs[9]};
	double x0[10] = { static_cast<double>(dstSurfCoeffs[0]),
					  static_cast<double>(dstSurfCoeffs[1]),
					  static_cast<double>(dstSurfCoeffs[2]),
					  static_cast<double>(dstSurfCoeffs[3]),
					  static_cast<double>(dstSurfCoeffs[4]),
					  static_cast<double>(dstSurfCoeffs[5]),
					  static_cast<double>(dstSurfCoeffs[6]),
					  static_cast<double>(dstSurfCoeffs[7]),
					  static_cast<double>(dstSurfCoeffs[8]),
					  static_cast<double>(dstSurfCoeffs[9]) };
	ceres::Problem problem;
	for (int i = 0; i < surfPtsTrans.rows(); i++)
	{
		problem.AddResidualBlock(
			new ceres::NumericDiffCostFunction<CostPtToQuadricFunctor, ceres::RIDDERS, 1, 10>(
				new CostPtToQuadricFunctor(Vector3f(surfPtsTrans(i, 0), surfPtsTrans(i, 1), surfPtsTrans(i, 2)))
			),
			nullptr,
			x0
		);
	}
	//problem.AddResidualBlock(
	//	new ceres::NumericDiffCostFunction<CostQuadricCoeffsFunctor, ceres::RIDDERS, 1, 10>(
	//		new CostQuadricCoeffsFunctor()
	//	),
	//	nullptr,
	//	x0
	//);

	ceres::Solver::Options options;
	options.max_num_iterations = 1000;
	//options.function_tolerance = 1e-12;//1e-4
	//options.gradient_tolerance = 1e-12;
	options.linear_solver_type = ceres::DENSE_QR;
	options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
	//options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
	options.minimizer_progress_to_stdout = true;
	//options.max_trust_region_radius = 1e6;  // ������������Ĵ�С
	//options.min_trust_region_radius = 1e-12;  // ��С��������
	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);
	std::cout << summary.BriefReport() << "\n";

	//cout << "x" << endl;
	//for (int i = 0; i < 6; i++)
	//{
	//	cout << x0[i] << endl;
	//}
	Matrix<float, 10, 1> finalSurfCoeffs;
	finalSurfCoeffs << x0[0], x0[1], x0[2], x0[3], x0[4], x0[5], x0[6], x0[7], x0[8], x0[9];
	//�Ż����������
	vector<double> fitErrorsFinal;
	for (int i = 0; i < surfPtsTrans.rows(); i++) {
		Vector3f tempPt = Vector3f(surfPtsTrans(i, 0), surfPtsTrans(i, 1), surfPtsTrans(i, 2));
		double dis = (double)CostPtToQuadricSurface(tempPt, finalSurfCoeffs);
		fitErrorsFinal.push_back(abs(dis));
	}
	cout << "final fitErrors: " << *max_element(fitErrorsFinal.begin(), fitErrorsFinal.end()) << " " << std::accumulate(fitErrorsFinal.begin(), fitErrorsFinal.end(), 0.0) / fitErrorsFinal.size() << endl;
	cout << "final coeffs:\n" << finalSurfCoeffs << endl;
	////�Ż����������-2
	//finalSurfCoeffs = TempCoeffs;
	//vector<double> fitErrorsFinal2;
	//for (int i = 0; i < surfPtsTrans.rows(); i++) {
	//	Vector3f tempPt = surfPts[i];
	//	double dis = (double)CostPtToQuadricSurface(tempPt, finalSurfCoeffs);
	//	fitErrorsFinal2.push_back(abs(dis));
	//}
	//cout << "final fitErrors2: " << *max_element(fitErrorsFinal2.begin(), fitErrorsFinal2.end()) << " " << std::accumulate(fitErrorsFinal2.begin(), fitErrorsFinal2.end(), 0.0) / fitErrorsFinal2.size() << endl;
	//cout << "final coeffs2:\n" << finalSurfCoeffs << endl;
}

void Rui::GetQuadricSurfIntersectPt_localCal(Eigen::VectorXf quaSurfCoeffs, Eigen::VectorXf camRay, Eigen::Matrix4f TransMat, Eigen::Vector3f& intersectPt) {
	//����任
	Vector3f P0 = Vector3f(camRay(0), camRay(1), camRay(2));
	Vector3f v2 = Vector3f(camRay(3), camRay(4), camRay(5));
	P0 = TransMat.block(0, 0, 3, 3) * P0 + TransMat.block(0, 3, 3, 1);
	v2 = TransMat.block(0, 0, 3, 3) * v2;
	//cout << "P0:\t" << P0.transpose() << endl;
	//cout << "v2:\t" << v2.transpose() << endl;

	float k1 = v2(0) / v2(2);
	float k2 = v2(1) / v2(2);
	float b1 = P0(0) - k1 * P0(2);
	float b2 = P0(1) - k2 * P0(2);
	VectorXf c = VectorXf::Zero(7, 1);
	c << 0, quaSurfCoeffs;
	//cout << c << endl;
	float coeff2 = c(4) * b1 * b1 + c(5) * b1 * b2 + c(2) * b1 + c(6) * b2 * b2 + c(3) * b2 + c(1);
	float coeff1 = c(2) * k1 + c(3) * k2 + 2 * b1 * c(4) * k1 + b1 * c(5) * k2 + b2 * c(5) * k1 + 2 * b2 * c(6) * k2 - 1;
	float coeff0 = c(4) * k1 * k1 + c(5) * k1 * k2 + c(6) * k2 * k2;
	if (coeff2 < 0) {
		coeff0 = -coeff0;
		coeff1 = -coeff1;
		coeff2 = -coeff2;
	}
	//����η��̵ĸ�
	//float z1 = (-coeff1 - sqrt(coeff1 * coeff1 - 4 * coeff2 * coeff0)) / (2 * coeff2);
	//float z2 = (-coeff1 + sqrt(coeff1 * coeff1 - 4 * coeff2 * coeff0)) / (2 * coeff2);
	Eigen::PolynomialSolver<double, Eigen::Dynamic> solver; //����ʽ���
	Eigen::VectorXd coeff(3);
	coeff[0] = coeff0;
	coeff[1] = coeff1;
	coeff[2] = coeff2;
	solver.compute(coeff);  //compute()�������ʽ������
	const Eigen::PolynomialSolver<double, Eigen::Dynamic>::RootsType& roots = solver.roots();
	vector<double> rootsReal;
	//cout << "roots.size: " << roots.size() << endl;
	//cout << "roots0: " << roots[0].real() << endl;;
	//cout << "roots1: " << roots[1].real() << endl;;
	float z1 = roots[0].real();
	float z2 = roots[1].real();
	float z = 0;
	if (abs(z1) < abs(z2))
		z = z1;
	else
		z = z2;
	//cout << z1 << '\t';
	//cout << z2 << endl;

	//if (z1 < 0)
	//	z1 = z2;
	//z = z2;
	//cout << coeff0 + coeff1 * z + coeff2 * z * z << endl;

	float x = k1 * z + b1;
	float y = k2 * z + b2;
	Vector3f dstPt = Vector3f(x, y, z);
	Matrix4f T = TransMat.inverse();
	dstPt = T.block(0, 0, 3, 3) * dstPt + T.block(0, 3, 3, 1);
	intersectPt = dstPt;
}

void Rui::QuarticSurfFittingAll_newForm(Eigen::MatrixXf ptsMat, Eigen::VectorXf& quarticSurfCoeffs, std::vector<double>& fitErrors, Eigen::Matrix4f& TransMatrix, double& maxZ, double& minZ) {
	//����任
	Eigen::MatrixXf surfPtsTrans;
	Eigen::Matrix4f TransMat;
	Rui::PtsCoorTrans2(ptsMat, surfPtsTrans, TransMat);
	TransMatrix = TransMat;
	Eigen::VectorXf tempSurfCoeffs;
	//�ҵ�surfPtsTrans�е������Сzֵ
	maxZ = surfPtsTrans.col(2).maxCoeff();
	minZ = surfPtsTrans.col(2).minCoeff();
	//cout << "initial Pts.size:\n" << Pts.rows() << endl;
	//�Ĵ��������
	QuarticSurfFitting(surfPtsTrans, tempSurfCoeffs, fitErrors);
	//ϵ���任
	Eigen::VectorXf dstSurfCoeffs;
	QuarticCoeffsTrans(tempSurfCoeffs, Matrix4f::Identity(4, 4), dstSurfCoeffs);
	dstSurfCoeffs.normalize();
	dstSurfCoeffs = dstSurfCoeffs * 1e3;
	quarticSurfCoeffs = dstSurfCoeffs;
}

void Rui::QuarticSurfFittingAll(Eigen::MatrixXf ptsMat, Eigen::VectorXf& quarticSurfCoeffs, std::vector<double>& fitErrors) {
	//����任
	Eigen::MatrixXf surfPtsTrans;
	Eigen::Matrix4f TransMat;
	Rui::PtsCoorTrans2(ptsMat, surfPtsTrans, TransMat);
	Eigen::VectorXf tempSurfCoeffs;
	//cout << "initial Pts.size:\n" << Pts.rows() << endl;
	//�Ĵ��������
	QuarticSurfFitting(surfPtsTrans, tempSurfCoeffs, fitErrors);
	//ϵ���任
	Eigen::VectorXf dstSurfCoeffs;
	QuarticCoeffsTrans(tempSurfCoeffs, TransMat, dstSurfCoeffs);
	dstSurfCoeffs.normalize();
	dstSurfCoeffs = dstSurfCoeffs * 1e3;
	quarticSurfCoeffs = dstSurfCoeffs;
	////���x����֤
	//for (int i = 0; i < ptsMat.rows(); i++) {
	//	double x = ptsMat(i, 0);
	//	double y = ptsMat(i, 1);
	//	double z = ptsMat(i, 2);
	//	double xsolve = 0;
	//	GetQuarticSurfXCoor(dstSurfCoeffs, x, y, z, xsolve);
	//}
}

void Rui::QuarticSurfFitting(Eigen::MatrixXf surfPts, Eigen::VectorXf& quarticSurfCoeffs, std::vector<double>& fitErrors) {
	Eigen::ArrayXf X_a = (surfPts.col(0)).array();
	Eigen::ArrayXf Y_a = (surfPts.col(1)).array();
	Eigen::ArrayXf Z_a = (surfPts.col(2)).array();
	Eigen::MatrixXf A = Eigen::MatrixXf::Ones(surfPts.rows(), 15);
	A.col(1) = X_a;
	A.col(2) = Y_a;
	A.col(3) = X_a * X_a;
	A.col(4) = X_a * Y_a;
	A.col(5) = Y_a * Y_a;
	A.col(6) = X_a * X_a * X_a;
	A.col(7) = X_a * X_a * Y_a;
	A.col(8) = X_a * Y_a * Y_a;
	A.col(9) = Y_a * Y_a * Y_a;
	A.col(10) = X_a * X_a * X_a * X_a;
	A.col(11) = X_a * X_a * X_a * Y_a;
	A.col(12) = X_a * X_a * Y_a * Y_a;
	A.col(13) = X_a * Y_a * Y_a * Y_a;
	A.col(14) = Y_a * Y_a * Y_a * Y_a;
	Eigen::MatrixXf B = Z_a;
	Eigen::MatrixXf X = (A.transpose() * A).inverse() * A.transpose() * B;
	quarticSurfCoeffs = X;
	//cout << "coeffs:\n " << X << endl;
	//����������
	Eigen::MatrixXf Z_f = A * X - B;
	//cout << "Z_f:\n " << Z_f << endl;
	std::vector<double> errors;
	for (int i = 0; i < Z_f.rows(); i++) {
		errors.push_back(abs(Z_f(i, 0)));
	}
	//cout << *max_element(errors.begin(), errors.end()) << endl;
	//cout << std::accumulate(errors.begin(), errors.end(), 0.0) / errors.size() << endl;
	fitErrors = errors;
}

void Rui::QuarticCoeffsTrans(Eigen::VectorXf quarticSurfCoeffs, Eigen::Matrix4f TransMat, Eigen::VectorXf& quarticSurfCoeffsTrans) {
	Eigen::MatrixXf T = TransMat;
	Eigen::VectorXf coeff = quarticSurfCoeffs;
	//cout << T << endl;
	Eigen::VectorXf c = coeff;
	float r11 = T(0, 0);
	float r12 = T(0, 1);
	float r13 = T(0, 2);
	float r21 = T(1, 0);
	float r22 = T(1, 1);
	float r23 = T(1, 2);
	float r31 = T(2, 0);
	float r32 = T(2, 1);
	float r33 = T(2, 2);
	float t1 = T(0, 3);
	float t2 = T(1, 3);
	float t3 = T(2, 3);
	//ϵ���任
	Eigen::VectorXf coeffQuartic = Eigen::VectorXf::Zero(35, 1);
	float a0 = c(10) * pow(r11, 4) + c(11) * pow(r11, 3) * r21 + c(12) * pow(r11, 2) * pow(r21, 2) + c(13) * r11 * pow(r21, 3) + c(14) * pow(r21, 4);
	float a1 = 4 * c(10) * pow(r11, 3) * r12 + c(11) * pow(r11, 3) * r22 + c(13) * r12 * pow(r21, 3) + 4 * c(14) * pow(r21, 3) * r22 + 3 * c(11) * pow(r11, 2) * r12 * r21 + 2 * c(12) * r11 * r12 * pow(r21, 2) + 2 * c(12) * pow(r11, 2) * r21 * r22 + 3 * c(13) * r11 * pow(r21, 2) * r22;
	float a2 = 6 * c(10) * pow(r11, 2) * pow(r12, 2) + 3 * c(11) * pow(r11, 2) * r12 * r22 + c(12) * pow(r11, 2) * pow(r22, 2) + 3 * c(11) * r11 * pow(r12, 2) * r21 + 4 * c(12) * r11 * r12 * r21 * r22 + 3 * c(13) * r11 * r21 * pow(r22, 2) + c(12) * pow(r12, 2) * pow(r21, 2) + 3 * c(13) * r12 * pow(r21, 2) * r22 + 6 * c(14) * pow(r21, 2) * pow(r22, 2);
	float a3 = 4 * c(10) * pow(r11, 3) * r13 + c(11) * pow(r11, 3) * r23 + c(13) * r13 * pow(r21, 3) + 4 * c(14) * pow(r21, 3) * r23 + 3 * c(11) * pow(r11, 2) * r13 * r21 + 2 * c(12) * r11 * r13 * pow(r21, 2) + 2 * c(12) * pow(r11, 2) * r21 * r23 + 3 * c(13) * r11 * pow(r21, 2) * r23;
	float a4 = 6 * c(10) * pow(r11, 2) * pow(r13, 2) + 3 * c(11) * pow(r11, 2) * r13 * r23 + c(12) * pow(r11, 2) * pow(r23, 2) + 3 * c(11) * r11 * pow(r13, 2) * r21 + 4 * c(12) * r11 * r13 * r21 * r23 + 3 * c(13) * r11 * r21 * pow(r23, 2) + c(12) * pow(r13, 2) * pow(r21, 2) + 3 * c(13) * r13 * pow(r21, 2) * r23 + 6 * c(14) * pow(r21, 2) * pow(r23, 2);
	float a5 = 12 * c(10) * pow(r11, 2) * r12 * r13 + 3 * c(11) * pow(r11, 2) * r12 * r23 + 3 * c(11) * pow(r11, 2) * r13 * r22 + 2 * c(12) * r12 * r13 * pow(r21, 2) + 2 * c(12) * pow(r11, 2) * r22 * r23 + 3 * c(13) * r12 * pow(r21, 2) * r23 + 3 * c(13) * r13 * pow(r21, 2) * r22 + 12 * c(14) * pow(r21, 2) * r22 * r23 + 6 * c(11) * r11 * r12 * r13 * r21 + 4 * c(12) * r11 * r12 * r21 * r23 + 4 * c(12) * r11 * r13 * r21 * r22 + 6 * c(13) * r11 * r21 * r22 * r23;
	float a6 = c(10) * pow(r12, 4) + c(11) * pow(r12, 3) * r22 + c(12) * pow(r12, 2) * pow(r22, 2) + c(13) * r12 * pow(r22, 3) + c(14) * pow(r22, 4);
	float a7 = 4 * c(10) * pow(r12, 3) * r13 + c(11) * pow(r12, 3) * r23 + c(13) * r13 * pow(r22, 3) + 4 * c(14) * pow(r22, 3) * r23 + 3 * c(11) * pow(r12, 2) * r13 * r22 + 2 * c(12) * r12 * r13 * pow(r22, 2) + 2 * c(12) * pow(r12, 2) * r22 * r23 + 3 * c(13) * r12 * pow(r22, 2) * r23;
	float a8 = 6 * c(10) * pow(r12, 2) * pow(r13, 2) + 3 * c(11) * pow(r12, 2) * r13 * r23 + c(12) * pow(r12, 2) * pow(r23, 2) + 3 * c(11) * r12 * pow(r13, 2) * r22 + 4 * c(12) * r12 * r13 * r22 * r23 + 3 * c(13) * r12 * r22 * pow(r23, 2) + c(12) * pow(r13, 2) * pow(r22, 2) + 3 * c(13) * r13 * pow(r22, 2) * r23 + 6 * c(14) * pow(r22, 2) * pow(r23, 2);
	float a9 = 4 * c(10) * r11 * pow(r12, 3) + c(11) * pow(r12, 3) * r21 + c(13) * r11 * pow(r22, 3) + 4 * c(14) * r21 * pow(r22, 3) + 3 * c(11) * r11 * pow(r12, 2) * r22 + 2 * c(12) * r11 * r12 * pow(r22, 2) + 2 * c(12) * pow(r12, 2) * r21 * r22 + 3 * c(13) * r12 * r21 * pow(r22, 2);
	float a10 = 12 * c(10) * r11 * pow(r12, 2) * r13 + 3 * c(11) * r11 * pow(r12, 2) * r23 + 3 * c(11) * pow(r12, 2) * r13 * r21 + 2 * c(12) * r11 * r13 * pow(r22, 2) + 2 * c(12) * pow(r12, 2) * r21 * r23 +
		3 * c(13) * r11 * pow(r22, 2) * r23 + 3 * c(13) * r13 * r21 * pow(r22, 2) + 12 * c(14) * r21 * pow(r22, 2) * r23 + 6 * c(11) * r11 * r12 * r13 * r22 + 4 * c(12) * r11 * r12 * r22 * r23 + 4 * c(12) * r12 * r13 * r21 * r22 + 6 * c(13) * r12 * r21 * r22 * r23;
	float a11 = c(10) * pow(r13, 4) + c(11) * pow(r13, 3) * r23 + c(12) * pow(r13, 2) * pow(r23, 2) + c(13) * r13 * pow(r23, 3) + c(14) * pow(r23, 4);
	float a12 = 4 * c(10) * r11 * pow(r13, 3) + c(11) * pow(r13, 3) * r21 + c(13) * r11 * pow(r23, 3) + 4 * c(14) * r21 * pow(r23, 3) + 3 * c(11) * r11 * pow(r13, 2) * r23 + 2 * c(12) * r11 * r13 * pow(r23, 2) + 2 * c(12) * pow(r13, 2) * r21 * r23 + 3 * c(13) * r13 * r21 * pow(r23, 2);
	float a13 = 4 * c(10) * r12 * pow(r13, 3) + c(11) * pow(r13, 3) * r22 + c(13) * r12 * pow(r23, 3) + 4 * c(14) * r22 * pow(r23, 3) + 3 * c(11) * r12 * pow(r13, 2) * r23 + 2 * c(12) * r12 * r13 * pow(r23, 2) + 2 * c(12) * pow(r13, 2) * r22 * r23 + 3 * c(13) * r13 * r22 * pow(r23, 2);
	float a14 = 12 * c(10) * r11 * r12 * pow(r13, 2) + 3 * c(11) * r11 * pow(r13, 2) * r22 + 3 * c(11) * r12 * pow(r13, 2) * r21 + 2 * c(12) * r11 * r12 * pow(r23, 2) + 2 * c(12) * pow(r13, 2) * r21 * r22 +
		3 * c(13) * r11 * r22 * pow(r23, 2) + 3 * c(13) * r12 * r21 * pow(r23, 2) + 12 * c(14) * r21 * r22 * pow(r23, 2) + 6 * c(11) * r11 * r12 * r13 * r23 + 4 * c(12) * r11 * r13 * r22 * r23 + 4 * c(12) * r12 * r13 * r21 * r23 + 6 * c(13) * r13 * r21 * r22 * r23;
	float a15 = c(6) * pow(r11, 3) + c(9) * pow(r21, 3) + c(7) * pow(r11, 2) * r21 + c(8) * r11 * pow(r21, 2) + 4 * c(10) * pow(r11, 3) * t1 + c(11) * pow(r11, 3) * t2 + c(13) * pow(r21, 3) * t1
		+ 4 * c(14) * pow(r21, 3) * t2 + 3 * c(11) * pow(r11, 2) * r21 * t1 + 2 * c(12) * r11 * pow(r21, 2) * t1 + 2 * c(12) * pow(r11, 2) * r21 * t2 + 3 * c(13) * r11 * pow(r21, 2) * t2;
	float a16 = 3 * c(6) * pow(r11, 2) * r12 + c(7) * pow(r11, 2) * r22 + c(8) * r12 * pow(r21, 2) + 3 * c(9) * pow(r21, 2) * r22 + 2 * c(7) * r11 * r12 * r21 + 2 * c(8) * r11 * r21 * r22 + 12 * c(10) * pow(r11, 2) * r12 * t1 + 3 * c(11) * pow(r11, 2) * r12 * t2 + 3 * c(11) * pow(r11, 2) * r22 * t1 + 2 * c(12) * r12 * pow(r21, 2) * t1 + 2 * c(12) * pow(r11, 2) * r22 * t2 + 3 * c(13) * r12 * pow(r21, 2) * t2 + 3 * c(13) * pow(r21, 2) * r22 * t1 + 12 * c(14) * pow(r21, 2) * r22 * t2 + 6 * c(11) * r11 * r12 * r21 * t1 + 4 * c(12) * r11 * r12 * r21 * t2 + 4 * c(12) * r11 * r21 * r22 * t1 + 6 * c(13) * r11 * r21 * r22 * t2;
	float a17 = 3 * c(6) * pow(r11, 2) * r13 + c(7) * pow(r11, 2) * r23 + c(8) * r13 * pow(r21, 2) + 3 * c(9) * pow(r21, 2) * r23 + 2 * c(7) * r11 * r13 * r21 + 2 * c(8) * r11 * r21 * r23 + 12 * c(10) * pow(r11, 2) * r13 * t1 + 3 * c(11) * pow(r11, 2) * r13 * t2 + 3 * c(11) * pow(r11, 2) * r23 * t1 + 2 * c(12) * r13 * pow(r21, 2) * t1 + 2 * c(12) * pow(r11, 2) * r23 * t2 + 3 * c(13) * r13 * pow(r21, 2) * t2 + 3 * c(13) * pow(r21, 2) * r23 * t1 + 12 * c(14) * pow(r21, 2) * r23 * t2 + 6 * c(11) * r11 * r13 * r21 * t1 + 4 * c(12) * r11 * r13 * r21 * t2 + 4 * c(12) * r11 * r21 * r23 * t1 + 6 * c(13) * r11 * r21 * r23 * t2;
	float a18 = c(6) * pow(r12, 3) + c(9) * pow(r22, 3) + c(7) * pow(r12, 2) * r22 + c(8) * r12 * pow(r22, 2) + 4 * c(10) * pow(r12, 3) * t1 + c(11) * pow(r12, 3) * t2 + c(13) * pow(r22, 3) * t1
		+ 4 * c(14) * pow(r22, 3) * t2 + 3 * c(11) * pow(r12, 2) * r22 * t1 + 2 * c(12) * r12 * pow(r22, 2) * t1 + 2 * c(12) * pow(r12, 2) * r22 * t2 + 3 * c(13) * r12 * pow(r22, 2) * t2;
	float a19 = 3 * c(6) * r11 * pow(r12, 2) + c(7) * pow(r12, 2) * r21 + c(8) * r11 * pow(r22, 2) + 3 * c(9) * r21 * pow(r22, 2) + 2 * c(7) * r11 * r12 * r22 + 2 * c(8) * r12 * r21 * r22 + 12 * c(10) * r11 * pow(r12, 2) * t1 + 3 * c(11) * r11 * pow(r12, 2) * t2 + 3 * c(11) * pow(r12, 2) * r21 * t1 + 2 * c(12) * r11 * pow(r22, 2) * t1 + 2 * c(12) * pow(r12, 2) * r21 * t2 + 3 * c(13) * r11 * pow(r22, 2) * t2 + 3 * c(13) * r21 * pow(r22, 2) * t1 + 12 * c(14) * r21 * pow(r22, 2) * t2 + 6 * c(11) * r11 * r12 * r22 * t1 + 4 * c(12) * r11 * r12 * r22 * t2 + 4 * c(12) * r12 * r21 * r22 * t1 + 6 * c(13) * r12 * r21 * r22 * t2;
	float a20 = 3 * c(6) * pow(r12, 2) * r13 + c(7) * pow(r12, 2) * r23 + c(8) * r13 * pow(r22, 2) + 3 * c(9) * pow(r22, 2) * r23 + 2 * c(7) * r12 * r13 * r22 + 2 * c(8) * r12 * r22 * r23 + 12 * c(10) * pow(r12, 2) * r13 * t1 + 3 * c(11) * pow(r12, 2) * r13 * t2 + 3 * c(11) * pow(r12, 2) * r23 * t1 + 2 * c(12) * r13 * pow(r22, 2) * t1 + 2 * c(12) * pow(r12, 2) * r23 * t2 + 3 * c(13) * r13 * pow(r22, 2) * t2 + 3 * c(13) * pow(r22, 2) * r23 * t1 + 12 * c(14) * pow(r22, 2) * r23 * t2 + 6 * c(11) * r12 * r13 * r22 * t1 + 4 * c(12) * r12 * r13 * r22 * t2 + 4 * c(12) * r12 * r22 * r23 * t1 + 6 * c(13) * r12 * r22 * r23 * t2;
	float a21 = c(6) * pow(r13, 3) + c(9) * pow(r23, 3) + c(7) * pow(r13, 2) * r23 + c(8) * r13 * pow(r23, 2) + 4 * c(10) * pow(r13, 3) * t1 + c(11) * pow(r13, 3) * t2 + c(13) * pow(r23, 3) * t1
		+ 4 * c(14) * pow(r23, 3) * t2 + 3 * c(11) * pow(r13, 2) * r23 * t1 + 2 * c(12) * r13 * pow(r23, 2) * t1 + 2 * c(12) * pow(r13, 2) * r23 * t2 + 3 * c(13) * r13 * pow(r23, 2) * t2;
	float a22 = 3 * c(6) * r11 * pow(r13, 2) + c(7) * pow(r13, 2) * r21 + c(8) * r11 * pow(r23, 2) + 3 * c(9) * r21 * pow(r23, 2) + 2 * c(7) * r11 * r13 * r23 + 2 * c(8) * r13 * r21 * r23 + 12 * c(10) * r11 * pow(r13, 2) * t1 + 3 * c(11) * r11 * pow(r13, 2) * t2 + 3 * c(11) * pow(r13, 2) * r21 * t1 + 2 * c(12) * r11 * pow(r23, 2) * t1 + 2 * c(12) * pow(r13, 2) * r21 * t2 + 3 * c(13) * r11 * pow(r23, 2) * t2 + 3 * c(13) * r21 * pow(r23, 2) * t1 + 12 * c(14) * r21 * pow(r23, 2) * t2 + 6 * c(11) * r11 * r13 * r23 * t1 + 4 * c(12) * r11 * r13 * r23 * t2 + 4 * c(12) * r13 * r21 * r23 * t1 + 6 * c(13) * r13 * r21 * r23 * t2;
	float a23 = 3 * c(6) * r12 * pow(r13, 2) + c(7) * pow(r13, 2) * r22 + c(8) * r12 * pow(r23, 2) + 3 * c(9) * r22 * pow(r23, 2) + 2 * c(7) * r12 * r13 * r23 + 2 * c(8) * r13 * r22 * r23 + 12 * c(10) * r12 * pow(r13, 2) * t1 + 3 * c(11) * r12 * pow(r13, 2) * t2 + 3 * c(11) * pow(r13, 2) * r22 * t1 + 2 * c(12) * r12 * pow(r23, 2) * t1 + 2 * c(12) * pow(r13, 2) * r22 * t2 + 3 * c(13) * r12 * pow(r23, 2) * t2 + 3 * c(13) * r22 * pow(r23, 2) * t1 + 12 * c(14) * r22 * pow(r23, 2) * t2 + 6 * c(11) * r12 * r13 * r23 * t1 + 4 * c(12) * r12 * r13 * r23 * t2 + 4 * c(12) * r13 * r22 * r23 * t1 + 6 * c(13) * r13 * r22 * r23 * t2;
	float a24 = 6 * c(6) * r11 * r12 * r13 + 2 * c(7) * r11 * r12 * r23 + 2 * c(7) * r11 * r13 * r22 + 2 * c(7) * r12 * r13 * r21 + 2 * c(8) * r11 * r22 * r23 + 2 * c(8) * r12 * r21 * r23 + 2 * c(8) * r13 * r21 * r22 + 6 * c(9) * r21 * r22 * r23 + 24 * c(10) * r11 * r12 * r13 * t1 + 6 * c(11) * r11 * r12 * r13 * t2 + 6 * c(11) * r11 * r12 * r23 * t1 + 6 * c(11) * r11 * r13 * r22 * t1 + 6 * c(11) * r12 * r13 * r21 * t1 + 4 * c(12) * r11 * r12 * r23 * t2 + 4 * c(12) * r11 * r13 * r22 * t2 + 4 * c(12) * r12 * r13 * r21 * t2 + 4 * c(12) * r11 * r22 * r23 * t1 + 4 * c(12) * r12 * r21 * r23 * t1 + 4 * c(12) * r13 * r21 * r22 * t1 + 6 * c(13) * r11 * r22 * r23 * t2 + 6 * c(13) * r12 * r21 * r23 * t2 + 6 * c(13) * r13 * r21 * r22 * t2 + 6 * c(13) * r21 * r22 * r23 * t1 + 24 * c(14) * r21 * r22 * r23 * t2;
	float a25 = 2 * c(3) * r11 * r12 + c(4) * r11 * r22 + c(4) * r12 * r21 + 2 * c(5) * r21 * r22 + 6 * c(6) * r11 * r12 * t1 + 2 * c(7) * r11 * r12 * t2 + 2 * c(7) * r11 * r22 * t1 + 2 * c(7) * r12 * r21 * t1 + 2 * c(8) * r11 * r22 * t2 + 2 * c(8) * r12 * r21 * t2 + 2 * c(8) * r21 * r22 * t1 + 6 * c(9) * r21 * r22 * t2 + 12 * c(10) * r11 * r12 * pow(t1, 2) + 2 * c(12) * r11 * r12 * pow(t2, 2) + 3 * c(11) * r11 * r22 * pow(t1, 2) + 3 * c(11) * r12 * r21 * pow(t1, 2) + 3 * c(13) * r11 * r22 * pow(t2, 2) + 3 * c(13) * r12 * r21 * pow(t2, 2) + 2 * c(12) * r21 * r22 * pow(t1, 2) + 12 * c(14) * r21 * r22 * pow(t2, 2) + 6 * c(11) * r11 * r12 * t1 * t2 + 4 * c(12) * r11 * r22 * t1 * t2 + 4 * c(12) * r12 * r21 * t1 * t2 + 6 * c(13) * r21 * r22 * t1 * t2;
	float a26 = 2 * c(3) * r12 * r13 + c(4) * r12 * r23 + c(4) * r13 * r22 + 2 * c(5) * r22 * r23 + 6 * c(6) * r12 * r13 * t1 + 2 * c(7) * r12 * r13 * t2 + 2 * c(7) * r12 * r23 * t1 + 2 * c(7) * r13 * r22 * t1 + 2 * c(8) * r12 * r23 * t2 + 2 * c(8) * r13 * r22 * t2 + 2 * c(8) * r22 * r23 * t1 + 6 * c(9) * r22 * r23 * t2 + 12 * c(10) * r12 * r13 * pow(t1, 2) + 2 * c(12) * r12 * r13 * pow(t2, 2) + 3 * c(11) * r12 * r23 * pow(t1, 2) + 3 * c(11) * r13 * r22 * pow(t1, 2) + 3 * c(13) * r12 * r23 * pow(t2, 2) + 3 * c(13) * r13 * r22 * pow(t2, 2) + 2 * c(12) * r22 * r23 * pow(t1, 2) + 12 * c(14) * r22 * r23 * pow(t2, 2) + 6 * c(11) * r12 * r13 * t1 * t2 + 4 * c(12) * r12 * r23 * t1 * t2 + 4 * c(12) * r13 * r22 * t1 * t2 + 6 * c(13) * r22 * r23 * t1 * t2;
	float a27 = 2 * c(3) * r11 * r13 + c(4) * r11 * r23 + c(4) * r13 * r21 + 2 * c(5) * r21 * r23 + 6 * c(6) * r11 * r13 * t1 + 2 * c(7) * r11 * r13 * t2 + 2 * c(7) * r11 * r23 * t1 + 2 * c(7) * r13 * r21 * t1 + 2 * c(8) * r11 * r23 * t2 + 2 * c(8) * r13 * r21 * t2 + 2 * c(8) * r21 * r23 * t1 + 6 * c(9) * r21 * r23 * t2 + 12 * c(10) * r11 * r13 * pow(t1, 2) + 2 * c(12) * r11 * r13 * pow(t2, 2) + 3 * c(11) * r11 * r23 * pow(t1, 2) + 3 * c(11) * r13 * r21 * pow(t1, 2) + 3 * c(13) * r11 * r23 * pow(t2, 2) + 3 * c(13) * r13 * r21 * pow(t2, 2) + 2 * c(12) * r21 * r23 * pow(t1, 2) + 12 * c(14) * r21 * r23 * pow(t2, 2) + 6 * c(11) * r11 * r13 * t1 * t2 + 4 * c(12) * r11 * r23 * t1 * t2 + 4 * c(12) * r13 * r21 * t1 * t2 + 6 * c(13) * r21 * r23 * t1 * t2;
	float a28 = 6 * c(10) * pow(r11, 2) * pow(t1, 2) + 3 * c(11) * pow(r11, 2) * t1 * t2 + 3 * c(6) * pow(r11, 2) * t1 + c(12) * pow(r11, 2) * pow(t2, 2) + c(7) * pow(r11, 2) * t2 + c(3) * pow(r11, 2) + 3 * c(11) * r11 * r21 * pow(t1, 2) + 4 * c(12) * r11 * r21 * t1 * t2 + 2 * c(7) * r11 * r21 * t1 + 3 * c(13) * r11 * r21 * pow(t2, 2) + 2 * c(8) * r11 * r21 * t2 + c(4) * r11 * r21 + c(12) * pow(r21, 2) * pow(t1, 2) + 3 * c(13) * pow(r21, 2) * t1 * t2 + c(8) * pow(r21, 2) * t1 + 6 * c(14) * pow(r21, 2) * pow(t2, 2) + 3 * c(9) * pow(r21, 2) * t2 + c(5) * pow(r21, 2);
	float a29 = 6 * c(10) * pow(r12, 2) * pow(t1, 2) + 3 * c(11) * pow(r12, 2) * t1 * t2 + 3 * c(6) * pow(r12, 2) * t1 + c(12) * pow(r12, 2) * pow(t2, 2) + c(7) * pow(r12, 2) * t2 + c(3) * pow(r12, 2) + 3 * c(11) * r12 * r22 * pow(t1, 2) + 4 * c(12) * r12 * r22 * t1 * t2 + 2 * c(7) * r12 * r22 * t1 + 3 * c(13) * r12 * r22 * pow(t2, 2) + 2 * c(8) * r12 * r22 * t2 + c(4) * r12 * r22 + c(12) * pow(r22, 2) * pow(t1, 2) + 3 * c(13) * pow(r22, 2) * t1 * t2 + c(8) * pow(r22, 2) * t1 + 6 * c(14) * pow(r22, 2) * pow(t2, 2) + 3 * c(9) * pow(r22, 2) * t2 + c(5) * pow(r22, 2);
	float a30 = 6 * c(10) * pow(r13, 2) * pow(t1, 2) + 3 * c(11) * pow(r13, 2) * t1 * t2 + 3 * c(6) * pow(r13, 2) * t1 + c(12) * pow(r13, 2) * pow(t2, 2) + c(7) * pow(r13, 2) * t2 + c(3) * pow(r13, 2) + 3 * c(11) * r13 * r23 * pow(t1, 2) + 4 * c(12) * r13 * r23 * t1 * t2 + 2 * c(7) * r13 * r23 * t1 + 3 * c(13) * r13 * r23 * pow(t2, 2) + 2 * c(8) * r13 * r23 * t2 + c(4) * r13 * r23 + c(12) * pow(r23, 2) * pow(t1, 2) + 3 * c(13) * pow(r23, 2) * t1 * t2 + c(8) * pow(r23, 2) * t1 + 6 * c(14) * pow(r23, 2) * pow(t2, 2) + 3 * c(9) * pow(r23, 2) * t2 + c(5) * pow(r23, 2);
	float a31 = c(1) * r11 - r31 + c(2) * r21 + 3 * c(6) * r11 * pow(t1, 2) + c(8) * r11 * pow(t2, 2) + 4 * c(10) * r11 * pow(t1, 3) + c(13) * r11 * pow(t2, 3) + c(7) * r21 * pow(t1, 2) + 3 * c(9) * r21 * pow(t2, 2) + c(11) * r21 * pow(t1, 3) + 4 * c(14) * r21 * pow(t2, 3) + 2 * c(3) * r11 * t1 + c(4) * r11 * t2 + c(4) * r21 * t1 + 2 * c(5) * r21 * t2 + 2 * c(7) * r11 * t1 * t2 + 2 * c(8) * r21 * t1 * t2
		+ 3 * c(11) * r11 * pow(t1, 2) * t2 + 2 * c(12) * r11 * t1 * pow(t2, 2) + 2 * c(12) * r21 * pow(t1, 2) * t2 + 3 * c(13) * r21 * t1 * pow(t2, 2);
	float a32 = c(1) * r12 - r32 + c(2) * r22 + 3 * c(6) * r12 * pow(t1, 2) + c(8) * r12 * pow(t2, 2) + 4 * c(10) * r12 * pow(t1, 3) + c(13) * r12 * pow(t2, 3) + c(7) * r22 * pow(t1, 2) + 3 * c(9) * r22 * pow(t2, 2) + c(11) * r22 * pow(t1, 3) + 4 * c(14) * r22 * pow(t2, 3) + 2 * c(3) * r12 * t1 + c(4) * r12 * t2 + c(4) * r22 * t1 + 2 * c(5) * r22 * t2 + 2 * c(7) * r12 * t1 * t2 + 2 * c(8) * r22 * t1 * t2
		+ 3 * c(11) * r12 * pow(t1, 2) * t2 + 2 * c(12) * r12 * t1 * pow(t2, 2) + 2 * c(12) * r22 * pow(t1, 2) * t2 + 3 * c(13) * r22 * t1 * pow(t2, 2);
	float a33 = c(1) * r13 - r33 + c(2) * r23 + 3 * c(6) * r13 * pow(t1, 2) + c(8) * r13 * pow(t2, 2) + 4 * c(10) * r13 * pow(t1, 3) + c(13) * r13 * pow(t2, 3) + c(7) * r23 * pow(t1, 2) + 3 * c(9) * r23 * pow(t2, 2) + c(11) * r23 * pow(t1, 3) + 4 * c(14) * r23 * pow(t2, 3) + 2 * c(3) * r13 * t1 + c(4) * r13 * t2 + c(4) * r23 * t1 + 2 * c(5) * r23 * t2 + 2 * c(7) * r13 * t1 * t2 + 2 * c(8) * r23 * t1 * t2
		+ 3 * c(11) * r13 * pow(t1, 2) * t2 + 2 * c(12) * r13 * t1 * pow(t2, 2) + 2 * c(12) * r23 * pow(t1, 2) * t2 + 3 * c(13) * r23 * t1 * pow(t2, 2);
	float a34 = c(0) - t3 + c(1) * t1 + c(2) * t2 + c(3) * pow(t1, 2) + c(5) * pow(t2, 2) + c(6) * pow(t1, 3) + c(9) * pow(t2, 3) + c(10) * pow(t1, 4) + c(14) * pow(t2, 4) + c(7) * pow(t1, 2) * t2 + c(8) * t1 * pow(t2, 2) + c(11) * pow(t1, 3) * t2 + c(13) * t1 * pow(t2, 3) + c(12) * pow(t1, 2) * pow(t2, 2) + c(4) * t1 * t2;
	coeffQuartic << a0, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34;
	//cout << "coeffQua:\n " << coeffQua << endl;
	quarticSurfCoeffsTrans = coeffQuartic;
}

void Rui::GetQuarticSurfXCoor(Eigen::VectorXf quarticSurfCoeffs, double x, double y, double z, double xslove) {
	Eigen::VectorXf c = quarticSurfCoeffs;
	float Y = y;
	float Z = z;
	float coeff4 = c(0);
	float coeff3 = c(15) + Y * c(1) + Z * c(3);
	float coeff2 = c(2) * pow(Y, 2) + c(5) * Y * Z + c(16) * Y + c(4) * pow(Z, 2) + c(17) * Z + c(28);
	float coeff1 = c(9) * pow(Y, 3) + c(10) * pow(Y, 2) * Z + c(19) * pow(Y, 2) + c(14) * Y * pow(Z, 2) + c(24) * Y * Z + c(25) * Y + c(12) * pow(Z, 3) + c(22) * pow(Z, 2) + c(27) * Z + c(31);
	float coeff0 = c(6) * pow(Y, 4) + c(7) * pow(Y, 3) * Z + c(18) * pow(Y, 3) + c(8) * pow(Y, 2) * pow(Z, 2) + c(20) * pow(Y, 2) * Z + c(29) * pow(Y, 2) + c(13) * Y * pow(Z, 3) + c(23) * Y * pow(Z, 2) + c(26) * Y * Z + c(32) * Y + c(11) * pow(Z, 4) + c(21) * pow(Z, 3) + c(30) * pow(Z, 2) + c(33) * Z + c(34);
	Eigen::PolynomialSolver<double, Eigen::Dynamic> solver; //����ʽ���
	Eigen::VectorXd coeff(5);
	coeff[0] = coeff0;
	coeff[1] = coeff1;
	coeff[2] = coeff2;
	coeff[3] = coeff3;
	coeff[4] = coeff4;
	solver.compute(coeff);  //compute()�������ʽ������
	const Eigen::PolynomialSolver<double, Eigen::Dynamic>::RootsType& roots = solver.roots();
	//cout << "roots: " << roots << endl;
	//cout << "realX:\t" << x << endl;
	double error = DBL_MAX;
	for (int i = 0; i < roots.size(); i++) {
		if (abs(roots[i].real() - x) < error) {
			error = abs(roots[i].real() - x);
		}
	}
	cout << "errorX:\t" << error << endl;
}

void Rui::GetQuarticSurfIntersectPt(Eigen::VectorXf quarticSurfCoeffs, Eigen::VectorXf camRay, Eigen::Vector3f& intersectPt) {
	float k1 = camRay(3) / camRay(5);
	float k2 = camRay(4) / camRay(5);
	float b1 = camRay(0) - k1 * camRay(2);
	float b2 = camRay(1) - k2 * camRay(2);
	VectorXf c = VectorXf::Zero(36, 1);
	c << 0, quarticSurfCoeffs;
	float coeff4 = c(1) * pow(k1, 4) + c(2) * pow(k1, 3) * k2 + c(4) * pow(k1, 3) + c(3) * pow(k1, 2) * pow(k2, 2) + c(6) * pow(k1, 2) * k2 + c(5) * pow(k1, 2) + c(10) * k1 * pow(k2, 3) + c(11) * k1 * pow(k2, 2) + c(15) * k1 * k2 + c(13) * k1 + c(7) * pow(k2, 4) + c(8) * pow(k2, 3) + c(9) * pow(k2, 2) + c(14) * k2 + c(12);
	float coeff3 = c(22) + b1 * c(13) + b2 * c(14) + c(23) * k1 + c(24) * k2 + c(16) * pow(k1, 3) + c(18) * pow(k1, 2) + c(19) * pow(k2, 3) + c(21) * pow(k2, 2) + 2 * b1 * c(5) * k1 + 2 * b2 * c(9) * k2 + b1 * c(15) * k2 + b2 * c(15) * k1 + c(25) * k1 * k2 + 4 * b1 * c(1) * pow(k1, 3) + 3 * b1 * c(4) * pow(k1, 2) + b2 * c(2) * pow(k1, 3) + b2 * c(6) * pow(k1, 2) + 4 * b2 * c(7) * pow(k2, 3) + 3 * b2 * c(8) * pow(k2, 2) + b1 * c(10) * pow(k2, 3) + b1 * c(11) * pow(k2, 2) + c(17) * pow(k1, 2) * k2 + c(20) * k1 * pow(k2, 2) + 3 * b1 * c(2) * pow(k1, 2) * k2 + 2 * b1 * c(3) * k1 * pow(k2, 2) + 2 * b2 * c(3) * pow(k1, 2) * k2 + 3 * b2 * c(10) * k1 * pow(k2, 2) + 2 * b1 * c(6) * k1 * k2 + 2 * b2 * c(11) * k1 * k2;
	float coeff2 = 6 * c(1) * pow(b1, 2) * pow(k1, 2) + 3 * c(2) * pow(b1, 2) * k1 * k2 + 3 * c(4) * pow(b1, 2) * k1 + c(3) * pow(b1, 2) * pow(k2, 2) + c(6) * pow(b1, 2) * k2 + c(5) * pow(b1, 2) + 3 * c(2) * b1 * b2 * pow(k1, 2) + 4 * c(3) * b1 * b2 * k1 * k2 + 2 * c(6) * b1 * b2 * k1 + 3 * c(10) * b1 * b2 * pow(k2, 2) + 2 * c(11) * b1 * b2 * k2 + c(15) * b1 * b2 + 3 * c(16) * b1 * pow(k1, 2) + 2 * c(17) * b1 * k1 * k2 + 2 * c(18) * b1 * k1 + c(20) * b1 * pow(k2, 2) + c(25) * b1 * k2 + c(23) * b1 + c(3) * pow(b2, 2) * pow(k1, 2) + 3 * c(10) * pow(b2, 2) * k1 * k2 + c(11) * pow(b2, 2) * k1 + 6 * c(7) * pow(b2, 2) * pow(k2, 2) + 3 * c(8) * pow(b2, 2) * k2 + c(9) * pow(b2, 2) + c(17) * b2 * pow(k1, 2) + 2 * c(20) * b2 * k1 * k2 + c(25) * b2 * k1 + 3 * c(19) * b2 * pow(k2, 2) + 2 * c(21) * b2 * k2 + c(24) * b2 + c(29) * pow(k1, 2) + c(26) * k1 * k2 + c(28) * k1 + c(30) * pow(k2, 2) + c(27) * k2 + c(31);
	float coeff1 = c(34) + b1 * c(28) + b2 * c(27) + c(32) * k1 + c(33) * k2 + pow(b1, 3) * c(4) + pow(b2, 3) * c(8) + pow(b1, 2) * c(18) + pow(b2, 2) * c(21) + b1 * b2 * c(25) + b1 * c(26) * k2 + b2 * c(26) * k1 + 2 * b1 * c(29) * k1 + 2 * b2 * c(30) * k2 + pow(b1, 2) * b2 * c(6) + b1 * pow(b2, 2) * c(11) + 4 * pow(b1, 3) * c(1) * k1 + pow(b1, 3) * c(2) * k2 + 4 * pow(b2, 3) * c(7) * k2 + pow(b2, 3) * c(10) * k1 + 3 * pow(b1, 2) * c(16) * k1 + pow(b1, 2) * c(17) * k2 + 3 * pow(b2, 2) * c(19) * k2 + pow(b2, 2) * c(20) * k1 + 3 * pow(b1, 2) * b2 * c(2) * k1 + 2 * b1 * pow(b2, 2) * c(3) * k1 + 2 * pow(b1, 2) * b2 * c(3) * k2 + 3 * b1 * pow(b2, 2) * c(10) * k2 + 2 * b1 * b2 * c(17) * k1 + 2 * b1 * b2 * c(20) * k2;
	float coeff0 = c(1) * pow(b1, 4) + c(2) * pow(b1, 3) * b2 + c(16) * pow(b1, 3) + c(3) * pow(b1, 2) * pow(b2, 2) + c(17) * pow(b1, 2) * b2 + c(29) * pow(b1, 2) + c(10) * b1 * pow(b2, 3) + c(20) * b1 * pow(b2, 2) + c(26) * b1 * b2 + c(32) * b1 + c(7) * pow(b2, 4) + c(19) * pow(b2, 3) + c(30) * pow(b2, 2) + c(33) * b2 + c(35);
	Eigen::PolynomialSolver<double, Eigen::Dynamic> solver; //����ʽ���
	Eigen::VectorXd coeff(5);
	coeff[0] = coeff0;
	coeff[1] = coeff1;
	coeff[2] = coeff2;
	coeff[3] = coeff3;
	coeff[4] = coeff4;
	solver.compute(coeff);  //compute()�������ʽ������
	const Eigen::PolynomialSolver<double, Eigen::Dynamic>::RootsType& roots = solver.roots();
	//cout << "roots: " << roots << endl;
	vector<float> zsolve;
	for (int i = 0; i < 4; i++) {
		if (abs(roots[i].imag()) < 1e-4) {
			zsolve.push_back(roots[i].real());
		}
	}
	if (zsolve.size() == 0) {
		//cout << "No real roots!" << endl;
		//cout << roots << endl;
		float tempZ = 0;
		float tempError = FLT_MAX;
		for (int i = 0; i < 4; i++) {
			if (abs(roots[i].imag()) < tempError) {
				tempZ = roots[i].real();
				tempError = abs(roots[i].imag());
			}
		}
		zsolve.push_back(tempZ);
		//cout << "zsolve only one: " << zsolve[0] << endl;
	}
	else if (zsolve.size() == 2) {
		if (zsolve[0] < zsolve[1] && zsolve[0] < 8e2 && zsolve[1] < 3e3) {
			//����
			float temp = zsolve[0];
			zsolve[0] = zsolve[1];
			zsolve[1] = temp;
		}
		else if (zsolve[0] > 3e3 && zsolve[1] < 3e3)
		{
			zsolve[0] = zsolve[1];
		}
		else if (zsolve[0] > 8e2 && zsolve[1] > 8e2 && zsolve[0] > zsolve[1]) {
			zsolve[0] = zsolve[1];
		}

		//cout << "zsolve: " << zsolve[0] << "\t" << zsolve[1] << endl;
	}
	float z = zsolve[0];
	//float z = roots[3].real();
	float x = k1 * z + b1;
	float y = k2 * z + b2;
	Vector3f dstPt = Vector3f(x, y, z);
	intersectPt = dstPt;
	//cout << "intersectPt:\t" << intersectPt.transpose() << endl;
}

void Rui::GetQuarticSurfIntersectPtNewForm(Eigen::VectorXf quarticSurfCoeffs, Eigen::VectorXf camRay, Eigen::Vector3f& intersectPt, double maxZ, double minZ) {
	//cout << "camRay:\t" << camRay.transpose() << endl;
	//cout << quarticSurfCoeffs << endl;
	float k1 = camRay(3) / camRay(5);
	float k2 = camRay(4) / camRay(5);
	float b1 = camRay(0) - k1 * camRay(2);
	float b2 = camRay(1) - k2 * camRay(2);
	VectorXf c = VectorXf::Zero(36, 1);
	c << 0, quarticSurfCoeffs;
	float coeff4 = c(1) * pow(k1, 4) + c(2) * pow(k1, 3) * k2 + c(4) * pow(k1, 3) + c(3) * pow(k1, 2) * pow(k2, 2) + c(6) * pow(k1, 2) * k2 + c(5) * pow(k1, 2) + c(10) * k1 * pow(k2, 3) + c(11) * k1 * pow(k2, 2) + c(15) * k1 * k2 + c(13) * k1 + c(7) * pow(k2, 4) + c(8) * pow(k2, 3) + c(9) * pow(k2, 2) + c(14) * k2 + c(12);
	float coeff3 = c(22) + b1 * c(13) + b2 * c(14) + c(23) * k1 + c(24) * k2 + c(16) * pow(k1, 3) + c(18) * pow(k1, 2) + c(19) * pow(k2, 3) + c(21) * pow(k2, 2) + 2 * b1 * c(5) * k1 + 2 * b2 * c(9) * k2 + b1 * c(15) * k2 + b2 * c(15) * k1 + c(25) * k1 * k2 + 4 * b1 * c(1) * pow(k1, 3) + 3 * b1 * c(4) * pow(k1, 2) + b2 * c(2) * pow(k1, 3) + b2 * c(6) * pow(k1, 2) + 4 * b2 * c(7) * pow(k2, 3) + 3 * b2 * c(8) * pow(k2, 2) + b1 * c(10) * pow(k2, 3) + b1 * c(11) * pow(k2, 2) + c(17) * pow(k1, 2) * k2 + c(20) * k1 * pow(k2, 2) + 3 * b1 * c(2) * pow(k1, 2) * k2 + 2 * b1 * c(3) * k1 * pow(k2, 2) + 2 * b2 * c(3) * pow(k1, 2) * k2 + 3 * b2 * c(10) * k1 * pow(k2, 2) + 2 * b1 * c(6) * k1 * k2 + 2 * b2 * c(11) * k1 * k2;
	float coeff2 = 6 * c(1) * pow(b1, 2) * pow(k1, 2) + 3 * c(2) * pow(b1, 2) * k1 * k2 + 3 * c(4) * pow(b1, 2) * k1 + c(3) * pow(b1, 2) * pow(k2, 2) + c(6) * pow(b1, 2) * k2 + c(5) * pow(b1, 2) + 3 * c(2) * b1 * b2 * pow(k1, 2) + 4 * c(3) * b1 * b2 * k1 * k2 + 2 * c(6) * b1 * b2 * k1 + 3 * c(10) * b1 * b2 * pow(k2, 2) + 2 * c(11) * b1 * b2 * k2 + c(15) * b1 * b2 + 3 * c(16) * b1 * pow(k1, 2) + 2 * c(17) * b1 * k1 * k2 + 2 * c(18) * b1 * k1 + c(20) * b1 * pow(k2, 2) + c(25) * b1 * k2 + c(23) * b1 + c(3) * pow(b2, 2) * pow(k1, 2) + 3 * c(10) * pow(b2, 2) * k1 * k2 + c(11) * pow(b2, 2) * k1 + 6 * c(7) * pow(b2, 2) * pow(k2, 2) + 3 * c(8) * pow(b2, 2) * k2 + c(9) * pow(b2, 2) + c(17) * b2 * pow(k1, 2) + 2 * c(20) * b2 * k1 * k2 + c(25) * b2 * k1 + 3 * c(19) * b2 * pow(k2, 2) + 2 * c(21) * b2 * k2 + c(24) * b2 + c(29) * pow(k1, 2) + c(26) * k1 * k2 + c(28) * k1 + c(30) * pow(k2, 2) + c(27) * k2 + c(31);
	float coeff1 = c(34) + b1 * c(28) + b2 * c(27) + c(32) * k1 + c(33) * k2 + pow(b1, 3) * c(4) + pow(b2, 3) * c(8) + pow(b1, 2) * c(18) + pow(b2, 2) * c(21) + b1 * b2 * c(25) + b1 * c(26) * k2 + b2 * c(26) * k1 + 2 * b1 * c(29) * k1 + 2 * b2 * c(30) * k2 + pow(b1, 2) * b2 * c(6) + b1 * pow(b2, 2) * c(11) + 4 * pow(b1, 3) * c(1) * k1 + pow(b1, 3) * c(2) * k2 + 4 * pow(b2, 3) * c(7) * k2 + pow(b2, 3) * c(10) * k1 + 3 * pow(b1, 2) * c(16) * k1 + pow(b1, 2) * c(17) * k2 + 3 * pow(b2, 2) * c(19) * k2 + pow(b2, 2) * c(20) * k1 + 3 * pow(b1, 2) * b2 * c(2) * k1 + 2 * b1 * pow(b2, 2) * c(3) * k1 + 2 * pow(b1, 2) * b2 * c(3) * k2 + 3 * b1 * pow(b2, 2) * c(10) * k2 + 2 * b1 * b2 * c(17) * k1 + 2 * b1 * b2 * c(20) * k2;
	float coeff0 = c(1) * pow(b1, 4) + c(2) * pow(b1, 3) * b2 + c(16) * pow(b1, 3) + c(3) * pow(b1, 2) * pow(b2, 2) + c(17) * pow(b1, 2) * b2 + c(29) * pow(b1, 2) + c(10) * b1 * pow(b2, 3) + c(20) * b1 * pow(b2, 2) + c(26) * b1 * b2 + c(32) * b1 + c(7) * pow(b2, 4) + c(19) * pow(b2, 3) + c(30) * pow(b2, 2) + c(33) * b2 + c(35);
	Eigen::PolynomialSolver<double, Eigen::Dynamic> solver; //����ʽ���
	Eigen::VectorXd coeff(5);
	coeff[0] = coeff0;
	coeff[1] = coeff1;
	coeff[2] = coeff2;
	coeff[3] = coeff3;
	coeff[4] = coeff4;
	solver.compute(coeff);  //compute()�������ʽ������
	const Eigen::PolynomialSolver<double, Eigen::Dynamic>::RootsType& roots = solver.roots();
	//cout << "roots: \n" << roots << endl;
	vector<float> zsolve;
	for (int i = 0; i < 4; i++) {
		if (abs(roots[i].imag()) < 1e-4) {
			zsolve.push_back(roots[i].real());
		}
	}
	if (zsolve.size() == 0) {
		//cout << "No real roots!" << endl;
		//cout << roots << endl;
		float tempZ = 0;
		float tempError = FLT_MAX;
		for (int i = 0; i < 4; i++) {
			if (abs(roots[i].imag()) < tempError) {
				tempZ = roots[i].real();
				tempError = abs(roots[i].imag());
			}
		}
		zsolve.push_back(tempZ);
		//cout << "zsolve only one: " << zsolve[0] << endl;
	}
	double z = 1e3; // ��ʼֵ��Ϊ���ֵ
	for (double val : zsolve) {
		if (val >= minZ - 10 && val <= maxZ + 10) {
			if (std::abs(val) < std::abs(z)) {
				z = val;
			}
		}
	}
	if (z > 1e2) {
		intersectPt = Vector3f(0, 0, 0);
		return;
	}

	float x = k1 * z + b1;
	float y = k2 * z + b2;
	Vector3f dstPt = Vector3f(x, y, z);
	intersectPt = dstPt;
	//cout << "intersectPt:\t" << intersectPt.transpose() << endl;
}