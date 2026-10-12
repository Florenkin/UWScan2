#include "scanner/RuiQuadricSurfaceFitting.h"
#include <unsupported/Eigen/Polynomials>
#include <numeric>

using namespace Eigen;
using namespace std;

float Rui::CostPtToQuadricSurface(Vector3f pt, VectorXf quadricSurfCoeffs) {
	//quadricSurfCoeffs = quadricSurfCoeffs / quadricSurfCoeffs.norm() * 1e3;
	//cout  << quadricSurfCoeffs.transpose() << "\t" << pt.transpose() << endl;
	float a0 = quadricSurfCoeffs(0);
	float a1 = quadricSurfCoeffs(1);
	float a2 = quadricSurfCoeffs(2);
	float a3 = quadricSurfCoeffs(3);
	float a4 = quadricSurfCoeffs(4);
	float a5 = quadricSurfCoeffs(5);
	float a6 = quadricSurfCoeffs(6);
	float a7 = quadricSurfCoeffs(7);
	float a8 = quadricSurfCoeffs(8);
	float a9 = quadricSurfCoeffs(9);

	float x = pt(0);
	float y = pt(1);
	float z = pt(2);

	float t_6 = 64 * a9 * pow(a0, 2) * pow(a1, 2) * pow(a2, 2) - 16 * pow(a0, 2) * pow(a1, 2) * a2 * pow(a8, 2) - 16 * pow(a0, 2) * a1 * pow(a2, 2) * pow(a7, 2) - 32 * a9 * pow(a0, 2) * a1 * a2 * pow(a4, 2) + 16 * pow(a0, 2) * a1 * a2 * a4 * a7 * a8 + 4 * pow(a0, 2) * a1 * pow(a4, 2) * pow(a8, 2) + 4 * pow(a0, 2) * a2 * pow(a4, 2) * pow(a7, 2) + 4 * a9 * pow(a0, 2) * pow(a4, 4) - 4 * pow(a0, 2) * pow(a4, 3) * a7 * a8 - 16 * a0 * pow(a1, 2) * pow(a2, 2) * pow(a6, 2) - 32 * a9 * a0 * pow(a1, 2) * a2 * pow(a5, 2) + 16 * a0 * pow(a1, 2) * a2 * a5 * a6 * a8 + 4 * a0 * pow(a1, 2) * pow(a5, 2) * pow(a8, 2) - 32 * a9 * a0 * a1 * pow(a2, 2) * pow(a3, 2) + 16 * a0 * a1 * pow(a2, 2) * a3 * a6 * a7 + 8 * a0 * a1 * a2 * pow(a3, 2) * pow(a8, 2) + 32 * a9 * a0 * a1 * a2 * a3 * a4 * a5 - 8 * a0 * a1 * a2 * a3 * a4 * a6 * a8 - 8 * a0 * a1 * a2 * a3 * a5 * a7 * a8 + 8 * a0 * a1 * a2 * pow(a4, 2) * pow(a6, 2) - 8 * a0 * a1 * a2 * a4 * a5 * a6 * a7 + 8 * a0 * a1 * a2 * pow(a5, 2) * pow(a7, 2) - 4 * a0 * a1 * a3 * a4 * a5 * pow(a8, 2) + 8 * a9 * a0 * a1 * pow(a4, 2) * pow(a5, 2) - 4 * a0 * a1 * pow(a4, 2) * a5 * a6 * a8 - 4 * a0 * a1 * a4 * pow(a5, 2) * a7 * a8 + 4 * a0 * pow(a2, 2) * pow(a3, 2) * pow(a7, 2) + 8 * a9 * a0 * a2 * pow(a3, 2) * pow(a4, 2) - 4 * a0 * a2 * pow(a3, 2) * a4 * a7 * a8 - 4 * a0 * a2 * a3 * pow(a4, 2) * a6 * a7 - 4 * a0 * a2 * a3 * a4 * a5
		* pow(a7, 2) - a0 * pow(a3, 2) * pow(a4, 2) * pow(a8, 2) - 8 * a9 * a0 * a3 * pow(a4, 3) * a5 + 2 * a0 * a3 * pow(a4, 3) * a6 * a8 + 6 * a0 * a3 * pow(a4, 2) * a5 * a7 * a8 - a0 * pow(a4, 4) * pow(a6, 2) + 2 * a0 * pow(a4, 3) * a5 * a6 * a7 - a0 * pow(a4, 2) * pow(a5, 2) * pow(a7, 2) + 4 * pow(a1, 2) * a2 * pow(a5, 2) * pow(a6, 2) + 4 * a9 * pow(a1, 2) * pow(a5, 4) - 4 * pow(a1, 2) * pow(a5, 3) * a6 * a8 + 4 * a1 * pow(a2, 2) * pow(a3, 2) * pow(a6, 2) + 8 * a9 * a1 * a2 * pow(a3, 2) * pow(a5, 2) - 4 * a1 * a2 * pow(a3, 2) * a5 * a6 * a8 - 4 * a1 * a2 * a3 * a4 * a5 * pow(a6, 2) - 4 * a1 * a2 * a3 * pow(a5, 2) * a6 * a7 - a1 * pow(a3, 2) * pow(a5, 2) * pow(a8, 2) - 8 * a9 * a1 * a3 * a4 * pow(a5, 3) + 6 * a1 * a3 * a4 * pow(a5, 2) * a6 * a8 + 2 * a1 * a3 * pow(a5, 3) * a7
		* a8 - a1 * pow(a4, 2) * pow(a5, 2) * pow(a6, 2) + 2 * a1 * a4 * pow(a5, 3) * a6 * a7 - a1 * pow(a5, 4) * pow(a7, 2) + 4 * a9 * pow(a2, 2) * pow(a3, 4) - 4 * pow(a2, 2) * pow(a3, 3) * a6 * a7 - a2
		* pow(a3, 4) * pow(a8, 2) - 8 * a9 * a2 * pow(a3, 3) * a4 * a5 + 2 * a2 * pow(a3, 3) * a4 * a6 * a8 + 2 * a2 * pow(a3, 3) * a5 * a7 * a8 - a2 * pow(a3, 2) * pow(a4, 2) * pow(a6, 2) + 6 * a2 * pow(a3, 2) * a4 * a5 * a6 * a7 - a2 * pow(a3, 2) * pow(a5, 2) * pow(a7, 2) + pow(a3, 3) * a4 * a5 * pow(a8, 2) + 4 * a9 * pow(a3, 2) * pow(a4, 2) * pow(a5, 2) - 2 * pow(a3, 2) * pow(a4, 2) * a5 * a6 *
		a8 - 2 * pow(a3, 2) * a4 * pow(a5, 2) * a7 * a8 + a3 * pow(a4, 3) * a5 * pow(a6, 2) - 2 * a3 * pow(a4, 2) * pow(a5, 2) * a6 * a7 + a3 * a4 * pow(a5, 3) * pow(a7, 2);

	float t_5 = 64 * a9 * pow(a0, 2) * pow(a1, 2) * a2 - 16 * pow(a0, 2) * pow(a1, 2) * pow(a8, 2) + 64 * a9 * pow(a0, 2) * a1 * pow(a2, 2) - 16 * pow(a0, 2) * a1 * a2 * pow(a7, 2) - 16 * pow(a0, 2) *
		a1 * a2 * pow(a8, 2) - 16 * a9 * pow(a0, 2) * a1 * pow(a4, 2) + 16 * pow(a0, 2) * a1 * a4 * a7 * a8 - 16 * pow(a0, 2) * pow(a2, 2) * pow(a7, 2) - 16 * a9 * pow(a0, 2) * a2 * pow(a4, 2) + 16 * pow(a0, 2) * a2 * a4 * a7 * a8 + 64 * a9 * a0 * pow(a1, 2) * pow(a2, 2) - 16 * a0 * pow(a1, 2) * a2 * pow(a6, 2) - 16 * a0 * pow(a1, 2) * a2 * pow(a8, 2) - 16 * a9 * a0 * pow(a1, 2) * pow(a5, 2) + 16
		* a0 * pow(a1, 2) * a5 * a6 * a8 - 16 * a0 * a1 * pow(a2, 2) * pow(a6, 2) - 16 * a0 * a1 * pow(a2, 2) * pow(a7, 2) - 32 * a9 * a0 * a1 * a2 * pow(a3, 2) + 16 * a0 * a1 * a2 * a3 * a6 * a7 - 32
		* a9 * a0 * a1 * a2 * pow(a4, 2) + 16 * a0 * a1 * a2 * a4 * a7 * a8 - 32 * a9 * a0 * a1 * a2 * pow(a5, 2) + 16 * a0 * a1 * a2 * a5 * a6 * a8 + 8 * a0 * a1 * pow(a3, 2) * pow(a8, 2) + 16 * a9
		* a0 * a1 * a3 * a4 * a5 - 8 * a0 * a1 * a3 * a4 * a6 * a8 - 8 * a0 * a1 * a3 * a5 * a7 * a8 + 4 * a0 * a1 * pow(a4, 2) * pow(a6, 2) + 4 * a0 * a1 * pow(a4, 2) * pow(a8, 2) - 8 * a0 * a1 * a4 * a5 * a6 * a7 + 4 * a0 * a1 * pow(a5, 2) * pow(a7, 2) + 4 * a0 * a1 * pow(a5, 2) * pow(a8, 2) - 16 * a9 * a0 * pow(a2, 2) * pow(a3, 2) + 16 * a0 * pow(a2, 2) * a3 * a6 * a7 + 4 * a0 * a2 * pow(a3, 2) * pow(a7, 2) + 4 * a0 * a2 * pow(a3, 2) * pow(a8, 2) + 16 * a9 * a0 * a2 * a3 * a4 * a5 - 8 * a0 * a2 * a3 * a4 * a6 * a8 - 8 * a0 * a2 * a3 * a5 * a7 * a8 + 4 * a0 * a2 * pow(a4, 2) * pow(a6, 2) + 4 * a0 * a2 * pow(a4, 2) * pow(a7, 2) - 8 * a0 * a2 * a4 * a5 * a6 * a7 + 8 * a0 * a2 * pow(a5, 2) * pow(a7, 2) + 4 * a9 * a0 * pow(a3, 2) * pow(a4, 2) - 4 * a0 * pow(a3, 2) * a4 *
		a7 * a8 + 4 * a9 * a0 * pow(a4, 4) - 4 * a0 * pow(a4, 3) * a7 * a8 + 4 * a9 * a0 * pow(a4, 2) * pow(a5, 2) - 4 * a0 * a4 * pow(a5, 2) * a7 * a8 - 16 * pow(a1, 2) * pow(a2, 2) * pow(a6, 2) - 16 *
		a9 * pow(a1, 2) * a2 * pow(a5, 2) + 16 * pow(a1, 2) * a2 * a5 * a6 * a8 - 16 * a9 * a1 * pow(a2, 2) * pow(a3, 2) + 16 * a1 * pow(a2, 2) * a3 * a6 * a7 + 4 * a1 * a2 * pow(a3, 2) * pow(a6, 2) + 4
		* a1 * a2 * pow(a3, 2) * pow(a8, 2) + 16 * a9 * a1 * a2 * a3 * a4 * a5 - 8 * a1 * a2 * a3 * a4 * a6 * a8 - 8 * a1 * a2 * a3 * a5 * a7 * a8 + 8 * a1 * a2 * pow(a4, 2) * pow(a6, 2) - 8 * a1 * a2 * a4 * a5 * a6 * a7 + 4 * a1 * a2 * pow(a5, 2) * pow(a6, 2) + 4 * a1 * a2 * pow(a5, 2) * pow(a7, 2) + 4 * a9 * a1 * pow(a3, 2) * pow(a5, 2) - 4 * a1 * pow(a3, 2) * a5 * a6 * a8 + 4 * a9 * a1 * pow(a4, 2) * pow(a5, 2) - 4 * a1 * pow(a4, 2) * a5 * a6 * a8 + 4 * a9 * a1 * pow(a5, 4) - 4 * a1 * pow(a5, 3) * a6 * a8 + 4 * a9 * a2 * pow(a3, 4) - 4 * a2 * pow(a3, 3) * a6 * a7 + 4 * a9 * a2
		* pow(a3, 2) * pow(a4, 2) + 4 * a9 * a2 * pow(a3, 2) * pow(a5, 2) - 4 * a2 * a3 * pow(a4, 2) * a6 * a7 - 4 * a2 * a3 * pow(a5, 2) * a6 * a7 - pow(a3, 4) * pow(a8, 2) - 4 * a9 * pow(a3, 3) * a4 * a5 + 2 * pow(a3, 3) * a4 * a6 * a8 + 2 * pow(a3, 3) * a5 * a7 * a8 - pow(a3, 2) * pow(a4, 2) * pow(a6, 2) - pow(a3, 2) * pow(a4, 2) * pow(a8, 2) + 2 * pow(a3, 2) * a4 * a5 * a6 * a7 - pow(a3, 2) * pow(a5, 2) * pow(a7, 2) - pow(a3, 2) * pow(a5, 2) * pow(a8, 2) - 4 * a9 * a3 * pow(a4, 3) * a5 + 2 * a3 * pow(a4, 3) * a6 * a8 + 2 * a3 * pow(a4, 2) * a5 * a7 * a8 - 4 * a9 * a3 * a4 * pow(a5, 3) + 2 * a3 * a4 * pow(a5, 2) * a6 * a8 + 2 * a3 * pow(a5, 3) * a7 * a8 - pow(a4, 4) * pow(a6, 2) + 2 * pow(a4, 3) * a5 * a6 * a7 - pow(a4, 2) * pow(a5, 2) * pow(a6, 2) - pow(a4, 2) * pow(a5, 2) * pow(a7, 2) + 2 * a4 * pow(a5, 3) * a6 * a7 - pow(a5, 4) * pow(a7, 2);

	float t_4 = 16 * pow(a0, 2) * pow(a1, 2) * a2 * pow(z, 2) + 16 * pow(a0, 2) * pow(a1, 2) * a8 * z + 16 * a9 * pow(a0, 2) * pow(a1, 2) + 16 * pow(a0, 2) * a1 * pow(a2, 2) * pow(y, 2) - 16 * pow(a0, 2) * a1 * a2 * a4 * y * z + 64 * a9 * pow(a0, 2) * a1 * a2 - 4 * pow(a0, 2) * a1 * pow(a4, 2) * pow(z, 2) - 8 * pow(a0, 2) * a1 * a4 * a7 * z - 8 * pow(a0, 2) * a1 * a4 * a8 * y - 4 * pow(a0, 2)
		* a1 * pow(a7, 2) - 16 * pow(a0, 2) * a1 * pow(a8, 2) + 16 * pow(a0, 2) * pow(a2, 2) * a7 * y + 16 * a9 * pow(a0, 2) * pow(a2, 2) - 4 * pow(a0, 2) * a2 * pow(a4, 2) * pow(y, 2) - 8 * pow(a0, 2) * a2 * a4 * a7 * z - 8 * pow(a0, 2) * a2 * a4 * a8 * y - 16 * pow(a0, 2) * a2 * pow(a7, 2) - 4 * pow(a0, 2) * a2 * pow(a8, 2) + 4 * pow(a0, 2) * pow(a4, 3) * y * z + 4 * pow(a0, 2) * pow(a4, 2) * a7 * y + 4 * pow(a0, 2) * pow(a4, 2) * a8 * z - 8 * a9 * pow(a0, 2) * pow(a4, 2) + 12 * pow(a0, 2) * a4 * a7 * a8 + 16 * a0 * pow(a1, 2) * pow(a2, 2) * pow(x, 2) - 16 * a0 * pow(a1, 2) * a2 * a5 * x * z + 64 * a9 * a0 * pow(a1, 2) * a2 - 4 * a0 * pow(a1, 2) * pow(a5, 2) * pow(z, 2) - 8 * a0 * pow(a1, 2) * a5 * a6 * z - 8 * a0 * pow(a1, 2) * a5 * a8 * x - 4 * a0 * pow(a1, 2) * pow(a6, 2) - 16
		* a0 * pow(a1, 2) * pow(a8, 2) - 16 * a0 * a1 * pow(a2, 2) * a3 * x * y + 64 * a9 * a0 * a1 * pow(a2, 2) - 8 * a0 * a1 * a2 * pow(a3, 2) * pow(z, 2) + 8 * a0 * a1 * a2 * a3 * a4 * x * z + 8 * a0 * a1 * a2 * a3 * a5 * y * z - 8 * a0 * a1 * a2 * pow(a4, 2) * pow(x, 2) + 8 * a0 * a1 * a2 * a4 * a5 * x * y - 8 * a0 * a1 * a2 * pow(a5, 2) * pow(y, 2) - 16 * a0 * a1 * a2 * pow(a6, 2) - 16 * a0 * a1 * a2 * pow(a7, 2) - 16 * a0 * a1 * a2 * pow(a8, 2) - 8 * a0 * a1 * pow(a3, 2) * a8 * z - 8 * a9 * a0 * a1 * pow(a3, 2) + 4 * a0 * a1 * a3 * a4 * a5 * pow(z, 2) + 4 * a0 * a1 * a3 *
		a4 * a6 * z + 4 * a0 * a1 * a3 * a4 * a8 * x + 4 * a0 * a1 * a3 * a5 * a7 * z + 4 * a0 * a1 * a3 * a5 * a8 * y + 4 * a0 * a1 * a3 * a6 * a7 + 4 * a0 * a1 * pow(a4, 2) * a5 * x * z - 16 * a9 * a0 * a1 * pow(a4, 2) + 4 * a0 * a1 * a4 * pow(a5, 2) * y * z + 4 * a0 * a1 * a4 * a5 * a6 * y + 4 * a0 * a1 * a4 * a5 * a7 * x + 16 * a0 * a1 * a4 * a7 * a8 - 16 * a9 * a0 * a1 * pow(a5, 2) + 16 * a0 * a1 * a5 * a6 * a8 - 4 * a0 * pow(a2, 2) * pow(a3, 2) * pow(y, 2) - 8 * a0 * pow(a2, 2) * a3 * a6 * y - 8 * a0 * pow(a2, 2) * a3 * a7 * x - 4 * a0 * pow(a2, 2) * pow(a6, 2) - 16 * a0 * pow(a2, 2) * pow(a7, 2) + 4 * a0 * a2 * pow(a3, 2) * a4 * y * z - 16 * a9 * a0 * a2 * pow(a3, 2) + 4 * a0 * a2 * a3 * pow(a4, 2) * x * y + 4 * a0 * a2 * a3 * a4 * a5 * pow(y, 2) + 4 * a0
		* a2 * a3 * a4 * a6 * z + 4 * a0 * a2 * a3 * a4 * a8 * x + 4 * a0 * a2 * a3 * a5 * a7 * z + 4 * a0 * a2 * a3 * a5 * a8 * y + 16 * a0 * a2 * a3 * a6 * a7 - 16 * a9 * a0 * a2 * pow(a4, 2) +
		4 * a0 * a2 * a4 * a5 * a6 * y + 4 * a0 * a2 * a4 * a5 * a7 * x + 16 * a0 * a2 * a4 * a7 * a8 - 8 * a0 * a2 * pow(a5, 2) * a7 * y - 8 * a9 * a0 * a2 * pow(a5, 2) + 4 * a0 * a2 * a5 * a6 * a8 + a0 * pow(a3, 2) * pow(a4, 2) * pow(z, 2) + 2 * a0 * pow(a3, 2) * a4 * a7 * z + 2 * a0 * pow(a3, 2) * a4 * a8 * y + a0 * pow(a3, 2) * pow(a7, 2) + 4 * a0 * pow(a3, 2) * pow(a8, 2) - 2 * a0 * a3 * pow(a4, 3) * x * z - 6 * a0 * a3 * pow(a4, 2) * a5 * y * z - 2 * a0 * a3 * pow(a4, 2) * a6 * y - 2 * a0 * a3 * pow(a4, 2) * a7 * x - 4 * a0 * a3 * a4 * a5 * a7 * y - 4 * a0 * a3 * a4 * a5
		* a8 * z + 8 * a9 * a0 * a3 * a4 * a5 - 6 * a0 * a3 * a4 * a6 * a8 - 6 * a0 * a3 * a5 * a7 * a8 + a0 * pow(a4, 4) * pow(x, 2) - 2 * a0 * pow(a4, 3) * a5 * x * y + a0 * pow(a4, 2) * pow(a5, 2)
		* pow(y, 2) - 2 * a0 * pow(a4, 2) * a5 * a6 * z - 2 * a0 * pow(a4, 2) * a5 * a8 * x + 2 * a0 * pow(a4, 2) * pow(a6, 2) + 2 * a0 * a4 * pow(a5, 2) * a7 * z + 2 * a0 * a4 * pow(a5, 2) * a8 * y - 6 * a0 * a4 * a5 * a6 * a7 + 4 * a0 * pow(a5, 2) * pow(a7, 2) + a0 * pow(a5, 2) * pow(a8, 2) + 16 * pow(a1, 2) * pow(a2, 2) * a6 * x + 16 * a9 * pow(a1, 2) * pow(a2, 2) - 4 * pow(a1, 2) * a2 * pow(a5, 2) * pow(x, 2) - 8 * pow(a1, 2) * a2 * a5 * a6 * z - 8 * pow(a1, 2) * a2 * a5 * a8 * x - 16 * pow(a1, 2) * a2 * pow(a6, 2) - 4 * pow(a1, 2) * a2 * pow(a8, 2) + 4 * pow(a1, 2) * pow(a5, 3) * x * z + 4 * pow(a1, 2) * pow(a5, 2) * a6 * x + 4 * pow(a1, 2) * pow(a5, 2) * a8 * z - 8 * a9 * pow(a1, 2) * pow(a5, 2) + 12 * pow(a1, 2) * a5 * a6 * a8 - 4 * a1 * pow(a2, 2) * pow(a3, 2) * pow(x, 2) - 8 * a1 * pow(a2, 2) * a3 * a6 * y - 8 * a1 * pow(a2, 2) * a3 * a7 * x - 16 * a1 * pow(a2, 2) * pow(a6, 2) - 4 * a1 * pow(a2, 2) * pow(a7, 2) + 4 * a1 * a2 * pow(a3, 2) * a5 * x * z - 16 * a9 *
		a1 * a2 * pow(a3, 2) + 4 * a1 * a2 * a3 * a4 * a5 * pow(x, 2) + 4 * a1 * a2 * a3 * a4 * a6 * z + 4 * a1 * a2 * a3 * a4 * a8 * x + 4 * a1 * a2 * a3 * pow(a5, 2) * x * y + 4 * a1 * a2 * a3 * a5 * a7 * z + 4 * a1 * a2 * a3 * a5 * a8 * y + 16 * a1 * a2 * a3 * a6 * a7 - 8 * a1 * a2 * pow(a4, 2) * a6 * x - 8 * a9 * a1 * a2 * pow(a4, 2) + 4 * a1 * a2 * a4 * a5 * a6 * y + 4 * a1 * a2
		* a4 * a5 * a7 * x + 4 * a1 * a2 * a4 * a7 * a8 - 16 * a9 * a1 * a2 * pow(a5, 2) + 16 * a1 * a2 * a5 * a6 * a8 + a1 * pow(a3, 2) * pow(a5, 2) * pow(z, 2) + 2 * a1 * pow(a3, 2) * a5 * a6 * z +
		2 * a1 * pow(a3, 2) * a5 * a8 * x + a1 * pow(a3, 2) * pow(a6, 2) + 4 * a1 * pow(a3, 2) * pow(a8, 2) - 6 * a1 * a3 * a4 * pow(a5, 2) * x * z - 4 * a1 * a3 * a4 * a5 * a6 * x - 4 * a1 * a3 * a4 * a5 * a8 * z + 8 * a9 * a1 * a3 * a4 * a5 - 6 * a1 * a3 * a4 * a6 * a8 - 2 * a1 * a3 * pow(a5, 3) * y * z - 2 * a1 * a3 * pow(a5, 2) * a6 * y - 2 * a1 * a3 * pow(a5, 2) * a7 * x - 6 * a1 * a3 * a5 * a7 * a8 + a1 * pow(a4, 2) * pow(a5, 2) * pow(x, 2) + 2 * a1 * pow(a4, 2) * a5 * a6 * z + 2 * a1 * pow(a4, 2) * a5 * a8 * x + 4 * a1 * pow(a4, 2) * pow(a6, 2) + a1 * pow(a4, 2) * pow(a8, 2) - 2 * a1 * a4 * pow(a5, 3) * x * y - 2 * a1 * a4 * pow(a5, 2) * a7 * z - 2 * a1 * a4 * pow(a5, 2) * a8 * y - 6 * a1 * a4 * a5 * a6 * a7 + a1 * pow(a5, 4) * pow(y, 2) + 2 * a1 * pow(a5, 2) * pow(a7, 2) + 4 * pow(a2, 2) * pow(a3, 3) * x * y + 4 * pow(a2, 2) * pow(a3, 2) * a6 * x + 4 * pow(a2, 2) * pow(a3, 2) * a7 * y - 8 * a9 * pow(a2, 2) * pow(a3, 2) + 12 * pow(a2, 2) * a3 * a6 * a7 + a2 * pow(a3, 4) * pow(z, 2) - 2 * a2 * pow(a3, 3) * a4 * x * z - 2 * a2 * pow(a3, 3) * a5 * y * z + a2 * pow(a3, 2) * pow(a4, 2) * pow(x, 2) - 6 * a2 * pow(a3, 2) * a4 * a5 * x * y - 2 * a2 * pow(a3, 2) * a4 * a7 * z - 2 * a2 * pow(a3, 2) * a4 * a8 * y + a2 * pow(a3, 2) * pow(a5, 2) * pow(y, 2) - 2 * a2 * pow(a3, 2) * a5 * a6 * z - 2 * a2 * pow(a3, 2) * a5 * a8 * x + 2 * a2 * pow(a3, 2) * pow(a8, 2) + 2 * a2 * a3 * pow(a4, 2) * a6 * y + 2 * a2 * a3 * pow(a4, 2) * a7 * x - 4 * a2 * a3 * a4 * a5 * a6 * x - 4 * a2 * a3 * a4 * a5 * a7 * y + 8 * a9 * a2 * a3 * a4 * a5 - 6 * a2 *
		a3 * a4 * a6 * a8 + 2 * a2 * a3 * pow(a5, 2) * a6 * y + 2 * a2 * a3 * pow(a5, 2) * a7 * x - 6 * a2 * a3 * a5 * a7 * a8 + 4 * a2 * pow(a4, 2) * pow(a6, 2) + a2 * pow(a4, 2) * pow(a7, 2) - 6 * a2 * a4 * a5 * a6 * a7 + a2 * pow(a5, 2) * pow(a6, 2) + 4 * a2 * pow(a5, 2) * pow(a7, 2) + pow(a3, 4) * a8 * z + a9 * pow(a3, 4) - pow(a3, 3) * a4 * a5 * pow(z, 2) - pow(a3, 3) * a4 * a6 * z - pow(a3, 3) * a4 * a8 * x - pow(a3, 3) * a5 * a7 * z - pow(a3, 3) * a5 * a8 * y - pow(a3, 3) * a6 * a7 + 2 * pow(a3, 2) * pow(a4, 2) * a5 * x * z + pow(a3, 2) * pow(a4, 2) * a6 * x + pow(a3, 2) * pow(a4, 2) * a8 * z + 2 * a9 * pow(a3, 2) * pow(a4, 2) + 2 * pow(a3, 2) * a4 * pow(a5, 2) * y * z + pow(a3, 2) * a4 * a5 * a6 * y + pow(a3, 2) * a4 * a5 * a7 * x - pow(a3, 2) * a4 * a7 * a8 + pow(a3, 2) * pow(a5, 2) * a7 * y + pow(a3, 2) * pow(a5, 2) * a8 * z + 2 * a9 * pow(a3, 2) * pow(a5, 2) - pow(a3, 2) * a5 * a6 * a8 - a3 * pow(a4, 3) * a5 * pow(x, 2) - a3 * pow(a4, 3) * a6 * z - a3 * pow(a4, 3) * a8 * x + 2 * a3 * pow(a4, 2) * pow(a5, 2) * x * y + a3 * pow(a4, 2) * a5 * a7 * z + a3 * pow(a4, 2) * a5 * a8 * y - a3 * pow(a4, 2) * a6 * a7 - a3 * a4 * pow(a5, 3) * pow(y, 2) + a3 * a4 * pow(a5, 2) * a6 * z + a3 * a4 * pow(a5, 2) * a8 * x + a3 * a4 * a5 * pow(a6, 2) + a3 * a4 * a5 * pow(a7, 2) + a3 * a4 * a5 * pow(a8, 2) - a3 * pow(a5, 3) * a7 * z - a3 * pow(a5, 3) * a8 * y - a3 * pow(a5, 2) * a6 * a7 + pow(a4, 4) * a6 * x + a9 * pow(a4, 4) - pow(a4, 3) * a5 * a6 * y - pow(a4, 3) * a5 * a7 * x - pow(a4, 3) * a7 * a8 + pow(a4, 2) * pow(a5, 2) * a6 * x + pow(a4, 2) * pow(a5, 2) * a7 * y + 2 * a9 * pow(a4, 2) * pow(a5, 2) - pow(a4, 2) * a5 * a6 * a8 - a4 * pow(a5, 3) * a6 * y - a4 * pow(a5, 3) * a7 * x - a4 * pow(a5, 2) * a7 * a8 + pow(a5, 4) * a7 * y + a9 * pow(a5, 4) - pow(a5, 3) * a6 * a8;

	float t_3 = 16 * pow(a0, 2) * a1 * a2 * pow(y, 2) + 16 * pow(a0, 2) * a1 * a2 * pow(z, 2) + 16 * pow(a0, 2) * a1 * a8 * z + 16 * a9 * pow(a0, 2) * a1 + 16 * pow(a0, 2) * a2 * a7 * y + 16 * a9 *
		pow(a0, 2) * a2 - 4 * pow(a0, 2) * pow(a4, 2) * pow(y, 2) - 4 * pow(a0, 2) * pow(a4, 2) * pow(z, 2) - 8 * pow(a0, 2) * a4 * a7 * z - 8 * pow(a0, 2) * a4 * a8 * y - 4 * pow(a0, 2) * pow(a7, 2) - 4 *
		pow(a0, 2) * pow(a8, 2) + 16 * a0 * pow(a1, 2) * a2 * pow(x, 2) + 16 * a0 * pow(a1, 2) * a2 * pow(z, 2) + 16 * a0 * pow(a1, 2) * a8 * z + 16 * a9 * a0 * pow(a1, 2) + 16 * a0 * a1 * pow(a2, 2) * pow(x, 2) + 16 * a0 * a1 * pow(a2, 2) * pow(y, 2) - 16 * a0 * a1 * a2 * a3 * x * y - 16 * a0 * a1 * a2 * a4 * y * z - 16 * a0 * a1 * a2 * a5 * x * z + 64 * a9 * a0 * a1 * a2 - 4 * a0 * a1 * pow(a4, 2) * pow(x, 2) - 4 * a0 * a1 * pow(a4, 2) * pow(z, 2) - 8 * a0 * a1 * a4 * a7 * z - 8 * a0 * a1 * a4 * a8 * y - 4 * a0 * a1 * pow(a5, 2) * pow(y, 2) - 4 * a0 * a1 * pow(a5, 2) * pow(z, 2) - 8 * a0 * a1 * a5 * a6 * z - 8 * a0 * a1 * a5 * a8 * x - 4 * a0 * a1 * pow(a6, 2) - 4 * a0 * a1 * pow(a7, 2) - 16 * a0 * a1 * pow(a8, 2) + 16 * a0 * pow(a2, 2) * a7 * y + 16 * a9 * a0 * pow(a2, 2) - 4 * a0 * a2 * pow(a3, 2) * pow(y, 2) - 4 * a0 * a2 * pow(a3, 2) * pow(z, 2) - 8 * a0 * a2 * a3 * a6 * y - 8 * a0 * a2 * a3 * a7 * x - 4 * a0 * a2 * pow(a4, 2) * pow(x, 2) - 4 * a0 * a2 * pow(a4, 2) * pow(y, 2) - 8 * a0 * a2 * a4 * a7 * z - 8 * a0 * a2 * a4 * a8 * y - 4 * a0 * a2 * pow(a6, 2) - 16 * a0 * a2 * pow(a7, 2) - 4 * a0 * a2 * pow(a8, 2) - 4 * a0 * pow(a3, 2) * a8 * z
		- 4 * a9 * a0 * pow(a3, 2) + 4 * a0 * a3 * pow(a4, 2) * x * y + 4 * a0 * a3 * a4 * a5 * pow(y, 2) + 4 * a0 * a3 * a4 * a5 * pow(z, 2) + 4 * a0 * a3 * a4 * a6 * z + 4 * a0 * a3 * a4 * a8 * x + 4 * a0 * a3 * a5 * a7 * z + 4 * a0 * a3 * a5 * a8 * y + 4 * a0 * a3 * a6 * a7 + 4 * a0 * pow(a4, 3) * y * z + 4 * a0 * pow(a4, 2) * a5 * x * z + 4 * a0 * pow(a4, 2) * a7 * y + 4 * a0 * pow(a4, 2) * a8 * z - 8 * a9 * a0 * pow(a4, 2) + 4 * a0 * a4 * a5 * a6 * y + 4 * a0 * a4 * a5 * a7 * x + 12 * a0 * a4 * a7 * a8 - 4 * a0 * pow(a5, 2) * a7 * y - 4 * a9 * a0 * pow(a5, 2) + 4 * a0
		* a5 * a6 * a8 + 16 * pow(a1, 2) * a2 * a6 * x + 16 * a9 * pow(a1, 2) * a2 - 4 * pow(a1, 2) * pow(a5, 2) * pow(x, 2) - 4 * pow(a1, 2) * pow(a5, 2) * pow(z, 2) - 8 * pow(a1, 2) * a5 * a6 * z - 8 *
		pow(a1, 2) * a5 * a8 * x - 4 * pow(a1, 2) * pow(a6, 2) - 4 * pow(a1, 2) * pow(a8, 2) + 16 * a1 * pow(a2, 2) * a6 * x + 16 * a9 * a1 * pow(a2, 2) - 4 * a1 * a2 * pow(a3, 2) * pow(x, 2) - 4 * a1 * a2 * pow(a3, 2) * pow(z, 2) - 8 * a1 * a2 * a3 * a6 * y - 8 * a1 * a2 * a3 * a7 * x - 4 * a1 * a2 * pow(a5, 2) * pow(x, 2) - 4 * a1 * a2 * pow(a5, 2) * pow(y, 2) - 8 * a1 * a2 * a5 * a6 * z - 8
		* a1 * a2 * a5 * a8 * x - 16 * a1 * a2 * pow(a6, 2) - 4 * a1 * a2 * pow(a7, 2) - 4 * a1 * a2 * pow(a8, 2) - 4 * a1 * pow(a3, 2) * a8 * z - 4 * a9 * a1 * pow(a3, 2) + 4 * a1 * a3 * a4 * a5 * pow(x, 2) + 4 * a1 * a3 * a4 * a5 * pow(z, 2) + 4 * a1 * a3 * a4 * a6 * z + 4 * a1 * a3 * a4 * a8 * x + 4 * a1 * a3 * pow(a5, 2) * x * y + 4 * a1 * a3 * a5 * a7 * z + 4 * a1 * a3 * a5 * a8 * y + 4 * a1 * a3 * a6 * a7 - 4 * a1 * pow(a4, 2) * a6 * x - 4 * a9 * a1 * pow(a4, 2) + 4 * a1 * a4 * pow(a5, 2) * y * z + 4 * a1 * a4 * a5 * a6 * y + 4 * a1 * a4 * a5 * a7 * x + 4 * a1 * a4 *
		a7 * a8 + 4 * a1 * pow(a5, 3) * x * z + 4 * a1 * pow(a5, 2) * a6 * x + 4 * a1 * pow(a5, 2) * a8 * z - 8 * a9 * a1 * pow(a5, 2) + 12 * a1 * a5 * a6 * a8 - 4 * pow(a2, 2) * pow(a3, 2) * pow(x, 2)
		- 4 * pow(a2, 2) * pow(a3, 2) * pow(y, 2) - 8 * pow(a2, 2) * a3 * a6 * y - 8 * pow(a2, 2) * a3 * a7 * x - 4 * pow(a2, 2) * pow(a6, 2) - 4 * pow(a2, 2) * pow(a7, 2) + 4 * a2 * pow(a3, 3) * x * y + 4 * a2 * pow(a3, 2) * a4 * y * z + 4 * a2 * pow(a3, 2) * a5 * x * z + 4 * a2 * pow(a3, 2) * a6 * x + 4 * a2 * pow(a3, 2) * a7 * y - 8 * a9 * a2 * pow(a3, 2) + 4 * a2 * a3 * a4 * a5 * pow(x, 2) + 4 * a2 * a3 * a4 * a5 * pow(y, 2) + 4 * a2 * a3 * a4 * a6 * z + 4 * a2 * a3 * a4 * a8 * x + 4 * a2 * a3 * a5 * a7 * z + 4 * a2 * a3 * a5 * a8 * y + 12 * a2 * a3 * a6 * a7 - 4 * a2 * pow(a4, 2) * a6 * x - 4 * a9 * a2 * pow(a4, 2) + 4 * a2 * a4 * a5 * a6 * y + 4 * a2 * a4 * a5 * a7 * x + 4 * a2 * a4 * a7 * a8 - 4 * a2 * pow(a5, 2) * a7 * y - 4 * a9 * a2 * pow(a5, 2) + 4 * a2 *
		a5 * a6 * a8 - 4 * pow(a3, 2) * a4 * a5 * x * y + 2 * pow(a3, 2) * pow(a8, 2) - 4 * a3 * pow(a4, 2) * a5 * y * z - 4 * a3 * a4 * pow(a5, 2) * x * z - 4 * a3 * a4 * a5 * a6 * x - 4 * a3 * a4 *
		a5 * a7 * y - 4 * a3 * a4 * a5 * a8 * z + 4 * a9 * a3 * a4 * a5 - 4 * a3 * a4 * a6 * a8 - 4 * a3 * a5 * a7 * a8 + 2 * pow(a4, 2) * pow(a6, 2) - 4 * a4 * a5 * a6 * a7 + 2 * pow(a5, 2) * pow(a7, 2);

	float t_2 = 4 * pow(a0, 2) * a1 * pow(y, 2) + 4 * pow(a0, 2) * a2 * pow(z, 2) + 4 * pow(a0, 2) * a4 * y * z + 4 * pow(a0, 2) * a7 * y + 4 * pow(a0, 2) * a8 * z + 4 * a9 * pow(a0, 2) + 4 * a0 * pow(a1, 2) * pow(x, 2) + 16 * a0 * a1 * a2 * pow(x, 2) + 16 * a0 * a1 * a2 * pow(y, 2) + 16 * a0 * a1 * a2 * pow(z, 2) - 4 * a0 * a1 * a3 * x * y + 16 * a0 * a1 * a8 * z + 16 * a9 * a0 * a1 + 4
		* a0 * pow(a2, 2) * pow(x, 2) - 4 * a0 * a2 * a5 * x * z + 16 * a0 * a2 * a7 * y + 16 * a9 * a0 * a2 - a0 * pow(a3, 2) * pow(y, 2) - 2 * a0 * a3 * a4 * x * z - 2 * a0 * a3 * a5 * y * z - 2 *
		a0 * a3 * a6 * y - 2 * a0 * a3 * a7 * x - 2 * a0 * pow(a4, 2) * pow(x, 2) - 4 * a0 * pow(a4, 2) * pow(y, 2) - 4 * a0 * pow(a4, 2) * pow(z, 2) - 2 * a0 * a4 * a5 * x * y - 8 * a0 * a4 * a7 * z - 8 * a0 * a4 * a8 * y - a0 * pow(a5, 2) * pow(z, 2) - 2 * a0 * a5 * a6 * z - 2 * a0 * a5 * a8 * x - a0 * pow(a6, 2) - 4 * a0 * pow(a7, 2) - 4 * a0 * pow(a8, 2) + 4 * pow(a1, 2) * a2 * pow(z, 2)
		+ 4 * pow(a1, 2) * a5 * x * z + 4 * pow(a1, 2) * a6 * x + 4 * pow(a1, 2) * a8 * z + 4 * a9 * pow(a1, 2) + 4 * a1 * pow(a2, 2) * pow(y, 2) - 4 * a1 * a2 * a4 * y * z + 16 * a1 * a2 * a6 * x + 16 * a9 * a1 * a2 - a1 * pow(a3, 2) * pow(x, 2) - 2 * a1 * a3 * a4 * x * z - 2 * a1 * a3 * a5 * y * z - 2 * a1 * a3 * a6 * y - 2 * a1 * a3 * a7 * x - a1 * pow(a4, 2) * pow(z, 2) - 2 * a1 * a4 * a5 * x * y - 2 * a1 * a4 * a7 * z - 2 * a1 * a4 * a8 * y - 4 * a1 * pow(a5, 2) * pow(x, 2) - 2 * a1 * pow(a5, 2) * pow(y, 2) - 4 * a1 * pow(a5, 2) * pow(z, 2) - 8 * a1 * a5 * a6 * z - 8 * a1 * a5 * a8 * x - 4 * a1 * pow(a6, 2) - a1 * pow(a7, 2) - 4 * a1 * pow(a8, 2) + 4 * pow(a2, 2) * a3 * x * y + 4 * pow(a2, 2) * a6 * x + 4 * pow(a2, 2) * a7 * y + 4 * a9 * pow(a2, 2) - 4 * a2 * pow(a3, 2) * pow(x, 2) - 4 * a2 * pow(a3, 2) * pow(y, 2) - 2 * a2 * pow(a3, 2) * pow(z, 2) - 2 * a2 * a3 * a4 * x * z - 2 * a2 * a3 * a5 * y * z - 8 * a2 * a3 * a6 * y - 8 * a2 * a3 * a7 * x - a2 * pow(a4, 2) * pow(y, 2) - 2 * a2 * a4 * a5 * x * y - 2 * a2 * a4 * a7 * z - 2 * a2 * a4 * a8 * y - a2 * pow(a5, 2) * pow(x, 2) - 2 * a2 * a5 * a6 * z - 2 * a2 * a5 * a8 * x - 4 * a2 * pow(a6, 2) - 4 * a2 * pow(a7, 2) - a2 * pow(a8, 2) + pow(a3, 3) * x * y + pow(a3, 2) * a4 * y * z + pow(a3, 2) * a5 * x * z + pow(a3, 2) * a6 * x + pow(a3, 2) * a7 * y - 2 * pow(a3, 2) * a8 * z - 2 * a9
		* pow(a3, 2) + a3 * pow(a4, 2) * x * y + 3 * a3 * a4 * a5 * pow(x, 2) + 3 * a3 * a4 * a5 * pow(y, 2) + 3 * a3 * a4 * a5 * pow(z, 2) + 3 * a3 * a4 * a6 * z + 3 * a3 * a4 * a8 * x + a3 * pow(a5, 2) * x * y + 3 * a3 * a5 * a7 * z + 3 * a3 * a5 * a8 * y + 3 * a3 * a6 * a7 + pow(a4, 3) * y * z + pow(a4, 2) * a5 * x * z - 2 * pow(a4, 2) * a6 * x + pow(a4, 2) * a7 * y + pow(a4, 2) * a8 * z - 2 * a9 * pow(a4, 2) + a4 * pow(a5, 2) * y * z + 3 * a4 * a5 * a6 * y + 3 * a4 * a5 * a7 * x + 3 * a4 * a7 * a8 + pow(a5, 3) * x * z + pow(a5, 2) * a6 * x - 2 * pow(a5, 2) * a7 * y + pow(a5, 2) * a8 * z - 2 * a9 * pow(a5, 2) + 3 * a5 * a6 * a8;

	float t_1 = 4 * a0 * a9 + 4 * a1 * a9 + 4 * a2 * a9 - pow(a6, 2) - pow(a7, 2) - pow(a8, 2) - pow(a3, 2) * pow(x, 2) - pow(a5, 2) * pow(x, 2) - pow(a3, 2) * pow(y, 2) - pow(a4, 2) * pow(y, 2) - pow(a4, 2) * pow(z, 2) - pow(a5, 2) * pow(z, 2) + 4 * a1 * a6 * x + 4 * a2 * a6 * x - 2 * a3 * a7 * x - 2 * a5 * a8 * x + 4 * a0 * a7 * y + 4 * a2 * a7 * y - 2 * a3 * a6 * y - 2 * a4 * a8 * y + 4
		* a0 * a8 * z + 4 * a1 * a8 * z - 2 * a4 * a7 * z - 2 * a5 * a6 * z + 4 * a0 * a1 * pow(x, 2) + 4 * a0 * a2 * pow(x, 2) + 4 * a0 * a1 * pow(y, 2) + 4 * a1 * a2 * pow(y, 2) + 4 * a0 * a2 * pow(z, 2) + 4 * a1 * a2 * pow(z, 2) + 4 * a2 * a3 * x * y - 2 * a4 * a5 * x * y + 4 * a1 * a5 * x * z - 2 * a3 * a4 * x * z + 4 * a0 * a4 * y * z - 2 * a3 * a5 * y * z;

	float t_0 = a0 * pow(x, 2) + a3 * x * y + a5 * x * z + a6 * x + a1 * pow(y, 2) + a4 * y * z + a7 * y + a2 * pow(z, 2) + a8 * z + a9;

	Eigen::PolynomialSolver<float, Eigen::Dynamic> solver; //����ʽ���
	Eigen::VectorXf coeff(7);
	coeff[0] = t_0;
	coeff[1] = t_1;
	coeff[2] = t_2;
	coeff[3] = t_3;
	coeff[4] = t_4;
	coeff[5] = t_5;
	coeff[6] = t_6;
	//if (t_6 == 0) {
	if (abs(t_6) < std::numeric_limits<float>::min()) {
		Eigen::VectorXf coeff2(6);
		coeff2[0] = t_0;
		coeff2[1] = t_1;
		coeff2[2] = t_2;
		coeff2[3] = t_3;
		coeff2[4] = t_4;
		coeff2[5] = t_5;
		coeff = coeff2;
	}
	//cout << coeff << endl;
	//cout << "t6 = " << t_6 << endl;
	solver.compute(coeff);  //compute()�������ʽ������
	//cout << coeff.transpose() << endl;
	const Eigen::PolynomialSolver<float, Eigen::Dynamic>::RootsType& roots = solver.roots();
	//cout << "Roots: " << roots << endl;
	//vector<float> rootsReal;
	//for (int i = 0; i < roots.rows(); i++)
	//{
	//	//cout << "Root " << i << " = " << roots[i] << endl;
	//	if (abs(roots[i].imag()) < 1e-6)
	//	{
	//		rootsReal.push_back(roots[i].real());
	//	}
	//}

	float minDis = FLT_MAX;
	for (int i = 0; i < roots.rows(); i++) {
		float t = roots[i].real();
		//�ж�t�Ƿ�Ϊ��������NaN
		if (t == INFINITY || t == -INFINITY || isnan(t)) {
			cout<< "cccccccccccccccccccccccccccccccccccccccccc";
			return -1;
		}
		float x0 = (x - a6 * t + pow(a4, 2) * a6 * pow(t, 3) - pow(a4, 2) * pow(t, 2) * x + 2 * a1 * t * x + 2 * a2 * t * x - a3 * t * y - a5 * t * z - 2 * a1 * a6 * pow(t, 2) - 2 * a2 * a6 * pow(t, 2) + a3 * a7 * pow(t, 2) + a5 * a8 * pow(t, 2) - 4 * a1 * a2 * a6 * pow(t, 3) + 2 * a2 * a3 * a7 * pow(t, 3) + 2 * a1 * a5 * a8 * pow(t, 3) - a3 * a4 * a8 * pow(t, 3) - a4 * a5 * a7 * pow(t, 3) + 4 * a1 * a2 * pow(t, 2) * x - 2 * a2 * a3 * pow(t, 2) * y + a4 * a5 * pow(t, 2) * y - 2 * a1 * a5 * pow(t, 2) * z + a3 * a4 * pow(t, 2) * z) / (2 * a0 * t + 2 * a1 * t + 2 * a2 * t - pow(a3, 2) * pow(t, 2) - pow(a4, 2) * pow(t, 2) - pow(a5, 2) * pow(t, 2) - 2 * a0 * pow(a4, 2) * pow(t, 3) - 2 * a2 * pow(a3, 2) * pow(t, 3) - 2 * a1 * pow(a5, 2) * pow(t, 3) + 4 * a0 * a1 * pow(t, 2) + 4 * a0 * a2 * pow(t, 2) + 4 * a1 * a2 * pow(t, 2) + 8 * a0 * a1 * a2 * pow(t, 3) + 2 * a3 * a4 * a5 * pow(t, 3) + 1);

		float y0 = (y - a7 * t + pow(a5, 2) * a7 * pow(t, 3) - pow(a5, 2) * pow(t, 2) * y - a3 * t * x + 2 * a0 * t * y + 2 * a2 * t * y - a4 * t * z - 2 * a0 * a7 * pow(t, 2) - 2 * a2 * a7 * pow(t, 2) + a3 * a6 * pow(t, 2) + a4 * a8 * pow(t, 2) - 4 * a0 * a2 * a7 * pow(t, 3) + 2 * a2 * a3 * a6 * pow(t, 3) + 2 * a0 * a4 * a8 * pow(t, 3) - a4 * a5 * a6 * pow(t, 3) - a3 * a5 * a8 * pow(t, 3) - 2 * a2 * a3 * pow(t, 2) * x + a4 * a5 * pow(t, 2) * x + 4 * a0 * a2 * pow(t, 2) * y - 2 * a0 * a4 * pow(t, 2) * z + a3 * a5 * pow(t, 2) * z) / (2 * a0 * t + 2 * a1 * t + 2 * a2 * t - pow(a3, 2) * pow(t, 2) - pow(a4, 2) * pow(t, 2) - pow(a5, 2) * pow(t, 2) - 2 * a0 * pow(a4, 2) * pow(t, 3) - 2 * a2 * pow(a3, 2) * pow(t, 3) - 2 * a1 * pow(a5, 2) * pow(t, 3) + 4 * a0 * a1 * pow(t, 2) + 4 * a0 * a2 * pow(t, 2) + 4 * a1 * a2 * pow(t, 2) + 8 * a0 * a1 * a2 * pow(t, 3) + 2 * a3 * a4 * a5 * pow(t, 3) + 1);

		float z0 = (z - a8 * t + pow(a3, 2) * a8 * pow(t, 3) - pow(a3, 2) * pow(t, 2) * z - a5 * t * x - a4 * t * y + 2 * a0 * t * z + 2 * a1 * t * z - 2 * a0 * a8 * pow(t, 2) - 2 * a1 * a8 * pow(t, 2) + a4 * a7 * pow(t, 2) + a5 * a6 * pow(t, 2) - 4 * a0 * a1 * a8 * pow(t, 3) + 2 * a0 * a4 * a7 * pow(t, 3) + 2 * a1 * a5 * a6 * pow(t, 3) - a3 * a4 * a6 * pow(t, 3) - a3 * a5 * a7 * pow(t, 3) - 2 * a1 * a5 * pow(t, 2) * x + a3 * a4 * pow(t, 2) * x - 2 * a0 * a4 * pow(t, 2) * y + a3 * a5 * pow(t, 2) * y + 4 * a0 * a1 * pow(t, 2) * z) / (2 * a0 * t + 2 * a1 * t + 2 * a2 * t - pow(a3, 2) * pow(t, 2) - pow(a4, 2) * pow(t, 2) - pow(a5, 2) * pow(t, 2) - 2 * a0 * pow(a4, 2) * pow(t, 3) - 2 * a2 * pow(a3, 2) * pow(t, 3) - 2 * a1 * pow(a5, 2) * pow(t, 3) + 4 * a0 * a1 * pow(t, 2) + 4 * a0 * a2 * pow(t, 2) + 4 * a1 * a2 * pow(t, 2) + 8 * a0 * a1 * a2 * pow(t, 3) + 2 * a3 * a4 * a5 * pow(t, 3) + 1);

		float ax = a6 + 2 * a0 * x0 + a3 * y0 + a5 * z0;

		float ay = a7 + a3 * x0 + 2 * a1 * y0 + a4 * z0;

		float az = a8 + a5 * x0 + a4 * y0 + 2 * a2 * z0;

		float dis = t * sqrt((ax * ax + ay * ay + az * az));
		if (abs(dis) < abs(minDis)) {
			minDis = dis;
		}
	}
	//cout << "minDis: " << minDis << endl;
	return minDis;
}

void Rui::PtsCoorTrans(Eigen::MatrixXf surfPts, Eigen::MatrixXf& tranSurfPts, Eigen::Matrix4f& TransMat) {
	//������淨��
	MatrixXf Pts = surfPts;
	//svd�ֽ�
	JacobiSVD<MatrixXf> svd(Pts, ComputeThinU | ComputeThinV);
	MatrixXf V = svd.matrixV();
	Vector3f N = V.col(3);
	N.normalize();
	//����������
	Vector3f PtsCenter = Vector3f::Zero();
	for (int i = 0; i < surfPts.rows(); i++) {
		PtsCenter += (surfPts.block(i, 0, 1, 3)).transpose();
	}
	PtsCenter /= surfPts.rows();
	//����ͶӰ
	MatrixXf ProjectPts(surfPts.rows(), 3);
	for (int i = 0; i < surfPts.rows(); i++) {
		Vector3f tempPt = (surfPts.block(i, 0, 1, 3)).transpose();
		double dis = (-(tempPt.transpose() * N - PtsCenter.transpose() * N))[0];
		//cout << dis << endl;
		Vector3f ProjectPt = tempPt + dis * N;
		ProjectPts.block(i, 0, 1, 3) = ProjectPt.transpose();
	}
	//cout << "ProjectPts:\n " << ProjectPts << endl;
	//��ProjectPts��Э�������
	MatrixXf ProjectPtsNormlize = ProjectPts;
	for (int i = 0; i < ProjectPtsNormlize.rows(); i++) {
		ProjectPtsNormlize.block(i, 0, 1, 3) = ProjectPtsNormlize.block(i, 0, 1, 3) - PtsCenter.transpose();
	}
	MatrixXf ProjectPtsCov = ProjectPtsNormlize.transpose() * ProjectPtsNormlize;
	//���Э������������ֵ����������
	SelfAdjointEigenSolver<MatrixXf> eig(ProjectPtsCov);
	MatrixXf eigVec = eig.eigenvectors();
	//cout << "eigVec:\n " << eigVec << endl;
	Vector3f Rz = eigVec.col(0);
	Vector3f Ry = eigVec.col(1);
	Vector3f Rx = Ry.cross(Rz);
	Matrix4f T;
	T << Rx, Ry, Rz, PtsCenter, 0, 0, 0, 1;
	//cout << T << endl;
	MatrixXf PtsTrans = Pts * (T.inverse()).transpose();
	//
	//cout << "PtsTrans:\n " << PtsTrans << endl;
	TransMat = T.inverse();
	tranSurfPts = PtsTrans;
}

void Rui::PtsCoorTrans2(Eigen::MatrixXf surfPts, Eigen::MatrixXf& tranSurfPts, Eigen::Matrix4f& TransMat) {
	MatrixXf Pts;
	if (surfPts.cols() == 3) {
		Pts = surfPts;
	}
	else if (surfPts.cols() == 4)
	{
		Pts = surfPts.block(0, 0, surfPts.rows(), 3);
	}
	else {
		cout << "surfPts error!" << endl;
		return;
	}
	//������Ƶ�����
	Vector3f PtsCenter = Vector3f::Zero();
	for (int i = 0; i < Pts.rows(); i++) {
		PtsCenter += Pts.block(i, 0, 1, 3).transpose();
	}
	PtsCenter = PtsCenter / Pts.rows();
	//cout << "surfPts:\n " << surfPts << endl;
	//cout << "PtsCenter:\n " << PtsCenter << endl;
	
	//�����һ����������
	MatrixXf PtsNormlize = Pts;
	for (int i = 0; i < Pts.rows(); i++) {
		Vector3f tempPt = Pts.block(i, 0, 1, 3).transpose();
		PtsNormlize.block(i, 0, 1, 3) = (tempPt - PtsCenter).transpose();
	}
	//SVD�ֽ�
	JacobiSVD<MatrixXf> svd(PtsNormlize, ComputeThinU | ComputeThinV);
	MatrixXf V = svd.matrixV();
	Matrix3f RotateMat = V;

	Matrix4f T;
	T << RotateMat, PtsCenter, 0, 0, 0, 1;
	MatrixXf Pts_homo = MatrixXf::Zero(surfPts.rows(), 4);
	Pts_homo << Pts, MatrixXf::Ones(surfPts.rows(), 1);
	MatrixXf PtsTrans_homo = Pts_homo * (T.inverse()).transpose();
	MatrixXf PtsTrans = PtsTrans_homo.block(0, 0, PtsTrans_homo.rows(), 3);

	//MatrixXf PtsTrans = Pts * (T.inverse()).transpose();
	
	//cout << "PtsTrans:\n " << PtsTrans << endl;
	//cout << "T:\n " << T << endl;
	TransMat = T.inverse();
	tranSurfPts = PtsTrans;
}

void Rui::QuadricSurfFittingAll(Eigen::MatrixXf ptsMat, Eigen::VectorXf& quaSurfCoeffs, std::vector<double>& fitErrors) {
	//����任
	Eigen::MatrixXf surfPtsTrans;
	Eigen::Matrix4f TransMat;
	PtsCoorTrans2(ptsMat, surfPtsTrans, TransMat);
	Eigen::VectorXf tempSurfCoeffs;
	//cout << "initial Pts.size:\n" << Pts.rows() << endl;
	//�����������
	QuadricSurfFitting(surfPtsTrans, tempSurfCoeffs, fitErrors);
	//ϵ���任
	Eigen::VectorXf dstSurfCoeffs;
	QuadricCoeffsTrans(tempSurfCoeffs, TransMat, dstSurfCoeffs);
	dstSurfCoeffs.normalize();
	dstSurfCoeffs = dstSurfCoeffs * 1e3;
	quaSurfCoeffs = dstSurfCoeffs;
}

void Rui::QuadricSurfFitting(Eigen::MatrixXf surfPts, Eigen::VectorXf& quaSurfCoeffs, std::vector<double>& fitErrors) {
	Eigen::ArrayXf X_a = (surfPts.col(0)).array();
	Eigen::ArrayXf Y_a = (surfPts.col(1)).array();
	Eigen::ArrayXf Z_a = (surfPts.col(2)).array();
	Eigen::MatrixXf A = Eigen::MatrixXf::Ones(surfPts.rows(), 6);
	A.col(1) = X_a;
	A.col(2) = Y_a;
	A.col(3) = X_a * X_a;
	A.col(4) = X_a * Y_a;
	A.col(5) = Y_a * Y_a;
	Eigen::MatrixXf B = Z_a;
	Eigen::MatrixXf X = (A.transpose() * A).inverse() * A.transpose() * B;
	quaSurfCoeffs = X;
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

void Rui::QuadricCoeffsTrans(Eigen::VectorXf quaSurfCoeffs, Eigen::Matrix4f TransMat, Eigen::VectorXf& quaSurfCoeffsTrans) {
	Eigen::MatrixXf T = TransMat;
	Eigen::VectorXf coeff = quaSurfCoeffs;
	//cout << T << endl;
	float c1 = coeff(0);
	float c2 = coeff(1);
	float c3 = coeff(2);
	float c4 = coeff(3);
	float c5 = coeff(4);
	float c6 = coeff(5);
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
	float cx2 = c4 * r11 * r11 + c5 * r11 * r21 + c6 * r21 * r21;
	float cy2 = c4 * r12 * r12 + c5 * r12 * r22 + c6 * r22 * r22;
	float cz2 = c4 * r13 * r13 + c5 * r13 * r23 + c6 * r23 * r23;
	float cxy = 2 * c4 * r11 * r12 + c5 * r21 * r12 + c5 * r11 * r22 + 2 * c6 * r21 * r22;
	float cyz = 2 * c4 * r12 * r13 + c5 * r22 * r13 + c5 * r12 * r23 + 2 * c6 * r22 * r23;
	float cxz = 2 * c4 * r11 * r13 + c5 * r21 * r13 + c5 * r11 * r23 + 2 * c6 * r21 * r23;
	float cx = c2 * r11 - r31 + c3 * r21 + 2 * c4 * r11 * t1 + c5 * r21 * t1 + c5 * r11 * t2 + 2 * c6 * r21 * t2;
	float cy = c2 * r12 - r32 + c3 * r22 + 2 * c4 * r12 * t1 + c5 * r22 * t1 + c5 * r12 * t2 + 2 * c6 * r22 * t2;
	float cz = c2 * r13 - r33 + c3 * r23 + 2 * c4 * r13 * t1 + c5 * r23 * t1 + c5 * r13 * t2 + 2 * c6 * r23 * t2;
	float cc = c4 * t1 * t1 + c5 * t1 * t2 + c2 * t1 + c6 * t2 * t2 + c3 * t2 + c1 - t3;
	Eigen::VectorXf coeffQua = Eigen::VectorXf::Zero(10, 1);
	coeffQua << cx2, cy2, cz2, cxy, cyz, cxz, cx, cy, cz, cc;
	//cout << "coeffQua:\n " << coeffQua << endl;
	quaSurfCoeffsTrans = coeffQua;
}

void Rui::GetQuadricSurfCoeffs(std::vector<Eigen::Vector3f> surfPts, Eigen::VectorXf& quaSurfCoeffs) {
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
	Eigen::VectorXf tempSurfCoeffs;
	vector<double> fitErrors;
	//cout << "initial Pts.size:\n" << Pts.rows() << endl;
	//�����������
	QuadricSurfFitting(surfPtsTrans, tempSurfCoeffs, fitErrors);
	//ȥ���ӵ㣬�����һ��
	double errorThr = 0.1;
	std::vector<Eigen::Vector3f> newSurfPts;
	for (int i = 0; i < surfPtsTrans.rows(); i++) {
		if (abs(fitErrors[i]) < errorThr) {
			newSurfPts.push_back(Eigen::Vector3f(surfPtsTrans(i, 0), surfPtsTrans(i, 1), surfPtsTrans(i, 2)));
		}
	}
	Eigen::MatrixXf newSurfPtsTrans = Eigen::MatrixXf::Ones(newSurfPts.size(), 4);
	for (int i = 0; i < newSurfPts.size(); i++) {
		newSurfPtsTrans(i, 0) = newSurfPts[i](0);
		newSurfPtsTrans(i, 1) = newSurfPts[i](1);
		newSurfPtsTrans(i, 2) = newSurfPts[i](2);
	}
	//cout << "newSurfPts.size:\n" << newSurfPts.size() << endl;
	vector<double> fitErrors2;
	QuadricSurfFitting(newSurfPtsTrans, tempSurfCoeffs, fitErrors2);
	//cout << "ϵ���任ǰ: " << tempSurfCoeffs << endl;

	//ϵ���任
	Eigen::VectorXf dstSurfCoeffs;
	QuadricCoeffsTrans(tempSurfCoeffs, TransMat, dstSurfCoeffs);
	//cout << "ϵ���任��: " << dstSurfCoeffs << endl;
	quaSurfCoeffs = dstSurfCoeffs;
	cout << surfPts.size() << "         " << *max_element(fitErrors.begin(), fitErrors.end()) << "         " << std::accumulate(fitErrors.begin(), fitErrors.end(), 0.0) / fitErrors.size() << "         " << newSurfPts.size() << "           " << *max_element(fitErrors2.begin(), fitErrors2.end()) << "           " << std::accumulate(fitErrors2.begin(), fitErrors2.end(), 0.0) / fitErrors2.size() << endl;
	//cout << surfPts.size() << "         " << *max_element(fitErrors.begin(), fitErrors.end()) << "         " << std::accumulate(fitErrors.begin(), fitErrors.end(), 0.0) / fitErrors.size() << endl;
}

void Rui::GetQuadricSurfCoeffs2(std::vector<Eigen::Vector3f> surfPts, Eigen::VectorXf& quaSurfCoeffs, Eigen::VectorXf& TransVec) {
	MatrixXf Pts(surfPts.size(), 3);
	for (int i = 0; i < surfPts.size(); i++) {
		Pts(i, 0) = surfPts[i](0);
		Pts(i, 1) = surfPts[i](1);
		Pts(i, 2) = surfPts[i](2);
	}
	//����任
	Eigen::MatrixXf surfPtsTrans;
	Eigen::Matrix4f TransMat;
	Rui::PtsCoorTrans2(Pts, surfPtsTrans, TransMat);
	Eigen::VectorXf tempSurfCoeffs;
	vector<double> fitErrors;
	//cout << "initial Pts.size:\n" << Pts.rows() << endl;
	//�����������
	Rui::QuadricSurfFitting(surfPtsTrans, tempSurfCoeffs, fitErrors);
	//ȥ���ӵ㣬�����һ��
	double errorThr = 0.1;
	std::vector<Eigen::Vector3f> newSurfPts;
	for (int i = 0; i < surfPtsTrans.rows(); i++) {
		if (abs(fitErrors[i]) < errorThr) {
			newSurfPts.push_back(Eigen::Vector3f(surfPtsTrans(i, 0), surfPtsTrans(i, 1), surfPtsTrans(i, 2)));
		}
	}
	Eigen::MatrixXf newSurfPtsTrans = Eigen::MatrixXf::Ones(newSurfPts.size(), 4);
	for (int i = 0; i < newSurfPts.size(); i++) {
		newSurfPtsTrans(i, 0) = newSurfPts[i](0);
		newSurfPtsTrans(i, 1) = newSurfPts[i](1);
		newSurfPtsTrans(i, 2) = newSurfPts[i](2);
	}
	//cout << "newSurfPts.size:\n" << newSurfPts.size() << endl;
	vector<double> fitErrors2;
	QuadricSurfFitting(newSurfPtsTrans, tempSurfCoeffs, fitErrors2);
	quaSurfCoeffs = tempSurfCoeffs;
	TransVec = Eigen::VectorXf::Zero(6, 1);
	Vector3f Rvec, tvec;
	//��ת����ת��ת����
	Matrix3f R = TransMat.block(0, 0, 3, 3);
	Eigen::AngleAxisf angleAxis(R);
	tvec = TransMat.block(0, 3, 3, 1);
	Rvec = angleAxis.angle() * angleAxis.axis();
	TransVec << Rvec, tvec;

	cout << surfPts.size() << "         " << *max_element(fitErrors.begin(), fitErrors.end()) << "         " << std::accumulate(fitErrors.begin(), fitErrors.end(), 0.0) / fitErrors.size() << "         " << newSurfPts.size() << "           " << *max_element(fitErrors2.begin(), fitErrors2.end()) << "           " << std::accumulate(fitErrors2.begin(), fitErrors2.end(), 0.0) / fitErrors2.size() << endl;
	//cout << surfPts.size() << "         " << *max_element(fitErrors.begin(), fitErrors.end()) << "         " << std::accumulate(fitErrors.begin(), fitErrors.end(), 0.0) / fitErrors.size() << endl;
}

void Rui::GetQuadricSurfIntersectPt(Eigen::VectorXf quaSurfCoeffs, Eigen::VectorXf camRay, Eigen::Vector3f& intersectPt) {
	float k1 = camRay(3) / camRay(5);
	float k2 = camRay(4) / camRay(5);
	float b1 = camRay(0) - k1 * camRay(2);
	float b2 = camRay(1) - k2 * camRay(2);
	VectorXf c = VectorXf::Zero(11, 1);
	c << 0, quaSurfCoeffs;
	//cout << c << endl;
	float coeff2 = c(1) * k1 * k1 + c(4) * k1 * k2 + c(6) * k1 + c(2) * k2 * k2 + c(5) * k2 + c(3);
	float coeff1 = c(9) + b1 * c(6) + b2 * c(5) + c(7) * k1 + c(8) * k2 + 2 * b1 * c(1) * k1 + 2 * b2 * c(2) * k2 + b1 * c(4) * k2 + b2 * c(4) * k1;
	float coeff0 = c(1) * b1 * b1 + c(4) * b1 * b2 + c(7) * b1 + c(2) * b2 * b2 + c(8) * b2 + c(10);
	if (coeff2 < 0) {
		coeff0 = -coeff0;
		coeff1 = -coeff1;
		coeff2 = -coeff2;
	}
	//coeff2 = 0.000000001376634;
	//coeff1 = 0.404588528577078;
	//coeff0 = -147.244360893542222;
	//cout << coeff2 << '\t';
	//cout << coeff1 << '\t';
	//cout << coeff0 << '\t';
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
	//cout << z1 << '\t';
	//cout << z2 << endl;
	float z;
	if (z2 > 0 && z2 < 1e4) {
		//cout << "choose z2222222222" << endl;
		z = z2;
	}
		
	else if (z1 > 0 && z1 < 1e4) {
		//cout << "choose z1" << endl;
		z = z1;
	}
	else {
		cout << z1 << endl;
		cout << z2 << endl;
		cout << "z1, z2 error!" << endl;
		return;
	}
	//if (z1 < 0)
	//	z1 = z2;
	//z = z2;
	//cout << coeff0 + coeff1 * z + coeff2 * z * z << endl;

	float x = k1 * z + b1;
	float y = k2 * z + b2;
	Vector3f dstPt = Vector3f(x, y, z);
	intersectPt = dstPt;
}

void Rui::GetQuadricSurfIntersectPtNewForm(Eigen::VectorXf quardricSurfCoeffs, Eigen::VectorXf camRay, Eigen::Vector3f& intersectPt, double maxZ, double minZ) {
	float k1 = camRay(3) / camRay(5);
	float k2 = camRay(4) / camRay(5);
	float b1 = camRay(0) - k1 * camRay(2);
	float b2 = camRay(1) - k2 * camRay(2);
	VectorXf c = VectorXf::Zero(11, 1);
	c << 0, quardricSurfCoeffs;
	//cout << c << endl;
	float coeff2 = c(1) * k1 * k1 + c(4) * k1 * k2 + c(6) * k1 + c(2) * k2 * k2 + c(5) * k2 + c(3);
	float coeff1 = c(9) + b1 * c(6) + b2 * c(5) + c(7) * k1 + c(8) * k2 + 2 * b1 * c(1) * k1 + 2 * b2 * c(2) * k2 + b1 * c(4) * k2 + b2 * c(4) * k1;
	float coeff0 = c(1) * b1 * b1 + c(4) * b1 * b2 + c(7) * b1 + c(2) * b2 * b2 + c(8) * b2 + c(10);
	if (coeff2 < 0) {
		coeff0 = -coeff0;
		coeff1 = -coeff1;
		coeff2 = -coeff2;
	}
	if (abs(coeff2) < 1e-10) {
		float z = -coeff0 / coeff1;
		float x = k1 * z + b1;
		float y = k2 * z + b2;
		intersectPt = Vector3f(x, y, z);
		return;
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
	
	float z = z1;
	if (z2 >= minZ - 10 && z2 <= maxZ + 10 && abs(z2) < abs(z1)) {
		z = z2;
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



