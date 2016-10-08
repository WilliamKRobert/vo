#include "cal_pose.h"

using namespace Eigen;

Vector4f triangulation(Point2i ptInLeftImg, Point2i ptInRightImg, MatrixXf P1, MatrixXf P2)
{
	Matrix4f A;
	A.block<1, 4>(0,0) = ptInLeftImg.x * P1.block<1, 4>(2, 0)- P1.block<1, 4>(0,0);
	A.block<1, 4>(1,0) = ptInLeftImg.y * P1.block<1, 4>(2, 0)- P1.block<1, 4>(1,0);
	A.block<1, 4>(2,0) = ptInRightImg.x * P2.block<1, 4>(2, 0)- P2.block<1, 4>(0,0);
	A.block<1, 4>(3,0) = ptInRightImg.y * P2.block<1, 4>(2, 0)- P2.block<1, 4>(1,0);

	JacobiSVD<MatrixXf> svdOfA(A, ComputeThinV);
	const Eigen::MatrixXf V = svdOfA.matrixV();

	Vector4f X = V.block<4, 1>(0,3);
	X = X / X(3);

	return X.transpose(); 
}
