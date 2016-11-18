#include "triangulation.h"
#include "tool.h"

using namespace std;
using namespace Eigen;
using namespace cv;

Vector4f triangulation::triangulate(Point2i ptInLeftImg, Point2i ptInRightImg)
{
	Matrix4f A;
	A.block<1, 4>(0,0) = ptInLeftImg.x * camera_matrix_1.block<1, 4>(2, 0)- camera_matrix_1.block<1, 4>(0,0);
	A.block<1, 4>(1,0) = ptInLeftImg.y * camera_matrix_1.block<1, 4>(2, 0)- camera_matrix_1.block<1, 4>(1,0);
	A.block<1, 4>(2,0) = ptInRightImg.x * camera_matrix_2.block<1, 4>(2, 0)- camera_matrix_2.block<1, 4>(0,0);
	A.block<1, 4>(3,0) = ptInRightImg.y * camera_matrix_2.block<1, 4>(2, 0)- camera_matrix_2.block<1, 4>(1,0);

	JacobiSVD<MatrixXf> svdOfA(A, ComputeThinV);
	const Eigen::MatrixXf V = svdOfA.matrixV();

	Vector4f X = V.block<4, 1>(0,3);
	X = X / X(3);

	return X.transpose(); 
}

void triangulation::pc_triangulate(vector<Point2f> &keypoionts_1, vector<Point2f> &keypoionts_2, vector<Point3f> &point_cloud)
{
    MatrixXf pc(keypoionts_1.size(), 4);
    
    for (int i=0; i<keypoionts_1.size(); i++)
        pc.block<1, 4>(i, 0) = triangulate(keypoionts_1[i], keypoionts_2[i]);
    
    removeColumn(pc, 3);
    
    //for (int i=0; i<pc.rows(); i++){
    //    if (( abs(pc(i, 0)) >= 50 )
    //        || ( abs(pc(i, 1)) >= 50 )
    //        || ( abs(pc(i, 2)) >= 50 ) ){
    //        removeRow(pc, i);
    //        keypoints2_l.erase(keypoints2_l.begin() + i);
    //        i--;
    //    }
    //}
    
    for (int i=0; i<pc.rows(); i++){
        Point3f p(pc(i, 0), pc(i, 1), pc(i, 2));
        point_cloud.push_back(p);
    }
    
}
