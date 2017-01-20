//#include <eigen3/Eigen/Dense>
//#include <eigen3/Eigen/Eigen>
#include <opencv2/core/core.hpp>
#include <Eigen/Dense>
#include <Eigen/Eigen>


#include <opengv/point_cloud/methods.hpp>
#include <opengv/point_cloud/PointCloudAdapter.hpp>
#include <opengv/sac/Ransac.hpp>
#include <opengv/sac_problems/point_cloud/PointCloudSacProblem.hpp>

#include <opengv/relative_pose/methods.hpp>
#include <opengv/relative_pose/CentralRelativeAdapter.hpp>
#include <opengv/sac_problems/relative_pose/CentralRelativePoseSacProblem.hpp>


using namespace Eigen;
using namespace opengv;
using namespace cv;
#include <opencv2/core/eigen.hpp>

#include "point_cloud_alignment.h"


/*
 *  Point cloud alignment
 *  Input: Two 3d point cloud -> computation of R and t
 *  Output: Rotation and translation matrix R, t
*/
void pointCloudAlign(MatrixXf pointCloud_1, MatrixXf pointCloud_2, Mat &R, Mat &t)
{
    points_t points1(pointCloud_1.rows()), points2(pointCloud_2.rows());
    
    for (int i=0; i<pointCloud_1.rows(); i++){
        point_t a(pointCloud_1(i, 0), pointCloud_1(i, 1), pointCloud_1(i, 2));
        points1[i] = a;
    }
    for (int i=0; i<pointCloud_2.rows(); i++){
        point_t b(pointCloud_2(i, 0), pointCloud_2(i, 1), pointCloud_2(i, 2));
        points2[i] = b;
    }

	// Using non-linear optimation
    point_cloud::PointCloudAdapter adapter(points1, points2); 

    transformation_t threept_transformation = 
        point_cloud::threept_arun(adapter);

    transformation_t nonlinear_transformation = 
        point_cloud::optimize_nonlinear(adapter);

    for (int i=0; i<3; i++){
        t.at<float>(i, 0) = nonlinear_transformation(i, 3);

        for (int j=0; j<3; j++){
            R.at<float>(i, j) = nonlinear_transformation(i, j);
        }
    }
	
}

void pointCloudAlign_sac(MatrixXf pointCloud_1, MatrixXf pointCloud_2, Mat &R, Mat &t)
{
	points_t points1(pointCloud_1.rows()), points2(pointCloud_2.rows());
	
    for (int i=0; i<pointCloud_1.rows(); i++){
        point_t a(pointCloud_1(i, 0), pointCloud_1(i, 1), pointCloud_1(i, 2));
        points1[i] = a;
    }
    for (int i=0; i<pointCloud_2.rows(); i++){
        point_t b(pointCloud_2(i, 0), pointCloud_2(i, 1), pointCloud_2(i, 2));
        points2[i] = b;
    }
	
	// Using non-linear optimation
    point_cloud::PointCloudAdapter adapter(points1, points2); 

	// create a RANSAC object
	sac::Ransac<sac_problems::point_cloud::PointCloudSacProblem> ransac;
	
	// create the sample consensus problem
	std::shared_ptr<sac_problems::point_cloud::PointCloudSacProblem>
	     relposeproblem_ptr(
	     new sac_problems::point_cloud::PointCloudSacProblem(adapter) );
	
	// run ransac
	ransac.sac_model_ = relposeproblem_ptr;
	ransac.threshold_ = 1; //threshold;
	ransac.max_iterations_ = 1000;//maxIterations;
	ransac.computeModel(0);
	
	transformation_t best_transformation =
		ransac.model_coefficients_;

	// return the result
	for (int i=0; i<3; i++){
        t.at<float>(i, 0) = best_transformation(i, 3);

        for (int j=0; j<3; j++){
            R.at<float>(i, j) = best_transformation(i, j);
    	}
    }
}
