#include <fstream>
#include <cmath>

#include <opencv2/calib3d/calib3d.hpp>

#include <opengv/point_cloud/methods.hpp>
#include <opengv/point_cloud/PointCloudAdapter.hpp>
#include <opengv/sac/Ransac.hpp>
#include <opengv/sac_problems/point_cloud/PointCloudSacProblem.hpp>

#include <opengv/relative_pose/methods.hpp>
#include <opengv/relative_pose/CentralRelativeAdapter.hpp>
#include <opengv/sac_problems/relative_pose/CentralRelativePoseSacProblem.hpp>


#include "cal_pose.h" 
#include "tool.h"

using namespace Eigen;
using namespace opengv;
#include <opencv2/core/eigen.hpp>

void pointCloudAlign(MatrixXf pointCloud_1, MatrixXf pointCloud_2, Mat &R, Mat &t);
void pointCloudAlign_sac(MatrixXf pointCloud_1, MatrixXf pointCloud_2, Mat &R, Mat &t);


void stereo_pose(Mat img1_l, Mat img1_r, Mat img2_l, Mat img2_r, MatrixXf &pointCloud_1, MatrixXf &pointCloud_2, MatrixXf P1, MatrixXf P2, Mat &R, Mat &t)
{
	vector<Point2f> M1_l, M1_r, M2_l, M2_r;
	
	featureMatching(img1_l, img1_r, M1_l, M1_r);
	featureMatching(img2_l, img2_r, M2_l, M2_r);

    pointCloud_1.resize(M1_l.size(), 4);
    pointCloud_2.resize(M2_l.size(), 4);

	for (int i=0; i<M1_l.size(); i++){
        pointCloud_1.block<1, 4>(i, 0) = triangulation(M1_l[i], M1_r[i], P1, P2);
    }

	for (int i=0; i<M2_l.size(); i++){
    	pointCloud_2.block<1, 4>(i, 0) = triangulation(M2_l[i], M2_r[i], P1, P2);
    }

	int MRow = M1_l.size();
	removeColumn(pointCloud_1, 3);
	removeColumn(pointCloud_2, 3);
    
	for (int i=0; i<pointCloud_1.rows(); i++){
		if ( (abs(pointCloud_1(i, 0)) >= 50 ) 
		|| ( abs(pointCloud_1(i, 1)) >= 50 ) 
		|| ( abs(pointCloud_1(i, 2)) >= 50 )){ 
			removeRow(pointCloud_1, i);
			i--;
		}
	}
	for (int i=0; i<pointCloud_2.rows(); i++){
		if ( (abs(pointCloud_2(i, 0)) >= 50 ) 
		|| ( abs(pointCloud_2(i, 1)) >= 50 ) 
		|| ( abs(pointCloud_2(i, 2)) >= 50 )){ 
			removeRow(pointCloud_2, i);
			i--;
		}
	}
    
	pointCloudAlign_sac(pointCloud_1, pointCloud_2, R, t);
	return; 	
}

void stereo_pose(Mat img2_l, Mat img2_r, MatrixXf &pointCloud_1, MatrixXf &pointCloud_2, MatrixXf P1, MatrixXf P2, Mat &R, Mat &t)
{
	clock_t begin = clock();
	clock_t end = clock();

	vector<Point2f> M2_l, M2_r;
    
	featureMatching(img2_l, img2_r, M2_l, M2_r);
    end = clock();
    double elapsed_secs = double( end - begin) / CLOCKS_PER_SEC;
    begin = end;
    cout << " featureMatching :" << elapsed_secs << "s" <<endl;
	
    //MatrixXf pointCloud_1(M1_l.size(), 4);
    //MatrixXf pointCloud_2(M2_l.size(), 4);
    
    pointCloud_2.resize(M2_l.size(), 4);
    
    for (int i=0; i<M2_l.size(); i++)
		pointCloud_2.block<1, 4>(i, 0) = triangulation(M2_l[i], M2_r[i], P1, P2);
    
    
	end = clock();
	elapsed_secs = double( end - begin) / CLOCKS_PER_SEC;
	begin = end;
	cout << " 3d point time :" << elapsed_secs << "s" <<endl;

    int MRow = M2_l.size();
    removeColumn(pointCloud_2, 3);
    
    for (int i=0; i<pointCloud_2.rows(); i++){
        if (( abs(pointCloud_2(i, 0)) >= 50 ) 
		 || ( abs(pointCloud_2(i, 1)) >= 50 ) 
		 || ( abs(pointCloud_2(i, 2)) >= 50 ) ){
            removeRow(pointCloud_2, i);
            i--;
        }
    }
    
	pointCloudAlign_sac(pointCloud_1, pointCloud_2, R, t);

    end = clock();
    elapsed_secs = double( end - begin) / CLOCKS_PER_SEC;
	begin = end;
	cout << "Pose estimation time :" << elapsed_secs << "s" <<endl;
	cout <<endl;

}

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
	ransac.max_iterations_ = 5000;//maxIterations;
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
