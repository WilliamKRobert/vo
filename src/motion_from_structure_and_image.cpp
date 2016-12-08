#include <opencv2/calib3d/calib3d.hpp>

#include "tool.h"
#include "feature_tracker.h"
#include "triangulation.h"
#include "motion_estimator.h"
#include "point_cloud_alignment.h"

using namespace Eigen;
#include <opencv2/core/eigen.hpp>


void motionEstimator::motionFromStructureAndImage::updatePose(const std::vector<Point3f> &point_cloud, const std::vector<Point2f> &points, Mat &R, Mat &t)
{
	Mat rvec = cv::Mat::zeros(3, 1, CV_64F);
    t = cv::Mat::zeros(3, 1, CV_64F);
    
    cv::Mat distCoeffs = cv::Mat::zeros(4, 1, CV_64FC1);
    
    solvePnPRansac(point_cloud, points, intrinsic_matrix, distCoeffs, rvec, t, useExtrinsicGuess, iterationsCount,
                   reprojectionError, confidence);
    
	rvec = -rvec;
    Rodrigues(rvec,R);
	t = -t;
}

//void stereo_pose(Mat img1_l, Mat img1_r, Mat img2_l, Mat img2_r, MatrixXf &pointCloud_1, MatrixXf &pointCloud_2, MatrixXf P1, MatrixXf P2, Mat &R, Mat &t)
//{
//	vector<Point2f> M1_l, M1_r, M2_l, M2_r;
//	
//	featureMatching(img1_l, img1_r, M1_l, M1_r);
//	featureMatching(img2_l, img2_r, M2_l, M2_r);
//
//    pointCloud_1.resize(M1_l.size(), 4);
//    pointCloud_2.resize(M2_l.size(), 4);
//
//	for (int i=0; i<M1_l.size(); i++){
//        pointCloud_1.block<1, 4>(i, 0) = triangulation(M1_l[i], M1_r[i], P1, P2);
//    }
//
//	for (int i=0; i<M2_l.size(); i++){
//    	pointCloud_2.block<1, 4>(i, 0) = triangulation(M2_l[i], M2_r[i], P1, P2);
//    }
//
//	int MRow = M1_l.size();
//	removeColumn(pointCloud_1, 3);
//	removeColumn(pointCloud_2, 3);
//    
//	for (int i=0; i<pointCloud_1.rows(); i++){
//		if ( (abs(pointCloud_1(i, 0)) >= 50 ) 
//		|| ( abs(pointCloud_1(i, 1)) >= 50 ) 
//		|| ( abs(pointCloud_1(i, 2)) >= 50 )){ 
//			removeRow(pointCloud_1, i);
//			i--;
//		}
//	}
//	for (int i=0; i<pointCloud_2.rows(); i++){
//		if ( (abs(pointCloud_2(i, 0)) >= 50 ) 
//		|| ( abs(pointCloud_2(i, 1)) >= 50 ) 
//		|| ( abs(pointCloud_2(i, 2)) >= 50 )){ 
//			removeRow(pointCloud_2, i);
//			i--;
//		}
//	}
//    
//	pointCloudAlign_sac(pointCloud_1, pointCloud_2, R, t);
//	return; 	
//}
//
//void stereo_pose(Mat img2_l, Mat img2_r, MatrixXf &pointCloud_1, MatrixXf &pointCloud_2, MatrixXf P1, MatrixXf P2, Mat &R, Mat &t)
//{
//	clock_t begin = clock();
//	clock_t end = clock();
//
//	vector<Point2f> M2_l, M2_r;
//    
//	featureMatching(img2_l, img2_r, M2_l, M2_r);
//    end = clock();
//    double elapsed_secs = double( end - begin) / CLOCKS_PER_SEC;
//    begin = end;
//    //cout << " featureMatching :" << elapsed_secs << "s" <<endl;
//	
//    //MatrixXf pointCloud_1(M1_l.size(), 4);
//    //MatrixXf pointCloud_2(M2_l.size(), 4);
//    
//    pointCloud_2.resize(M2_l.size(), 4);
//    
//    for (int i=0; i<M2_l.size(); i++)
//		pointCloud_2.block<1, 4>(i, 0) = triangulation(M2_l[i], M2_r[i], P1, P2);
//    
//    
//	end = clock();
//	elapsed_secs = double( end - begin) / CLOCKS_PER_SEC;
//	begin = end;
//	//cout << " 3d point time :" << elapsed_secs << "s" <<endl;
//
//    int MRow = M2_l.size();
//    removeColumn(pointCloud_2, 3);
//    
//    for (int i=0; i<pointCloud_2.rows(); i++){
//        if (( abs(pointCloud_2(i, 0)) >= 50 ) 
//		 || ( abs(pointCloud_2(i, 1)) >= 50 ) 
//		 || ( abs(pointCloud_2(i, 2)) >= 50 ) ){
//            removeRow(pointCloud_2, i);
//            i--;
//        }
//    }
//    
//	pointCloudAlign_sac(pointCloud_1, pointCloud_2, R, t);
//
//    end = clock();
//    elapsed_secs = double( end - begin) / CLOCKS_PER_SEC;
//	begin = end;
//	//cout << "Pose estimation time :" << elapsed_secs << "s" <<endl;
//	//cout <<endl;
//
//}

