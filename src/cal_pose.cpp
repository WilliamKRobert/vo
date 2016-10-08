#include <fstream>
#include <cmath>

#include <opencv2/calib3d/calib3d.hpp>
#include <opengv/point_cloud/methods.hpp>
#include <opengv/point_cloud/PointCloudAdapter.hpp>
#include <opengv/sac/Ransac.hpp>
#include <opengv/sac_problems/point_cloud/PointCloudSacProblem.hpp>

#include "cal_pose.h" 

using namespace Eigen;
using namespace opengv;
#include <opencv2/core/eigen.hpp>


int cal_pose(Mat img1_l, Mat img1_r, Mat img2_l, Mat img2_r, MatrixXf P1, MatrixXf P2)
{
    // stereo map
    int numberOfDisparities = 128;
    int SADWindowSize = 9;
    float scale = 1.0;
    bool no_display = false;

	// CV_16S
    Mat disparityMap1_16s, disparityMap2_16s;
	// CV_32F
	Mat disparityMap1, disparityMap2;

    stereo_matching(disparityMap1_16s, img1_l, img1_r, STEREO_SGBM, numberOfDisparities, SADWindowSize, scale, no_display);
    stereo_matching(disparityMap2_16s, img2_l, img2_r, STEREO_SGBM, numberOfDisparities, SADWindowSize, scale, no_display);
	
	disparityMap1_16s.convertTo(disparityMap1, CV_32F, 1./16);
	disparityMap2_16s.convertTo(disparityMap2, CV_32F, 1./16);

	//cout <<disparityMap1.type() <<endl;
	//for (int i=10; i<11; i++)
	//	for (int j=100; j<300; j++)
	//		cout <<disparityMap1.at<float>(i,j) <<" ";
	//cout <<endl;
	//return 0;
	
    // bucket features
    int row = img1_l.rows, col = img1_l.cols;
    int h_break= 30, b_break= 40;
    int maxPerCell= 10;

    vector<KeyPoint> prevKeyPts;
    bucket_features(img1_l, prevKeyPts, row, col, h_break, b_break, maxPerCell);

    Mat img_keypoints_1;
    drawKeypoints( img1_l, prevKeyPts, img_keypoints_1, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
    
	imshow("Keypoints in image 1", img_keypoints_1);

	// Comparison between bucketing and without bucketing
	//Ptr<FastFeatureDetector> fast1 = FastFeatureDetector::create();	
	//vector<KeyPoint> kps;
	//Mat img_tmp;
	//fast1->detect(img1_l, kps);
	//drawKeypoints( img1_l, kps, img_tmp, Scalar::all(-1), DrawMatchesFlags::DEFAULT);

	//imshow("Keypoints without bucketing", img_tmp);

    waitKey(0);
    
    // optical flow tracking     
    vector<uchar> status;
    vector<float> err;

    vector<Point2f> features_prev;
    vector<Point2f> features_next; 

    for (int i=0; i<prevKeyPts.size(); i++)
        features_prev.push_back(prevKeyPts[i].pt);

    calcOpticalFlowPyrLK(img1_l, img2_l, features_prev, features_next, status, err);
	
	for (int i=0; i<status.size(); i++){
        if (status[i] == 0){
            features_prev.erase(features_prev.begin()+i);
            features_next.erase(features_next.begin()+i);
            status.erase(status.begin()+i);
            i--; 
        }
    }
	
	//for (int i=0; i<features_prev.size(); i++)
	//	cout <<features_prev[i].x <<" " <<features_prev[i].y << " "<<features_next[i].x <<" " <<features_next[i].y <<endl;

	//return 0;	

	// visualize the results of optical flow tracking
	//for (int i=0; i<status.size(); i++){
	//	line(img1_l, features_prev[i], features_next[i], Scalar(255,0,0));
	//	circle(img1_l, features_prev[i], 1, Scalar(0,0,0), -1);
	//}
	//imshow("Optical Flow", img1_l);
	//waitKey(0);

	// triangulation of 3D point cloud
    vector<Point2f> M1_l(features_prev.begin(), features_prev.end());
    vector<Point2f> M2_l(features_next.begin(), features_next.end());
    vector<Point2f> M1_r(M1_l);
    vector<Point2f> M2_r(M2_l);
	
	// It seems that the stereo map computation algorithms in 
	// OpenCV do not find or mark the occlusion points
	// Therefore we don't have to find and delete occlusion pts
    for (int i=0; i<M1_l.size(); i++){
		if (disparityMap1.at<float>(M1_l[i].y, M1_l[i].x)>0 && disparityMap1.at<float>(M1_l[i].y, M1_l[i].x) < 100 && disparityMap2.at<float>(M2_l[i].y, M2_l[i].x) > 0 && disparityMap2.at<float>(M2_l[i].y, M2_l[i].x) < 100){
    		M1_r[i].x = ( M1_l[i].x - disparityMap1.at<float>(M1_l[i].y, M1_l[i].x));
        	M2_r[i].x = ( M2_l[i].x - disparityMap2.at<float>(M2_l[i].y, M2_l[i].x));
		}
		else{
			M1_l.erase(M1_l.begin()+i);
			M1_r.erase(M1_r.begin()+i);
			M2_l.erase(M2_l.begin()+i);
			M2_r.erase(M2_r.begin()+i);
			i--;	
		}
    }    

	MatrixXf point3D_1(M1_l.size(), 4);
    MatrixXf point3D_2(M2_l.size(), 4);

	//  test triangulation: the output is correct
	//Point2i ptleft(1, 1);
	//Point2i ptright(10,8);
	//Vector4f position = triangulation(ptleft, ptright, P1, P2);
    //cout <<"First result: " <<position <<endl;
	//return 0;
    for (int i=0; i<M1_l.size(); i++){
        point3D_1.block<1, 4>(i, 0) = triangulation(M1_l[i], M1_r[i], P1, P2);
    }
	
	for (int i=0; i<M2_l.size(); i++){
    	point3D_2.block<1, 4>(i, 0) = triangulation(M2_l[i], M2_r[i], P1, P2);
    }

	int MRow = M1_l.size();
	removeColumn(point3D_1, 3);
	removeColumn(point3D_2, 3);

	cout <<point3D_1.rows() <<" " <<point3D_1.cols() <<endl;
	for (int i=0; i<point3D_1.rows(); i++){
		if ( (abs(point3D_1(i, 0)) >= 50 ) || ( abs(point3D_1(i, 1)) >= 50 ) || ( abs(point3D_1(i, 2)) >= 50 ) || ( abs(point3D_2(i, 0)) >= 50 ) || ( abs(point3D_2(i, 1)) >= 50 ) || ( abs(point3D_2(i, 2)) >= 50 ) ){
			removeRow(point3D_1, i);
			removeRow(point3D_2, i);
			i--;
		}
	}
	cout <<point3D_1.rows() <<" " <<point3D_1.cols() <<endl;

	ofstream output("pointCloud.txt");
	for (int i=0; i<point3D_1.rows(); i++){
		for (int j=0; j<point3D_1.cols(); j++){
			output<<point3D_1(i, j) <<" ";
		}
		output<< endl;
	}
	
    // absolute orientation problem
	// 2d point -> Rotation and translation 
	double focal = P1(0,0);
	Mat E, R, t, mask;
	Point2d pp(P1(0,2), P1(1, 2));
	
	E = findEssentialMat(features_next, features_prev, focal, pp, RANSAC, 0.999, 1.0, mask);
	recoverPose(E, features_next, features_prev, R, t, focal, pp, mask);
	cout <<R <<endl;
	cout <<t <<endl;
	return 0;	
    // 3d point -> computation of R and t
	points_t points1(point3D_1.rows()), points2(point3D_1.rows());
	for (int i=0; i<point3D_1.rows(); i++){
		point_t a(point3D_1(i, 0), point3D_1(i, 1), point3D_1(i, 2)), b(point3D_2(i, 0), point3D_2(i, 1), point3D_2(i, 2));
		points1[i] = a;
	   	points2[i] = b;	
	}

	point_cloud::PointCloudAdapter adapter(points1, points2); 
	// create a RANSAC object
	sac::Ransac<sac_problems::point_cloud::PointCloudSacProblem> ransac;
	// create the sample consensus problem
	//std::shared_ptr<sac_problems::point_cloud::PointCloudSacProblem>
	//     relposeproblem_ptr(
	//     new sac_problems::point_cloud::PointCloudSacProblem(adapter) );
	//// run ransac
	return 0;
//	ransac.sac_model_ = relposeproblem_ptr;
//	ransac.threshold_ = 0.1; //threshold;
//	ransac.max_iterations_ = 50;//maxIterations;
//	ransac.computeModel(0);
//	// return the result
//	transformation_t best_transformation =
//		ransac.model_coefficients_;
//
    return 0; 
}
