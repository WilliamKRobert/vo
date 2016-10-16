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


vector<Point2f> stereo_pose(Mat img1_l, Mat img1_r, Mat img2_l, Mat img2_r, MatrixXf P1, MatrixXf P2, Mat &R, Mat &t)
{
    // stereo map
    int numberOfDisparities = 128;
    int SADWindowSize = 9;
    float scale = 1.0;
    bool no_display = true;

	// CV_16S
    Mat disparityMap1_16s, disparityMap2_16s;
	// CV_32F
	Mat disparityMap1, disparityMap2;

    stereo_matching(disparityMap1_16s, img1_l, img1_r, STEREO_SGBM, numberOfDisparities, SADWindowSize, scale, no_display);
    stereo_matching(disparityMap2_16s, img2_l, img2_r, STEREO_SGBM, numberOfDisparities, SADWindowSize, scale, no_display);
	
	disparityMap1_16s.convertTo(disparityMap1, CV_32F, 1./16);
	disparityMap2_16s.convertTo(disparityMap2, CV_32F, 1./16);

    // bucket features
//    int row = img1_l.rows, col = img1_l.cols;
//    int h_break= 30, b_break= 40;
//    int maxPerCell= 10;
//
//    vector<KeyPoint> prevKeyPts;
//    bucket_features(img1_l, prevKeyPts, row, col, h_break, b_break, maxPerCell);
    
	vector<Point2f> features_prev, features_next;
    featureDetection(img1_l, features_prev);

    //Mat img_keypoints_1;
    //drawKeypoints( img1_l, prevKeyPts, img_keypoints_1, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
	//imshow("Keypoints in image 1", img_keypoints_1);

	// Comparison between bucketing and without bucketing
	//Ptr<FastFeatureDetector> fast1 = FastFeatureDetector::create();	
	//vector<KeyPoint> kps;
	//Mat img_tmp;
	//fast1->detect(img1_l, kps);
	//drawKeypoints( img1_l, kps, img_tmp, Scalar::all(-1), DrawMatchesFlags::DEFAULT);

	//imshow("Keypoints without bucketing", img_tmp);

    //waitKey(0);
	
    // optical flow tracking     
    vector<uchar> status;
    vector<float> err;

//    for (int i=0; i<prevKeyPts.size(); i++)
//        features_prev.push_back(prevKeyPts[i].pt);

	Size winSize=Size(21, 21);
	TermCriteria termcrit=TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01);
    calcOpticalFlowPyrLK(img1_l, img2_l, features_prev, features_next, status, err, winSize, 3, termcrit, 0, 0.001);
	
	for (int i=0; i<status.size(); i++){
        if (status[i] == 0 || features_next[i].x<0 || features_next[i].y<0){
            features_prev.erase(features_prev.begin()+i);
            features_next.erase(features_next.begin()+i);
            status.erase(status.begin()+i);
            i--; 
        }
    }

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
		if (disparityMap1.at<float>(M1_l[i].y, M1_l[i].x)>0
	&& disparityMap1.at<float>(M1_l[i].y, M1_l[i].x) < 100 
	&& disparityMap2.at<float>(M2_l[i].y, M2_l[i].x) > 0 
	&& disparityMap2.at<float>(M2_l[i].y, M2_l[i].x) < 100){
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

	// ===================================================
	//  test triangulation: the output is correct
	//Point2i ptleft(1, 1);
	//Point2i ptright(10,8);
	//Vector4f position = triangulation(ptleft, ptright, P1, P2);
    //cout <<"First result: " <<position <<endl;
	//return 0;
	// ===================================================
	
    for (int i=0; i<M1_l.size(); i++){
        point3D_1.block<1, 4>(i, 0) = triangulation(M1_l[i], M1_r[i], P1, P2);
    }
	
	for (int i=0; i<M2_l.size(); i++){
    	point3D_2.block<1, 4>(i, 0) = triangulation(M2_l[i], M2_r[i], P1, P2);
    }

	int MRow = M1_l.size();
	removeColumn(point3D_1, 3);
	removeColumn(point3D_2, 3);

	for (int i=0; i<point3D_1.rows(); i++){
		if ( (abs(point3D_1(i, 0)) >= 50 ) 
		|| ( abs(point3D_1(i, 1)) >= 50 ) 
		|| ( abs(point3D_1(i, 2)) >= 50 ) 
		|| ( abs(point3D_2(i, 0)) >= 50 ) 
		|| ( abs(point3D_2(i, 1)) >= 50 ) 
		|| ( abs(point3D_2(i, 2)) >= 50 ) ){
			removeRow(point3D_1, i);
			removeRow(point3D_2, i);
			i--;
		}
	}

	// ofstream output("pointCloud.txt");
	// for (int i=0; i<point3D_1.rows(); i++){
	// 	for (int j=0; j<point3D_1.cols(); j++){
	// 		output<<point3D_1(i, j) <<" ";
	// 	}
	// 	output<< endl;
	// }
	
    // absolute orientation problem
	// 2d point -> eotation and translation 
	double focal = P1(0,0);
	Mat E, mask;
	Point2d pp(P1(0,2), P1(1, 2));

    // 3d point -> computation of R and t
		
    points_t points1(point3D_1.rows()), points2(point3D_1.rows());
    for (int i=0; i<point3D_1.rows(); i++){
        point_t a(point3D_1(i, 0), point3D_1(i, 1), point3D_1(i, 2)), b(point3D_2(i, 0), point3D_2(i, 1), point3D_2(i, 2));
        points1[i] = a;
        points2[i] = b;	
    }

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
	
	//// create a RANSAC object
	//sac::Ransac<sac_problems::point_cloud::PointCloudSacProblem> ransac;
	//
	//// create the sample consensus problem
	//std::shared_ptr<sac_problems::point_cloud::PointCloudSacProblem>
	//     relposeproblem_ptr(
	//     new sac_problems::point_cloud::PointCloudSacProblem(adapter) );
	//
	//// run ransac
	//ransac.sac_model_ = relposeproblem_ptr;
	//ransac.threshold_ = 1; //threshold;
	//ransac.max_iterations_ = 5000;//maxIterations;
	//ransac.computeModel(0);
	//
	//// return the result
	//transformation_t best_transformation =
	//	ransac.model_coefficients_;
	//cout << ransac.model_coefficients_ <<endl;

    return features_next;
}

int stereo_pose(Mat img1_l, Mat img1_r, Mat img2_l, Mat img2_r, vector<Point2f> &features_prev, vector<Point2f> &features_next, MatrixXf P1, MatrixXf P2, Mat &R, Mat &t)
{
    // stereo map
    int numberOfDisparities = 128;
    int SADWindowSize = 9;
    float scale = 1.0;
    bool no_display = true;
    
	clock_t begin = clock();	

    // CV_16S
    Mat disparityMap1_16s, disparityMap2_16s;
    // CV_32F
    Mat disparityMap1, disparityMap2;
    
    stereo_matching(disparityMap1_16s, img1_l, img1_r, STEREO_BM, numberOfDisparities, SADWindowSize, scale, no_display);
    stereo_matching(disparityMap2_16s, img2_l, img2_r, STEREO_BM, numberOfDisparities, SADWindowSize, scale, no_display);
    
    disparityMap1_16s.convertTo(disparityMap1, CV_32F, 1./16);
    disparityMap2_16s.convertTo(disparityMap2, CV_32F, 1./16);
    imshow("dis map", disparityMap1); 
	clock_t end = clock();
	double elapsed_secs = double( end - begin) / CLOCKS_PER_SEC;
	begin = end;
	cout << " disparity time :" << elapsed_secs << "s" <<endl;


	vector<uchar> status;
	vector<float> err;

    Size winSize=Size(21, 21);
    TermCriteria termcrit=TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01);
    calcOpticalFlowPyrLK(img1_l, img2_l, features_prev, features_next, status, err, winSize, 3, termcrit, 0, 0.001);
    
    for (int i=0; i<status.size(); i++){
        if (status[i] == 0 || features_next[i].x<0 || features_next[i].y<0){
            features_prev.erase(features_prev.begin()+i);
            features_next.erase(features_next.begin()+i);
            status.erase(status.begin()+i);
            i--;
        }
    }
    
	end = clock();
    elapsed_secs = double( end - begin) / CLOCKS_PER_SEC;
	begin = end;
	cout << " tracking time :" << elapsed_secs << "s" <<endl;




    // triangulation of 3D point cloud
    vector<Point2f> M1_l(features_prev.begin(), features_prev.end());
    vector<Point2f> M2_l(features_next.begin(), features_next.end());
    vector<Point2f> M1_r(M1_l);
    vector<Point2f> M2_r(M2_l);
    
    // It seems that the stereo map computation algorithms in
    // OpenCV do not find or mark the occlusion points
    // Therefore we don't have to find and delete occlusion pts
    for (int i=0; i<M1_l.size(); i++){
        if (disparityMap1.at<float>(M1_l[i].y, M1_l[i].x)>0 
	&& disparityMap1.at<float>(M1_l[i].y, M1_l[i].x) < 100 
	&& disparityMap2.at<float>(M2_l[i].y, M2_l[i].x) > 0 
	&& disparityMap2.at<float>(M2_l[i].y, M2_l[i].x) < 100){
            M1_r[i].x = ( M1_l[i].x - disparityMap1.at<float>(M1_l[i].y, M1_l[i].x) );
            M2_r[i].x = ( M2_l[i].x - disparityMap2.at<float>(M2_l[i].y, M2_l[i].x) );
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
    
    for (int i=0; i<M1_l.size(); i++){
        point3D_1.block<1, 4>(i, 0) = triangulation(M1_l[i], M1_r[i], P1, P2);
		point3D_2.block<1, 4>(i, 0) = triangulation(M2_l[i], M2_r[i], P1, P2);

    }
    
	end = clock();
	elapsed_secs = double( end - begin) / CLOCKS_PER_SEC;
	begin = end;
	cout << " 3d point time :" << elapsed_secs << "s" <<endl;

    int MRow = M1_l.size();
    removeColumn(point3D_1, 3);
    removeColumn(point3D_2, 3);
    
    for (int i=0; i<point3D_1.rows(); i++){
        if ( (abs(point3D_1(i, 0)) >= 50 ) 
		 || ( abs(point3D_1(i, 1)) >= 50 ) 
		 || ( abs(point3D_1(i, 2)) >= 50 ) 
		 || ( abs(point3D_2(i, 0)) >= 50 ) 
		 || ( abs(point3D_2(i, 1)) >= 50 ) 
		 || ( abs(point3D_2(i, 2)) >= 50 ) ){
            removeRow(point3D_1, i);
            removeRow(point3D_2, i);
            i--;
        }
    }
    
    // absolute orientation problem
    // 2d point -> eotation and translation
    double focal = P1(0,0);
    Mat E, mask;
    Point2d pp(P1(0,2), P1(1, 2));
    
    // 3d point -> computation of R and t
    points_t points1(point3D_1.rows()), points2(point3D_1.rows());
    for (int i=0; i<point3D_1.rows(); i++){
        point_t a(point3D_1(i, 0), point3D_1(i, 1), point3D_1(i, 2)), b(point3D_2(i, 0), point3D_2(i, 1), point3D_2(i, 2));
        points1[i] = a;
        points2[i] = b;
    }
    
    point_cloud::PointCloudAdapter adapter(points1, points2);
    
    //transformation_t threept_transformation =
    //point_cloud::threept_arun(adapter);
    //
    //transformation_t nonlinear_transformation =
    //point_cloud::optimize_nonlinear(adapter);
    //
    //for (int i=0; i<3; i++){
    //    t.at<float>(i, 0) = nonlinear_transformation(i, 3);
    //    
    //    for (int j=0; j<3; j++){
    //        R.at<float>(i, j) = nonlinear_transformation(i, j);
    //    }
    //}
    
	
    // create a RANSAC object
    sac::Ransac<sac_problems::point_cloud::PointCloudSacProblem> ransac;
    
    // create the sample consensus problem
    std::shared_ptr<sac_problems::point_cloud::PointCloudSacProblem>
         relposeproblem_ptr(
         new sac_problems::point_cloud::PointCloudSacProblem(adapter) );
    
    // run ransac
    ransac.sac_model_ = relposeproblem_ptr;
    ransac.threshold_ = 0.1; //threshold;
    ransac.max_iterations_ = 50;//maxIterations;
    ransac.computeModel(0);
    
    // return the result
    transformation_t best_transformation =
    	ransac.model_coefficients_;
	for (int i=0; i<3; i++){
        t.at<float>(i, 0) = best_transformation(i, 3);
        
        for (int j=0; j<3; j++){
            R.at<float>(i, j) = best_transformation(i, j);
        }
    }


    end = clock();
    elapsed_secs = double( end - begin) / CLOCKS_PER_SEC;
	begin = end;
	cout << "Pose estimation time :" << elapsed_secs << "s" <<endl;
	cout <<endl;


    return 0; 
}

