#include <stdio.h>
#include <unistd.h>

#include <opencv2/core/core.hpp>

#include "visualization.h"
#include "feature_detector.h"
#include "feature_tracker.h"
#include "triangulation.h"
#include "motion_estimator.h"
#include "point_cloud_alignment.h"

using namespace std;
using namespace cv;
using namespace Eigen;

#define MAX_FRAME 1000//5000 
#define MIN_NUM_FEATURES 2000
#define CHAR_SIZE 200


int test_stereo(char *dataset_dir, char *resFile)
{
    Mat img1_l, img1_r, img2_l, img2_r;
    Mat R_res, t_res;
    
	clock_t begin = clock();
	clock_t end;
	double elapsed_secs;
	double distance = 0;

	Mat traj = Mat::zeros(600, 600, CV_8UC3);
	Visualization showTraj(traj, resFile);
	
    // =======================================================
    // Preprocessing
    // =======================================================
    char filename1_l[CHAR_SIZE], filename1_r[CHAR_SIZE], filename2_l[CHAR_SIZE], filename2_r[CHAR_SIZE];

	sprintf(filename1_l, "%simage_0/%06d.png", dataset_dir, 0);
	sprintf(filename1_r, "%simage_1/%06d.png", dataset_dir, 0 );
	sprintf(filename2_l, "%simage_0/%06d.png", dataset_dir, 1 );
	sprintf(filename2_r, "%simage_1/%06d.png", dataset_dir, 1);

   	img1_l = imread( filename1_l, CV_LOAD_IMAGE_GRAYSCALE );
   	img1_r = imread( filename1_r, CV_LOAD_IMAGE_GRAYSCALE );
   	img2_l = imread( filename2_l, CV_LOAD_IMAGE_GRAYSCALE );
   	img2_r = imread( filename2_r, CV_LOAD_IMAGE_GRAYSCALE );

    if(! img1_l.data || ! img1_r.data || !img2_l.data || !img2_r.data)
    {
        cout <<" Could not open or find the image" << endl;
        return -1;
    }
    
    Eigen::MatrixXf P1(3, 4), P2(3, 4);
    P1 << 718.8560,        0, 607.1928,         0,
			     0, 718.8560, 185.2157,         0,
    			 0,        0,   1.0000,         0;
    P2 << 718.8560,        0, 607.1928, -386.1448,
			      0, 718.8560, 185.2157,         0,
			      0,        0,   1.0000,         0;
    
    // =======================================================
    // Initialization
    // =======================================================
    
    //vector<Point2f> features_prev, features_next;

    Mat R = Mat::eye(3, 3, CV_32F);
    Mat t = Mat(3, 1, CV_32F, cvScalar(0.));
    showTraj.writeRes(R, t);

    vector<Point2f> keypoints1_l, keypoints1_r, keypoints2_l, keypoints2_r;

	int img_row = img1_l.rows, img_col = img1_l.cols;

	featureDetector detector(img_row, img_col, 100);
	detector.directDetect(img1_l, keypoints1_l);
    
	featureTracker tracker;
    
    tracker.featureTrack(img1_l, img2_l, keypoints1_l, keypoints2_l);
	tracker.featureTrack(img1_l, img1_r, img2_l, img2_r, keypoints1_l, keypoints1_r, keypoints2_l, keypoints2_r);

	triangulation tri(P1, P2);
	vector<Point3f> point_cloud_1, point_cloud_2;
	tri.pc_triangulate(keypoints1_l, keypoints1_r, point_cloud_1);
    tri.pc_triangulate(keypoints2_l, keypoints2_r, point_cloud_2);

//	motionEstimator::motionFromStructure pose_estimator;
//    pose_estimator.updatePose(point_cloud_1, point_cloud_2, R, t);
//    cout <<"ufff" <<endl <<R <<endl <<t;
    
    vector<int> index;
    for (int i=0; i<point_cloud_1.size(); i++){
        if (point_cloud_1[i].z < 20 && point_cloud_2[i].z < 20)
            index.push_back(i);
    }
    
    int num_point = index.size();
    MatrixXf pointCloud1(num_point, 3);
    MatrixXf pointCloud2(num_point, 3);
    
    
    for (int i=0; i<num_point; i++){
        pointCloud1(i, 0) = point_cloud_1[index[i]].x;
        pointCloud1(i, 1) = point_cloud_1[index[i]].y;
        pointCloud1(i, 2) = point_cloud_1[index[i]].z;
        
        pointCloud2(i, 0) = point_cloud_2[index[i]].x;
        pointCloud2(i, 1) = point_cloud_2[index[i]].y;
        pointCloud2(i, 2) = point_cloud_2[index[i]].z;
    }
    
    pointCloudAlign_sac(pointCloud1, pointCloud2, R, t);
    
    R_res = R.clone();
    t_res = t.clone();
	showTraj.writeRes(R_res, t_res);
    cout <<endl <<R_res <<endl;
    cout <<t_res <<endl;
    
    
    
//    return 0;
    
    Mat previousImg_l = img2_l.clone();
    Mat previousImg_r = img2_r.clone();
    Mat currentImg_l, currentImg_r;
    vector<Point2f> previousKeypoints_l = keypoints2_l;
	vector<Point2f> previousKeypoints_r = keypoints2_r;
    vector<Point2f> currentKeypoints_l, currentKeypoints_r;
    
    char filename_l[100], filename_r[100];
        
    // =======================================================
    // Loop
    // =======================================================
   	int iframe;
   	for (iframe=2; iframe<MAX_FRAME; iframe++){
        sprintf(filename_l, "%simage_0/%06d.png", dataset_dir, iframe);
        sprintf(filename_r, "%simage_1/%06d.", dataset_dir, iframe);
        currentImg_l = imread( filename_l, CV_LOAD_IMAGE_GRAYSCALE);
        currentImg_r = imread( filename_r, CV_LOAD_IMAGE_GRAYSCALE);

		if(! currentImg_l.data)
    	{
			cout << "\nTotal image pairs: " <<iframe <<endl;
        	cout << "Finish image processing!" << endl;
        	break;
    	}
        
        tracker.featureTrack(previousImg_l, currentImg_l, previousKeypoints_l, currentKeypoints_l);
		tracker.featureTrack(previousImg_l, previousImg_r, currentImg_l, currentImg_r, previousKeypoints_l, previousKeypoints_r, currentKeypoints_l, currentKeypoints_r);
        cout <<"Observed " <<endl;
		vector<Point3f> point_cloud_pre, point_cloud_cur;
		tri.pc_triangulate(previousKeypoints_l, previousKeypoints_r, point_cloud_pre);
        tri.pc_triangulate(currentKeypoints_l, currentKeypoints_r, point_cloud_cur);
        cout <<"Get here!" <<endl;
//		pose_estimator.updatePose(point_cloud_pre, point_cloud_cur, R, t);
        index.clear();
        for (int i=0; i<point_cloud_1.size(); i++){
            if (point_cloud_1[i].z < 20 && point_cloud_2[i].z < 20)
                index.push_back(i);
        }
        
        num_point = index.size();
        MatrixXf pointCloud1(num_point, 3);
        MatrixXf pointCloud2(num_point, 3);
        
        
        for (int i=0; i<num_point; i++){
            pointCloud1(i, 0) = point_cloud_pre[index[i]].x;
            pointCloud1(i, 1) = point_cloud_pre[index[i]].y;
            pointCloud1(i, 2) = point_cloud_pre[index[i]].z;
            
            pointCloud2(i, 0) = point_cloud_cur[index[i]].x;
            pointCloud2(i, 1) = point_cloud_cur[index[i]].y;
            pointCloud2(i, 2) = point_cloud_cur[index[i]].z;
        }
        
        pointCloudAlign_sac(pointCloud1, pointCloud2, R, t);
        cout <<"Oh now is here!" <<endl;
        if (abs(t.at<float>(2)) > abs(t.at<float>(1)) && abs(t.at<float>(2)) > abs(t.at<float>(0)) ){
            t_res = t_res + R_res * t;
            R_res = R * R_res;
			distance  += sqrt( t.at<float>(0) * t.at<float>(0) + t.at<float>(2) * t.at<float>(2) );

        }
        
        if ( currentKeypoints_l.size() < MIN_NUM_FEATURES ){
        	vector<uchar> status;
        	detector.directDetect(currentImg_l, currentKeypoints_l);
            currentKeypoints_r.clear();
        }
        
        imshow("Camera", currentImg_l);
        showTraj.updateTraj(t_res);
		showTraj.writeRes(R_res, t_res);
        
        previousImg_l = currentImg_l.clone();
        previousImg_r = currentImg_r.clone();
        previousKeypoints_l = currentKeypoints_l;
        previousKeypoints_r = currentKeypoints_r;
    }
    
	end = clock();
	elapsed_secs = double( end - begin ) / CLOCKS_PER_SEC;
	cout << "Total duration for " << iframe << " frames: " <<elapsed_secs <<" s" <<endl;
	cout << "Frame rate: " << iframe / elapsed_secs <<" fps" <<endl;
    
    cout <<R_res <<endl;
    cout <<t_res <<endl;

    waitKey(0);
    return 0;
    
}


int main(int argc, char *argv[])
{
	if ( argc != 2 ){
		cerr << "Usage: ./test_stereo test_sequence_no" << endl;
		return -1;
	}	

	char dataset_dir[CHAR_SIZE] = "/Users/Muyuan/Documents/vo/evaluation/kitti/data/sequences/";
	//char dataset_dir[CHAR_SIZE] = "../evaluation/kitti/data/sequences/";

	char res_dir[CHAR_SIZE]; 
	int seq_no = atoi(argv[1]);

	sprintf(dataset_dir, "%s%.02d/", dataset_dir, seq_no);
	sprintf(res_dir, "/Users/Muyuan/Documents/vo/evaluation/results/data/%.02d.txt", seq_no);
	
	printf("%s\n", dataset_dir);
	printf("%s", res_dir);

    test_stereo(dataset_dir, res_dir);

    return 0;
}

