#include <stdio.h>
#include <unistd.h>

#include <opencv2/core/core.hpp>

#include "visualization.h"
#include "feature_detector.h"
#include "feature_tracker.h"
#include "triangulation.h"
#include "motion_estimator.h"

using namespace std;
using namespace cv;

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

    //img1_l = Mat(img1_l, Rect(0, 0, 400, 376)); 
	//img1_r = Mat(img1_r, Rect(0, 0, 400, 376)); 
	//img2_l = Mat(img2_l, Rect(0, 0, 400, 376)); 
	//img2_r = Mat(img2_r, Rect(0, 0, 400, 376)); 

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
    
	Mat P = (Mat_<double>(3, 3) <<718.8560, 0, 607.1928, 0, 718.8560, 185.2157, 0, 0, 1);
    // =======================================================
    // Initialization
    // =======================================================
    
    //vector<Point2f> features_prev, features_next;

    Mat R = Mat::eye(3, 3, CV_64F);
    Mat t = Mat(3, 1, CV_64F, cvScalar(0.));
    showTraj.writeRes(R, t);

    vector<Point2f> keypoints1_l, keypoints1_r, keypoints2_l, keypoints2_r;

	//Ptr<ORB> orb = ORB::create();
	//orb->setMaxFeatures(2000);
	//Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
	//featureTracker orb_tracker(orb, matcher);

    //featureDetection(img1_l, keypoints1_l, 3);
	
	//Ptr<Feature2D> f2d = xfeatures2d::SIFT::create();
	//vector<KeyPoint> k1_l;
	//f2d->detect(img1_l, k1_l);
	//for (int i=0; i<k1_l.size(); i++){
	//	keypoints1_l.push_back(k1_l[i].pt);
	//}

	int img_row = img1_l.rows, img_col = img1_l.cols;

	featureDetector detector(img_row, img_col, 100);
	detector.directDetect(img1_l, keypoints1_l);
	
	featureTracker tracker;
	
	tracker.featureTrack(img1_l, img1_r, img2_l, keypoints1_l, keypoints1_r, keypoints2_l);

	triangulation tri(P1, P2);
	vector<Point3f> point_cloud;
	tri.pc_triangulate(keypoints1_l, keypoints1_r, point_cloud);

	motionEstimator::motionFromStructureAndImage pose_estimator(P);
	pose_estimator.updatePose(point_cloud, keypoints2_l, R, t);
    
    R_res = R.clone();
    t_res = t.clone();
	showTraj.writeRes(R_res, t_res);

	distance  += sqrt( t.at<double>(0) * t.at<double>(0) + t.at<double>(2) * t.at<double>(2) ); 

    Mat previousImg_l = img2_l.clone();
    Mat previousImg_r = img2_r.clone();
    Mat currentImg_l, currentImg_r;
    vector<Point2f> previousKeypoints = keypoints2_l;
	vector<Point2f> previousKeypoints_r;
    vector<Point2f> currentKeypoints;
    
    char filename_l[100], filename_r[100];
        
    // =======================================================
    // Loop
    // =======================================================
   	int iframe; 
   	for (iframe=2; iframe<MAX_FRAME; iframe++){
        sprintf(filename_l, "%simage_0/%06d.png", dataset_dir, iframe);
        sprintf(filename_r, "%simage_1/%06d.png", dataset_dir, iframe);
        currentImg_l = imread( filename_l, CV_LOAD_IMAGE_GRAYSCALE);
        currentImg_r = imread( filename_r, CV_LOAD_IMAGE_GRAYSCALE);

		if(! currentImg_l.data)
    	{
			cout << "\nTotal image pairs: " <<iframe <<endl;
        	cout << "Finish image processing!" << endl;
        	break;
    	}
	
		tracker.featureTrack(previousImg_l, previousImg_r, currentImg_l, previousKeypoints, previousKeypoints_r, currentKeypoints);

		vector<Point3f> point_cloud;
		tri.pc_triangulate(previousKeypoints, previousKeypoints_r, point_cloud);

		pose_estimator.updatePose(point_cloud, currentKeypoints, R, t);
    
        if (abs(t.at<double>(2)) > abs(t.at<double>(1)) && abs(t.at<double>(2)) > abs(t.at<double>(0)) ){
            t_res = t_res + R_res * t;
            R_res = R * R_res;
			distance  += sqrt( t.at<double>(0) * t.at<double>(0) + t.at<double>(2) * t.at<double>(2) ); 

        }
        
        if ( currentKeypoints.size() < MIN_NUM_FEATURES ){
        	vector<uchar> status;
        	detector.directDetect(currentImg_l, currentKeypoints);
			//vector<KeyPoint> k1_l;
			//f2d->detect(currentImg_l, k1_l);
			//for (int i=0; i<k1_l.size(); i++){
			//	currentKeypoints.push_back(k1_l[i].pt);
			//}

        }
        
        imshow("Camera", currentImg_l);
        showTraj.updateTraj(t_res);
		showTraj.writeRes(R_res, t_res);
        
        previousImg_l = currentImg_l.clone();
        previousImg_r = currentImg_r.clone();
        previousKeypoints = currentKeypoints;
    }
    
	end = clock();
	elapsed_secs = double( end - begin ) / CLOCKS_PER_SEC;
	cout << "Total duration for " << iframe << " frames: " <<elapsed_secs <<" s" <<endl;
	cout << "Frame rate: " << iframe / elapsed_secs <<" fps" <<endl;

    waitKey(0);
    return 0;
    
}


int main(int argc, char *argv[])
{
	if ( argc != 2 ){
		cerr << "Usage: ./test_stereo test_sequence_no" << endl;
		return -1;
	}	

	//char dataset_dir[CHAR_SIZE] = "/Users/Muyuan/Documents/vo/evaluation/kitti/data/sequences/";
	char dataset_dir[CHAR_SIZE] = "../evaluation/kitti/data/sequences/";

	char res_dir[CHAR_SIZE]; 
	int seq_no = atoi(argv[1]);

	sprintf(dataset_dir, "%s%.02d/", dataset_dir, seq_no);
	sprintf(res_dir, "../evaluation/results/data/%.02d.txt", seq_no);
	
	printf("%s\n", dataset_dir);
	printf("%s", res_dir);

    test_stereo(dataset_dir, res_dir);

    return 0;
}

