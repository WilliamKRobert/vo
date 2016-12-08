#include <cmath>
#include <cstdio>
#include <iostream>

#include <stdio.h>
#include <unistd.h>

#include <opencv2/core/core.hpp>

#include "ceres/rotation.h"

#include "show_res.h"
#include "feature_detector.h"
#include "feature_tracker.h"
#include "triangulation.h"
#include "motion_estimator.h"
#include "map_association.h"
#include "optimizer.h"

using namespace std;
using namespace cv;

#define MAX_FRAME 1000//5000
#define MIN_NUM_FEATURES 100
#define CHAR_SIZE 200


int test_ba(char *dataset_dir, char *resFile, char *optimized_res_file)
{
    Mat img1_l, img1_r, img2_l, img2_r;
    Mat R_res, t_res;
    
//    clock_t begin = clock();
//    clock_t end;
//    double elapsed_secs;
    double distance = 0;
    
    Mat traj = Mat::zeros(600, 600, CV_8UC3);
    showRes showTraj(traj, resFile);
    
    showRes optimized_res(optimized_res_file);
    
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
    
    Mat P = (Mat_<double>(3, 3) <<718.8560, 0, 607.1928, 0, 718.8560, 185.2157, 0, 0, 1);

    
    Mat R = Mat::eye(3, 3, CV_64F);
    Mat t = Mat(3, 1, CV_64F, cvScalar(0.));
    showTraj.writeRes(R, t);
    optimized_res.writeRes(R, t);
    
    
    
    
    // --------------------------------------------------------------------------------
    // --------------------------------------------------------------------------------
    vector<Point2f> keypoints1_l, keypoints1_r, keypoints2_l, keypoints2_r;
    
    int img_row = img1_l.rows, img_col = img1_l.cols;
    
    featureDetector detector(img_row, img_col, 100);
    detector.directDetect(img1_l, keypoints1_l);
   
    featureTracker tracker;
    tracker.featureTrack(img1_l, img1_r, img2_l, keypoints1_l, keypoints1_r, keypoints2_l);

    vector<int> init_index;
	tracker.getInitIndex(init_index);
    
    triangulation tri(P1, P2);
    vector<Point3f> point_cloud;
    tri.pc_triangulate(keypoints1_l, keypoints1_r, point_cloud);
    cout <<"How can it be?" <<point_cloud.size() <<endl;
    
    motionEstimator::motionFromStructureAndImage pose_estimator(P);
    pose_estimator.updatePose(point_cloud, keypoints2_l, R, t);
    
    R_res = R.clone();
    t_res = t.clone();
    showTraj.writeRes(R_res, t_res);
    
    Mat rvec = cv::Mat::zeros(3, 1, CV_64F);
    Rodrigues(R, rvec);
    
    localMapAssociation local_map(point_cloud, keypoints2_l, rvec, t_res);

    distance  += sqrt( t.at<double>(0) * t.at<double>(0) + t.at<double>(2) * t.at<double>(2) );
    
    Mat previousImg_l = img2_l.clone();
    Mat previousImg_r = img2_r.clone();
    Mat currentImg_l, currentImg_r;
    vector<Point2f> previousKeypoints = keypoints2_l;
    vector<Point2f> previousKeypoints_r;
    vector<Point2f> currentKeypoints;
    
    char filename_l[100], filename_r[100];
    google::InitGoogleLogging("Local bundle Adjustment");
    
    // --------------------------------------------------------------------------------
    // --------------------------------------------------------------------------------
   	int iframe;
    double *parameter = NULL;
    Mat key_frame_l = img1_l, key_frame_r = img1_r;
    vector<Point2f> key_frame_keypoints = keypoints1_l;
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
        
        //tracker.featureTrack(key_frame_l, key_frame_r, currentImg_l, key_frame_keypoints, previousKeypoints_r, currentKeypoints);
        
        vector<int> track_index;
        tracker.getPointIndex(init_index, track_index);

        vector<Point3f> point_cloud;
        tri.pc_triangulate(previousKeypoints, previousKeypoints_r, point_cloud);
        
        pose_estimator.updatePose(point_cloud, currentKeypoints, R, t);
        
        if (abs(t.at<double>(2)) > abs(t.at<double>(1)) && abs(t.at<double>(2)) > abs(t.at<double>(0)) ){
            t_res = t_res + R_res * t;
            R_res = R * R_res;
            distance  += sqrt( t.at<double>(0) * t.at<double>(0) + t.at<double>(2) * t.at<double>(2) );
            
        }
        
        Rodrigues(R_res, rvec);
		local_map.addObservation(currentKeypoints, track_index, rvec, t_res);
        
        init_index = track_index;


        if ( currentKeypoints.size() < MIN_NUM_FEATURES ){
            int num_cameras = local_map.num_cameras();
            int num_points = local_map.num_points();
            int num_parameters = 6*num_cameras + 3*num_points;
            cout <<"-----------------" <<num_cameras <<"-----------------" <<endl;
            if (parameter != NULL) {
                delete[] parameter;
                parameter = NULL;
            }
            
            parameter = new double[num_parameters];
            
            //local_optimizer = optimizer();
            optimizer local_optimizer;
            local_optimizer.localBundleAdjustment(local_map, parameter);
        
            imshow("Camera", currentImg_l);
            showTraj.updateTraj(t_res);
            showTraj.writeRes(R_res, t_res);
            
            for(int j=0; j<local_map.num_cameras(); j++){
                optimized_res.writeRes(&parameter[6*j]);
            }
            
            vector<uchar> status;
            detector.directDetect(currentImg_l, currentKeypoints);
            
            previousImg_l = currentImg_l.clone();
            previousImg_r = currentImg_r.clone();
            previousKeypoints = currentKeypoints;
            
            iframe++;
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
            
//            vector<int> track_index;
//            tracker.getPointIndex(init_index, track_index);
            tracker.getInitIndex(init_index);
            
            vector<Point3f> point_cloud;
            tri.pc_triangulate(previousKeypoints, previousKeypoints_r, point_cloud);
            
            pose_estimator.updatePose(point_cloud, currentKeypoints, R, t);
            
            if (abs(t.at<double>(2)) > abs(t.at<double>(1)) && abs(t.at<double>(2)) > abs(t.at<double>(0)) ){
                t_res = t_res + R_res * t;
                R_res = R * R_res;
                distance  += sqrt( t.at<double>(0) * t.at<double>(0) + t.at<double>(2) * t.at<double>(2) );
                
            }
            
            Rodrigues(R_res, rvec);
            local_map = localMapAssociation(point_cloud, currentKeypoints, rvec, t_res);
        }
        
        imshow("Camera", currentImg_l);
        showTraj.updateTraj(t_res);
        showTraj.writeRes(R_res, t_res);
        
        previousImg_l = currentImg_l.clone();
        previousImg_r = currentImg_r.clone();
        previousKeypoints = currentKeypoints;
    }
    
    
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
//    char dataset_dir[CHAR_SIZE] = "../evaluation/kitti/data/sequences/";
    
    char res_file[CHAR_SIZE];
    char optimized_res_file[CHAR_SIZE];
    int seq_no = atoi(argv[1]);
    
    cout <<"Enjoy XCode!" <<endl;
    sprintf(dataset_dir, "%s%.02d/", dataset_dir, seq_no);
    sprintf(res_file, "/Users/Muyuan/vo/evaluation/results/data/%.02d.txt", seq_no);
    sprintf(optimized_res_file, "/Users/Muyuan/vo/evaluation/results/data/optimized_%.02d.txt", seq_no);
    
//    printf("%s\n", dataset_dir);
//    printf("%s", res_dir);
    
    test_ba(dataset_dir, res_file, optimized_res_file);
    
    return 0;
}
