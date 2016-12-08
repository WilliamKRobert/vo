#include "feature_tracker.h"

#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/video/video.hpp>

using namespace std;
using namespace cv;

featureTracker::featureTracker(){
    winSize  = cv::Size(21, 21);
    termcrit = TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01);
}

void featureTracker::featureTrack(Mat img_1, Mat img_2, vector<Point2f>& points1, vector<Point2f>& points2)	{
    
    //this function automatically gets rid of points for which tracking fails
    calcOpticalFlowPyrLK(img_1, img_2, points1, points2, status_1, err, winSize, 3, termcrit, 0, 0.001);
    
    //getting rid of points for which the KLT tracking failed or those who have gone outside the frame
    int indexCorrection = 0;
    for( int i=0; i<status_1.size(); i++)
    {
        Point2f pt = points2.at(i- indexCorrection);
        if ((status_1.at(i) == 0)||(pt.x<0)||(pt.y<0))	{
            if((pt.x<0)||(pt.y<0))	{
                status_1.at(i) = 0;
            }
            points1.erase (points1.begin() + (i - indexCorrection));
            points2.erase (points2.begin() + (i - indexCorrection));
            indexCorrection++;
        }
        
    }
    
}

void featureTracker::featureTrack(Mat img1_l, Mat img1_r, Mat img2_l, vector<Point2f>& points1_l, vector<Point2f>& points1_r, vector<Point2f>& points2_l)	{
    //this function automatically gets rid of points for which tracking fails
    calcOpticalFlowPyrLK(img1_l, img1_r, points1_l, points1_r, status_1, err, winSize, 3, termcrit, 0, 0.001);
    
    //getting rid of points for which the KLT tracking failed or those who have gone outside the frame
    int indexCorrection = 0;
    for( int i=0; i<status_1.size(); i++)
    {  
		Point2f pt = points1_r.at(i- indexCorrection);
		Point2f pt0 = points1_l.at(i - indexCorrection);
        if ( (status_1.at(i) == 0) || (pt.x<0) || (pt.y<0) || (pt0.x - pt.x < 0) )	{
            if((pt.x<0)||(pt.y<0) || (pt0.x-pt.x<0))	{
                status_1.at(i) = 0;
            }
            points1_l.erase (points1_l.begin() + (i - indexCorrection));
            points1_r.erase (points1_r.begin() + (i - indexCorrection));
            indexCorrection++;
        }
        
    }
    
    
    calcOpticalFlowPyrLK(img1_l, img2_l, points1_l, points2_l, status_2, err, winSize, 3, termcrit, 0, 0.001);
    
    
    indexCorrection = 0;
    for( int i=0; i<status_2.size(); i++)
    {
        if ( status_2.at(i) == 0 )	{
            points1_l.erase (points1_l.begin() + (i - indexCorrection));
            points1_r.erase (points1_r.begin() + (i - indexCorrection));
            points2_l.erase (points2_l.begin() + (i - indexCorrection));
            indexCorrection++;
        }
        
    }

	feature_num = points2_l.size();

}

void featureTracker::featureTrack(Mat img1_l, Mat img1_r, Mat img2_l, Mat img2_r, vector<Point2f>& points1_l, vector<Point2f>& points1_r, vector<Point2f>& points2_l, vector<Point2f> &points2_r)
{
    
    //this function automatically gets rid of points for which tracking fails
    calcOpticalFlowPyrLK(img1_l, img1_r, points1_l, points1_r, status_1, err, winSize, 3, termcrit, 0, 0.001);
	calcOpticalFlowPyrLK(img2_l, img2_r, points2_l, points2_r, status_2, err, winSize, 3, termcrit, 0, 0.001);
    
    //getting rid of points for which the KLT tracking failed or those who have gone outside the frame
    int indexCorrection = 0;
    for( int i=0; i<status_1.size(); i++){
		Point2f pt1 = points1_r.at(i-indexCorrection);
		Point2f pt2 = points2_r.at(i-indexCorrection);
        Point2f pt1_l = points1_l.at(i-indexCorrection);
        Point2f pt2_l = points2_l.at(i-indexCorrection);
        if ( (status_1[i] == 0) || (pt1.x<0) || (pt1.y<0) || (abs(pt1.y-pt1_l.y)>5)
		  || (status_2[i] == 0) || (pt2.x<0) || (pt2.y<0) || (abs(pt2.y-pt2_l.y)>5))			{
            points1_l.erase ( points1_l.begin() + i-indexCorrection ); 
			points1_r.erase ( points1_r.begin() + i-indexCorrection ); 
			points2_l.erase ( points2_l.begin() + i-indexCorrection ); 
			points2_r.erase ( points2_r.begin() + i-indexCorrection ); 
			status_1[i] = 0;
			status_2[i] = 0;
		    indexCorrection++;	

        }
    }
    
}

void featureTracker::getInitIndex(vector<int> &init_index)
{
    if (status_1.size() == 0){
        cerr<< "Track keypoints before getting the point index!" <<endl;
        return;
    }
    
    if (init_index.size() != 0) init_index.clear();
    for (int i=0; i<feature_num; i++){
        init_index.push_back(i);
    }
    
    return;
}

void featureTracker::getPointIndex(vector<int> &old_index, vector<int> &new_index)
{
    
	if (status_1.size() == 0){
		cerr<< "Track keypoints before getting the point index!" <<endl;
		return;
	}

    vector<int> point_index;
    
	for (int i=0; i<status_1.size(); i++){
		if (status_1[i] == 1){
			point_index.push_back( old_index[i] );
		}	
	}
    
    if (new_index.size() != 0) new_index.clear();
    
	for (int i=0; i<status_2.size(); i++){
		if (status_2[i] == 1){
			new_index.push_back ( point_index[i] );
		}
	}
   
    return;

}
