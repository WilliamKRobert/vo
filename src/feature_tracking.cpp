#include "feature_tracking.h"

#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/video/video.hpp>

using namespace std;
using namespace cv;


void featureTracking(Mat img1_l, Mat img1_r, Mat img2_l, vector<Point2f>& points1_l, vector<Point2f>& points1_r, vector<Point2f>& points2_l, vector<uchar>& status)	{
    
    //this function automatically gets rid of points for which tracking fails
    
    vector<float> err;
    Size winSize=Size(21,21);
    TermCriteria termcrit=TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01);
    
    calcOpticalFlowPyrLK(img1_l, img1_r, points1_l, points1_r, status, err, winSize, 3, termcrit, 0, 0.001);
    
    //getting rid of points for which the KLT tracking failed or those who have gone outside the frame
    int indexCorrection = 0;
    for( int i=0; i<status.size(); i++)
    {  Point2f pt = points1_r.at(i- indexCorrection);
        if ((status.at(i) == 0)||(pt.x<0)||(pt.y<0))	{
            if((pt.x<0)||(pt.y<0))	{
                status.at(i) = 0;
            }
            points1_l.erase (points1_l.begin() + (i - indexCorrection));
            points1_r.erase (points1_r.begin() + (i - indexCorrection));
            indexCorrection++;
        }
        
    }
    
    vector<uchar> status2;
    calcOpticalFlowPyrLK(img1_l, img2_l, points1_l, points2_l, status2, err, winSize, 3, termcrit, 0, 0.001);
    
    indexCorrection = 0;
    for( int i=0; i<status2.size(); i++)
    {
        if ( status2.at(i) == 0 )	{
            points1_l.erase (points1_l.begin() + (i - indexCorrection));
            points1_r.erase (points1_r.begin() + (i - indexCorrection));
            points2_l.erase (points2_l.begin() + (i - indexCorrection));
            indexCorrection++;
        }
        
    }

    
}

void featureTracking(Mat img_1, Mat img_2, vector<Point2f>& points1, vector<Point2f>& points2, vector<uchar>& status)	{
    
    //this function automatically gets rid of points for which tracking fails
    
    vector<float> err;
    Size winSize=Size(21,21);
    TermCriteria termcrit=TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01);
    
    calcOpticalFlowPyrLK(img_1, img_2, points1, points2, status, err, winSize, 3, termcrit, 0, 0.001);
    
    //getting rid of points for which the KLT tracking failed or those who have gone outside the frame
    int indexCorrection = 0;
    for( int i=0; i<status.size(); i++)
    {  Point2f pt = points2.at(i- indexCorrection);
        if ((status.at(i) == 0)||(pt.x<0)||(pt.y<0))	{
            if((pt.x<0)||(pt.y<0))	{
                status.at(i) = 0;
            }
            points1.erase (points1.begin() + (i - indexCorrection));
            points2.erase (points2.begin() + (i - indexCorrection));
            indexCorrection++;
        }
        
    }
    
}

void featureTracking(Mat img1_l, Mat img1_r, Mat img2_l, Mat img2_r, vector<Point2f>& points1_l, vector<Point2f>& points1_r, vector<Point2f>& points2_l, vector<Point2f> &points2_r)
{
    
    //this function automatically gets rid of points for which tracking fails
    vector<uchar> status1, status2; 

    vector<float> err;
    Size winSize=Size(21,21);
    TermCriteria termcrit=TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01);
    
    calcOpticalFlowPyrLK(img1_l, img1_r, points1_l, points1_r, status1, err, winSize, 3, termcrit, 0, 0.001);
	calcOpticalFlowPyrLK(img2_l, img2_r, points2_l, points2_r, status2, err, winSize, 3, termcrit, 0, 0.001);
    
    //getting rid of points for which the KLT tracking failed or those who have gone outside the frame
    int indexCorrection = 0;
    for( int i=0; i<status1.size(); i++){  
		Point2f pt1 = points1_r.at(i);
		Point2f pt2 = points2_r.at(i);
        Point2f pt1_l = points1_l.at(i);
        Point2f pt2_l = points2_l.at(i);
        if ( (status1[i] == 0) || (pt1.x<0) || (pt1.y<0) || (abs(pt1.y-pt1_l.y)>5)
		  || (status2[i] == 0) || (pt2.x<0) || (pt2.y<0) || (abs(pt2.y-pt2_l.y)>5))			{
            points1_l.erase ( points1_l.begin() + i ); 
			points1_r.erase ( points1_r.begin() + i ); 
			points2_l.erase ( points2_l.begin() + i ); 
			points2_r.erase ( points2_r.begin() + i ); 
			status1.erase ( status1.begin() + i ); 
			status2.erase ( status2.begin() + i ); 
			i--;	

        }
    }
    
}

