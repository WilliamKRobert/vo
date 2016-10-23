#include "tool.h"

#include <fstream>

using namespace std;


void removeRow(Eigen::MatrixXf& matrix, unsigned int rowToRemove)
{
    unsigned int numRows = matrix.rows()-1;
    unsigned int numCols = matrix.cols();
    
    if( rowToRemove < numRows )
        matrix.block(rowToRemove,0,numRows-rowToRemove,numCols) = matrix.block(rowToRemove+1,0,numRows-rowToRemove,numCols);
    
    matrix.conservativeResize(numRows,numCols);
}

void removeColumn(Eigen::MatrixXf& matrix, unsigned int colToRemove)
{
    unsigned int numRows = matrix.rows();
    unsigned int numCols = matrix.cols()-1;
    
    if( colToRemove < numCols )
        matrix.block(0,colToRemove,numRows,numCols-colToRemove) = matrix.block(0,colToRemove+1,numRows,numCols-colToRemove);
    
    matrix.conservativeResize(numRows,numCols);
}

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

void featureDetection(Mat img_1, vector<Point2f>& points1)	{   //uses FAST as of now, modify parameters as necessary
    vector<KeyPoint> keypoints_1;
    int fast_threshold = 20;
    bool nonmaxSuppression = true;
    FAST(img_1, keypoints_1, fast_threshold, nonmaxSuppression);
    KeyPoint::convert(keypoints_1, points1, vector<int>());
}

double getAbsoluteScale(int frame_id, int sequence_id, double z_cal)	{
    
    string line;
    int i = 0;
    ifstream myfile ("/Users/Muyuan/Downloads/dataset/sequences/00/00.txt", ifstream::in);
    double x =0, y=0, z = 0;
    double x_prev, y_prev, z_prev;
    if (myfile.is_open())
    {
        while (( getline (myfile,line) ) && (i<=frame_id))
        {
            z_prev = z;
            x_prev = x;
            y_prev = y;
            std::istringstream in(line);
            //cout << line << '\n';
            for (int j=0; j<12; j++)  {
                in >> z ;
                if (j==7) y=z;
                if (j==3)  x=z;
            }
            
            i++;
        }
        myfile.close();
    }
    
    else {
        cout << "Unable to open file";
        return 0;
    }
    
    return sqrt((x-x_prev)*(x-x_prev) + (y-y_prev)*(y-y_prev) + (z-z_prev)*(z-z_prev)) ;
    
}


