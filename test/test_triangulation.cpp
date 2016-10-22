#include <iostream>

#include </usr/local/Cellar/eigen/3.2.8/include/eigen3/Eigen/Dense>
#include </usr/local/Cellar/eigen/3.2.8/include/eigen3/Eigen/Eigen>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/core/eigen.hpp"

#include "cal_pose.h"
#include "tool.h"

using namespace std;
using namespace cv;


int main( int argc, char** argv )
{

	Eigen::MatrixXf P1(3, 4), P2(3, 4);
    P1 << 718.8560,        0, 607.1928,         0,
    	 		 0, 718.8560, 185.2157,         0,
			     0,        0,   1.0000,         0;
    P2 << 718.8560,        0, 607.1928, -386.1448,
                 0, 718.8560, 185.2157,         0,
                 0,        0,   1.0000,         0;
	
	Point2i p1(10, 5), p2(8, 5);
	Eigen::Vector4f a = triangulation(p1, p2, P1, P2);
	cout << a << endl;

	return 0;

}

