#include "cal_pose.h"

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

/*
template<typename _Tp, int _rows, int _cols, int _options, int _maxRows, int _maxCols>
void eigen2cv(const Eigen::Matrix<_Tp, _rows, _cols, _options, _maxRows, _maxCols>& src, cv::Mat& dst)
{
    if (!(src.Flags & Eigen::RowMajorBit))
    {
        cv::Mat _src(src.cols(), src.rows(), cv::DataType<_Tp>::type,
                     (void*)src.data(), src.stride() * sizeof(_Tp));
        cv::transpose(_src, dst);
    }
    else
    {
        cv::Mat _src(src.rows(), src.cols(), cv::DataType<_Tp>::type,
                     (void*)src.data(), src.stride() * sizeof(_Tp));
        _src.copyTo(dst);
    }
}

template<typename _Tp, int _rows, int _cols, int _options, int _maxRows, int _maxCols>
void cv2eigen( const Mat& src,
              Eigen::Matrix<_Tp, _rows, _cols, _options, _maxRows, _maxCols>& dst )
{
    CV_DbgAssert(src.rows == _rows && src.cols == _cols);
    if( !(dst.Flags & Eigen::RowMajorBit) )
    {
        Mat _dst(src.cols, src.rows, DataType<_Tp>::type,
                 dst.data(), (size_t)(dst.stride()*sizeof(_Tp)));
        if( src.type() == _dst.type() )
            transpose(src, _dst);
        else if( src.cols == src.rows )
        {
            src.convertTo(_dst, _dst.type());
            transpose(_dst, _dst);
        }
        else
            Mat(src.t()).convertTo(_dst, _dst.type());
        CV_DbgAssert(_dst.data == (uchar*)dst.data());
    }
    else
    {
        Mat _dst(src.rows, src.cols, DataType<_Tp>::type,
                 dst.data(), (size_t)(dst.stride()*sizeof(_Tp)));
        src.convertTo(_dst, _dst.type());
        CV_DbgAssert(_dst.data == (uchar*)dst.data());
    }
}

template<typename _Tp>
void cv2eigen( const Mat& src,
              Eigen::Matrix<_Tp, Eigen::Dynamic, Eigen::Dynamic>& dst )
{
    dst.resize(src.rows, src.cols);
    if( !(dst.Flags & Eigen::RowMajorBit) )
    {
        Mat _dst(src.cols, src.rows, DataType<_Tp>::type,
                 dst.data(), (size_t)(dst.stride()*sizeof(_Tp)));
        if( src.type() == _dst.type() )
            transpose(src, _dst);
        else if( src.cols == src.rows )
        {
            src.convertTo(_dst, _dst.type());
            transpose(_dst, _dst);
        }
        else
            Mat(src.t()).convertTo(_dst, _dst.type());
        CV_DbgAssert(_dst.data == (uchar*)dst.data());
    }
    else
    {
        Mat _dst(src.rows, src.cols, DataType<_Tp>::type,
                 dst.data(), (size_t)(dst.stride()*sizeof(_Tp)));
        src.convertTo(_dst, _dst.type());
        CV_DbgAssert(_dst.data == (uchar*)dst.data());
    }
}


template<typename _Tp>
void cv2eigen( const Mat& src,
              Eigen::Matrix<_Tp, Eigen::Dynamic, 1>& dst )
{
    CV_Assert(src.cols == 1);
    dst.resize(src.rows);
    
    if( !(dst.Flags & Eigen::RowMajorBit) )
    {
        Mat _dst(src.cols, src.rows, DataType<_Tp>::type,
                 dst.data(), (size_t)(dst.stride()*sizeof(_Tp)));
        if( src.type() == _dst.type() )
            transpose(src, _dst);
        else
            Mat(src.t()).convertTo(_dst, _dst.type());
        CV_DbgAssert(_dst.data == (uchar*)dst.data());
    }
    else
    {
        Mat _dst(src.rows, src.cols, DataType<_Tp>::type,
                 dst.data(), (size_t)(dst.stride()*sizeof(_Tp)));
        src.convertTo(_dst, _dst.type());
        CV_DbgAssert(_dst.data == (uchar*)dst.data());
    }
}


template<typename _Tp>
void cv2eigen( const Mat& src,
              Eigen::Matrix<_Tp, 1, Eigen::Dynamic>& dst )
{
    CV_Assert(src.rows == 1);
    dst.resize(src.cols);
    if( !(dst.Flags & Eigen::RowMajorBit) )
    {
        Mat _dst(src.cols, src.rows, DataType<_Tp>::type,
                 dst.data(), (size_t)(dst.stride()*sizeof(_Tp)));
        if( src.type() == _dst.type() )
            transpose(src, _dst);
        else
            Mat(src.t()).convertTo(_dst, _dst.type());
        CV_DbgAssert(_dst.data == (uchar*)dst.data());
    }
    else
    {
        Mat _dst(src.rows, src.cols, DataType<_Tp>::type,
                 dst.data(), (size_t)(dst.stride()*sizeof(_Tp)));
        src.convertTo(_dst, _dst.type());
        CV_DbgAssert(_dst.data == (uchar*)dst.data());
    }
}
*/
