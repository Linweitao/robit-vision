#include<ros/ros.h> //ros标准库头文件  
#include<iostream> //C++标准输入输出库
#include<opencv2/core/core.hpp>  //OpenCV2标准头文件
#include<opencv2/highgui/highgui.hpp>  
#include<opencv2/imgproc/imgproc.hpp>    
#include <vector>
#include <cv.h>  
#include <algorithm>

#include "gige_cap/gige_config.h"
#include "gige_cap/SetParam.h"
using namespace gige;
using namespace gige_cap;

static const float PI = 3.1415926535f;
static const float eps = 0.001f;

using namespace std;
using namespace cv;

class Image_Thresholder
{
    private:
        float curvature[150][2];
        float ball_xcordinate;
        float ball_ycordinate;
        float field_xcordinate;
        float field_ycordinate;
    public:
  void Converter(const cv::Mat& src, cv::Mat& dst);//将RGB图像转化为HSI图像
        void Threshold_Ball(const cv::Mat& src, cv::Mat& dst, int max_h, int min_h, int max_s, int min_s, int max_i, int min_i);//对球进行阈值
        void Threshold_Obstacle(const cv::Mat& src, cv::Mat& dst, int max_h, int min_h, int max_s, int min_s, int max_i, int min_i);//对黑障进行阈值
        void Obstacle_Area(Mat& src,Mat& dst,char type,vector<float> &theta,vector<float> &distance);//找出黑障的区域并标记
        void Ball_Area(Mat& src,Mat& dst,char type,float &theta,float &distance);//找出球的区域并标记
        float Calculate_Distance(float x, float y);//从像素位置计算实际距离
        void curvature_lookup_table();//查表生成curvature二维表
};
	
