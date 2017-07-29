#include<ros/ros.h> //ros标准库头文件  
#include<iostream> //C++标准输入输出库
#include "localization/Location.h"
#include "localization/Samples.h"
#include<opencv2/core/core.hpp>  //OpenCV2标准头文件
#include<opencv2/highgui/highgui.hpp>  
#include<opencv2/imgproc/imgproc.hpp>    
#include <vector>
#include <cv.h>  
#include <algorithm>
#include<cv_bridge/cv_bridge.h>  //cv_bridge中包含CvBridge库  
#include<sensor_msgs/image_encodings.h>   //ROS图象类型的编码函数
#include<image_transport/image_transport.h>   //image_transport 头文件用来在ROS系统中的话题上发布和订阅图象消息  

#include "gige_cap/gige_config.h"
#include "gige_cap/SetParam.h"


using namespace gige;
using namespace gige_cap;
using namespace std;
using namespace cv;
using namespace localization;


static const float PI = 3.1415926535f;
static const float eps = 0.001f;
static const int sign_num = 100;
static const int scan_num = 50;
static const int white_threshold=90;

    

class sign_mark  
{  
    private:  
    ros::NodeHandle nh_; //定义ROS句柄  
    ros::Publisher location_pub;
    image_transport::ImageTransport it_; //定义一个image_transport实例  
    image_transport::Subscriber image_sub_; //定义ROS图象接收器  
    localization::Samples msg_output;
    float curvature[150][2];

    

    public:  
    sign_mark(); //构造函数  
    void Msg_Subsribe(); //消息订阅函数
    void Loc_Publisher();//消息发布函数
    void callback(const sensor_msgs::ImageConstPtr& msg); //回调函数，转换消息格式
    
    void curvature_lookup_table();
   
    float Calculate_Distance(float x, float y);



  
};

