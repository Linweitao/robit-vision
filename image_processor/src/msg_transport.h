#include "Thresholder.h"
#include<ros/ros.h> //ros标准库头文件  
#include<iostream> //C++标准输入输出库  
#include "image_processor/Ball_Location.h"
#include "image_processor/Obstacle_Location.h"
#include<cv_bridge/cv_bridge.h>  //cv_bridge中包含CvBridge库  
#include<sensor_msgs/image_encodings.h>   //ROS图象类型的编码函数
#include<image_transport/image_transport.h>   //image_transport 头文件用来在ROS系统中的话题上发布和订阅图象消息  
#include<opencv2/core/core.hpp>  //OpenCV2标准头文件
#include<opencv2/highgui/highgui.hpp>  
#include<opencv2/imgproc/imgproc.hpp>    
#include <vector>
#include <cv.h>  
#include <algorithm>

static const std::string INPUT = "Input"; //定义输入窗口名称  
static const std::string OUTPUT = "Output"; //定义输出窗口名称 
using namespace std;
using namespace cv;
using namespace image_processor;

//定义一个转换的类  
class RGB_Transport  
{  
    private:  
    ros::NodeHandle nh_; //定义ROS句柄  
    image_transport::ImageTransport it_; //定义一个image_transport实例  
    image_transport::Subscriber image_sub_; //定义ROS图象接收器  
    Ball_Location ball_location;//用来发布的球位置信息
    Obstacle_Location obstacle_location;//用来发布的黑障位置信息
    ros::Publisher location_pub_ball;
    ros::Publisher location_pub_obstacle;
    Mat src;
    Mat dst_hsi,dst_ball,dst_obstacle,dst_field;
    int max_h, max_s, max_i;
    int min_h, min_s, min_i;
    Image_Thresholder thresholder;
    char type;
    int count;
    

    public:  
    RGB_Transport(); //构造函数  
    ~RGB_Transport(); //析构函数 
    void Msg_Subsribe(); //消息订阅函数
    void Msg_Publisher();//消息发布函数
    void convert_callback(const sensor_msgs::ImageConstPtr& msg); //回调函数，转换消息格式
    void Create_Trackbar();
    void set_value(int maxh, int maxs, int maxi, int minh, int mins, int mini, char object);
    Mat get_src();
    int get_max_h();
    int get_min_h();
    int get_max_s();
    int get_min_s();
    int get_max_i();
    int get_min_i();
};





