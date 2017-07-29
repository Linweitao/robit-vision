#include <ros/ros.h> //ros标准库头文件  
#include <iostream> //C++标准输入输出库
#include"localization/Samples.h"
#include<opencv2/core/core.hpp>  //OpenCV2标准头文件
#include<opencv2/highgui/highgui.hpp>  
#include<opencv2/imgproc/imgproc.hpp>   
static const float PI = 3.1415926535f;
static const int sign_num = 100;
using namespace std;
using namespace cv;

class Near_table
{
    private:
    float dis;
    float x;
    float y;
    
    public:
    Near_table();
    void set(float a,float b,float c);
    float getx();
    float gety();
    float getdis();
    
};
static Near_table data[1805][1205]; 
class localizer
{
    private:
    ros::NodeHandle it; //定义句柄
    localization::Samples msg_input; //定义Sample消息
    ros::Subscriber sub;  //定义订阅器
    double robot_x, robot_y, robot_theta;  //机器人坐标，机器人朝向角
    float nearestx[sign_num], nearesty[sign_num];  //定义最近点的x,y坐标
    float sub_local_x[sign_num], sub_local_y[sign_num], sub_local_theta[sign_num], sub_world_x[sign_num], sub_world_y[sign_num];  //订阅的数据
    float dx, dy, dtheta; //x,y,theta的微分变量

    Mat src,show;
    public:
    localizer();
    ~localizer();
    void set_robot_x(float x);
    void set_robot_y(float y);
    void set_robot_theta(float theta);// set_系列为变量赋值     
    void msg_subsribe();           // 消息订阅函数
    void convert_callback(const localization::Samples::ConstPtr& msg);  // 消息回调函数
    void gredient_desent();   // 微分计算
    float match_degree(float x,float y,float theta);
    void calculate();   //计算每次迭代
    void Near_lookup_table();
    //图形化界面显示定位结果
    void shape_sign(Mat &src,int x ,int y);
    void shape_robot(Mat &src,int x ,int y);
};

