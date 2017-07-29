/*
 该节点主要是获取笔记本自身图像并发布
  20170727
*/
#include "Thresholder.h"
#include<ros/ros.h> //ros标准库头文件  
#include<iostream> //C++标准输入输出库  
#include "image_processor/Ball_Location.h"
#include "image_processor/Obstacle_Location.h"
#include<cv_bridge/cv_bridge.h>  //cv_bridge中包含CvBridge库  
#include<sensor_msgs/image_encodings.h>   //ROS图象类型的编码函数
#include<image_transport/image_transport.h>   //image_transport 头文件用来在ROS系统中的话题上发布和订阅图象消息  
#include<opencv2/core/core.hpp>  //OpenCV2标准头文件
#include <opencv2/opencv.hpp> 
#include<opencv2/imgproc/imgproc.hpp>    
#include <vector>
#include <cv.h>  
#include <algorithm>
int main(int argc, char** argv)
{
	ros::init(argc,argv,"talker");
	ros::NodeHandle n1;
        image_transport::ImageTransport it(n1);
	image_transport::Publisher pub=it.advertise("talker",1);
        //ros::Rate loop_rate(100);
 	cv::Mat img;
  //cv::VideoCapture cap(0);
   cv::VideoCapture cap;
   cout<<cap.isOpened()<<endl;
   cap.open("/home/tony/test/important.avi");
   cout<<cap.isOpened()<<endl;
 	cv::Mat frame;
	    sensor_msgs::ImagePtr frame_msg;
	    //ros::Rate loop_rate(5);
	    while(ros::ok())
	    {
		cap >> frame;
                imshow("1",frame);
                waitKey(1);
		if (!frame.empty())
		{
		    frame_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
		    pub.publish(frame_msg);
		    cv::waitKey(1);
		}
		ros::spinOnce();
		//loop_rate.sleep();
	    }
	return 0;
        /*VideoCapture cap(0);
 	Mat frame;
        cap>>frame;
        while(1){cap >> frame;
                imshow("1",frame);
                waitKey(20);}*/
}

