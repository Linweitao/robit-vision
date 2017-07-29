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
void chattercallback_obstacle(const image_processor::Obstacle_Location msg)//location类型的形参
{
        cout<<"obstacle"<<" ";
        int i=0;
        float sub=0;
	cout<<msg.distance.size()<<endl;
        while(1)
	{
         if(i>=msg.distance.size())break;
      	  sub=msg.distance[i];
      	  cout<<"dstance:"<<sub<<" ";
          sub=msg.theta[i++];
          cout<<"theta:"<<sub<<" ";
          
	}
        cout<<endl;
        
}
void chattercallback_ball(const image_processor::Ball_Location msg)//location类型的形参
{
        int i=0;
        cout<<"ball"<<" ";
          cout<<"dstance:"<<msg.distance<<" ";
          cout<<"theta:"<<msg.theta<<" ";
        cout<<endl;

}
int main(int argc, char** argv)
{
  ros::init(argc,argv,"listenner");
	ros::NodeHandle nh;
  ros::Subscriber so=nh.subscribe("location_obstacle",10,chattercallback_obstacle);
  ros::Subscriber sb=nh.subscribe("location_ball",10,chattercallback_ball);
        //ros::Rate loop_rate(100);
 	ros::spin();
	return 0;
}
