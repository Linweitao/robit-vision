#include "msg_transport.h"
//#define GIGN
//vector<float>c;
int flag=0;
RGB_Transport::RGB_Transport():it_(nh_),max_h(360),min_h(0),max_s(255),min_s(0),max_i(255),min_i(0),count(0)//构造函数
{
    thresholder.curvature_lookup_table(); //初始化标定表，即将qulv.txt的值存入thresholder的curvature二维表中
}   

RGB_Transport::~RGB_Transport() 
{  

}

void RGB_Transport::Create_Trackbar()
{
  if(type=='B'){
    namedWindow("ball_trackbar", 1);
    createTrackbar("h_max", "ball_trackbar", &max_h, 360, 0);
    createTrackbar("h_min", "ball_trackbar", &min_h, 360, 0);
    createTrackbar("s_max", "ball_trackbar", &max_s, 255, 0);
    createTrackbar("s_min", "ball_trackbar", &min_s, 255, 0);
    createTrackbar("i_max", "ball_trackbar", &max_i, 255, 0);
    createTrackbar("i_min", "ball_trackbar", &min_i, 255, 0);
  }
  else if(type=='O'){
    namedWindow("obstacle_trackbar", 1);
    createTrackbar("h_max", "obstacle_trackbar", &max_h, 360, 0);
    createTrackbar("h_min", "obstacle_trackbar", &min_h, 360, 0);
    createTrackbar("s_max", "obstacle_trackbar", &max_s, 255, 0);
    createTrackbar("s_min", "obstacle_trackbar", &min_s, 255, 0);
    createTrackbar("i_max", "obstacle_trackbar", &max_i, 255, 0);
    createTrackbar("i_min", "obstacle_trackbar", &min_i, 255, 0);
  }
}

void RGB_Transport::Msg_Subsribe()
{
	image_sub_ = it_.subscribe("talker", 1, &RGB_Transport::convert_callback, this);
        if(image_sub_==NULL)cout<<"error"; //定义图象接受器，订阅话题是“image”

}

void RGB_Transport::Msg_Publisher()
{   //建立两个发布器，一个发布球位置，一个发布黑障位置。
    location_pub_ball=nh_.advertise<Ball_Location>("location_ball",10);
    location_pub_obstacle=nh_.advertise<Obstacle_Location>("location_obstacle",10);
}

void RGB_Transport::convert_callback(const sensor_msgs::ImageConstPtr& msg)   
{     
    //接受如图像消息并转为Mat型对象
    try
    {  
         src=cv_bridge::toCvShare(msg, "bgr8")->image.clone();
    }  
    catch(cv_bridge::Exception& e)  //异常处理  
    {  
        ROS_ERROR("cv_bridge exception: %s", e.what());  
        return;  
    } 
    if(src.empty()) cout<<"error"<<endl;
    //将RGB图像转化为hsi图像
    thresholder.Converter(src,dst_hsi);
    //黑障信息传递
    if(type=='O'){
      dst_obstacle = dst_hsi.clone();
      thresholder.Threshold_Obstacle(src,dst_obstacle,max_h,min_h,max_s,min_s,max_i,min_i);
      //vector类型接收到的数据和角度数组
      vector<float> receive_dis;
      vector<float> receive_the;
      thresholder.Obstacle_Area(src,dst_obstacle,type,receive_the,receive_dis);
      int i=0;
      //判断是否阈值到目标，若阈值到则将距离和角度分别插入到要发送的消息中去
      //插入距离信息
      if(receive_dis.size()!=0)
        while(receive_dis.size()>i)
      {
          obstacle_location.distance.push_back(receive_dis[i++]);//obstacle_location类里面插入数据
      }
      i=0;
      //插入角度信息
      if(receive_the.size()!=0)
        while(receive_the.size()>i)
      {
          obstacle_location.theta.push_back(receive_the[i++]);
      }
      location_pub_obstacle.publish(obstacle_location);//将黑障消息发布
      obstacle_location.distance.clear();//发布完清空location型的obstacle_location的数据，以待下次调用
      obstacle_location.theta.clear();
    }
    //球信息传递
    else if(type=='B'){
      dst_ball = dst_hsi.clone();
      thresholder.Threshold_Ball(src,dst_ball,max_h,min_h,max_s,min_s,max_i,min_i);
      thresholder.Ball_Area(src,dst_ball,type,ball_location.theta,ball_location.distance);
      location_pub_ball.publish(ball_location);//将消息发布

    }
    //
}
void RGB_Transport::set_value(int maxh, int maxs, int maxi, int minh, int mins, int mini, char object)
{
    max_h=maxh;
    max_s=maxs;
    max_i=maxi;
    min_h=minh;
    min_s=mins;
    min_i=mini;
    type=object;
    Create_Trackbar(); //creat滑块
}

Mat RGB_Transport::get_src()
{
    //cout << "get_src" << endl;
    //imshow("output_src_get",src);
    //waitKey(1);
    return src;
}

int RGB_Transport::get_max_h()
{
    return max_h;
}

int RGB_Transport::get_min_h()
{
    return min_h;
}

int RGB_Transport::get_max_s()
{
    return max_s;
}

int RGB_Transport::get_min_s()
{
    return min_s;
}

int RGB_Transport::get_max_i()
{
    return max_i;
}

int RGB_Transport::get_min_i()
{
    return min_i;
}




