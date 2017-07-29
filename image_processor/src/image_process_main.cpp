/*
 该节点主要根据Threshold记录的阈值来对图像进行识别。
  20170727
*/
#include "msg_transport.h"

using namespace gige;
using namespace gige_cap;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "threshold");
    //首先在threshold_value.txt中获取球和黑障的阈值化信息
    int bmax_h=0, bmax_s=0, bmax_i=0;
    int bmin_h=0, bmin_s=0, bmin_i=0;
    int omax_h=0, omax_s=0, omax_i=0;
    int omin_h=0, omin_s=0, omin_i=0;
    char s[100];
    FILE *fp;
    if((fp= fopen("threshold_value.txt", "r"))==NULL)
    {
	cout << "file does not excist" << endl;
        return 0;
    }
    else
    {
        for(int i=0;i<12;i++)
    	{
	    fgets(s, 100, fp);
            if(s[0]=='B')
	    {
    fscanf(fp, "%d %d %d %d %d %d", &bmax_h,&bmax_s,&bmax_i,&bmin_h,&bmin_s,&bmin_i);
	    }
            else if(s[0]=='O')
      {
    fscanf(fp, "%d %d %d %d %d %d", &omax_h,&omax_s,&omax_i,&omin_h,&omin_s,&omin_i);
      }
      }
    }
    //调用对象方法，订阅图像信息并处理。
    RGB_Transport Image_ball;
    RGB_Transport Image_obstacle;
    Image_ball.set_value(bmax_h, bmax_s, bmax_i, bmin_h, bmin_s, bmin_i, 'B');
    Image_obstacle.set_value(omax_h, omax_s, omax_i, omin_h, omin_s, omin_i, 'O');
    Image_ball.Msg_Subsribe();
    Image_obstacle.Msg_Subsribe();
    Image_ball.Msg_Publisher();
    Image_obstacle.Msg_Publisher();

    ros::NodeHandle n;
    //设置摄像头的图像增益服务，这段可以暂时忽略
    ros::ServiceClient client = n.serviceClient<SetParam>("/GigE/set_param");
    SetParam srv;
    srv.request.name = PARAM_GAIN;
    srv.request.value = 10.0f;
    if(client.call(srv))
    {
      cout << "Yes" << endl;
    }
    else
    {
      cout << "No" << endl;
    }

    while(ros::ok())
    {
        ros::spinOnce();
        //waitKey(10);
    }
}
