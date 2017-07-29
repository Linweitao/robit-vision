/*
 该节点主要是进行图像阈值化并记录阈值化值
  20170727
*/
#include "msg_transport.h"

using namespace gige;
using namespace gige_cap;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Thresholder");
    RGB_Transport Image;
    int max_h=360, max_s=255, max_i=255;
    int min_h=0, min_s=0, min_i=0;
    char type;
    //设置你需要阈值的东西，暂时只支持球和黑障
    cout << "Please input the object you thresholded: " ;
    cout << " 'B' represents Ball, 'L' represents Line, 'F' represents Field, 'O' represents   Obstacle " << endl;
    cin>>type;
    Image.set_value(max_h, max_s, max_i, min_h, min_s, min_i, type);
    Image.Msg_Subsribe();
    Image.Msg_Publisher();
    ros::NodeHandle n;
    //设置摄像头图像增益服务，暂时忽略这一段
    ros::ServiceClient client = n.serviceClient<SetParam>("/GigE/set_param");
    char flag;
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
    ros::spin();
    //结束程序，写入阈值
    cout << "Please" << endl;
    max_h=Image.get_max_h();
    min_h=Image.get_min_h();
    max_s=Image.get_max_s();
    min_s=Image.get_min_s();
    max_i=Image.get_max_i();
    min_i=Image.get_min_i();
    cout << " input the object you thresholded " << endl;
    flag = type;
    FILE *fp=fopen("threshold_value.txt","a+");
    fseek(fp,0l,SEEK_END);
    if(flag=='B')
    {
        fputs("\nBall_Threshold\n",fp);
    }
    else if(flag=='L')
    {
        fputs("\nLine_Threshold\n",fp);
    }
    else if(flag=='F')
    {
        fputs("\nField_Threshold\n",fp);
    }
    else if(flag=='O')
    {
        fputs("\nObstacle_Threshold\n",fp);
    }
    else
    {
        cout << "wrong type;Exiting" << endl;
    }
    cout << "recording threshold value from H->S->I by max->min" << endl;
    fprintf(fp,"%d %d %d %d %d %d\n",max_h,max_s,max_i,min_h,min_s,min_i); 
    fclose(fp);
}




















