#include "sign.h"
#include <math.h>


sign_mark::sign_mark() :it_(nh_)
{  
    	curvature_lookup_table(); 

}


void sign_mark::Msg_Subsribe()
{
	//image_sub_ = it_.subscribe("/GigE/image", 1, &sign_mark::callback, this);
	image_sub_ = it_.subscribe("talker", 1, &sign_mark::callback, this);//接受本机摄像头信息，用于测试

}
void sign_mark::Loc_Publisher()
{
        location_pub = nh_.advertise<localization::Samples>("signlocation",10);
    
}

void sign_mark::callback(const sensor_msgs::ImageConstPtr& msg)  
{
    Mat src,src_hsi;
    /*count+=1;
    gettimeofday(&thresholder.end,NULL);
    if(thresholder.end.tv_sec-thresholder.start.tv_sec>=1)
    {
	cout << "fps= " << count << endl;
        thresholder.start.tv_sec=thresholder.end.tv_sec;
        count=0;
    }*/
    try  
    {  
         src=cv_bridge::toCvShare(msg, "bgr8")->image.clone(); //将ROS消息中图象信息提取，生成新cv类型的图象复制给CvImage指针
    }  
    catch(cv_bridge::Exception& e)  //异常处理  
    {  
        ROS_ERROR("cv_bridge exception: %s", e.what());  
        return;  
    } 
    //src=imread("4-180.jpg",1);
    int i, j, n;//i是angle的个数，j是像素距离    
    float angle=PI/scan_num;
    int x,y;
    
    int num=0;
    int d=0;
    Vec3b red(0,0,255);
    Mat src_2=src.clone();
    for(i=0;i < 2*scan_num;i++)//角度从0到<360
    {   n=0;//n计算白点元素的个数，大于三个时才计入
  if(num>=sign_num) break;
      for(j=40;j<200;j++)//距离从30到200个像素点
        {   if(num>=sign_num) break;
            y=int(src.rows/2+j*sin(i*angle));
            x=int(src.cols/2-j*cos(i*angle));
	    //如果为白点标红并记录连续白点个数n
            if(src.at<Vec3b>(y, x)[0]>white_threshold&&src.at<Vec3b>(y, x)[1]>white_threshold&&src.at<Vec3b>(y, x)[2]>white_threshold)
              //if(src.at<Vec3b>(y, x)[0]>40&&src.at<Vec3b>(y, x)[1]>40&&src.at<Vec3b>(y, x)[2]>40)
              {
                 src_2.at<Vec3b>(y, x)[0]=0;
               src_2.at<Vec3b>(y, x)[1]=0;
                 src_2.at<Vec3b>(y, x)[2]=255;
         	      n++;
              }
	    //否则标为黑，若连续白点个数足够则标记为特征点且将数据传入msg_output中，并重置n
            else{
            src_2.at<Vec3b>(y, x)[0]=0;
            src_2.at<Vec3b>(y, x)[1]=0;
            src_2.at<Vec3b>(y, x)[2]=0;
                 if(n>=2){

            	   for(int i1=y-2;i1<=y+2;i1++)
              	   for(int j1=x-2;j1<=x+2;j1++){
                   //cout<<"0"<<endl;
                   if(i1<1200&&j1<1800&&i1>=0&&j1>=0) src_2.at<Vec3b>(i1, j1)=red;
                   }

	           //cout<<x<<" "<<y<<endl;
	           d=Calculate_Distance(x,y);
	           //cout<<d<<endl;
	           if(d!=0){
	           //cout<<x<<" "<<y<<endl;
			 msg_output.local_x[num]=cos(i*angle)*d;
			 msg_output.local_y[num]=sin(i*angle)*d;
			 msg_output.local_theta[num]=i*angle;
			 num++;
			 n=0;
       //j+=10;
		    }
		   //cout<<i*angle/PI*180<<" ";
		   //cout<<Calculate_Distance(x,y)<<endl;
                   n=0;
              	  }
                 
                 n=0;
                }
		
            if(i==(2*scan_num-1)&&num<sign_num){
        i=0;
        if(num==0) cout<<"can't find suitable sign."<<endl;
	      break;
              }
        }
    }

    //cout<<endl;
    //for(i=0;i<sign_num;i++)
    //cout<<msg_output.local_x[i]<<" "<<msg_output.local_y[i]<<" "<<msg_output.local_theta[i]*180/PI<<endl;

    location_pub.publish(msg_output);
    imshow("src",src_2);
    waitKey(1);
}



float sign_mark::Calculate_Distance(float x, float y)
{
	
	x -= 320.0;
	y -= 240.0;
	char s[100];
	int i,count=0;
	int distance = sqrt(x*x + y*y);
	//cout<<"distance"<<distance<<endl;
	float real_dis=0.0;
	if (distance <= 50)
	{
		//cout << "within the agent" << endl;
	}
  else if (distance >= 180)
	{
		//cout << "data can not be trusted" << endl;
	}
	else
	{      
		for (int i = 0; i < 200; i++)
		{       //cout<<"dis"<<distance<<"curvature"<<curvature[i][0]<<endl;
			if (distance == curvature[i][0])//对照curvature数组得到distance
			{
				real_dis= curvature[i][1];
				
				//cout << "real distance：" << real_dis << endl;
				break;
			}
		}
	}
        return real_dis;
}
void sign_mark::curvature_lookup_table()//查表qulv得到curvature[][]
{
	char s[100];
	float i;
	int j, k = 0;
	FILE *fp;
	fp = fopen("qulv.txt", "r"); /*打开文字文件只读*/
	fgets(s, 8, fp); /*从文件中读取23个字符*/
	//cout << s << endl;
	for (j = 0; j < 150; j++)
	{
		k = 0;
		fscanf(fp, "%f", &i); /*读取整型数*/
		curvature[j][k] = i;
		//cout<<curvature[j][k]<<endl;
		fgetc(fp); /*读取一个字符*/
		fscanf(fp, "%f", &i); /*读取整型数*/
		curvature[j][k+1] = i;
		//printf("%f\n", curvature[j][k+1]);

	}
	fclose(fp);
	/*for (j = 0; j < 150; j++)
	{
		cout<<curvature[j][0]<<" "<<curvature[j][1]<<endl;
	}*/

}

