#include "Thresholder.h"
#include <math.h>
int ct=0;
void Image_Thresholder::Converter(const cv::Mat& src, cv::Mat& dst)
{
    dst = cv::Mat_<Vec3f>(src.rows, src.cols);
    int i, j;
    float rv, gv, bv;
    int ri, gi, bi;
    double hv, sv, iv, minv;
    for (i = 0; i < src.rows; i++)
    {
	const uchar* datIn = src.ptr<const uchar>(i);
	float* datOut = dst.ptr<float>(i);
	for (j = 0; j < src.cols; j++)
	{
	    //转化成0.0 ~ 1.0
	    ri = *(datIn++);
            gi = *(datIn++);
	    bi = *(datIn++);
	    rv = ri / 255.0f;
	    gv = gi / 255.0f;
            bv = bi / 255.0f;
	    minv = min(rv, min(gv, bv));
	    iv = (rv + gv + bv) / 3.0f;
	    sv = 1.0f - minv / (iv + eps);
	    //这里h采用近似算法
	    if (minv == rv)
	    {
		if ((gv + bv - 2 * rv) <= 0.001)
		{
		    hv = 0.0;
		}
		if (rv <= 0.001&&gv <= 0.001&&bv <= 0.001)
		{
	            hv = 0;
		}
		else
		{
		    hv = (bv - rv) / 3 / (gv + bv - 2 * rv) + 1.0 / 3.0;
		}
	    }
	    else if (minv == gv)
	    {
		if ((rv + bv - 2 * gv) <= 0.001)
		{
		    hv = 0.0;
		}
		if (rv <= 0.001&&gv <= 0.001&&bv <= 0.001)
		{
		    hv = 0;
		}
		else
		{
		    hv = (rv - gv) / 3 / (rv + bv - 2 * gv) + 2.0 / 3.0;
		}
	    }
	    else
	    {
		if ((rv + gv - 2 * bv) <= 0.001)
		{
		    hv = 0.0;
		}
		else
		{
		    if (rv <= 0.001&&gv <= 0.001&&bv <= 0.001)
		    {
			hv = 0;
		    }
		    hv = (gv - bv) / 3 / (rv + gv - 2 * bv) + 0.0001;
		}
	    }
	    //输出
	    *(datOut++) = hv*360;
	    *(datOut++) = sv*255;
	    *(datOut++) = iv*255;
	}
    }
    //cout << hv << " " << sv << " " << iv << endl;
    //cv::imshow("output_converter_hsi", dst);  
    cv::waitKey(5);
}

void Image_Thresholder::Threshold_Ball(const cv::Mat& src, cv::Mat& dst, int max_h, int min_h, int max_s, int min_s, int max_i, int min_i)
{
    float hv, sv, iv;
    int i, j;
    for (i = 0; i < src.rows; i++)
    {
	float* datIn = dst.ptr<float>(i);
	float* datOut = dst.ptr<float>(i);
	for (j = 0; j < src.cols; j++)
	{		
	    hv = *(datIn++);
	    sv = *(datIn++);
	    iv = *(datIn++);
	    int a = src.at<Vec3b>(i, j)[0];
	    int b = src.at<Vec3b>(i, j)[1];
            int c = src.at<Vec3b>(i, j)[2];
	    if (a == b||b == c)
	    {
		hv = 0;
	    }
	    if (hv >= min_h&&hv <= max_h&&sv >= min_s&&sv <= max_s&& iv >= min_i&&iv <= max_i)
	    {
		hv = 1.0;
		sv = 1.0;
		iv = 1.0;
	    }
            else if (min_h>max_h&&(hv <= min_h||hv >= max_h)&&sv >= min_s&&sv <= max_s&& iv >= min_i&&iv <= max_i)
	    {
		hv = 1.0;
		sv = 1.0;
		iv = 1.0;
	    }
 
	    else
	    {
		hv = 0.0;
		sv = 0.0;
		iv = 0.0;
		
	    }
	    *(datOut++) = hv;
	    *(datOut++) = sv;
	    *(datOut++) = iv;
         }
    }
    //cv::imshow("threshold_ball", dst);  
    cv::waitKey(1);   
}
void Image_Thresholder::Threshold_Obstacle(const cv::Mat& src, cv::Mat& dst, int max_h, int min_h, int max_s, int min_s, int max_i, int min_i)
{
    float hv, sv, iv;
    int i, j;
    for (i = 0; i < src.rows; i++)
    {
  float* datIn = dst.ptr<float>(i);
  float* datOut = dst.ptr<float>(i);
  for (j = 0; j < src.cols; j++)
  {
      hv = *(datIn++);
      sv = *(datIn++);
      iv = *(datIn++);
      int a = src.at<Vec3b>(i, j)[0];
      int b = src.at<Vec3b>(i, j)[1];
            int c = src.at<Vec3b>(i, j)[2];
      if (a == b||b == c)
      {
    hv = 0;
      }
      if (hv >= min_h&&hv <= max_h&&sv >= min_s&&sv <= max_s&& iv >= min_i&&iv <= max_i)
      {
    hv = 1.0;
    sv = 1.0;
    iv = 1.0;
      }
            else if (min_h>max_h&&(hv <= min_h||hv >= max_h)&&sv >= min_s&&sv <= max_s&& iv >= min_i&&iv <= max_i)
      {
    hv = 1.0;
    sv = 1.0;
    iv = 1.0;
      }

      else
      {
    hv = 0.0;
    sv = 0.0;
    iv = 0.0;

      }
      *(datOut++) = hv;
      *(datOut++) = sv;
      *(datOut++) = iv;
         }
    }
    //cv::imshow("threshold_ball", dst);
    cv::waitKey(1);
}

void Image_Thresholder::Obstacle_Area(Mat& src,Mat& dst,char type,vector<float> &theta, vector<float> &distance)
{
    float re_dis,ta;//计算出float类型距离和角度
    vector<vector <cv::Point> > contours;//轮廓容器数组
    vector<Mat> vec;
    split(dst, vec);
    Mat result1;
    vec[0].convertTo(vec[0], CV_8U);//把矩阵vec[0]转为unsing char类型的矩阵，注在转换过程中有可能数值上会出现一些变化，这个要注意
    cv::findContours(vec[0], contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE); // 寻找连通域轮廓（多个同时可以找到）
    dst.copyTo(result1);
    distance.clear();//先清空vector<float>distance里的数据再进行数据传入
    theta.clear();
        for (size_t i = 0; i < contours.size(); i++)
        {
	    double area = cv::contourArea(contours[i]);
            
            if(area>=1000&&area<=1500)
            {
               cv::Rect Rect = cv::boundingRect(contours[i]);
               field_xcordinate = Rect.x+0.5*Rect.width;
               field_ycordinate = Rect.y-0.5*Rect.height;
               cv::rectangle(result1, Rect, cv::Scalar(255));
               re_dis=Calculate_Distance(field_xcordinate,field_ycordinate);
               if(re_dis!=0.0)
                   {
                     distance.push_back(re_dis);
                     ta =atan2(field_xcordinate,field_ycordinate)/2/PI*360;
	   	     if(ta<0)
		       {
			   ta=360+ta;
		       }
			theta.push_back(ta);
                   }
            }
        }
        cv::imshow("Obstacle", result1);
        cv::waitKey(1);
}

void Image_Thresholder::Ball_Area(Mat& src,Mat& dst,char type,float &theta,float &distance)
{
    float re_dis,ta;//计算出float类型距离和角度
    vector<vector <cv::Point> > contours;//轮廓容器数组
    vector<Mat> vec;
    split(dst, vec);
    Mat result1;
    int i = vec[0].type();
    vec[0].convertTo(vec[0], CV_8U);
    cv::findContours(vec[0], contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE); // 寻找连通域轮廓（多个同时可以找到）
    dst.copyTo(result1);
        double maxArea = 0;
        vector<cv::Point> maxContour;//最大轮廓
        for (size_t i = 0; i < contours.size(); i++)
        {
      double area = cv::contourArea(contours[i]);
            if (area > maxArea)
            {
          maxArea = area;
          maxContour = contours[i];
            }
        }
        if (maxContour.empty())
        {
      cout << "ball did not detected" << endl;
        }
        cv::Rect maxRect = cv::boundingRect(maxContour);//形成矩形框
  ball_xcordinate = maxRect.x + 0.5*maxRect.width;
  ball_ycordinate = maxRect.y - 0.5*maxRect.height;
  cv::rectangle(result1, maxRect, cv::Scalar(255));
        cv::imshow("Ball", result1);
        cv::waitKey(1);
        re_dis=Calculate_Distance(ball_xcordinate,ball_ycordinate);
        distance=re_dis;
             ta =atan2(ball_xcordinate,ball_ycordinate)/2/PI*360;
       if(ta<0)
         {
       ta=360+ta;
         }
    theta=ta;//同步算出角度
}

float Image_Thresholder::Calculate_Distance(float x, float y)
{
	
	x -= 320.0;
	y -= 240.0;
	char s[100];
	int i,count=0;
	int distance = sqrt(x*x + y*y);
	float real_dis=0.0;
	if (distance <= 50)
	{
		//cout << "within the agent" << endl;
	}
	else if (distance >= 200)
	{
		//cout << "data can not be trusted" << endl;
	}
	else
	{
		for (int i = 0; i < 200; i++)
		{
			if (distance == curvature[i][0])//对照curvature数组得到distance
			{
				real_dis= curvature[i][1];
				//cout << "real distance：" << real_distance << endl;
				break;
			}
		}
	}
        return real_dis;
}

void Image_Thresholder::curvature_lookup_table()//查表qulv得到curvature[][]
{
	char s[100];
	float i;
	int j, k = 0;
	FILE *fp;
	fp = fopen("qulv.txt", "r"); /*打开文字文件只读*/
	fgets(s, 7, fp); /*从文件中读取23个字符*/
	//cout << s << endl;
	for (j = 0; j < 152; j++)
	{
		k = 0;
		fscanf(fp, "%f", &i); /*读取整型数*/
		curvature[j][k] = i;
		//printf("%f", curvature[j][k]);
		fgetc(fp); /*读取一个字符*/
		fscanf(fp, "%f", &i); /*读取整型数*/
		curvature[j][k+1] = i;
		//printf("%f\n", curvature[j][k+1]);

	}
	fclose(fp);
}

















