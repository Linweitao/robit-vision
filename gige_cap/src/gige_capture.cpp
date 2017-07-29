#include "gige_common.h"
#include <std_msgs/UInt32.h>
#include <opencv2/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
using FlyCapture2::Error;
using FlyCapture2::ErrorType;
using FlyCapture2::Image;
using std_msgs::UInt32;

namespace gige
{
  LoopBody callCapture(GlobalData gd, LoopContinuation on_success, LoopContinuation on_failure)
  {
    return [=](void* ptr)->void
    {
      LoopBody* run = reinterpret_cast<LoopBody*>(ptr);
      Error error;

      //捕获相机图像
      Image raw_image;
      error = gd.camera_->RetrieveBuffer(&raw_image);
      if(error != FlyCapture2::PGRERROR_OK)
      {
        //这里判断是超时错误还是格式转换错误
        if(error == FlyCapture2::PGRERROR_TIMEOUT)
        {
          //如果是超时错误，直接发消息并转换为连接状态
          //因为局域网在未断线的情况下是不会有超时的
          //如果需要超时次数设定，给callCapture增加新的参数
          ROS_ERROR("Timeout, re-connecting");
          UInt32 msg;
          msg.data = static_cast<unsigned long>(STATUS_ON_DISCONN);
          gd.status_pub_->publish(msg);

          if(gd.camera_->IsConnected())
          {
            gd.camera_->StopCapture();
            gd.camera_->Disconnect();
          }

          *run = on_failure(gd);
          return ;
        }
        else
        {
          //格式转换错误只记录INFO，不退出
          ROS_INFO("RetrieveBuffer failed with (%d): %s", error.GetType(), error.GetDescription());
        }
      }//end if

      //Camera => OpenCV
      Image rgb_image;
      raw_image.Convert(FlyCapture2::PIXEL_FORMAT_BGR, &rgb_image);
      unsigned int row_bytes = (double)rgb_image.GetReceivedDataSize()/(double)rgb_image.GetRows();
      cv::Mat cv_image = cv::Mat(rgb_image.GetRows(), rgb_image.GetCols(), CV_8UC3, rgb_image.GetData(), row_bytes);

      //CvMat => bridge
      cv_bridge::CvImage cvi;
      cvi.header.stamp = ros::Time::now();
      cvi.header.frame_id = "image";
      cvi.encoding = "bgr8";
      cvi.image = cv_image;

      //bridge => message
      sensor_msgs::Image im;
      cvi.toImageMsg(im);
      gd.image_pub_->publish(im);

      //继续
      *run = on_success(gd);
      return ;
    };
  } //end callCapture

}; //end namespace gige
