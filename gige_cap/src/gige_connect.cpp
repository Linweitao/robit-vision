#include "gige_common.h"
#include <std_msgs/UInt32.h>
using FlyCapture2::Error;
using FlyCapture2::Property;
using FlyCapture2::PropertyType;
using FlyCapture2::ErrorType;
using FlyCapture2::FC2Config;
using std_msgs::UInt32;

namespace gige
{
  /**
   * @brief 设置相机属性(单值)
   *
   * 以绝对值方式设置一个单值的相机属性，不支持白平衡。
   * @param camera 要设置的相机
   * @param name 要设置的属性名在参数服务器中的名字，私有参数前加~
   * @param type 要设置的属性类型
   * @param default_val 参数服务器未配置时，属性的默认值(极少使用)
   */
  bool setProperty(Camera* camera, const char* name, PropertyType type, float default_val)
  {
    Property property(type);
    property.absControl = true;
    property.onePush = false;
    property.onOff = true;
    property.autoManualMode = false;

    if(ros::param::has(name))
    {
      ros::param::get(name, property.absValue);
    }
    else
    {
      property.absValue = default_val;
    }

    Error error = camera->SetProperty(&property, false);
    if (error != FlyCapture2::PGRERROR_OK)
    {
      ROS_WARN("SetProperty %s failed with (%d): %s", name, error.GetType(), error.GetDescription());
      return false;
    }
    else
      return true;
  } //end setProperty

  LoopBody callConnect(GlobalData gd, LoopContinuation on_success, LoopContinuation on_failure)
  {
    return [=](void* ptr)->void
    {
      LoopBody* run = reinterpret_cast<LoopBody*>(ptr);
      Error error;

      //连接相机，如果失败了就继续连接，不发送状态
      error = gd.camera_->Connect(0);
      if(error != FlyCapture2::PGRERROR_OK)
      {
        ROS_WARN("Connect failed with (%d): %s", error.GetType(), error.GetDescription());
        *run = on_failure(gd);
        return ;
      }

      //配置相机参数(顺序错可能失败)，忽略错误
      setProperty(gd.camera_, PARAM_FRAME, FlyCapture2::FRAME_RATE, DEFAULT_FRAME);
      setProperty(gd.camera_, PARAM_SHUTTER, FlyCapture2::SHUTTER, DEFAULT_SHUTTER);
      setProperty(gd.camera_, PARAM_GAIN, FlyCapture2::GAIN, DEFAULT_GAIN);

      //配置连接选项，忽略错误
      FC2Config config;
      gd.camera_->GetConfiguration(&config);
      config.grabTimeout = GRAB_TIMEOUT;
      gd.camera_->SetConfiguration(&config);

      //开始捕获，如果失败了就继续连接，不发送状态
      error = gd.camera_->StartCapture();
      if(error != FlyCapture2::PGRERROR_OK)
      {
        ROS_WARN("StartCapture failed with (%d): %s", error.GetType(), error.GetDescription());
        *run = on_failure(gd);
        return ;
      }

      //连接成功，发送状态消息
      UInt32 msg;
      msg.data = static_cast<unsigned long>(STATUS_ON_CONNECTED);
      gd.status_pub_->publish(msg);
      *run = on_success(gd);
      return ;
    };
  } //end callConnect

}; //end namespace gige
