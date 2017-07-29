#include "gige_common.h"
using namespace gige;
using namespace gige_cap;
using FlyCapture2::Error;
using FlyCapture2::Property;
using FlyCapture2::PropertyType;
using FlyCapture2::ErrorType;
using FlyCapture2::FC2Config;

bool setParam(SetParam::Request &req, SetParam::Response &res)
{
  PropertyType type;
  if(req.name == PARAM_FRAME)
  {
    type = FlyCapture2::FRAME_RATE;
  }
  else if(req.name == PARAM_SHUTTER)
  {
    type = FlyCapture2::SHUTTER;
  }
  else if(req.name == PARAM_GAIN)
  {
    type = FlyCapture2::GAIN;
  }
  else
  {
    return false;
  }

  Property property(type);
  property.absControl = true;
  property.onePush = false;
  property.onOff = true;
  property.autoManualMode = false;
  property.absValue = req.value;

  Error error = g_gige_camera->SetProperty(&property, false);
  if (error != FlyCapture2::PGRERROR_OK)
  {
    ROS_WARN("SetProperty %s failed with (%d): %s", req.name.c_str(), error.GetType(), error.GetDescription());
    res.result = true;
    return true;
  }
  else
  {
    res.result = false;
    return false;
  }
}

