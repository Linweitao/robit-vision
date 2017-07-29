#include "gige_common.h"
#include <std_msgs/UInt32.h>
using namespace gige;
using std_msgs::UInt32;

Camera* g_gige_camera;

/**
 * @defgroup retryover_group 自动重连循环
 * 定义循环结构：失败自动重连。
 * 无限尝试且没有等待时间。
 */
/**@{*/
LoopBody connectRetry(GlobalData gd);
LoopBody captureRetry(GlobalData gd);
LoopBody connectRetry(GlobalData gd)
{
  return callConnect(gd, captureRetry, connectRetry);
}
LoopBody captureRetry(GlobalData gd)
{
  return callCapture(gd, captureRetry, connectRetry);
}
/**@}*/

int main(int argc, char *argv[])
{
  ros::init(argc, argv, NODE_NAME);
  ros::NodeHandle handle;
  Camera camera;

  //发布图像主题
  image_transport::ImageTransport transport(handle);
  image_transport::Publisher image_pub = transport.advertise(IMAGE_OUTPUT, 2);

  //发布状态主题
  ros::Publisher status_pub = handle.advertise<UInt32>(STATUS_OUTPUT, 5);
  ros::Rate loop_rate(SPIN_RATE);

  //发布服务
  g_gige_camera = &camera;
  ros::ServiceServer service = handle.advertiseService("set_param", setParam);

  //初始状态 = 未连接
  LoopBody run = connectRetry(GlobalData(&camera, &image_pub, &status_pub));

  //主循环
  while(ros::ok() && run != nullptr)
  {
    run(&run); //注意run是状态机，执行时会改变其动态绑定
    ros::spinOnce();
    loop_rate.sleep();
  }

  //关闭相机结束
  if(camera.IsConnected())
  {
    camera.StopCapture();
    camera.Disconnect();
  }

  return 0;
}
