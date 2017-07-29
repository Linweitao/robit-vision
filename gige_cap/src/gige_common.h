#ifndef GIGE_GIGE_COMMON_H
#define GIGE_GIGE_COMMON_H

#include "gige_cap/gige_config.h"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <flycapture/FlyCapture2.h>
#include <functional>
using FlyCapture2::Camera;

namespace gige
{
  /**
   * @brief 全局公共结构
   *
   * 存储公用的全局变量，包括相机和消息发布器。
   */
  struct GlobalData
  {
    /**
     * @brief 相机的指针
     *
     * 只使用吉兆局域网相机的情况下，可以用GigECamera代替。
     */
    Camera* camera_;
    /**
     * @brief 发布图像消息的指针
     *
     * 消息发布到的话题为 #IMAGE_OUTPUT \n
     * 格式为 sensor_msgs::Image
     */
    image_transport::Publisher *image_pub_;
    /**
     * @brief 发布状态消息的指针
     *
     * 消息发布到的话题为 #STATUS_OUTPUT \n
     * 格式为 std_msgs::UInt32
     * @see StatusMsg
     */
    ros::Publisher *status_pub_;

    GlobalData()
    {
    }

    GlobalData(Camera* camera, image_transport::Publisher *image_pub, ros::Publisher *status_pub) :
      camera_(camera), image_pub_(image_pub), status_pub_(status_pub)
    {
    }
  };

  /**
   * @brief 主循环的循环体
   *
   * LoopBody是一个递归函数构成的状态机。 \n
   * 因为C++不方便直接使用递归，本意是写作 run = run(); \n
   * 因语法不支持故改为 run(&run); \n
   * 每次执行时，除了按当前的逻辑执行一帧，还会修改run为下一帧要做的事。
   */
  typedef std::function<void(void*)> LoopBody;
  /**
   * @brief 主循环的连续
   *
   * “连续”就是指接下来会发生的事。 \n
   * 因为一些状态的构造函数希望知道后续要执行的动作(成功之后做什么，失败后做什么)，
   * 所以需要传入LoopContinuation。 \n
   * 相应状态的LoopBody会调用LoopContinuation产生一个新的LoopBody并将其绑定到run。
   */
  typedef std::function<LoopBody(GlobalData)> LoopContinuation;

  /**
   * @brief 连接相机
   *
   * 当前相机处于断开状态，产生一个尝试连接相机的动作。
   * @param gd 全局变量
   * @param on_success 连接成功后的动作
   * @param on_failure 失败后的动作
   * @return 生成的循环体
   */
  LoopBody callConnect(GlobalData gd, LoopContinuation on_success, LoopContinuation on_failure);

  /**
   * @brief 捕获相机图像
   *
   * 当前相机处于连通状态，产生一个捕获相机图像的动作。
   * 捕获到的图像将会被作为消息发送到 gd.image_pub_ 。
   * @param gd 全局变量
   * @param on_success 连接成功后的动作
   * @param on_failure 失败后的动作
   * @return 生成的循环体
   */
  LoopBody callCapture(GlobalData gd, LoopContinuation on_success, LoopContinuation on_failure);
}; //end namespace gige

/**
 * @brief 全局相机指针，给SetParam用的
 */
extern Camera* g_gige_camera;

/**
 * @brief SetParam服务回调函数
 *
 * SetParam服务的回调函数
 * @param req 调参请求
 * @param res 调参响应
 * @return 服务是否成功执行
 */
bool setParam(gige_cap::SetParam::Request &req, gige_cap::SetParam::Response &res);

#endif
