#ifndef GIGE_GIGE_CONFIG_H
#define GIGE_GIGE_CONFIG_H

#include "gige_cap/SetParam.h"

namespace gige
{
  /**@brief 节点名称*/
  const char NODE_NAME[] = "gige_cap";

  /**@brief 图像输出消息*/
  const char IMAGE_OUTPUT[] = "image";
  /**@brief 相机状态输出消息*/
  const char STATUS_OUTPUT[] = "status";

  /**@brief 参数：帧率*/
  const char PARAM_FRAME[] =  "~frame";
  /**@brief 帧率的默认值*/
  const float DEFAULT_FRAME = 30.0f;
  /**@brief 参数：快门时间*/
  const char PARAM_SHUTTER[] =  "~shutter";
  /**@brief 快门时间的默认值*/
  const float DEFAULT_SHUTTER = 10.0f;
  /**@brief 参数：增益*/
  const char PARAM_GAIN[] =  "~gain";
  /**@brief 增益的默认值*/
  const float DEFAULT_GAIN = 10.0f;

  /**
   * @namespace gige
   * @todo 白平衡的调整
   */

  /**@brief 超时时间(ms)*/
  const int GRAB_TIMEOUT = 200;
  /**@brief spin循环速率*/
  const int SPIN_RATE = 100;

  /**
   * @brief 状态消息
   *
   * 相机捕获的状态发生变化时发送的消息
   */
  enum StatusMsg
  {
    STATUS_NIL = 0, /**< 空白 */
    STATUS_ON_CONNECTED = 1, /**< 连接成功 */
    STATUS_ON_DISCONN = 2, /**< 连接断开 */
  };
};//end namespace gige

#endif
