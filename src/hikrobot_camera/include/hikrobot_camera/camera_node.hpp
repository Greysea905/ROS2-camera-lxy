#ifndef HIKROBOT_CAMERA__CAMERA_NODE_HPP_
#define HIKROBOT_CAMERA__CAMERA_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "hikrobot_camera/mvs_camera.hpp"

#include <atomic>
#include <thread>
#include <vector>

namespace hikrobot_camera
{

class CameraNode : public rclcpp::Node
{
public:
  explicit CameraNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~CameraNode();

private:
  void grabLoop();                 // 抓图线程主循环：逐帧抓取并发布
  void publishFrame(Frame frame);  // 把一帧图像发布出去
  void reconnectTimerCallback();   // 断线后周期性尝试重连
  bool tryConnect(const std::string & serial);  // 打开相机 + 开始取流（首次连接与重连共用）
  // 参数回调：收到 ros2 param set 时被调用，校验并应用参数
  rcl_interfaces::msg::SetParametersResult onSetParameters(
    const std::vector<rclcpp::Parameter> & parameters);
  void applyParameters();  // 连接成功后把当前参数应用到相机
  void queryParameterRanges();  // 连接后从相机查询参数真实范围

  MvsCamera camera_;
  rclcpp::TimerBase::SharedPtr reconnect_timer_;  // 重连定时器（1s）
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;  // 图像发布器
  std::thread grab_thread_;                       // 抓图线程（逐帧抓取）
  std::atomic<bool> running_{false};              // 抓图线程运行标志
  int grab_fail_count_ = 0;  // 连续取帧失败次数
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;  // 参数回调句柄
  // 相机支持的真实参数范围（连接后查询得到；未连接前用这里的默认值）
  float exposure_min_ = 1.0f;
  float exposure_max_ = 1000000.0f;
  float gain_min_ = 0.0f;
  float gain_max_ = 24.0f;
  float framerate_min_ = 1.0f;
  float framerate_max_ = 100.0f;
};

}  // namespace hikrobot_camera

#endif  // HIKROBOT_CAMERA__CAMERA_NODE_HPP_
