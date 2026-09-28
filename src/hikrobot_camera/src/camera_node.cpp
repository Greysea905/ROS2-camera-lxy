#include "hikrobot_camera/camera_node.hpp"

#include <chrono>
#include <cstdio>   // snprintf（报错信息格式化）
#include <utility>  // std::move

namespace hikrobot_camera
{

CameraNode::CameraNode(const rclcpp::NodeOptions & options)
: Node("hikrobot_camera", options)
{
  // 声明参数
  this->declare_parameter<std::string>("serial_number", "DA0734524");
  this->declare_parameter<std::string>("image_topic", "image_raw");
  

  // 声明可调参数（单位见注释；真实范围连接后从相机查询，越界会在设置时报错）
  this->declare_parameter<double>("exposure_time", 10000.0);  // 曝光时间，微秒（10ms 则帧率上限可达 100fps）
  this->declare_parameter<double>("gain", 0.0);               // 增益，dB
  this->declare_parameter<double>("frame_rate", 100.0);       // 帧率，fps
  this->declare_parameter<int>("pixel_format", 0x02180014);   // 像素格式枚举（RGB8）

  // 注册参数回调：ros2 param set 时触发（保存句柄，句柄存活期间回调一直有效）（对 Lambda 表达式（匿名函数）不是很理解）
  params_callback_handle_ = this->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & params) {
      return this->onSetParameters(params);
    });


  // 创建图像发布器
  std::string topic = this->get_parameter("image_topic").as_string();
  image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(topic, 10);
  RCLCPP_INFO(get_logger(), "图像将发布到话题 /%s", topic.c_str());

  // 首次连接（失败不退出，交给重连定时器持续重试）
  std::string serial = this->get_parameter("serial_number").as_string();
  if (tryConnect(serial)) {
    RCLCPP_INFO(get_logger(), "相机连接成功, 序列号=%s", serial.c_str());
  } else {
    RCLCPP_WARN(get_logger(), "连接失败(%s)，将自动重试", camera_.lastError().c_str());
  }

  // 重连定时器：每 1s 检查一次，断线则尝试重连
  reconnect_timer_ = this->create_wall_timer(
    std::chrono::seconds(1),
    [this]() { this->reconnectTimerCallback(); });

  // 启动抓图线程：逐帧抓取，相机出多快抓多快（不丢帧）
  running_ = true;
  grab_thread_ = std::thread([this]() { this->grabLoop(); });
}

CameraNode::~CameraNode()
{
  running_ = false;  // 通知抓图线程退出
  if (grab_thread_.joinable()) {
    grab_thread_.join();  // 等待抓图线程结束
  }
  if (camera_.isOpen()) {
    camera_.stopGrabbing();
  }
}

// 开机 + 开始取流；首次连接和断线重连
bool CameraNode::tryConnect(const std::string & serial)
{
  if (!camera_.openBySerial(serial)) {
    return false;
  }
  if (!camera_.startGrabbing()) {
    camera_.close();
    return false;
  }
  applyParameters();       // 连接成功后，把当前参数应用到相机
  queryParameterRanges();  // 查询参数真实范围，用于越界校验和报错提示
  return true;
}

// 连接成功后，把当前参数值应用到相机（失败也继续，不中断连接）
void CameraNode::applyParameters()
{
  camera_.setExposureTime(static_cast<float>(this->get_parameter("exposure_time").as_double()));
  camera_.setGain(static_cast<float>(this->get_parameter("gain").as_double()));
  camera_.setFrameRate(static_cast<float>(this->get_parameter("frame_rate").as_double()));
  camera_.setPixelFormat(static_cast<uint64_t>(this->get_parameter("pixel_format").as_int()));
}

// 连接后从相机查询参数真实范围，用于越界校验和报错提示
void CameraNode::queryParameterRanges()
{
  float mn, mx;
  if (camera_.getExposureRange(mn, mx)) {
    exposure_min_ = mn;
    exposure_max_ = mx;
    RCLCPP_INFO(get_logger(), "曝光范围: [%.0f, %.0f] 微秒", mn, mx);
  }
  if (camera_.getGainRange(mn, mx)) {
    gain_min_ = mn;
    gain_max_ = mx;
    RCLCPP_INFO(get_logger(), "增益范围: [%.1f, %.1f] dB", mn, mx);
  }
  if (camera_.getFrameRateRange(mn, mx)) {
    framerate_min_ = mn;
    framerate_max_ = mx;
    RCLCPP_INFO(get_logger(), "帧率范围: [%.1f, %.1f] fps", mn, mx);
  }
}

// 参数回调：收到 ros2 param set 时被调用，校验并应用参数
rcl_interfaces::msg::SetParametersResult CameraNode::onSetParameters(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  // 相机未连接时只保存参数值（重连后会重新应用），不直接应用
  if (!camera_.isOpen()) {
    return result;
  }

  for (const auto & param : parameters) {
    const std::string & name = param.get_name();
    if (name == "exposure_time") {
      double v = param.as_double();
      if (v < exposure_min_ || v > exposure_max_) {
        char buf[128];
        std::snprintf(buf, sizeof(buf), "曝光时间 %.0f 超出范围 [%.0f, %.0f] 微秒",
          v, exposure_min_, exposure_max_);
        result.successful = false;
        result.reason = buf;
      } else if (!camera_.setExposureTime(static_cast<float>(v))) {
        result.successful = false;
        result.reason = camera_.lastError();
      }
    } else if (name == "gain") {
      double v = param.as_double();
      if (v < gain_min_ || v > gain_max_) {
        char buf[128];
        std::snprintf(buf, sizeof(buf), "增益 %.1f 超出范围 [%.1f, %.1f] dB",
          v, gain_min_, gain_max_);
        result.successful = false;
        result.reason = buf;
      } else if (!camera_.setGain(static_cast<float>(v))) {
        result.successful = false;
        result.reason = camera_.lastError();
      }
    } else if (name == "frame_rate") {
      double v = param.as_double();
      if (v < framerate_min_ || v > framerate_max_) {
        char buf[128];
        std::snprintf(buf, sizeof(buf), "帧率 %.1f 超出范围 [%.1f, %.1f] fps",
          v, framerate_min_, framerate_max_);
        result.successful = false;
        result.reason = buf;
      } else if (!camera_.setFrameRate(static_cast<float>(v))) {
        result.successful = false;
        result.reason = camera_.lastError();
      }
    } else if (name == "pixel_format") {
      if (!camera_.setPixelFormat(static_cast<uint64_t>(param.as_int()))) {
        result.successful = false;
        result.reason = camera_.lastError();
      }
    }
    // serial_number、image_topic 等其它参数这里不需要额外处理
  }

  return result;
}

// 取图：逐帧抓取并发布（不丢帧）
void CameraNode::grabLoop()
{
  while (running_) {
    // 未连接：交给重连定时器处理，短暂休眠后再试
    if (!camera_.isOpen()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }

    Frame frame;
    if (camera_.grab(frame)) {
      grab_fail_count_ = 0;
      publishFrame(std::move(frame));
    } else {
      // 取帧失败：累计次数，连续失败超过阈值则判定为断线
      grab_fail_count_++;
      if (grab_fail_count_ >= 3) {
        RCLCPP_WARN(get_logger(), "连续 %d 次取帧失败，判定断线", grab_fail_count_);
        camera_.close();  // 触发重连
        grab_fail_count_ = 0;
      }
    }
  }
}

// 把一帧图像发布出去
void CameraNode::publishFrame(Frame frame)
{
  sensor_msgs::msg::Image msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = "camera_frame";
  msg.height = frame.height;
  msg.width = frame.width;
  msg.encoding = "rgb8";             // 红绿蓝各 8 位
  msg.is_bigendian = 0;              // 小端字节序
  msg.step = frame.width * 3;        // 每行字节数 = 宽 × 3 字节/像素
  msg.data = std::move(frame.data);  // 转移数据，避免再拷贝一遍

  image_pub_->publish(msg);
}

// 重连定时器回调：断线后尝试重连
void CameraNode::reconnectTimerCallback()
{
  if (camera_.isOpen()) {
    return;
  }

  std::string serial = this->get_parameter("serial_number").as_string();
  if (tryConnect(serial)) {
    RCLCPP_INFO(get_logger(), "重连成功, 序列号=%s", serial.c_str());
  } else {
    RCLCPP_WARN(get_logger(), "重连失败(%s)，1 秒后重试", camera_.lastError().c_str());
  }
}

}  // namespace hikrobot_camera
