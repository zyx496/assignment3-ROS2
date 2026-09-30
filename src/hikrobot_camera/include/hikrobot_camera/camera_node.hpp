#ifndef HIKROBOT_CAMERA__CAMERA_NODE_HPP_
#define HIKROBOT_CAMERA__CAMERA_NODE_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

// 海康机器人 MVS SDK
#include "MvCameraControl.h"

namespace hikrobot_camera
{

class CameraNode : public rclcpp::Node
{
public:
  explicit CameraNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~CameraNode() override;

private:
  // ---- 参数 ----
  void declare_parameters();
  rcl_interfaces::msg::SetParametersResult on_parameters_changed(
    const std::vector<rclcpp::Parameter> & parameters);

  // ---- 相机操作（对应 SDK 示例 ConnectSpecCamera / ParametrizeCamera）----
  bool open_camera();                      // 枚举 -> 选择 -> 打开 -> 配置 -> 开始取流
  void close_camera();                     // 停取流 -> 关设备 -> 销毁句柄
  void apply_camera_parameters_locked();   // 把当前参数写入相机（调用前须持锁）

  // ---- 取流与重连（对应 GrabImage / ReconnectDemo）----
  void grab_loop();                        // 取流线程主循环
  bool grab_once();                        // 取一帧并发布
  void sleep_interruptible(double seconds);

  // ROS 接口
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  // MVS 句柄与线程
  void * handle_ = nullptr;
  std::mutex camera_mutex_;                // 保护所有 SDK 调用
  std::thread grab_thread_;
  std::atomic<bool> stopping_{false};
  std::atomic<bool> connected_{false};
  int consecutive_failures_ = 0;
  static constexpr int kMaxConsecutiveFailures = 10;

  // 可配置参数（与 config/camera.yaml 对应）
  std::string serial_number_;              // 相机序列号，空 = 第一台
  std::string image_topic_;                // 图像话题名
  std::string frame_id_;                   // 图像坐标系
  std::string pixel_format_;               // 像素格式，如 Mono8 / BayerRG8
  double exposure_time_;                   // 曝光时间（us）
  double gain_;                            // 增益（dB）
  double frame_rate_;                      // 采集帧率（fps）
  int grab_timeout_ms_;                    // 单次取流超时
  double reconnect_interval_;              // 重连间隔（s）
  bool image_qos_reliable_;                // true=RELIABLE，false=BEST_EFFORT
};

}  // namespace hikrobot_camera

#endif  // HIKROBOT_CAMERA__CAMERA_NODE_HPP_
