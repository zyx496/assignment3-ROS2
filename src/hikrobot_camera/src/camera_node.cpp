#include "hikrobot_camera/camera_node.hpp"

#include <chrono>
#include <cstring>
#include <functional>

#include "sensor_msgs/image_encodings.hpp"

using namespace std::chrono_literals;

namespace hikrobot_camera
{

namespace
{
// 把 MVS 像素格式映射为 ROS 图像编码，空串表示暂不支持
std::string pixel_type_to_encoding(MvGvspPixelType type)
{
  switch (type) {
    case PixelType_Gvsp_Mono8:        return sensor_msgs::image_encodings::MONO8;
    case PixelType_Gvsp_BayerGR8:     return sensor_msgs::image_encodings::BAYER_GRBG8;
    case PixelType_Gvsp_BayerRG8:     return sensor_msgs::image_encodings::BAYER_RGGB8;
    case PixelType_Gvsp_BayerGB8:     return sensor_msgs::image_encodings::BAYER_GBRG8;
    case PixelType_Gvsp_BayerBG8:     return sensor_msgs::image_encodings::BAYER_BGGR8;
    case PixelType_Gvsp_RGB8_Packed:  return sensor_msgs::image_encodings::RGB8;
    case PixelType_Gvsp_BGR8_Packed:  return sensor_msgs::image_encodings::BGR8;
    default:                          return "";
  }
}

rcl_interfaces::msg::ParameterDescriptor make_desc(const std::string & text)
{
  rcl_interfaces::msg::ParameterDescriptor d;
  d.description = text;
  return d;
}
}  // namespace

CameraNode::CameraNode(const rclcpp::NodeOptions & options)
: Node("hikrobot_camera", options)
{
  declare_parameters();

  serial_number_      = get_parameter("serial_number").as_string();
  image_topic_        = get_parameter("image_topic").as_string();
  frame_id_           = get_parameter("frame_id").as_string();
  pixel_format_       = get_parameter("pixel_format").as_string();
  exposure_time_      = get_parameter("exposure_time").as_double();
  gain_               = get_parameter("gain").as_double();
  frame_rate_         = get_parameter("frame_rate").as_double();
  grab_timeout_ms_    = static_cast<int>(get_parameter("grab_timeout_ms").as_int());
  reconnect_interval_ = get_parameter("reconnect_interval").as_double();
  image_qos_reliable_ = get_parameter("image_qos_reliable").as_bool();

  // 图像话题默认用传感器 QoS（BEST_EFFORT），对应教程第七章；
  // rviz2 看图时若订阅端要求 RELIABLE，可把 image_qos_reliable 设为 true
  rclcpp::QoS qos = image_qos_reliable_
    ? rclcpp::QoS(rclcpp::KeepLast(5)).reliable()
    : rclcpp::QoS(rclcpp::KeepLast(5)).best_effort();
  image_pub_ = create_publisher<sensor_msgs::msg::Image>(image_topic_, qos);

  param_callback_handle_ = add_on_set_parameters_callback(
    std::bind(&CameraNode::on_parameters_changed, this, std::placeholders::_1));

  grab_thread_ = std::thread(&CameraNode::grab_loop, this);
  RCLCPP_INFO(get_logger(), "hikrobot_camera 节点已启动，等待连接相机……");
}

CameraNode::~CameraNode()
{
  stopping_ = true;
  if (grab_thread_.joinable()) {
    grab_thread_.join();
  }
  close_camera();   // 停取流、关设备、销毁句柄：资源清理
}

void CameraNode::declare_parameters()
{
  declare_parameter<std::string>(
    "serial_number", "", make_desc("相机序列号，留空则连接枚举到的第一台相机"));
  declare_parameter<std::string>(
    "image_topic", "/image_raw", make_desc("图像发布话题（修改后需重启生效）"));
  declare_parameter<std::string>(
    "frame_id", "camera", make_desc("图像消息的 frame_id"));
  declare_parameter<std::string>(
    "pixel_format", "Mono8",
    make_desc("像素格式：Mono8/BayerRG8/BayerBG8/BayerGB8/BayerGR8（重启生效）"));

  auto float_range_desc = [](const std::string & text, double lo, double hi) {
    rcl_interfaces::msg::ParameterDescriptor d = make_desc(text);
    rcl_interfaces::msg::FloatingPointRange r;
    r.from_value = lo; r.to_value = hi; r.step = 0.0;
    d.floating_point_range = {r};
    return d;
  };
  declare_parameter<double>(
    "exposure_time", 10000.0, float_range_desc("曝光时间（us），运行时可调", 0.0, 1e7));
  declare_parameter<double>(
    "gain", 0.0, float_range_desc("增益（dB），运行时可调", 0.0, 40.0));
  declare_parameter<double>(
    "frame_rate", 30.0, float_range_desc("采集帧率（fps），运行时可调", 0.1, 1000.0));

  declare_parameter<int>(
    "grab_timeout_ms", 1000, make_desc("单次取流超时时间（ms）"));
  declare_parameter<double>(
    "reconnect_interval", 1.0, make_desc("断线重连间隔（s）"));
  declare_parameter<bool>(
    "image_qos_reliable", false,
    make_desc("图像话题 QoS：true=RELIABLE，false=BEST_EFFORT（重启生效）"));
}

bool CameraNode::open_camera()
{
  std::lock_guard<std::mutex> lock(camera_mutex_);

  // 1. 枚举设备（GigE + USB3）
  MV_CC_DEVICE_INFO_LIST dev_list;
  std::memset(&dev_list, 0, sizeof(dev_list));
  int ret = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &dev_list);
  if (ret != MV_OK || dev_list.nDeviceNum == 0) {
    RCLCPP_WARN(get_logger(), "未枚举到相机（ret=0x%08x），%.1f 秒后重试",
      static_cast<unsigned int>(ret), reconnect_interval_);
    return false;
  }

  // 2. 打印设备列表并按序列号选择（serial_number 为空则取第一台）
  MV_CC_DEVICE_INFO * target = nullptr;
  for (unsigned int i = 0; i < dev_list.nDeviceNum; ++i) {
    MV_CC_DEVICE_INFO * info = dev_list.pDeviceInfo[i];
    std::string model, serial;
    if (info->nTLayerType == MV_GIGE_DEVICE) {
      model  = reinterpret_cast<char *>(info->SpecialInfo.stGigEInfo.chModelName);
      serial = reinterpret_cast<char *>(info->SpecialInfo.stGigEInfo.chSerialNumber);
    } else if (info->nTLayerType == MV_USB_DEVICE) {
      model  = reinterpret_cast<char *>(info->SpecialInfo.stUsb3VInfo.chModelName);
      serial = reinterpret_cast<char *>(info->SpecialInfo.stUsb3VInfo.chSerialNumber);
    } else {
      continue;
    }
    RCLCPP_INFO(get_logger(), "发现设备 [%u]: %s (SN: %s)", i, model.c_str(), serial.c_str());
    if (serial_number_.empty() && target == nullptr) { target = info; }
    if (!serial_number_.empty() && serial == serial_number_) { target = info; }
  }
  if (target == nullptr) {
    RCLCPP_WARN(get_logger(), "未找到序列号为 \"%s\" 的相机", serial_number_.c_str());
    return false;
  }

  // 3. 创建句柄并打开设备
  if (MV_CC_CreateHandle(&handle_, target) != MV_OK) {
    RCLCPP_ERROR(get_logger(), "MV_CC_CreateHandle 失败");
    handle_ = nullptr;
    return false;
  }
  if (MV_CC_OpenDevice(handle_) != MV_OK) {
    RCLCPP_ERROR(get_logger(), "MV_CC_OpenDevice 失败（GigE 相机请检查 IP 是否同网段）");
    MV_CC_DestroyHandle(handle_);
    handle_ = nullptr;
    return false;
  }

  // GigE 相机：设置最优网口包大小（官方示例做法，减少丢包）
  if (target->nTLayerType == MV_GIGE_DEVICE) {
    int packet_size = MV_CC_GetOptimalPacketSize(handle_);
    if (packet_size > 0) {
      MV_CC_SetIntValue(handle_, "GevSCPSPacketSize", packet_size);
    }
  }

  // 4. 采集模式：连续采集 + 关闭软/硬触发
  MV_CC_SetEnumValue(handle_, "AcquisitionMode", 2);   // 2 = Continuous
  MV_CC_SetEnumValue(handle_, "TriggerMode", MV_TRIGGER_MODE_OFF);

  // 5. 像素格式
  if (MV_CC_SetEnumValueByString(handle_, "PixelFormat", pixel_format_.c_str()) != MV_OK) {
    RCLCPP_WARN(get_logger(), "设置像素格式 %s 失败，使用相机当前格式", pixel_format_.c_str());
  }

  // 6. 写入曝光 / 增益 / 帧率
  apply_camera_parameters_locked();

  // 7. 回读实际参数并打印（对应“参数查看”要求）
  MVCC_INTVALUE st_w{}, st_h{};
  MV_CC_GetIntValue(handle_, "Width", &st_w);
  MV_CC_GetIntValue(handle_, "Height", &st_h);
  MVCC_FLOATVALUE st_exp{}, st_gain{}, st_fps{};
  MV_CC_GetFloatValue(handle_, "ExposureTime", &st_exp);
  MV_CC_GetFloatValue(handle_, "Gain", &st_gain);
  MV_CC_GetFloatValue(handle_, "ResultingFrameRate", &st_fps);
  RCLCPP_INFO(get_logger(),
    "相机已连接：%ux%u | 曝光 %.1f us（范围 %.1f~%.1f）| 增益 %.1f dB | 实际帧率 %.2f fps",
    st_w.nCurValue, st_h.nCurValue,
    st_exp.fCurValue, st_exp.fMin, st_exp.fMax, st_gain.fCurValue, st_fps.fCurValue);

  // 8. 开始取流
  if (MV_CC_StartGrabbing(handle_) != MV_OK) {
    RCLCPP_ERROR(get_logger(), "MV_CC_StartGrabbing 失败");
    MV_CC_CloseDevice(handle_);
    MV_CC_DestroyHandle(handle_);
    handle_ = nullptr;
    return false;
  }
  return true;
}

void CameraNode::close_camera()
{
  std::lock_guard<std::mutex> lock(camera_mutex_);
  if (handle_ == nullptr) { return; }
  MV_CC_StopGrabbing(handle_);
  MV_CC_CloseDevice(handle_);
  MV_CC_DestroyHandle(handle_);
  handle_ = nullptr;
  RCLCPP_INFO(get_logger(), "相机资源已释放");
}

void CameraNode::apply_camera_parameters_locked()
{
  // 曝光：先关自动再写值；增益同理；帧率需要先打开使能
  MV_CC_SetEnumValue(handle_, "ExposureAuto", 0);   // 0 = Off
  if (MV_CC_SetFloatValue(handle_, "ExposureTime",
      static_cast<float>(exposure_time_)) != MV_OK) {
    RCLCPP_WARN(get_logger(), "设置曝光时间 %.1f us 失败（超出相机范围？）", exposure_time_);
  }
  MV_CC_SetEnumValue(handle_, "GainAuto", 0);
  if (MV_CC_SetFloatValue(handle_, "Gain", static_cast<float>(gain_)) != MV_OK) {
    RCLCPP_WARN(get_logger(), "设置增益 %.1f dB 失败", gain_);
  }
  MV_CC_SetBoolValue(handle_, "AcquisitionFrameRateEnable", true);
  if (MV_CC_SetFloatValue(handle_, "AcquisitionFrameRate",
      static_cast<float>(frame_rate_)) != MV_OK) {
    RCLCPP_WARN(get_logger(), "设置帧率 %.1f fps 失败", frame_rate_);
  }
}

void CameraNode::grab_loop()
{
  while (!stopping_) {
    if (!connected_) {
      if (!open_camera()) {
        sleep_interruptible(reconnect_interval_);
        continue;
      }
      connected_ = true;
      consecutive_failures_ = 0;
      RCLCPP_INFO(get_logger(), "开始取流，发布到 %s", image_topic_.c_str());
    }

    if (grab_once()) {
      consecutive_failures_ = 0;
      continue;
    }

    // 取流失败：判断是超时抖动还是真掉线
    ++consecutive_failures_;
    bool device_gone = !MV_CC_IsDeviceConnected(handle_);
    if (device_gone || consecutive_failures_ >= kMaxConsecutiveFailures) {
      RCLCPP_ERROR(get_logger(), "相机掉线或连续取流失败，释放资源并尝试重连");
      close_camera();
      connected_ = false;
      consecutive_failures_ = 0;
      sleep_interruptible(reconnect_interval_);
    }
  }
}

bool CameraNode::grab_once()
{
  MV_FRAME_OUT frame;
  std::memset(&frame, 0, sizeof(frame));

  int ret;
  {
    std::lock_guard<std::mutex> lock(camera_mutex_);
    ret = MV_CC_GetImageBuffer(handle_, &frame, grab_timeout_ms_);
  }
  if (ret != MV_OK) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "取流失败 ret=0x%08x", static_cast<unsigned int>(ret));
    return false;
  }

  const auto & info = frame.stFrameInfo;
  std::string encoding = pixel_type_to_encoding(info.enPixelType);
  if (encoding.empty() || info.nHeight == 0 || info.nFrameLen == 0) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "暂不支持的像素格式 0x%08x，丢帧", static_cast<unsigned int>(info.enPixelType));
    std::lock_guard<std::mutex> lock(camera_mutex_);
    MV_CC_FreeImageBuffer(handle_, &frame);
    return true;   // 格式不支持不算掉线
  }

  sensor_msgs::msg::Image msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = frame_id_;
  msg.height = info.nHeight;
  msg.width = info.nWidth;
  msg.encoding = encoding;
  msg.is_bigendian = false;
  msg.step = info.nFrameLen / info.nHeight;   // 每行字节数
  msg.data.assign(frame.pBufAddr, frame.pBufAddr + info.nFrameLen);

  {
    std::lock_guard<std::mutex> lock(camera_mutex_);
    MV_CC_FreeImageBuffer(handle_, &frame);
  }
  image_pub_->publish(msg);
  return true;
}

void CameraNode::sleep_interruptible(double seconds)
{
  // 分段睡眠，保证 Ctrl+C 时能及时退出
  auto remaining = std::chrono::duration<double>(seconds);
  while (!stopping_ && remaining > 0s) {
    std::this_thread::sleep_for(50ms);
    remaining -= 50ms;
  }
}

rcl_interfaces::msg::SetParametersResult CameraNode::on_parameters_changed(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto & p : parameters) {
    const std::string name = p.get_name();

    if (name == "exposure_time" || name == "gain" || name == "frame_rate") {
      // 容忍命令行传入整数值
      double value = (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
        ? static_cast<double>(p.as_int()) : p.as_double();

      const char * key = nullptr;
      if (name == "exposure_time") { exposure_time_ = value; key = "ExposureTime"; }
      if (name == "gain")          { gain_ = value;          key = "Gain"; }
      if (name == "frame_rate")    { frame_rate_ = value;    key = "AcquisitionFrameRate"; }

      if (connected_ && handle_ != nullptr) {
        std::lock_guard<std::mutex> lock(camera_mutex_);
        if (MV_CC_SetFloatValue(handle_, key, static_cast<float>(value)) == MV_OK) {
          RCLCPP_INFO(get_logger(), "参数已生效：%s = %.2f", name.c_str(), value);
        } else {
          result.successful = false;
          result.reason = "相机拒绝参数 " + name + "（超出设备范围？）";
        }
      } else {
        RCLCPP_INFO(get_logger(), "相机未连接，%s=%.2f 已记录，连接后生效",
          name.c_str(), value);
      }
    } else if (name == "frame_id") {
      frame_id_ = p.as_string();
    } else if (name == "serial_number" || name == "image_topic" ||
      name == "pixel_format" || name == "image_qos_reliable") {
      RCLCPP_WARN(get_logger(), "参数 %s 需要重启节点后生效", name.c_str());
    }
  }
  return result;
}

}  // namespace hikrobot_camera
