#include "hik_camera_driver/hik_camera_driver.hpp"
#include "hik_camera_driver/mvs_sdk_wrapper.hpp"
#ifdef HAVE_MVS_SDK
#include "MvCameraControl.h"
#include "PixelType.h"
#endif
#include <chrono>
#include <thread>
#include <opencv2/opencv.hpp>
#include <rclcpp/logging.hpp>

namespace hik_camera_driver
{

HikCameraDriver::HikCameraDriver()
    : Node("hik_camera_driver")
    , camera_connected_(false)
    , should_stop_(false)
    , frame_count_(0)
{
    // 声明参数
    this->declare_parameter("camera_ip", "");
    this->declare_parameter("camera_serial", "");
    this->declare_parameter("topic_name", "/image_raw");
    this->declare_parameter("frame_rate", 30.0);
    this->declare_parameter("exposure_time", 1000.0);
    this->declare_parameter("gain", 1.0);
    this->declare_parameter("pixel_format", "bgr8");
    this->declare_parameter("auto_reconnect", true);
    this->declare_parameter("reconnect_interval", 5);
    // 测试图像已移除，不再声明
    
    // 获取参数值
    camera_ip_ = this->get_parameter("camera_ip").as_string();
    camera_serial_ = this->get_parameter("camera_serial").as_string();
    topic_name_ = this->get_parameter("topic_name").as_string();
    frame_rate_ = this->get_parameter("frame_rate").as_double();
    exposure_time_ = this->get_parameter("exposure_time").as_double();
    gain_ = this->get_parameter("gain").as_double();
    pixel_format_ = this->get_parameter("pixel_format").as_string();
    auto_reconnect_ = this->get_parameter("auto_reconnect").as_bool();
    reconnect_interval_ = this->get_parameter("reconnect_interval").as_int();
    // 测试图像已移除
    
    // 设置参数变更回调
    auto param_callback = [this](const std::vector<rclcpp::Parameter> & parameters) -> rcl_interfaces::msg::SetParametersResult {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        this->onParameterChange(parameters);
        return result;
    };
    auto callback_handle = this->add_on_set_parameters_callback(param_callback);
    (void)callback_handle; // 避免未使用警告
    
    // 使用定时器延迟初始化（避免在构造函数中调用shared_from_this）
    init_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        [this]() {
            initializeAfterConstruction();
            init_timer_->cancel(); // 只执行一次
        }
    );
}

void HikCameraDriver::initializeAfterConstruction()
{
    // 创建图像发布者（在构造函数完成后）
    image_transport::ImageTransport it(shared_from_this());
    image_pub_ = it.advertise(topic_name_, 1);
    RCLCPP_INFO(this->get_logger(), "图像发布者已创建，话题: %s", topic_name_.c_str());
    
    // 初始化相机
    if (initializeCamera()) {
        RCLCPP_INFO(this->get_logger(), "相机初始化成功");
        
        // 启动采集线程
        should_stop_ = false;
        capture_thread_ = std::thread(&HikCameraDriver::cameraCaptureThread, this);
        
        // 设置重连定时器
        if (auto_reconnect_) {
            reconnect_timer_ = this->create_wall_timer(
                std::chrono::seconds(reconnect_interval_),
                [this]() {
                    if (!camera_connected_) {
                        RCLCPP_WARN(this->get_logger(), "尝试重新连接相机...");
                        reconnectCamera();
                    }
                }
            );
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "相机初始化失败");
    }
}

HikCameraDriver::~HikCameraDriver()
{
    should_stop_ = true;
    
    if (capture_thread_.joinable()) {
        capture_thread_.join();
    }
    
    releaseCamera();
}

bool HikCameraDriver::initializeCamera()
{
    std::lock_guard<std::mutex> lock(camera_mutex_);
    
    try {
        // 这里需要根据海康威视MVS SDK的实际API进行实现
        // 以下是伪代码示例
        
        RCLCPP_INFO(this->get_logger(), "正在初始化海康威视相机...");
        
        // 1. 初始化并打开设备（按序列号优先，其次IP）
#ifdef HAVE_MVS_SDK
        MV_CC_DEVICE_INFO_LIST dev_list{};
        memset(&dev_list, 0, sizeof(dev_list));
        int enum_ret = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &dev_list);
        if (enum_ret != MV_OK) {
            RCLCPP_ERROR(this->get_logger(), "枚举设备失败: %d", enum_ret);
            return false;
        }

        auto ip_to_string = [](unsigned int ip)->std::string {
            unsigned char b1 = (ip & 0xFF);
            unsigned char b2 = ((ip >> 8) & 0xFF);
            unsigned char b3 = ((ip >> 16) & 0xFF);
            unsigned char b4 = ((ip >> 24) & 0xFF);
            char buf[32];
            snprintf(buf, sizeof(buf), "%u.%u.%u.%u", b1, b2, b3, b4);
            return std::string(buf);
        };

        auto get_serial = [&](MV_CC_DEVICE_INFO* info)->std::string {
            if (!info) return "";
            if (info->nTLayerType == MV_GIGE_DEVICE) {
                const auto & g = info->SpecialInfo.stGigEInfo;
                if (g.chSerialNumber[0]) return std::string(reinterpret_cast<const char*>(g.chSerialNumber));
                if (g.chUserDefinedName[0]) return std::string(reinterpret_cast<const char*>(g.chUserDefinedName));
            } else if (info->nTLayerType == MV_USB_DEVICE) {
                const auto & u = info->SpecialInfo.stUsb3VInfo;
                if (u.chSerialNumber[0]) return std::string(reinterpret_cast<const char*>(u.chSerialNumber));
                if (u.chUserDefinedName[0]) return std::string(reinterpret_cast<const char*>(u.chUserDefinedName));
            }
            return "";
        };

        MV_CC_DEVICE_INFO* selected = nullptr;
        // 打印所有设备，帮助定位
        for (unsigned int i = 0; i < dev_list.nDeviceNum; ++i) {
            MV_CC_DEVICE_INFO* info = dev_list.pDeviceInfo[i];
            if (!info) continue;
            std::string kind = (info->nTLayerType == MV_GIGE_DEVICE) ? "GigE" : (info->nTLayerType == MV_USB_DEVICE ? "USB" : "Other");
            std::string sn = get_serial(info);
            std::string ip;
            if (info->nTLayerType == MV_GIGE_DEVICE) {
                ip = ip_to_string(info->SpecialInfo.stGigEInfo.nCurrentIp);
            }
            RCLCPP_INFO(this->get_logger(), "发现设备[%u]: 类型=%s, SN=%s, IP=%s", i, kind.c_str(), sn.c_str(), ip.c_str());
        }

        // 先按序列号匹配
        if (!camera_serial_.empty()) {
            for (unsigned int i = 0; i < dev_list.nDeviceNum; ++i) {
                MV_CC_DEVICE_INFO* info = dev_list.pDeviceInfo[i];
                if (!info) continue;
                if (get_serial(info) == camera_serial_) { selected = info; break; }
            }
        }
        // 再按IP匹配
        if (!selected && !camera_ip_.empty()) {
            for (unsigned int i = 0; i < dev_list.nDeviceNum; ++i) {
                MV_CC_DEVICE_INFO* info = dev_list.pDeviceInfo[i];
                if (!info || info->nTLayerType != MV_GIGE_DEVICE) continue;
                std::string ip = ip_to_string(info->SpecialInfo.stGigEInfo.nCurrentIp);
                if (ip == camera_ip_) { selected = info; break; }
            }
        }

        if (!selected) {
            RCLCPP_ERROR(this->get_logger(), "未找到匹配的海康设备（SN=%s, IP=%s）", camera_serial_.c_str(), camera_ip_.c_str());
            camera_connected_ = false;
            return false;
        }

        int nRet = MV_CC_CreateHandle(&camera_handle_, selected);
        if (nRet != MV_OK || camera_handle_ == nullptr) {
            RCLCPP_ERROR(this->get_logger(), "CreateHandle失败: %d, handle=%p", nRet, camera_handle_);
            return false;
        }

        // 打开设备：独占→控制→监视→默认
        nRet = MV_CC_OpenDevice(camera_handle_, MV_ACCESS_Exclusive, 0);
        if (nRet != MV_OK) {
            RCLCPP_WARN(this->get_logger(), "独占模式打开失败(%d)，尝试控制模式...", nRet);
            nRet = MV_CC_OpenDevice(camera_handle_, MV_ACCESS_Control, 0);
        }
        if (nRet != MV_OK) {
            RCLCPP_WARN(this->get_logger(), "控制模式打开失败(%d)，尝试监视模式...", nRet);
            nRet = MV_CC_OpenDevice(camera_handle_, MV_ACCESS_Monitor, 0);
        }
        if (nRet != MV_OK) {
            RCLCPP_WARN(this->get_logger(), "监视模式打开失败(%d)，尝试默认模式...", nRet);
            nRet = MV_CC_OpenDevice(camera_handle_);
        }
        if (nRet != MV_OK) {
            RCLCPP_ERROR(this->get_logger(), "OpenDevice失败: %d", nRet);
            MV_CC_DestroyHandle(camera_handle_);
            camera_handle_ = nullptr;
            camera_connected_ = false;
            return false;
        }

        // 设置连续采集模式与触发关闭（若可用）
        // 忽略错误但打印日志，避免部分虚拟设备不支持时报错中断
        do {
            int r1 = MV_CC_SetEnumValue(camera_handle_, "TriggerMode", 0); // Off
            if (r1 != MV_OK) RCLCPP_WARN(this->get_logger(), "设置TriggerMode失败: %d", r1);
            int r2 = MV_CC_SetEnumValue(camera_handle_, "AcquisitionMode", 2); // Continuous(一般为2)
            if (r2 != MV_OK) RCLCPP_WARN(this->get_logger(), "设置AcquisitionMode失败: %d", r2);
            int r3 = MV_CC_SetFloatValue(camera_handle_, "AcquisitionFrameRate", static_cast<float>(frame_rate_));
            if (r3 != MV_OK) RCLCPP_WARN(this->get_logger(), "设置AcquisitionFrameRate失败: %d", r3);
        } while (0);

        int sg = MV_CC_StartGrabbing(camera_handle_);
        if (sg != MV_OK) {
            RCLCPP_ERROR(this->get_logger(), "StartGrabbing失败: %d", sg);
            MV_CC_CloseDevice(camera_handle_);
            MV_CC_DestroyHandle(camera_handle_);
            camera_handle_ = nullptr;
            camera_connected_ = false;
            return false;
        }
#endif
        
        // 2. 创建设备句柄
        // MV_CC_DEVICE_INFO_LIST stDeviceList;
        // memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
        
        // 3. 枚举设备
        // int nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
        
        // 4. 根据IP或序列号选择设备
        // 这里需要根据实际需求实现设备选择逻辑
        
        // 5. 创建设备句柄
        // nRet = MV_CC_CreateHandle(&camera_handle_, &stDeviceList.pDeviceInfo[0]);
        
        // 6. 打开设备
        // nRet = MV_CC_OpenDevice(camera_handle_);
        
        // 7. 设置相机参数
        setExposureTime(exposure_time_);
        setGain(gain_);
        setFrameRate(frame_rate_);
        setPixelFormat(pixel_format_);
        
        // 8. 开始采集
        // nRet = MV_CC_StartGrabbing(camera_handle_);
        
        camera_connected_ = true;
        RCLCPP_INFO(this->get_logger(), "相机连接成功");
        
        return true;
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "相机初始化异常: %s", e.what());
        camera_connected_ = false;
        return false;
    }
}

void HikCameraDriver::releaseCamera()
{
    std::lock_guard<std::mutex> lock(camera_mutex_);
    
    if (camera_connected_) {
        try {
            // 停止采集
#ifdef HAVE_MVS_SDK
            if (camera_handle_) {
                MV_CC_StopGrabbing(camera_handle_);
                MV_CC_CloseDevice(camera_handle_);
                MV_CC_DestroyHandle(camera_handle_);
                camera_handle_ = nullptr;
            }
#endif
            
            // 关闭设备
            // MV_CC_CloseDevice(camera_handle_);
            
            // 销毁句柄
            // MV_CC_DestroyHandle(camera_handle_);
            
            camera_connected_ = false;
            RCLCPP_INFO(this->get_logger(), "相机资源已释放");
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "释放相机资源时发生异常: %s", e.what());
        }
    }
}

void HikCameraDriver::cameraCaptureThread()
{
    RCLCPP_INFO(this->get_logger(), "相机采集线程已启动");
    
    while (!should_stop_ && rclcpp::ok()) {
        if (camera_connected_) {
            try {
                cv::Mat frame;

#ifdef HAVE_MVS_SDK
                if (camera_handle_) {
                    MV_FRAME_OUT_INFO_EX info{};
                    memset(&info, 0, sizeof(info));
                    static std::vector<unsigned char> buffer(5*1024*1024);
                    int nRet = MV_CC_GetOneFrameTimeout(camera_handle_, buffer.data(), buffer.size(), &info, 1000);
                    if (nRet == MV_OK && info.nWidth > 0 && info.nHeight > 0) {
                        // 常见格式处理：BGR/RGB/Mono/Bayer
                        if (info.enPixelType == PixelType_Gvsp_BGR8_Packed) {
                            frame = cv::Mat(info.nHeight, info.nWidth, CV_8UC3, buffer.data()).clone();
                        } else if (info.enPixelType == PixelType_Gvsp_RGB8_Packed) {
                            cv::Mat rgb(info.nHeight, info.nWidth, CV_8UC3, buffer.data());
                            cv::cvtColor(rgb, frame, cv::COLOR_RGB2BGR);
                        } else if (info.enPixelType == PixelType_Gvsp_Mono8) {
                            cv::Mat mono(info.nHeight, info.nWidth, CV_8UC1, buffer.data());
                            cv::cvtColor(mono, frame, cv::COLOR_GRAY2BGR);
                        } else if (info.enPixelType == PixelType_Gvsp_BayerRG8) {
                            cv::Mat raw(info.nHeight, info.nWidth, CV_8UC1, buffer.data());
                            cv::cvtColor(raw, frame, cv::COLOR_BayerRG2BGR);
                        } else if (info.enPixelType == PixelType_Gvsp_BayerBG8) {
                            cv::Mat raw(info.nHeight, info.nWidth, CV_8UC1, buffer.data());
                            cv::cvtColor(raw, frame, cv::COLOR_BayerBG2BGR);
                        } else if (info.enPixelType == PixelType_Gvsp_BayerGB8) {
                            cv::Mat raw(info.nHeight, info.nWidth, CV_8UC1, buffer.data());
                            cv::cvtColor(raw, frame, cv::COLOR_BayerGB2BGR);
                        } else if (info.enPixelType == PixelType_Gvsp_BayerGR8) {
                            cv::Mat raw(info.nHeight, info.nWidth, CV_8UC1, buffer.data());
                            cv::cvtColor(raw, frame, cv::COLOR_BayerGR2BGR);
                        } else {
                            RCLCPP_WARN(this->get_logger(), "不支持的像素格式: %d", info.enPixelType);
                        }
                    }
                }
#endif

                if (frame.empty()) {
                    // 未获取到帧，跳过本轮
                    std::this_thread::sleep_for(std::chrono::milliseconds(5));
                    continue;
                }

                if (image_pub_.getNumSubscribers() >= 0) {
                    auto now = this->now();
                    publishImage(frame, now);
                }
                
                // 控制帧率
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(static_cast<int>(1000.0 / frame_rate_))
                );
                
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "图像采集异常: %s", e.what());
                camera_connected_ = false;
            }
        } else {
            // 相机未连接，等待一段时间后重试
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    
    RCLCPP_INFO(this->get_logger(), "相机采集线程已停止");
}

void HikCameraDriver::onParameterChange(const std::vector<rclcpp::Parameter> & parameters)
{
    for (const auto & param : parameters) {
        if (param.get_name() == "exposure_time") {
            exposure_time_ = param.as_double();
            setExposureTime(exposure_time_);
            RCLCPP_INFO(this->get_logger(), "曝光时间设置为: %f", exposure_time_);
        }
        else if (param.get_name() == "gain") {
            gain_ = param.as_double();
            setGain(gain_);
            RCLCPP_INFO(this->get_logger(), "增益设置为: %f", gain_);
        }
        else if (param.get_name() == "frame_rate") {
            frame_rate_ = param.as_double();
            setFrameRate(frame_rate_);
            RCLCPP_INFO(this->get_logger(), "帧率设置为: %f", frame_rate_);
        }
        else if (param.get_name() == "pixel_format") {
            pixel_format_ = param.as_string();
            setPixelFormat(pixel_format_);
            RCLCPP_INFO(this->get_logger(), "像素格式设置为: %s", pixel_format_.c_str());
        }
    }
}

bool HikCameraDriver::setExposureTime(double exposure_time)
{
    if (!camera_connected_) return false;
    
    try {
        // 这里需要根据海康威视MVS SDK的实际API进行实现
        // int nRet = MV_CC_SetFloatValue(camera_handle_, "ExposureTime", exposure_time);
        RCLCPP_DEBUG(this->get_logger(), "设置曝光时间: %f", exposure_time);
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "设置曝光时间失败: %s", e.what());
        return false;
    }
}

bool HikCameraDriver::setGain(double gain)
{
    if (!camera_connected_) return false;
    
    try {
        // 这里需要根据海康威视MVS SDK的实际API进行实现
        // int nRet = MV_CC_SetFloatValue(camera_handle_, "Gain", gain);
        RCLCPP_DEBUG(this->get_logger(), "设置增益: %f", gain);
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "设置增益失败: %s", e.what());
        return false;
    }
}

bool HikCameraDriver::setFrameRate(double frame_rate)
{
    if (!camera_connected_) return false;
    
    try {
        // 这里需要根据海康威视MVS SDK的实际API进行实现
        // int nRet = MV_CC_SetFloatValue(camera_handle_, "AcquisitionFrameRate", frame_rate);
        RCLCPP_DEBUG(this->get_logger(), "设置帧率: %f", frame_rate);
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "设置帧率失败: %s", e.what());
        return false;
    }
}

bool HikCameraDriver::setPixelFormat(const std::string & pixel_format)
{
    if (!camera_connected_) return false;
    
    try {
        // 这里需要根据海康威视MVS SDK的实际API进行实现
        // 需要将字符串格式转换为SDK对应的枚举值
        RCLCPP_DEBUG(this->get_logger(), "设置像素格式: %s", pixel_format.c_str());
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "设置像素格式失败: %s", e.what());
        return false;
    }
}

void HikCameraDriver::reconnectCamera()
{
    RCLCPP_INFO(this->get_logger(), "尝试重新连接相机...");
    
    releaseCamera();
    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    if (initializeCamera()) {
        RCLCPP_INFO(this->get_logger(), "相机重连成功");
    } else {
        RCLCPP_WARN(this->get_logger(), "相机重连失败，将在 %d 秒后重试", reconnect_interval_);
    }
}

void HikCameraDriver::publishImage(const cv::Mat & image, const rclcpp::Time & timestamp)
{
    try {
        std_msgs::msg::Header header;
        header.stamp = timestamp;
        header.frame_id = "camera_link";
        
        sensor_msgs::msg::Image::SharedPtr img_msg = cv_bridge::CvImage(
            header, pixel_format_, image).toImageMsg();
        
        image_pub_.publish(img_msg);
        
        // 更新统计信息
        last_frame_time_ = timestamp;
        
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge异常: %s", e.what());
    }
}

} // namespace hik_camera_driver

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<hik_camera_driver::HikCameraDriver>();
    
    RCLCPP_INFO(node->get_logger(), "海康威视相机驱动节点已启动");
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}
