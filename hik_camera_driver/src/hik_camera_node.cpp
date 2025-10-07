#include "hik_camera_driver/hik_camera_driver.hpp"
#ifdef HAVE_MVS_SDK
#include "MvCameraControl.h"
#include "PixelType.h"
#endif
#include <chrono>
#include <thread>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
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
        RCLCPP_INFO(this->get_logger(), "收到参数更新请求(%zu)", parameters.size());
        this->onParameterChange(parameters);
        return result;
    };
    param_cb_handle_ = this->add_on_set_parameters_callback(param_callback);
    
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
    // 打印是否启用SDK分支
#ifdef HAVE_MVS_SDK
    RCLCPP_INFO(this->get_logger(), "HAVE_MVS_SDK: 启用");
#else
    RCLCPP_WARN(this->get_logger(), "HAVE_MVS_SDK: 未启用(将不会对相机硬件写入)");
#endif
    
    // 初始化相机
    if (initializeCamera()) {
        RCLCPP_INFO(this->get_logger(), "相机初始化成功");
        
        // 启动采集线程
        should_stop_ = false;
        capture_thread_ = std::thread(&HikCameraDriver::cameraCaptureThread, this);
    } else {
        RCLCPP_ERROR(this->get_logger(), "相机初始化失败");
    }

    // 设置重连定时器（无论初始化是否成功，都建立定时器以便后续自动重连）
    if (auto_reconnect_) {
        reconnect_timer_ = this->create_wall_timer(
            std::chrono::seconds(reconnect_interval_),
            [this]() {
                if (!camera_connected_) {
                    RCLCPP_WARN(this->get_logger(), "尝试重新连接相机...");
                    reconnectCamera();
                } else {
                    // 若长时间无帧或超时累计，触发保护性重连
                    auto now = this->now();
                    double since_last = (now - last_frame_time_).seconds();
                    if (since_last > 5.0) {
                        RCLCPP_WARN(this->get_logger(), "检测到%.1f秒无帧输出，执行安全重连", since_last);
                        reconnectCamera();
                    }
                }
            }
        );
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

        // 设置连续采集与采样限速（若可用）
        // 顺序：TriggerMode=Off -> AcquisitionMode=Continuous -> AcquisitionFrameRateEnable=1 -> AcquisitionFrameRate=目标值
        do {
            int r1 = MV_CC_SetEnumValue(camera_handle_, "TriggerMode", 0); // Off
            if (r1 != MV_OK) RCLCPP_WARN(this->get_logger(), "设置TriggerMode失败: %d", r1);

            int r2 = MV_CC_SetEnumValue(camera_handle_, "AcquisitionMode", 2); // Continuous
            if (r2 != MV_OK) RCLCPP_WARN(this->get_logger(), "设置AcquisitionMode失败: %d", r2);

            int rEn = MV_CC_SetBoolValue(camera_handle_, "AcquisitionFrameRateEnable", true);
            if (rEn != MV_OK) RCLCPP_WARN(this->get_logger(), "开启AcquisitionFrameRateEnable失败: %d", rEn);

            int r3 = MV_CC_SetFloatValue(camera_handle_, "AcquisitionFrameRate", static_cast<float>(frame_rate_));
            if (r3 != MV_OK) RCLCPP_WARN(this->get_logger(), "设置AcquisitionFrameRate失败: %d", r3);

            // 读回确认
            MVCC_FLOATVALUE fps_val{};
            int gr = MV_CC_GetFloatValue(camera_handle_, "AcquisitionFrameRate", &fps_val);
            if (gr == MV_OK)
                RCLCPP_INFO(this->get_logger(), "采样帧率(读取): %.2f FPS (目标=%.2f)", fps_val.fCurValue, frame_rate_);
            else
                RCLCPP_WARN(this->get_logger(), "读取AcquisitionFrameRate失败: %d", gr);
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
                            RCLCPP_WARN(this->get_logger(), "不支持的像素格式: %ld", static_cast<long>(info.enPixelType));
                        }
                    } else if (nRet == MV_E_HANDLE || nRet == 0x8000000B /*MV_E_ACCESS*/ || nRet == 0x8000000C /*MV_E_DISCONNECT*/ ) {
                        // 句柄错误/访问错误/断连
                        RCLCPP_ERROR(this->get_logger(), "采集失败错误码=%d，标记断开连接", nRet);
                        camera_connected_ = false;
                        continue;
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
                
                // 控制帧率（使用本地快照以减少数据竞争）
                double target_rate = frame_rate_;
                if (target_rate < 1e-3) {
                    target_rate = 1.0; // 避免除零
                }
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(static_cast<int>(1000.0 / target_rate))
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
        RCLCPP_DEBUG(this->get_logger(), "设置曝光时间: %f", exposure_time);
#ifdef HAVE_MVS_SDK
        if (!camera_handle_) return false;
        // 关闭自动曝光以确保手动曝光生效
        MV_CC_SetEnumValue(camera_handle_, "ExposureAuto", 0); // Off
        // 限幅到设备允许范围
        MVCC_FLOATVALUE range{};
        if (MV_CC_GetFloatValue(camera_handle_, "ExposureTime", &range) == MV_OK) {
            if (exposure_time < range.fMin) exposure_time = range.fMin;
            if (exposure_time > range.fMax) exposure_time = range.fMax;
        }
        // 曝光单位通常为微秒(us)
        int nRet = MV_CC_SetFloatValue(camera_handle_, "ExposureTime", static_cast<float>(exposure_time));
        if (nRet != MV_OK) {
            RCLCPP_WARN(this->get_logger(), "设置ExposureTime失败(%d)，尝试ExposureTimeAbs...", nRet);
            nRet = MV_CC_SetFloatValue(camera_handle_, "ExposureTimeAbs", static_cast<float>(exposure_time));
            if (nRet != MV_OK) {
                RCLCPP_ERROR(this->get_logger(), "ExposureTimeAbs仍失败: %d", nRet);
                return false;
            }
        }
        // 读回确认
        MVCC_FLOATVALUE cur{};
        if (MV_CC_GetFloatValue(camera_handle_, "ExposureTime", &cur) == MV_OK) {
            RCLCPP_INFO(this->get_logger(), "曝光(读取): %.2f us", cur.fCurValue);
        }
#endif
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
        RCLCPP_DEBUG(this->get_logger(), "设置增益: %f", gain);
#ifdef HAVE_MVS_SDK
        if (!camera_handle_) return false;
        // 根据型号，可能为Gain或AnalogGain等，这里先用通用节点名
        // 关闭自动增益
        MV_CC_SetEnumValue(camera_handle_, "GainAuto", 0); // Off
        // 限幅
        MVCC_FLOATVALUE range{};
        if (MV_CC_GetFloatValue(camera_handle_, "Gain", &range) == MV_OK) {
            if (gain < range.fMin) gain = range.fMin;
            if (gain > range.fMax) gain = range.fMax;
        }
        int nRet = MV_CC_SetFloatValue(camera_handle_, "Gain", static_cast<float>(gain));
        if (nRet != MV_OK) {
            RCLCPP_WARN(this->get_logger(), "设置Gain失败(%d)，尝试AnalogGain...", nRet);
            nRet = MV_CC_SetFloatValue(camera_handle_, "AnalogGain", static_cast<float>(gain));
            if (nRet != MV_OK) {
                RCLCPP_WARN(this->get_logger(), "AnalogGain失败(%d)，尝试GainRaw...", nRet);
                nRet = MV_CC_SetIntValue(camera_handle_, "GainRaw", static_cast<int>(gain));
                if (nRet != MV_OK) {
                    RCLCPP_ERROR(this->get_logger(), "GainRaw仍失败: %d", nRet);
                    return false;
                }
            }
        }
        MVCC_FLOATVALUE cur{};
        if (MV_CC_GetFloatValue(camera_handle_, "Gain", &cur) == MV_OK) {
            RCLCPP_INFO(this->get_logger(), "增益(读取): %.2f", cur.fCurValue);
        }
#endif
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
#ifdef HAVE_MVS_SDK
        if (!camera_handle_) return false;
        // 确保连续采集 + 启用采样限速
        MV_CC_SetEnumValue(camera_handle_, "TriggerMode", 0); // Off
        MV_CC_SetEnumValue(camera_handle_, "AcquisitionMode", 2); // Continuous
        MV_CC_SetBoolValue(camera_handle_, "AcquisitionFrameRateEnable", true);

        // 限幅
        MVCC_FLOATVALUE range{};
        if (MV_CC_GetFloatValue(camera_handle_, "AcquisitionFrameRate", &range) == MV_OK) {
            if (frame_rate < range.fMin) frame_rate = range.fMin;
            if (frame_rate > range.fMax) frame_rate = range.fMax;
        }

        int nRet = MV_CC_SetFloatValue(camera_handle_, "AcquisitionFrameRate", static_cast<float>(frame_rate));
        if (nRet != MV_OK) {
            RCLCPP_WARN(this->get_logger(), "设置AcquisitionFrameRate失败: %d", nRet);
            return false;
        }

        // 读回确认
        MVCC_FLOATVALUE fps_val{};
        int gr = MV_CC_GetFloatValue(camera_handle_, "AcquisitionFrameRate", &fps_val);
        if (gr == MV_OK) {
            RCLCPP_INFO(this->get_logger(), "采样帧率(读取): %.2f FPS (目标=%.2f)", fps_val.fCurValue, frame_rate);
        } else {
            RCLCPP_WARN(this->get_logger(), "读取AcquisitionFrameRate失败: %d", gr);
        }

        // 若设备支持，读取实际输出帧率
        MVCC_FLOATVALUE resulting{};
        if (MV_CC_GetFloatValue(camera_handle_, "ResultingFrameRate", &resulting) == MV_OK) {
            RCLCPP_INFO(this->get_logger(), "实际输出帧率(ResultingFrameRate): %.2f FPS", resulting.fCurValue);
        }
#else
        RCLCPP_DEBUG(this->get_logger(), "设置帧率(模拟): %f", frame_rate);
#endif
        frame_rate_ = frame_rate;
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
        RCLCPP_DEBUG(this->get_logger(), "设置像素格式: %s", pixel_format.c_str());
#ifdef HAVE_MVS_SDK
        if (!camera_handle_) return false;
        // 停采集后修改像素格式更稳妥
        MV_CC_StopGrabbing(camera_handle_);
        // 将常用格式字符串映射为 MVS PixelType
        int pixel_enum = -1;
        if (pixel_format == "bgr8") {
            pixel_enum = PixelType_Gvsp_BGR8_Packed;
        } else if (pixel_format == "rgb8") {
            pixel_enum = PixelType_Gvsp_RGB8_Packed;
        } else if (pixel_format == "mono8" || pixel_format == "mono8") {
            pixel_enum = PixelType_Gvsp_Mono8;
        } else if (pixel_format == "bayer_rg8") {
            pixel_enum = PixelType_Gvsp_BayerRG8;
        } else if (pixel_format == "bayer_bg8") {
            pixel_enum = PixelType_Gvsp_BayerBG8;
        } else if (pixel_format == "bayer_gr8") {
            pixel_enum = PixelType_Gvsp_BayerGR8;
        } else if (pixel_format == "bayer_gb8") {
            pixel_enum = PixelType_Gvsp_BayerGB8;
        }

        if (pixel_enum != -1) {
            int nRet = MV_CC_SetEnumValue(camera_handle_, "PixelFormat", pixel_enum);
            if (nRet != MV_OK) {
                RCLCPP_WARN(this->get_logger(), "设置PixelFormat失败: %d", nRet);
                MV_CC_StartGrabbing(camera_handle_);
                return false;
            }
            // 读回确认
            MVCC_ENUMVALUE cur{};
            if (MV_CC_GetEnumValue(camera_handle_, "PixelFormat", &cur) == MV_OK) {
                RCLCPP_INFO(this->get_logger(), "像素格式(读取): %u", cur.nCurValue);
            }
            MV_CC_StartGrabbing(camera_handle_);
        } else {
            RCLCPP_WARN(this->get_logger(), "未知的像素格式字符串: %s", pixel_format.c_str());
            return false;
        }
#endif
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
