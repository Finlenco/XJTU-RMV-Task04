#ifndef MVS_SDK_WRAPPER_HPP
#define MVS_SDK_WRAPPER_HPP

#include <string>
#include <vector>
#include <memory>
#include <functional>

// 海康威视MVS SDK头文件
#ifdef HAVE_MVS_SDK
#include "MvCameraControl.h"
#include "MvErrorDefine.h"
#include "CameraParams.h"
#endif

namespace hik_camera_driver
{

// 相机设备信息结构体
struct CameraDeviceInfo
{
    std::string device_name;
    std::string serial_number;
    std::string ip_address;
    std::string mac_address;
    std::string device_type;
    bool is_connected;
};

// 图像数据回调函数类型
using ImageCallback = std::function<void(const unsigned char*, int, int, int, const std::string&)>;

// 错误信息结构体
struct CameraError
{
    int error_code;
    std::string error_message;
};

class MVSSDKWrapper
{
public:
    MVSSDKWrapper();
    ~MVSSDKWrapper();

    // 初始化SDK
    bool initialize();
    
    // 释放SDK资源
    void cleanup();
    
    // 枚举设备
    std::vector<CameraDeviceInfo> enumerateDevices();
    
    // 连接设备
    bool connectDevice(const std::string& device_identifier, bool by_ip = true);
    
    // 断开设备
    void disconnectDevice();
    
    // 开始采集
    bool startGrabbing();
    
    // 停止采集
    void stopGrabbing();
    
    // 设置图像回调
    void setImageCallback(ImageCallback callback);
    
    // 相机参数设置
    bool setExposureTime(double exposure_time);
    bool setGain(double gain);
    bool setFrameRate(double frame_rate);
    bool setPixelFormat(const std::string& format);
    bool setImageSize(int width, int height);
    
    // 相机参数获取
    double getExposureTime();
    double getGain();
    double getFrameRate();
    std::string getPixelFormat();
    std::pair<int, int> getImageSize();
    
    // 获取设备信息
    CameraDeviceInfo getDeviceInfo();
    
    // 检查设备连接状态
    bool isDeviceConnected();
    
    // 获取错误信息
    CameraError getLastError();

private:
    // 内部实现
    class Impl;
    std::unique_ptr<Impl> pImpl_;
};

} // namespace hik_camera_driver

#endif // MVS_SDK_WRAPPER_HPP
