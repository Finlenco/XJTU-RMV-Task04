#ifndef HIK_CAMERA_DRIVER_HPP
#define HIK_CAMERA_DRIVER_HPP

#include <memory>
#include <string>
#include <thread>
#include <atomic>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "image_transport/image_transport.hpp"
#include "mvs_sdk_wrapper.hpp"
#include <opencv2/core.hpp>

namespace hik_camera_driver
{

class HikCameraDriver : public rclcpp::Node
{
public:
    HikCameraDriver();
    ~HikCameraDriver();

private:
    // 延迟初始化方法（在构造函数完成后调用）
    void initializeAfterConstruction();
    
    // 初始化相机
    bool initializeCamera();
    
    // 释放相机资源
    void releaseCamera();
    
    // 相机采集线程
    void cameraCaptureThread();
    
    // 参数回调函数
    void onParameterChange(const std::vector<rclcpp::Parameter> & parameters);
    
    // 设置相机参数
    bool setExposureTime(double exposure_time);
    bool setGain(double gain);
    bool setFrameRate(double frame_rate);
    bool setPixelFormat(const std::string & pixel_format);
    
    // 获取相机参数
    double getExposureTime();
    double getGain();
    double getFrameRate();
    std::string getPixelFormat();
    
    // 相机重连
    void reconnectCamera();
    
    // 发布图像
    void publishImage(const cv::Mat & image, const rclcpp::Time & timestamp);

private:
    // ROS2相关
    image_transport::Publisher image_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr reconnect_timer_;
    rclcpp::TimerBase::SharedPtr init_timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
    
    // 参数
    std::string camera_ip_;
    std::string camera_serial_;
    std::string topic_name_;
    double frame_rate_;
    double exposure_time_;
    double gain_;
    std::string pixel_format_;
    bool auto_reconnect_;
    int reconnect_interval_;
    // 测试图像功能已移除
    
    // 相机相关
    std::unique_ptr<MVSSDKWrapper> sdk_wrapper_;
    std::atomic<bool> camera_connected_;
    std::atomic<bool> should_stop_;
    std::thread capture_thread_;
    std::mutex camera_mutex_;
    
    // 图像相关
    cv::Mat current_image_;
    std::mutex image_mutex_;
    
    // 统计信息
    std::atomic<uint64_t> frame_count_;
    rclcpp::Time last_frame_time_;

#ifdef HAVE_MVS_SDK
    // MVS SDK 相关
    void* camera_handle_ = nullptr;
#endif
};

} // namespace hik_camera_driver

#endif // HIK_CAMERA_DRIVER_HPP
