#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace hik_camera_driver
{

class ParamInfoNode : public rclcpp::Node
{
public:
    ParamInfoNode() : Node("param_info")
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
        
        // 创建参数信息发布者
        param_info_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/camera_system/param_info",
            10
        );
        
        // 创建定时器定期发布参数信息
        timer_ = this->create_wall_timer(
            std::chrono::seconds(5),
            std::bind(&ParamInfoNode::publishParamInfo, this)
        );
        
        // 立即发布一次参数信息
        publishParamInfo();
        
        RCLCPP_INFO(this->get_logger(), "参数信息节点已启动");
    }

private:
    void publishParamInfo()
    {
        // 获取所有参数
        std::string camera_ip = this->get_parameter("camera_ip").as_string();
        std::string camera_serial = this->get_parameter("camera_serial").as_string();
        std::string topic_name = this->get_parameter("topic_name").as_string();
        double frame_rate = this->get_parameter("frame_rate").as_double();
        double exposure_time = this->get_parameter("exposure_time").as_double();
        double gain = this->get_parameter("gain").as_double();
        std::string pixel_format = this->get_parameter("pixel_format").as_string();
        bool auto_reconnect = this->get_parameter("auto_reconnect").as_bool();
        
        // 构建参数信息字符串
        std::string param_info = "📋 相机系统参数信息:\n";
        param_info += "================================\n";
        
        // 连接参数
        param_info += "🔗 连接参数:\n";
        if (!camera_ip.empty()) {
            param_info += "   相机IP: " + camera_ip + "\n";
        }
        if (!camera_serial.empty()) {
            param_info += "   相机序列号: " + camera_serial + "\n";
        }
        param_info += "   连接方式: " + std::string(camera_ip.empty() ? "序列号" : "IP地址") + "\n";
        param_info += "   自动重连: " + std::string(auto_reconnect ? "启用" : "禁用") + "\n";
        
        // 相机参数
        param_info += "\n⚙️ 相机参数:\n";
        param_info += "   设定帧率: " + std::to_string(frame_rate) + " FPS\n";
        param_info += "   曝光时间: " + std::to_string(exposure_time) + " μs\n";
        param_info += "   增益值: " + std::to_string(gain) + "\n";
        param_info += "   像素格式: " + pixel_format + "\n";
        
        // 图像参数
        param_info += "\n🖼️ 图像参数:\n";
        param_info += "   话题名称: " + topic_name + "\n";
        
        // 参数说明
        param_info += "\n💡 参数说明:\n";
        param_info += "   • 帧率: 相机采集频率，影响图像流畅度和带宽\n";
        param_info += "   • 曝光时间: 控制图像亮度，值越大图像越亮\n";
        param_info += "   • 增益: 信号放大倍数，提高亮度但增加噪声\n";
        param_info += "   • 像素格式: 图像颜色格式，影响图像质量和大小\n";
        param_info += "   • 自动重连: 网络断开时自动尝试重新连接\n";
        
        param_info += "\n🔧 修改参数命令:\n";
        param_info += "   ros2 param set /hik_camera_driver <参数名> <值>\n";
        param_info += "   例如: ros2 param set /hik_camera_driver frame_rate 60.0\n";
        
        // 发布参数信息
        std_msgs::msg::String msg;
        msg.data = param_info;
        param_info_pub_->publish(msg);
        
        // 输出到日志
        RCLCPP_INFO(this->get_logger(), "\n%s", param_info.c_str());
    }
    
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr param_info_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace hik_camera_driver

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<hik_camera_driver::ParamInfoNode>();
    
    RCLCPP_INFO(node->get_logger(), "参数信息节点已启动");
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}
