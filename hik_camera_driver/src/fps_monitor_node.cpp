#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float64.hpp>
#include <chrono>
#include <atomic>

namespace hik_camera_driver
{

class FPSMonitorNode : public rclcpp::Node
{
public:
    FPSMonitorNode() : Node("fps_monitor")
    {
        // 声明参数
        this->declare_parameter("monitor_topic", "/image_raw");
        this->declare_parameter("update_interval", 1.0);
        this->declare_parameter("window_size", 10);
        
        // 获取参数
        monitor_topic_ = this->get_parameter("monitor_topic").as_string();
        update_interval_ = this->get_parameter("update_interval").as_double();
        window_size_ = this->get_parameter("window_size").as_int();
        
        // 创建订阅者
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            monitor_topic_,
            10,
            std::bind(&FPSMonitorNode::imageCallback, this, std::placeholders::_1)
        );
        
        // 创建实际帧率发布者
        fps_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            monitor_topic_ + "/actual_fps",
            10
        );
        
        // 创建定时器用于定期输出统计信息
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(update_interval_ * 1000)),
            std::bind(&FPSMonitorNode::printStats, this)
        );
        
        // 初始化统计变量
        frame_count_ = 0;
        last_frame_count_ = 0;
        start_time_ = this->now();
        last_print_time_ = this->now();
        
        RCLCPP_INFO(this->get_logger(), "帧率监控节点已启动");
        RCLCPP_INFO(this->get_logger(), "监控话题: %s", monitor_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "更新间隔: %.1f 秒", update_interval_);
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        frame_count_++;
        
        // 记录帧时间戳用于计算实际帧率
        auto current_time = this->now();
        frame_times_.push_back(current_time);
        
        // 保持窗口大小
        if (frame_times_.size() > static_cast<size_t>(window_size_)) {
            frame_times_.erase(frame_times_.begin());
        }
    }
    
    void printStats()
    {
        auto current_time = this->now();
        auto elapsed_time = (current_time - start_time_).seconds();
        auto print_elapsed = (current_time - last_print_time_).seconds();
        
        // 计算平均帧率
        double avg_fps = 0.0;
        if (elapsed_time > 0) {
            avg_fps = frame_count_ / elapsed_time;
        }
        
        // 计算当前帧率（基于最近几帧）
        double current_fps = 0.0;
        if (frame_times_.size() >= 2) {
            auto time_diff = (frame_times_.back() - frame_times_.front()).seconds();
            if (time_diff > 0) {
                current_fps = (frame_times_.size() - 1) / time_diff;
            }
        }
        
        // 计算最近更新间隔内的帧率
        double recent_fps = 0.0;
        if (print_elapsed > 0) {
            recent_fps = (frame_count_ - last_frame_count_) / print_elapsed;
        }
        
        // 发布实际帧率
        std_msgs::msg::Float64 fps_msg;
        fps_msg.data = current_fps;
        fps_pub_->publish(fps_msg);
        
        // 输出精简统计
        RCLCPP_INFO(this->get_logger(), 
            "FPS topic=%s avg=%.2f cur=%.2f recent=%.2f", 
            monitor_topic_.c_str(), avg_fps, current_fps, recent_fps);
        
        // 更新统计变量
        last_frame_count_ = frame_count_;
        last_print_time_ = current_time;
    }
    
    // 参数
    std::string monitor_topic_;
    double update_interval_;
    int window_size_;
    
    // 订阅者和发布者
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr fps_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // 统计变量
    std::atomic<uint64_t> frame_count_;
    uint64_t last_frame_count_;
    rclcpp::Time start_time_;
    rclcpp::Time last_print_time_;
    std::vector<rclcpp::Time> frame_times_;
};

} // namespace hik_camera_driver

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<hik_camera_driver::FPSMonitorNode>();
    
    RCLCPP_INFO(node->get_logger(), "帧率监控节点已启动");
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}
