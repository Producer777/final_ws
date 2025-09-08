#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>

class CameraInfoPublisher : public rclcpp::Node
{
public:
    CameraInfoPublisher()
    : Node("camera_info_publisher")
    {
        // 声明参数（默认值可以写成 YAML 中的值）
        this->declare_parameter<int>("image_width", 1280);
        this->declare_parameter<int>("image_height", 1024);

        this->declare_parameter<std::vector<double>>("camera_matrix",
            {1307.16695,    0.     ,  648.51847,
            0.     , 1304.73248,  503.16415,
            0.     ,    0.     ,    1.     });

        this->declare_parameter<std::vector<double>>("distortion_coefficients",
            {-0.046674, 0.044424, -0.001226, 0.002452, 0.000000});

        this->declare_parameter<std::vector<double>>("rectification_matrix",
            {1.0, 0.0, 0.0,
             0.0, 1.0, 0.0,
             0.0, 0.0, 1.0});

        this->declare_parameter<std::vector<double>>("projection_matrix",
            {1295.59743,    0.     ,  651.02498,    0.     ,
            0.     , 1296.76514,  502.30498,    0.     ,
            0.     ,    0.     ,    1.     ,    0.     });

        // 获取参数值
        msg_.width  = this->get_parameter("image_width").as_int();
        msg_.height = this->get_parameter("image_height").as_int();

        auto k_vec = this->get_parameter("camera_matrix").as_double_array();
        std::copy(k_vec.begin(), k_vec.end(), msg_.k.begin());

        auto d_vec = this->get_parameter("distortion_coefficients").as_double_array();
        msg_.d = d_vec;

        auto r_vec = this->get_parameter("rectification_matrix").as_double_array();
        std::copy(r_vec.begin(), r_vec.end(), msg_.r.begin());

        auto p_vec = this->get_parameter("projection_matrix").as_double_array();
        std::copy(p_vec.begin(), p_vec.end(), msg_.p.begin());

        publisher_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
            "/camera/camera_info", 10);


        // 设置定时器，每秒发布一次
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&CameraInfoPublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        publisher_->publish(msg_);
        RCLCPP_INFO(this->get_logger(), "Publishing camera info");
    }

    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr publisher_;
    sensor_msgs::msg::CameraInfo msg_;
    rclcpp::TimerBase::SharedPtr timer_;
};
 
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraInfoPublisher>());
    rclcpp::shutdown();
    return 0;
}