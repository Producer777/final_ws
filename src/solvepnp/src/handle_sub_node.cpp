#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/video/tracking.hpp>  // 👈 KalmanFilter
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/float32.hpp>

struct ArmorPoints {
    cv::Point2f pt1; // 左上
    cv::Point2f pt2; // 左下
    cv::Point2f pt3; // 右下
    cv::Point2f pt4; // 右上
};

// 读取 CSV 文件
std::vector<ArmorPoints> readCSV(const std::string& filename) {
    std::vector<ArmorPoints> data;
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open CSV file");
    }

    std::string line;
    std::getline(file, line); // 先读掉表头
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string item;
        std::vector<float> nums;

        while (std::getline(ss, item, ',')) {
            nums.push_back(std::stof(item));
        }

        if (nums.size() != 8) continue;

        ArmorPoints pts;
        pts.pt1 = cv::Point2f(nums[0], nums[1]);
        pts.pt2 = cv::Point2f(nums[2], nums[3]);
        pts.pt3 = cv::Point2f(nums[4], nums[5]);
        pts.pt4 = cv::Point2f(nums[6], nums[7]);
        data.push_back(pts);
    }

    return data;
}

class CameraInfoSubscriber : public rclcpp::Node
{
public:
    CameraInfoSubscriber()
    : Node("camera_info_subscriber")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera/camera_info",   // 👈 和发布节点保持一致
            10,
            std::bind(&CameraInfoSubscriber::camera_info_callback, this, std::placeholders::_1));

        kf_pose_pub_ = this->create_publisher<geometry_msgs::msg::Point>("kf_position", 10);
        pose_pub_ = this->create_publisher<geometry_msgs::msg::Point>("obj_position", 10);
        kf_yaw_pub_  = this->create_publisher<std_msgs::msg::Float32>("kf_yaw", 10);
        yaw_pub_  = this->create_publisher<std_msgs::msg::Float32>("obj_yaw", 10);          
        pitch_pub_  = this->create_publisher<std_msgs::msg::Float32>("obj_pitch", 10);
        
        // 读取 CSV（只读一次）
        csv_points_ = readCSV("/home/ypq/文档/算法组——最终考核/data.csv");
        RCLCPP_INFO(this->get_logger(), "Loaded %zu frames from CSV", csv_points_.size());
        
        float w = 135; // 135mm 宽
        float h = 56; // 56mm 高
        objectPoints = {
            {-w/2,  h/2, 0}, // 左上
            {-w/2, -h/2, 0}, // 左下
            { w/2, -h/2, 0}, // 右下
            { w/2,  h/2, 0}  // 右上
        };

        // 初始化KF: 状态维度=8 (x,y,z,yaw,vx,vy,vz,vyaw)，测量维度=4 (x,y,z,yaw)
        kf_ = cv::KalmanFilter(8, 4, 0);

        // 状态转移矩阵 F
        kf_.transitionMatrix = (cv::Mat_<float>(8, 8) <<
            1,0,0,0, 1,0,0,0,
            0,1,0,0, 0,1,0,0,
            0,0,1,0, 0,0,1,0,
            0,0,0,1, 0,0,0,1,
            0,0,0,0, 1,0,0,0,
            0,0,0,0, 0,1,0,0,
            0,0,0,0, 0,0,1,0,
            0,0,0,0, 0,0,0,1);

        // 测量矩阵 H
        kf_.measurementMatrix = cv::Mat::zeros(4, 8, CV_32F);
        kf_.measurementMatrix.at<float>(0,0) = 1.0f; // x
        kf_.measurementMatrix.at<float>(1,1) = 1.0f; // y
        kf_.measurementMatrix.at<float>(2,2) = 1.0f; // z
        kf_.measurementMatrix.at<float>(3,3) = 1.0f; // yaw

        // 噪声协方差
        // 声明参数（构造函数里）
        this->declare_parameter<std::vector<double>>(
            "kf_process_noise",
            std::vector<double>{0.0001,0.0001,0.0001,0.0001,0.0001,0.0001,0.0001,0.0001});
        this->declare_parameter<std::vector<double>>(
            "kf_measurement_noise",
            std::vector<double>{0.01,0.01,0.01,0.01});

        // 获取参数
        std::vector<double> q_vals = this->get_parameter("kf_process_noise").as_double_array();
        std::vector<double> r_vals = this->get_parameter("kf_measurement_noise").as_double_array();

        // 设置过程噪声 Q
        kf_.processNoiseCov = cv::Mat::zeros(8, 8, CV_32F);
        for (int i = 0; i < std::min((int)q_vals.size(), 8); i++) {
            kf_.processNoiseCov.at<float>(i,i) = static_cast<float>(q_vals[i]);
        }

        // 设置测量噪声 R
        kf_.measurementNoiseCov = cv::Mat::zeros(4, 4, CV_32F);
        for (int i = 0; i < std::min((int)r_vals.size(), 4); i++) {
            kf_.measurementNoiseCov.at<float>(i,i) = static_cast<float>(r_vals[i]);
        }

        // 注册参数动态更新回调
        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&CameraInfoSubscriber::parametersCallback, this, std::placeholders::_1)
        );

        setIdentity(kf_.errorCovPost, cv::Scalar::all(1));
        kf_.errorCovPost.at<float>(4,4) = 1000.0f;
        kf_.errorCovPost.at<float>(5,5) = 1000.0f;
        kf_.errorCovPost.at<float>(6,6) = 1000.0f;
        kf_.errorCovPost.at<float>(7,7) = 1000.0f;

        this->declare_parameter<int>("time",200);
        int t=this->get_parameter("time").as_int(); 
        //定时器
        timer_ = this->create_wall_timer(
        std::chrono::milliseconds(t),
        std::bind(&CameraInfoSubscriber::process_frame, this));
    }

private:
    cv::KalmanFilter kf_;
    bool kf_initialized =false;
    std::vector<cv::Point3f> objectPoints;
    cv::Mat K;
    cv::Mat distCoeffs;
    size_t current_frame_index_ = 0;
    rclcpp::TimerBase::SharedPtr timer_;
    void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        // 相机内参矩阵 K
        K = (cv::Mat_<double>(3, 3) <<
            msg->k[0], msg->k[1], msg->k[2],
            msg->k[3], msg->k[4], msg->k[5],
            msg->k[6], msg->k[7], msg->k[8]);

        // 畸变系数
        distCoeffs = cv::Mat(msg->d.size(), 1, CV_64F);
        for (size_t i = 0; i < msg->d.size(); i++) {
            distCoeffs.at<double>(i,0) = msg->d[i];
        }

        RCLCPP_INFO(this->get_logger(),
            "Received CameraInfo: width=%d, height=%d, fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f",
            msg->width, msg->height,
            K.at<double>(0,0), K.at<double>(1,1),
            K.at<double>(0,2), K.at<double>(1,2));
    };
    void process_frame()
    {
        if (K.empty() || distCoeffs.empty()) {
        RCLCPP_WARN(this->get_logger(), "CameraInfo not received yet!");
        return;
        }
        if (current_frame_index_ >= csv_points_.size()) return;

        const auto& pts = csv_points_[current_frame_index_++];
        std::vector<cv::Point2f> imagePoints = { pts.pt1, pts.pt2, pts.pt3, pts.pt4 };

        cv::Mat rvec, tvec;
        bool success = cv::solvePnP(objectPoints, imagePoints, K, distCoeffs, rvec, tvec);

        if (success) {
            cv::Mat R;
            cv::Rodrigues(rvec, R);

            double sy = std::sqrt(R.at<double>(0,0) * R.at<double>(0,0) + R.at<double>(1,0) * R.at<double>(1,0));
            bool singular = sy < 1e-6;
            double x, y, z; // pitch, yaw, roll
            if (!singular) {
                x = std::atan2(R.at<double>(2,1), R.at<double>(2,2));
                y = std::atan2(-R.at<double>(2,0), sy);
                z = std::atan2(R.at<double>(1,0), R.at<double>(0,0));
            } else {
                x = std::atan2(-R.at<double>(1,2), R.at<double>(1,1));
                y = std::atan2(-R.at<double>(2,0), sy);
                z = 0;
            }
                
            // 输出位姿
            RCLCPP_INFO(this->get_logger(),
                "Frame %zu: tvec=[%.3f, %.3f, %.3f] mm, pitch=%.2f°, yaw=%.2f°, roll=%.2f°",
                current_frame_index_ ,tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2),
                x*180.0/CV_PI, y*180.0/CV_PI, z*180.0/CV_PI);

            // 初始状态
            if(!kf_initialized)
            {
                kf_.statePost.at<float>(0) = static_cast<float>(tvec.at<double>(0));
                kf_.statePost.at<float>(1) = static_cast<float>(tvec.at<double>(1));
                kf_.statePost.at<float>(2) = static_cast<float>(tvec.at<double>(2));
                kf_.statePost.at<float>(3) = static_cast<float>(y); // yaw


                kf_.statePost.at<float>(4) = 0.0f; // vx
                kf_.statePost.at<float>(5) = 0.0f; // vy
                kf_.statePost.at<float>(6) = 0.0f; // vz
                kf_.statePost.at<float>(7) = 0.0f; // vyaw
                kf_initialized=true;
            }
            
            // KF 处理 
            cv::Mat prediction = kf_.predict();  // 预测一步

            cv::Mat measurement = (cv::Mat_<float>(4,1) << 
            static_cast<float>(tvec.at<double>(0)),
            static_cast<float>(tvec.at<double>(1)),
            static_cast<float>(tvec.at<double>(2)),
            static_cast<float>(y)
            );

            cv::Mat estimated = kf_.correct(measurement); // 校正

            float est_x = estimated.at<float>(0);
            float est_y = estimated.at<float>(1);
            float est_z = estimated.at<float>(2);
            float est_yaw = estimated.at<float>(3);
                                        
            RCLCPP_INFO(this->get_logger(),
                "Raw: x=%.2f,y=%.2f,z=%.2f,yaw=%.2f | KF: x=%.2f,y=%.2f,z=%.2f,yaw=%.2f",
                tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2), y*180.0/CV_PI, est_x,est_y,est_z,est_yaw*180.0/CV_PI);
            
            geometry_msgs::msg::Point kf_p_msg;
            kf_p_msg.x = est_x;
            kf_p_msg.y = est_y;
            kf_p_msg.z = est_z;
            kf_pose_pub_->publish(kf_p_msg);

            std_msgs::msg::Float32 kf_yaw_msg;
            kf_yaw_msg.data = est_yaw * 180.0 / CV_PI;
            kf_yaw_pub_->publish(kf_yaw_msg);

            geometry_msgs::msg::Point p_msg;
            p_msg.x = tvec.at<double>(0);
            p_msg.y = tvec.at<double>(1);
            p_msg.z = tvec.at<double>(2);
            pose_pub_->publish(p_msg);

            std_msgs::msg::Float32 yaw_msg;
            yaw_msg.data = y*180.0/CV_PI;
            yaw_pub_->publish(yaw_msg);

            std_msgs::msg::Float32 pitch_msg;
            pitch_msg.data = x*180.0/CV_PI;
            pitch_pub_->publish(pitch_msg);
        };
    };

    // 参数更新回调
    rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter> &params)
    {
        for (const auto &param : params) {
            if (param.get_name() == "kf_process_noise") {
                auto q_vals = param.as_double_array();
                for (int i = 0; i < std::min((int)q_vals.size(), 8); i++) {
                    kf_.processNoiseCov.at<float>(i,i) = static_cast<float>(q_vals[i]);
                }
                RCLCPP_INFO(this->get_logger(), "Updated processNoiseCov (Q)");
            }
            else if (param.get_name() == "kf_measurement_noise") {
                auto r_vals = param.as_double_array();
                for (int i = 0; i < std::min((int)r_vals.size(), 4); i++) {
                    kf_.measurementNoiseCov.at<float>(i,i) = static_cast<float>(r_vals[i]);
                }
                RCLCPP_INFO(this->get_logger(), "Updated measurementNoiseCov (R)");
            }
        }

        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
    }

    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr kf_pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pose_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr kf_yaw_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pitch_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr yaw_pub_;
    std::vector<ArmorPoints> csv_points_;
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraInfoSubscriber>());
    rclcpp::shutdown();
    return 0;
}