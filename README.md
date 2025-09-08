## 最终考核说明
# 思路
总共创建三个包，一个包内写一个话题发布相机参数，第二个包订阅此话题并利用solvepnp解算位置位姿，然后利用kf进行估计预测，画图均利用foxglove实现，第三个包用于存放launch文件
# 代码
**相机参数：**
形如：
```cpp
this->declare_parameter<int>("image_width", 1280);
msg_.width  = this->get_parameter("image_width").as_int();
```
声明并获取参数
参数文件存放在config下，形如：
```yaml
/**:
   ros__parameters:
    image_width: 1280
```
**解算：**
```cpp
struct ArmorPoints {
    cv::Point2f pt1; // 左上
    cv::Point2f pt2; // 左下
    cv::Point2f pt3; // 右下
    cv::Point2f pt4; // 右上
};
ArmorPoints pts;
pts.pt1 = cv::Point2f(nums[0], nums[1]);
pts.pt2 = cv::Point2f(nums[2], nums[3]);
pts.pt3 = cv::Point2f(nums[4], nums[5]);
pts.pt4 = cv::Point2f(nums[6], nums[7]);
data.push_back(pts);
```
先定义csv中2D坐标，然后读取csv文件，存储2D和自身坐标系坐标：
```cpp
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
        std::vector<cv::Point3f> objectPoints;
```
订阅其他参数：
```cpp
    cv::Mat K;
    cv::Mat distCoeffs;
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
```
利用以上信息进行solvepnp，获取rvec和tvec:
```cpp
cv::solvePnP(objectPoints, imagePoints, K, distCoeffs, rvec, tvec);
```
获取旋转矩阵：
```cpp
cv::Mat R;
cv::Rodrigues(rvec, R);
```
然后获取欧拉角:
```cpp
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
```
**KF:**
利用kf进行预测估计,先初始化：
```cpp
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
    std::vector<double>{1e-4,1e-4,1e-4,1e-4,1e-4,1e-4,1e-4,1e-4});
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

setIdentity(kf_.errorCovPost, cv::Scalar::all(1));
kf_.errorCovPost.at<float>(4,4) = 1000.0f;
kf_.errorCovPost.at<float>(5,5) = 1000.0f;
kf_.errorCovPost.at<float>(6,6) = 1000.0f;
kf_.errorCovPost.at<float>(7,7) = 1000.0f;

cv::KalmanFilter kf_;
bool kf_initialized =false;
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
```
初始状态用第一帧数据作为x,y,z,yaw,假设速度都是0,由于速度不可信将速度协方差矩阵调到1000，之后进行预测、矫正，并保存新数据：
```cpp
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
```
最后发布处理后新数据，用于画图调参：
```cpp
kf_pose_pub_ = this->create_publisher<geometry_msgs::msg::Point>("kf_position", 10);
pose_pub_ = this->create_publisher<geometry_msgs::msg::Point>("obj_position", 10);
kf_yaw_pub_  = this->create_publisher<std_msgs::msg::Float32>("kf_yaw", 10);
yaw_pub_  = this->create_publisher<std_msgs::msg::Float32>("obj_yaw", 10);          
pitch_pub_  = this->create_publisher<std_msgs::msg::Float32>("obj_pitch", 10);

rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr kf_pose_pub_;
rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pose_pub_;
rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr kf_yaw_pub_;
rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pitch_pub_;
rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr yaw_pub_;

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
```
目前参数设置：
```py
parameters=[{'kf_process_noise': [1e-4,1e-4,1e-4,1e-4,1e-4,1e-4,1e-4,1e-4],#Q
                'kf_measurement_noise': [0.01,0.01,0.01,0.01]}]#R
```
# 数学原理
**求解PNP：**
$$
s \begin{bmatrix} u \\ v \\ 1 \end{bmatrix} = 
K \begin{bmatrix} R & t \end{bmatrix} 
\begin{bmatrix} X \\ Y \\ Z \\ 1 \end{bmatrix}
$$
$(X,Y,Z)$ : 物体坐标系下的点
$(u,v)$ : 图像坐标
$R$ : 3x3 旋转矩阵
$t$ : 平移向量
$s$ : 缩放因子（深度相关）

**欧拉角转换：**
假设旋转矩阵为：
$$
R =
\begin{bmatrix}
r_{00} & r_{01} & r_{02} \\
r_{10} & r_{11} & r_{12} \\
r_{20} & r_{21} & r_{22}
\end{bmatrix}
$$
欧拉角 (roll 𝜙,pitch 𝜃,yaw 𝜓)的转换公式为：
$$
\theta = \arctan2\Big(-r_{20}, \, r_{00}^2 + r_{10}^2 \Big)
$$
\[
\phi = \arctan2(r_{21}, r_{22})
\]
\[
\psi = \arctan2(r_{10}, r_{00})
\]
如果出现奇异情况,即$\sqrt{r_{00}^2 + r_{10}^2} \approx 0$，那么：
\[
\theta = \pm \frac{\pi}{2}, \quad \phi = 0, \quad \psi = \operatorname{atan2}(-r_{01}, r_{11})
\]
**卡尔曼滤波：**
状态外插方程:
$$\hat{x}_{n+1,n}=F\hat{x}_{n,n}+Gu_n+w_n$$
式中：
$\hat{x}_{n+1,n}$ 是 $n$ 时刻对 $n+1$ 时刻系统状态的预测
$\hat{x}_{n,n}$ 是 $n$ 时刻系统状态向量的估计
$u_n$ 是 **控制向量** 或 **输入向量** - 该系统的一个 可测量的（确定性的）输入
$w_n$是 **过程噪声** 或 扰动 - 能够影响系统状态的 不可测量的 输入
$F$ 是 **状态转移矩阵**
$G$ 是 **控制矩阵** 或 **输入转移矩阵** （将控制量映射到状态变量上）

协方差外插方程：
$$P_{n+1,n}=FP_{n,n}F^T+Q$$
其中：
$P_{n,n}$ 是当前状态估计的不确定性的平方（协方差矩阵）
$P_{n+1,n}$ 是下一个状态预测的不确定性的平方（协方差矩阵）
$F$ 是状态转移矩阵
$Q$ 是过程噪声矩阵

测量方程:
$$z_n=Hx_n+v_n$$
其中：
$z_n$ 是测量向量
$x_n$ 是真实系统状态量（隐藏状态）
$v_n$ 是随机噪声向量
$H$ 是**观测矩阵**

状态更新方程:
$$\hat{x}_{n,n}=\hat{x}_{n,n−1}+K_n(z_n−H\hat{x}_{n,n−1})$$
式中：
$\hat{x}_{n,n}$ 是 $n$ 时刻的系统状态估计
$\hat{x}_{n,n−1}$ 是 ${n−1}$ 时刻对 $n$ 时刻系统状态的预测
$K_n$ 是卡尔曼增益
$z_n$ 是测量值
$H$ 是观测矩阵

协方差更新方程:
$$P_{n,n}=(I−K_nH)P_{n,n−1}(I−K_nH)^T+K_nR_nK^T_n$$
式中：
$P_{n,n}$ 是当前状态估计的协方差矩阵
$P_{n,n−1}$ 是前一时刻对当前状态的预测的协方差矩阵
$K_n$ 是卡尔曼增益
$H$ 是观测矩阵
$R_n$ 是测量噪声的协方差矩阵
$I$ 是单位阵（一个 $n×n$ 且仅有对角元素为1、其余元素均为0的方阵）

卡尔曼增益:
$$K_n=P_{n,n−1}H^T(HP_{n,n−1}H^T+R_n)^{−1}$$
式中：
$K_n$ 是卡尔曼增益
$P_{n,n−1}$ 是前一时刻对当前状态的预测的协方差矩阵
$H$ 是观测矩阵
$R_n$ 是测量噪声的协方差矩阵