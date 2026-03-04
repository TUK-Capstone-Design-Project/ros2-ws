#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <opencv2/highgui.hpp>

#include <tf2/utils.h> 
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include "ConfigSettings/ConfigSettings.hpp"
#include "Digitizer/Digitizer.hpp"
#include "DeBruijnSeq/GenBGArray.hpp"
#include "ImgPreprocessor/ImgPreprocessor.hpp"

class LCodeLocalizerNode : public rclcpp::Node {
public:
    LCodeLocalizerNode() : Node("lcode_localizer_node") {
        config_ = std::make_shared<LCODE::ConfigSettings>();
        ba_ = std::make_unique<LCODE::BGArray>(*config_);
        digitizer_ = std::make_unique<LCODE::Digitizer>(*config_);
        
        // 2. TF 리스너 초기화 (바퀴 위치를 읽어오기 위함)
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        // odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        
        img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/downward_camera/image_raw", rclcpp::SensorDataQoS(), 
            std::bind(&LCodeLocalizerNode::image_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "L-Code Localizer Node 시작됨 (GPU 가속 대응 완료)");
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    cv::Mat src;
    try {
        src = cv_bridge::toCvCopy(msg, "bgr8")->image;
    } catch (cv_bridge::Exception& e) { return; }
        auto preprocessor_ = std::make_unique<LCODE::ImgPreprocessor>(*config_);

    rclcpp::Time stamp = msg->header.stamp;

    if (preprocessor_->run(src)) {
        if (digitizer_->runDigitize(*preprocessor_)) {
            // 반환 형식이 std::tuple<double, double, double>인 경우 예시
            //auto [raw_x, raw_y, raw_angle] = ba_->getPos(digitizer_->digitizied_array, digitizer_->movementIndex);
            auto coords = ba_->getPos(digitizer_->digitizied_array, digitizer_->movementIndex);

            int raw_x = coords.first;
            int raw_y = coords.second;
            double raw_angle = preprocessor_->camera_up_angle_;

            if (raw_x != -1 && raw_y != -1) {
                // 1. 위치 계산 (해상도 2mm 가정)
                last_x = (raw_x - 275.0) * 0.002;
                last_y = (275.0 - raw_y) * 0.002;

                
                // 2. 방향(Heading) 계산 (L-Code에서 준 angle 반영);
                // last_th = raw_angle;
                // L-Code 기준 각도를 ROS 라디안 단위로 변환이 필요할 수 있습니다.
                // caemra_up_angle_ -> cv의 y,x좌표계 반영 ( 0도 부근에서 +, - 교차함)
                double raw_angle_rad = raw_angle * (M_PI / 180.0);
                double ros_angle_rad = -raw_angle_rad;                
                last_th = ros_angle_rad;

                initial_pos_found = true;
                std::cout<<"L-Code 해독 성공  x: "<<raw_x << " , y : "<<raw_y<<" angle : "<< ros_angle_rad<<"\n";
            }
        }
    }

    // [중요] 인식이 실패해도 마지막 위치를 계속 발행 (map -> odom 연결 유지)
    if (initial_pos_found) {
        // publish_all(last_x, last_y, last_th, stamp);
        publish_all(last_x, last_y, last_th, this->get_clock()->now());
    }
}

    void publish_all(double map_x, double map_y, double map_th, const rclcpp::Time & stamp) {
    geometry_msgs::msg::TransformStamped odom_to_base;
    try {
        odom_to_base = tf_buffer_->lookupTransform("odom", "base_link", tf2::TimePointZero);
    } catch (tf2::TransformException & ex) { return; }

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = stamp;
    t.header.frame_id = "map";
    t.child_frame_id = "odom";

    // 1. 위치 차이 계산
    t.transform.translation.x = map_x - odom_to_base.transform.translation.x;
    t.transform.translation.y = map_y - odom_to_base.transform.translation.y;
    t.transform.translation.z = 0.0;

    auto & q_msg = odom_to_base.transform.rotation;
    double siny_cosp = 2 * (q_msg.w * q_msg.z + q_msg.x * q_msg.y);
    double cosy_cosp = 1 - 2 * (q_msg.y * q_msg.y + q_msg.z * q_msg.z);
    double odom_yaw = std::atan2(siny_cosp, cosy_cosp);

    double delta_th = map_th - odom_yaw;

    // 3. [수정] tf2::Quaternion의 setRPY는 그대로 사용하되, 값을 하나씩 대입
    tf2::Quaternion q;
    q.setRPY(0, 0, delta_th); 
    
    // tf2 객체의 값을 메시지에 수동으로 할당 (링킹 에러 방지)
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(t);
}
    std::shared_ptr<LCODE::ConfigSettings> config_;
    std::unique_ptr<LCODE::BGArray> ba_;
    std::unique_ptr<LCODE::Digitizer> digitizer_;
    // std::unique_ptr<LCODE::ImgPreprocessor> preprocessor_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    double last_x = 0.0, last_y = 0.0, last_th = 0.0;
    bool initial_pos_found = false;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LCodeLocalizerNode>());
    rclcpp::shutdown();
    return 0;
}