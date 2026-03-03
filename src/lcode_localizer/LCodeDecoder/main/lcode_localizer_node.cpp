#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <opencv2/highgui.hpp>

// 프로젝트 구조에 맞춘 헤더 경로
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

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        
        img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/downward_camera/image_raw", rclcpp::SensorDataQoS(), 
            std::bind(&LCodeLocalizerNode::image_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "L-Code Localizer Node Initialized (Clean-slate mode).");
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv::Mat src;
        try {
            src = cv_bridge::toCvCopy(msg, "bgr8")->image;
        } catch (cv_bridge::Exception& e) {
            return;
        }

        // [핵심 수정] 매 콜백마다 새로운 전처리기 객체 생성 (내부 변수 초기화 효과)
        auto local_preprocessor = std::make_unique<LCODE::ImgPreprocessor>(*config_);

        // 이미지 메시지의 헤더 시간을 가져옴 (Extrapolation Error 방지 핵심)
        rclcpp::Time stamp = msg->header.stamp;

        if (local_preprocessor->run(src)) {
            if (digitizer_->runDigitize(*local_preprocessor)) {
                auto coords = ba_->getPos(digitizer_->digitizied_array, digitizer_->movementIndex);

                std::cout << "x: " << coords.first << ", y: " << coords.second << "\n";
                
                if (coords.first != -1 && coords.second != -1) {
                    last_x = (coords.first - 275.0) * 0.002;
                    last_y = (275.0 - coords.second) * 0.002;
                    publish_all(last_x, last_y, last_th, stamp);
                    initial_pos_sent = true;
                }
            }
        }

        // 초기화되지 않았을 때만 0,0 발행 (최초 1회)
        if (!initial_pos_sent) {
            publish_all(0.0, 0.0, 0.0, stamp);
            initial_pos_sent = true; 
        }
    }

    void publish_all(double x, double y, double th, const rclcpp::Time & stamp) {
        // 1. TF 발행 (map -> odom)
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = stamp; // 이미지의 시간을 그대로 사용하여 동기화
        t.header.frame_id = "map";
        t.child_frame_id = "odom";
        t.transform.translation.x = x;
        t.transform.translation.y = y;
        t.transform.translation.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, th);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();
        tf_broadcaster_->sendTransform(t);

        // 2. Odometry 발행
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = stamp;
        odom.header.frame_id = "map";
        odom.child_frame_id = "base_link";
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.orientation = t.transform.rotation;
        odom_pub_->publish(odom);
    }

    // 멤버 변수 (preprocessor_ 제거됨)
    std::shared_ptr<LCODE::ConfigSettings> config_;
    std::unique_ptr<LCODE::BGArray> ba_;
    std::unique_ptr<LCODE::Digitizer> digitizer_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    double last_x = 0.0, last_y = 0.0, last_th = 0.0;
    bool initial_pos_sent = false;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LCodeLocalizerNode>());
    rclcpp::shutdown();
    return 0;
}
