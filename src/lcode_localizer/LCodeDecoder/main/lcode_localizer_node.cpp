#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <opencv2/highgui.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_ros/transform_listener.hpp>

#include "ConfigSettings/ConfigSettings.hpp"
#include "DeBruijnSeq/GenBGArray.hpp"
#include "Digitizer/Digitizer.hpp"
#include "ImgPreprocessor/ImgPreprocessor.hpp"

class LCodeLocalizerNode : public rclcpp::Node
{
public:
    LCodeLocalizerNode() : Node("lcode_localizer_node")
    {
        config_    = std::make_shared<LCODE::ConfigSettings>();
        ba_        = std::make_unique<LCODE::BGArray>(*config_);
        digitizer_ = std::make_unique<LCODE::Digitizer>(*config_);

        // 2. TF 리스너 초기화 (바퀴 위치를 읽어오기 위함)
        tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        // odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

        img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/downward_camera/image_raw", rclcpp::SensorDataQoS(),
            std::bind(&LCodeLocalizerNode::image_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "L-Code Localizer Node 시작됨 (GPU 가속 대응 완료)");
    }

private:
    std::shared_ptr<LCODE::ConfigSettings> config_;
    std::unique_ptr<LCODE::BGArray>        ba_;
    std::unique_ptr<LCODE::Digitizer>      digitizer_;
    // std::unique_ptr<LCODE::ImgPreprocessor> preprocessor_;

    std::unique_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr    odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster>           tf_broadcaster_;

    double last_x = 0.0, last_y = 0.0, last_th = 0.0;
    bool   initial_pos_found = false;

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv::Mat src;
        try {
            src = cv_bridge::toCvCopy(msg, "bgr8")->image;
        } catch (cv_bridge::Exception &e) {
            return;
        }
        auto preprocessor_ = std::make_unique<LCODE::ImgPreprocessor>(*config_);

        rclcpp::Time stamp = msg->header.stamp;

        if (preprocessor_->run(src)) {
            if (digitizer_->runDigitize(*preprocessor_)) {
                // 반환 형식이 std::tuple<double, double, double>인 경우 예시
                // auto [raw_x, raw_y, raw_angle] = ba_->getPos(digitizer_->digitizied_array, digitizer_->movementIndex);
                auto coords = ba_->getPos(digitizer_->digitizied_array, digitizer_->movementIndex);

                int    raw_x     = coords.first;
                int    raw_y     = coords.second;
                double raw_angle = preprocessor_->camera_up_angle_;

                if (raw_x != -1 && raw_y != -1) {
                    // [수정] L-Code 좌표 (550×550, 좌상단 원점) -> ROS 좌표 (우상향 X, 상향 Y)
                    // 1. 중심 이동: (275, 275) -> (0, 0)
                    double centered_x = raw_x - 275.0;
                    double centered_y = raw_y - 275.0;

                    // 2. 좌표계 회전: L-Code(→X, ↓Y) -> ROS(→X, ↑Y)
                    // L-Code의 Y축이 아래쪽이므로 반전 필요
                    double ros_x = centered_x * 0.002;  // 우측이 +X
                    double ros_y = -centered_y * 0.002; // 상단이 +Y

                    // 2. 방향(Heading) 계산 (L-Code에서 준 angle 반영);
                    // last_th = raw_angle;
                    // L-Code 기준 각도를 ROS 라디안 단위로 변환이 필요할 수 있습니다.
                    // caemra_up_angle_ -> cv의 y,x좌표계 반영 ( 0도 부근에서 +, - 교차함)
                    double raw_angle_rad = raw_angle * (M_PI / 180.0);
                    double ros_angle_rad = -raw_angle_rad;
                    last_th              = ros_angle_rad;

                    initial_pos_found = true;
                    std::cout << "L-Code 해독 성공  x: " << raw_x << " , y : " << raw_y << " angle : " << ros_angle_rad << "\n";
                }
            }
        }

        // [중요] 인식이 실패해도 마지막 위치를 계속 발행 (map -> odom 연결 유지)
        if (initial_pos_found) {
            // publish_all(last_x, last_y, last_th, stamp);
            publish_all(last_x, last_y, last_th, this->get_clock()->now());
        }
    }

    void publish_all(double map_x, double map_y, double map_th, const rclcpp::Time &stamp)
    {
        // [중요] map -> odom 변환만 발행 (odom -> base_link는 Gazebo가 담당)
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp    = stamp;
        t.header.frame_id = "map";
        t.child_frame_id  = "odom"; // ✓ base_link 아님!

        t.transform.translation.x = map_x;
        t.transform.translation.y = map_y;
        t.transform.translation.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, map_th);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(t);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LCodeLocalizerNode>());
    rclcpp::shutdown();
    return 0;
}
