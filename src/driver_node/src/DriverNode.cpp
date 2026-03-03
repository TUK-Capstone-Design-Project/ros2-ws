#include "DriverNode.hpp"

DriverNode::DriverNode() : Node("driver_node") {
  // 1. 하드웨어 인터페이스 초기화
  // 기본 장치 /dev/i2c-1, 주소 0x2B 사용
  robot_ = std::make_unique<KinematicsInterface>("/dev/i2c-1", 0x2B);

  if (!robot_->is_connected()) {
    RCLCPP_ERROR(this->get_logger(), "로봇 하드웨어 연결 실패! I2C 설정을 확인하세요.");
  }

  // 2. /cmd_vel 토픽 구독 (QoS 설정: 10)
  subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, std::bind(&DriverNode::cmd_vel_callback, this, std::placeholders::_1));

  // 3. 안전을 위한 타임아웃 타이머 (20Hz로 체크)
  last_command_time_ = this->get_clock()->now();
  timeout_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50), std::bind(&DriverNode::check_timeout, this));

  RCLCPP_INFO(this->get_logger(), "Driver Node가 시작되었습니다. /cmd_vel 대기 중...");
}

void DriverNode::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
  if (!robot_->is_connected()) return;

  // 마지막 명령 시간 업데이트
  last_command_time_ = this->get_clock()->now();

  // Twist -> KinematicsInterface 호출
  // 메카넘 휠이므로 vx(전진), vy(측면), wz(회전) 모두 전달
  robot_->drive(msg->linear.x, msg->linear.y, msg->angular.z);
  
  // 디버깅 로그 (필요 시 주석 해제)
  // RCLCPP_DEBUG(this->get_logger(), "Drive - vx: %.2f, vy: %.2f, wz: %.2f", 
  //              msg->linear.x, msg->linear.y, msg->angular.z);
}

void DriverNode::check_timeout() {
  auto now = this->get_clock()->now();
  auto duration = (now - last_command_time_).seconds();

  // 설정된 시간 동안 메시지가 오지 않으면 로봇을 정지시켜 폭주 방지
  if (duration > TIMEOUT_THRESHOLD_) {
    robot_->stop();
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DriverNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
