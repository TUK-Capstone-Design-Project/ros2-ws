#ifndef DRIVER_NODE_HPP
#define DRIVER_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "KinematicsInterface.hpp"

/**
 * @class DriverNode
 * @brief ROS 2 Twist 메시지를 받아 KinematicsInterface를 통해 로봇을 제어하는 노드
 */
class DriverNode : public rclcpp::Node {
public:
  DriverNode();
  virtual ~DriverNode() = default;

private:
  /**
   * @brief /cmd_vel 토픽 콜백 함수
   * @param msg 수신된 Twist 메시지 (선속도, 각속도)
   */
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

  /**
   * @brief 통신 타임아웃 체크 (명령이 끊기면 로봇 정지)
   */
  void check_timeout();

  // ROS 2 통신 객체
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timeout_timer_;

  // 하드웨어 인터페이스
  std::unique_ptr<KinematicsInterface> robot_;

  // 상태 변수
  rclcpp::Time last_command_time_;
  const double TIMEOUT_THRESHOLD_ = 0.5; // 0.5초 동안 명령 없으면 정지
};

#endif // DRIVER_NODE_HPP
