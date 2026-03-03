#ifndef KINEMATICS_INTERFACE_HPP
#define KINEMATICS_INTERFACE_HPP

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fcntl.h>
#include <iostream>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <thread>
#include <unistd.h>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

/**
 * @class KinematicsInterface
 * @brief I2C 통신을 통해 4륜 메카넘/옴니 로봇을 제어하는 클래스
 */
class KinematicsInterface {
private:
  int fd_;
  std::string i2c_device_ = "/dev/i2c-1";
  const int i2c_addr_ = 0x2B;

  bool is_simulation_ = false; // 시뮬레이션 모드 플래그

  // 로봇 물리 파라미터 (단위: m)
  const double L_ = 0.15;
  const double W_ = 0.15;
  const double K_ = L_ + W_; // 회전 계수

  // 속도 변환 계수 (m/s -> PWM)
  const double SPEED_SCALE_ = 40.0; // 재조정 필요시 실험적으로 조정

  // 옴니바퀴 미끄러짐 보정용 마지막 명령 저장
  double last_vx_ = 0.0;
  double last_vy_ = 0.0;
  double last_wz_ = 0.0;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr sim_pub_;

public:
  /**
   * @brief I2C 장치 초기화 및 연결 확인
   */
  KinematicsInterface(const std::string &device = "/dev/i2c-1",
                      uint8_t addr = 0x2B);

  KinematicsInterface(rclcpp::Node::SharedPtr node, 
                      const std::string &device = "/dev/i2c-1",
                      uint8_t addr = 0x2B);
                      
  /**
   * @brief 객체 소멸 시 로봇 정지 및 I2C 닫기
   */
  ~KinematicsInterface();

  /**
   * @brief 개별 모터 제어 (Low-level)
   * @param id 모터 ID (0~3)
   * @param speed 모터 속도 (-255 FF~ 255)
   */
  void set_motor(int id, int speed);

  /**
   * @brief Nav2/ROS 2 표준 속도 명령 인터페이스 (High-level)
   * @param vx 전진 선속도 (m/s)
   * @param vy 측면 선속도 (m/s)
   * @param wz 각속도 (rad/s)
   */
  void drive(double vx, double vy, double wz);

  /**
   * @brief 모든 구동 정지
   */
  void stop();

  /**
   * @brief I2C 통신 연결 상태 확인
   * @return 연결 성공 시 true
   */
  bool is_connected() const;
};

#endif // KINEMATICS_INTERFACE_HPP