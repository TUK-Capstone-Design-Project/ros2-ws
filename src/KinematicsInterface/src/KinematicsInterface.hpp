#ifndef ROBOT_MANAGER_HPP
#define ROBOT_MANAGER_HPP

#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <thread>
#include <chrono>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <unistd.h>
#include <cstdint>

/**
 * @class KinematicsInterface
 * @brief I2C 통신을 통해 4륜 메카넘/옴니 로봇을 제어하는 클래스
 */
class KinematicsInterface {
private:
    int fd_;
    const char* i2c_device_ = "/dev/i2c-1";
    const int I2C_ADDR_ = 0x2B;

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

public:
    /**
     * @brief I2C 장치 초기화 및 연결 확인
     */
    RobotManager();

    /**
     * @brief 객체 소멸 시 로봇 정지 및 I2C 닫기
     */
    ~RobotManager();

    /**
     * @brief 개별 모터 제어 (Low-level)
     * @param id 모터 ID (0~3)
     * @param speed 모터 속도 (-255 ~ 255)
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

#endif // ROBOT_MANAGER_HPP