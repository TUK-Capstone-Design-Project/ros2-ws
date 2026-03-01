#include "KinematicsInterface.hpp"

/**
 * @brief 생성자: I2C 디바이스를 열고 슬레이브 주소를 설정합니다.
 */
KinematicsInterface::KinematicsInterface() : fd_(-1) {
    fd_ = open(i2c_device_, O_RDWR);
    if (fd_ < 0) {
        std::cerr << "[ERROR] I2C 장치를 열 수 없습니다. (/dev/i2c-1)" << std::endl;
        return;
    }

    if (ioctl(fd_, I2C_SLAVE, I2C_ADDR_) < 0) {
        std::cerr << "[ERROR] 슬레이브 주소(0x2B) 설정 실패." << std::endl;
        close(fd_);
        fd_ = -1;
        return;
    }

    std::cout << "[SUCCESS] KinematicsInterface 하드웨어 초기화 완료." << std::endl;
}

/**
 * @brief 소멸자: 안전을 위해 로봇을 멈추고 장치를 닫습니다.
 */
KinematicsInterface::~KinematicsInterface() {
    stop();
    if (fd_ >= 0) {
        close(fd_);
    }
}

/**
 * @brief I2C 통신 연결 상태 확인
 */
bool KinematicsInterface::is_connected() const {
    return fd_ >= 0;
}

/**
 * @brief 모든 모터를 정지 상태로 만듭니다.
 */
void KinematicsInterface::stop() {
    drive(0.0, 0.0, 0.0);
}

/**
 * @brief 개별 모터 제어 (4바이트 데이터 전송)
 */
void KinematicsInterface::set_motor(int id, int speed) {
    if (fd_ < 0) return;

    // 속도 범위 제한 (-255 ~ 255)
    int safe_speed = std::max(-255, std::min(255, speed));
    
    uint8_t dir = (safe_speed < 0) ? 1 : 0;
    uint8_t abs_speed = static_cast<uint8_t>(std::abs(safe_speed));
    uint8_t motor_id = static_cast<uint8_t>(id);

    // 야붐(Yahboom) 등의 표준 프로토콜: [Register, ID, Direction, Speed]
    uint8_t buffer[4] = {0x01, motor_id, dir, abs_speed};
    
    if (write(fd_, buffer, 4) != 4) {
        // 통신 노이즈 시 경고를 출력하려면 주석 해제
        // std::cerr << "[WARN] I2C Write Failed for Motor " << id << std::endl;
    }

    // RPi 5 및 I2C 버스 안정화를 위한 미세 지연
    std::this_thread::sleep_for(std::chrono::microseconds(500));
}

/**
 * @brief Nav2 표준 Twist 명령을 메카넘 휠 역기구학으로 변환
 */
void KinematicsInterface::drive(double vx, double vy, double wz) {
    if (fd_ < 0) return;

    // [메카넘 휠 역기구학 공식]
    // K = L + W (회전 계수)
    double fl = vx - vy - (K_ * wz);
    double fr = vx + vy + (K_ * wz);
    double rl = vx + vy - (K_ * wz);
    double rr = vx - vy + (K_ * wz);

    // 물리 속도(m/s)를 모터 드라이버 PWM 값으로 스케일링하여 전송
    set_motor(0, static_cast<int>(std::round(fl * SPEED_SCALE_)));
    set_motor(1, static_cast<int>(std::round(fr * SPEED_SCALE_)));
    set_motor(2, static_cast<int>(std::round(rl * SPEED_SCALE_)));
    set_motor(3, static_cast<int>(std::round(rr * SPEED_SCALE_)));
}