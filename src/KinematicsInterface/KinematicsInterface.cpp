#include "KinematicsInterface.hpp"

KinematicsInterface::KinematicsInterface(const std::string &device,
                                         uint8_t addr)
    : fd_(-1), i2c_device_(device), i2c_addr_(addr) {

  // 1. I2C 장치 파일 열기
  fd_ = open(i2c_device_.c_str(), O_RDWR);
  if (fd_ < 0) {
    std::cerr << "[ERROR] I2C 장치 열기 실패: " << i2c_device_ << std::endl;
    return;
  }

  // 2. 슬레이브 주소 설정
  if (ioctl(fd_, I2C_SLAVE, i2c_addr_) < 0) {
    std::cerr << "[ERROR] I2C 슬레이브 주소 설정 실패: 0x" << std::hex
              << (int)i2c_addr_ << std::endl;
    close(fd_);
    fd_ = -1;
    return;
  }

  std::cout << "[SUCCESS] 로봇 인터페이스 연결 완료 (Addr: 0x" << std::hex
            << (int)i2c_addr_ << ")" << std::endl;
}

KinematicsInterface::~KinematicsInterface() {
  stop();
  if (fd_ >= 0) {
    close(fd_);
  }
}

bool KinematicsInterface::is_connected() const { return fd_ >= 0; }

void KinematicsInterface::stop() { drive(0.0, 0.0, 0.0); }

void KinematicsInterface::set_motor(int id, int speed) {
  if (fd_ < 0)
    return;

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

void KinematicsInterface::drive(double vx, double vy, double wz) {
  if (fd_ < 0)
    return;

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