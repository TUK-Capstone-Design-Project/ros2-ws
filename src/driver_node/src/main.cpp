#include "rclcpp/rclcpp.hpp"
#include "DriverNode.hpp"  // DriverNode 클래스가 정의된 헤더

int main(int argc, char** argv) {
    // 1. ROS 2 초기화
    rclcpp::init(argc, argv);
    
    // 2. 노드 생성 (클래스 이름이 DriverNode라고 가정)
    auto node = std::make_shared<DriverNode>();
    
    // 3. 노드 실행 (종료될 때까지 대기)
    rclcpp::spin(node);
    
    // 4. 종료
    rclcpp::shutdown();
    return 0;
}