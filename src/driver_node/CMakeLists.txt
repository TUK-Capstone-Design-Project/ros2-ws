cmake_minimum_required(VERSION 3.8)
project(driver_node)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 컴파일 옵션 설정
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# 1. 의존성 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Threads REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# 2. 실행 파일 생성
# [중요] src/main.cpp를 추가하여 'undefined reference to main' 에러를 해결합니다.
add_executable(driver_node 
  src/main.cpp
  src/DriverNode.cpp 
  src/KinematicsInterface.cpp
)

# 3. 헤더 파일 포함 경로 지정
# generator expression을 위해 PRIVATE 키워드를 사용합니다.
target_include_directories(driver_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 4. 의존성 라이브러리 연결
# ament_target_dependencies와의 충돌 방지를 위해 키워드 없이(Plain) 연결합니다.
target_link_libraries(driver_node
  Threads::Threads
)

# 5. ROS 2 패키지 의존성 연결
ament_target_dependencies(driver_node
  rclcpp
  geometry_msgs
  std_msgs
  nav_msgs
  tf2_ros
)

# 6. 설치 규칙
install(TARGETS
  driver_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# 7. 테스트 및 린트 설정
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 8. 패키지 구성 완료
ament_package()