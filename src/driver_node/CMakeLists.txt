cmake_minimum_required(VERSION 3.8)
project(driver_node)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 컴파일 옵션 설정
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3) # 실시간 제어를 위해 최적화 옵션 추가
endif()

# 1. 의존성 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# 2. 실행 파일 생성
# src 폴더 내의 소스 파일들을 합쳐서 driver_node 실행 파일을 만듭니다.
add_executable(driver_node 
    src/DriverNode.cpp 
    src/KinematicsInterface.cpp
)

# 3. 헤더 파일 포함 경로 지정
# include 폴더를 컴파일러가 찾을 수 있도록 설정합니다.
target_include_directories(driver_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# 4. 의존성 라이브러리 연결
# rclcpp와 메시지 패키지들을 타겟에 연결합니다.
ament_target_dependencies(driver_node
  rclcpp
  geometry_msgs
  std_msgs
)

# 5. 설치 규칙 (Deployment)
# 빌드된 바이너리를 설치 폴더로 이동시켜 ros2 run 명령어가 찾을 수 있게 합니다.
install(TARGETS
  driver_node
  DESTINATION lib/${PROJECT_NAME}
)

# include 폴더 전체를 설치 (타 패키지 참조용)
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# 6. 테스트 및 린트 설정 (기존 유지)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 7. 패키지 구성 완료
ament_package()
