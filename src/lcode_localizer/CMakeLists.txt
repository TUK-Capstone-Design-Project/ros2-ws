cmake_minimum_required(VERSION 3.8)
project(lcode_localizer)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3) # 성능을 위해 -O3 추가
endif()

# 1. 의존성 패키지 찾아오기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)      # ROS-OpenCV 브릿지
find_package(OpenCV REQUIRED)         # OpenCV 라이브러리

# 2. 실행 파일 생성 및 소스 등록
# main/lcode_localizer_node.cpp 가 메인 파일임을 명시합니다.
add_executable(lcode_localizer_node
  LCodeDecoder/main/lcode_localizer_node.cpp
  LCodeDecoder/ConfigSettings/ConfigSettings.cpp
  LCodeDecoder/DeBruijnSeq/DeBruijnSeq.cpp
  LCodeDecoder/DeBruijnSeq/GenBGArray.cpp
  LCodeDecoder/Digitizer/Digitizer.cpp
  LCodeDecoder/DotDetector/DotDetector.cpp
  LCodeDecoder/ImgPreprocessor/ImgPreprocessor.cpp
  LCodeDecoder/LineProcessor/LineProcessor.cpp
)

# 3. 헤더 파일 포함 경로
# LCodeDecoder/include를 루트로 잡으면 내부에서 "Common/Line.h" 식으로 접근 가능합니다.
target_include_directories(lcode_localizer_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/LCodeDecoder/include>
  $<INSTALL_INTERFACE:include>
)

# 4. 의존성 및 라이브러리 연결
ament_target_dependencies(lcode_localizer_node
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  cv_bridge
)

# OpenCV는 ament_target_dependencies가 아닌 target_link_libraries로 연결하는 것이 안전합니다.
target_link_libraries(lcode_localizer_node
  ${OpenCV_LIBRARIES}
)

# 5. 설치 규칙
install(TARGETS lcode_localizer_node
  DESTINATION lib/${PROJECT_NAME}
)

# params 폴더 설치 (yaml, pgm 파일 등 중복 설치 제거)
install(DIRECTORY params
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch params DESTINATION share/${PROJECT_NAME})

# 6. 테스트 설정 (기존 유지)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
