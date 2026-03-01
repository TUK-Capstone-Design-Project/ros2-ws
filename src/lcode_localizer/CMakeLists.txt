cmake_minimum_required(VERSION 3.8)
project(lcode_localizer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)


# 2. 실행 파일 생성 (소스 파일 경로 일일이 명시)
add_executable(lcode_localizer_node
  LCodeDecoder/ConfigSettings/ConfigSettings.cpp
  LCodeDecoder/DeBruijnSeq/DeBruijnSeq.cpp
  LCodeDecoder/DeBruijnSeq/GenBGArray.cpp
  LCodeDecoder/Digitizer/Digitizer.cpp
  LCodeDecoder/DotDetector/DotDetector.cpp
  LCodeDecoder/ImgPreprocessor/ImgPreprocessor.cpp
  LCodeDecoder/LineProcessor/LineProcessor.cpp
)
# 3. 헤더 파일 포함 경로 (트리 구조 맞춤)
target_include_directories(lcode_localizer_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/LCodeDecoder/include>
  $<INSTALL_INTERFACE:include>
)

# 4. 의존성 연결
ament_target_dependencies(lcode_localizer_node
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  sensor_msgs
)


# 5. 설치 규칙
install(TARGETS lcode_localizer_node
  DESTINATION lib/${PROJECT_NAME}
)

# params 폴더 설치 (yaml, pgm 파일 등)
install(DIRECTORY params
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

install(DIRECTORY
  params
  DESTINATION share/${PROJECT_NAME}
)
