#include "DriverNode.hpp"
#include <tf2/LinearMath/Quaternion.h>

DriverNode::DriverNode() : Node("driver_node"), x_(0.0), y_(0.0), th_(0.0), vx_(0.0), vy_(0.0), wz_(0.0) {
  robot_ = std::make_unique<KinematicsInterface>("/dev/i2c-1", 0x2B);
  
  // 퍼블리셔 및 TF 브로드캐스터 초기화
  odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, std::bind(&DriverNode::cmd_vel_callback, this, std::placeholders::_1));

  // 오도메트리 업데이트 타이머 (50Hz)
  last_odom_time_ = this->get_clock()->now();
  odom_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(20), std::bind(&DriverNode::update_odometry, this));

  last_command_time_ = this->get_clock()->now();
  timeout_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50), std::bind(&DriverNode::check_timeout, this));
}

void DriverNode::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
  if (!robot_->is_connected()) return;
  last_command_time_ = this->get_clock()->now();

  // 명령된 속도 저장 (오도메트리 계산에 사용)
  vx_ = msg->linear.x;
  vy_ = msg->linear.y;
  wz_ = msg->angular.z;

  robot_->drive(vx_, vy_, wz_);
}

void DriverNode::update_odometry() {
  auto now = this->get_clock()->now();
  double dt = (now - last_odom_time_).seconds();
  last_odom_time_ = now;

  // 1. 추측 항법(Dead Reckoning) 계산
  // 로봇 좌표계 속도를 세계 좌표계(odom)로 변환하여 적분
  double delta_x = (vx_ * cos(th_) - vy_ * sin(th_)) * dt;
  double delta_y = (vx_ * sin(th_) + vy_ * cos(th_)) * dt;
  double delta_th = wz_ * dt;

  x_ += delta_x;
  y_ += delta_y;
  th_ += delta_th;

  // 2. TF 발행 (odom -> base_link)
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = now;
  t.header.frame_id = "map";
  t.child_frame_id = "odom";
  t.transform.translation.x = x_;
  t.transform.translation.y = y_;
  t.transform.translation.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, th_);
  t.transform.rotation.x = q.x();
  t.transform.rotation.y = q.y();
  t.transform.rotation.z = q.z();
  t.transform.rotation.w = q.w();
  tf_broadcaster_->sendTransform(t);

  // 3. Odometry 메시지 발행
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = now;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_link";
  odom.pose.pose.position.x = x_;
  odom.pose.pose.position.y = y_;
  odom.pose.pose.orientation = t.transform.rotation;
  // odom_publisher_->publish(odom);
}

void DriverNode::check_timeout() {
  if ((this->get_clock()->now() - last_command_time_).seconds() > TIMEOUT_THRESHOLD_) {
    vx_ = 0.0; vy_ = 0.0; wz_ = 0.0;
    robot_->stop();
  }
}