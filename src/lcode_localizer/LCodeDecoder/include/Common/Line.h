#pragma once
#include <cmath> // std::atan2, std::abs
#include <opencv2/core.hpp>

namespace LCODE {
template <typename T>
struct Line {
  cv::Point_<T> pt1, pt2;

  Line() noexcept : pt1(), pt2() {}
  Line(const cv::Point_<T> &point1, const cv::Point_<T> &point2) noexcept : pt1(point1), pt2(point2) {}
  Line(T x1, T y1, T x2, T y2) noexcept : pt1(x1, y1), pt2(x2, y2) {}

  auto operator=(const Line &) -> Line & = default;

  /**
   * @brief 현재 선분의 기울기(라디안)를 계산합니다.
   * atan2를 사용하므로 -PI ~ PI 사이의 값을 반환합니다.
   */
  auto calculateSlope() const -> double {
    double deltaX = static_cast<double>(pt2.x) - static_cast<double>(pt1.x);
    double deltaY = static_cast<double>(pt2.y) - static_cast<double>(pt1.y);
    return std::abs(std::atan2(deltaY, deltaX));
  }

  /**
   * @brief (옵션) 도(Degree) 단위가 필요할 경우
   */
  auto calculateSlopeDegrees() const -> double {
    return calculateSlope() * 180.0 / CV_PI;
  }
};
} // namespace LCODE