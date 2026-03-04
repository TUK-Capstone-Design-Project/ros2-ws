#include "ImgPreprocessor/ImgPreprocessor.hpp"
#include "Common/Line.h"
#include "ConfigSettings/ConfigSettings.hpp"
using namespace std;

// #define SHOW_VIDEO

namespace LCODE {

ImgPreprocessor::ImgPreprocessor(ConfigSettings &config) : lineProcessor_(config) {

  this->ROI_width_dots  = config.getROI_width_dot();  /// ROI 영역의 가로 점 갯수
  this->ROI_height_dots = config.getROI_height_dot(); /// ROI 영역의 세로 점 갯수
  // this->required_baseline_cnt = 계산 공식 - (ROI_width_dots - 1) / 6 + 1;
  this->required_baseline_cnt = 3;

  this->expansion_point_num = (ROI_height_dots - 1) / 6;

  int ROI_size = 1200; // ROI 영역 size

  this->ROI_width_size  = ROI_size * (static_cast<float>(ROI_width_dots) / (ROI_height_dots + ROI_width_dots));
  this->ROI_height_size = ROI_size * (static_cast<float>(ROI_height_dots) / (ROI_height_dots + ROI_width_dots));

  params_.minThreshold  = config.minThreshold;  // opencv default : 50
  params_.maxThreshold  = config.maxThreshold;  // opencv default : 220
  params_.thresholdStep = config.thresholdStep; // opencv default : 20

  params_.filterByArea        = config.filterByArea;
  params_.minArea             = config.minArea; // opencv default : 25
  params_.maxArea             = config.maxArea; // opencv default : 5000
  params_.filterByCircularity = config.filterByCircularity;
  params_.minCircularity      = config.min_circularity;
  params_.maxCircularity      = config.max_circularity;
  params_.filterByConvexity   = config.filterByConvexity;
  params_.minConvexity        = config.min_convexity;
  params_.maxConvexity        = config.max_convexity;

  params_.filterByInertia  = config.filterByInertia;
  params_.minInertiaRatio  = config.min_inertia;
  params_.maxInertiaRatio  = config.max_inertia;
  params_.minRepeatability = config.min_repeatability;
};

void ImgPreprocessor::drawDetectedDots(cv::Mat &img, vector<cv::KeyPoint> &keypoints) {
  if (img.type() == CV_8UC1) {
    cvtColor(img, img, COLOR_GRAY2BGR);
  }
  cv::drawKeypoints(img, keypoints, img, cv::Scalar(0, 255, 0));
#ifdef SHOW_VIDEO
  imshow("detected_dots", img);
#endif
};

void ImgPreprocessor::eraseDetectedDots(cv::Mat &img, vector<cv::KeyPoint> &dots) {
  for (const auto &dot : dots) {
    circle(img, dot.pt, static_cast<int>(dot.size + 0.5), {0}, -1);
  }
#ifdef SHOW_VIDEO
  imshow("after eraseDetectedDots, binarized", img);
#endif
  this->warp_source_orient = img.clone();
}

void ImgPreprocessor::binarizeImage() {
  constexpr int maxValue       = 255;
  constexpr int adaptiveMethod = cv::ADAPTIVE_THRESH_GAUSSIAN_C;
  constexpr int thresholdType  = cv::THRESH_BINARY_INV;
  constexpr int blockSize      = 15; // 11~21 normally  , 홀수
  int           C = 8; //-10~ 10 normally 가중평균에서 빼는 상수 .. -> threahold 조정역할 클수록 이미지 어두움
  cv::adaptiveThreshold(this->gray, this->binarized, maxValue, adaptiveMethod, thresholdType, blockSize, C);

#ifdef SHOW_VIDEO
  imshow("Thresholded", binarized);
  // waitKey(1)
#endif //! SHOW_VIDEO
}

void ImgPreprocessor::findClosestDotToCP() {

  // 1. 중심점 계산
  this->img_cross_point = cv::Point2f(src.cols * 0.5f, src.rows * 0.5f);

  const float         dist_tol_sq = 50.0f * 50.0f; // 허용 범위의 제곱
  float               min_dist_sq = std::numeric_limits<float>::max();
  const cv::KeyPoint *best_dot    = nullptr;

  for (const auto &dot : all_detected_dots_) {
    // 2. 사각형 범위(Bounding Box)로 1차 필터링 (매우 빠름)
    float dx = dot.pt.x - img_cross_point.x;
    if (std::abs(dx) > 50.0f)
      continue;

    float dy = dot.pt.y - img_cross_point.y;
    if (std::abs(dy) > 50.0f)
      continue;

    // 3. 제곱 거리를 이용한 비교 (sqrt 생략으로 성능 향상)
    float current_dist_sq = dx * dx + dy * dy;

    if (current_dist_sq < min_dist_sq && current_dist_sq <= dist_tol_sq) {
      min_dist_sq = current_dist_sq;
      best_dot    = &dot;
    }
  }

  // 4. 결과 업데이트 및 시각화
  if (best_dot != nullptr) {
    this->closest_dot_to_cross_point = *best_dot;
    cv::circle(src, closest_dot_to_cross_point.pt, 5, cv::Scalar(255, 20, 5), -1);
  } else {
    // 찾지 못했을 경우의 예외 처리 (예: 점 초기화)
    this->closest_dot_to_cross_point = cv::KeyPoint();
  }
}

void ImgPreprocessor::orderBaseLinesByClosestDot() {
  std::vector<std::pair<double, std::vector<Line<int>>>> line_to_dot_distances; // CenterDot 거리와 해당 라인을 묶음
  for (const auto &cluster : clustered_lines) {
    for (const auto &line : cluster) {
      double dist = distanceFromPointToLine(closest_dot_to_cross_point.pt, line.pt1, line.pt2);
      line_to_dot_distances.push_back({dist, cluster});
    }
  }
  sort(line_to_dot_distances.begin(), line_to_dot_distances.end(),
       [](const pair<double, vector<Line<int>>> &a, const pair<double, vector<Line<int>>> &b) {
         return a.first < b.first;
       });

  for (int i = 0; i < line_to_dot_distances.size(); i++) {
    this->closestLines.push_back(line_to_dot_distances[i].second);
  }
}

auto ImgPreprocessor::distanceFromPointToLine(cv::Point pt, cv::Point lineStart, cv::Point lineEnd) -> double {
  double numer = abs((lineEnd.y - lineStart.y) * pt.x - (lineEnd.x - lineStart.x) * pt.y + lineEnd.x * lineStart.y -
                     lineEnd.y * lineStart.x);
  double denom = sqrt(pow(lineEnd.y - lineStart.y, 2) + pow(lineEnd.x - lineStart.x, 2));
  return numer / denom;
}

auto ImgPreprocessor::findValidLinesByDotsLocation() -> bool {
  vector<vector<Line<int>>> validated_lines;
  double                    tolerenceMax_between_line_to_dot = closest_dot_to_cross_point.size * 1.5;
  // cout<<this->closest_dot_to_cross_point.size;

  // 선 위 점이 없는 선들을 제거
  for (const auto &line : closestLines) {
    for (const auto &lin : line) {
      int               cnt = 0;
      vector<Line<int>> validated_line;
      for (const auto &dot : all_detected_dots_) {
        if (this->isKeyPointNearLine(dot, lin.pt1, lin.pt2, tolerenceMax_between_line_to_dot)) {
          cnt++;
        }
        if (cnt >= (expansion_point_num * 2) + 1) {
          break;
        }
      }
      if (cnt >= (expansion_point_num * 2) + 1) {
        validated_line.push_back(lin);
        validated_lines.push_back(validated_line);
      } else {
        break;
      }
    }
  }
  closestLines = validated_lines;
  return true;
}
auto ImgPreprocessor::findValidLinesByDotsNum() -> bool {
  bool callCnt = 0;
  // 한 프레임에 두번만 반복
  for (int i = 0; i < 2; i++) {
    if (closestLines.size() < this->required_baseline_cnt) {
      LBS_error_code = findValidLines_error;
      return false;
    }

    std::vector<Line<int>> validate_lines;
    for (int j = 0; j < this->required_baseline_cnt; j++) {
      validate_lines.push_back({this->closestLines.at(j).at(0).pt1, this->closestLines.at(j).at(0).pt2});
    }
    nth_closest_keypoints_on_baseline.clear();
    nth_closest_keypoints_on_baseline.resize(required_baseline_cnt);

    double dist_tolerance = this->closest_dot_to_cross_point.size * 1;

    for (const auto &key : all_detected_dots_) {
      for (int k = 0; k < required_baseline_cnt; k++) {
        if (isKeyPointNearLine(key, validate_lines[k].pt1, validate_lines[k].pt2, dist_tolerance)) {
          nth_closest_keypoints_on_baseline[k].push_back(key);
        }
      }
    }
    // isKeyPointNearLine을 통해 도출된 각 기준선 마다의 Dot의 갯수를 검사. 2.5배 이상의 갯수를 가지고 있는 해당라인을
    // 지우고 다시 검사. check validate Lines
    for (int l = 0; l < required_baseline_cnt; l++) {
      bool  check              = true;
      float comp_keypoitns_cnt = static_cast<float>(nth_closest_keypoints_on_baseline[i].size());
      for (int j = l; j < required_baseline_cnt; j++) {
        float comp2_cnt = static_cast<float>(nth_closest_keypoints_on_baseline[j].size()) * 2.5;
        if (comp_keypoitns_cnt >= comp2_cnt) {
          this->closestLines.erase(closestLines.begin() + i + 1);
          check = false;
          break;
        }
      }
      if (!check) {
        break;
      }
    }
    if (closestLines.size() < this->required_baseline_cnt) {
      return false;
    }
    break;
  }
  return true;
}
auto ImgPreprocessor::validateClosestBaseLinesByDirection() -> bool {
  lines_dist_radian_idx_representative.resize(2);

  // CrossPoint 가 선 위에 있는지 고려함.
  this->is_crosshair_on_baseline = this->isKeyPointNearLine(
      cv::KeyPoint(this->img_cross_point, 0), closestLines.at(0).at(0).pt1, closestLines.at(0).at(0).pt2, 1);
  std::vector<double> lines_radians_from_imgCP;
  size_t              idx = 0;
  if (is_crosshair_on_baseline) {
    lines_dist_radian_idx.emplace_back();
    idx++;
  }

  for (size_t i = idx; i < required_baseline_cnt; i++) {
    cv::Point2f tmp_pt =
        getClosestPointOnLine(this->closestLines[i][0].pt1, this->closestLines[i][0].pt2, this->img_cross_point);
    cv::Point2f temp_direction_vector = (tmp_pt - this->img_cross_point);
    float       temp_dist             = norm(this->img_cross_point - tmp_pt);
    double      temp_radian           = atan2(temp_direction_vector.y, temp_direction_vector.x);
    lines_dist_radian_idx.emplace_back(temp_dist, temp_radian, i);
    lines_radians_from_imgCP.push_back(temp_radian);
  }

  // 첫번째 라디안을 비교기준으로, 나머지 라디안 값들이 모두 같다고 판정하면 return false;
  float lines_vector_similar_tolerence = 2.0; // 2.5 radians -> angle 143
  float radians_compare_source         = lines_radians_from_imgCP[idx];
  for (; idx < lines_dist_radian_idx.size(); idx++) {

    bool representative_vector = this->isAngleSimilar(radians_compare_source, std::get<1>(lines_dist_radian_idx[idx]),
                                                      lines_vector_similar_tolerence);
    if (representative_vector) {
      lines_dist_radian_idx_representative[0].push_back(lines_dist_radian_idx[idx]); // true-> 0에 삽입
    } else {
      lines_dist_radian_idx_representative[1].push_back(lines_dist_radian_idx[idx]); // false -> 1 에 삽입
    }
  }

  // crosshair를 기준, 기준선이 한쪽으로 쏠려있는 구조라면 false
  for (const auto &line_info : lines_dist_radian_idx_representative) {
    if (line_info.size() == required_baseline_cnt) {
      return false;
    }
  }

  return true;
}

auto ImgPreprocessor::getClosestPointOnLine(const cv::Point2f &line_start, const cv::Point2f &line_end,
                                            const cv::Point2f &P0) -> cv::Point2f {
  float dx = line_end.x - line_start.x;
  float dy = line_end.y - line_start.y;
  if (dx == 0 && dy == 0) {
    // 동일 점인 경우 line_start 반환 (error case)
    return line_start;
  }
  float t = ((P0.x - line_start.x) * dx + (P0.y - line_start.y) * dy) / (dx * dx + dy * dy);
  if (t < 0) {
    return line_start; // 점이 선분의 line_start쪽에 가장 가까움 (error case)
  } else if (t > 1) {
    return line_end; // 점이 선분의 line_end 쪽에 가장 가까움 (error case)
  }
  return cv::Point2f(line_start.x + t * dx, line_start.y + t * dy); // 선분 위의 점 반환
}

auto ImgPreprocessor::isKeyPointNearLine(const cv::KeyPoint &keypoint, const cv::Point2f &lineStart,
                                         const cv::Point2f &lineEnd, double maxDistance) -> bool {
  double dx = lineEnd.x - lineStart.x;
  double dy = lineEnd.y - lineStart.y;

  // 분자 계산
  double numerator = dy * keypoint.pt.x - dx * keypoint.pt.y + lineEnd.x * lineStart.y - lineEnd.y * lineStart.x;
  // 분모의 제곱 계산
  double denominator_sq = dy * dy + dx * dx;

  // sqrt 연산을 피하기 위해 양변을 제곱하여 비교 (거리^2 <= 최대거리^2)
  return (numerator * numerator) <= (maxDistance * maxDistance * denominator_sq);
}

auto ImgPreprocessor::findClosestBaselineDotToNearCP() -> void {

  cv::KeyPoint closestKeyPoint;
  float        minDistance = std::numeric_limits<float>::max();

  for (const auto &keypoint : nth_closest_keypoints_on_baseline[0]) {
    float distance = cv::norm(keypoint.pt - closest_dot_to_cross_point.pt);

    if (distance < minDistance) {
      minDistance     = distance;
      closestKeyPoint = keypoint;
    }
  }

  if (minDistance != std::numeric_limits<float>::max()) {
    closest_baseline_dot = (closestKeyPoint);
  }
}

void ImgPreprocessor::drawClusteredLines() {
  if (src.type() == CV_8UC1) {
    cv::cvtColor(src, src, COLOR_GRAY2BGR);
  }
  for (const auto &ke : clustered_lines) {
    cv::line(src, cv::Point(ke[0].pt1.x, ke[0].pt1.y), cv::Point(ke[0].pt2.x, ke[0].pt2.y), Scalar(0, 0, 255), 1);
  }
#ifdef SHOW_VIDEO
  imshow("drawClusteredLines", this->src);
#endif
}

auto ImgPreprocessor::findwarpPerspectiveCornerPoint() -> bool {

  for (const auto &tuples : lines_dist_radian_idx_representative) {
    Point2f temp_dot = {-1, -1};
    if (tuples.empty()) {
      if (!is_crosshair_on_baseline) // cross-point가 라인 위에 있었을 때
      {                              // 라인 정보가 비어 있음 (error)
        LBS_error_code = findwarpPerspectiveCornerPoint_fatal_error;
        return false;
      } else {
        side_line_info.emplace_back(closest_baseline_dot.pt, closestLines.at(0).at(0), 0);
        continue;
      }
    }
    int         line_idx = -1;
    cv::Point2f min_dist_dot;
    for (const auto &tuple : tuples) {
      if (temp_dot.x == -1) {
        temp_dot = closest_baseline_dot.pt;
      }
      float min_dist = numeric_limits<float>::max();
      line_idx       = std::get<2>(tuple);

      for (const auto &ke : this->nth_closest_keypoints_on_baseline[line_idx]) {
        float temp_dot_dist = norm(temp_dot - ke.pt);
        if (min_dist > temp_dot_dist) {
          min_dist     = temp_dot_dist;
          min_dist_dot = ke.pt;
        }
      }
    }
    side_line_info.emplace_back(min_dist_dot, closestLines[line_idx][0], line_idx);
  }
// test cornet dots Info
#ifdef SHOW_VIDEO
  for (auto dot : nth_closest_keypoints_on_baseline[get<2>(side_line_info[0])]) {
    circle(src, dot.pt, dot.size, {0, 255, 0}, -1);
  }
  for (auto dot : nth_closest_keypoints_on_baseline[get<2>(side_line_info[1])]) {
    circle(src, dot.pt, dot.size * 1.2, {0, 0, 255}, -1);
  }
  constexpr int CP_size = 6;
  cv::line(this->src, cv::Point(static_cast<int>(img_cross_point.x) - CP_size, static_cast<int>(img_cross_point.y)),
           cv::Point(static_cast<int>(img_cross_point.x) + CP_size, static_cast<int>(img_cross_point.y)), {233, 233, 1},
           1);
  cv::line(this->src, cv::Point(static_cast<int>(img_cross_point.x), static_cast<int>(img_cross_point.y) - CP_size),
           cv::Point(static_cast<int>(img_cross_point.x), static_cast<int>(img_cross_point.y) + CP_size), {233, 233, 1},
           1);
  imshow("corner dots candidate", src);
  waitKey(1);
#endif //! SHOW_VIDEO

  // 확장 기준점으로부터 선분의 방향에 대해
  // 얼마만큼의 확장이 필요한지의 갯수
  int start_expansion1 = 0, end_expansion1 = 0;
  int start_expansion2 = 0, end_expansion2 = 0;

  std::vector<cv::KeyPoint> dots_to_start1, dots_to_end1;
  std::vector<cv::KeyPoint> dots_to_start2, dots_to_end2;

  for (int i = 0; i < 2; i++) {
    if (i == 1) {
      if ((this->two_Corner_dot_on_side_Line1.size() < 2 || this->two_Corner_dot_on_side_Line2.size() < 2)) {
        std::swap(get<1>(side_line_info[0]).pt1, get<1>(side_line_info[0]).pt2);
        start_expansion1 = 0, end_expansion1 = 0, start_expansion2 = 0, end_expansion2 = 0;
        dots_to_start1.clear();
        dots_to_end1.clear();
        dots_to_start2.clear();
        dots_to_end2.clear();
      } else
        break;
    }
    findCornerDotCandidates(get<1>(side_line_info[0]).pt1, get<1>(side_line_info[0]).pt2, get<0>(side_line_info[0]),
                            nth_closest_keypoints_on_baseline[get<2>(side_line_info[0])], start_expansion1,
                            end_expansion1, dots_to_start1, dots_to_end1);
    findCornerDotCandidates(get<1>(side_line_info[1]).pt1, get<1>(side_line_info[1]).pt2, get<0>(side_line_info[1]),
                            nth_closest_keypoints_on_baseline[get<2>(side_line_info[1])], start_expansion2,
                            end_expansion2, dots_to_start2, dots_to_end2);

    // 코너닷 과정
    bool is_start_expasion = start_expansion1 != 0 || start_expansion2 != 0;
    bool is_end_expasion   = end_expansion1 != 0 || end_expansion2 != 0;

    float pt_size = 2; // 이니셜라이저 맞추기 위한 점의 크기 ,중요하지 않음
    if (is_start_expasion) {
      int max_exp_nums = std::max(start_expansion1, start_expansion2);
      // 각 방향의 점이 조건에 따라 충분히 존재하는지 검사
      bool check_start_num = dots_to_start1.size() >= (expansion_point_num) + max_exp_nums &&
                             dots_to_start2.size() >= (expansion_point_num) + max_exp_nums;
      bool check_end_num = dots_to_end1.size() >= (expansion_point_num)-max_exp_nums &&
                           dots_to_end2.size() >= (expansion_point_num)-max_exp_nums;

      if (check_start_num && check_end_num) // 확장점 갯수가 expansion_point_num과 같은 경우.
      {
        this->two_Corner_dot_on_side_Line1.push_back(dots_to_start1[max_exp_nums + (expansion_point_num)-1]);
        this->two_Corner_dot_on_side_Line2.push_back(dots_to_start2[max_exp_nums + (expansion_point_num)-1]);

        if (max_exp_nums == this->expansion_point_num) {
          this->two_Corner_dot_on_side_Line1.push_back({get<0>(side_line_info[0]), pt_size});
          this->two_Corner_dot_on_side_Line2.push_back({get<0>(side_line_info[1]), pt_size});
        } else {
          this->two_Corner_dot_on_side_Line1.push_back(dots_to_end1[(expansion_point_num)-max_exp_nums - 1]);
          this->two_Corner_dot_on_side_Line2.push_back(dots_to_end2[(expansion_point_num)-max_exp_nums - 1]);
        }
      }
    } else if (is_end_expasion) {
      int max_exp_nums = std::max(end_expansion1, end_expansion2);

      bool check_end_num = dots_to_end1.size() >= expansion_point_num + max_exp_nums &&
                           dots_to_end2.size() >= expansion_point_num + max_exp_nums;
      bool check_start_num = dots_to_start1.size() >= expansion_point_num - max_exp_nums &&
                             dots_to_start2.size() >= expansion_point_num - max_exp_nums;
      if (check_end_num && check_start_num) {
        this->two_Corner_dot_on_side_Line1.push_back(dots_to_end1[max_exp_nums + (expansion_point_num)-1]);
        this->two_Corner_dot_on_side_Line2.push_back(dots_to_end2[max_exp_nums + (expansion_point_num)-1]);
        if (max_exp_nums == this->expansion_point_num) // 확장점 갯수가 expansion_point_num과 같은 경우.
        {
          this->two_Corner_dot_on_side_Line1.push_back({get<0>(side_line_info[0]), pt_size});
          this->two_Corner_dot_on_side_Line2.push_back({get<0>(side_line_info[1]), pt_size});
        } else {
          this->two_Corner_dot_on_side_Line1.push_back(dots_to_start1[(expansion_point_num)-max_exp_nums - 1]);
          this->two_Corner_dot_on_side_Line2.push_back(dots_to_start2[(expansion_point_num)-max_exp_nums - 1]);
        }
      }
    } else {
      // 깔끔하게 나올경우.
      this->two_Corner_dot_on_side_Line1.push_back(dots_to_start1[(expansion_point_num)-1]);
      this->two_Corner_dot_on_side_Line1.push_back(dots_to_end1[(expansion_point_num)-1]);
      this->two_Corner_dot_on_side_Line2.push_back(dots_to_start2[(expansion_point_num)-1]);
      this->two_Corner_dot_on_side_Line2.push_back(dots_to_end2[(expansion_point_num)-1]);
    }
  }

  if (two_Corner_dot_on_side_Line1.size() < 2 || two_Corner_dot_on_side_Line2.size() < 2) {

    LBS_error_code = findwarpPerspectiveCornerPoint_sidelines_error;
    return false;
  }
  return true;
}

auto ImgPreprocessor::findCornerDotCandidates(const cv::Point2f &lineStart, const cv::Point2f &lineEnd,
                                              const cv::Point2f &startDot, const std::vector<cv::KeyPoint> &keypoints,
                                              int &start_expansion, int &end_expansion,
                                              std::vector<cv::KeyPoint> &dots_to_start,
                                              std::vector<cv::KeyPoint> &dots_to_end) -> bool {

  cv::Point2f directionToStart = lineStart - startDot;
  cv::Point2f directionToEnd   = lineEnd - startDot;

  // 확장 기준점에서 선분의 해당 방향으로의 점의 갯수
  std::vector<std::pair<float, cv::Point2f>> points_to_start, points_to_end;

  for (int i = 0; i < keypoints.size(); ++i) {
    // 동일 위치 제거
    if (cv::norm(keypoints[i].pt - startDot) <= std::numeric_limits<float>::epsilon())
      continue;

    // 방향이 맞는지 확인 (dotProduct가 양수인지 -> 양수면은 해당 방향의 정보가 있는 곳으로 pushback 한다.)
    cv::Point2f to_dot               = keypoints[i].pt - startDot;
    float       dot_product_to_start = to_dot.dot(directionToStart);
    float       dot_product_to_end   = to_dot.dot(directionToEnd);
    if (dot_product_to_start > 0) {
      float distance = cv::norm(to_dot);
      points_to_start.emplace_back(distance, cv::Point2f(keypoints[i].pt));
    }
    if (dot_product_to_end > 0) {
      float distance = cv::norm(to_dot);
      points_to_end.emplace_back(distance, cv::Point2f(keypoints[i].pt));
    }
  }
  // dist 오름차순 정렬
  auto sortByDistance = [](const std::pair<float, Point2f> &a, const std::pair<float, Point2f> &b) {
    return a.first < b.first;
  };
  std::sort(points_to_start.begin(), points_to_start.end(), sortByDistance);
  std::sort(points_to_end.begin(), points_to_end.end(), sortByDistance);

  for (const auto &pt : points_to_start) {
    dots_to_start.emplace_back(pt.second, 2);
  }
  for (const auto &pt : points_to_end) {
    dots_to_end.emplace_back(pt.second, 2);
  }

  // 확장 기준점으로부터 선분의 start 방향에 점이 한개도 없다면 end방향 점들의 갯수가 expansion_point_num만큼 더
  // 있어야함.
  if (points_to_start.size() == 0) {
    end_expansion = this->expansion_point_num;
  }
  // expansion_point_num보다 적게 있다면 반대 방향에 적은 숫자만큼 더 있어야함.
  else if (points_to_start.size() < expansion_point_num) {
    end_expansion = expansion_point_num - points_to_start.size();
  }

  if (points_to_end.size() == 0) {
    start_expansion = this->expansion_point_num;
  } else if (points_to_end.size() < expansion_point_num) {
    start_expansion = expansion_point_num - points_to_end.size();
  }

  return true;
}

auto ImgPreprocessor::isRectangleWithMargin(cv::Point p1, cv::Point p2, cv::Point p3, cv::Point p4, double margin)
    -> bool {
  auto angleDiff = [](double angle1, double angle2) {
    double diff = fabs(angle1 - angle2);
    return std::min(diff, CV_PI - diff);
  };

  // 각도 계산
  double angle1 = atan2(p2.y - p1.y, p2.x - p1.x);
  double angle2 = atan2(p3.y - p2.y, p3.x - p2.x);
  double angle3 = atan2(p4.y - p3.y, p4.x - p3.x);
  double angle4 = atan2(p1.y - p4.y, p1.x - p4.x);

  // 변의 길이 계산
  double dist1 = sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2));
  double dist2 = sqrt(pow(p3.x - p2.x, 2) + pow(p3.y - p2.y, 2));
  double dist3 = sqrt(pow(p4.x - p3.x, 2) + pow(p4.y - p3.y, 2));
  double dist4 = sqrt(pow(p1.x - p4.x, 2) + pow(p1.y - p4.y, 2));

  // 변 길이와 각도 비교
  bool lengthCheck1 = fabs(dist1 - dist3) <= dist1 * margin;
  bool lengthCheck2 = fabs(dist2 - dist4) <= dist2 * margin;
  bool angleCheck = angleDiff(angle1, angle3) <= CV_PI / 2 * margin && angleDiff(angle2, angle4) <= CV_PI / 2 * margin;

  return lengthCheck1 && lengthCheck2 && angleCheck;
}

void ImgPreprocessor::drawDetectedCornerDots() {
  if (src.type() == CV_8UC1) {
    cv::cvtColor(src, src, COLOR_GRAY2BGR);
  }
  for (auto &ke : two_Corner_dot_on_side_Line1) {
    cv::circle(src, ke.pt, 10, {204, 51, 0}, -1);
    ke.size = 20;
  }
  for (auto &ke : two_Corner_dot_on_side_Line2) {
    cv::circle(src, ke.pt, 10, {0, 153, 255}, -1);
    ke.size = 20;
  }

  // cv::drawKeypoints(src, two_Corner_dot_on_side_Line1, src, cv::Scalar(255, 255, 0),
  // cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS); cv::drawKeypoints(src, two_Corner_dot_on_side_Line2, src, cv::Scalar(0,
  // 0, 0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
}

auto ImgPreprocessor::arrangeDstQuad(const std::vector<cv::Point2f> &warp_point_candidate) {
  std::vector<cv::Point2f> dst_quad(4);

  // 첫 번째 쌍과 두 번째 쌍의 중점을 계산
  cv::Point2f midpoint1 = (warp_point_candidate[0] + warp_point_candidate[1]) * 0.5f;
  cv::Point2f midpoint2 = (warp_point_candidate[2] + warp_point_candidate[3]) * 0.5f;

  // 중점 x좌표 같은 경우를 처리 /////////////////////////////
  if (std::abs(midpoint1.x - midpoint2.x) < 1) {
    // y 좌표의 평균을 기준으로 왼쪽과 오른쪽을 구분
    float avg_y = (warp_point_candidate[0].y + warp_point_candidate[1].y + warp_point_candidate[2].y +
                   warp_point_candidate[3].y) /
                  4.0f;

    std::vector<cv::Point2f> left_points, right_points;

    for (const auto &point : warp_point_candidate) {
      if (point.y > avg_y) {
        left_points.push_back(point);
      } else {
        right_points.push_back(point);
      }
    }
    // 왼쪽 점들 중에서 위/아래 결정
    dst_quad[0] = left_points[0].x < left_points[1].x ? left_points[0] : left_points[1]; // top-left
    dst_quad[1] = left_points[0].x < left_points[1].x ? left_points[1] : left_points[0]; // bottom-left

    double norm1 = norm(dst_quad[1] - right_points[0]);
    double norm2 = norm(dst_quad[1] - right_points[1]);
    dst_quad[2]  = norm1 < norm2 ? right_points[0] : right_points[1]; // bottom-right
    dst_quad[3]  = norm1 < norm2 ? right_points[1] : right_points[0]; // top-right
  } else if (abs(midpoint1.y - midpoint2.y) < 1) {
    int                             left_y_avg         = 0;
    int                             right_y_avg        = 0;
    bool                            is_first_pair_left = midpoint1.x < midpoint2.x;
    const std::vector<cv::Point2f> &left_pair =
        is_first_pair_left ? std::vector<cv::Point2f>{warp_point_candidate[0], warp_point_candidate[1]}
                           : std::vector<cv::Point2f>{warp_point_candidate[2], warp_point_candidate[3]};
    const std::vector<cv::Point2f> &right_pair =
        is_first_pair_left ? std::vector<cv::Point2f>{warp_point_candidate[2], warp_point_candidate[3]}
                           : std::vector<cv::Point2f>{warp_point_candidate[0], warp_point_candidate[1]};

    for (auto it : left_pair) {
      left_y_avg += it.y;
    }
    for (auto it : right_pair) {
      right_y_avg += it.y;
    }
    left_y_avg /= 2;
    right_y_avg /= 2;
    if (left_y_avg < right_y_avg) {
      dst_quad[0] = left_pair[0].x > left_pair[1].x ? left_pair[0] : left_pair[1]; // top-left
      dst_quad[1] = left_pair[0].x > left_pair[1].x ? left_pair[1] : left_pair[0]; // bottom-left
    } else {
      dst_quad[0] = left_pair[0].y < left_pair[1].y ? left_pair[0] : left_pair[1]; // top-left
      dst_quad[1] = left_pair[0].y < left_pair[1].y ? left_pair[1] : left_pair[0]; // bottom-left
    }
    double norm1 = norm(dst_quad[1] - right_pair[0]);
    double norm2 = norm(dst_quad[1] - right_pair[1]);
    dst_quad[2]  = norm1 < norm2 ? right_pair[0] : right_pair[1]; // bottom-right
    dst_quad[3]  = norm1 < norm2 ? right_pair[1] : right_pair[0]; // top-right
  } else {
    bool                            is_first_pair_left = midpoint1.x < midpoint2.x;
    const std::vector<cv::Point2f> &left_pair =
        is_first_pair_left ? std::vector<cv::Point2f>{warp_point_candidate[0], warp_point_candidate[1]}
                           : std::vector<cv::Point2f>{warp_point_candidate[2], warp_point_candidate[3]};
    const std::vector<cv::Point2f> &right_pair =
        is_first_pair_left ? std::vector<cv::Point2f>{warp_point_candidate[2], warp_point_candidate[3]}
                           : std::vector<cv::Point2f>{warp_point_candidate[0], warp_point_candidate[1]};

    dst_quad[0] = left_pair[0].y < left_pair[1].y ? left_pair[0] : left_pair[1]; // top-left
    dst_quad[1] = left_pair[0].y < left_pair[1].y ? left_pair[1] : left_pair[0]; // bottom-left

    double norm1 = norm(dst_quad[1] - right_pair[0]);
    double norm2 = norm(dst_quad[1] - right_pair[1]);

    dst_quad[2] = norm1 < norm2 ? right_pair[0] : right_pair[1]; // bottom-right
    dst_quad[3] = norm1 < norm2 ? right_pair[1] : right_pair[0]; // top-right
  }

  return dst_quad;
}

auto ImgPreprocessor::adjustCornersToPredefinedArea() -> bool {

  this->dst_quad.resize(4);
  this->src_quad.resize(4);

  vector<cv::Point2f> warp_point_candidate;
  warp_point_candidate.reserve(4);
  warp_point_candidate.push_back(this->two_Corner_dot_on_side_Line1[0].pt);
  warp_point_candidate.push_back(this->two_Corner_dot_on_side_Line1[1].pt);
  warp_point_candidate.push_back(this->two_Corner_dot_on_side_Line2[0].pt);
  warp_point_candidate.push_back(this->two_Corner_dot_on_side_Line2[1].pt);

  dst_quad = arrangeDstQuad(warp_point_candidate);

  // draw warpPerspective order

  for (int i = 0; i < 3; i++) {
    line(src, dst_quad[i], dst_quad[i + 1], Scalar(255, 0, (i == 0 ? 255 : 0)), 4);
  }

  src_quad = {Point2f(0, 0), Point2f(0, ROI_height_size - 1), Point2f(ROI_width_size - 1, ROI_height_size - 1),
              Point2f(ROI_width_size - 1, 0)};

  if (isRectangleWithMargin(dst_quad[0], dst_quad[1], dst_quad[2], dst_quad[3], 0.3)) {
    cv::Mat pers;
    try {
      pers = cv::getPerspectiveTransform(dst_quad, src_quad);

      cv::Point2f img_up_point = img_cross_point + cv::Point2f(0.0f, -10.0f); // 10픽셀 위쪽 점

      // 3. 두 점을 dst_quad 좌표계로 변환합니다.
      // perspectiveTransform은 벡터(std::vector)를 인자로 받습니다.
      std::vector<cv::Point2f> src_pts = {img_cross_point, img_up_point};
      std::vector<cv::Point2f> dst_pts;
      cv::perspectiveTransform(src_pts, dst_pts, pers);

      // 4. dst 공간에서의 변위 벡터를 구합니다.
      cv::Point2f transformed_vec = dst_pts[1] - dst_pts[0];

      // 5. atan2를 이용하여 각도(radian)를 구합니다.
      // 결과값은 -PI ~ PI 범위입니다.
      double angle_rad = std::atan2(transformed_vec.y, transformed_vec.x);

      // 필요하다면 degree로 변환 (0도: 우측, 90도: 하단, -90도: 상단)
      camera_up_angle_ = angle_rad * 180.0 / CV_PI;

    } catch (...) {
      LBS_error_code = adjustCornersToPredefinedArea_Transform_error;
      return false;
    }

    cv::warpPerspective(this->gray, warpPerspectived_image, pers,
                        cv::Size(ROI_width_size, static_cast<int>(ROI_height_size)));

    cv::warpPerspective(this->binarized, warp_source_orient, pers,
                        cv::Size(ROI_width_size, static_cast<int>(ROI_height_size)));

  } else {
    LBS_error_code = adjustCornersToPredefinedArea_shape_error;
    return false;
  }
  return true;
}

auto ImgPreprocessor::isAngleSimilar(double angle1, double angle2, double tolernace) -> bool {
  double differ = std::abs(angle1 - angle2); // 두 앵글값의 차
  if (differ > CV_PI) {
    differ = 2 * CV_PI - differ;
  }
  return differ <= tolernace;
}

auto ImgPreprocessor::preprocessPointData() -> bool {
  Mat                      warpMatrix              = cv::getPerspectiveTransform(this->dst_quad, this->src_quad);
  std::vector<cv::Point2f> transformed_cross_point = {this->img_cross_point};

  // 4개의 코너점을 통해 영역을 형성하고 그 영역안에 dot이 있는지 검사.
  warpPerspectiveArea.reserve(warpPerspectiveArea.size() + dst_quad.size());
  warpPerspectiveArea.insert(warpPerspectiveArea.end(), dst_quad.begin(), dst_quad.end());

  size_t margin = 10;
  int    max_x  = std::max({dst_quad[0].x, dst_quad[1].x, dst_quad[2].x, dst_quad[3].x}) + margin;
  int    min_x  = std::min({dst_quad[0].x, dst_quad[1].x, dst_quad[2].x, dst_quad[3].x}) - margin;
  int    max_y  = std::max({dst_quad[0].y, dst_quad[1].y, dst_quad[2].y, dst_quad[3].y}) + margin;
  int    min_y  = std::min({dst_quad[0].y, dst_quad[1].y, dst_quad[2].y, dst_quad[3].y}) - margin;
  for (const auto &dot : all_detected_dots_) {
    if (min_x <= dot.pt.x && dot.pt.x <= max_x && min_y <= dot.pt.y && dot.pt.y <= max_y) {
      point_to_transform.push_back(dot.pt);
    }
  }

  const int MARGIN_LOSS_LINES       = 2;
  const int DASHLINES_LOSS_DOTS_NUM = ((ROI_height_dots - 1) / 3) * 2;
  auto      required_minimal_dot =
      ((ROI_width_dots - MARGIN_LOSS_LINES) * (ROI_height_dots - MARGIN_LOSS_LINES)) - DASHLINES_LOSS_DOTS_NUM;
  if ((required_minimal_dot > point_to_transform.size())) {
    LBS_error_code = preprocessPointData_lackPts_error;
    return false;
  }
  size_t      i = 0;
  vector<int> side_line_idx;
  for (const auto &info : side_line_info) {
    side_line_idx.push_back(std::get<2>(info));
  }
  for (const auto &lines : nth_closest_keypoints_on_baseline) {
    bool side_check = false;
    for (const auto &idx : side_line_idx) {
      if (idx == i) {
        side_check = true;
        i++;
        break;
      }
    }
    if (!side_check) {
      for (const auto &dot : nth_closest_keypoints_on_baseline[i]) {
        if (isPointInsidePolygon(warpPerspectiveArea, dot.pt)) {
          middle_baseLine_dots_ROI.push_back(dot.pt);
        }
      }
      i++;
    } else {
      continue;
    }
    if (middle_baseLine_dots_ROI.size() == 0) {
      LBS_error_code = preprocessPointData_noMidBaseLine_dots_error;
      return false;
    }

    cv::perspectiveTransform(middle_baseLine_dots_ROI, middle_baseLine_dots_ROI, warpMatrix);
    this->middle_baseLines_dots_in_ROI.push_back(middle_baseLine_dots_ROI);
  }
  try {
    for (const auto &dot : nth_closest_keypoints_on_baseline[0]) {
      if (isPointInsidePolygon(warpPerspectiveArea, dot.pt)) {
        middle_baseLine_dots_ROI.emplace_back(dot.pt);
      }
    }
  } catch (...) {
    LBS_error_code = preprocessPointData_error;
    return false;
  }

  try {
    cv::perspectiveTransform(point_to_transform, transformed_points, warpMatrix);
    cv::perspectiveTransform(transformed_cross_point, transformed_cross_point, warpMatrix);
  } catch (...) {
    LBS_error_code = preprocessPointData_transform_error;
    return false;
  }
  this->transformed_CP = transformed_cross_point[0];

  return true;
}

inline void ImgPreprocessor::drawCPOnImg() {
  constexpr int CP_size = 6;
  cv::line(this->src, cv::Point(static_cast<int>(img_cross_point.x) - CP_size, static_cast<int>(img_cross_point.y)),
           cv::Point(static_cast<int>(img_cross_point.x) + CP_size, static_cast<int>(img_cross_point.y)), {233, 233, 1},
           1);
  cv::line(this->src, cv::Point(static_cast<int>(img_cross_point.x), static_cast<int>(img_cross_point.y) - CP_size),
           cv::Point(static_cast<int>(img_cross_point.x), static_cast<int>(img_cross_point.y) + CP_size), {233, 233, 1},
           1);
}

auto ImgPreprocessor::excludeMarginDots() -> bool {
  int cut_margin_size = static_cast<int>(this->closest_dot_to_cross_point.size * 2);
  transformed_points.erase(std::remove_if(transformed_points.begin(), transformed_points.end(),
                                          [this, cut_margin_size](const Point &pt) {
                                            return pt.x <= cut_margin_size || pt.y <= cut_margin_size ||
                                                   this->ROI_width_size - cut_margin_size <= pt.x ||
                                                   this->ROI_height_size - cut_margin_size <= pt.y;
                                          }),
                           transformed_points.end());

  // mid baseline의 margin영역또한 지움
  for (auto &mid_lines_dots : middle_baseLines_dots_in_ROI) {
    mid_lines_dots.erase(std::remove_if(mid_lines_dots.begin(), mid_lines_dots.end(),
                                        [this, cut_margin_size](const Point &pt) {
                                          return pt.x <= cut_margin_size || pt.y <= cut_margin_size ||
                                                 this->ROI_width_size - cut_margin_size <= pt.x ||
                                                 this->ROI_height_size - cut_margin_size <= pt.y;
                                        }),
                         mid_lines_dots.end());
    if (mid_lines_dots.size() > 10) {
      LBS_error_code = excludeMarginDots_lackMidPts_error;
      return false;
    }
  }
  return true;
}

auto ImgPreprocessor::isPointInsidePolygon(const std::vector<cv::Point> &polygon, const cv::Point &point) -> bool {
  double result = cv::pointPolygonTest(polygon, point, false);
  return result >= 0;
}

void ImgPreprocessor::drawTransformedDots() {
  if (warpPerspectived_image.type() == CV_8UC1) {
    cv::cvtColor(warpPerspectived_image, warpPerspectived_image, COLOR_GRAY2BGR);
  }
  for (const auto &dot : transformed_points) {
    circle(warpPerspectived_image, dot, 2, cv::Scalar(0, 0, 255), 2, -1);
  }

  // Cross Point 표시
  int size = 10;
  cv::line(this->warpPerspectived_image, cv::Point(transformed_CP.x - size, transformed_CP.y),
           cv::Point(transformed_CP.x + size, transformed_CP.y), {233, 233, 1}, 2);
  cv::line(this->warpPerspectived_image, cv::Point(transformed_CP.x, transformed_CP.y - size),
           cv::Point(transformed_CP.x, transformed_CP.y + size), {233, 233, 1}, 2);

  // 가상 그리드라인 그리기
  float line_start = 0;
  float line_step  = this->ROI_width_size / (ROI_width_dots - 1);
  for (int i = 0; i <= (ROI_width_dots - 1); i++) {
    Point2f start = {line_start, 0};
    Point2f end   = {line_start, 1000};
    line(warpPerspectived_image, start, end, Scalar(0, 0, 255), 1);
    line_start += line_step;
  }
  line_start       = 0;
  float line_step2 = this->ROI_height_size / (ROI_height_dots - 1);
  for (int i = 0; i <= (ROI_height_dots - 1); i++) {
    Point2f start = {0, line_start};
    Point2f end   = {1000, line_start};
    line(warpPerspectived_image, start, end, Scalar(0, 0, 255), 1);
    line_start += line_step2;
  }
}

auto ImgPreprocessor::setupPreprocessing(Mat &source) -> bool {
  this->src = source.clone();

  // 그레이스케일로 변환
  if (src.channels() != 1) {
    cv::cvtColor(this->src, this->gray, COLOR_BGR2GRAY);
  }

  // 노이즈 제거를 위해 가우시안 블러 적용 (매우 작은 사진에서는 유의해야 함)
  cv::GaussianBlur(this->gray, this->gray, cv::Size(5, 5), 0);

  return true;
}

auto ImgPreprocessor::mergeCloseLines(const vector<Vec4i> &lines, float maxDistance) -> vector<Vec4i> {
  vector<Vec4i> mergedLines;
  vector<bool>  merged(lines.size(), false); // 선분이 합쳐졌는지 여부를 추적하는 벡터

  for (size_t i = 0; i < lines.size(); ++i) {
    if (merged[i])
      continue; // 이미 합쳐진 선분은 건너뛴다

    Vec4i currentLine = lines[i];
    Point start(currentLine[0], currentLine[1]), end(currentLine[2], currentLine[3]);

    // 현재 선분과 가까운 다른 선분을 찾아 합친다
    for (size_t j = i + 1; j < lines.size(); ++j) {
      if (merged[j])
        continue; // 이미 합쳐진 선분은 건너뛴다

      Vec4i compare_line = lines[j];
      Point start_compare(compare_line[0], compare_line[1]), end_compare(compare_line[2], compare_line[3]);

      // 선분의 시작점 또는 끝점이 가까운 경우 선분을 합친다
      if (norm(start - start_compare) < maxDistance || norm(end - end_compare) < maxDistance ||
          norm(start - end_compare) < maxDistance || norm(end - start_compare) < maxDistance) {
        // 선분을 합치기 위해 시작점과 끝점을 갱신한다
        start     = Point(min(start.x, start_compare.x), min(start.y, start_compare.y));
        end       = Point(max(end.x, end_compare.x), max(end.y, end_compare.y));
        merged[j] = true; // 선분을 합쳤음을 표시
      }
    }
    mergedLines.emplace_back(start.x, start.y, end.x, end.y);
  }

  return mergedLines;
}

void ImgPreprocessor::normalizeLinePoints(vector<Vec4i> &lines) {
  for (auto &line : lines) {
    Point start(line[0], line[1]);
    Point end(line[2], line[3]);

    // 시작점이 끝점보다 오른쪽에 있거나, 더 아래에 있는 경우, 시작점과 끝점을 교환
    if (start.x > end.x || start.y > end.y) {
      swap(line[0], line[2]);
      swap(line[1], line[3]);
    }
  }
}

void ImgPreprocessor::setHoughPAndFindOrientLines(cv::Mat &img_ROI, std::vector<Vec4i> &lines) {
  // HoughLinesP를 위한 세팅값
  int threshold_value = 60;
  int minLineLength   = 50;
  int maxLineGap      = 10;

  HoughLinesP(img_ROI, lines, 1, CV_PI / 180, threshold_value, minLineLength, maxLineGap);
}

auto ImgPreprocessor::checkAndCorrectOrientation() -> bool {
  bool is_flipped = false;

  // 중앙 Triple-Order Line 체크를 위한 영역 설정 (이미지 뒤집힘 체크)
  int         size_roi = 50;                                                       // 임의 설정
  cv::Point2i start_pt = {static_cast<int>(this->warp_source_orient.cols / 2), 0}; // ROI 상단 중앙 좌표
  cv::Rect    roi_area = {start_pt.x - size_roi, start_pt.y, size_roi * 2,
                          static_cast<int>(this->warp_source_orient.rows)};
  Mat         img_ROI  = this->warp_source_orient(roi_area).clone();

  if (img_ROI.channels() != 1) {
    cvtColor(img_ROI, img_ROI, COLOR_BGR2GRAY);
  }

  // 선 소실점 복원에 대한 모폴로지 연산
  Mat kernel     = getStructuringElement(MORPH_RECT, Size(2, 4));
  int close_iter = 1;
  cv::morphologyEx(img_ROI, img_ROI, cv::MORPH_CLOSE, kernel, {}, close_iter);
  cv::threshold(img_ROI, img_ROI, 0, 255, THRESH_BINARY | THRESH_OTSU);

  vector<Vec4i> lines;
  setHoughPAndFindOrientLines(img_ROI, lines);
  normalizeLinePoints(lines);

  int           line_merge_tolerance = 30;
  vector<Vec4i> merged_lines         = mergeCloseLines(lines, line_merge_tolerance);
  // double overlappedLineCut = 10; // test 겹친 선 제거 위한 범위 변수

  // 선 길이,위치 정보를 저장
  vector<pair<float, Point>> linesInfo;
  for (auto l : merged_lines) {
    float length = sqrt(pow(l[2] - l[0], 2) + pow(l[3] - l[1], 2));
    Point line_center_location((l[2] + l[0]) / 2, (l[3] + l[1]) / 2);
    linesInfo.emplace_back(length, line_center_location);
  }

  // 선의 위치(높이)에 따라 정렬
  sort(linesInfo.begin(), linesInfo.end(),
       [](const pair<float, Point> &a, const pair<float, Point> &b) { return a.second.y < b.second.y; });

  if (linesInfo.size() != 0) {
    for (int i = 0; i < linesInfo.size() - 1; i++) {
      double                     temp;
      vector<pair<float, Point>> savedLines = linesInfo;

      // 겹치는 라인 제거
      try {
        if (abs(linesInfo[i + 1].second.y - linesInfo[i].second.y) < 5) {
          temp = linesInfo[i + 1].first >= linesInfo[i].first ? i : i + 1;
          linesInfo.erase(linesInfo.begin() + temp);
        }
      } catch (...) { return false; }
    }
  } else {
    return false;
  }
  // 선 그리기

  // 라인의 갯수가 타겟 범위에 들어오지 않을 경우 false  warpPerspective Size가 13 -> 라인이 4개가 들어옴.
  // int linesInfoSize = ROI_height_dots=
  if (!(linesInfo.size() == 4)) {
    LBS_error_code = orienting_LineNums_error;
#ifdef SHOW_VIDEO
    cout << "orienting_line_num_error, line size : " << linesInfo.size() << endl; // 테스트 코드
    imshow("orienting_line_num_error", img_ROI);
    waitKey(1);
#endif //! SHOW_VIDEO
    return false;
  }

  // validate dash line
  float       max_ = max({linesInfo[0].first, linesInfo[1].first, linesInfo[2].first}); // 각 위치 길이 저장..
  const float line_diff_max_tolerence = 1.8;
  if (max_ >= linesInfo[0].first * line_diff_max_tolerence || max_ >= linesInfo[1].first * line_diff_max_tolerence ||
      max_ >= linesInfo[2].first * line_diff_max_tolerence) {
    // test

#ifdef SHOW_VIDEO
    imshow("line_diff_error -> max is so big", img_ROI);
    // waitKey(1);
    cout << "orient Line error(max가 나머지것에 비해 1.8배가 넘어감)" << endl;
    cout << "max : " << max_ << endl;
    cout << "1 : " << linesInfo[0].first << "," << linesInfo[1].first << "," << linesInfo[2].first << endl;
#endif //! SHOW_VIDEO

    LBS_error_code = orienting_line_max_too_big_error;
    return false;
  }
  float temp_a       = std::abs(linesInfo[0].first - linesInfo[1].first);
  float temp_b       = std::abs(linesInfo[1].first - linesInfo[2].first);
  float temp_c       = std::abs(linesInfo[2].first - linesInfo[0].first);
  float len_diff_min = 6;
  if (temp_a < len_diff_min || temp_b < len_diff_min || temp_c < len_diff_min) {

#ifdef SHOW_VIDEO
    cvtColor(img_ROI, img_ROI, COLOR_GRAY2BGR);
    for (auto l : merged_lines) {
      line(img_ROI, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 2);
    }

    cout << "diff_min_error" << endl;
    cout << "Line Top " << linesInfo[0].first << ", Line mid:" << linesInfo[1].first
         << ", Line bot  : " << linesInfo[2].first << endl;
    imshow("diff_min_error", img_ROI);
    // waitKey(1);
#endif //! SHOW_VIDEO

    LBS_error_code = orienting_cantKnowLineDiff_error;
    return false;
  }

  // check orient
  if (linesInfo.size() == 4) {
    int top = static_cast<int>(linesInfo[0].first);
    int mid = static_cast<int>(linesInfo[1].first);
    int bot = static_cast<int>(linesInfo[2].first);

    vector<int> length_order = {top, mid, bot};
    std::sort(length_order.begin(), length_order.end(), [](int a, int b) { return a > b; });

    int A = length_order[0];
    int B = length_order[1];

    if (A == top) {
      if (B != mid) {
        is_flipped = true;
      }
    } else if (A == mid) {
      if (B != bot) {
        is_flipped = true;
      }
    } else if (A == bot) {
      if (B != top) {
        is_flipped = true;
      }
    }
  } else {
    LBS_error_code = orienting_lackLineNums_error;
    return false;
  }

  // transfomed_points _filp
  if (is_flipped) {
    for (auto &point : this->transformed_points) {
      point.x = this->ROI_width_size - point.x;
      point.y = this->ROI_height_size - point.y;
    }
    this->transformed_CP.x = this->ROI_width_size - transformed_CP.x;
    this->transformed_CP.y = this->ROI_height_size - transformed_CP.y;

    // 뒤집힘 여부에 맞춰 angle 보정 (180도 회전)
    this->camera_up_angle_ = std::abs(this->camera_up_angle_ + 180);
  }

#ifdef SHOW_VIDEO
  imshow("orientation_check", img_ROI);
  std::cout << "flip : " << is_flipped << std::endl;
  waitKey(1);
#endif

  return true;
}

bool ImgPreprocessor::verifyDotCount() {
  // 필요한 ROI 내의 전체 점의 수를 계산
  // subRows, subCols 따라 ROI 내 점의 갯수가 모두 필요하지 않으므로 해당 부분 확인해 느슨한 기준 가능
  int total_dots = (this->ROI_width_dots) * (this->ROI_height_dots);
  // 대시라인으로 인해 사라지는 점의 수를 계산
  int dash_loss = (this->required_baseline_cnt) * 8;

  int nums_of_required_pts = total_dots - dash_loss;

  if (all_detected_dots_.size() < nums_of_required_pts) {
    LBS_error_code = SBD_error;
    return false;
  }
  return true;
}

auto ImgPreprocessor::run(cv::Mat source) -> bool {
  // 이미지 없음
  if (source.empty()) {
    return false;
  }

  // 이미지 전처리
  if (!this->setupPreprocessing(source)) {
    return false;
  };

  DotDetector dotDetector;
  dotDetector.setupParameters(params_);
  dotDetector.detectBySimpleBlobDetector(this->gray, all_detected_dots_);
  drawDetectedDots(this->src, all_detected_dots_);

  // 검출된 점의 수가 필요한 갯수보다 적을 경우, 에러 처리
  if (!this->verifyDotCount()) {
    return false;
  }

  // 이미지 이진화
  this->binarizeImage();

  // 이미지에서 검출된 점들을 지우기 (선 검출에 방해가 될 수 있기 때문)
  eraseDetectedDots(binarized, all_detected_dots_);

  // 에지 검출
  cv::Canny(binarized, edge, 0, 0);

  if (!lineProcessor_.detectBaseLines(edge)) {
    return false;
  }
  clustered_lines = lineProcessor_.getClusteredLines();

  // 검출된 선분 그리기
  this->drawClusteredLines();

  this->findClosestDotToCP();

  this->orderBaseLinesByClosestDot();

  this->findValidLinesByDotsLocation();
  if (!this->findValidLinesByDotsNum()) {
    LBS_error_code = findValidLines_error;
    return false;
  }

  if (!this->validateClosestBaseLinesByDirection()) {
    this->LBS_error_code = validateClosestBaseLines_error;
    return false;
  }

  this->findClosestBaselineDotToNearCP();

  if (!this->findwarpPerspectiveCornerPoint()) {
    return false;
  }
  this->drawDetectedCornerDots();

  if (!this->adjustCornersToPredefinedArea()) {
    return false;
  }

  if (!this->preprocessPointData()) {
    return false;
  }

  if (!this->excludeMarginDots()) {
    return false;
  }

  this->drawTransformedDots();

  if (!this->checkAndCorrectOrientation()) {
    return false;
  }

#ifdef SHOW_VIDEO
  imshow("warpTest", this->warpPerspectived_image);
#endif
  return true;
}
} // namespace LCODE
