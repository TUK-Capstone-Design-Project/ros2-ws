#include "LineProcessor/LineProcessor.hpp"

namespace LCODE {

auto LineProcessor::detectBaseLines(cv::Mat &img) -> bool {
  std::vector<cv::Vec2f> detected_lines_polar;     // houghlines 함수 통해 추출된라인들 (극좌표계 형태)
  std::vector<Line<int>> detected_lines_cartesian; // 극좌표계 라인을 데카르트 좌표계로 변환한 라인들

  // bool cluster_valid_check = false; // clusterLines함수의 조건을 만족시키고 루프문을 탈출 시킬수있는 bool조건문
  detected_lines_polar.reserve(128);
  detected_lines_cartesian.reserve(128);

  for (int i = 0; i < 2; i++) {
    // 루프 시작시마다 검출된 라인들 초기화
    detected_lines_polar.clear();
    detected_lines_cartesian.clear();
    clustered_lines.clear();

    // Houghlines로 라인 검출, 실패시 루프 재시작
    if (!doHoughLines(img, detected_lines_polar)) {
      continue;
    }
    detected_lines_cartesian = convertLinesPolarToCartesian(img, detected_lines_polar);

    // validateCartesianLineByDots(detected_lines_cartesian);

    // clusterLines(cluster_valid_check, detected_lines_cartesian);
    Line<int>       representative; // cluster 수행중에 대표라인을 담을 Line 구조체 선언
    constexpr float RANGE_TOLERANCE = 1.3f;
    constexpr float DIST_TOLERANCE =
        40; // 선분의 시작점과 끝점이 모두 이 수치 내에 있을 때 같은 군으로 판단하는 임의의 수치
    // 위치 클러스터링
    clusterLinesByDist(detected_lines_cartesian, DIST_TOLERANCE);

    // 클러스터된 선의 갯수가 타겟 갯수 내애 들어오지 않으면 최대 n번 반복
    if (!validateClusterSize()) {
      std::cout << "Cluster size not valid : " << clustered_lines.size() << "\n";
      continue;
    }
    std::vector<Line<int>> candidateLines;
    candidateLines = extractRepresentativeLines(clustered_lines, DIST_TOLERANCE);

    // 기울기로 대표적 선 탐색 선별함.
    auto clusters_by_slope = clusterLinesBySlope(candidateLines, RANGE_TOLERANCE);

    // 클러스터가 비어있지 않은지 안전장치 추가
    clustered_lines = filterLargestCluster(clusters_by_slope);
    // // MAX_LINE_COUNT 보다 클 경우 threshold 증가, 너무 적을 경우 threshold 감소
    if (clustered_lines.size() < required_baseline_cnt_) {
      this->threshold_ -= this->threshold_step_minus_ / 2;
      continue;
    }

    // 조건을 만족하는 clustered_lines가 존재할 경우 루프 탈출
    break;
  }

  return true;
}
// void LineProcessor::validateCartesianLineByDots(std::vector<Line<int>> &polar_to_cartesian_lines) {
//   constexpr double dot_to_line_dist_tolerance = 5.0; // 타입 일치를 위해 double 사용

//   // C++20의 std::erase_if 활용 (C++20 미만일 경우 erase-remove idiom 사용)
//   // 라인 주변에 점이 '하나도 없으면(none_of)' 해당 라인을 제거합니다.
//   std::erase_if(polar_to_cartesian_lines, [&](const Line<int> &line) {
//     return std::none_of(all_detected_dots_.begin(), all_detected_dots_.end(), [&](const auto &dot) {
//       return this->isKeyPointNearLine(dot, line.pt1, line.pt2, dot_to_line_dist_tolerance);
//     });
//   });
// }

auto LineProcessor::convertLinesPolarToCartesian(cv::Mat &src, std::vector<cv::Vec2f> polar_lines)
    -> std::vector<Line<int>> {
  std::vector<Line<int>> converted_lines;
  converted_lines.reserve(polar_lines.size());

  int line_size   = 1000; // temp
  int img_w_h_max = std::max(src.cols, src.rows);

  if (line_size < img_w_h_max) {
    line_size = img_w_h_max * 1.5;
  }

  for (auto &detected_line : polar_lines) {
    float     rho = detected_line[0], theta = detected_line[1];
    cv::Point pt1, pt2;
    double    a = cos(theta), b = sin(theta);
    double    x0 = a * rho, y0 = b * rho;

    pt1.x = cvRound(x0 + line_size * (-b));
    pt1.y = cvRound(y0 + line_size * (a));
    pt2.x = cvRound(x0 - line_size * (-b));
    pt2.y = cvRound(y0 - line_size * (a));
    cv::clipLine(src.size(), pt1, pt2);

#ifdef SHOW_VIDEO
    line(houghlines_raw_img, pt1, pt2, {0, 0, 255}, 2); // test
#endif                                                  //! SHOW_VIDEO
    converted_lines.emplace_back(pt1, pt2);
  }
#ifdef SHOW_VIDEO

  imshow("houghlines_raw_img", houghlines_raw_img);
#endif //! SHOW_VIDEO

  return converted_lines;
}

void LineProcessor::clusterLinesByDist(const std::vector<Line<int>> &lines, int distTolerance) {
  constexpr float range_tolerance   = 1.3f;
  const float     relaxed_tolerance = distTolerance * range_tolerance;

  // 두 선분 간의 좌표가 허용 오차 내에 있는지 확인하는 람다 함수
  auto is_matching = [distTolerance, relaxed_tolerance](const auto &p1_rep, const auto &p2_rep, const auto &p1_line,
                                                        const auto &p2_line) {
    float dx1 = std::abs(static_cast<float>(p1_rep.x - p1_line.x));
    float dy1 = std::abs(static_cast<float>(p1_rep.y - p1_line.y));
    float dx2 = std::abs(static_cast<float>(p2_rep.x - p2_line.x));
    float dy2 = std::abs(static_cast<float>(p2_rep.y - p2_line.y));

    bool pt1_strict  = dx1 < distTolerance && dy1 < distTolerance;
    bool pt2_strict  = dx2 < distTolerance && dy2 < distTolerance;
    bool pt1_relaxed = dx1 < relaxed_tolerance && dy1 < relaxed_tolerance;
    bool pt2_relaxed = dx2 < relaxed_tolerance && dy2 < relaxed_tolerance;

    return (pt1_strict && pt2_relaxed) || (pt1_relaxed && pt2_strict);
  };

  // 위치 클러스터링
  for (const auto &line : lines) {
    // 조건에 맞는 첫 번째 클러스터 탐색
    auto it = std::ranges::find_if(clustered_lines, [&](const auto &cluster) {
      const auto &rep = cluster.front();
      // 정방향(pt1-pt1, pt2-pt2) 또는 역방향(pt2-pt1, pt1-pt2) 매칭 검사
      return is_matching(rep.pt1, rep.pt2, line.pt1, line.pt2) || is_matching(rep.pt2, rep.pt1, line.pt1, line.pt2);
    });

    // 매칭되는 클러스터를 찾았다면 추가, 없다면 새로운 클러스터 생성
    if (it != clustered_lines.end()) {
      it->push_back(line);
    } else {
      clustered_lines.push_back({line});
    }
  }
}

bool LineProcessor::validateClusterSize() {
  if (clustered_lines.size() >= MAX_LINE_COUNT) {
    this->threshold_ += this->threshold_step_plus_;
    return false;
  } else if (clustered_lines.size() < required_baseline_cnt_) {
    if (clustered_lines.empty()) {
      this->threshold_ -= static_cast<int>(this->threshold_step_minus_ * (1.2));
    }
    this->threshold_ -= this->threshold_step_minus_;
    return false;
  };
  return true;
}

auto LineProcessor::extractRepresentativeLines(const std::vector<std::vector<Line<int>>> &clustered_lines,
                                               int DIST_TOLERANCE) -> std::vector<Line<int>> {
  std::vector<Line<int>> candidateLines;
  candidateLines.reserve(clustered_lines.size()); // 메모리 재할당 방지 (최적화)

  // 두 점이 허용 오차 내에 있는지 확인하는 람다 함수
  auto is_close = [DIST_TOLERANCE](const auto &p1, const auto &p2) {
    return std::abs(p1.x - p2.x) <= DIST_TOLERANCE && std::abs(p1.y - p2.y) <= DIST_TOLERANCE;
  };

  // 두 선이 겹치는지(같은 선으로 취급할지) 확인하는 람다 함수
  auto is_overlap = [&is_close](const Line<int> &l1, const Line<int> &l2) {
    bool condition_one = is_close(l1.pt1, l2.pt1) && is_close(l1.pt2, l2.pt2);
    bool condition_two = is_close(l1.pt1, l2.pt2) && is_close(l1.pt2, l2.pt1); // Swap 로직 수정됨
    return condition_one || condition_two;
  };

  for (const auto &cluster : clustered_lines) {
    if (cluster.empty())
      continue; // 빈 클러스터 예외 처리

    if (cluster.size() == 1) {
      candidateLines.push_back(cluster.front());
      continue;
    }

    // 클러스터 내에서 가장 오버랩이 많은 선 찾기
    size_t best_idx    = 0;
    int    max_overlap = -1;

    for (size_t i = 0; i < cluster.size(); ++i) {
      // C++20 std::ranges::count_if를 사용하여 겹치는 선의 개수를 셈
      int overlap = std::ranges::count_if(cluster, [&](const Line<int> &other) {
        // 메모리 주소가 다를 때만 비교 (자기 자신 제외)
        return (&cluster[i] != &other) && is_overlap(cluster[i], other);
      });

      if (overlap > max_overlap) {
        max_overlap = overlap;
        best_idx    = i;
      }
    }

    // 기존처럼 -1,-1 초기화 검사를 할 필요 없이,
    // 오버랩이 0이어도(모두 떨어져 있어도) 첫 번째 선(best_idx = 0)이 안전하게 선택됨
    candidateLines.push_back(cluster[best_idx]);
  }

  return candidateLines;
}

auto isSimilarSlope(double slope1, double slope2, double threshold) -> bool {
  if (std::isinf(slope1) && std::isinf(slope2))
    return true;
  return std::abs(slope1 - slope2) < threshold;
}
auto LineProcessor::clusterLinesBySlope(const std::vector<Line<int>> &candidateLines,
                                        double                        slope_threshold)
    -> std::vector<std::vector<Line<int>>> // Radian 단위, 기본값 17.1도
{
  std::vector<std::vector<Line<int>>> clusters_slope;

  for (const auto &original_line : candidateLines) {
    Line<int> line = original_line; // 스왑(swap)될 수 있으므로 복사본 생성

    double normal_slope = line.calculateSlope();

    Line<int> reversed_line  = {line.pt2, line.pt1};
    double    reversed_slope = reversed_line.calculateSlope();

    // 1. 정방향(Normal) 검사: 모든 클러스터를 대상으로 먼저 탐색
    auto normal_it = std::ranges::find_if(clusters_slope, [&](const auto &cluster) {
      return isSimilarSlope(normal_slope, cluster.front().calculateSlope(), slope_threshold);
    });

    if (normal_it != clusters_slope.end()) {
      normal_it->push_back(line);
      continue; // 추가 성공 시 다음 라인으로 넘어감
    }

    // 2. 역방향(Reversed) 검사: 정방향 일치가 없을 경우에만 탐색
    auto reversed_it = std::ranges::find_if(clusters_slope, [&](const auto &cluster) {
      return isSimilarSlope(reversed_slope, cluster.front().calculateSlope(), slope_threshold);
    });

    if (reversed_it != clusters_slope.end()) {
      std::swap(line.pt1, line.pt2); // 역방향이 일치하므로 포인트를 뒤집어서 저장
      reversed_it->push_back(line);
      continue;
    }

    // 3. 정방향/역방향 모두 일치하는 클러스터가 없으면 새로운 군으로 인정
    clusters_slope.push_back({line});
  }

  return clusters_slope;
}

auto LineProcessor::filterLargestCluster(const std::vector<std::vector<Line<int>>> &clusters)
    -> std::vector<std::vector<Line<int>>> {
  if (clusters.empty()) {
    return {}; // 빈 경우 빈 벡터 반환
  }

  // 1. 가장 큰 클러스터 찾기 (C++20 ranges & 투영)
  auto largest_it = std::ranges::max_element(clusters, std::less<>{}, &std::vector<Line<int>>::size);

  // 2. 결과 생성 및 반환
  std::vector<std::vector<Line<int>>> result;
  result.reserve(largest_it->size());

  for (const auto &line : *largest_it) {
    result.push_back({line});
  }

  return result; // RVO(Return Value Optimization)로 인해 복사 비용 없이 효율적으로 반환됨
}
auto LineProcessor::getClusteredLines() -> std::vector<std::vector<Line<int>>> {
  return this->clustered_lines;
}

auto LineProcessor::clusterLines(bool &cluster_valid_check, std::vector<Line<int>> &polar_to_cartesian_lines) -> bool {
  Line<int>       representative; // cluster 수행중에 대표라인을 담을 Line 구조체 선언
  constexpr float RANGE_TOLERANCE = 1.3f;
  constexpr float DIST_TOLERANCE =
      40; // 선분의 시작점과 끝점이 모두 이 수치 내에 있을 때 같은 군으로 판단하는 임의의 수치
  // 위치 클러스터링

  clusterLinesByDist(polar_to_cartesian_lines, DIST_TOLERANCE);

  // 클러스터된 선의 갯수가 타겟 갯수 내애 들어오지 않으면 최대 n번 반복
  if (!validateClusterSize()) {
    std::cout << "Cluster size not valid : " << clustered_lines.size() << "\n";
    return false;
  }

  std::vector<Line<int>> candidateLines;
  candidateLines = extractRepresentativeLines(clustered_lines, DIST_TOLERANCE);

  // 기울기로 대표적 선 탐색 선별함.
  auto clusters_by_slope = clusterLinesBySlope(candidateLines, RANGE_TOLERANCE);

  // 클러스터가 비어있지 않은지 안전장치 추가
  clustered_lines = filterLargestCluster(clusters_by_slope);

  // // MAX_LINE_COUNT 보다 클 경우 threshold 증가, 너무 적을 경우 threshold 감소
  if (clustered_lines.size() < required_baseline_cnt_) {
    this->threshold_ -= this->threshold_step_minus_ / 2;
    return false;
  }

  cluster_valid_check = true;
  return true;
}

auto LineProcessor::doHoughLines(cv::Mat &src, std::vector<cv::Vec2f> &detected_polar_lines) -> bool {
  // 허프라인의 오류 발생시 return;
  try {
    cv::HoughLines(src, detected_polar_lines, this->Hough_rho_, CV_PI / 180, 100);
  } catch (...) { return false; }
  return true;
}

} // namespace LCODE