#pragma once
// opencv
#include "Common/Line.h"
#include "ConfigSettings/ConfigSettings.hpp"
#include <opencv2/opencv.hpp>

namespace LCODE {
class LineProcessor {
public:
  LineProcessor(ConfigSettings &config) {
    // 전달받은 config로 즉시 초기화
    this->threshold_            = config.minThreshold;
    this->threshold_step_plus_  = config.thresholdStep;
    this->threshold_step_minus_ = config.thresholdStep;

    this->required_baseline_cnt_ = config.getROI_width_dot() / 6 + 1;
  }

  auto detectBaseLines(cv::Mat &img) -> bool;
  auto getClusteredLines() -> std::vector<std::vector<Line<int>>>;

private:
  std::vector<std::vector<Line<int>>> clustered_lines;
  auto                 detectBaseLine_Hough(cv::Mat &src, std::vector<cv::Vec2f> &detected_lines_polar) -> bool;
  static constexpr int MAX_LINE_COUNT = 13; // 너무 많은 라인이 검출 될 경우를 제외 시키기 위한 임의의 수치

  float Hough_rho_             = 1.1;                  ///	cv::Houghlines rho 파라미터
  int   saved_threshold_val_   = 200;                  /// threshold 기본값 저장
  int   threshold_             = saved_threshold_val_; ///	cv::Houghlines threshold
  int   threshold_step_plus_   = 15;                   ///	cv::Houghlines 재 호출 threshold 증가값
  int   threshold_step_minus_  = 15;                   /// cv::Houghlines 재 호출 threshold 감소값
  int   required_baseline_cnt_ = 3; // ROI_width_dots 13 => 3   19 => 4  6증가 할때, 1씩 증가 , 기본값 3

  auto doHoughLines(cv::Mat &src, std::vector<cv::Vec2f> &detected_lines_polar) -> bool;

  auto convertLinesPolarToCartesian(cv::Mat &src, std::vector<cv::Vec2f> polar_lines) -> std::vector<Line<int>>;

  void validateCartesianLineByDots(std::vector<Line<int>> &detected_lines_cartesian);

  auto clusterLines(bool &cluster_valid_check, std::vector<Line<int>> &polar_to_cartesian_lines) -> bool;

  void clusterLinesByDist(const std::vector<Line<int>> &lines, int distTolerance = 40);

  bool validateClusterSize();

  auto filterLargestCluster(const std::vector<std::vector<Line<int>>> &clusters) -> std::vector<std::vector<Line<int>>>;

  auto clusterLinesBySlope(const std::vector<Line<int>> &candidateLines, double slope_threshold = 0.3)
      -> std::vector<std::vector<Line<int>>>;

  auto extractRepresentativeLines(const std::vector<std::vector<Line<int>>> &clustered_lines, int DIST_TOLERANCE)
      -> std::vector<Line<int>>;

  // void validateCartesianLineByDots(std::vector<Line<int>> &polar_to_cartesian_lines);
};
}; // namespace LCODE