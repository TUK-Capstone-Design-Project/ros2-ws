#pragma once
#include "nlohmann/json.hpp"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <opencv2/core.hpp>
#include <string>

using std::string;

namespace LCODE {
class ConfigSettings {
private:
  /// setting 로드 확인 여부
  bool loaded;
  /// LBS Main
  int         subCol;         /// subseq의 가로 점 개수
  int         subRow;         /// subseq의 세로 점 개수
  std::string alpha;          /// alphabet 유형
  int         ROI_width_dot;  /// ROI에 가로 점 개수
  int         ROI_height_dot; /// ROI에 세로 점 개수
  float       EPS;            /// 점들의 오차값 (px)
  float       maxAngle;       /// 이상적 점 위치와 디지타이징된 점의 최대오차 각도 허용 범위 값

  std::string LBS_Config_Path; /// folder path setting

  bool endsWith(const std::string &str, const std::string &suffix);
  auto createConfigurationFile(std::string filePath) -> bool;
  auto readConfigurationFile(std::string path) -> bool;

public:
  ConfigSettings(std::string path = "./");

  auto saveConfigSetting(std::string filePath = "./") -> bool;
  void loadSettings(string path = "./");

  auto getSubRow() const -> std::size_t;
  auto getSubCol() const -> std::size_t;
  auto getROI_width_dot() const -> std::size_t;
  auto getROI_height_dot() const -> std::size_t;
  auto getAlpha() const -> std::string;
  auto getMaxAngle() const -> float;
  auto getEPS() const -> float;
  auto getPagination() const -> bool;
  auto getPageSize() const -> std::pair<int, int>;

  auto setSubRow(int subRow_) -> void;
  auto setSubCol(int subCol_) -> void;
  auto setROI_width_dot(int ROI_width_dot_) -> void;
  auto setROI_height_dot(int ROI_height_dot_) -> void;
  auto setAlpha(std::string alpha_) -> void;
  auto setMaxAngle(float maxAngle_) -> void;
  auto setEPS(float EPS_) -> void;
  auto setPagination(bool pagination_opt) -> void;
  auto setPageSize(std::pair<int, int> Page) -> void;

  ///// LBS Sclae adjustment
  auto getOneGridLength() const -> double;
  auto getDotRadius() const -> double;
  auto getGridCrossToDot() const -> double;
  auto getDashWidthMm() const -> double;
  auto getDashLengthMm() const -> double;

  auto setGridUnitLength(double grid_unit_length_) -> void;
  auto setDotRadius(double dot_radius_) -> void;
  auto setGridCrossToDot(double grid_cross_to_dot_) -> void;
  auto setDashWidth(double dashWidth_mm_) -> void;
  auto setDashLength(double dashLength_mm_) -> void;

  // ImgPreprocessor params
  int line_size                = 900; /// polar_to_cartesian 작업에서 선분 시작 지점과 끝 지점의 사이즈에 영향
  int clusterDistanceTolerance = 40;  /// start, end 지점의 포인트 클러스터 유효 범위 거리 지정.
  // SimpleBlobDetector params
  int minThreshold        = 60;  // opencv default : 50
  int maxThreshold        = 220; // opencv default : 220
  int thresholdStep       = 15;  // opencv default : 20
  int minDistBetweenBlobs = 5;   // opencv default : 10

  bool filterByArea = true;
  int  minArea      = 25;   // opencv default : 25
  int  maxArea      = 5000; // opencv default : 5000

  bool  filterByCircularity = true;
  float min_circularity     = 0.8f;
  float max_circularity     = 1.0f;

  bool  filterByConvexity = false;
  float min_convexity     = 0.8f;
  float max_convexity     = 1.0f;

  bool  filterByInertia = false;
  float min_inertia     = 0.1f;
  float max_inertia     = 1.0f;

  int min_repeatability = 2;
};
} // namespace LCODE