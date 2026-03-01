#pragma once
#include "ConfigSettings/ConfigSettings.hpp"
#include "ImgPreprocessor/ImgPreprocessor.hpp"
#include "hash_pair.h"
#include <algorithm>
#include <iostream>
#include <opencv2/core/types.hpp>
#include <unordered_map>

using namespace std;

namespace LCODE {
class Digitizer {
private:
  enum DOTTYPE {
    CENTER = -1, /// 가운데 점
    LEFT,        /// 왼쪽 점
    TOP,         /// 위쪽 점
    RIGHT,       /// 오른쪽 점
    BOTTOM,      /// 아래쪽 점
    LEFT_TOP,
    RIGHT_TOP,
    RIGHT_BOTTOM,
    LEFT_BOTTOM,
  };
  int   entireRow; /// ROI에 세로 점 개수
  int   entireCol; /// ROI에 가로 점 개수
  int   subRow;    /// 서브셋 Row
  int   subCol;    /// 서브셋 Col
  float EPS;       /// 점들의 오차값 (px)
  float MaxAngle;  /// 최대오차값 허용 범위

  /// 2차원상의 두 좌표의 거리를 구하는 함수
  float GetDistance(float x1, float y1, float x2, float y2) {
    return cv::norm(cv::Point2f{x1, y1} - cv::Point2f{x2, y2});
  }

  /// @brief 1차원 배열로 들어온 KeyPoint값을 2차원배열로 정렬
  /// @param dotsInfo 모든 점들의 정보 (input)
  /// @param devidedWidth 점과 점 사이의 거리 (가로)(input)
  /// @param devidedHeight 점과 점 사이의 거리 (세로)(input)
  /// @param outputDotsInfo 정렬된 값을 받을 변수 (output)
  void Sort2DdotArr(vector<cv::Point2f> *dotsInfo, float devidedWidth, float devidedHeight,
                    unordered_map<pair<int, int>, cv::Point2f, pair_hash> *outputDotsInfo);

  /// @brief Dash로 인해 사라진 점을 위끝과 아래끝 점의 좌표를 이용해 보간하는 함수
  /// @param sortedEntiredots 2차원 배열로 정렬된 점들 (input & output)
  /// @param devidedHeight 점과 점 사이의 거리 (세로)(input)
  /// @param baselineDots 기준선 위의 점들 (input)
  void InterpolatingDashArea(unordered_map<pair<int, int>, Point2f, pair_hash> *sortedEntiredots, float devidedHeight,
                             vector<Point2f> &baselineDots);

  /// @brief CP의 Index값을 찾는 함수
  /// @param sortedEntiredots 정렬된 전체 점들 (input)
  /// @param CP CP좌표 (input)
  /// @param CP_dotIndex CP index (output)
  void FindCP_Index(unordered_map<pair<int, int>, Point2f, pair_hash> *sortedEntiredots, Point2f CP,
                    pair<int, int> *CP_dotIndex);

  /// @brief 2차원으로 정렬된 배열에서 CP주변으로 COL x ROW 영역을 추출하는 함수 (COL = 6, ROW = 8)
  /// @param sortedEntiredots 2차원 배열로 정렬된 점들 (input)
  /// @param CP_dotIndex CP index (input)
  /// @param subset COL x ROW 서브셋 2차원 배열(output)
  /// @param subRect COL x ROW 서브셋의 index 범위(output)
  /// @return 서브셋이 추출되지 않으면 false, 추출되면 true
  bool ExtractSubset(unordered_map<pair<int, int>, cv::Point2f, pair_hash> *sortedEntiredots,
                     pair<int, int> CP_dotIndex, unordered_map<pair<int, int>, cv::Point2f, pair_hash> *subset,
                     pair<cv::Point2i, cv::Point2i> *subRect);

  /// @brief 원래 그리드의 교점을 찾는 함수
  /// @param subRect COL x ROW 서브셋의 index 범위(input)
  /// @param devidedWidth 점과 점 사이의 거리 (가로)(input)
  /// @param devidedHeight 점과 점 사이의 거리 (세로)(input)
  /// @param crossPoints 교점좌표 (output)
  void FindGridCrossPoint(pair<cv::Point2i, cv::Point2i> *subRect, float devidedWidth, float devidedHeight,
                          unordered_map<pair<int, int>, cv::Point2f, pair_hash> *crossPoints);

  /// @brief 서브셋 배열에서 기준선의 인덱스 값과 x값을 탐색함
  /// @param subset 전체 배열에서 COL x ROW로 추출된 배열 (input)
  /// @param baselineDots 기준선 위의 점들 좌표 배열(input)
  /// @param baselineIndex 서브셋에서 기준선이 가지는 X index 값 (output)
  /// @param baselineX 기준선의 x좌표 값 (output)
  void FindBaselineInSubset(unordered_map<pair<int, int>, cv::Point2f, pair_hash> *subset,
                            vector<cv::Point2f> &baselineDots, vector<int> *baselineIndex, vector<float> *baselineX);

  /// @brief subset 점들이 grid교점으로부터 상하좌우에 있는지 판단하여 디지털화 함.
  /// @param crossPoints grid 교점들을 담고 있는 배열 (input)
  /// @param subset subCol x subRow 크기로 추출된 서브셋 점들 (input)
  /// @param baselineIndex 서브셋에서 기준선이 가지는 X index 값 (input)
  /// @param digitizedOutput 디지털화된 2차원 배열 (output)
  /// @return digitizing 성공 여부
  bool digitize(unordered_map<pair<int, int>, cv::Point2f, pair_hash> *crossPoints,
                unordered_map<pair<int, int>, Point2f, pair_hash> *subset, vector<int> baselineIndex);

public:
  Digitizer(ConfigSettings &config) {
    constexpr int required_cols_num = 1;
    this->entireCol                 = config.getROI_width_dot();
    this->entireRow                 = config.getROI_height_dot();
    this->subRow                    = config.getSubRow();
    this->subCol                    = config.getSubCol() + required_cols_num;
    this->EPS                       = config.getEPS();
    this->MaxAngle                  = config.getMaxAngle();
  }

  vector<vector<int>> digitizied_array; /// digitizedOutput 디지털화된 2차원 배열 (output)
  pair<int, int>      movementIndex;    /// movementIndex 나중 최종 결과값에서 보정해야할 값(output)

  /// @brief 디지털화 과정을 하나로 합쳐둔 함수 (include시 이 함수만 실행)
  /// @param entireDotsInfo 모든 점들의 정보 (input)
  /// @param CP CenterPoint (input)
  /// @param baselineDots 기준선 위의 점들 (input)
  /// @param ROIwidth 이미지 가로길이 (input)
  /// @param ROIheight 이미지 세로길이 (input)
  /// @return 디지털화 실패시 false, 성공시 true
  bool runDigitize(vector<cv::Point2f> &entireDotsInfo, cv::Point2f CP, vector<vector<cv::Point2f>> &baselineDots,
                   int ROIwidth, int ROIheight);
  bool runDigitize(ImgPreprocessor &ImgPre);
};
} // namespace LCODE