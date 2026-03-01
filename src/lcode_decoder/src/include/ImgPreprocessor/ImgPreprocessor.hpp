#pragma once

// I/O
#include <format>
#include <iostream>

// STL
#include <algorithm>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

// opencv
#include <opencv2/opencv.hpp>

// mine
#include "Common/Line.h"
#include "ConfigSettings/ConfigSettings.hpp"
#include "DotDetector/DotDetector.hpp"
#include "LineProcessor/LineProcessor.hpp"

using namespace std;
using namespace cv;

namespace LCODE {
/**
 * @brief 카메라로 찍힌 이미지를 받아 디지타이징 형식에 맞게 점의 정보를 가공한다.
 */
class ImgPreprocessor {
public:
  /**
   * @brief ImgPreprocessor 생성자
   * @param config : LBS::ConfigSettings, config 세팅 클래스
   * @details ROI_width_dots_ : std::size_t, ROI(Region Of Interest)영역의 가로 점 갯수
   * @details ROI_height_dots_ : std::size_t, ROI(Region Of Interest)영역의 세로 점 갯수
   * @details required_baseline_cnt : int, 필요한 선의 갯수 지정 , 일반적으로 ROI_height_dots의 값의 비율을 따라가나,
   * 임의적로 ROI_height_dots의 값보다 작게 설정가능 (다만 7 이상일것)
   * @details expasion_point_num : int, 확장 기준점으로부터 각 양쪽 방향의 n번 너머의 점을 warpPerspective의 코너점으로
   * 사용
   */
  ImgPreprocessor(ConfigSettings &ConfigSetter);

  /**
   * @brief preprocessor process flow
   * @param source : cv::Mat, 전처리를 할 이미지 할당
   */
  auto run(Mat source) -> bool;

  // 보는 각도 위로 angle 확인
  float camera_up_angle_;

  // 디지타이징 과정에서 필요한 변수들
  float ROI_width_size;  // ROI 영역의 가로 길이
  float ROI_height_size; // ROI 영역의 세로 길이

  std::vector<cv::Point2f>              transformed_points;           /// warpPerspective 변환된 ROI 영역의 Dot 포인트들
  cv::Point2f                           transformed_CP;               /// warpPerspective ROI 영역의 cross_point의 위치
  std::vector<std::vector<cv::Point2f>> middle_baseLines_dots_in_ROI; /// ROI 영역 내부의 기준선들 위에 있는 점들을 담음

private:
  LineProcessor lineProcessor_;
  cv::Mat       src;       ///  정보를 추출할 원본 이미지, 이곳에 스마트폰, 혹은 사진파일을 할당후 전처리 시작
  cv::Mat       gray;      /// 그레이스케일
  cv::Mat       binarized; /// 이진화된 이미지
  cv::Mat       edge;      /// Canny로 추출된 엣지 이미지

  cv::Mat warp_source_orient;     ///  orient 영역을 위한 원본 소스 저장 (for orient Area)
  cv::Mat warpsource;             ///  warpPerspective 결과 이미지를 위한 origin source 저장.
  cv::Mat warpPerspectived_image; ///  warpPerspective 한 이미지 담을 객체

  cv::Point2f img_cross_point; // 이미지 정 중앙 x,y위치

  std::vector<cv::Point2f>
      middle_baseLine_dots_ROI; /// warpPerspective 변환된 이미지에서 가운데 기준선 라인의 Dot을 담음

  int LBS_error_code = -1;
  /// @brief LBS 에러 메세지 확인
  auto getErrorMessage() -> String;

  int ROI_width_dots;
  int ROI_height_dots;
  int expansion_point_num;   /// ROI_height_dots 13 => 2   19 => 3  6증가 할때, 1씩 증가
  int required_baseline_cnt; /// ROI_width_dots  13 => 3   19 => 4  6증가 할때, 1씩 증가

  /**
   * @brief 라인의 시작점과 끝점을 담을 구조체 선언
   */
  //   struct Line {
  //     cv::Point pt1, pt2;
  //   };

  // Houghlines
  float Hough_rho            = 1.1;                 ///	cv::Houghlines rho 파라미터
  int   saved_threshold_val  = 200;                 /// threshold 기본값 저장
  int   threshold            = saved_threshold_val; ///	cv::Houghlines threshold
  int   threshold_step_plus  = 15;                  ///	cv::Houghlines 재 호출 threshold 증가값
  int   threshold_step_minus = 15;                  /// cv::Houghlines 재 호출 threshold 감소값
  // SimpleBlobDetector Params

  void                           setupSimpleBlobParameters();
  cv::SimpleBlobDetector::Params params_;
  bool                           filterByArea_;
  bool                           filterByCircularity_;
  float                          min_circularity_;
  float                          max_circularity_;
  bool                           filterByConvexity_;
  float                          min_convexity_;
  float                          max_convexity_;
  bool                           filterByInertia_;
  float                          min_inertia_;
  float                          max_inertia_;
  int                            min_repeatability_;
  int                            minThreshold_;
  int                            maxThreshold_;
  int                            thresholdStep_;
  int                            minArea_;
  int                            maxArea_;
  int                            minDistBetweenBlobs_;

  std::vector<cv::KeyPoint>           all_detected_dots_; /// SBD를 통해 검출된 모든 점들의 좌표를 담음
  std::vector<std::vector<Line<int>>> clustered_lines;    /// 클러스터 완료된 라인들; clustered_lines
  std::vector<std::vector<Line<int>>> closestLines;       /// closest_dot_to_image_center랑 가장 가까운 n개의 기준선

  bool is_crosshair_on_baseline = false; /// crosshair가 baseline의 위에 위치하는지 확인.

  std::vector<tuple<float, double, float>> lines_dist_radian_idx; /// image_center_point으로부터 기준선들의 정보를 담음
  std::vector<vector<tuple<float, double, float>>>
      lines_dist_radian_idx_representative; /// 라디안의 유사성을 기준으로 라인을 나눔

  cv::KeyPoint closest_dot_to_cross_point; /// img_cross_point 제일 가까운 Dot을 선정
  cv::KeyPoint closest_baseline_dot;       /// closest_dot_to_cross_point 가장 가까운 baselines 위에 있는 Dot

  std::vector<std::vector<cv::KeyPoint>>
      nth_closest_keypoints_on_baseline; /// closetstLine의 순서대로 baseline 위에 있는 점들을 담은 객체
  std::vector<std::tuple<cv::Point2f, Line<int>, int>>
      side_line_info; /// 양쪽 side를 담당할 baseline들의 Line과 closestLiens객체 내부의 idx 위치를 담은 객체
  std::vector<cv::KeyPoint>
      two_Corner_dot_on_side_Line1; /// warpPerspective를 위한 각 양쪽 closest_line의 n개의 코너점 추출
  std::vector<cv::KeyPoint>
      two_Corner_dot_on_side_Line2; /// warpPerspective를 위한 각 양쪽 closest_line의 n개의 코너점 추출

  std::vector<cv::Point2f> dst_quad; /// warpPerspective를 위해 선별된 코너 4점(원본 이미지위의 x,y좌표)
  std::vector<cv::Point2f> src_quad; /// warpPerspg ective를 위해 미리 정해둔 영역의 스케일 (정사각형 혹은 직사각형)

  std::vector<cv::Point>
      warpPerspectiveArea; /// warpPerspective 변환할 이미지에서 그 안에 내포될 Dot들을 선별하기 위한 프레임 영역
  std::vector<cv::Point2f> point_to_transform; /// warpPerspective를 위한 source 포인트들 -> isPointInsidePolygon을 통해
                                               /// detected_all_keypoints을 필터링.

  /**
   * @brief 이미지 이진화(적응형 사용) ,
   * @details
   * 1.이미지에 적응형 이진화 적용 \n
   * 2.SBD로 검출된 점 지움 \n
   * 3.Canny openCV API로 edge 검출
   */
  void binarizeImage();

  bool verifyDotCount();
  /**
   * @brief 이미지 세팅
   * @details setup for ImgPrerocess
   * @param Mat& 받아온 이미지 소스 할당
   */
  bool setupPreprocessing(Mat &source);

  /**
   * @brief SimpleBlobDetector 이용, 점 검출
   * @details OpenCV API SimpleBlobDetector 이용, 점 검출
   * @return 점 검출 성공 여부
   */
  auto DetectDotsBySimpleBlobDetector() -> bool;

  /**
   * @brief DetectDotsBySimpleBlobDetector로 검출된 점을 src에서 지움
   */
  auto eraseDetectedDots(cv::Mat &src, std::vector<cv::KeyPoint> &dots) -> void;

  /**
   * @brief DetectDotsBySimpleBlobDetector로 검출된 점 src에 그림
   */
  void drawDetectedDots(cv::Mat &img, vector<cv::KeyPoint> &keypoints);

  /// </summary>
  /**
   * @brief 선 검출 파트 제어 함수
   * @details
   * 조건에 따라 \n
   * detectBaseLine_hough, convertPolarToCartesian, clusterLines \n
   * 세 함수의 반복 호출을 제어하는 함수
   * @return 선 검출 성공 여부
   */
  //   bool findBaseLines();

  /** @brief houghlines를 이용해 baseline의 후보군들을 추출.
   *  @details OpenCV API Houghlines 이용해 선 검출
   *  @param Mat&  source  선을 검출할 이미지 소스
   *  @param std::vector<cv::Vec2f>&  Houghlines로 검출된 선을 담을 변수
   * @return Houghlines 선 검출 성공 여부
   */
  //   bool detectBaseLine_Hough(cv::Mat &img, std::vector<cv::Vec2f> &detected_polar_lines);

  /** @brief Houghlines로 검출된 polar 좌표계의 라인들 x,y좌표평면계로 전환후 polar_to_cartesian_lines에 전환된 라인들을
   *넣음.
   *  @param detected_polar_lines  houghlines를 통해 검출된 극좌표계 상의 선들
   *	@param polar_to_cartesian_lines  극좌표계 -> 데카르트 좌표계로 전환된 선들
   *  @return 선 전환 성공 여부
   */
  //   bool convertPolarToCartesian(cv::Mat &src, std::vector<cv::Vec2f> detected_polar_lines,
  //                                std::vector<Line<int>> &polar_to_cartesian_lines);

  /** @brief converPolarToCartesian으로 전환된 선들을 점으로 유효성 검사
   *  @param polar_to_cartesian_lines 극좌표계 -> 데카르트 좌표계로 전환된 선들
   */
  //   void validateCartesianLineByDots(std::vector<Line<int>> &polar_to_cartesian_lines);

  /**
   * @brief 검출된 라인 클러스터링 수행
   * @details  1. 시작점과 끝점을 기반, 위치가 비슷한 선끼리 클러스터링 \n 2. test \n 3. test2
   * @param cluster_valid_check : bool, 클러스터링 된 함수들 유효성 확인
   * @param polar_to_cartesian_lines : std::vector<Line>, 극좌표계 -> 데카르트 좌표계로 전환된 선들
   * @return 클러스터링 수행 성공 여부
   */
  //   bool clusterLines(bool &cluster_valid_check, std::vector<Line<int>> &polar_to_cartesian_lines);

  /**
   * @brief 클러스터링 수행이 끝난 라인 그림
   */
  void drawClusteredLines();

  ///
  /**
   * @brief imgs_CrossPoint(CP)에서 가장 가까운 점 찾음
   * @details detected_all_dots에서 imgs_center_point 기준, 가장 가까운 점을 \n
   * closest_dot_to_cross_point 할당.
   */
  void findClosestDotToCP();

  /**
   * @brief closest_dot_to_image_center을 기준, 라인 정렬
   * @details
   * 1. closest_dot_to_image_center - 해당라인 최단 거리 지점 pair -> line_to_dot_distances 삽입 \n
   * 2. 내림차순 정렬 \n
   * 3. 정렬된 순서대로 closestLines에 삽입
   */
  void orderBaseLinesByClosestDot();

  /// closestLInes를 가져와 isKeyPointNearLine 함수를 활용해 유효성 검사

  /**
   * @brief 클러스터 된 선 유효성 검사
   * @details isKeyPointNearLine 함수 이용 \n
   * 선위의 점의 갯수로 라인 필터링
   * @return 필터링된 라인의 적합성
   */
  bool findValidLinesByDotsLocation();

  /// @brief 클러스터 된 선을 점의 갯수로 유효성 검사
  /// @return 성공 여부
  bool findValidLinesByDotsNum();

  /**
   * @brief 클러스터 된 선의 형태 유효성검사
   * @details imgs_cross_point가 \n
   * 1. required_baseline에 따라 img_cp위치 확인 \n
   * 2. img_cp dash_라인 위 위치 확인 \n
   * @return 라인 적합성
   */
  bool validateClosestBaseLinesByDirection();

  /**
   * @brief 선분과 점 사이의 최단 거리에 있는 점을 계산하는 함수
   * @param line_start : cv::Point2f, 선분의 시작점
   * @param line_end : cv::Point2f,  선분의 끝점
   * @param P0 : cv::Point2f, 거리 비교할 점
   * @return : cv::Poin2f, 선분 위의 점 반환
   */
  cv::Point2f getClosestPointOnLine(const cv::Point2f &line_start, const cv::Point2f &line_end, const cv::Point2f &P0);

  /// 가장 가까운 기준선 위에있는 점을 찾음.
  /**
   * @brief closest_img_cp와 가장 가까운 기준선 위에 있는 점 찾음
   * @details closest_img_cp와 가장 가까운 기준선 위에 있는 점 찾아 \n
   * closest_baseline_dot 에 할당
   */
  void findClosestBaselineDotToNearCP();

  /**
   * @brief 해당 선분 위에 점이 있는지 검사
   * @param keypoint : cv::KeyPoint, 대상 점
   * @param lineStart : cv::Point2f, 선분 시작 점
   * @param lineEnd : cv::Point2f,  선분 끝 점
   * @param maxDistance : double, 해당 라인 위 존재 인정 거리 허용값
   * @return 라인 위 존재 여부
   */
  bool isKeyPointNearLine(const cv::KeyPoint &keypoint, const cv::Point2f &lineStart, const cv::Point2f &lineEnd,
                          double maxDistance);

  /**
   * @brief 점과 라인 사이의 거리를 리턴하는 함수
   * @param pt : cv::Point, 검사할 대상 점
   * @param lineStart : cv::Point, 선분 시작점
   * @param lineEnd : cv::Point, 선분 끝점
   * @return 선분과 점 사이 거리
   */
  double distanceFromPointToLine(cv::Point pt, cv::Point lineStart, cv::Point lineEnd);

  /**
   * @brief warpPerpective를 위한 4개의 코너점 후보 선별
   * @details
   * @param lineStart : cv::Point2f, 사이드 라인 시작점
   * @param lineEnd : cv::Point2f, 사이드 라인 끝점
   * @param startDot : cv::Point2f, 사이드라인 대칭점
   * @param dots_on_line : std::vector<KeyPoint>, 사이드라인 선위의 점들
   * @param start_expansion : int, 선분 start방향의 확장점
   * @param end_expansion : int, 선분 end방향의 확장점
   * @param dots_to_start : std::vector<KeyPoint>, start방향의 점들
   * @param dots_to_end : std::vector<KeyPoint>, end방향의 점들
   * @return 유효성 판단 여부
   */
  bool findCornerDotCandidates(const cv::Point2f &lineStart, const cv::Point2f &lineEnd, const cv::Point2f &startDot,
                               const std::vector<cv::KeyPoint> &dots_on_line, int &start_expansion, int &end_expansion,
                               std::vector<cv::KeyPoint> &dots_to_start, std::vector<cv::KeyPoint> &dots_to_end);

  /**
   * @brief 선별된 코너점을 원하는 위치로 정렬
   * @return 코너점 유효성 여부
   */
  bool findwarpPerspectiveCornerPoint();

  /**
   * @brief 기울기 도출함수
   * @return 기울기 도출
   */
  double calculateSlope(const Line<int> &line);

  /**
   * @brief 두 기울기가 비슷한지 검사
   */
  bool isSimilarSlope(double slope1, double slope2, double threshold);

  /**
   * @brief 4개의 코너점이 사각형인지 검사
   * @param p1 : cv::Point, p1
   * @param p2 : cv::Point, p2
   * @param p3 : cv::Point, p3
   * @param p4 : cv::Point, p4
   * @param margin : double, 사각형의 코너점의 유효각도
   * @return 사각형 여부 판단
   */
  bool isRectangleWithMargin(cv::Point p1, cv::Point p2, cv::Point p3, cv::Point p4, double margin);

  /**
   * @brief WarpPerspective를 위한 코너점들을 Dst_quad에 알맞게 정렬, 삽입함.
   * @param warp_point_candidate : std::vector<cv::Point2f>, 4개의 원근변환 후보점
   */
  auto arrangeDstQuad(const std::vector<cv::Point2f> &warp_point_candidate);

  /// @brief 추출한 코너점들을 원하는 위치에 알맞게 정렬.
  bool adjustCornersToPredefinedArea();

  /// @brief 코너점을 그림
  void drawDetectedCornerDots();

  /**
   * @brief 두개의 라디안 값이 서로 비슷한지 비교
   * @param angle1 : double, angle1
   * @param angle2 : double, angle2
   * @param tolerance : dobule, tolerance
   * @return 유사성 여부
   */
  bool isAngleSimilar(double angle1, double angle2, double tolerance = 0.3);

  /**
   * @brief warpPerspective로 전환된 영역의 점들을 warpPerspectived_image에 그림
   */
  void drawTransformedDots();

  /**
   * @brief cross point를 src에 그림
   */
  void drawCPOnImg();

  /**
   * @brief 도출된 포인트 데이터들을 전처리
   * @return 성공 여부
   */
  bool preprocessPointData();
  //

  /**
   * @brief 4개의 코너 점으로 polygon 영역 형성, 영역 내부에 해당 점 존재 검사
   * @param polygon : std::vector<cv::Point>, warpPerspective Area의 선별된 코너 점 4개
   * @param point : cv::Point, 검사 대상 점
   * @return 영역 내 점 존재 여부
   */
  bool isPointInsidePolygon(const std::vector<cv::Point> &polygon, const cv::Point &point);

  /**
   * @brief 도출된 포인트 데이터들이 테두리에 가깝게 있으면 제거
   * @return 성공 여부
   */
  bool excludeMarginDots();

  /**
   * @brief 선과 선사이 지정 거리 내 있는 선을 병합
   * @param lines : vector<Vec4i>, houghlinesP로 도출된 선
   * @param maxDistance : float, 동일 직선 인정 거리
   * @return 병합된 라인들
   */
  vector<Vec4i> mergeCloseLines(const vector<Vec4i> &lines, float maxDistance);

  /**
   * @brief 시작점과 끝점을 동일한 방향으로 균일화
   * @param lines : vector<Vec4i>, houghlinesP로 도출된 선
   */
  void normalizeLinePoints(vector<Vec4i> &lines);

  /**
   * @brief 이미지 위아래 방향 보정을 위한 HoughlinesP 파라미터 세팅과 실행
   * @param Img_ROI 이미지 가운데 영역만 추출된 이미지
   * @param liens 이미지 가운데 영역에서 추출된 선들의 정보
   */
  void setHoughPAndFindOrientLines(cv::Mat &img_ROI, std::vector<Vec4i> &lines);

  /**
   * @brief 이미지 뒤집힘 판단 여부 확인 후 보정 진행
   * @return 프로세스 성공 여부
   */
  bool checkAndCorrectOrientation();

  enum {
    SBD_error = 1,
    findBaselines_error,
    findValidLines_error,
    validateClosestBaseLines_error,
    findwarpPerspectiveCornerPoint_fatal_error,
    findwarpPerspectiveCornerPoint_sidelines_error,
    adjustCornersToPredefinedArea_Transform_error,
    adjustCornersToPredefinedArea_shape_error,
    preprocessPointData_lackPts_error,
    preprocessPointData_noMidBaseLine_dots_error,
    preprocessPointData_error,
    preprocessPointData_transform_error,
    excludeMarginDots_lackMidPts_error,
    orienting_LineNums_error,
    orienting_line_max_too_big_error,
    orienting_cantKnowLineDiff_error,
    orienting_lackLineNums_error,
  };
};
}; // namespace LCODE
