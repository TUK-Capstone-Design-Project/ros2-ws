#include "DotDetector/DotDetector.hpp"

namespace LCODE {
void DotDetector::setupParameters(cv::SimpleBlobDetector::Params params) {
  params_.minThreshold  = params.minThreshold;  // opencv default : 50
  params_.maxThreshold  = params.maxThreshold;  // opencv default : 220
  params_.thresholdStep = params.thresholdStep; // opencv default : 20

  params_.filterByArea        = params.filterByArea;
  params_.minArea             = params.minArea; // opencv default : 25
  params_.maxArea             = params.maxArea; // opencv default : 5000
  params_.filterByCircularity = params.filterByCircularity;
  params_.minCircularity      = params.minCircularity;
  params_.maxCircularity      = params.maxCircularity;
  params_.filterByConvexity   = params.filterByConvexity;
  params_.minConvexity        = params.minConvexity;
  params_.maxConvexity        = params.maxConvexity;

  params_.filterByInertia  = params.filterByInertia;
  params_.minInertiaRatio  = params.minInertiaRatio;
  params_.maxInertiaRatio  = params.maxInertiaRatio;
  params_.minRepeatability = params.minRepeatability;
}
auto DotDetector::detectBySimpleBlobDetector(const cv::Mat &img, std::vector<cv::KeyPoint> &dots) -> bool {
  cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params_);
  if (!detector) {
    std::cerr << "SimpleBlobDetector 생성 실패!" << std::endl;
    return false;
  }
  if (img.channels() != 1) {
    std::cout << "simpleblobdetector는 그레이스케일 이미지에서만 작동하므로, 입력 이미지를 그레이스케일로 변환합니다."
              << std::endl;
    return false;
  }

  detector->detect(img, dots);
#ifdef DEBUG_LOG
  std::cout << "검출된 점의 수: " << dots.size() << std::endl;
#endif
  return true;
}

// simpleblob 경량화 코드
// auto ImgPreprocessor::DetectDotsBySimpleBlobDetector() -> bool {
//   cv::SimpleBlobDetector::Params  params;
//   cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);

//   if (!detector) {
//     this->checkFlow      = false;
//     this->LBS_error_code = SBD_error;
//     return false;
//   }

//   try {
//     this->detected_all_dots.clear();
//     cv::medianBlur(gray, gray, 3);
//     detector->detect(gray, detected_all_dots);
// #ifdef DEBUG_LOG
//     std::cerr << "[SBD] detected keypoints: " << this->detected_all_dots.size() << "\n";
// #endif
//   } catch (const std::exception &e) {
//     this->LBS_error_code = SBD_error;
// #ifdef DEBUG_LOG
//     std::cerr << "[SBD] exception: " << e.what() << "\n";
// #endif
//     return false;
//   } catch (...) {
//     this->LBS_error_code = SBD_error;
//     return false;
//   }

//   return true;
// }

// auto ImgPreprocessor::DetectDotsBySimpleBlobDetector() -> bool {
//   if (this->src.empty()) {
//     this->checkFlow      = false;
//     this->LBS_error_code = SBD_error;
//     return false;
//   }

//   try {
//     this->detected_all_dots.clear();

//     // cv::Mat blurred;
//     // cv::medianBlur(this->gray, blurred, 3);

//     // 2. 이진화
//     cv::Mat binary;
//     cv::threshold(gray, binary, 0, 255, cv::THRESH_BINARY_INV | cv::THRESH_OTSU);

//     // 3. 고속 레이블링
//     cv::Mat labels, stats, centroids;
//     int     nLabels = cv::connectedComponentsWithStats(binary, labels, stats, centroids, 8, CV_32S);

//     for (int i = 1; i < nLabels; i++) {
//       int    area = stats.at<int>(i, cv::CC_STAT_AREA);
//       int    w    = stats.at<int>(i, cv::CC_STAT_WIDTH);
//       int    h    = stats.at<int>(i, cv::CC_STAT_HEIGHT);
//       double cx   = centroids.at<double>(i, 0);
//       double cy   = centroids.at<double>(i, 1);

//       // 필터링 로직 (Area, AspectRatio, Solidity 등)
//       // if (this->filterByArea && (area < this->minArea || area > this->maxArea))
//       //   continue;

//       double aspectRatio = static_cast<double>(w) / h;
//       if (aspectRatio < 0.4 || aspectRatio > 2.5)
//         continue;

//       // if (this->filterByCircularity) {
//       //   double rectArea = static_cast<double>(w * h);
//       //   if (((double)area / rectArea) < this->min_circularity)
//       //     continue;
//       // }

//       // if (this->filterByConvexity) {
//       //   double inertia = (w < h) ? (double)w / h : (double)h / w;
//       //   if (inertia < this->min_inertia)
//       //     continue;
//       // }

//       float size = std::sqrt(area * 4.0f / CV_PI);
//       this->detected_all_dots.emplace_back(cv::Point2f(static_cast<float>(cx), static_cast<float>(cy)), size);
//     }

//     // --- 시각화 로직 시작 ---
//     cv::Mat visImg;
//     if (this->src.channels() == 1)
//       cv::cvtColor(this->src, visImg, cv::COLOR_GRAY2BGR);
//     else
//       visImg = this->src.clone();

//     for (size_t i = 0; i < this->detected_all_dots.size(); i++) {
//       const auto &kp = this->detected_all_dots[i];
//       // 1. 검출된 점에 원 그리기 (녹색)
//       cv::circle(visImg, kp.pt, static_cast<int>(kp.size / 2), cv::Scalar(0, 255, 0), 2);
//       // 2. 점 번호 매기기 (빨간색)
//     }

//     std::cerr << "[FastBlob] Detected points: " << this->detected_all_dots.size() << "\n";
//     cv::imshow("Binary Mask", binary);      // 이진화 결과 확인 (임계값 튜닝용)
//     cv::imshow("Detection Result", visImg); // 최종 검출 결과 확인
//     cv::waitKey(1);                         // 0이면 멈춤, 1이면 영상처럼 흐름
//     // --- 시각화 로직 끝 ---

//   } catch (const std::exception &e) {
//     this->checkFlow      = false;
//     this->LBS_error_code = SBD_error;
// #ifdef DEBUG_LOG
//     std::cerr << "[SBD_Replacement] Exception: " << e.what() << "\n";
// #endif
//     return false;
//   }

//   return true;
// }

} // namespace LCODE
