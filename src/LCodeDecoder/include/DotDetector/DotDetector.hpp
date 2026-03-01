#include "opencv2/opencv.hpp"
#include <opencv2/features2d.hpp>
namespace LCODE {

class DotDetector {
public:
  auto detectBySimpleBlobDetector(const cv::Mat &img, std::vector<cv::KeyPoint> &dots) -> bool;

  void setupParameters(cv::SimpleBlobDetector::Params params);

private:
  cv::SimpleBlobDetector::Params params_;
};
} // namespace LCODE