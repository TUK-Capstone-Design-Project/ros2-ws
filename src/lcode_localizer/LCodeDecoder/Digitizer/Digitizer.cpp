#include "Digitizer/Digitizer.hpp"

// #define DEBUG_LOG

namespace LCODE {

void Digitizer::Sort2DdotArr(vector<Point2f> *dotsInfo, float devidedWidth, float devidedHeight,
                             unordered_map<pair<int, int>, Point2f, pair_hash> *outputDotsInfo) {
  //(1,1) ~ (entireCol - 2, entireRow - 2)
  for (int i = 1; i < entireRow - 1; i++) {
    for (int j = 1; j < entireCol - 1; j++) {
      float x_range_min = devidedWidth * (j - 1) + devidedWidth / 2;
      float x_range_max = x_range_min + devidedWidth;
      float y_range_min = devidedHeight * (i - 1) + devidedHeight / 2;
      float y_range_max = y_range_min + devidedHeight;

      for (Point2f dot_var : *dotsInfo) {
        float kp_x = dot_var.x;
        float kp_y = dot_var.y;
        if (x_range_min <= kp_x && kp_x <= x_range_max && y_range_min <= kp_y && kp_y <= y_range_max) {
          (*outputDotsInfo)[make_pair(j, i)] = dot_var;
          break;
        }
      }
    }
  }
}

void Digitizer::InterpolatingDashArea(unordered_map<pair<int, int>, Point2f, pair_hash> *sortedEntiredots,
                                      float devidedHeight, vector<Point2f> &baselineDots) {
  vector<pair<pair<int, int>, Point2f>> baselineIndex;
  for (Point2f var : baselineDots) {
    for (pair<pair<int, int>, Point2f> entire_var : *sortedEntiredots) {
      if (abs(var.x - entire_var.second.x) < EPS) {
        baselineIndex.push_back(entire_var);
        break;
      }
    }
  }

  for (pair<pair<int, int>, Point2f> var : baselineIndex) {
    int   indexX = var.first.first;
    int   indexY = var.first.second;
    float realX  = var.second.x;
    float realY  = var.second.y;
    for (int i = indexY - 1; i >= 1; i--) {
      // 원래 있던 점은 무시할지 덮어씌울지 해보고 결정
      float minus_amount                        = (i - indexY) * devidedHeight;
      (*sortedEntiredots)[make_pair(indexX, i)] = Point2f(realX, realY + minus_amount);
    }

    for (int i = indexY + 1; i <= entireRow - 2; i++) {
      float plus_amount                         = (i - indexY) * devidedHeight;
      (*sortedEntiredots)[make_pair(indexX, i)] = Point2f(realX, realY + plus_amount);
    }
  }
}

void Digitizer::FindCP_Index(unordered_map<pair<int, int>, Point2f, pair_hash> *sortedEntiredots, Point2f CP,
                             pair<int, int> *CP_dotIndex) {
  float minLength = std::numeric_limits<float>::max();
  for (pair<pair<int, int>, Point2f> var : *sortedEntiredots) {
    float klength = GetDistance(var.second.x, var.second.y, CP.x, CP.y);
    if (minLength > klength) {
      minLength      = klength;
      (*CP_dotIndex) = var.first;
    }
  }
}

bool Digitizer::ExtractSubset(unordered_map<pair<int, int>, Point2f, pair_hash> *sortedEntiredots,
                              pair<int, int> CP_dotIndex, unordered_map<pair<int, int>, Point2f, pair_hash> *subset,
                              pair<Point2i, Point2i> *subRect) {
  // subset을 하나의 rect라고 할때 그 rect가 전체 범위를 벗어나지 않도록 함 그 후 좌상 점을 보정함
  int leftIndexX   = CP_dotIndex.first - subCol / 2;
  int rightIndexX  = leftIndexX + subCol - 1;
  int topIndexY    = CP_dotIndex.second - subRow / 2;
  int bottomIndexY = topIndexY + subRow - 1;

  this->movementIndex = make_pair(subCol / 2, subRow / 2); // 나중에 이정도 움직여야 진짜 CP 위치가 나옴 x, y

  if (leftIndexX < 1) {
    this->movementIndex.first += -(1 - leftIndexX);
    leftIndexX  = 1;
    rightIndexX = subCol;
  } else if (rightIndexX > entireCol - 2) {
    this->movementIndex.first += -(entireCol - 2 - rightIndexX);
    leftIndexX  = (entireCol - 2) - subCol + 1;
    rightIndexX = entireCol - 2;
  }
  if (topIndexY < 1) {
    this->movementIndex.second += -(1 - topIndexY);
    topIndexY    = 1;
    bottomIndexY = subRow;
  } else if (bottomIndexY > entireRow - 2) {
    this->movementIndex.second += -(entireRow - 2 - bottomIndexY);
    topIndexY    = (entireRow - 2) - subRow + 1;
    bottomIndexY = entireRow - 2;
  }

  // subset (0 ,0) ~ (Col - 1, Row - 1)
  for (int i = 0; i < subRow; i++) {
    for (int j = 0; j < subCol; j++) {
      auto item = sortedEntiredots->find(make_pair(leftIndexX + j, topIndexY + i));
      if (item == sortedEntiredots->end())
        return false;
      (*subset)[make_pair(j, i)] = item->second;
    }
  }

  (*subRect) = make_pair(Point2i(leftIndexX, topIndexY), Point2i(rightIndexX, bottomIndexY));

  return true;
}

void Digitizer::FindGridCrossPoint(pair<Point2i, Point2i> *subRect, float devidedWidth, float devidedHeight,
                                   unordered_map<pair<int, int>, Point2f, pair_hash> *crossPoints) {
  int leftIndexX   = subRect->first.x;
  int topIndexY    = subRect->first.y;
  int rightIndexX  = subRect->second.x;
  int bottomIndexY = subRect->second.y;

  for (int i = topIndexY; i <= bottomIndexY; i++) {
    for (int j = leftIndexX; j <= rightIndexX; j++) {
      (*crossPoints)[make_pair(j - leftIndexX, i - topIndexY)] = Point2f(devidedWidth * j, devidedHeight * i);
    }
  }
}

void Digitizer::FindBaselineInSubset(unordered_map<pair<int, int>, Point2f, pair_hash> *subset,
                                     vector<Point2f> &baselineDots, vector<int> *baselineIndex,
                                     vector<float> *baselineX) {
  for (Point2f var : baselineDots) {
    for (pair<pair<int, int>, Point2f> sub_var : *subset) {
      if (abs(var.x - sub_var.second.x) < EPS) {
        baselineIndex->push_back(sub_var.first.first);
        baselineX->push_back(var.x);
        break;
      }
    }
  }
}

// /// @brief 서브셋에서 올바른 grid를 그렸을때 생기는 grid의 모든 교점 좌표를 만들어 내는 함수
// /// @param subset COL x ROW 크기로 추출된 서브셋 점들(input)
// /// @param devidedWidth 점과 점사이의 거리 (input)
// /// @param baselineIndex 서브셋에서 기준선이 가지는 X index 값 (input)
// /// @param baselineX 기준선의 x좌표 값 (input)
// /// @param crossPoints grid의 교점들 (output)
// void FindCrossPoint(unordered_map<pair<int, int>, Point2f, pair_hash>* subset, float devidedWidth, vector<int>
// baselineIndex,
//                      vector<float> baselineX, unordered_map<pair<int, int>, Point2f, pair_hash>* crossPoints)
// {
//     vector<float> grid_X; //세로방향
//     for (int i = -baselineIndex[0]; i < subCol - baselineIndex[0]; i++)
//     {
//         grid_X.push_back(baselineX[0] + devidedWidth * i);
//     }

//     for (int i = 0; i < subRow; i++)
//     {
//         for (int j = 0; j < subCol; j++)
//         {
//             //dotPoint dotpoint = { grid_X[j], (*subset)[make_pair(baselineIndex, i)].y, j, i, DOTTYPE::NOISE};
//             (*crossPoints)[make_pair(j, i)] = Point2f(grid_X[j], (*subset)[make_pair(baselineIndex[0], i)].y);
//         }
//     }
// }

bool Digitizer::digitize(unordered_map<pair<int, int>, Point2f, pair_hash> *crossPoints,
                         unordered_map<pair<int, int>, Point2f, pair_hash> *subset, vector<int> baselineIndex) {
  float               degreeErr = 0.0f;
  vector<vector<int>> digitizedOutput;
  for (int i = 0; i < subRow; i++) {
    vector<int> temp;
    for (int j = 0; j < subCol; j++) {
      bool isBaseline = false;
      for (int var : baselineIndex) {
        if (j == var) {
          temp.push_back(CENTER);
          isBaseline = true;
          break;
        }
      }
      if (isBaseline)
        continue;

      Point2f centerDot  = (*crossPoints)[make_pair(j, i)];
      Point2f targetDot  = (*subset)[make_pair(j, i)];
      float   y_distance = targetDot.y - centerDot.y;
      float   x_distance = targetDot.x - centerDot.x;
      float   degree     = atan2(y_distance, x_distance) * 180 / CV_PI;

      float deltaDegree = 360 / (8.0 * 2);
      // int dotType = 0;
      // degree += 180;
      // bool isFine = false;
      //  for (int i = 0; i <= 45; i += 45)
      //  {
      //      for (int j = 0; j <= 360; j += 90)
      //      {
      //          int centerDegree = i + j;

      //         if(centerDegree - deltaDegree <= degree && degree <= centerDegree + deltaDegree)
      //         {

      //             isFine = true;
      //             break;
      //         }

      //         dotType += 1;
      //     }
      //     if(isFine) break;
      //     dotType += 4;
      // }

      if (degree < -180 + deltaDegree || degree >= 180 - deltaDegree) {
        degreeErr = max(abs(abs(degree) - 180), degreeErr);
        temp.push_back(LEFT);
      } else if (degree >= 90 - deltaDegree && degree < 90 + deltaDegree) {
        degreeErr = max(abs(degree - 90), degreeErr);
        temp.push_back(BOTTOM);
      } else if (degree >= 0 - deltaDegree && degree < 0 + deltaDegree) {
        degreeErr = max(abs(degree), degreeErr);
        temp.push_back(RIGHT);
      } else if (degree >= -90 - deltaDegree && degree < -90 + deltaDegree) {
        degreeErr = max(abs(degree + 90), degreeErr);
        temp.push_back(TOP);
      } else if (degree >= -135 - deltaDegree && degree < -135 + deltaDegree) {
        degreeErr = max(abs(degree + 135), degreeErr);
        temp.push_back(LEFT_TOP);
      } else if (degree >= -45 - deltaDegree && degree < -45 + deltaDegree) {
        degreeErr = max(abs(degree + 45), degreeErr);
        temp.push_back(RIGHT_TOP);
      } else if (degree >= 45 - deltaDegree && degree < 45 + deltaDegree) {
        degreeErr = max(abs(degree - 45), degreeErr);
        temp.push_back(RIGHT_BOTTOM);
      } else if (degree >= 135 - deltaDegree && degree < 135 + deltaDegree) {
        degreeErr = max(abs(degree - 135), degreeErr);
        temp.push_back(LEFT_BOTTOM);
      }
    }
    digitizedOutput.push_back(temp);
    if (MaxAngle < degreeErr)
      return false;
  }
  this->digitizied_array = digitizedOutput;
  // cout << "(디지털화 부분) 각도(육십분법) 최대 오차 :" << degreeErr << endl;
  return true;
}

bool Digitizer::runDigitize(vector<Point2f> &entireDotsInfo, Point2f CP, vector<vector<Point2f>> &baselineDots,
                            int ROIwidth, int ROIheight) {
  if ((entireCol - 2) < subCol || (entireRow - 2) < subRow) {
#ifdef DEBUG_LOG
    cout << "(디지털화 설정값 ERROR) 서브셋" << endl;
#endif
    return false;
  }
  if (entireDotsInfo.size() < subRow * subCol) {
#ifdef DEBUG_LOG
    cout << "(디지털화 ERROR) 점 개수 부족" << endl;
#endif
    return false;
  }

  vector<Point2f> baselineOneDots;

  for (vector<Point2f> var : baselineDots) {
    if (var.empty()) {
#ifdef DEBUG_LOG
      cout << "(디지털화 ERROR) 기준선 점 개수 부족" << endl;
#endif
      return false;
    }

    baselineOneDots.push_back(var[0]);
  }

  unordered_map<pair<int, int>, Point2f, pair_hash> entireDots;
  unordered_map<pair<int, int>, Point2f, pair_hash> subsetDots;
  unordered_map<pair<int, int>, Point2f, pair_hash> crossPoints;

  vector<int>            baselineIndex;
  vector<float>          baselineX;
  pair<int, int>         CP_dotIndex;
  pair<Point2i, Point2i> subRect;
  float                  devidedWidth  = float(ROIwidth) / (entireCol - 1);
  float                  devidedHeight = float(ROIheight) / (entireRow - 1);

  Sort2DdotArr(&entireDotsInfo, devidedWidth, devidedHeight, &entireDots); // 1
  InterpolatingDashArea(&entireDots, devidedHeight, baselineOneDots);      // 2

  FindCP_Index(&entireDots, CP, &CP_dotIndex);
  if (ExtractSubset(&entireDots, CP_dotIndex, &subsetDots, &subRect) == false) {
    // cout << "(디지털화 ERROR) 서브셋 내에 비어있는 점 있음" << endl;
    return false; // 3
  }
  // FindCrossPoint(&subsetDots, devidedHeight, baselineIndex, baselineX, &crossPoints);// 5
  FindGridCrossPoint(&subRect, devidedWidth, devidedHeight, &crossPoints);
  FindBaselineInSubset(&subsetDots, baselineOneDots, &baselineIndex, &baselineX); // 4

  if (digitize(&crossPoints, &subsetDots, baselineIndex) == false) {
#ifdef DEBUG_LOG
    cout << "(디지털화 ERROR) 오차값" << MaxAngle << "이상" << endl;
#endif
    return false;
  }

  return true;
}

bool Digitizer::runDigitize(ImgPreprocessor &ImgPre) {
  if ((entireCol - 2) < subCol || (entireRow - 2) < subRow) {
#ifdef DEBUG_LOG
    cout << "(디지털화 설정값 ERROR) 서브셋" << endl;
#endif
    return false;
  }
  if (ImgPre.transformed_points.size() < subRow * subCol) {
#ifdef DEBUG_LOG
    cout << "(디지털화 ERROR) 점 개수 부족" << endl;
#endif
    return false;
  }

  vector<Point2f> baselineOneDots;

  for (vector<Point2f> var : ImgPre.middle_baseLines_dots_in_ROI) {
    if (var.empty()) {
#ifdef DEBUG_LOG
      cout << "(디지털화 ERROR) 기준선 점 개수 부족" << endl;
#endif
      return false;
    }

    baselineOneDots.push_back(var[0]);
  }

  unordered_map<pair<int, int>, Point2f, pair_hash> entireDots;
  unordered_map<pair<int, int>, Point2f, pair_hash> subsetDots;
  unordered_map<pair<int, int>, Point2f, pair_hash> crossPoints;

  vector<int>            baselineIndex;
  vector<float>          baselineX;
  pair<int, int>         CP_dotIndex;
  pair<Point2i, Point2i> subRect;
  float                  devidedWidth  = ImgPre.ROI_width_size / (entireCol - 1);
  float                  devidedHeight = ImgPre.ROI_height_size / (entireRow - 1);
  Sort2DdotArr(&ImgPre.transformed_points, devidedWidth, devidedHeight, &entireDots); // 1
  InterpolatingDashArea(&entireDots, devidedHeight, baselineOneDots);                 // 2

  FindCP_Index(&entireDots, ImgPre.transformed_CP, &CP_dotIndex);
  if (ExtractSubset(&entireDots, CP_dotIndex, &subsetDots, &subRect) == false) {
#ifdef DEBUG_LOG
    cout << "(디지털화 ERROR) 서브셋 내에 비어있는 점 있음" << endl;
#endif
    return false; // 3
  }
  // FindCrossPoint(&subsetDots, devidedHeight, baselineIndex, baselineX, &crossPoints);// 5
  FindGridCrossPoint(&subRect, devidedWidth, devidedHeight, &crossPoints);
  FindBaselineInSubset(&subsetDots, baselineOneDots, &baselineIndex, &baselineX); // 4

  if (digitize(&crossPoints, &subsetDots, baselineIndex) == false) {
#ifdef DEBUG_LOG
    cout << "(디지털화 ERROR) 오차값" << MaxAngle << "이상" << endl;
#endif
    return false;
  }

  return true;
}

} // namespace LCODE

// using namespace std;

// void printArray(const vector<vector<int>>& array) {
//     for (const auto& row : array) {
//         for (int value : row) {
//             cout << value;
//         }
//         cout << endl;
//     }
//     cout << "----------------" << endl;
// }

// int main()
// {

//     vector<cv::Point2f> entire_dots_coordinates =
//     {{537.883,549.021},{488.051,549.017},{438.059,548.981},{238.44,548.995},{188.485,548.994},{138.609,549.007},{88.6733,549.003},{38.7609,549.061},{399.331,537.927},{349.378,537.905},{388.13,499.105},{338.28,499.203},{549.076,488.081},{499.158,487.992},{449.305,488.071},{249.529,487.997},{199.637,488.074},{149.715,488.024},{99.823,488.065},{49.836,488.037},{537.904,449.186},{488.052,449.185},{438.038,449.288},{299.429,449.269},{238.439,449.284},{188.455,449.18},{138.611,449.213},{88.5847,449.258},{38.7573,449.269},{399.299,438.036},{349.468,438.025},{338.261,399.336},{537.921,399.381},{488.038,399.38},{438.059,399.266},{388.18,399.274},{249.539,388.148},{199.64,388.252},{149.728,388.13},{99.8721,388.201},{49.8338,388.157},{388.181,349.457},{338.275,349.415},{238.477,349.458},{188.51,349.36},{138.587,349.384},{88.6276,349.449},{38.7029,349.452},{549.102,338.285},{499.112,338.269},{449.316,338.319},{537.91,299.551},{488.057,299.549},{438.04,299.439},{299.427,299.451},{238.443,299.422},{188.452,299.539},{138.606,299.505},{88.5576,299.445},{38.7632,299.461},{399.317,288.406},{349.415,288.391},{537.923,249.598},{488.043,249.611},{438.077,249.612},{388.133,249.621},{338.265,249.466},{249.551,238.472},{199.634,238.493},{149.743,238.468},{99.846,238.445},{49.9371,238.468},{537.896,199.723},{488.062,199.723},{438.069,199.726},{388.176,199.632},{338.239,199.815},{238.459,199.746},{188.458,199.707},{138.597,199.701},{88.625,199.695},{38.7429,199.642},{537.943,149.7},{488.034,149.699},{438.066,149.676},{388.209,149.789},{338.258,149.678},{299.465,149.698},{238.465,149.669},{188.443,149.688},{138.591,149.696},{88.5894,149.69},{38.7543,149.767},{537.936,99.8468},{488.044,99.8448},{438.08,99.8786},{388.083,99.8151},{338.289,99.8979},{238.426,99.873},{188.469,99.8111},{138.56,99.8324},{88.696,99.8246},{38.6424,99.8472},{537.911,61.0494},{488.043,61.0588},{438.072,61.1771},{388.214,49.9607},{338.24,49.958},{238.441,49.9453},{188.432,49.9232},{138.582,49.9452},{88.5831,49.9533},{38.7475,49.937}};

//     vector<cv::Point2f> MID = {{299.429,449.269},{299.427,299.451},{299.465,149.698}};
//     cv::Point2f CP = { 34.6116,33.5256 };

//     vector<vector<cv::Point2f>> Midadf;
//     Midadf.push_back(MID);

//     vector<vector<int>> test;
//     pair<int, int> pasdff;

//     if(LBS_Digit::runDigitize(entire_dots_coordinates, CP, Midadf, 600, 600, &test, &pasdff))
//     {
//         printArray(test);
//         cout << pasdff.first << "," << pasdff.second << endl;
//     }
//     else
//     {
//         cout << "NOPE" << endl;
//     }

//     return 0;
// }