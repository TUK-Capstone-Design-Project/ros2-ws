#include "DeBruijnSeq/GenBGArray.hpp"

// My libraries
#include "DeBruijnSeq/DeBruijnSeq.hpp"

// Standard libraries
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <utility>

// #include <iconv.h>
#include <cstring>

// #define SHOW_TIME

namespace LCODE {
constexpr auto DIRECTORY_PATH = "../../output/";

BGArray::BGArray(ConfigSettings &config, const std::size_t &_lineInterval) :
    a(std::move(config.getAlpha())), k1(config.getSubRow()), k2(config.getSubCol()), lineInterval(_lineInterval) {
  initMemberVars();
}

BGArray::BGArray(std::string _a, const std::size_t &_k1, const std::size_t &_k2, const std::size_t &_lineInterval) :
    a(std::move(_a)), k1(_k1), k2(_k2), lineInterval(_lineInterval) {
  initMemberVars();
}

void BGArray::initMemberVars() {
  aLen = a.length();

  deSeqK1 = DBSeq(a, aLen, k1).getDBSeq();     // 서브셋의 행 길이 기반 시퀀스
  deSeqK2 = DBSeq(a, aLen, k2 - 1).getDBSeq(); // 서브셋의 열 길이 기반 시퀀스z

  deSeqK1Len = deSeqK1.length() - 1; // 2차원 배경 배열의 행 크기 저장
  deSeqK2Len = deSeqK2.length();     // 2차원 배경 배열의 열 크기 저장

  BGRows = deSeqK1Len;
  BGCols = deSeqK2Len + deSeqK2Len / lineInterval;

  // s-seq 생성
  sSeq = deSeqK1.substr(0, deSeqK1Len);

  // t-seq 생성
  tSeq = deSeqK2.substr(0, deSeqK2Len - 1);

  // t-seq에 따라 밀린 누적 횟수를 각각 저장한다.
  unsigned int count = 0;
  for (std::size_t i = 0; i < deSeqK2Len; i++) {
    if (i == 0) { // 첫번째 열은 s-seq를 그대로 사용하였기 때문에 0을 넣는다.
      countShift.push_back(0);
      continue;
    }
    count += tSeq[i - 1] - '0';
    countShift.push_back(count);
  }
}

void BGArray::setBGArr() {
  BGArr.assign(BGRows, std::vector<int>(BGCols)); // 배경 배열 크기 지정
  std::size_t sSeqLen = sSeq.size();

  for (std::size_t i = 0; i < BGCols; i++) {
    if (i == 0) { // 첫째 열은 s-seq 그대로 출력
      for (std::size_t j = 0; j < BGRows; j++) {
        BGArr[j][i] = sSeq[j] - '0';
      }
      continue;
    }

    for (std::size_t j = 0; j < BGRows; j++) {
      BGArr[j][i] = sSeq[(sSeqLen * i + (j - countShift[i])) % sSeqLen] - '0';
    }
  }
}

void BGArray::printBGArr(std::string filepath, std::string fileName) {
  std::string fileLocation = filepath + fileName + ".txt";

  if (filepath.empty()) {
    return;
  }

  if (filepath[filepath.size() - 1] != '/') {
    filepath.push_back('/');
  }

  if (!std::filesystem::exists(filepath)) {
    std::filesystem::create_directory(filepath);
  }

  std::ofstream writeFile(std::filesystem::u8path((const char *)(fileLocation).c_str()));
  if (!writeFile.is_open()) {
    std::cerr << "File open error.\n";
    std::exit(1);
  }

  std::size_t sSeqLen = sSeq.length();

  for (std::size_t i = 0; i < BGRows; i++) {
    std::size_t colCorrection = 0;

    for (std::size_t j = 0; j < BGCols; j++) {
      if (j % (lineInterval + 1) == 0) {
        writeFile << "-1 ";
        colCorrection++;
        continue;
      }
      // if (j > 0 && j % k2 == 0) {
      //     writeFile << std::format("{} ", 4);
      // }
      char val = sSeq[(sSeqLen * j + (i - countShift[j - colCorrection])) % sSeqLen];
      writeFile << val << " ";
    }
    writeFile << "\n";
  }

  writeFile.close();
}

void BGArray::printSpecificRange(const std::size_t &col1, const std::size_t &row1, const std::size_t &col2,
                                 const std::size_t &row2, std::string filepath) {
  std::size_t rowRange = std::abs(static_cast<int>(row2) - static_cast<int>(row1)) + 1;
  std::size_t colRange = std::abs(static_cast<int>(col2) - static_cast<int>(col1)) + 1;

  if (filepath.empty()) {
    return;
  }

  if (filepath[filepath.size() - 1] != '/' && filepath[filepath.size() - 1] != '\\') {
    filepath.push_back('/');
  }

  if (rowRange > BGRows || colRange > BGCols) {
    std::cerr << "Range error.\n";
    return;
  }

  if (row1 > BGRows - 1 || row2 > BGRows - 1) {
    std::cerr << "Row size error.\n";
    return;
  }

  if (col1 > BGCols - 1 || col2 > BGCols - 1) {
    std::cerr << "Col size error.\n";
    return;
  }

  std::string directoryPath = DIRECTORY_PATH;
  std::string fileLocation  = filepath + "print_specific_range.txt";

  if (!std::filesystem::exists(filepath)) {
    std::filesystem::create_directory(filepath);
  }
  // std::string euckr_fileLocation = utf8_to_euckr(fileLocation);

  std::ofstream writeFile(std::filesystem::u8path((const char *)fileLocation.c_str()));
  if (!writeFile.is_open()) {
    std::cerr << "File open error.\n";
    std::exit(1);
  }

  std::size_t sSeqLen = sSeq.length();

  for (std::size_t i = std::min(row1, row2); i < std::max(row1, row2) + 1; i++) {
    std::size_t colCorrection = std::min(col1, col2) / (lineInterval + 1);
    for (std::size_t j = std::min(col1, col2); j < std::max(col1, col2) + 1; j++) {
      if (j == std::min(col1, col2) && j % (lineInterval + 1) != 0) {
        colCorrection++;
      }
      if (j % (lineInterval + 1) == 0) {
        writeFile << "-1 ";
        colCorrection++;
        continue;
      }
      char val = sSeq[(sSeqLen * j + (i - countShift[j - colCorrection])) % sSeqLen];
      writeFile << val << " ";
    }
    writeFile << "\n";
  }
  writeFile.close();
}

auto BGArray::getSubset(const std::size_t &x, const std::size_t &y, const std::size_t &col_size,
                        const std::size_t &row_size) -> std::vector<std::vector<int>> {
  if (x > BGCols - 1 || y > BGRows - 1) {
    std::vector<std::vector<int>> emptyVec;
    return emptyVec;
  }
  std::size_t sSeqLen = sSeq.length();

  std::vector<std::vector<int>> input;
  for (std::size_t i = y; i < y + row_size; i++) {
    std::size_t      colCorrection = x / (lineInterval + 1);
    std::vector<int> tempInput;
    for (std::size_t j = x; j < x + col_size; j++) {
      if (j == x && j % (lineInterval + 1) != 0) {
        colCorrection++;
      }
      if (j % (lineInterval + 1) == 0) {
        tempInput.push_back(-1);
        colCorrection++;
        continue;
      }
      char val = sSeq[(sSeqLen * j + (i - countShift[j - colCorrection])) % sSeqLen];
      tempInput.push_back(val - '0');
    }
    input.push_back(tempInput);
  }

  return input;
}

void BGArray::setElemt(const std::string &_a, const std::size_t &_k1, const std::size_t &_k2) {
  a  = _a;
  k1 = _k1;
  k2 = _k2;

  initMemberVars();
  setBGArr();
}

auto BGArray::bgRowsSize() -> std::size_t {
  return BGRows;
}

auto BGArray::bgColsSize() -> std::size_t {
  return BGCols;
}

auto BGArray::getPos(const std::vector<std::vector<int>> &subset, const std::pair<int, int> &center)
    -> std::pair<int, int> {
  std::string tempSSeq = sSeq + sSeq;
  std::string subsetSSeq; // 입력한 서브셋의 첫번째 열
  std::string subsetTSeq; // 입력한 서브셋의 t-seq

  // cout<<"this getpos's subset"<<endl;
  // for(int i=0 ;)

  std::pair<int, int> subsetLocation = std::make_pair(-1, -1); // 행과 열의 위치 저장
  // if (subset.size() != k1 + (aLen - 1) || subset[0].size() != k2 + 1) {
  //     return subsetLocation;
  // }

  std::vector<std::string> subsetToChar;

  int locationRow = -1; // 찾아낸 열 번호
  int locationCol = -1; // 찾아낸 행 번호

  std::size_t locationColCorrection = 0; // 열 위치 보정

#ifdef SHOW_TIME
  std::chrono::steady_clock::time_point start, end;

  start = std::chrono::steady_clock::now();
#endif
  // 입력받은 서브셋을 문자로 변환
  for (const auto &v : subset) {
    std::string tempStr;
    for (const auto &num : v) {
      if (num == -1) {
        tempStr.push_back(static_cast<char>(' '));
      } else {
        tempStr.push_back(static_cast<char>(num + '0'));
      }
    }
    subsetToChar.push_back(tempStr);
  }

  // 고유한 문자열 외의 문자가 입력되면 반환
  bool wrongType = true;
  for (const auto &vec : subsetToChar) {
    for (const auto &val : vec) {
      wrongType = true;
      for (const char &alpha : a) {
        if (val == alpha || val == ' ') {
          wrongType = false;
          break;
        }
      }
      if (wrongType) {
        return subsetLocation;
      }
    }
  }
#ifdef SHOW_TIME
  end = std::chrono::steady_clock::now();

  auto valCheckTime = std::chrono::duration_cast<std::chrono::duration<double>>(end - start).count();

  start = std::chrono::steady_clock::now();
#endif

  // 서브셋에서 서브 s-seq 찾기
  for (const auto &vec : subsetToChar) {
    if (subsetSSeq.size() >= k1) {
      break;
    }
    if (vec[0] == ' ') {
      subsetSSeq.push_back(vec[1]);
    } else {
      subsetSSeq.push_back(vec[0]);
    }
  }
#ifdef SHOW_TIME
  end = std::chrono::steady_clock::now();

  start = std::chrono::steady_clock::now();
#endif
  auto bmsFirstCol = std::boyer_moore_searcher(subsetSSeq.begin(), subsetSSeq.end());
  auto firstColPos = std::search(tempSSeq.begin(), tempSSeq.end(), bmsFirstCol);
#ifdef SHOW_TIME
  end = std::chrono::steady_clock::now();

  auto findRowPosTime = std::chrono::duration_cast<std::chrono::duration<double>>(end - start).count();
#endif
  if (firstColPos == tempSSeq.end()) {
    return {-1, -1};
  }
  auto distFirstColPos = std::distance(tempSSeq.begin(), firstColPos);

#ifdef SHOW_TIME
  auto findSubSSeqTime = std::chrono::duration_cast<std::chrono::duration<double>>(end - start).count();

  start = std::chrono::steady_clock::now();
#endif
  // 입력한 서브셋의 t-seq를 찾는다.
  auto rangeForFindSeq = tempSSeq.substr(tempSSeq.size() / 2 + distFirstColPos - aLen * k1, aLen * k1 + k1);
  if (static_cast<std::size_t>(distFirstColPos) > aLen * k1) {
    rangeForFindSeq = tempSSeq.substr(distFirstColPos - aLen * k1, aLen * k1 + k1);
  }
  int plusCol1 = 0;
  int plusCol2 = 0;
  for (std::size_t i = 0; i < subsetToChar[0].size(); i++) {
    std::string col;  // 현재 열
    std::string col2; // 다음 열
    std::size_t rowLen = k1;

    if (subsetTSeq.size() >= k2 - 1) {
      plusCol1 = 0;
      plusCol2 = 0;
      break;
    }
    // division by zero 경고 제거
    if (rowLen == 0) {
      throw std::runtime_error("rowLen is zero.");
    }

    // 현재 열이 라인이면 plusCol1을 1 증가시켜 넘기기
    if (subsetToChar[0][i + plusCol1] == ' ') {
      plusCol1++;
    }

    // 다음 열이 라인이면 plusCol2를 1 증가시켜 넘기기
    if (subsetToChar[0][i + 1 + plusCol2] == ' ') {
      plusCol2++;
    }

    // 처음 열이 라인이면 plusCol2를 1 증가시켜 넘기기
    if (i == 0 && subsetToChar[0][0] == ' ') {
      plusCol2++;
    }

    for (const auto &vec : subsetToChar) {
      if (col.size() >= k1) {
        break;
      }
      col.push_back(vec[i + plusCol1]);
      // if (col.size() == 5) {
      //     break;
      // }
    }

    for (const auto &vec : subsetToChar) {
      if (col2.size() >= k1) {
        break;
      }
      col2.push_back(vec[(i + 1) + plusCol2]);
    }

    auto bmsCol1 = std::boyer_moore_searcher(col.begin(), col.end());
    auto bmsCol2 = std::boyer_moore_searcher(col2.begin(), col2.end());

#ifdef SHOW_TIME
    auto testStart = std::chrono::steady_clock::now();
#endif
    auto findCol1 = std::search(rangeForFindSeq.begin(), rangeForFindSeq.end(), bmsCol1);
    auto findCol2 = std::search(rangeForFindSeq.begin(), rangeForFindSeq.end(), bmsCol2);

#ifdef SHOW_TIME
    auto testEnd = std::chrono::steady_clock::now();
#endif

#ifdef SHOW_TIME
    auto testDuration = std::chrono::duration_cast<std::chrono::duration<double>>(testEnd - testStart).count();

    std::cout << std::format("test time: {:.6f}s\n", testDuration);
#endif

    // tempDist = static_cast<long>(std::distance(sSeq.begin(), findCol2) % (sSeq.size() / 2));

    if (findCol1 == rangeForFindSeq.end() || findCol2 == rangeForFindSeq.end()) {
      plusCol1 = 0;
      plusCol2 = 0;
      return {-1, -1};
    }

    int distCol1 = static_cast<int>(std::distance(rangeForFindSeq.begin(), findCol1));
    int distCol2 = static_cast<int>(std::distance(rangeForFindSeq.begin(), findCol2));

    subsetTSeq.push_back(static_cast<char>(std::abs(distCol2 - distCol1) + '0'));

    // auto findStr = std::search(col2.begin(), col2.end(), col.begin(), col.end());
    // if (findStr != col2.end()) {
    //     std::size_t posStr = std::distance(col2.begin(), findStr);
    //     subsetTSeq.push_back(static_cast<char>(posStr + '0'));
    // };

    // 반복문 끝에 도달하면 plusCol1과 plusCol2를 0으로 초기화
    if (i == k2 - 2) {
      plusCol1 = 0;
      plusCol2 = 0;
    }
  }
#ifdef SHOW_TIME
  end = std::chrono::steady_clock::now();

  auto findSubTSeqTime = std::chrono::duration_cast<std::chrono::duration<double>>(end - start).count();

  start = std::chrono::steady_clock::now();
#endif
  // 열 위치 찾기
  auto bmsCol  = std::boyer_moore_searcher(subsetTSeq.begin(), subsetTSeq.end());
  auto findCol = std::search(tSeq.begin(), tSeq.end(), bmsCol);
  if (findCol != tSeq.end()) {
    std::size_t posCol    = std::distance(tSeq.begin(), findCol);
    locationCol           = static_cast<int>(posCol);
    locationColCorrection = posCol / lineInterval + 1;
    if (subsetToChar[0][0] == ' ') {
      locationColCorrection--;
    }
  } else {
    return {-1, -1};
  }

#ifdef SHOW_TIME
  end = std::chrono::steady_clock::now();

  auto findColTime = std::chrono::duration_cast<std::chrono::duration<double>>(end - start).count();

#endif
  // 행 위치 찾기
  if (firstColPos != tempSSeq.end()) {
    locationRow = static_cast<int>((distFirstColPos + countShift[locationCol]) % (sSeq.size()));
  } else {
    return {-1, -1};
  }

#ifdef SHOW_TIME

  std::cout << std::format("Check invalid val: {:.6f}s\n", valCheckTime);
  std::cout << std::format("Find subset s_seq: {:.6f}s\n", findSubSSeqTime);
  std::cout << std::format("Find subset t_seq: {:.6f}s\n", findSubTSeqTime);
  std::cout << std::format("Find col pos: {:.6f}s\n", findColTime);
  std::cout << std::format("Find row pos: {:.6f}s\n", findRowPosTime);
#endif

  // 최종 위치 정보 저장
  subsetLocation.first  = locationCol + static_cast<int>(locationColCorrection) + center.first;
  subsetLocation.second = locationRow + center.second;
  return subsetLocation;
}

auto BGArray::getBGArr() const -> std::vector<std::vector<int>> {
  return BGArr;
}

auto BGArray::getSubRow() const -> std::size_t {
  return k1;
}

auto BGArray::getSubCol() const -> std::size_t {
  return k2;
}

auto BGArray::getAlphaLenght() const -> std::size_t {
  return aLen;
}

auto BGArray::printTSeq(std::string filepath) const -> void {
  if (filepath.empty()) {
    return;
  }
  if (filepath[filepath.size() - 1] != '/' && filepath[filepath.size() - 1] != '\\') {
    if (filepath[0] == 'C' || filepath[0] == 'c') {
      filepath.push_back('\\');
    } else if (filepath[filepath.size() - 1] != '/') {
      filepath.push_back('/');
    }
  }

  std::string filename = std::to_string(k1) + "x" + 
                       std::to_string(k2) + "_AL" + 
                       std::to_string(a.length()) + "_t-seq.txt";
  std::string fileLocation = filepath + filename + ".txt";

  if (!std::filesystem::exists(filepath)) {
    std::filesystem::create_directory(filepath);
  }

  // std::string utf_8_to_euckr_fileLocation = utf8_to_euckr(fileLocation);
  std::ofstream writeFile(std::filesystem::u8path((const char *)fileLocation.c_str()));

  if (!writeFile.is_open()) {
    std::cerr << "File open error.\n";

    std::exit(1);
  }

  writeFile << tSeq << "\n";

  writeFile.close();
}

auto BGArray::printSSeq(std::string filepath) const -> void {
  if (filepath.empty()) {
    return;
  }
  if (filepath[filepath.size() - 1] != '/' && filepath[filepath.size() - 1] != '\\') {
    if (filepath[0] == 'C' || filepath[0] == 'c') {
      filepath.push_back('\\');
    } else if (filepath[filepath.size() - 1] != '/') {
      filepath.push_back('/');
    }
  }

  std::string filename = std::to_string(k1) + "x" + 
                       std::to_string(k2) + "_AL" + 
                       std::to_string(a.length()) + "_s-seq.txt";
  std::string fileLocation = filepath + filename;

  if (!std::filesystem::exists(filepath)) {
    std::filesystem::create_directory(filepath);
  }

  // std::string utf_8_to_euckr_fileLocation = utf8_to_euckr(fileLocation);
  std::ofstream writeFile(std::filesystem::u8path((const char *)fileLocation.c_str()));
  if (!writeFile.is_open()) {
    std::cerr << "File open error.\n";
    std::exit(1);
  }

  writeFile << sSeq << "\n";

  writeFile.close();
}
} // namespace LCODE
