#pragma once

// Standard libraries
#include <cmath>
#include <string>
#include <vector>

#include "ConfigSettings/ConfigSettings.hpp"

namespace LCODE {
/**
 * @brief 드 브루인 시퀀스 생성 함수를 이용하여 `s-seq`와 `t-seq`를 생성하고
 *        `s-seq`와 `t-seq`를 이용해서 2차원 배경 배열을 생성한다.
 *        실제로 2차원 배경 배열을 메모리에 저장하는 것은 아니며 모든 연산은
 *        `s-seq`와 `t-seq`를 사용한다. 사용자는 자신이 찾고싶은
 *        2차원 배경 배열의 일부를 `getPos()`함수에 입력하면 해당 위치가 반환된다.
 *
 */
class BGArray {
private:
  std::string                   a;                          // 고유한 알파벳 문자열
  std::size_t                   aLen{};                     // 알파벳 문자열의 길이
  std::string                   deSeqK1, deSeqK2;           // 서브셋 행 기반 시퀀스, 서브셋 열 기반 시퀀스
  std::string                   sSeq, tSeq;                 // s-seq, t-seq
  std::vector<std::vector<int>> BGArr;                      // 2차원 배경 배열
  std::size_t                   k1, k2;                     // 서브셋 행, 열의 크기
  std::size_t                   deSeqK1Len{}, deSeqK2Len{}; // 배경 배열의 s-seq, t-seq 길이
  std::size_t                   BGCols{}, BGRows{};         // 2차원 배경 배열 행, 열의 크기
  std::vector<std::size_t>      countShift;                 // 밀린 횟수
  std::size_t                   lineInterval;               // 라인 간격

  /**
   * @brief 멤버 변수 초기화
   *
   */
  void initMemberVars();

public:
  /**
   * @brief `BGArray` 생성자
   *
   * @param ConfigSettings
   * @param _lineInterval 기준선 간격
   */
  BGArray(std::string _a, const std::size_t &_k1, const std::size_t &_k2, const std::size_t &_lineInterval = 5);

  BGArray(ConfigSettings &config, const std::size_t &_lineInterval = 5);

  /**
   * @brief 2차원 배경 배열 생성
   *        시퀀스의 길이가 길어질 경우 사용 금지(메모리 부족)
   *        `printBGArr()` 함수를 사용해 파일로 출력하는 것을 권장한다.
   *
   */
  void setBGArr();

  /**
   * @brief 2차원 배경 배열을 파일로 출력
   *
   * @param filepath 저장할 파일 경로
   */
  void printBGArr(std::string filepath = "./", std::string fileName = "print_bg_array");

  /**
   * @brief 2개의 행렬값을 받아서 그 범위 내에 있는 배경 배열만 파일로 출력
   *
   * @param row1 첫 번째 행
   * @param col1 첫 번째 열
   * @param row2 두 번째 행
   * @param col2 두 번째 열
   * @param filepath 저장할 파일 경로
   */
  void printSpecificRange(const std::size_t &row1, const std::size_t &col1, const std::size_t &row2,
                          const std::size_t &col2, std::string filepath = "./");

  /**
   * @brief 생성자를 이용해 멤버변수 초기화를 하지 않은 경우 멤버변수 설정
   *
   * @param _a 고유한 문자 조합
   * @param _k1 서브셋 행 크기
   * @param _k2 서브셋 열 크기
   */
  void setElemt(const std::string &_a, const std::size_t &_k1, const std::size_t &_k2);

  /**
   * @brief 특정 범위의 서브셋 가져오기
   *
   * @param x x좌표
   * @param y y좌표
   * @param col_size 가져올 열의 크기
   * @param row_size 가져올 행의 크기
   * @return std::vector<std::vector<int>> 서브셋
   */
  auto getSubset(const std::size_t &x, const std::size_t &y, const std::size_t &col_size, const std::size_t &row_size)
      -> std::vector<std::vector<int>>;

  /**
   * @brief 2차원 배경 배열의 행 길이 반환
   *
   * @return std::size_t 행 길이
   */
  auto bgRowsSize() -> std::size_t;

  /**
   * @brief 2차원 배경 배열의 열 길이 반환
   *
   * @return std::size_t 열 길이
   */
  auto bgColsSize() -> std::size_t;

  /**
   * @brief 찾고 싶은 행열 셋과 해당 셋의 진짜 위치를 입력받고
   *        전체 2차원 배경 배열에서 해당 셋의 위치를 찾아서 반환한다.
   *
   * @param subset 찾고싶은 셋
   * @param center 셋의 정확한 위치
   * @return std::pair<int, int> 찾은 위치
   */
  [[nodiscard]] auto getPos(const std::vector<std::vector<int>> &subset, const std::pair<int, int> &center)
      -> std::pair<int, int>;

  /**
   * @brief 배경 배열 반환
   *
   * @return std::vector<std::vector<int>> 2차원 배경 배열
   */
  [[nodiscard]] auto getBGArr() const -> std::vector<std::vector<int>>;

  /**
   * @brief 서브셋의 열 크기 반환 반환
   *
   * @return std::size_t 열 크기
   */
  [[nodiscard]] auto getSubRow() const -> std::size_t;

  /**
   * @brief 서브셋의 행 크기 반환
   *
   * @return std::size_t 행 크기
   */
  [[nodiscard]] auto getSubCol() const -> std::size_t;

  /**
   * @brief 알파벳 길이 반환
   *
   * @return std::size_t 알파벳 길이
   */
  [[nodiscard]] auto getAlphaLenght() const -> std::size_t;

  /**
   * @brief t-seq 전체 출력
   *
   */
  auto printTSeq(std::string filepath) const -> void;

  /**
   * @brief s-seq 전체 출력
   *
   */
  auto printSSeq(std::string filepath) const -> void;
};
} // namespace LCODE
