#pragma once

// Standard libraries
#include <cmath>
#include <cstdint>
#include <deque>
#include <stack>
#include <string>
#include <unordered_set>
#include <vector>
namespace LCODE {

class DBSeq {
private:
  std::string A;      // 고유한 문자열
  std::size_t k, n;   // 문자열 개수, 시퀀스 길이
  bool        cyclic; // 시퀀스 순환 여부
  std::string dBSeq;  // 시퀀스 저장

  std::unordered_set<std::string> seen;       // 깊이 우선 탐색에서 방문한 노드 저장
  std::vector<bool>               seen_edges; // 간선(edge) 방문 여부 체크 (k^n 크기)

  // std::vector<uint8_t>            edges;             // 깊이 우선 탬색 과정에서 탐색된 간선들의 정보 저장 , for OLD
  // /**
  //  * @brief 깊이 우선 탐색 알고리즘(재귀) , OLD version
  //  *
  //  * @param startingNode 시작 노드
  //  */
  // void dfs(std::string startingNode);

  /**
   * @brief FKM(Fredricksen-Maiorana-Kessler) 알고리즘을 이용한 드 브루인 시퀀스 생성
   * * @details
   * 기존의 재귀 DFS 방식 대신 Lyndon Word 생성 원리를 이용한 반복문 기반 알고리즘을 사용합니다.
   * 1GB 이상의 대용량 시퀀스 생성을 위해 다음과 같은 최적화가 적용되었습니다:
   * - **메모리 직기록**: 중간 매개체(edges) 없이 dBSeq 문자열 포인터에 직접 알파벳을 씁니다.
   * - **시간 복잡도**: Amortized O(1) per character로 이론적 최적 속도를 보장합니다.
   * - **공간 최적화**: unordered_set 등의 무거운 자료구조를 배제하여 캐시 효율을 극대화했습니다.
   * * @param startingNode FKM 알고리즘 특성상 내부에서 초기화되므로 하위 호환성을 위해 유지됩니다.
   */
  void dfsIterative();

  /**
   * @brief 드 브루인 시퀀스 생성 프로세스 통합 실행
   * * @details
   * 시퀀스 생성을 위한 메모리를 사전 할당하고, dfsIterative를 호출하여 결과를 dBSeq에 저장합니다.
   * - **메모리 관리**: k^n 크기의 대규모 메모리(n=10, k=8 기준 약 1GB)를 assign()으로 선점하여
   * 실행 중 발생할 수 있는 동적 재할당(Reallocation) 오버헤드와 Assertion 에러를 방지합니다.
   * - **순환/비순환 처리**: cyclic 플래그에 따라 시퀀스 끝에 n-1개의 접두사를 추가합니다.
   * * @note
   * 호출 전 k와 n 값이 유효한지 확인해야 하며, 시스템 RAM 용량이 k^n 바이트 이상인지 고려해야 합니다.
   */
  void genDeBruijnSeq();

public:
  /**
   * @brief DBSeq 생성자
   *
   * @param _A 고유 알파벳 문자열
   * @param _k 고유 알파벳 문자열의 길이
   * @param _n 시퀀스 길이
   * @param _cyclic 시퀀스 순환 여부
   */
  DBSeq(std::string _A, const std::size_t &_k, const std::size_t &_n, const bool &_cyclic = true);

  /**
   * @brief DBSeq 멤버 변수 설정
   *
   * @param _A 고유 알파벳 문자열
   * @param _k 고유 알파벳 문자열의 길이
   * @param _n 시퀀스 길이
   * @param _cyclic 시퀀스 순환 여부
   */
  void setDBSeq(const std::string &_A, const std::size_t &_k, const std::size_t &_n, const bool &_cyclic = true);

  /**
   * @brief 드 브루인 시퀀스 반환
   *
   * @return std::string dBSeq
   */
  [[nodiscard]] auto getDBSeq() const -> std::string;
};
} // namespace LCODE
