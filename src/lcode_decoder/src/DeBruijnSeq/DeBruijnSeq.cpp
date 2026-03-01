#include "DeBruijnSeq/DeBruijnSeq.hpp"

// Standard libraries
#include <utility>

namespace LCODE {

// OLD
// void DBSeq::dfs(std::string startingNode) {
//   for (std::size_t i = 0; i < k; ++i) {
//     std::string str = startingNode + A[i];
//     if (seen.find(str) == seen.end()) {
//       seen.insert(str);
//       dfs(str.substr(1));
//       edges.push_back(static_cast<int>(i));
//     }
//   }
// }

// uint_8 버전
void DBSeq::dfsIterative() {
  size_t total_elements = 1;
  for (size_t i = 0; i < n; ++i)
    total_elements *= k;

  // dBSeq에 즉시 공간 할당 (1GB)
  dBSeq.assign(total_elements, 0);
  char       *out_ptr  = &dBSeq[0];
  const char *alphabet = A.c_str();

  std::vector<uint8_t> a(n + 1, 0);
  int                  N       = static_cast<int>(n);
  int                  K_LIMIT = static_cast<int>(k) - 1;
  int                  t = 1, p = 1;

  while (true) {
    if (t <= N) {
      a[t] = a[t - p];
      t++;
    } else {
      if (N % p == 0) {
        for (int i = 1; i <= p; ++i) {
          // 여기서 alphabet 매핑을 직접 수행하여 dBSeq에 기록
          *out_ptr++ = alphabet[a[i]];
        }
      }
      t = N;
      while (t > 0 && a[t] == K_LIMIT)
        t--;
      if (t == 0)
        break;
      a[t]++;
      p = t;
      t++;
    }
  }

  // 비순환(Non-cyclic) 처리: n-1개를 뒤에 붙임
  if (!cyclic) {
    for (size_t i = 0; i < n - 1; ++i) {
      dBSeq.push_back(dBSeq[i]);
    }
  }
}

void DBSeq::genDeBruijnSeq() {
  // 1. 기존의 무거운 컨테이너들 정리
  seen.clear();

  // 2. 최적화된 생성 함수 호출
  dfsIterative();
}

DBSeq::DBSeq(std::string _A, const std::size_t &_k, const std::size_t &_n, const bool &_cyclic) :
    A(std::move(_A)), k(_k), n(_n), cyclic(_cyclic) {
  genDeBruijnSeq();
}

void DBSeq::setDBSeq(const std::string &_A, const std::size_t &_k, const std::size_t &_n, const bool &_cyclic) {
  A      = _A;
  k      = _k;
  n      = _n;
  cyclic = _cyclic;

  genDeBruijnSeq();
}

auto DBSeq::getDBSeq() const -> std::string {
  return dBSeq;
}

} // namespace LCODE
