#pragma once
#include <utility>

// map에서 key 를 pair로 사용하기 위해서 만들어진 구조체

// A hash function used to hash a pair of any kind
struct pair_hash {
  template <class T1, class T2>
  std::size_t operator()(const std::pair<T1, T2> &p) const {
    auto h1 = std::hash<T1>{}(p.first);
    auto h2 = std::hash<T2>{}(p.second);

    // Mainly for demonstration purposes, i.e. works but is overly simple
    // In the real world, use sth. like boost.hash_combine
    if (h1 != h2) {
      return h1 ^ h2;
    }

    return h1;
  }
};