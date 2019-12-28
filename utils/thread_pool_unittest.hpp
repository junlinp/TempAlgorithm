//
// Created by junlinp on 2019-12-10.
//

#ifndef ALGORITHM_UTILS_THREAD_POOL_UNITTEST_HPP_
#define ALGORITHM_UTILS_THREAD_POOL_UNITTEST_HPP_

#include <gtest/gtest.h>
#include "ThreadPool.hpp"
void Functor(int* arr, int N, int idx, ThreadPool &pool) {
  if (idx < N) {
    arr[idx] = idx;
    pool.AddTask(std::bind(Functor, arr, N, idx + 1, std::ref(pool)));
  }
}
TEST(ThreadPool, Simple) {
  const int N = 1024 * 1024;
  int arr[N] = {0};

  {

    ThreadPool thread_pool;
    thread_pool.AddTask(
        std::bind(Functor, arr, N, 0, std::ref(thread_pool))
      );
  }
  for(int i = 0; i < N; i++) {
    EXPECT_EQ(arr[i], i);
  }
}
#endif //ALGORITHM_UTILS_THREAD_POOL_UNITTEST_HPP_
