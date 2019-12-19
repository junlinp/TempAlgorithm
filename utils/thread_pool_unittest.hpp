//
// Created by junlinp on 2019-12-10.
//

#ifndef ALGORITHM_UTILS_THREAD_POOL_UNITTEST_HPP_
#define ALGORITHM_UTILS_THREAD_POOL_UNITTEST_HPP_

#include <gtest/gtest.h>
#include "ThreadPool.hpp"
TEST(ThreadPool, Simple) {
  auto functor = [](int id) {
    std::cout << "task " << id << " Run" << std::endl;
  };
  ThreadPool thread_pool(2);

  for(int i = 0; i < 1024; i++) {
    thread_pool.AddTask(
        std::bind(functor, i)
    );
  }
}
#endif //ALGORITHM_UTILS_THREAD_POOL_UNITTEST_HPP_
