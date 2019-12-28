//
// Created by junlinp on 2019-12-10.
//

#ifndef ALGORITHM_UTILS_THREADPOOL_HPP_
#define ALGORITHM_UTILS_THREADPOOL_HPP_
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
class ThreadPool {

 private:
  bool b_stop = false;
  std::mutex _mutex_queue;
  std::condition_variable _cv;
  std::queue<std::function<void(void)>> _queue;
  std::vector<std::thread> _threads;

  void ThreadProcess();
 public:
  ThreadPool(size_t num_thread = std::thread::hardware_concurrency());
  ~ThreadPool();

  void AddTask(const std::function<void(void)>& callable);
};

#endif //ALGORITHM_UTILS_THREADPOOL_HPP_
