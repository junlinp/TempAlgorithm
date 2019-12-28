//
// Created by junlinp on 2019-12-10.
//

#include "ThreadPool.hpp"
#include "iostream"

ThreadPool::ThreadPool(size_t num_thread) :
      _mutex_queue(),
      _cv(),
      _queue(),
      _threads()
{
  for(size_t i = 0; i < num_thread; i++) {
    _threads.push_back(
        std::thread(&ThreadPool::ThreadProcess, this));
  }
}
ThreadPool::~ThreadPool() {
  {
    std::lock_guard<std::mutex> lg(_mutex_queue);
    b_stop = true;
  }
  _cv.notify_all();
  for(auto& t : _threads) {
    t.join();
  }
}
void ThreadPool::ThreadProcess() {
  while(true) {
    std::function<void(void)> task;
    {
      std::unique_lock<std::mutex> ul(_mutex_queue);
      _cv.wait(ul, [this]{return this->b_stop ||!_queue.empty();});
      if(this->b_stop && _queue.empty()) return;
      task = _queue.front();
      _queue.pop();
    }

    task();
  }

}
void ThreadPool::AddTask(const std::function<void(void)> &callable) {
  std::unique_lock<std::mutex> lg(_mutex_queue);
  _queue.push(callable);
}



